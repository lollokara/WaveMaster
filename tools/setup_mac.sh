#!/usr/bin/env bash
# WaveMaster macOS setup: Homebrew, Python >= 3.10, ./.venv with PlatformIO + pyserial,
# pre-downloads both toolchains. Idempotent: safe to re-run.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV="$REPO/.venv"

say()  { printf '\033[36m[..]\033[0m %s\n' "$*"; }
good() { printf '\033[32m[OK]\033[0m %s\n' "$*"; }
warn() { printf '\033[33m[!!]\033[0m %s\n' "$*"; }
die()  { printf '\033[31m[XX]\033[0m %s\n' "$*" >&2; exit 1; }

confirm() { # confirm "question"  -> returns 0 on yes
    local ans
    read -r -p "$1 [y/N] " ans || return 1
    [[ "$ans" =~ ^[Yy] ]]
}

[[ "$(uname -s)" == "Darwin" ]] || die "This script is for macOS (found $(uname -s)). On Linux: python3 -m venv .venv && .venv/bin/pip install 'platformio>=6.1.16' pyserial"

# ---- Rosetta 2 (Apple Silicon) ------------------------------------------------
# Some PlatformIO packages only ship Intel (x86_64) binaries for macOS: the AVR
# toolchain (avr-gcc) used for the ATmega, and tool-ninja used by the ESP-IDF
# build. On Apple Silicon they fail with "Bad CPU type in executable" /
# "Unknown system error -86" unless Rosetta 2 is installed.
if [[ "$(uname -m)" == "arm64" ]]; then
    if /usr/bin/arch -x86_64 /usr/bin/true 2>/dev/null; then
        good "Rosetta 2 present (needed for Intel-only PlatformIO tools)"
    else
        warn "Apple Silicon without Rosetta 2: the AVR toolchain and ninja (Intel-only) cannot run."
        if confirm "Install Rosetta 2 now (softwareupdate --install-rosetta)?"; then
            softwareupdate --install-rosetta --agree-to-license \
                || die "Rosetta install failed; run: softwareupdate --install-rosetta --agree-to-license"
            /usr/bin/arch -x86_64 /usr/bin/true 2>/dev/null || die "Rosetta still not usable after install."
            good "Rosetta 2 installed"
        else
            die "Rosetta 2 is required. Run: softwareupdate --install-rosetta --agree-to-license"
        fi
    fi
fi

# ---- Homebrew ---------------------------------------------------------------
find_brew() {
    command -v brew 2>/dev/null || { [[ -x /opt/homebrew/bin/brew ]] && echo /opt/homebrew/bin/brew; } \
        || { [[ -x /usr/local/bin/brew ]] && echo /usr/local/bin/brew; } || true
}
BREW="$(find_brew)"
if [[ -z "$BREW" ]]; then
    warn "Homebrew not found."
    if confirm "Install Homebrew now (runs the official installer from brew.sh)?"; then
        /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
        BREW="$(find_brew)"
        [[ -n "$BREW" ]] || die "Homebrew install did not produce a brew binary."
    else
        warn "Skipping Homebrew; you must provide Python >= 3.10 yourself."
    fi
fi
if [[ -n "$BREW" ]]; then
    eval "$("$BREW" shellenv)"
    good "Homebrew: $BREW"
fi

# ---- Python >= 3.10 ---------------------------------------------------------
py_ok() { "$1" -c 'import sys; sys.exit(0 if sys.version_info >= (3, 10) else 1)' 2>/dev/null; }
PY=""
for cand in python3.12 python3.13 python3.11 python3.10 python3; do
    if command -v "$cand" >/dev/null 2>&1 && py_ok "$(command -v "$cand")"; then
        PY="$(command -v "$cand")"; break
    fi
done
if [[ -z "$PY" ]]; then
    [[ -n "$BREW" ]] || die "Python >= 3.10 not found and Homebrew unavailable."
    warn "Python 3.10+ not found."
    confirm "Install python@3.12 with Homebrew?" || die "Python >= 3.10 is required."
    "$BREW" install python@3.12
    PY="$("$BREW" --prefix python@3.12)/bin/python3.12"
    [[ -x "$PY" ]] || PY="$(command -v python3.12)"
fi
good "Python: $PY ($("$PY" --version 2>&1))"

# ---- venv + packages --------------------------------------------------------
if [[ -x "$VENV/bin/python" ]] && ! py_ok "$VENV/bin/python"; then
    warn "Existing .venv uses Python < 3.10; recreating."
    rm -rf "$VENV"
fi
if [[ ! -x "$VENV/bin/python" ]]; then
    say "Creating venv at $VENV"
    "$PY" -m venv "$VENV"
fi
say "Installing platformio and pyserial"
"$VENV/bin/python" -m pip install --quiet --upgrade pip
"$VENV/bin/python" -m pip install --quiet --upgrade 'platformio>=6.1.16' pyserial
# PlatformIO runs esptool (tool-esptoolpy) with this venv's Python, so its
# dependencies must live here too - otherwise the ESP32 build fails at the
# very end with "ModuleNotFoundError: No module named 'intelhex'".
say "Installing esptool dependencies into the venv"
"$VENV/bin/python" -m pip install --quiet --upgrade intelhex bitstring reedsolo ecdsa pyyaml cryptography
good "PlatformIO: $("$VENV/bin/pio" --version)"

# ---- .gitignore -------------------------------------------------------------
GI="$REPO/.gitignore"
touch "$GI"
for entry in ".venv/" "tools/backups/"; do
    grep -qxF "$entry" "$GI" || { printf '%s\n' "$entry" >> "$GI"; say "Added $entry to .gitignore"; }
done

# ---- toolchains -------------------------------------------------------------
say "Downloading ESP32-S3 platform/toolchain (first run is large, several minutes)"
"$VENV/bin/pio" pkg install -d "$REPO"
say "Downloading Atmel AVR platform/toolchain"
"$VENV/bin/pio" pkg install -d "$REPO/ArduinoCompanion"
good "Toolchains installed"

# ---- notes + ports ----------------------------------------------------------
cat <<'NOTE'

CH340 driver note:
  The ATmega328P Arduino Mini/Nano clone uses a CH340/CH341 USB-serial chip. macOS 10.15 (Catalina)
  and later, including Big Sur and newer, ship a built-in CH34x driver. If the board does not show
  up as /dev/cu.usbserial-* or /dev/cu.wchusbserial*, install WCH's driver from
  https://www.wch-ic.com/downloads/CH34XSER_MAC_ZIP.html and allow the system extension in
  System Settings > Privacy & Security, then replug the board. Use a data-capable USB cable.

NOTE
say "Detected serial ports:"
shopt -s nullglob
ports=(/dev/cu.usbmodem* /dev/cu.usbserial* /dev/cu.wchusbserial*)
if ((${#ports[@]})); then printf '    %s\n' "${ports[@]}"; else warn "none yet (plug the boards in)"; fi

cat <<NEXT

Setup complete. Next:
  1. Read $REPO/tools/README_FLASHING.md (laser OFF, DB25 unplugged while flashing).
  2. Run the wizard:   $VENV/bin/python $REPO/tools/flash_boards.py
     (or individually: detect | build | flash-atmega | flash-esp32 | verify)
NEXT
