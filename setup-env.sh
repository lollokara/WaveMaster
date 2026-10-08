#!/bin/bash
set -e
export DEBIAN_FRONTEND=noninteractive
export PATH="$HOME/.local/bin:$PATH"

# ==============================================================================
# WaveMaster Development Environment Setup
# Targets: ESP32-S3 (ESP-IDF via PlatformIO) + ATmega328P (Arduino AVR) + Tools
# ==============================================================================

# --- System Dependencies ---
apt-get update -qq
apt-get install -y -qq --no-install-recommends \
  build-essential \
  cmake \
  ninja-build \
  pkg-config \
  git \
  curl \
  wget \
  file \
  zip \
  unzip \
  python3-venv \
  python3-pip \
  python3-dev \
  libusb-1.0-0-dev \
  libudev-dev \
  udev

# --- Python Environment & PlatformIO ---
# Install PlatformIO and python utilities used by test/diagnostic scripts
pip install -q --break-system-packages \
  "platformio==6.1.16" \
  pyserial \
  rich \
  pytest

# Configure udev rules for serial adapters and ESP32 USB-JTAG
if [ -d /etc/udev/rules.d ] && [ ! -f /etc/udev/rules.d/99-platformio-udev.rules ]; then
  curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules \
    -o /etc/udev/rules.d/99-platformio-udev.rules 2>/dev/null || true
fi

# --- Pre-cache Toolchains & Platforms ---
# Automatically find local repo directory, or clone temporarily to pre-cache packages
REPO="${WAVEMASTER_REPO:-/home/user/WaveMaster}"
if [ ! -d "$REPO/.git" ]; then
  REPO=$(mktemp -d)
  git clone --depth 1 https://github.com/lollokara/WaveMaster.git "$REPO" || REPO=""
fi

if [ -n "$REPO" ]; then
  cd "$REPO"

  # 1. Cache ESP32-S3 + ESP-IDF toolchain
  if [ -f "platformio.ini" ]; then
    echo "Installing and caching ESP32-S3 / ESP-IDF toolchain..."
    pio pkg install -d . || true
  fi

  # 2. Cache ATmega328P / Atmel AVR toolchain for ArduinoCompanion
  if [ -f "ArduinoCompanion/platformio.ini" ]; then
    echo "Installing and caching ATmega328P Arduino toolchain..."
    pio pkg install -d ArduinoCompanion || true
  fi

  # Clean up temp clone if used
  if [ "$REPO" != "${WAVEMASTER_REPO:-/home/user/WaveMaster}" ]; then
    rm -rf "$REPO"
  fi
fi

echo "=========================================================="
echo " WaveMaster environment setup complete!"
echo " ESP32-S3 (ESP-IDF) and ATmega328P toolchains installed."
echo "=========================================================="
