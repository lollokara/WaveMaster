"""PlatformIO extra script for the ESPFleet agent.

Add to platformio.ini:

    extra_scripts = pre:espfleet_build.py
    custom_espfleet_hub = http://192.168.3.87:8080

It does two things:

1. Injects the current git identity into the build, so a running device can
   tell the hub exactly which commit it is executing:
       ESPFLEET_VERSION, ESPFLEET_GIT_COMMIT, ESPFLEET_GIT_BRANCH,
       ESPFLEET_GIT_DIRTY, ESPFLEET_BUILD_TIME, ESPFLEET_PROJECT
   ESPFLEET_PROJECT is derived from the Gogs `origin` remote, so no manual
   configuration is needed.

2. Adds a `publish` target that uploads the built .bin to the hub:
       pio run -e <env> -t publish
"""

import datetime
import json
import mimetypes
import os
import re
import subprocess
import urllib.error
import urllib.request
import uuid

Import("env")  # noqa: F821  (injected by SCons)


def _git(*args, cwd=None):
    try:
        out = subprocess.run(
            ["git", *args],
            cwd=cwd or env.subst("$PROJECT_DIR"),
            capture_output=True,
            text=True,
            timeout=10,
        )
        return out.stdout.strip() if out.returncode == 0 else ""
    except (OSError, subprocess.SubprocessError):
        return ""


def _slug_from_remote(url: str) -> str:
    """http://host:3000/owner/repo.git or ssh://git@host/owner/repo.git -> owner/repo"""
    if not url:
        return ""
    url = url.strip().removesuffix(".git")
    match = re.search(r"[:/]([^/:]+)/([^/]+)$", url)
    # Gogs stores repositories under lowercased directory names.
    return f"{match.group(1).lower()}/{match.group(2).lower()}" if match else ""


def _remote_url() -> str:
    """Prefer origin, then a Gogs-looking remote, then whatever exists."""
    remotes = [line.strip() for line in _git("remote").splitlines() if line.strip()]
    for name in ("origin", "gogs", *remotes):
        if name in remotes:
            url = _git("remote", "get-url", name)
            if url:
                return url
    return ""


def _project_option(name: str, default: str = "") -> str:
    try:
        value = env.GetProjectOption(name)
    except Exception:
        value = None
    return (value or os.environ.get(name.upper(), "") or default).strip()


commit = _git("rev-parse", "HEAD")
branch = _git("rev-parse", "--abbrev-ref", "HEAD")
describe = _git("describe", "--tags", "--always", "--dirty")
dirty = 1 if _git("status", "--porcelain") else 0
slug = _project_option("custom_espfleet_project") or _slug_from_remote(_remote_url())
hub = _project_option("custom_espfleet_hub")
build_time = datetime.datetime.now().astimezone().replace(microsecond=0).isoformat()

defines = [
    ("ESPFLEET_VERSION", env.StringifyMacro(describe or "0.0.0")),
    ("ESPFLEET_GIT_COMMIT", env.StringifyMacro(commit)),
    ("ESPFLEET_GIT_BRANCH", env.StringifyMacro(branch)),
    ("ESPFLEET_GIT_DIRTY", dirty),
    ("ESPFLEET_BUILD_TIME", env.StringifyMacro(build_time)),
]
if slug:
    defines.append(("ESPFLEET_PROJECT", env.StringifyMacro(slug)))
if hub:
    defines.append(("ESPFLEET_HUB", env.StringifyMacro(hub)))

env.Append(CPPDEFINES=defines)

print(
    f"espfleet: version={describe or '?'} commit={commit[:8] or '?'} branch={branch or '?'} "
    f"dirty={dirty} project={slug or '(unset)'} hub={hub or '(unset)'}"
)


# ------------------------------------------------------------------ publish

def _multipart(fields: dict, filename: str, payload: bytes):
    boundary = uuid.uuid4().hex
    parts = []
    for key, value in fields.items():
        parts.append(
            f"--{boundary}\r\nContent-Disposition: form-data; name=\"{key}\"\r\n\r\n{value}\r\n".encode()
        )
    content_type = mimetypes.guess_type(filename)[0] or "application/octet-stream"
    parts.append(
        f"--{boundary}\r\nContent-Disposition: form-data; name=\"file\"; filename=\"{filename}\"\r\n"
        f"Content-Type: {content_type}\r\n\r\n".encode()
    )
    parts.append(payload)
    parts.append(f"\r\n--{boundary}--\r\n".encode())
    return b"".join(parts), f"multipart/form-data; boundary={boundary}"


def publish_firmware(target=None, source=None, env=None):  # noqa: ARG001 - SCons signature
    hub_url = _project_option("custom_espfleet_hub")
    if not hub_url:
        raise SystemExit("espfleet: set custom_espfleet_hub in platformio.ini (or the ESPFLEET_HUB env var)")
    if not slug:
        raise SystemExit("espfleet: no Gogs project. Add a git remote, or set custom_espfleet_project")

    binary = os.path.join(env.subst("$BUILD_DIR"), env.subst("$PROGNAME") + ".bin")
    if not os.path.isfile(binary):
        raise SystemExit(
            f"espfleet: {binary} not found.\n"
            f"          Build first:  pio run -e {env['PIOENV']}"
        )

    elf = os.path.splitext(binary)[0] + ".elf"
    if os.path.isfile(elf) and os.path.getmtime(elf) > os.path.getmtime(binary) + 1:
        print("espfleet: WARNING the .elf is newer than the .bin, you may be publishing a stale image")

    with open(binary, "rb") as handle:
        payload = handle.read()

    body, content_type = _multipart(
        {
            "project": slug,
            "version": describe or "0.0.0",
            "chip": env.BoardConfig().get("build.mcu", ""),
            "git_commit": commit,
            "git_branch": branch,
            "git_dirty": str(dirty),
            "notes": os.environ.get("ESPFLEET_NOTES", ""),
            "uploaded_by": os.environ.get("USERNAME") or os.environ.get("USER") or "pio",
        },
        os.path.basename(binary),
        payload,
    )

    request = urllib.request.Request(
        hub_url.rstrip("/") + "/api/v1/firmware",
        data=body,
        headers={"Content-Type": content_type},
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=60) as response:
            result = json.loads(response.read().decode())
    except urllib.error.HTTPError as exc:
        raise SystemExit(f"espfleet: hub rejected the upload ({exc.code}): {exc.read().decode()[:300]}") from exc
    except urllib.error.URLError as exc:
        raise SystemExit(f"espfleet: cannot reach {hub_url}: {exc.reason}") from exc

    firmware = result.get("firmware", {})
    state = "already published" if result.get("duplicate") else "published"
    print(
        f"espfleet: {state} #{firmware.get('id')} {firmware.get('version')} "
        f"({len(payload)} bytes) -> {hub_url}/#/project/{slug}"
    )


# NOTE: do NOT give this target a dependency on "$BUILD_DIR/${PROGNAME}.bin".
# Aliasing that node breaks the platform's ELF-to-BIN builder
# ("TypeError: unsupported operand type(s) for +: '_Null' and 'str'").
# Publishing therefore acts on the artifact left by the previous build.
env.AddCustomTarget(
    name="publish",
    dependencies=None,
    actions=[publish_firmware],
    title="Publish to ESPFleet",
    description="Upload the built firmware to the ESPFleet hub",
)
