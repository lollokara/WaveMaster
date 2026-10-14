# Vendored: ESPFleet device agent

- Source: `git@192.168.3.87:lorenzo/ESPFleet.git`, directory `firmware/espfleet`
- Commit: `d734ea1c5b4be22eb3c0b973ec2c35e87f7b7665`
- Agent version string in the source: 1.1.0

Not copied: `library.json` and `scripts/espfleet_build.py` (PlatformIO `lib_deps` path only). Z-Bro builds
with `framework = espidf`, so the component's own `cmake/espfleet_git.cmake` injects the git identity
(`ESPFLEET_VERSION`, commit, branch, dirty, build time, project slug).

## Local patches (search for `VENDORED PATCH` in the sources)

1. `EspFleet.cpp` log hook: from an ISR or before the scheduler runs it only chains to the previous handler;
   per-task recursion guard; the line is parsed in place (one 384 byte stack buffer instead of 512 + 400).
2. Heartbeat JSON is built in a 1792 byte heap buffer (was 896 on the stack, too small for ~20 metrics).
   `ESPFLEET_MAX_METRICS` 16 -> 28, `ESPFLEET_MAX_EXTRA` 4 -> 8, HEALTH `custom` buffer 256 -> 800 bytes.
3. New `espfleet_set_gate()` callback: asked before flash writes of an OTA (`"ota"`, may refuse), before the
   hub `reboot` command (`"reboot"`, may refuse) and before the restart after a good OTA (`"restart"`).
4. New `espfleet_set_health_cb()`: lets the application add per-task stack headroom and counters
   (`wifi_reconn`, ...) to the HEALTH record (upstream hard-codes `wifi_reconn: 0`).
5. New `espfleet_hub_url()` getter (resolved hub, configured or found by mDNS).
6. mDNS hub discovery is rate-limited to once per 30 s (upstream: a 2 s blocking query every loop).
7. `cmake/espfleet_git.cmake`: a `github.com` remote no longer produces a project slug (the hub only knows
   Gogs projects); the project stays auto.

To update: re-copy upstream, then re-apply the patches above.
