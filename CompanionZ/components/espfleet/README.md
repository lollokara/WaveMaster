# espfleet (device library)

Fleet agent for ESP32 boards. Reports what it is and what commit it is running to
an ESPFleet hub, and accepts OTA updates from it.

Targets **esp32, esp32s2, esp32s3, esp32c3, esp32c5, esp32c6** under both
`framework = arduino` and `framework = espidf`. It only calls ESP-IDF APIs, so
there is no Arduino dependency and nothing chip-specific in the source.

## Integration

```ini
lib_deps      = symlink://../espfleet
extra_scripts = pre:../espfleet/scripts/espfleet_build.py
custom_espfleet_hub = http://192.168.3.87:8080
```

```cpp
#include <EspFleet.h>

void setup() {
  WiFi.begin(ssid, pass);       // or esp_wifi_* / Ethernet, the agent waits for an IP
  EspFleet.begin();
}
```

That is the whole integration. `begin()` spawns the agent task, so `loop()` needs
nothing.

## What it reports

`id` (MAC), name, chip + revision, IP, MAC, flash size, heap total/free/minimum,
PSRAM free, uptime, boot count, reset reason, RSSI, SSID, IDF version, agent
version, firmware name (from the app descriptor), version, git commit, branch,
dirty flag, build time, Gogs project slug, web UI port/path, OTA support and the
result of the last update.

## Build flags

| Flag | Default | Meaning |
|---|---|---|
| `ESPFLEET_HUB` | from `custom_espfleet_hub` | hub base URL |
| `ESPFLEET_PROJECT` | from the git `origin` remote | Gogs slug `owner/repo` |
| `ESPFLEET_VERSION` | `git describe --tags --always --dirty` | version string |
| `ESPFLEET_INTERVAL_MS` | `30000` | heartbeat period, the hub can override it |
| `ESPFLEET_WEB_PORT` | `0` | port of the web UI your project already serves |
| `ESPFLEET_WEB_PATH` | `/` | path of that UI |
| `ESPFLEET_INFO_PORT` | `0` | serve `/fleet/info` on this port, `0` disables it |
| `ESPFLEET_ENABLE_OTA` | `1` | accept updates |
| `ESPFLEET_ENABLE_MDNS` | `1` | advertise `_espfleet._tcp` |
| `ESPFLEET_STACK_SIZE` | `6144` | agent task stack, in bytes |
| `ESPFLEET_TASK_PRIORITY` | `1` | agent task priority |
| `ESPFLEET_MAX_EXTRA` | `4` | key/value slots for `setExtra()` |

`ESPFLEET_VERSION`, `ESPFLEET_GIT_COMMIT`, `ESPFLEET_GIT_BRANCH`,
`ESPFLEET_GIT_DIRTY`, `ESPFLEET_BUILD_TIME` and `ESPFLEET_PROJECT` are injected by
`scripts/espfleet_build.py`. Set them by hand only if you build without it.

## API

```cpp
EspFleet.begin();                              // build-flag configuration
EspFleet.begin("http://192.168.3.87:8080");    // or override the hub
EspFleet.begin(config);                        // or a full EspFleetConfig

EspFleet.setName("kitchen-sensor");
EspFleet.setProject("lorenzo/sensor-node");
EspFleet.setWebUi(80, "/");
EspFleet.setInterval(60000);
EspFleet.setExtra("temperature", "21.4");      // shows on the device page
EspFleet.onCommand(handler);                   // your own commands
EspFleet.infoJson(buf, sizeof(buf));           // the status document
EspFleet.poke();                               // heartbeat now
```

The command handler returns `true` if it handled the command:

```cpp
bool handler(const char* cmd, const char* arg) {
  if (!strcmp(cmd, "identify")) { blink(); return true; }
  return false;
}
```

`reboot`, `ota` and `ping` are handled internally, so the handler only sees them
if you want to react first.

## OTA

Requires a partition table with two app slots. `board_build.partitions = default.csv`
(the Arduino default for 4 MB boards) already has them.

The hub sends a URL and a SHA-256. The agent streams the image in 1460-byte chunks
straight into the inactive slot, verifies the digest before switching the boot
partition, and reboots. After the new image gets its first successful heartbeat it
calls `esp_ota_mark_app_valid_cancel_rollback()`, so an image that cannot reach the
hub rolls back on its own.

## Cost

- one task, 6 KB stack, priority 1, wakes every 200 ms to check a timer
- ~1.2 KB of static state, plus 1460 bytes transiently during an update
- one ~550 byte HTTP POST per interval
- no JSON library, no HTTPClient, no allocation in the steady state
