# Z-Bro — wireless Z axis for WaveMaster

Z-Bro is an ESP32-C3 + A4988 stepper axis on the same WiFi as the WaveMaster S3. The S3 sends it
home / move / stop commands over TCP and gets position reports back. Firmware: `CompanionZ/`
(ESP-IDF 5.5, PlatformIO `espressif32@6.13.0`, board `esp32-c3-devkitm-1`). S3 side: `src/zbro_link.*`
plus hooks in `grbl.c` / `calib.*`. Shared wire-protocol code: `shared/zbro_proto.*` (pure C, host-tested).

## 1. IO review (ESP32-C3 ↔ A4988)

| Signal | GPIO | Verdict |
|---|---|---|
| ENABLE (active low) | 0 | OK, no strapping role. **Add 10 kΩ pull-up to 3.3 V**: the A4988 pulls ENABLE low internally, so the motor is energised from power-up until firmware drives the pin. |
| ENDSTOP | 5 | OK. RTC-capable, ADC2_CH0, JTAG MTDI only if eFuse selects pin-JTAG (default is USB-JTAG). Internal pull-up + switch to GND (NC wiring recommended) + 10 nF to GND. |
| MS1 / MS2 | 6 / 7 | OK. Free pins (SPI flash is 12–17). A4988 has 100 kΩ pull-downs on MSx. |
| MS3 | 8 | **Caution.** Strapping pin (GPIO2/8/9). Harmless in normal boot (only read when GPIO9 is low), but many C3 dev boards wire an RGB LED to GPIO8, so MS3 would flicker it / be loaded. Check the board. MS3 is required: 1/16 step is MS1=MS2=MS3=high. |
| RESET (active low) | 9 | **Caution.** Strapping pin = BOOT button. If the A4988 board or the button pulls it low at reset, the C3 enters ROM download mode. Make sure nothing holds it low at power-up. The firmware drives it high after boot. |
| SLEEP (active low) | 10 | OK. After wake-up wait ≥1 ms before STEP (charge pump). |
| DIR | 21 | **Caution.** U0TXD: the ROM boot log toggles it at start-up. Harmless while ENABLE is held high by the pull-up. Console must stay on USB-Serial-JTAG (GPIO18/19), UART0 console disabled. |
| STEP | 20 | **Caution.** U0RXD, input at boot. Same note. Fine as RMT output after boot. |

Nothing is unusable. Needed external parts: pull-up on ENABLE, check GPIO8/9 loading, 100 µF across VMOT.
A4988 limits: 1/16 maximum microstep, min STEP high/low 1 µs, motor current set by the Vref pot (no firmware control).
**Max range cannot be auto-measured** with one endstop and open-loop stepping (no stall feedback): range is set by the user (UI "set range from current position"). Calibration covers the rest.

## 2. Architecture

- C3: STEP generation by RMT TX (trapezoid with independent accel/decel, modelled on ESP-IDF
  `peripherals/rmt/stepper_motor`; moves are queued in chunks so position is known to one chunk during
  motion and exact at standstill; the C3 has no PCNT). Endstop on a GPIO ISR with debounce.
- Libraries: ESP-IDF `esp_http_server` (+WebSocket), `mdns`, `nvs_flash`, `cJSON`, `esp_wifi`, LittleFS
  (`joltwallet/littlefs` component) for the gzipped Vite build.
- Network: HTTP :80 (UI + REST + `/ws`), **TCP :7700 COM link**, **UDP :7701 beacon**
  (`ZBRO 1 <ip> 7700 <hostname>` broadcast every 2 s) so the S3 finds the C3 without config. mDNS `zbro.local`.
  Provisioning: no stored credentials → SoftAP `Z-Bro-XXXX` with the same web UI (WiFi page).
- S3: `zbro_link` task (core 1, prio 4) owns the socket; G-code Z moves are executed by the `grbl` task
  after `wait_idle()` (laser is gated off, XY pipeline empty), so the streaming path is never touched.
  Never call it from `dac_task`/`motion` tasks/ISRs.

## 3. COM protocol v1 (TCP :7700, ASCII, `\n`-terminated, one client; a new client replaces the old)

Requests (S3 → C3), replies are in order, one per request:

| Request | Reply |
|---|---|
| `HELLO` | `OK ZBRO 1 <fw>` |
| `PING` | `OK` |
| `POS` | `POS <mm> <state> <homed 0/1> <endstop 0/1>` |
| `HOME` | `OK` (accepted) … later `EVT HOMED <mm>` or `EVT ERR <code>` |
| `MOVE <mm> [<mm/s>]` (absolute) | `OK` … later `EVT DONE <mm>` or `EVT ERR <code>` |
| `JOG <dmm> [<mm/s>]` (relative) | as MOVE |
| `STOP` (decelerate with decel) / `ESTOP` (halt now, axis becomes unhomed) | `OK` then `EVT DONE <mm>` |
| `GET <key>` / `SET <key> <value>` / `SAVE` | `VAL <key> <value>` / `OK` / `OK` ; invalid → `ERR <code> <text>` |
| `CAL <start [tests…]\|abort\|status>` | `OK` / `CAL <json>` |

`<state>` ∈ `IDLE MOVING HOMING CAL FAULT`. Unsolicited lines: `POS …` (every `rpt_hz` while moving, once on stop),
`EVT HOMED|DONE|ERR|CAL …`. Error codes: 1 busy, 2 not homed, 3 out of range, 4 bad argument, 5 endstop hit
unexpectedly, 6 homing failed (no trigger within range), 7 fault, 8 unknown command. Position is in mm,
0 = home reference (see `home_to_max`). Moves are rejected (`ERR 2`) while not homed, except `HOME`/`ESTOP`/`JOG` with
`SET allow_unhomed 1`. Max line 128 bytes. The C3 never moves without a command.

## 4. Settings (C3 NVS namespace `zbro`, keys ≤ 15 chars; also exposed by web/`GET`/`SET`)

`steps_rev`(200) `lead_mm`(8) `microstep`(1/2/4/8/16, default 16) `dir_invert`(0) `es_invert`(0=NC wiring) `home_to_max`(1)
`max_range`(50 mm) `max_speed`(20 mm/s) `accel`(100 mm/s²) `decel`(100) `home_fast`(10) `home_slow`(1) `home_backoff`(2 mm)
`es_debounce_us`(2000) `pulse_us`(2) `idle_off_s`(5, 0 = never) `sleep_idle`(1) `rpt_hz`(10) `allow_unhomed`(0)
`hostname`. Calibration results: `lim_speed` `lim_accel` `lim_decel` `lim_home_fast` `es_repeat_um` `cal_time` — the
settings validator caps `max_speed`/`accel`/`decel`/`home_fast` at the measured limit once calibrated.

## 5. Calibration (runs on the C3, started from UI or `CAL START`)

Requires homed axis and free travel (UI asks for confirmation). Lost-step detection: after N out-and-back cycles at the
candidate parameter, slow-probe the endstop and compare its trigger position to the reference taken at the start; error
> 0.05 mm = fail. Each test sweeps geometrically upward until failure, bisects 3 times, stores the limit and sets the
working value to 80 %.
Tests: `endstop` (10 probes → repeatability in µm, picks `es_debounce_us`), `speed` (max_speed, step-rate bound ≤ ~100 kHz),
`accel`, `decel`, `homing` (home_fast: highest speed that still stops inside `home_backoff`). Progress/results via
`/api/cal`, `/ws`, `CAL status`. Abort at any time (STOP + unhomed).

## 6. REST / WebSocket (web UI contract)

`GET /api/status` → `{"pos":0,"state":"IDLE","homed":false,"endstop":false,"fw":"","rssi":-60,"s3":true,"uptime":0}`
`GET /api/settings` → `{"groups":[{"name":"Motion","items":[{"key","label","value","min","max","step","unit","type":"number|bool|enum","options":[]}]}]}`
`PUT /api/settings` `{key:value,…}` → saved (400 with `{"error","key"}` on invalid)
`POST /api/move {"z","f"}` `/api/jog {"dz","f"}` `/api/home` `/api/stop` `/api/estop`
`POST /api/cal/start {"tests":[…]}` `/api/cal/abort`, `GET /api/cal` → `{"running":bool,"current":"speed","tests":[{"name","status":"pending|running|ok|failed|skipped","progress":0-1,"result":null|{"limit","unit","working"},"trials":[{"value","ok"}]}]}`
`GET/PUT /api/wifi` (`{ssid,pass}` write-only pass), `POST /api/reboot`. `WS /ws` pushes `{"t":"status",…}` and `{"t":"cal",…}`.
UI: Vite + TypeScript, no framework (or tiny Preact), hand-designed instrument-panel look, light/dark; build output is
gzipped into `CompanionZ/data/` for LittleFS. No UI kit, no stock gradients/emoji icons.

## 7. S3 side

Settings: `$250` Z-Bro enable (0), `$251` Z move timeout s (60), `$252` default Z speed mm/s (0 = C3's max_speed).
G-code: `Z<n>` in G0/G1 (absolute/relative follows G90/G91; metres/inches follow G20/G21) → barrier, `MOVE`, wait `DONE`.
`$HZ` home, `$ZSTOP`, `$Z` status line, `$ZGET key` / `$ZSET key=value` / `$ZSAVE` / `$ZCAL[=tests|ABORT]` proxy to the C3.
Status report shows the real Z in `MPos:x,y,z`. Hold/Ctrl-X during a Z move sends `STOP`. Link lost mid-move → alarm-style
error for the job, never blocks `?`. Offline Z-Bro (not found) → Z words error out (`error:20`-class) only when `$250=1`;
with `$250=0` Z words are ignored as today.

## 8. Work packages (parallel; each agent commits only its own paths, no co-author trailer)

| Pkg | Owner | Paths |
|---|---|---|
| A core | executor/sonnet | `CompanionZ/main/zb_motion.c`, `zb_endstop.c`, `zb_driver.c` (A4988 pins/microstep), `zb_settings.c`, `zb_pins.h` |
| B net | executor/sonnet | `CompanionZ/main/main.c`, `zb_net.c` (WiFi/SoftAP/mDNS/TCP/beacon), `zb_web.c`, `CompanionZ/{platformio.ini,CMakeLists.txt,partitions.csv,sdkconfig.defaults}`, `shared/zbro_proto.*`, `test/host/zbro/` |
| C cal | executor/sonnet | `CompanionZ/main/zb_cal.c` |
| D ui | designer/sonnet | `CompanionZ/ui/` (+ build into `CompanionZ/data/`) |
| E S3 | executor/sonnet | `src/zbro_link.*`, `src/grbl.c`, `src/calib.*`, `src/CMakeLists.txt`, `src/main.c`, docs |

A/B/C code against the headers `CompanionZ/main/zb_motion.h` and `zb_settings.h` (committed with this plan).

## 9. Fleet (ESPFleet device agent)

The C3 reports to an ESPFleet hub (default `http://192.168.3.87:8080`): heartbeat, metrics, logs, HEALTH records
and OTA. Component: `CompanionZ/components/espfleet` (vendored, patches and source commit in its `VENDORED.md`),
glue: `CompanionZ/main/zb_fleet.{c,h}`.

- **Start**: `zb_fleet_start()` in `app_main` only creates a priority-1 `zb_fleet` task. That task starts the agent
  once the STA has an IP, so boot never waits for WiFi or the hub, and SoftAP-only provisioning never starts it.
  The agent task (priority 1, 8 KB stack) tolerates a missing network and hub. Name = the Z-Bro hostname; the project
  slug comes from a Gogs-style git origin at build time, else stays auto.
- **Hub setting**: NVS `zbro`/`fleet_hub`, set on the WiFi page (`PUT /api/wifi {"fleet_hub": ...}`, applied after
  a reboot). Unset = `http://192.168.3.87:8080`; empty = discover `_espfleet._tcp` over mDNS; `off` = agent disabled.
- **Metrics** (heartbeat, about every 30 s, updated every 1 s): `state pos_mm homed endstop driver_en speed_mm_s step_hz
  s3_link tcp_clients motion_err last_err estop moves dist_mm es_repeat_um lim_speed lim_accel lim_decel lim_home_fast
  cal_time wifi_reconn cal_running`. Extras: `fw microstep steps_mm calstatus identify`. `dist_mm` is sampled travel
  (approximate); `tcp_clients` is 0 or 1 (one COM client at a time).
- **HEALTH** (log tag `health`, first 10 s after the hub is reachable, then hourly, or on the `health` command): heap,
  reset reason, per-task stack headroom (`zb_motion zb_tcp zb_beacon zb_fleet zb_cal zb_wspush httpd tiT sys_evt wifi
  mdns main`), counters `wifi_reconn estop motion_err moves hub_fail log_drop`, and the metrics as `custom`.
- **Logs**: the agent hooks `esp_log` (chained to the previous vprintf, ISR-safe, never blocks, drops when busy) and
  ships batches to `/api/v1/ingest/logs` (hub source `c6`; the hub has no `c3` source). Domain events are logged with
  tag `zb_evt` as `EVT <name> key=value ...`: `boot homed move_done err estop s3_link cal_start cal_test cal_done
  cal_error fleet_cmd identify`. The hub's own patterns also match `WiFi connected, IP: x`, `WiFi disconnected,
  reason: n`, `Booting firmware <ver>`, `OTA partition marked valid` and `Heap: free=n, min=n`.
- **Motion path**: untouched. The motion and cal tasks only bump counters and post to a 16-entry queue / task
  notification (never blocking); metrics, logging and HTTP are done by the `zb_fleet` and agent tasks.
- **Commands** (hub `ALLOWED`: reboot, identify, ota, ping, factory_reset): `identify` logs and sets an extra (no
  motion); `stop`, `estop`, `home` (only when IDLE), `health`, `calstatus` are accepted if a hub sends them; no
  arbitrary moves; `factory_reset` is unsupported.
- **OTA safety**: the agent calls the gate callback first. `ota` is refused unless the axis is IDLE (the driver is then
  disabled before flash writes); `reboot`/`restart` E-stop a moving axis and disable the driver first. Rollback is on
  (`CONFIG_APP_ROLLBACK_ENABLE`): the agent confirms the image after the first hub heartbeat, `zb_fleet` after 90 s if
  WiFi (or the provisioning AP) is up. Known gap: a move commanded by the S3 or the web UI during the ~20 s download
  is not blocked; flash writes can disturb step timing, so do not move during an update.
- **Partitions** (4 MB): `nvs` 24 KB, `otadata` 8 KB, `phy_init`, `ota_0`/`ota_1` 1.5 MB each, `storage` (LittleFS)
  256 KB, 0x360000-0x400000 spare. The layout changed from the single-app table, so the first flash must be a full
  USB flash: `pio run -t upload && pio run -t uploadfs` (OTA cannot change the partition table).
- **Status**: `GET /api/status` and `GET /api/wifi` carry `fleet: {enabled, registered, hb_age, hub}`; the WiFi page
  shows it with the hub URL field.
