# WiFi host link: plan

Branch: `feature/wifi-ota-printing`. Status: plan only, no code yet.

Goal: Rayforge connects to the WaveMaster over WiFi and runs jobs with the stock
Rayforge driver, with no change to the streaming path, its timing or its safety
rules (`AGENTS.md` section 2).

## Decisions

| # | Decision | Why |
|---|---|---|
| 1 | **Transport is a raw TCP socket, port 23, used with Rayforge's `GrblTelnetDriver` ("GRBL (Telnet)").** | Checked in the Rayforge source (clone of `main`, 2026-10-12): `GrblTelnetDriver` runs the same `raydriver` `GrblSession` as the serial driver, with the same character counting. Only the transport differs. The firmware already speaks that protocol over USB, so WiFi is a second byte pipe into the same code. |
| 2 | **No upload-and-run ("GRBL (Network)") in this plan.** | `GrblNetworkDriver` uploads a whole file by HTTP (`/upload`), then starts it with `[ESP220]`, and reads status over a WebSocket. That needs an on-flash file system, a job runner in the firmware, and ESP3D/FluidNC endpoint emulation (`[ESP420]`, `[ESP800]`, `[ESP400]`). It is a separate, larger project. Its one real benefit is isolating a running job from WiFi quality. Revisit only if the streaming tests in phase 4 fail. |
| 3 | **WiFi is off by default.** New setting enables it. | A WiFi-reachable laser controller must be opt-in. |
| 4 | **Station mode only, WPA2 or better, no SoftAP, no OTA.** | Smaller heap and attack surface. A SoftAP and OTA can be added later; OTA must refuse while the laser is armed. |
| 5 | **Credentials are set over USB and stored in their own NVS namespace.** | `calib.c` holds numeric settings only. Strings do not fit its table, so do not stretch it. The enable flag is a normal `$` setting. |
| 6 | **One host at a time.** USB and TCP feed the same stream buffer. Replies and logs go to the active host. The active host can change only when the machine is idle. Ctrl-X from either transport always works. | Two hosts must never interleave G-code or replies. |
| 7 | **TCP disconnect during a job = abort, same path as Ctrl-X** (gate off, disarm). | Rayforge's session cannot resume a job after reconnecting, and a buffered job must not keep marking with nobody attached. `poll_status_while_running` is false in the shipped profile, so silence is not a usable watchdog. Detect loss with TCP keepalive instead. |
| 8 | **Everything WiFi runs on core 1.** | Core 0 belongs to `dac_task` and the gate ISRs. |
| 9 | **No authentication in phase 1; say so in the docs.** | Raw GRBL over TCP has no auth. Anyone on the LAN can send G-code. Mitigations: off by default, one client, laser disarmed unless armed, and a note to put the controller on its own network. |

## Architecture

```
USB-Serial-JTAG --\                                   /--> reply/log TX (active host)
                   >-- host_link (transport layer) --<
TCP :23 (core 1) --/        |                         \--> realtime handlers (existing)
                            v
                   RX stream buffer (existing) --> grbl task (unchanged)
```

- `host_serial.c` keeps the realtime-byte handling, `HOST_RX_RESET_MARK`, the TX mutex and the `[MSG:...]` log
  redirect. Its USB-specific code (driver install, `usb_serial_jtag_*` calls) moves behind a small transport interface:
  `read`, `write`, `connected`.
- `net_wifi.c` brings the station up and reconnects. `net_server.c` owns the listener and one client socket.
- `grbl.c` and everything below it do not change, apart from the disconnect abort hook (decision 7).

## Real-time and heap rules for the new code

- Never call SPI, touch the DAC, or take the TX mutex from a core-0 task. Never block `dac_task`.
- No new work in ISRs. No logging from ISRs.
- Socket writes use a short timeout and drop the data on timeout, like `tx_locked()` does today, so a stuck client cannot stall `grbl` or the logs.
- Disable Nagle (`TCP_NODELAY`). Use `WIFI_PS_NONE` while a client is connected.
- Set `SO_KEEPALIVE` (about 3 s idle, 1 s interval, 3 probes) so a dead client is detected in about 6 s.
- No large buffers. Record the internal heap before and after WiFi starts, and during a job.

## Phases

Every phase must build with `pio run` and zero warnings in `src/`. If `motion.c`, `motion.h` or `gcode_parse.c` change, run the
two host test suites. Nothing below counts as verified until the listed measurement was done.

### Phase 1. Build config and transport split (USB behaviour unchanged)
1. `sdkconfig.defaults`: pin the WiFi task and the lwIP `tcpip_thread` to core 1, and move the `esp_timer` task to core 1.
   Use RAM storage for WiFi config. Trim WiFi and lwIP buffer counts. Add the IRAM options for WiFi RX and lwIP only if the
   heap allows it. Confirm each option name in menuconfig for ESP-IDF 5.5 when doing this; I have not checked them.
2. Split the USB code in `host_serial.c` behind the transport interface.
3. **Gate:** with WiFi still disabled, the firmware behaves exactly as before. Run the USB checks from `STATUS.md` (`$S`, `$RB`, a job with
   `underruns=0`).

### Phase 2. WiFi station and TCP server
1. `net_wifi.c`: start WiFi from a task pinned to core 1 so its interrupts are allocated there. Reconnect with backoff.
2. USB commands to provision it (for example `$WIFI=<ssid>,<password>`, `$WIFI?` to show state and IP), plus the enable setting
   (new `$` number, off by default). Follow the `calib.h` / `calib.c` rules: no reused number, NVS key of 15 characters or fewer.
3. `net_server.c`: listen on port 23, accept one client, refuse and close any extra one. Feed received bytes into the host link.
   Send replies and logs to the client.
4. Active-host switching and the disconnect abort (decisions 6 and 7).
5. **Gate:** `nc <ip> 23`, then `$I`, `?` and a few jogs work. A second client is refused. Pulling the client mid-job aborts with the gate off.

### Phase 3. Rayforge profile and docs
1. Add a Telnet variant of `docs/rayforge/wavemaster-machine.yaml` (`GrblTelnetDriver`, `host`, `port: 23`, same
   `poll_status_while_running: false`, same `driver_config.rx_buffer_size: 1024`). Keep the USB profile. Add the matching `.rfdevice` variant.
2. Update `docs/RAYFORGE.md` (connection section) and `README.md`. Document the no-auth warning and the provisioning commands.
3. `tools/stream_test.py`: add a `host:port` form for `port` so the same patterns can run over TCP.
4. **Gate:** Rayforge connects, frames (guide laser, `$231`), and runs a short job over WiFi.

### Phase 4. Measurements (this decides whether the plan is good enough)
Each result goes into `STATUS.md` under "Verified on hardware". Anything not measured stays under "Not verified".

| Test | Pass criterion |
|---|---|
| Scope SCLK gap at chunk boundaries and gate-to-DAC offset, WiFi idle vs. heavy traffic (for example a large `ping -f` plus a job) | No visible change in gap or offset compared with USB. If it changes, fix the core/interrupt placement before continuing. |
| Internal heap at boot, WiFi up, client connected and during a job | Record the minimum free internal heap. It must stay clear of the point where the chunk pool allocation or task creation would fail. Decide the numeric floor after the first measurement. |
| Long dense raster job over WiFi | `$S` shows `underruns=0`. |
| Ctrl-X over WiFi on a long line | Gate off immediately. Record the stop latency and compare it with the roughly 60 ms USB target. If WiFi cannot meet it, say so in the docs. |
| Router off / cable pulled mid-job | Abort within about 6 s, laser off and disarmed. |
| WiFi stall during a job (move away from the AP) | Job either completes from the buffer or underruns cleanly with the laser gated off. It must never leave the laser on. |

## Out of scope for now
- Upload-and-run through `GrblNetworkDriver` (decision 2).
- OTA updates, SoftAP provisioning, mDNS (`wavemaster.local`), TLS or any authentication.
- Moving the ATmega link or the DAC path. They are untouched.

## Open risks
- WiFi interrupt placement and `esp_timer` affinity are my expectation from ESP-IDF behaviour, not verified here. The phase 4 scope
  test is the check.
- `GrblTelnetDriver` is documented upstream for grblHAL and ESP3D raw-TCP bridges. It has not been tried against this firmware. Phase 2 tests
  it with `nc` first, and phase 3 with Rayforge itself.
- Rayforge's `raydriver` session is a compiled dependency, so its exact handling of the `Bf:` and `[OPT:` replies over TCP is read from the
  Python side only. Phase 3 is where this is really tested.
