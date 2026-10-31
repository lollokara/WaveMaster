# Owner decisions (2026-10-31)

Answers to the five questions in PLAN.md section 5, given by the owner.

1. `$231` auto guide: keep default ON (1). No change to `src/calib.c`.
2. Z-Bro ESP32-C3 firmware (`CompanionZ/main/zb_net.c`): workers MAY edit and build it, the owner flashes.
   Do the evidence step first (REVIEW.md: read the C3 log and reset reason at a link drop) before changing it.
3. `$H` with the Z-Bro enabled (`$250=1`, `CAL_ZBRO_EN`): `$H` also homes Z, exactly like `$HZ`.
   With the Z-Bro disabled `$H` stays an accepted no-op. (PLAN.md called the setting `$300`; the real one is `$250`.)
4. Rayforge machine profile `supports_wcs`: ENABLE it. The flag exists only in the owner's uncommitted edits
   to `docs/rayforge/wavemaster-machine*.yaml` on the Rayforge Mac (not in this repo yet), so this is blocked
   until that Mac is awake and the edits are committed. Needs a bench check that G54/G92 behave.
5. ESPFleet telemetry: stays always on.
