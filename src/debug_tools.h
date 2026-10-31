#ifndef DEBUG_TOOLS_H_
#define DEBUG_TOOLS_H_

#include "sdkconfig.h"

/*
 * Diagnostics for resets and freezes. They exist only in the debug build
 * (pio run -e esp32-s3-devkitc-1-debug, which sets CONFIG_WM_DEBUG); a normal build compiles them out and these
 * calls are empty.
 *
 *   - which task ran on core 1 in the seconds before a reset (sampled from core 0 into RTC memory)
 *   - the task watchdog's starved-task text and the interrupt hook that records it
 *   - minimum free stack of each task, every 30 s
 *   - the core dump summary (task, PC, backtrace) after a panic or watchdog reset
 *   - freeze_watch: stalls of core 0's tick interrupt, seen from core 1, with the interrupt controller state
 *
 * Everything is logged, so it reaches the hub through the ESPFleet agent.
 */
#if CONFIG_WM_DEBUG

/* Call once, before the ESPFleet agent starts. */
void debug_tools_start(void);

/* Call every 200 ms with a running counter. */
void debug_tools_poll(unsigned tick);

#else

static inline void debug_tools_start(void) {}
static inline void debug_tools_poll(unsigned tick) { (void)tick; }

#endif

#endif /* DEBUG_TOOLS_H_ */
