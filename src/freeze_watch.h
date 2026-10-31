#ifndef FREEZE_WATCH_H_
#define FREEZE_WATCH_H_

#include "sdkconfig.h"

#if CONFIG_WM_DEBUG

/*
 * Detects stalls of core 0's tick interrupt from core 1 and records what the chip was doing.
 *
 * Only core 0 advances the FreeRTOS tick, so a task on core 1 that sleeps with vTaskDelay stops waking when
 * core 0 stalls. The detector therefore runs in the core 1 idle hook, which keeps running without ticks. While
 * a stall lasts it keeps a snapshot in RTC memory (it survives a watchdog reset): the task each core was
 * running and the raw interrupt controller status words of both cores.
 */

/* Registers the idle hook and starts a fresh snapshot, keeping the previous one if the last reset was abnormal. */
void freeze_watch_start(void);

/* Logs a stall that ended by itself, and (after an abnormal reset) the one that was running at the reset. */
void freeze_watch_report(void);

#endif /* CONFIG_WM_DEBUG */

#endif /* FREEZE_WATCH_H_ */
