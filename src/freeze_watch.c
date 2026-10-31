#include "freeze_watch.h"

#if CONFIG_WM_DEBUG

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "esp_freertos_hooks.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "soc/interrupt_core0_reg.h"
#include "soc/interrupt_core1_reg.h"
#include "soc/soc.h"

static const char *TAG = "freeze";

#define CHECK_US        10000            /* poll every 10 ms */
#define STALL_US        150000           /* tick silent for this long = a stall */
#define REFRESH_US      250000           /* snapshot refresh while a stall lasts */
#define MAGIC           0x667a7774u

struct snap {
    uint32_t magic;
    uint32_t episodes;
    uint32_t stall_ms;                   /* length of the latest episode so far */
    uint32_t start_ms;                   /* uptime at which it began */
    uint32_t c0_tcb, c1_tcb;             /* task running on each core at the last refresh */
    uint32_t src0[4], src1[4];           /* interrupt controller status, sources 0..127, per core */
};

static RTC_NOINIT_ATTR struct snap s_snap;
static struct snap s_prev;
static bool s_prev_valid;
static uint32_t s_reported;

static int64_t s_last_check_us, s_tick_change_us, s_stall_start_us, s_last_refresh_us;
static TickType_t s_last_tick;
static bool s_stalled;

static const char *const k_tasks[] = {
    "dac_task", "grbl", "motion", "host_rx", "zbro", "atmega_link", "net_srv", "net_tx", "net_mgr",
    "espfleet", "tiT", "sys_evt", "esp_timer", "main", "wifi", "IDLE0", "IDLE1", "ipc0", "ipc1",
};

static void IRAM_ATTR capture(int64_t now_us)
{
    s_snap.stall_ms = (uint32_t)((now_us - s_stall_start_us) / 1000);
    s_snap.c0_tcb = (uint32_t)xTaskGetCurrentTaskHandleForCore(0);
    s_snap.c1_tcb = (uint32_t)xTaskGetCurrentTaskHandleForCore(1);
    s_snap.src0[0] = REG_READ(INTERRUPT_CORE0_INTR_STATUS_0_REG);
    s_snap.src0[1] = REG_READ(INTERRUPT_CORE0_INTR_STATUS_1_REG);
    s_snap.src0[2] = REG_READ(INTERRUPT_CORE0_INTR_STATUS_2_REG);
    s_snap.src0[3] = REG_READ(INTERRUPT_CORE0_INTR_STATUS_3_REG);
    s_snap.src1[0] = REG_READ(INTERRUPT_CORE1_INTR_STATUS_0_REG);
    s_snap.src1[1] = REG_READ(INTERRUPT_CORE1_INTR_STATUS_1_REG);
    s_snap.src1[2] = REG_READ(INTERRUPT_CORE1_INTR_STATUS_2_REG);
    s_snap.src1[3] = REG_READ(INTERRUPT_CORE1_INTR_STATUS_3_REG);
}

/* Core 1 idle hook: runs on every pass of IDLE1, with or without ticks. Must not block or log. */
static bool IRAM_ATTR watch_hook(void)
{
    int64_t now = esp_timer_get_time();
    TickType_t tk;

    if (now - s_last_check_us < CHECK_US)
        return true;
    s_last_check_us = now;
    tk = xTaskGetTickCount();
    if (tk != s_last_tick) {
        if (s_stalled)
            s_snap.stall_ms = (uint32_t)((now - s_stall_start_us) / 1000);
        s_last_tick = tk;
        s_tick_change_us = now;
        s_stalled = false;
        return true;
    }
    if (now - s_tick_change_us < STALL_US)
        return true;
    if (!s_stalled) {
        s_stalled = true;
        s_stall_start_us = s_tick_change_us;
        s_snap.episodes++;
        s_snap.start_ms = (uint32_t)(s_tick_change_us / 1000);
        s_last_refresh_us = 0;
    }
    if (now - s_last_refresh_us >= REFRESH_US) {
        s_last_refresh_us = now;
        capture(now);
    }
    return true;
}

static const char *name_of(uint32_t tcb)
{
    for (size_t i = 0; i < sizeof(k_tasks) / sizeof(k_tasks[0]); i++)
        if ((uint32_t)xTaskGetHandle(k_tasks[i]) == tcb)
            return k_tasks[i];
    return "?";
}

static bool abnormal_reset(void)
{
    esp_reset_reason_t r = esp_reset_reason();

    return r == ESP_RST_PANIC || r == ESP_RST_INT_WDT || r == ESP_RST_TASK_WDT || r == ESP_RST_WDT ||
           r == ESP_RST_BROWNOUT;
}

void freeze_watch_start(void)
{
    if (s_snap.magic == MAGIC && abnormal_reset() && s_snap.episodes) {
        s_prev = s_snap;
        s_prev_valid = true;
    }
    memset(&s_snap, 0, sizeof(s_snap));
    s_snap.magic = MAGIC;
    s_last_tick = xTaskGetTickCount();
    s_tick_change_us = esp_timer_get_time();
    esp_register_freertos_idle_hook_for_cpu(watch_hook, 1);
}

static void log_snap(const char *what, const struct snap *s)
{
    ESP_LOGE(TAG, "%s: stall #%lu began at %lu ms uptime, lasted >= %lu ms; core0 ran %s (0x%08lx), core1 ran %s (0x%08lx)",
             what, (unsigned long)s->episodes, (unsigned long)s->start_ms, (unsigned long)s->stall_ms,
             name_of(s->c0_tcb), (unsigned long)s->c0_tcb, name_of(s->c1_tcb), (unsigned long)s->c1_tcb);
    ESP_LOGE(TAG, "%s: pending interrupt sources core0 %08lx %08lx %08lx %08lx | core1 %08lx %08lx %08lx %08lx", what,
             (unsigned long)s->src0[0], (unsigned long)s->src0[1], (unsigned long)s->src0[2],
             (unsigned long)s->src0[3], (unsigned long)s->src1[0], (unsigned long)s->src1[1],
             (unsigned long)s->src1[2], (unsigned long)s->src1[3]);
}

static void log_tasks(void)
{
    char out[200];
    size_t len = 0;

    for (size_t i = 0; i < sizeof(k_tasks) / sizeof(k_tasks[0]); i++) {
        TaskHandle_t t = xTaskGetHandle(k_tasks[i]);

        if (!t)
            continue;
        len += snprintf(out + len, sizeof(out) - len, " %s=0x%08lx", k_tasks[i], (unsigned long)t);
        if (len > sizeof(out) - 28 || i + 1 == sizeof(k_tasks) / sizeof(k_tasks[0])) {
            ESP_LOGI(TAG, "task handles:%s", out);
            len = 0;
        }
    }
}

void freeze_watch_report(void)
{
    struct snap now;

    if (s_prev_valid) {
        s_prev_valid = false;
        log_snap("before the last reset", &s_prev);
        log_tasks();
    }
    now = s_snap;
    if (now.episodes != s_reported && !s_stalled) {
        s_reported = now.episodes;
        log_snap("stall recovered", &now);
    }
}

#endif /* CONFIG_WM_DEBUG */
