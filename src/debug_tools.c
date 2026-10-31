#include "debug_tools.h"

#if CONFIG_WM_DEBUG

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_core_dump.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freeze_watch.h"

static const char *TAG = "debug";

#define CRASH_REPORT_TICKS 150           /* 30 s after boot: the hub link is up */

#define TWDT_MAGIC 0x74776474u

/* The task watchdog only names the starved task on the console, which the hub never sees. The interrupt hook
 * below keeps that text, plus what each core was running, in RTC memory (it survives the reset) for the next
 * boot to log. */
static RTC_NOINIT_ATTR uint32_t s_twdt_magic;
static RTC_NOINIT_ATTR char s_twdt_info[224];
static RTC_NOINIT_ATTR uint32_t s_crash_seen;       /* fingerprint of the core dump already sent to the hub */

static void twdt_collect(void *opaque, const char *msg)
{
    (void)opaque;
    strlcat(s_twdt_info, msg, sizeof(s_twdt_info));
}

void IRAM_ATTR esp_task_wdt_isr_user_handler(void)
{
    char part[40];
    int cpus = 0;

    s_twdt_info[0] = '\0';
    esp_task_wdt_print_triggered_tasks(twdt_collect, NULL, &cpus);
    for (int core = 0; core < 2; core++) {
        TaskHandle_t t = xTaskGetCurrentTaskHandleForCore(core);

        snprintf(part, sizeof(part), " | core%d running %s", core, t ? pcTaskGetName(t) : "?");
        strlcat(s_twdt_info, part, sizeof(s_twdt_info));
    }
    s_twdt_magic = TWDT_MAGIC;
}

static void twdt_report(void)
{
    size_t n = strnlen(s_twdt_info, sizeof(s_twdt_info) - 1);
    char chunk[100];

    if (s_twdt_magic != TWDT_MAGIC)
        return;
    s_twdt_magic = 0;
    s_twdt_info[sizeof(s_twdt_info) - 1] = '\0';
    for (size_t i = 0; i < n; i++)
        if (s_twdt_info[i] == '\n' || s_twdt_info[i] == '\r')
            s_twdt_info[i] = ' ';
    for (size_t off = 0; off < n; off += sizeof(chunk) - 1) {
        strlcpy(chunk, s_twdt_info + off, sizeof(chunk));
        ESP_LOGE(TAG, "task watchdog:%s", chunk);
    }
}

#define SAMPLE_MS    100
#define SAMPLE_N     64
#define SAMPLE_MAGIC 0x736d706cu

/* Which task held core 1 in the seconds before a reset: an esp_timer callback (it runs on core 0) records the
 * task running on core 1 every SAMPLE_MS into RTC memory, and the boot after an abnormal reset logs the
 * sequence. A task that never yields shows up as one long run. */
struct sample {
    uint32_t tick;
    char core1[8];
};

static RTC_NOINIT_ATTR uint32_t s_smp_magic;
static RTC_NOINIT_ATTR uint32_t s_smp_n;
static RTC_NOINIT_ATTR struct sample s_smp[SAMPLE_N];
static struct sample s_prev[SAMPLE_N];
static uint32_t s_prev_n;
static bool s_prev_lost;

static bool abnormal_reset(void)
{
    esp_reset_reason_t r = esp_reset_reason();

    return r == ESP_RST_PANIC || r == ESP_RST_INT_WDT || r == ESP_RST_TASK_WDT || r == ESP_RST_WDT ||
           r == ESP_RST_BROWNOUT;
}

static void sample_cb(void *arg)
{
    TaskHandle_t t = xTaskGetCurrentTaskHandleForCore(1);
    struct sample *s = &s_smp[s_smp_n % SAMPLE_N];

    (void)arg;
    s->tick = xTaskGetTickCount();
    strlcpy(s->core1, t ? pcTaskGetName(t) : "?", sizeof(s->core1));
    s_smp_n++;
}

static void sample_start(void)
{
    static esp_timer_handle_t timer;
    const esp_timer_create_args_t args = { .callback = sample_cb, .name = "smp" };

    if (abnormal_reset()) {
        if (s_smp_magic == SAMPLE_MAGIC) {
            memcpy(s_prev, s_smp, sizeof(s_prev));
            s_prev_n = s_smp_n;
        } else {
            s_prev_lost = true;
        }
    }
    s_smp_magic = SAMPLE_MAGIC;
    s_smp_n = 0;
    if (esp_timer_create(&args, &timer) == ESP_OK)
        esp_timer_start_periodic(timer, (uint64_t)SAMPLE_MS * 1000);
}

static void sample_report(void)
{
    uint32_t count = s_prev_n < SAMPLE_N ? s_prev_n : SAMPLE_N;
    uint32_t base = s_prev_n - count;
    char out[200];
    size_t len = 0;

    if (s_prev_lost)
        ESP_LOGW(TAG, "no core 1 samples: RTC memory did not survive the reset");
    for (uint32_t k = 0; k < count;) {
        const struct sample *a = &s_prev[(base + k) % SAMPLE_N];
        uint32_t run = 1;

        if (k) {
            uint32_t gap = a->tick - s_prev[(base + k - 1) % SAMPLE_N].tick;

            if (gap > 3 * SAMPLE_MS)
                len += snprintf(out + len, sizeof(out) - len, " [+%lums]", (unsigned long)gap);
        }
        while (k + run < count && strcmp(s_prev[(base + k + run) % SAMPLE_N].core1, a->core1) == 0)
            run++;
        len += snprintf(out + len, sizeof(out) - len, " %s*%lu", a->core1, (unsigned long)run);
        k += run;
        if (len > sizeof(out) - 28 || k >= count) {
            ESP_LOGE(TAG, "core1 before the reset (%d ms samples, oldest first):%s", SAMPLE_MS, out);
            len = 0;
        }
    }
}

/* Minimum free stack (bytes) of each task, logged every 30 s: an overflow shows up as a value near zero. */
static void stack_report(void)
{
    static const char *const names[] = {
        "dac_task", "grbl", "motion", "host_rx", "zbro", "atmega_link", "net_srv", "net_tx", "net_mgr",
        "espfleet", "tiT", "sys_evt", "esp_timer", "main", "wifi", "IDLE1",
    };
    char out[200];
    size_t len = 0;

    for (size_t i = 0; i < sizeof(names) / sizeof(names[0]); i++) {
        TaskHandle_t t = xTaskGetHandle(names[i]);

        if (!t)
            continue;
        len += snprintf(out + len, sizeof(out) - len, " %s=%u", names[i], (unsigned)uxTaskGetStackHighWaterMark(t));
        if (len > sizeof(out) - 24 || i + 1 == sizeof(names) / sizeof(names[0])) {
            ESP_LOGI(TAG, "stack min free (B):%s", out);
            len = 0;
        }
    }
}

/* A panic or watchdog reset leaves its summary in the coredump partition. Log it so the hub gets the task, the
 * PC and the backtrace (resolve them with the firmware.elf whose sha is printed). The dump stays in flash: it
 * holds every task's stack, and esptool can read it until the next crash replaces it. Each dump is logged once
 * (fingerprint kept in RTC memory). */
static void crash_report(void)
{
    esp_core_dump_summary_t *s;
    char line[96];
    size_t len = 0;
    uint32_t fp;
    esp_err_t err = esp_core_dump_image_check();

    twdt_report();
    sample_report();
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "no core dump from the last reset (%s)", esp_err_to_name(err));
        return;
    }
    s = malloc(sizeof(*s));
    if (!s)
        return;
    if (esp_core_dump_get_summary(s) == ESP_OK) {
        uint32_t n = s->exc_bt_info.depth < 16 ? s->exc_bt_info.depth : 16;

        fp = s->exc_pc;
        for (uint32_t i = 0; i < n; i++)
            fp = fp * 31u + s->exc_bt_info.bt[i];
        if (fp == s_crash_seen) {
            free(s);
            return;
        }
        s_crash_seen = fp;
        ESP_LOGE(TAG, "last crash: task %s pc 0x%08lx elf %.9s%s", s->exc_task, (unsigned long)s->exc_pc,
                 (const char *)s->app_elf_sha256, s->exc_bt_info.corrupted ? " (backtrace corrupted)" : "");
        line[0] = '\0';
        for (uint32_t i = 0; i < n; i++) {
            len += snprintf(line + len, sizeof(line) - len, " 0x%08lx", (unsigned long)s->exc_bt_info.bt[i]);
            if (len > sizeof(line) - 14 || i + 1 == n) {
                ESP_LOGE(TAG, "backtrace:%s", line);
                len = 0;
                line[0] = '\0';
            }
        }
    }
    free(s);
}

void debug_tools_start(void)
{
    sample_start();
    freeze_watch_start();
    ESP_LOGW(TAG, "debug instrumentation is on (CONFIG_WM_DEBUG)");
}

void debug_tools_poll(unsigned tick)
{
    if (tick == CRASH_REPORT_TICKS)
        crash_report();
    if (tick % 150 == 149)
        stack_report();
    if (tick % 5 == 0)
        freeze_watch_report();
}

#endif /* CONFIG_WM_DEBUG */
