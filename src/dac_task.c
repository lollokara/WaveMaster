/*
 * Output stage, consumer side (core 0): owns the SPI bus / AD3552R.
 *
 * Chunks from galvo_out.c are clocked out as ONE hardware-paced stream: the
 * SPI clock is 12 / tick_us MHz (12 quad clocks per X+Y point), CS stays low
 * across chunks, and each chunk is one data-only QIO transaction. Up to
 * MAX_INFLIGHT transactions are queued so the SPI driver's ISR starts the
 * next chunk immediately when one finishes.
 *
 * Laser gate lock-step: the SPI driver calls our pre-callback in the ISR
 * right before it starts a chunk. We capture a free-running GPTimer there,
 * set the chunk's initial gate level and arm the timer alarm for the first
 * gate edge at start + edge_tick * tick. The alarm ISR toggles the gate and
 * arms the next edge. Everything the ISRs touch lives in IRAM / DRAM.
 *
 * The task never sleeps in the streaming path: it blocks on a task
 * notification (new chunk, completion, hold release, abort).
 */
#include "dac_task.h"
#include "galvo_chunk.h"
#include "galvo_out.h"
#include "ad3552r_board.h"
#include "no_os_spi.h"
#include "atmega_link.h"
#include "calib.h"
#include "laser_io.h"
#include "driver/gptimer.h"
#include "driver/spi_master.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>

static const char *TAG = "dac_task";

#define DAC_TASK_STACK     6144
#define DAC_TASK_PRIORITY  10
#define DAC_TASK_CORE      0

#define MAX_INFLIGHT       3        /* transactions queued in the SPI driver */
#define IDLE_CLOSE_US      30000    /* close the stream after this long idle */
#define POWER_WAIT_MS      50

#define TIMER_RES_HZ       20000000u
#define TIMER_RES_MHZ      20u
/* Gate edge alarms never get armed closer than this to "now" (2 us). */
#define ARM_MARGIN_COUNTS  (2u * TIMER_RES_MHZ)
/* Delay between the pre-callback and the first SCLK edge of a chunk, in
 * timer counts. To be measured on the scope; compensates a constant skew
 * between the gate and the DAC samples (the laser on/off delays absorb any
 * remainder). */
#define START_LATENCY_COUNTS 0u

/* ---- ISR-shared state (DRAM) ------------------------------------------ */
static gptimer_handle_t s_timer;
static gptimer_alarm_config_t s_alarm_cfg;
static TaskHandle_t s_task;
static struct galvo_chunk *s_cur;       /* chunk currently on the wire */
static uint32_t s_edge_idx;
static uint64_t s_chunk_start;
static uint64_t s_next_edge_time;
static volatile bool s_alarm_armed;
static volatile bool s_gate_state;
static portMUX_TYPE s_edge_mux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_tick_q8;              /* timer counts per tick * 256 */
static atomic_int s_isr_inflight;       /* queued and not yet completed */
static volatile bool s_isr_drained;     /* inflight reached 0 */
static volatile bool s_isr_drain_lit;   /* ...while the gate was on */

static uint64_t IRAM_ATTR timer_now(void)
{
    unsigned long long v = 0;

    gptimer_get_raw_count(s_timer, &v);
    return v;
}

static void IRAM_ATTR isr_disarm(void)
{
    if (s_alarm_armed) {
        gptimer_set_alarm_action(s_timer, NULL);
        s_alarm_armed = false;
    }
}

static void IRAM_ATTR isr_toggle(void)
{
    s_gate_state = !s_gate_state;
    laser_io_gate(s_gate_state);
}

/* Arm the alarm for the next unapplied edge of s_cur; edges already in the
 * past are applied immediately. */
static void IRAM_ATTR isr_arm_next(void)
{
    struct galvo_chunk *c = s_cur;

    while (c && s_edge_idx < c->n_edges) {
        uint64_t t = s_chunk_start + (((uint64_t)c->edge_tick[s_edge_idx] * s_tick_q8) >> 8);
        uint64_t now = timer_now();

        if (t <= now) {
            isr_toggle();
            s_edge_idx++;
            continue;
        }
        if (t < now + ARM_MARGIN_COUNTS)
            t = now + ARM_MARGIN_COUNTS;
        s_next_edge_time = t;
        s_alarm_cfg.alarm_count = t;
        s_alarm_cfg.reload_count = 0;
        s_alarm_cfg.flags.auto_reload_on_alarm = false;
        s_alarm_armed = true;
        if (gptimer_set_alarm_action(s_timer, &s_alarm_cfg) != ESP_OK)
            s_alarm_armed = false;
        return;
    }
    isr_disarm();
}

static bool IRAM_ATTR on_alarm(gptimer_handle_t t, const gptimer_alarm_event_data_t *e, void *ctx)
{
    (void)t; (void)e; (void)ctx;
    portENTER_CRITICAL_ISR(&s_edge_mux);
    if (!s_cur || !s_alarm_armed) {
        portEXIT_CRITICAL_ISR(&s_edge_mux);
        return false; /* stale */
    }
    if (timer_now() < s_next_edge_time) {
        s_alarm_armed = false;
        isr_arm_next(); /* early/stale wake: re-arm the same edge */
        portEXIT_CRITICAL_ISR(&s_edge_mux);
        return false;
    }
    s_alarm_armed = false;
    isr_toggle();
    s_edge_idx++;
    isr_arm_next();
    portEXIT_CRITICAL_ISR(&s_edge_mux);
    return false;
}

/* Runs in the SPI ISR immediately before the chunk's transaction starts. */
static void IRAM_ATTR stream_pre(void *user)
{
    struct galvo_chunk *c = (struct galvo_chunk *)user;
    uint64_t now = timer_now();

    portENTER_CRITICAL_ISR(&s_edge_mux);
    isr_disarm();
    s_edge_idx = 0;
    s_chunk_start = now + START_LATENCY_COUNTS;
    s_gate_state = c->gate_start;
    laser_io_gate(c->gate_start);
    s_cur = c;
    isr_arm_next();
    portEXIT_CRITICAL_ISR(&s_edge_mux);
}

/* Runs in the SPI ISR right after the chunk's transaction completed (and,
 * if more are queued, just before the next stream_pre()). */
static void IRAM_ATTR stream_post(void *user)
{
    struct galvo_chunk *c = (struct galvo_chunk *)user;
    BaseType_t hp = pdFALSE;
    int left;

    portENTER_CRITICAL_ISR(&s_edge_mux);
    isr_disarm();
    s_cur = NULL;
    left = atomic_fetch_sub(&s_isr_inflight, 1) - 1;
    if (left <= 0) {
        /* Nothing queued behind us: the laser must not stay on. If it was
         * on, the motion task did not get to close the polyline in time:
         * that is a real underrun (a visible stop in a mark). */
        s_isr_drain_lit = s_gate_state || c->gate_end;
        s_gate_state = false;
        laser_io_gate(false);
        s_isr_drained = true;
    } else {
        s_gate_state = c->gate_end;
        laser_io_gate(c->gate_end);
    }
    portEXIT_CRITICAL_ISR(&s_edge_mux);

    if (s_task) {
        vTaskNotifyGiveFromISR(s_task, &hp);
        if (hp == pdTRUE)
            portYIELD_FROM_ISR();
    }
}

/* ---- task-side state ----------------------------------------------------- */
static SemaphoreHandle_t s_init_sem;
static SemaphoreHandle_t s_bus_mtx;     /* held for the whole stream session */
static bool s_init_ok;
static bool s_session_open;
static bool s_fault;
static spi_device_handle_t s_dev;
static struct galvo_chunk *s_next;      /* dequeued, waiting to be started */
static int s_outstanding;               /* queued minus reaped */
static int64_t s_idle_since_us;
static bool s_drained;
static bool s_arm_failed;
static bool s_power_warned;
static DMA_ATTR uint8_t s_last_point[GALVO_BYTES_PER_TICK];
static DMA_ATTR uint8_t s_final_point[GALVO_BYTES_PER_TICK];
static spi_transaction_t s_final_trans;

#if CONFIG_WM_DEBUG
/* Why the DAC lights its ALERT output: what each stream session leaves in the
 * status registers, and a controlled session ($DT) to tell the open/close
 * frames from the data frames. */
#define DBG_ALERT_LOG_MAX 20
#define DT_MAX_TICKS 128
#define DT_MAX_CHUNKS 8
static uint32_t s_dbg_chunks, s_dbg_ticks, s_dbg_last_ticks, s_dbg_alert_logs;
static uint16_t s_dbg_iface, s_dbg_err;
static SemaphoreHandle_t s_dt_done;
static volatile bool s_dt_pending;
static uint32_t s_dt_ticks, s_dt_chunks;
static int s_dt_rc;
static DMA_ATTR uint8_t s_dt_buf[DT_MAX_TICKS * GALVO_BYTES_PER_TICK];
#endif

void dac_task_notify(void)
{
    if (s_task)
        xTaskNotifyGive(s_task);
}

static void recycle(struct galvo_chunk *c)
{
    xQueueSend(g_galvo.free_q, &c, 0);
}

static void complete(struct galvo_chunk *c, bool done)
{
    if (done) {
        memcpy(s_last_point, c->wire + ((size_t)c->n_ticks - 1) * GALVO_BYTES_PER_TICK,
               GALVO_BYTES_PER_TICK);
        portENTER_CRITICAL(&g_galvo.pos_lock);
        g_galvo.pos_x = c->end_x;
        g_galvo.pos_y = c->end_y;
        portEXIT_CRITICAL(&g_galvo.pos_lock);
        atomic_fetch_add(&g_galvo.chunks_done, 1);
        atomic_fetch_add(&g_galvo.ticks_done, c->n_ticks);
    }
    atomic_fetch_sub(&g_galvo.ticks_queued, c->n_ticks);
    recycle(c);
}

static void reap(void)
{
    spi_transaction_t *t;

    while (s_outstanding > 0 && spi_device_get_trans_result(s_dev, &t, 0) == ESP_OK) {
        s_outstanding--;
        complete((struct galvo_chunk *)t->user, true);
    }
    if (s_outstanding == 0 && s_isr_drained) {
        s_isr_drained = false;
        s_drained = true;
        s_idle_since_us = esp_timer_get_time();
        /* A chunk already waiting in s_next means the drain was deliberate
         * (power/PRR barrier or arming); feed hold is deliberate too. */
        if (s_isr_drain_lit && !s_next && !atomic_load(&g_galvo.held)) {
            /* Counted when it happens, not when motion resumes, so the
             * stats are right even if the job never continues. */
            atomic_fetch_add(&g_galvo.underruns, 1);
            ESP_LOGW(TAG, "stream underrun with the laser on (host too slow?)");
        }
        s_isr_drain_lit = false;
    }
}

static bool session_open(void)
{
    float tick_us = galvo_out_tick_us();
    uint32_t hz = (uint32_t)lroundf(12.0e6f / tick_us);
    uint32_t actual;
    int32_t err;

    xSemaphoreTake(s_bus_mtx, portMAX_DELAY);
    err = ad3552r_board_stream_open(hz);
    if (err) {
        if (!s_fault)
            ESP_LOGE(TAG, "stream open failed: %ld", (long)err);
        xSemaphoreGive(s_bus_mtx);
        return false;
    }
    actual = ad3552r_board_spi_actual_hz();
    if (actual == 0)
        actual = hz;
    {
        float real_tick_us = 12.0e6f / (float)actual;

        if (fabsf(real_tick_us - tick_us) > 0.01f * tick_us)
            ESP_LOGW(TAG, "SPI clock %lu Hz gives tick %.3f us, producer assumes %.3f us",
                     (unsigned long)actual, (double)real_tick_us, (double)tick_us);
        s_tick_q8 = (uint32_t)lroundf(real_tick_us * (float)TIMER_RES_MHZ * 256.0f);
    }
    s_dev = (spi_device_handle_t)ad3552r_board_spi_dev();
    s_outstanding = 0;
    atomic_store(&s_isr_inflight, 0);
    s_isr_drained = false;
    s_drained = false;
    s_session_open = true;
    s_fault = false;
#if CONFIG_WM_DEBUG
    s_dbg_chunks = s_dbg_ticks = s_dbg_last_ticks = 0;
#endif
    s_idle_since_us = esp_timer_get_time();
    return true;
}

#if CONFIG_WM_DEBUG
/* Single-lane, stream closed, bus still held. Clears what it reads, so the
 * next session's flag is attributable to that session. NOT_READY (0x80) is a
 * status, not an error. */
static void dbg_alert_check(void)
{
    uint16_t mask = 0;

    s_dbg_iface = s_dbg_err = 0xFFFF;
    if (ad3552r_board_alert_status(&s_dbg_iface, &mask, &s_dbg_err, true) != 0)
        return;
    if (((s_dbg_iface & 0x7F) || s_dbg_err) && s_dbg_alert_logs < DBG_ALERT_LOG_MAX) {
        s_dbg_alert_logs++;
        ESP_LOGW(TAG, "DAC alert after session: iface=0x%02x err=0x%02x (chunks=%lu ticks=%lu last=%lu)",
                 s_dbg_iface, s_dbg_err, (unsigned long)s_dbg_chunks, (unsigned long)s_dbg_ticks,
                 (unsigned long)s_dbg_last_ticks);
    }
}
#endif

/* Precondition: nothing outstanding. Deasserts CS with a last transaction
 * that repeats the last point (the DAC holds its value), then returns the
 * bus to normal single-lane operation. */
static void session_close(void)
{
    esp_err_t err;

    if (!s_session_open)
        return;
    laser_io_gate(false);
    memcpy(s_final_point, s_last_point, sizeof(s_final_point));
    memset(&s_final_trans, 0, sizeof(s_final_trans));
    s_final_trans.flags = SPI_TRANS_MODE_QIO; /* no CS_KEEP_ACTIVE: CS rises */
    s_final_trans.length = GALVO_BYTES_PER_TICK * 8;
    s_final_trans.tx_buffer = s_final_point;
    err = spi_device_transmit(s_dev, &s_final_trans);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "final stream transaction failed: %s", esp_err_to_name(err));
    ad3552r_board_stream_close();
    s_session_open = false;
    s_drained = false;
    s_isr_drained = false;
#if CONFIG_WM_DEBUG
    dbg_alert_check();
#endif
    xSemaphoreGive(s_bus_mtx);
}

#if CONFIG_WM_DEBUG
/* $DT: a controlled session, run from this task so it cannot race the stream
 * state. Streams the held point (nothing moves, gate off), then reports what
 * session_close() found in the alert registers. */
static void dbg_run_session_test(void)
{
    spi_transaction_t t;
    uint16_t mask = 0, iface = 0, err = 0;
    uint32_t i;

    s_dt_rc = -1;
    s_dbg_iface = s_dbg_err = 0xFFFF;
    laser_io_gate(false);
    xSemaphoreTake(s_bus_mtx, portMAX_DELAY);
    ad3552r_board_alert_status(&iface, &mask, &err, true);
    xSemaphoreGive(s_bus_mtx);
    if (session_open()) {
        for (i = 0; i < s_dt_ticks; i++)
            memcpy(s_dt_buf + i * GALVO_BYTES_PER_TICK, s_last_point, GALVO_BYTES_PER_TICK);
        for (i = 0; s_dt_ticks && i < s_dt_chunks; i++) {
            memset(&t, 0, sizeof(t));
            t.flags = SPI_TRANS_MODE_QIO | SPI_TRANS_CS_KEEP_ACTIVE;
            t.length = (size_t)s_dt_ticks * GALVO_BYTES_PER_TICK * 8;
            t.tx_buffer = s_dt_buf;
            if (spi_device_transmit(s_dev, &t) != ESP_OK)
                break;
        }
        session_close();
        s_dt_rc = 0;
    }
    s_dt_pending = false;
    xSemaphoreGive(s_dt_done);
}

int dac_task_session_test(uint32_t ticks, uint32_t chunks, uint16_t *iface, uint16_t *err)
{
    if (!s_init_ok || ticks > DT_MAX_TICKS || chunks > DT_MAX_CHUNKS || s_dt_pending)
        return -1;
    s_dt_ticks = ticks;
    s_dt_chunks = chunks;
    s_dt_pending = true;
    dac_task_notify();
    if (xSemaphoreTake(s_dt_done, pdMS_TO_TICKS(3000)) != pdTRUE)
        return -1;
    *iface = s_dbg_iface;
    *err = s_dbg_err;
    return s_dt_rc;
}
#endif

/* True if this chunk will open the gate but the ATmega does not hold the
 * power word it was generated for (e.g. M11 zeroed it after the last job). */
static bool needs_power(const struct galvo_chunk *c)
{
    struct atmega_status st;

    if (s_fault || c->power_word < 0 || (!c->gate_start && c->n_edges == 0))
        return false;
    atmega_link_get_status(&st);
    return st.link_ok && st.power != (uint8_t)c->power_word;
}

static void apply_power(int16_t pw)
{
    atmega_link_set_power((uint8_t)pw);
    if (!atmega_link_wait_power((uint8_t)pw, POWER_WAIT_MS) && !s_power_warned) {
        ESP_LOGW(TAG, "ATmega did not confirm power %d within %d ms; continuing",
                 pw, POWER_WAIT_MS);
        s_power_warned = true;
    }
}

static void apply_barrier(struct galvo_chunk *c)
{
    if (c->barrier & GALVO_BARRIER_POWER) {
        apply_power(c->barrier_power);
    }
    if (c->barrier & GALVO_BARRIER_PRR)
        laser_io_set_prr(c->barrier_prr_hz, calib_get(CAL_PRR_DUTY));
    c->barrier = 0;
    atomic_fetch_add(&g_galvo.barriers, 1);
}

static bool needs_arm(const struct galvo_chunk *c)
{
    struct atmega_status st;

    if (s_fault || s_arm_failed || calib_get(CAL_AUTO_ARM) == 0.0f)
        return false;
    if (!c->gate_start && c->n_edges == 0)
        return false;
    atmega_link_get_status(&st);
    return !st.ready;
}

static void do_arm(void)
{
    uint32_t tmo = (uint32_t)calib_get(CAL_ARM_TIMEOUT_MS);

    /* Arming can take seconds: do not sit with CS low. */
    session_close();
    atmega_link_set_armed(true);
    if (!atmega_link_wait_ready(tmo)) {
        ESP_LOGE(TAG, "laser not ready after %lu ms; continuing without waiting again this session",
                 (unsigned long)tmo);
        s_arm_failed = true;
    }
}

/* The stream cannot open: drop the chunk, but no faster than it would have
 * played. Recycling instantly lets the motion task spin through the whole job
 * without ever blocking and starve IDLE1 into a task watchdog reset. The wait
 * is sliced so an abort or a feed hold is seen within DROP_SLICE_MS: a chunk
 * at a slow tick setting outlasts galvo_out_abort()'s 300 ms acknowledgement. */
#define DROP_SLICE_MS 10

static void drop_paced(struct galvo_chunk *c)
{
    uint32_t ms = (uint32_t)((float)c->n_ticks * galvo_out_tick_us() / 1000.0f) + 1;

    complete(c, false);
    while (ms && !atomic_load(&g_galvo.abort) && !atomic_load(&g_galvo.held)) {
        uint32_t slice = ms < DROP_SLICE_MS ? ms : DROP_SLICE_MS;

        vTaskDelay(pdMS_TO_TICKS(slice));
        ms -= slice;
    }
}

/* Returns 0 when the chunk was consumed (queued or dropped), 1 if it must
 * wait (kept in s_next). */
static int try_start(struct galvo_chunk *c)
{
    spi_transaction_t *t;
    esp_err_t err;

    if (c->barrier) {
        if (s_outstanding)
            return 1;
        apply_barrier(c);
    }
    if (c->n_ticks == 0) {
        recycle(c);
        return 0;
    }
    if (needs_arm(c)) {
        if (s_outstanding)
            return 1;
        do_arm();
    }
    if (needs_power(c)) {
        if (s_outstanding)
            return 1;               /* drain first: the gate goes off */
        apply_power(c->power_word);
        atomic_fetch_add(&g_galvo.barriers, 1);
    }
    if (!s_session_open && !session_open()) {
        if (!s_fault) {
            s_fault = true;
            laser_io_gate(false);
            atmega_link_set_armed(false);
            atmega_link_set_power(0);
            ESP_LOGE(TAG, "stream open failed; laser disarmed and power zeroed");
        }
        drop_paced(c);
        return 0;
    }
    if (s_outstanding >= MAX_INFLIGHT)
        return 1;

    s_drained = false;

    t = &c->trans;
    memset(t, 0, sizeof(*t));
    t->flags = SPI_TRANS_MODE_QIO | SPI_TRANS_CS_KEEP_ACTIVE;
    t->length = (size_t)c->n_ticks * GALVO_BYTES_PER_TICK * 8;
    t->tx_buffer = c->wire;
    t->user = c;

    atomic_fetch_add(&s_isr_inflight, 1);
    err = spi_device_queue_trans(s_dev, t, 0);
    if (err != ESP_OK) {
        atomic_fetch_sub(&s_isr_inflight, 1);
        ESP_LOGE(TAG, "queue_trans failed: %s", esp_err_to_name(err));
        complete(c, false);
        return 0;
    }
    s_outstanding++;
#if CONFIG_WM_DEBUG
    s_dbg_chunks++;
    s_dbg_ticks += (uint32_t)c->n_ticks;
    s_dbg_last_ticks = (uint32_t)c->n_ticks;
#endif
    return 0;
}

static void do_abort(void)
{
    spi_transaction_t *t;
    struct galvo_chunk *c;
    int timeouts = 0;

    if (s_next) {
        complete(s_next, false);
        s_next = NULL;
    }
    /* In-flight DMA transfers cannot be cancelled: let them finish (the
     * producer killed the gate, so nothing fires). */
    while (s_outstanding > 0 && timeouts < 20) {
        if (spi_device_get_trans_result(s_dev, &t, pdMS_TO_TICKS(50)) == ESP_OK) {
            s_outstanding--;
            complete((struct galvo_chunk *)t->user, true);
        } else {
            timeouts++;
        }
    }
    if (s_outstanding > 0)
        ESP_LOGE(TAG, "abort: %d transactions did not complete", s_outstanding);
    while (xQueueReceive(g_galvo.ready_q, &c, 0) == pdTRUE)
        complete(c, false);
    laser_io_gate(false);
    if (s_session_open && s_outstanding == 0)
        session_close();
    s_arm_failed = false;
    s_drained = false;
    atomic_store(&g_galvo.abort_go, false);
    xSemaphoreGive(g_galvo.abort_done);
}

/* One pass of the scheduler; returns how long to sleep for a notification. */
static TickType_t service(void)
{
    bool held;

    if (!atomic_load(&g_galvo.ready))
        return pdMS_TO_TICKS(50);

    if (s_session_open)
        reap();

    if (atomic_load(&g_galvo.abort)) {
        if (atomic_load(&g_galvo.abort_go))
            do_abort();
        return pdMS_TO_TICKS(5);
    }

#if CONFIG_WM_DEBUG
    if (s_dt_pending && !s_session_open && s_outstanding == 0 && !s_next)
        dbg_run_session_test();
#endif

    held = atomic_load(&g_galvo.held);
    while (!held && !atomic_load(&g_galvo.abort)) {
        if (!s_next && xQueueReceive(g_galvo.ready_q, &s_next, 0) != pdTRUE) {
            s_next = NULL;
            break;
        }
        if (try_start(s_next))
            break;
        s_next = NULL;
        held = atomic_load(&g_galvo.held);
    }

    if (s_session_open && s_outstanding == 0 && (!s_next || held)) {
        int64_t idle = esp_timer_get_time() - s_idle_since_us;
        bool more = uxQueueMessagesWaiting(g_galvo.ready_q) > 0 && !held;

        if (!more) {
            if (idle >= IDLE_CLOSE_US) {
                session_close();
                s_arm_failed = false;
                return portMAX_DELAY;
            }
            return pdMS_TO_TICKS((IDLE_CLOSE_US - idle) / 1000 + 1);
        }
    }
    if (s_outstanding > 0)
        return pdMS_TO_TICKS(5); /* safety net; completions notify us */
    return s_session_open ? pdMS_TO_TICKS(5) : portMAX_DELAY;
}

/* ---- boot --------------------------------------------------------------- */

static void boot_self_test(void)
{
    uint16_t want0 = 0, want1 = 0, got0 = 0, got1 = 0, zero0 = 0, zero1 = 0;
    int32_t e;

    e = ad3552r_board_write_volts(0, 2.5f);
    e |= ad3552r_board_write_volts(1, -2.5f);
    e |= ad3552r_board_volts_to_code(0, 2.5f, &want0);
    e |= ad3552r_board_volts_to_code(1, -2.5f, &want1);
    e |= ad3552r_read_reg(g_dac, AD3552R_REG_ADDR_CH_DAC_24B(0), &got0);
    e |= ad3552r_read_reg(g_dac, AD3552R_REG_ADDR_CH_DAC_24B(1), &got1);
    if (e)
        ESP_LOGE(TAG, "self-test: SPI/convert error %ld", (long)e);
    else if (got0 == want0 && got1 == want1)
        ESP_LOGI(TAG, "self-test OK: CH0=0x%04x CH1=0x%04x", got0, got1);
    else
        ESP_LOGE(TAG, "self-test MISMATCH: CH0=0x%04x (want 0x%04x) CH1=0x%04x (want 0x%04x)",
                 got0, want0, got1, want1);

    /* Park at 0 V and remember the 0 V point for the stream's closing hold. */
    ad3552r_board_write_volts(0, 0.0f);
    ad3552r_board_write_volts(1, 0.0f);
    ad3552r_board_volts_to_code(0, 0.0f, &zero0);
    ad3552r_board_volts_to_code(1, 0.0f, &zero1);
    s_last_point[0] = (uint8_t)(zero1 >> 8);
    s_last_point[1] = (uint8_t)zero1;
    s_last_point[2] = 0;
    s_last_point[3] = (uint8_t)(zero0 >> 8);
    s_last_point[4] = (uint8_t)zero0;
    s_last_point[5] = 0;
}

static bool timer_setup(void)
{
    gptimer_config_t cfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = TIMER_RES_HZ,
        .intr_priority = 3, /* above the SPI ISR so gate edges are not delayed by it */
    };
    gptimer_event_callbacks_t cbs = { .on_alarm = on_alarm };

    if (gptimer_new_timer(&cfg, &s_timer) != ESP_OK ||
        gptimer_register_event_callbacks(s_timer, &cbs, NULL) != ESP_OK ||
        gptimer_enable(s_timer) != ESP_OK ||
        gptimer_start(s_timer) != ESP_OK) {
        ESP_LOGE(TAG, "gate timer setup failed");
        return false;
    }
    return true;
}

static void dac_task(void *arg)
{
    (void)arg;
    s_task = xTaskGetCurrentTaskHandle();
    laser_io_gate(false);

    /* Everything bus-related is initialised on this (core 0) task. */
    if (!ad3552r_board_init() || !timer_setup()) {
        s_init_ok = false;
        xSemaphoreGive(s_init_sem);
        vTaskDelete(NULL);
        return;
    }
    ad3552r_esp32_spi_set_stream_cbs(stream_pre, stream_post);
    boot_self_test();
    ad3552r_board_guard_snapshot();     /* reference for the config guard */
    s_init_ok = true;
    xSemaphoreGive(s_init_sem);

    for (;;) {
        TickType_t wait = service();

        ulTaskNotifyTake(pdTRUE, wait);
    }
}

bool dac_task_start(void)
{
    s_init_sem = xSemaphoreCreateBinary();
    s_bus_mtx = xSemaphoreCreateMutex();
#if CONFIG_WM_DEBUG
    s_dt_done = xSemaphoreCreateBinary();
    if (!s_dt_done)
        return false;
#endif
    if (!s_init_sem || !s_bus_mtx)
        return false;
    if (xTaskCreatePinnedToCore(dac_task, "dac_task", DAC_TASK_STACK, NULL,
                                DAC_TASK_PRIORITY, NULL, DAC_TASK_CORE) != pdPASS)
        return false;
    if (xSemaphoreTake(s_init_sem, pdMS_TO_TICKS(10000)) != pdTRUE)
        return false;
    return s_init_ok;
}

bool dac_task_readback(uint16_t *code_x, uint16_t *code_y)
{
    int32_t e;

    if (!s_init_ok || xSemaphoreTake(s_bus_mtx, 0) != pdTRUE)
        return false; /* stream open (or not ready) */
    e = ad3552r_read_reg(g_dac, AD3552R_REG_ADDR_CH_DAC_24B(0), code_x);
    e |= ad3552r_read_reg(g_dac, AD3552R_REG_ADDR_CH_DAC_24B(1), code_y);
    xSemaphoreGive(s_bus_mtx);
    return e == 0;
}

int dac_task_regdump(const uint8_t **addrs, uint16_t *vals, size_t max)
{
    int n;

    if (!s_init_ok || xSemaphoreTake(s_bus_mtx, 0) != pdTRUE)
        return -1; /* stream open (or not ready) */
    n = (int)ad3552r_board_dump_regs(addrs, vals, max);
    xSemaphoreGive(s_bus_mtx);
    return n;
}

int dac_task_alert(uint16_t *iface, uint16_t *mask, uint16_t *err, bool clear)
{
    int32_t e;

    if (!s_init_ok || xSemaphoreTake(s_bus_mtx, 0) != pdTRUE)
        return -1; /* stream open (or not ready) */
    e = ad3552r_board_alert_status(iface, mask, err, clear);
    xSemaphoreGive(s_bus_mtx);
    return e ? -1 : 0;
}
