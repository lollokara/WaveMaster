/*
 * Output stage, producer side (core 1, single producer = motion_task).
 *
 * galvo_out_tick() turns one trajectory sample (machine mm + power word)
 * into 6 wire bytes for the AD3552R stream and a laser-gate waveform, and
 * packs them into chunks that dac_task.c clocks out. See galvo_chunk.h for
 * the chunk contract and galvo_out.h for the public behaviour.
 */
#include "galvo_out.h"
#include "galvo_chunk.h"
#include "ad3552r_board.h"
#include "no_os_util.h"
#include "calib.h"
#include "laser_io.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

static const char *TAG = "galvo_out";

struct galvo_shared g_galvo = {
    .pos_lock = portMUX_INITIALIZER_UNLOCKED,
};

#define PENDING_FIFO_LEN 256   /* pending delayed gate edges */
#define SPI_QUAD_CLOCKS_PER_POINT 12.0f
#define SPI_MAX_CLOCK_HZ 60000000.0f
#define CHUNK_TARGET_US 20000.0f /* cap on chunk duration (abort latency) */

/* ---- cached configuration (galvo_out_reload) -------------------------- */
static float s_tick_us = 10.0f;
static float s_org_x, s_org_y;
static float s_sgn_x = 1.0f, s_sgn_y = 1.0f;
static bool s_swap;
static float s_k1;
static float s_m0, s_c0, s_lo0, s_hi0; /* code = f*m + c, clamped to [lo,hi] */
static float s_m1, s_c1, s_lo1, s_hi1;
static uint32_t s_delay_on_ticks, s_delay_off_ticks;
static bool s_pd_mode;
static uint32_t s_pd_n;
static int s_pd_word;
static uint32_t s_chunk_limit = GALVO_CHUNK_TICKS;

/* ---- producer state ---------------------------------------------------- */
static struct galvo_chunk *s_pool;
static struct galvo_chunk *s_cur;
static bool s_cur_gate;        /* gate level at the last tick written to s_cur */
static uint32_t s_tick_no;     /* global tick counter (edge due times) */
static bool s_des_last;        /* desired gate at the previous tick */
static bool s_out_gate;        /* output gate level (after delays) */
static uint32_t s_pd_phase;
static int s_atmega_power = -1; /* last power word requested; -1 unknown */
static uint8_t s_pend_flags;
static uint8_t s_pend_power;
static float s_pend_prr;
static atomic_bool s_in_prod;
/* galvo_out_abort() is reached from two tasks at once on a soft reset
 * (motion_abort() in the USB RX task, and the motion task's own abort
 * handler); both reset the producer and return s_cur to the pool, so
 * they must not overlap. A second abort right after the first is cheap. */
static SemaphoreHandle_t s_abort_mtx;

static struct {
    uint32_t due;
    bool level;
} s_fifo[PENDING_FIFO_LEN];
static uint32_t s_fifo_head, s_fifo_count;

static inline bool prod_enter(void)
{
    atomic_store(&s_in_prod, true);
    if (atomic_load(&g_galvo.abort) || !atomic_load(&g_galvo.ready)) {
        atomic_store(&s_in_prod, false);
        return false;
    }
    return true;
}

static inline void prod_leave(void)
{
    atomic_store(&s_in_prod, false);
}

/* ---- configuration ------------------------------------------------------ */

static void code_map(uint8_t ch, float scale_v_per_mm, float off_v,
                     float *m, float *c, float *lo, float *hi)
{
    float inv = 1.0f, offc = -32768.0f;
    float a, b;

    if (g_dac && ad3552r_board_code_map(ch, &inv, &offc) != 0) {
        inv = 1.0f / 0.305176f;
        offc = -32768.0f;
        ESP_LOGE(TAG, "code map lookup failed for ch%u, using +-10V defaults", ch);
    } else if (!g_dac) {
        inv = 1.0f / 0.305176f;
    }
    *m = scale_v_per_mm * 1000.0f * inv;
    *c = off_v * 1000.0f * inv - offc;
    /* +-10 V clamp expressed in code domain, then the 16-bit range. */
    a = -10000.0f * inv - offc;
    b = 10000.0f * inv - offc;
    *lo = fmaxf(0.0f, fminf(a, b));
    *hi = fminf(65535.0f, fmaxf(a, b));
}

void galvo_out_reload(void)
{
    float req = calib_get(CAL_TICK_US);
    float hz = SPI_QUAD_CLOCKS_PER_POINT * 1.0e6f / req;
    uint32_t inv_mask = (uint32_t)calib_get(CAL_AXIS_INVERT);

    if (hz > SPI_MAX_CLOCK_HZ)
        hz = SPI_MAX_CLOCK_HZ;
    uint32_t actual = g_dac ? ad3552r_board_quantize_clock((uint32_t)hz) : (uint32_t)hz;
    if (actual == 0)
        actual = (uint32_t)hz;
    s_tick_us = SPI_QUAD_CLOCKS_PER_POINT * 1.0e6f / (float)actual;

    if (calib_get(CAL_ORIGIN_CENTER) != 0.0f) {
        s_org_x = s_org_y = 0.0f;
    } else {
        s_org_x = calib_get(CAL_FIELD_W) * 0.5f;
        s_org_y = calib_get(CAL_FIELD_H) * 0.5f;
    }
    s_sgn_x = (inv_mask & 1) ? -1.0f : 1.0f;
    s_sgn_y = (inv_mask & 2) ? -1.0f : 1.0f;
    s_swap = calib_get(CAL_SWAP_XY) != 0.0f;
    s_k1 = calib_get(CAL_RADIAL_K1);

    /* Field axis 0 is DAC channel 0 (X), axis 1 is channel 1 (Y). */
    code_map(0, calib_get(CAL_X_SCALE), calib_get(CAL_X_OFFSET), &s_m0, &s_c0, &s_lo0, &s_hi0);
    code_map(1, calib_get(CAL_Y_SCALE), calib_get(CAL_Y_OFFSET), &s_m1, &s_c1, &s_lo1, &s_hi1);

    s_delay_on_ticks = (uint32_t)lroundf(calib_get(CAL_LASER_ON_DELAY) / s_tick_us);
    s_delay_off_ticks = (uint32_t)lroundf(calib_get(CAL_LASER_OFF_DELAY) / s_tick_us);

    s_pd_mode = calib_get(CAL_POWER_MODE) != 0.0f;
    s_pd_n = (uint32_t)calib_get(CAL_PD_PERIOD);
    if (s_pd_n < 2)
        s_pd_n = 2;
    s_pd_word = (int)lroundf(calib_get(CAL_POWER_MAX) * 255.0f / 100.0f);
    if (s_pd_word > 255)
        s_pd_word = 255;
    s_pd_phase = 0;
    s_atmega_power = -1; /* mode / max may have changed: resend on demand */

    float lim = CHUNK_TARGET_US / s_tick_us;
    s_chunk_limit = lim >= (float)GALVO_CHUNK_TICKS ? GALVO_CHUNK_TICKS
                    : (lim < 64.0f ? 64u : (uint32_t)lim);

    laser_io_set_gate_active_low(calib_get(CAL_GATE_ACTIVE_LOW) != 0.0f);
}

void galvo_out_init(void)
{
    size_t i;

    if (atomic_load(&g_galvo.ready))
        return;

    s_pool = heap_caps_calloc(GALVO_CHUNK_POOL, sizeof(*s_pool), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    g_galvo.free_q = xQueueCreate(GALVO_CHUNK_POOL, sizeof(struct galvo_chunk *));
    g_galvo.ready_q = xQueueCreate(GALVO_CHUNK_POOL, sizeof(struct galvo_chunk *));
    g_galvo.abort_done = xSemaphoreCreateBinary();
    s_abort_mtx = xSemaphoreCreateMutex();
    if (!s_abort_mtx || !s_pool || !g_galvo.free_q || !g_galvo.ready_q || !g_galvo.abort_done) {
        ESP_LOGE(TAG, "out of memory for chunk pool");
        return;
    }
    for (i = 0; i < GALVO_CHUNK_POOL; i++) {
        struct galvo_chunk *c = &s_pool[i];

        c->wire = heap_caps_aligned_alloc(16, GALVO_CHUNK_TICKS * GALVO_BYTES_PER_TICK,
                                          MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        if (!c->wire) {
            ESP_LOGE(TAG, "out of DMA memory for chunk %u", (unsigned)i);
            return;
        }
        xQueueSend(g_galvo.free_q, &c, 0);
    }

    galvo_out_reload();
    laser_io_set_prr(calib_get(CAL_PRR_HZ), calib_get(CAL_PRR_DUTY));
    ESP_LOGI(TAG, "tick %.3f us (requested %.3f), chunk %u ticks, pool %d x %d B",
             (double)s_tick_us, (double)calib_get(CAL_TICK_US), (unsigned)s_chunk_limit,
             GALVO_CHUNK_POOL, GALVO_CHUNK_TICKS * GALVO_BYTES_PER_TICK);
    atomic_store(&g_galvo.ready, true);
    dac_task_notify();
}

float galvo_out_tick_us(void)
{
    return s_tick_us;
}

/* ---- chunk handling ----------------------------------------------------- */

static void submit_cur(void)
{
    struct galvo_chunk *c = s_cur;

    if (!c)
        return;
    s_cur = NULL;
    if (c->n_ticks == 0 && c->barrier == 0) {
        xQueueSend(g_galvo.free_q, &c, 0);
        return;
    }
    c->gate_end = s_cur_gate;
    atomic_fetch_add(&g_galvo.ticks_queued, c->n_ticks);
    xQueueSend(g_galvo.ready_q, &c, 0); /* queue length == pool: never full */
    dac_task_notify();
}

static bool open_chunk(bool gate)
{
    struct galvo_chunk *c;

    for (;;) {
        if (atomic_load(&g_galvo.abort))
            return false;
        if (xQueueReceive(g_galvo.free_q, &c, pdMS_TO_TICKS(2)) == pdTRUE)
            break;
    }
    c->n_ticks = 0;
    c->n_edges = 0;
    c->gate_start = gate;
    c->gate_end = gate;
    c->barrier = s_pend_flags;
    c->barrier_power = s_pend_power;
    c->barrier_prr_hz = s_pend_prr;
    c->power_word = (int16_t)s_atmega_power;
    s_pend_flags = 0;
    s_cur = c;
    s_cur_gate = gate;
    return true;
}

/* Barrier before the next tick: close the current chunk if it has data and
 * attach the action to the chunk that follows. */
static void request_barrier(uint8_t flag, uint8_t power, float prr_hz)
{
    if (s_cur && s_cur->n_ticks > 0)
        submit_cur();
    if (flag & GALVO_BARRIER_POWER) {
        if (s_cur) {
            s_cur->barrier |= GALVO_BARRIER_POWER;
            s_cur->barrier_power = power;
            s_cur->power_word = power;
        }
        else { s_pend_flags |= GALVO_BARRIER_POWER; s_pend_power = power; }
    }
    if (flag & GALVO_BARRIER_PRR) {
        if (s_cur) { s_cur->barrier |= GALVO_BARRIER_PRR; s_cur->barrier_prr_hz = prr_hz; }
        else { s_pend_flags |= GALVO_BARRIER_PRR; s_pend_prr = prr_hz; }
    }
}

/* ---- gate edge delay FIFO ------------------------------------------------ */

static inline void fifo_schedule(bool level)
{
    uint32_t due = s_tick_no + (level ? s_delay_on_ticks : s_delay_off_ticks);

    if (s_fifo_count) {
        uint32_t tail = (s_fifo_head + s_fifo_count - 1) % PENDING_FIFO_LEN;

        /* Would land at or before the previous pending edge: the two
         * cancel (a pulse or a gap shorter than the delay skew vanishes). */
        if ((int32_t)(due - s_fifo[tail].due) <= 0) {
            s_fifo_count--;
            return;
        }
        if (s_fifo_count == PENDING_FIFO_LEN) {
            /* Pathological (huge delay with a very fast gate): apply the
             * oldest edge now rather than lose one. */
            s_out_gate = s_fifo[s_fifo_head].level;
            s_fifo_head = (s_fifo_head + 1) % PENDING_FIFO_LEN;
            s_fifo_count--;
        }
    }
    uint32_t idx = (s_fifo_head + s_fifo_count) % PENDING_FIFO_LEN;
    s_fifo[idx].due = due;
    s_fifo[idx].level = level;
    s_fifo_count++;
}

/* ---- mm -> codes -------------------------------------------------------- */

static inline uint16_t to_code(float f, float m, float c, float lo, float hi)
{
    float v = f * m + c;

    if (v < lo) v = lo;
    if (v > hi) v = hi;
    return (uint16_t)(v + 0.5f);
}

void galvo_out_tick(float x_mm, float y_mm, uint8_t power)
{
    struct galvo_chunk *c;
    uint8_t *p;
    bool des;

    if (!prod_enter())
        return;

    /* Desired gate, with pulse-density modulation if selected. */
    if (s_pd_mode) {
        uint32_t ph = s_pd_phase;
        uint32_t on = ((uint32_t)power * s_pd_n + 127u) / 255u;

        s_pd_phase = (ph + 1 >= s_pd_n) ? 0 : ph + 1;
        des = power != 0 && ph < on;
    } else {
        des = power != 0;
    }
    if (des != s_des_last) {
        s_des_last = des;
        fifo_schedule(des);
    }
    while (s_fifo_count && (int32_t)(s_fifo[s_fifo_head].due - s_tick_no) <= 0) {
        s_out_gate = s_fifo[s_fifo_head].level;
        s_fifo_head = (s_fifo_head + 1) % PENDING_FIFO_LEN;
        s_fifo_count--;
    }

    /* Power word barrier. */
    if (power != 0) {
        int want = s_pd_mode ? s_pd_word : (int)power;

        if (want != s_atmega_power) {
            s_atmega_power = want;
            request_barrier(GALVO_BARRIER_POWER, (uint8_t)want, 0.0f);
        }
    }

    /* Make room for a gate edge. */
    c = s_cur;
    if (c && c->n_ticks > 0 && s_out_gate != s_cur_gate && c->n_edges >= GALVO_CHUNK_MAX_EDGES) {
        submit_cur();
        c = NULL;
    }
    if (!c) {
        if (!open_chunk(s_out_gate)) {
            prod_leave();
            return;
        }
        c = s_cur;
    }
    if (s_out_gate != s_cur_gate) {
        if (c->n_ticks == 0)
            c->gate_start = s_out_gate;
        else
            c->edge_tick[c->n_edges++] = c->n_ticks;
        s_cur_gate = s_out_gate;
    }

    /* mm -> field -> codes. */
    {
        float x = (x_mm - s_org_x) * s_sgn_x;
        float y = (y_mm - s_org_y) * s_sgn_y;
        float t;
        uint16_t c0, c1;

        if (s_swap) { t = x; x = y; y = t; }
        if (s_k1 != 0.0f) {
            float f = 1.0f + s_k1 * (x * x + y * y);

            x *= f;
            y *= f;
        }
        c0 = to_code(x, s_m0, s_c0, s_lo0, s_hi0);
        c1 = to_code(y, s_m1, s_c1, s_lo1, s_hi1);

        p = c->wire + (size_t)c->n_ticks * GALVO_BYTES_PER_TICK;
        p[0] = (uint8_t)(c1 >> 8);
        p[1] = (uint8_t)c1;
        p[2] = 0;
        p[3] = (uint8_t)(c0 >> 8);
        p[4] = (uint8_t)c0;
        p[5] = 0;
    }
    c->end_x = x_mm;
    c->end_y = y_mm;
    c->n_ticks++;
    s_tick_no++;

    if (c->n_ticks >= s_chunk_limit)
        submit_cur();
    prod_leave();
}

void galvo_out_set_prr(float hz)
{
    if (!prod_enter())
        return;
    request_barrier(GALVO_BARRIER_PRR, 0, hz);
    prod_leave();
}

void galvo_out_flush(void)
{
    if (!prod_enter())
        return;
    if (!s_cur && s_pend_flags)
        open_chunk(s_out_gate); /* zero-tick chunk carrying the barrier */
    if (s_cur && (s_cur->n_ticks > 0 || s_cur->barrier))
        submit_cur();
    prod_leave();
}

/* ---- status --------------------------------------------------------------- */

float galvo_out_buffered_us(void)
{
    struct galvo_chunk *c = s_cur;
    uint32_t n = atomic_load(&g_galvo.ticks_queued);

    if (c)
        n += c->n_ticks;
    return (float)n * s_tick_us;
}

bool galvo_out_busy(void)
{
    struct galvo_chunk *c = s_cur;
    int outstanding;

    if (!atomic_load(&g_galvo.ready))
        return false;
    outstanding = GALVO_CHUNK_POOL - (int)uxQueueMessagesWaiting(g_galvo.free_q) - (c ? 1 : 0);
    if (outstanding > 0 || s_pend_flags)
        return true;
    return c && (c->n_ticks > 0 || c->barrier);
}

void galvo_out_position(float *x_mm, float *y_mm)
{
    portENTER_CRITICAL(&g_galvo.pos_lock);
    *x_mm = g_galvo.pos_x;
    *y_mm = g_galvo.pos_y;
    portEXIT_CRITICAL(&g_galvo.pos_lock);
}

void galvo_out_get_stats(struct galvo_out_stats *s)
{
    s->chunks = atomic_load(&g_galvo.chunks_done);
    s->underruns = atomic_load(&g_galvo.underruns);
    s->barriers = atomic_load(&g_galvo.barriers);
    s->ticks = atomic_load(&g_galvo.ticks_done);
}

/* ---- abort / hold --------------------------------------------------------- */

void galvo_out_abort(void)
{
    int waited_ms = 0;

    if (!atomic_load(&g_galvo.ready))
        return;

    /* Kill first, before waiting for the lock: the laser must go off
     * immediately even if another abort is still in progress. */
    atomic_store(&g_galvo.abort, true);
    laser_io_gate_kill(true);
    xSemaphoreTake(s_abort_mtx, portMAX_DELAY);
    atomic_store(&g_galvo.abort, true);
    laser_io_gate_kill(true);
    dac_task_notify(); /* stop queuing now */

    /* Let any producer call notice the flag and leave (it polls every 2 ms
     * while waiting for a free chunk). */
    while (atomic_load(&s_in_prod) && waited_ms < 100) {
        vTaskDelay(pdMS_TO_TICKS(1));
        waited_ms++;
    }

    xSemaphoreTake(g_galvo.abort_done, 0); /* clear a stale ack */
    atomic_store(&g_galvo.abort_go, true);
    dac_task_notify();
    if (xSemaphoreTake(g_galvo.abort_done, pdMS_TO_TICKS(300)) != pdTRUE) {
        ESP_LOGE(TAG, "abort: dac_task did not acknowledge within 300 ms");
        atomic_store(&g_galvo.abort_go, false);
    }

    /* Reset the producer. */
    if (s_cur) {
        struct galvo_chunk *c = s_cur;

        s_cur = NULL;
        xQueueSend(g_galvo.free_q, &c, 0);
    }
    s_pend_flags = 0;
    s_fifo_head = s_fifo_count = 0;
    s_out_gate = false;
    s_cur_gate = false;
    s_des_last = false;
    s_pd_phase = 0;
    s_atmega_power = -1; /* dropped barriers: resend on demand */
    if (uxQueueMessagesWaiting(g_galvo.free_q) != GALVO_CHUNK_POOL)
        ESP_LOGE(TAG, "abort: %u of %d chunks not returned",
                 (unsigned)(GALVO_CHUNK_POOL - uxQueueMessagesWaiting(g_galvo.free_q)),
                 GALVO_CHUNK_POOL);

    atomic_store(&g_galvo.held, false);
    atomic_store(&g_galvo.abort, false);
    laser_io_gate_kill(false);
    xSemaphoreGive(s_abort_mtx);
}

void galvo_out_hold(bool hold)
{
    atomic_store(&g_galvo.held, hold);
    if (hold)
        laser_io_gate_kill(true);
    else if (!atomic_load(&g_galvo.abort))
        laser_io_gate_kill(false);
    dac_task_notify();
}

bool galvo_out_is_held(void)
{
    return atomic_load(&g_galvo.held);
}
