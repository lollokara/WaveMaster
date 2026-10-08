/*
 * motion_task - owns the trajectory generator (motion.c), core 1.
 * See motion_task.h for the flow-control / underrun / abort contract.
 *
 * Abort design: motion_abort() bumps s_abort_req (an epoch), kills the laser
 * and the output stage itself (galvo_out_abort() - this also releases a
 * motion task blocked inside galvo_out_tick()), resets the queue and waits
 * until the motion task has caught up (s_abort_ack == req). While
 * req != ack the sink callback discards ticks, so a generator that is in the
 * middle of a long segment finishes its loop in microseconds and the task
 * notices the abort right after motion_gen_push()/finish() returns. Queue
 * items carry the epoch they were submitted in; items from an older epoch
 * (a submitter that raced the abort) are dropped by the task.
 */
#include "motion_task.h"

#include <stdatomic.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "calib.h"
#include "galvo_out.h"

static const char *TAG = "motion";

#define MOTION_TASK_PRIO     8
#define MOTION_TASK_STACK    4096
#define MOTION_TASK_CORE     1
#define IDLE_POLL_MS         50
#define ABORT_WAIT_MS        300
#define SUBMIT_SLICE_MS      20
#define SEG_MARKER           0xFF   /* wake-up item pushed by motion_abort() */

struct qitem {
    uint32_t epoch;
    struct motion_seg seg;
};

static QueueHandle_t s_queue;
static SemaphoreHandle_t s_ack_sem;
static TaskHandle_t s_task;
static struct motion_gen s_gen;

static atomic_uint s_abort_req;
static atomic_uint s_abort_ack;
static volatile float s_abort_x, s_abort_y;
static volatile bool s_processing;
static volatile bool s_reload_req;
static unsigned s_calib_seen;
static bool s_need_flush;

static inline bool abort_pending(void)
{
    return atomic_load(&s_abort_req) != atomic_load(&s_abort_ack);
}

static void load_params(struct motion_params *p)
{
    p->tick_us              = galvo_out_tick_us();
    p->jump_speed_mm_s      = calib_get(CAL_JUMP_SPEED);
    p->jump_accel_mm_s2     = calib_get(CAL_JUMP_ACCEL);
    p->jump_delay_us        = calib_get(CAL_JUMP_DELAY);
    p->jump_delay_per_mm_us = calib_get(CAL_JUMP_DELAY_PER_MM);
    p->mark_delay_us        = calib_get(CAL_MARK_DELAY);
    p->laser_off_delay_us   = calib_get(CAL_LASER_OFF_DELAY);
    p->polygon_delay_us     = calib_get(CAL_POLYGON_DELAY);
    p->polygon_angle_deg    = calib_get(CAL_POLYGON_ANGLE);
    p->wobble_diameter_mm   = calib_get(CAL_WOBBLE_D);
    p->wobble_pitch_mm      = calib_get(CAL_WOBBLE_PITCH);
}

static void sink_tick(void *ctx, float x, float y, uint8_t power)
{
    (void)ctx;
    if (abort_pending())
        return;                       /* drop: the abort is tearing everything down */
    galvo_out_tick(x, y, power);
}

/* Only while the generator and the output stage are idle. */
static void maybe_reload(void)
{
    unsigned gen = calib_generation();
    if (gen == s_calib_seen && !s_reload_req)
        return;
    if (motion_gen_has_pending(&s_gen) || galvo_out_busy())
        return;
    s_reload_req = false;
    s_calib_seen = gen;
    galvo_out_reload();               /* may change the tick period */
    struct motion_params p;
    load_params(&p);
    motion_gen_set_params(&s_gen, &p);
    ESP_LOGI(TAG, "params reloaded, tick %.2f us", (double)p.tick_us);
}

static void handle_abort(void)
{
    unsigned req = atomic_load(&s_abort_req);
    if (req == atomic_load(&s_abort_ack))
        return;
    motion_gen_reset(&s_gen, 0, 0);
    xQueueReset(s_queue);
    galvo_out_abort();                /* blocks until the output stage is stopped */
    float x, y;
    galvo_out_position(&x, &y);
    motion_gen_reset(&s_gen, x, y);
    s_abort_x = x;
    s_abort_y = y;
    s_need_flush = false;
    atomic_store(&s_abort_ack, req);
    xSemaphoreGive(s_ack_sem);
}

static void process(const struct qitem *it)
{
    if (it->seg.type == SEG_MARKER || it->epoch != atomic_load(&s_abort_req))
        return;                       /* wake-up marker or stale submission */
    motion_gen_push(&s_gen, &it->seg);
    s_need_flush = true;
}

/* Idle start-up prebuffer. Called with the first segment peeked. */
static void prebuffer(void)
{
    UBaseType_t last = uxQueueMessagesWaiting(s_queue);
    TickType_t last_change = xTaskGetTickCount();
    for (;;) {
        if (abort_pending() || last >= MOTION_PREBUFFER_SEGS)
            return;
        vTaskDelay(1);
        UBaseType_t n = uxQueueMessagesWaiting(s_queue);
        TickType_t now = xTaskGetTickCount();
        if (n != last) {
            last = n;
            last_change = now;
        } else if ((now - last_change) >= pdMS_TO_TICKS(MOTION_PREBUFFER_IDLE_MS)) {
            return;
        }
    }
}

static void motion_task(void *arg)
{
    (void)arg;
    struct qitem it;

    for (;;) {
        handle_abort();
        maybe_reload();

        if (!motion_gen_has_pending(&s_gen)) {
            /* Idle: nothing open. */
            if (uxQueueMessagesWaiting(s_queue) == 0 && s_need_flush) {
                galvo_out_flush();
                s_need_flush = false;
            }
            if (xQueuePeek(s_queue, &it, pdMS_TO_TICKS(IDLE_POLL_MS)) != pdTRUE)
                continue;
            s_processing = true;      /* queue is still non-empty: no busy() gap */
            maybe_reload();
            if (!galvo_out_busy())
                prebuffer();
        } else {
            /* A polyline / jump chain is open. */
            if (xQueueReceive(s_queue, &it, 0) == pdTRUE) {
                s_processing = true;
                process(&it);
                s_processing = false;
                continue;
            }
            /* The host is behind. Whatever is sitting in the partially
             * filled chunk has to start streaming now: buffered_us counts
             * it, but it never drains until it is handed over, so without
             * this a lone G1 would wait here forever. */
            if (s_need_flush) {
                galvo_out_flush();
                s_need_flush = false;
            }
            float allowed_us = galvo_out_buffered_us() - MOTION_UNDERRUN_MARGIN_US;
            int wait_ms = (int)(allowed_us * 0.001f);
            if (wait_ms < 1) {
                /* Ring about to run dry: close the polyline cleanly. */
                s_processing = true;
                motion_gen_finish(&s_gen);
                s_need_flush = true;
                s_processing = false;
                continue;
            }
            if (wait_ms > IDLE_POLL_MS)
                wait_ms = IDLE_POLL_MS;
            if (xQueuePeek(s_queue, &it, pdMS_TO_TICKS(wait_ms)) != pdTRUE)
                continue;             /* re-evaluate the margin */
            s_processing = true;
        }

        /* An item was peeked (s_processing set): drain what is queued. */
        while (!abort_pending() && xQueueReceive(s_queue, &it, 0) == pdTRUE) {
            process(&it);
            if (uxQueueMessagesWaiting(s_queue) == 0)
                break;
        }
        s_processing = false;
    }
}

bool motion_task_start(void)
{
    s_queue = xQueueCreate(MOTION_QUEUE_LEN, sizeof(struct qitem));
    s_ack_sem = xSemaphoreCreateBinary();
    if (!s_queue || !s_ack_sem)
        return false;

    struct motion_params p;
    load_params(&p);
    s_calib_seen = calib_generation();
    struct motion_sink sink = { sink_tick, NULL };
    motion_gen_init(&s_gen, &p, &sink, 0.0f, 0.0f);

    if (xTaskCreatePinnedToCore(motion_task, "motion", MOTION_TASK_STACK, NULL,
                                MOTION_TASK_PRIO, &s_task, MOTION_TASK_CORE) != pdPASS)
        return false;
    ESP_LOGI(TAG, "started, tick %.2f us, queue %d", (double)p.tick_us, MOTION_QUEUE_LEN);
    return true;
}

bool motion_submit(const struct motion_seg *seg, uint32_t wait_ms)
{
    struct qitem it;
    it.seg = *seg;
    TickType_t start = xTaskGetTickCount();

    for (;;) {
        if (abort_pending())
            return false;
        it.epoch = atomic_load(&s_abort_req);

        uint32_t slice = SUBMIT_SLICE_MS;
        if (wait_ms != UINT32_MAX) {
            uint32_t elapsed = (uint32_t)((xTaskGetTickCount() - start) * portTICK_PERIOD_MS);
            if (elapsed >= wait_ms)
                slice = 0;
            else if (wait_ms - elapsed < slice)
                slice = wait_ms - elapsed;
        }
        if (xQueueSend(s_queue, &it, pdMS_TO_TICKS(slice)) == pdTRUE)
            return !abort_pending();
        if (wait_ms != UINT32_MAX && slice == 0)
            return false;
        if (wait_ms != UINT32_MAX &&
            (uint32_t)((xTaskGetTickCount() - start) * portTICK_PERIOD_MS) >= wait_ms)
            return false;
    }
}

void motion_abort(float *x, float *y)
{
    unsigned req = atomic_fetch_add(&s_abort_req, 1) + 1;

    galvo_out_abort();                /* laser off now; releases a blocked galvo_out_tick() */
    xQueueReset(s_queue);
    struct qitem wake = { .epoch = req };
    wake.seg.type = SEG_MARKER;
    xQueueSendToFront(s_queue, &wake, 0);   /* wake an idle peek (best effort) */

    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(ABORT_WAIT_MS);
    bool acked = false;
    for (;;) {
        if ((int)(atomic_load(&s_abort_ack) - req) >= 0) {
            acked = true;
            break;
        }
        TickType_t now = xTaskGetTickCount();
        if ((int32_t)(deadline - now) <= 0)
            break;
        xSemaphoreTake(s_ack_sem, deadline - now);
    }

    float px, py;
    if (acked) {
        px = s_abort_x;
        py = s_abort_y;
    } else {
        /* The task will still finish the abort on its own; submits stay
         * refused until it does. */
        ESP_LOGW(TAG, "abort ack timeout");
        galvo_out_position(&px, &py);
    }
    if (x) *x = px;
    if (y) *y = py;
}

bool motion_busy(void)
{
    return s_processing || uxQueueMessagesWaiting(s_queue) > 0 ||
           motion_gen_has_pending(&s_gen) || galvo_out_busy();
}

size_t motion_queue_free(void)
{
    return s_queue ? (size_t)uxQueueSpacesAvailable(s_queue) : 0;
}

void motion_reload_params(void)
{
    s_reload_req = true;
}
