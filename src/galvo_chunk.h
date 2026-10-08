#ifndef GALVO_CHUNK_H_
#define GALVO_CHUNK_H_

/*
 * Private interface between galvo_out.c (producer, core 1) and dac_task.c
 * (consumer, core 0). Nothing else should include this.
 *
 * A chunk is a run of up to GALVO_CHUNK_TICKS DAC points in AD3552R wire
 * format plus the laser-gate waveform for exactly those points. Lifecycle:
 *
 *   free_q --(producer)--> filling --(submit)--> ready_q --(dac_task)-->
 *   queued to SPI (in flight) --(completion reaped)--> free_q
 *
 * Gate model: the gate level at tick 0 is gate_start; at each tick listed
 * in edge_tick[] (strictly increasing, all > 0, all < n_ticks) it toggles.
 * gate_end is the level after the last tick. Barrier actions are applied by
 * dac_task before the chunk's data is clocked out, with the stream drained
 * and the gate off. A chunk with n_ticks == 0 may carry only a barrier.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/spi_master.h"

#define GALVO_CHUNK_POOL       8
#define GALVO_CHUNK_TICKS      2048
#define GALVO_CHUNK_MAX_EDGES  512
#define GALVO_BYTES_PER_TICK   6

#define GALVO_BARRIER_POWER    0x01
#define GALVO_BARRIER_PRR      0x02

struct galvo_chunk {
    spi_transaction_t trans;       /* embedded; trans.user == this chunk */
    uint8_t *wire;                 /* DMA-capable, GALVO_CHUNK_TICKS * 6 bytes */
    uint16_t n_ticks;
    uint16_t n_edges;
    bool gate_start;
    bool gate_end;
    uint8_t barrier;               /* GALVO_BARRIER_* */
    uint8_t barrier_power;
    float barrier_prr_hz;
    float end_x, end_y;            /* machine mm after the last tick */
    uint16_t edge_tick[GALVO_CHUNK_MAX_EDGES];
};

/* State shared between the two sides. Defined in galvo_out.c. */
struct galvo_shared {
    QueueHandle_t free_q;
    QueueHandle_t ready_q;
    SemaphoreHandle_t abort_done;
    atomic_bool ready;             /* queues + pool created */
    atomic_bool abort;             /* abort requested: producer drops ticks, dac_task stops queuing */
    atomic_bool abort_go;          /* producer is quiet: dac_task may drain and acknowledge */

    atomic_bool held;
    atomic_uint ticks_queued;      /* submitted, not yet completed */
    atomic_uint chunks_done;
    atomic_uint underruns;
    atomic_uint barriers;
    atomic_ullong ticks_done;
    portMUX_TYPE pos_lock;
    float pos_x, pos_y;
};
extern struct galvo_shared g_galvo;

/* dac_task.c: wake the task (new chunk, hold released, abort requested). */
void dac_task_notify(void);

#endif /* GALVO_CHUNK_H_ */
