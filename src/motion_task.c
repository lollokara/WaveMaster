/* STUB - replaced by the motion agent. */
#include "motion_task.h"
bool motion_task_start(void) { return true; }
bool motion_submit(const struct motion_seg *seg, uint32_t wait_ms) { (void)seg; (void)wait_ms; return true; }
void motion_abort(float *x, float *y) { *x = 0; *y = 0; }
bool motion_busy(void) { return false; }
size_t motion_queue_free(void) { return MOTION_QUEUE_LEN; }
void motion_reload_params(void) {}
