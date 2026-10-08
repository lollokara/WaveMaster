/* STUB - replaced by the stream agent. */
#include "galvo_out.h"
#include <string.h>
void galvo_out_init(void) {}
float galvo_out_tick_us(void) { return 10.0f; }
void galvo_out_reload(void) {}
void galvo_out_tick(float x_mm, float y_mm, uint8_t power) { (void)x_mm; (void)y_mm; (void)power; }
void galvo_out_set_prr(float hz) { (void)hz; }
void galvo_out_flush(void) {}
float galvo_out_buffered_us(void) { return 0.0f; }
bool galvo_out_busy(void) { return false; }
void galvo_out_abort(void) {}
void galvo_out_hold(bool hold) { (void)hold; }
bool galvo_out_is_held(void) { return false; }
void galvo_out_position(float *x_mm, float *y_mm) { *x_mm = 0; *y_mm = 0; }
void galvo_out_get_stats(struct galvo_out_stats *s) { memset(s, 0, sizeof(*s)); }
