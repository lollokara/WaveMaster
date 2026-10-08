/* STUB - replaced by the motion agent. */
#include "motion.h"
#include <string.h>
void motion_gen_init(struct motion_gen *g, const struct motion_params *p,
                     const struct motion_sink *sink, float x0, float y0)
{ memset(g, 0, sizeof(*g)); g->p = *p; g->sink = *sink; g->x = x0; g->y = y0; }
void motion_gen_set_params(struct motion_gen *g, const struct motion_params *p) { g->p = *p; }
void motion_gen_push(struct motion_gen *g, const struct motion_seg *seg) { (void)g; (void)seg; }
void motion_gen_finish(struct motion_gen *g) { (void)g; }
bool motion_gen_has_pending(const struct motion_gen *g) { return g->have_pending; }
void motion_gen_reset(struct motion_gen *g, float x, float y) { g->x = x; g->y = y; g->have_pending = false; }
