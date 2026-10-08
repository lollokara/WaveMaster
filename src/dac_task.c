/* STUB - replaced by the stream agent. */
#include "dac_task.h"
#include "ad3552r_board.h"
bool dac_task_start(void) { return ad3552r_board_init() != NULL; }
bool dac_task_readback(uint16_t *code_x, uint16_t *code_y) { *code_x = 0; *code_y = 0; return false; }
