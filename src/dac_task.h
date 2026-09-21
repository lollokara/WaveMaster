#ifndef DAC_TASK_H_
#define DAC_TASK_H_

#include <stdbool.h>

/* Starts the DAC/SPI task pinned to core 0. Must be called after
 * laser_ctrl_init(). Returns true if the AD3552R initialized and the task
 * was created successfully. */
bool dac_task_start(void);

#endif /* DAC_TASK_H_ */
