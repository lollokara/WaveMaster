/* STUB - replaced by the host/GRBL agent. */
#include "calib.h"
#include "laser_io.h"
#include "dac_task.h"
#include "galvo_out.h"
#include "motion_task.h"
#include "atmega_link.h"
#include "grbl.h"
void app_main(void)
{
    laser_io_early_init();
    calib_init();
    atmega_link_init();
    if (!dac_task_start())
        return;
    galvo_out_init();
    motion_task_start();
    grbl_task_start();
}
