#include "zb_driver.h"

#include "driver/gpio.h"
#include "esp_rom_sys.h"

#include "zb_motion.h"
#include "zb_pins.h"
#include "zb_settings.h"

/* A4988 microstep table: 1/1 .. 1/16 = MS1..3 000 / 100 / 010 / 110 / 111. */
static const struct { int ms; int m1, m2, m3; } k_ms[] = {
    { 1, 0, 0, 0 }, { 2, 1, 0, 0 }, { 4, 0, 1, 0 }, { 8, 1, 1, 0 }, { 16, 1, 1, 1 },
};

static bool s_inited;
static bool s_enabled;
static bool s_awake;
static int  s_ms = 16;

static void out_pin(int pin, int level)
{
    gpio_set_level(pin, level);          /* preload the level, then switch to output: no glitch */
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
}

void zb_driver_init(void)
{
    if (s_inited) return;
    out_pin(ZB_PIN_ENABLE, 1);           /* motor de-energised before anything else */
    s_inited = true;
    s_enabled = false;
    out_pin(ZB_PIN_RESET, 1);
    /* sleep_idle: stay asleep until the first move; otherwise keep the driver awake */
    bool awake = zb_set_get(ZS_SLEEP_IDLE) == 0.0f;
    out_pin(ZB_PIN_SLEEP, awake ? 1 : 0);
    s_awake = awake;
    out_pin(ZB_PIN_MS1, 0);
    out_pin(ZB_PIN_MS2, 0);
    out_pin(ZB_PIN_MS3, 0);
    out_pin(ZB_PIN_DIR, 0);
    zb_driver_set_microstep((int)zb_set_get(ZS_MICROSTEP));
}

void zb_driver_set_dir(bool positive)
{
    bool inv = zb_set_get(ZS_DIR_INVERT) != 0.0f;
    gpio_set_level(ZB_PIN_DIR, (positive != inv) ? 1 : 0);
}

void zb_driver_enable(bool on)
{
    zb_driver_init();
    if (on) {
        if (!s_awake) {
            gpio_set_level(ZB_PIN_SLEEP, 1);
            s_awake = true;
            esp_rom_delay_us(1500);      /* charge pump: >= 1 ms before the first STEP */
        }
        gpio_set_level(ZB_PIN_ENABLE, 0);
        s_enabled = true;
    } else {
        gpio_set_level(ZB_PIN_ENABLE, 1);
        s_enabled = false;
        if (zb_set_get(ZS_SLEEP_IDLE) != 0.0f) {
            gpio_set_level(ZB_PIN_SLEEP, 0);
            s_awake = false;
        }
    }
}

bool zb_driver_is_enabled(void) { return s_enabled; }
int  zb_driver_microstep(void)  { return s_ms; }

void zb_driver_set_microstep(int ms)
{
    for (unsigned i = 0; i < sizeof k_ms / sizeof k_ms[0]; i++) {
        if (k_ms[i].ms != ms) continue;
        gpio_set_level(ZB_PIN_MS1, k_ms[i].m1);
        gpio_set_level(ZB_PIN_MS2, k_ms[i].m2);
        gpio_set_level(ZB_PIN_MS3, k_ms[i].m3);
        s_ms = ms;
        return;
    }
}
