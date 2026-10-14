#ifndef ZB_PINS_H_
#define ZB_PINS_H_

/* ESP32-C3 <-> A4988 wiring (docs/ZBRO_PLAN.md section 1). */
#define ZB_PIN_ENABLE   0   /* active low; external 10k pull-up to 3.3 V */
#define ZB_PIN_ENDSTOP  5   /* internal pull-up, switch to GND */
#define ZB_PIN_MS1      6
#define ZB_PIN_MS2      7
#define ZB_PIN_MS3      8   /* strapping pin, see plan */
#define ZB_PIN_SLEEP    10  /* active low */
#define ZB_PIN_RESET    9   /* active low, strapping pin (BOOT) */
#define ZB_PIN_STEP     20
#define ZB_PIN_DIR      21

#endif
