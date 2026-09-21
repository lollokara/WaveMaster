#ifndef PRR_PWM_H_
#define PRR_PWM_H_

/* Pulse-repetition-rate output: a square wave on a dedicated GPIO whose
 * frequency the laser driver reads as the pulse rate. Generated with the
 * ESP32 LEDC peripheral. Pin choice is a placeholder - adjust
 * BOARD_PIN_PRR in prr_pwm.c to match the actual wiring. */
void prr_pwm_init(void);

/* Set the output frequency in Hz. 0 stops the signal (pin driven low). */
void prr_pwm_set_hz(float hz);

#endif /* PRR_PWM_H_ */
