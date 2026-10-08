#include <Arduino.h>

// Pin definitions per laser control pinout
// Power bits D0-D7 (8-bit parallel word to laser)
const uint8_t PIN_D0 = A7;
const uint8_t PIN_D1 = A6;
const uint8_t PIN_D2 = A4;
const uint8_t PIN_D3 = A2;
const uint8_t PIN_D4 = A1;
const uint8_t PIN_D5 = 13;
const uint8_t PIN_D6 = 3;
const uint8_t PIN_D7 = 5;

// Control/status lines
const uint8_t PIN_EMISSION_ENABLE = A0;  // Arm
const uint8_t PIN_EMISSION_MOD = 2;      // PWM
const uint8_t PIN_SYNC = 4;              // PWM
const uint8_t PIN_LATCH = 6;
const uint8_t PIN_GUIDE_LASER = 7;
const uint8_t PIN_AUX_OFF = 8;
const uint8_t PIN_5V_DETECT = A5;
const uint8_t PIN_STAT_16 = A3;
const uint8_t PIN_STAT_11 = 9;
const uint8_t PIN_STAT_12 = 10;
const uint8_t PIN_RED_LED = 11;
const uint8_t PIN_GREEN_LED = 12;

// PWM state for software PWM on D2 (EMISSION_MOD) and D4 (SYNC)
volatile uint16_t pwm_emit_freq = 1000;  // Hz
volatile uint8_t pwm_emit_duty = 128;    // 0-255 (0-100%)
volatile uint16_t pwm_sync_freq = 1000;
volatile uint8_t pwm_sync_duty = 128;

// Software PWM counters (use 16-bit timer for timing)
volatile uint16_t emit_counter = 0;
volatile uint16_t sync_counter = 0;
volatile uint8_t emit_pwm_state = LOW;
volatile uint8_t sync_pwm_state = LOW;

// Command buffer
#define BUFFER_SIZE 64
uint8_t cmd_buffer[BUFFER_SIZE];
uint8_t cmd_len = 0;

// Status tracking
struct {
    uint8_t armed = 0;
    uint8_t firing = 0;
    uint8_t guide_on = 0;
    uint8_t aux_off = 0;
    uint8_t power_value = 0;
    uint16_t vcc_mv = 0;
    uint8_t stat_11 = 0;
    uint8_t stat_12 = 0;
    uint8_t stat_16 = 0;
} laser_status;

void setup_pins() {
    // Power bits and controls
    pinMode(PIN_D0, OUTPUT);
    pinMode(PIN_D1, OUTPUT);
    pinMode(PIN_D2, OUTPUT);
    pinMode(PIN_D3, OUTPUT);
    pinMode(PIN_D4, OUTPUT);
    pinMode(PIN_D5, OUTPUT);
    pinMode(PIN_D6, OUTPUT);
    pinMode(PIN_D7, OUTPUT);
    pinMode(PIN_LATCH, OUTPUT);
    pinMode(PIN_EMISSION_ENABLE, OUTPUT);
    pinMode(PIN_EMISSION_MOD, OUTPUT);
    pinMode(PIN_SYNC, OUTPUT);
    pinMode(PIN_GUIDE_LASER, OUTPUT);
    pinMode(PIN_AUX_OFF, OUTPUT);

    // LEDs
    pinMode(PIN_RED_LED, OUTPUT);
    pinMode(PIN_GREEN_LED, OUTPUT);

    // Status inputs
    pinMode(PIN_5V_DETECT, INPUT);
    pinMode(PIN_STAT_11, INPUT);
    pinMode(PIN_STAT_12, INPUT);
    pinMode(PIN_STAT_16, INPUT);

    // All outputs low initially (safe state), except AUX_OFF stays HIGH
    digitalWrite(PIN_D0, LOW);
    digitalWrite(PIN_D1, LOW);
    digitalWrite(PIN_D2, LOW);
    digitalWrite(PIN_D3, LOW);
    digitalWrite(PIN_D4, LOW);
    digitalWrite(PIN_D5, LOW);
    digitalWrite(PIN_D6, LOW);
    digitalWrite(PIN_D7, LOW);
    digitalWrite(PIN_LATCH, LOW);
    digitalWrite(PIN_EMISSION_ENABLE, LOW);
    digitalWrite(PIN_EMISSION_MOD, LOW);
    digitalWrite(PIN_SYNC, LOW);
    digitalWrite(PIN_GUIDE_LASER, LOW);
    digitalWrite(PIN_AUX_OFF, HIGH);  // AUX_OFF always HIGH
    digitalWrite(PIN_RED_LED, LOW);
    digitalWrite(PIN_GREEN_LED, LOW);
}

void setup_timers() {
    // Setup Timer 1 to interrupt at ~8kHz for software PWM
    // ATmega328P Timer 1 at 8MHz/8 = 1MHz counter freq
    TCCR1A = 0;
    TCCR1B = 0b010;  // CTC mode, prescaler 8
    TCCR1C = 0;
    OCR1A = 124;     // ~8kHz interrupt (1MHz / 125 = 8kHz)
    TIMSK1 = 0b010;  // Enable OCIE1A
}

// Timer 1 interrupt for software PWM (fires ~8kHz)
ISR(TIMER1_COMPA_vect) {
    // Emission modulation PWM on D2 - ONLY if armed
    if (pwm_emit_freq > 0 && laser_status.armed) {
        emit_counter++;
        uint16_t period = 8000 / pwm_emit_freq;  // counts per period
        uint16_t on_time = (period * pwm_emit_duty) / 256;

        if (emit_counter >= period) {
            emit_counter = 0;
        }
        emit_pwm_state = (emit_counter < on_time) ? HIGH : LOW;
        digitalWrite(PIN_EMISSION_MOD, emit_pwm_state);
    } else {
        // Not armed - keep D2 LOW
        digitalWrite(PIN_EMISSION_MOD, LOW);
    }

    // Sync PWM on D4 - ONLY if armed
    if (pwm_sync_freq > 0 && laser_status.armed) {
        sync_counter++;
        uint16_t period = 8000 / pwm_sync_freq;
        uint16_t on_time = (period * pwm_sync_duty) / 256;

        if (sync_counter >= period) {
            sync_counter = 0;
        }
        sync_pwm_state = (sync_counter < on_time) ? HIGH : LOW;
        digitalWrite(PIN_SYNC, sync_pwm_state);
    } else {
        // Not armed - keep D4 LOW
        digitalWrite(PIN_SYNC, LOW);
    }
}

void set_armed(uint8_t on) {
    laser_status.armed = on ? 1 : 0;

    if (on) {
        // ARM sequence: set EMISSION_ENABLE high and EMISSION_MOD low for 2 seconds
        digitalWrite(PIN_EMISSION_ENABLE, HIGH);

        // Force EMISSION_MOD low by setting PWM duty to 0 (overrides PWM interrupt)
        pwm_emit_duty = 0;  // PWM interrupt will hold D2 LOW when duty=0
        delay(2000);  // Hold low for 2 seconds
        // EMISSION_MOD can now be controlled by PWM via set_pwm_emit()
    } else {
        // DISARM: pull everything down
        digitalWrite(PIN_EMISSION_ENABLE, LOW);
        pwm_emit_duty = 0;  // Force PWM low when disarmed
        pwm_sync_duty = 0;  // Also kill sync PWM
    }
}

void set_firing(uint8_t on) {
    laser_status.firing = on ? 1 : 0;
    // Note: D2 is PWM, but can also be used as direct fire if needed
    // Currently managed by PWM state
}

void set_guide(uint8_t on) {
    laser_status.guide_on = on ? 1 : 0;
    digitalWrite(PIN_GUIDE_LASER, on ? HIGH : LOW);
}

void set_power(uint8_t value) {
    // Set all 8 power bits (D0-D7) from the value byte
    laser_status.power_value = value;

    // Set each bit of the 8-bit power word
    digitalWrite(PIN_D0, (value >> 0) & 1 ? HIGH : LOW);
    digitalWrite(PIN_D1, (value >> 1) & 1 ? HIGH : LOW);
    digitalWrite(PIN_D2, (value >> 2) & 1 ? HIGH : LOW);
    digitalWrite(PIN_D3, (value >> 3) & 1 ? HIGH : LOW);
    digitalWrite(PIN_D4, (value >> 4) & 1 ? HIGH : LOW);
    digitalWrite(PIN_D5, (value >> 5) & 1 ? HIGH : LOW);
    digitalWrite(PIN_D6, (value >> 6) & 1 ? HIGH : LOW);
    digitalWrite(PIN_D7, (value >> 7) & 1 ? HIGH : LOW);

    // Pulse latch to commit power bits
    digitalWrite(PIN_LATCH, HIGH);
    delayMicroseconds(100);
    digitalWrite(PIN_LATCH, LOW);
}

void set_pwm_emit(uint16_t freq, uint8_t duty) {
    if (freq > 0 && freq <= 8000) {
        pwm_emit_freq = freq;
    }
    pwm_emit_duty = duty;
}

void set_pwm_sync(uint16_t freq, uint8_t duty) {
    if (freq > 0 && freq <= 8000) {
        pwm_sync_freq = freq;
    }
    pwm_sync_duty = duty;
}

void pulse_latch() {
    digitalWrite(PIN_LATCH, HIGH);
    delayMicroseconds(100);
    digitalWrite(PIN_LATCH, LOW);
}

void read_status() {
    // Read digital status pins
    laser_status.stat_11 = digitalRead(PIN_STAT_11);
    laser_status.stat_12 = digitalRead(PIN_STAT_12);
    laser_status.stat_16 = digitalRead(PIN_STAT_16);

    // Check 5V presence
    uint16_t v5_raw = analogRead(PIN_5V_DETECT);
    // Assuming 5V input gets divided or direct measurement
    laser_status.vcc_mv = (v5_raw * 5000) / 1024;

    // Set LED indicators
    // Red LED = 5V present (high analog reading)
    // Green LED = 0V present (low analog reading)
    if (v5_raw > 700) {  // ~3.4V threshold for "5V present"
        digitalWrite(PIN_RED_LED, HIGH);
        digitalWrite(PIN_GREEN_LED, LOW);
    } else if (v5_raw < 300) {  // ~1.5V threshold for "0V/low present"
        digitalWrite(PIN_RED_LED, LOW);
        digitalWrite(PIN_GREEN_LED, HIGH);
    } else {
        // Ambiguous state
        digitalWrite(PIN_RED_LED, LOW);
        digitalWrite(PIN_GREEN_LED, LOW);
    }
}

void send_status() {
    // Send 8-byte status packet: armed, firing, guide, power, stat11, stat12, stat16, vcc_h, vcc_l
    uint8_t response[8] = {
        laser_status.armed,
        laser_status.firing,
        laser_status.guide_on,
        laser_status.power_value,
        laser_status.stat_11,
        laser_status.stat_12,
        laser_status.stat_16,
        (laser_status.vcc_mv >> 8) & 0xFF
    };
    Serial.write(response, 8);
}

void process_command() {
    if (cmd_len == 0) return;

    uint8_t cmd = cmd_buffer[0];

    switch (cmd) {
        case 0x01:  // SET_ARM
            if (cmd_len >= 2) {
                set_armed(cmd_buffer[1]);
                Serial.write(0x01);  // ACK
            }
            break;

        case 0x02:  // SET_FIRING
            if (cmd_len >= 2) {
                set_firing(cmd_buffer[1]);
                Serial.write(0x02);
            }
            break;

        case 0x03:  // SET_GUIDE
            if (cmd_len >= 2) {
                set_guide(cmd_buffer[1]);
                Serial.write(0x03);
            }
            break;

        case 0x04:  // SET_POWER
            if (cmd_len >= 2) {
                set_power(cmd_buffer[1]);
                Serial.write(0x04);
            }
            break;

        case 0x05:  // SET_PWM_EMIT (freq_h, freq_l, duty)
            if (cmd_len >= 4) {
                uint16_t freq = (cmd_buffer[1] << 8) | cmd_buffer[2];
                uint8_t duty = cmd_buffer[3];
                set_pwm_emit(freq, duty);
                Serial.write(0x05);
            }
            break;

        case 0x06:  // SET_PWM_SYNC (freq_h, freq_l, duty)
            if (cmd_len >= 4) {
                uint16_t freq = (cmd_buffer[1] << 8) | cmd_buffer[2];
                uint8_t duty = cmd_buffer[3];
                set_pwm_sync(freq, duty);
                Serial.write(0x06);
            }
            break;

        case 0x07:  // GET_STATUS
            read_status();
            send_status();
            break;

        case 0x08:  // PULSE_LATCH
            pulse_latch();
            Serial.write(0x08);
            break;

        default:
            Serial.write(0xFF);  // NACK
    }

    cmd_len = 0;
}

void setup() {
    Serial.begin(250000);
    setup_pins();
    setup_timers();
    sei();  // Enable interrupts
}

void loop() {
    // Read serial data
    while (Serial.available() > 0 && cmd_len < BUFFER_SIZE) {
        cmd_buffer[cmd_len++] = Serial.read();
    }

    // Process complete commands
    if (cmd_len > 0) {
        // Check if we have enough bytes for the command
        uint8_t expected_len = 1;
        if (cmd_buffer[0] == 0x01 || cmd_buffer[0] == 0x02 ||
            cmd_buffer[0] == 0x03 || cmd_buffer[0] == 0x04) {
            expected_len = 2;
        } else if (cmd_buffer[0] == 0x05 || cmd_buffer[0] == 0x06) {
            expected_len = 4;
        } else if (cmd_buffer[0] == 0x07 || cmd_buffer[0] == 0x08) {
            expected_len = 1;
        }

        if (cmd_len >= expected_len) {
            process_command();
        }
    }

    // Periodic status update
    static unsigned long last_status = 0;
    if (millis() - last_status > 500) {  // Every 500ms
        last_status = millis();
        read_status();
    }
}
