// WaveMaster ATmega328P companion firmware - protocol v2.
//
// Owns the SLOW lines of the IPG DB25 interface: power word D0..D7 + LATCH,
// EMISSION ENABLE (arm), guide laser, AUX OFF, status inputs and LEDs.
// The FAST lines (EMISSION MODULATION and SYNC) are driven directly by the
// ESP32 and are NOT connected to this board any more.
//
// Wire protocol: see ArduinoCompanion/README.md.

#include <Arduino.h>
#include <avr/wdt.h>
#include <avr/io.h>

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------
#define PROTOCOL_VERSION     2
#define SERIAL_BAUD          250000UL
#define ARM_SETTLE_MS        2000UL  // EMISSION ENABLE high -> "ready"
#define HEARTBEAT_TIMEOUT_MS 1000UL  // armed + no valid frame -> disarm
#define FRAME_TIMEOUT_MS     5UL     // partial frame is dropped after this
#define STATUS_PERIOD_MS     100UL   // analog 5V-detect / LED refresh

// LATCH timing. Assumption: the IPG interface captures the power word on the
// LATCH strobe and needs the data stable a little before the strobe edge.
// We allow LATCH_SETUP_US of settling after the last data write, then hold
// the strobe LATCH_PULSE_US. Increase if the laser misses updates.
#define LATCH_SETUP_US       2
#define LATCH_PULSE_US       3

// ---------------------------------------------------------------------------
// Pin map (Arduino pin numbers). Edit here to rewire.
// Power word bit n -> PIN_PWR_BITn. A6/A7 are analog-input-only on the
// ATmega328P and can NOT be used as outputs, hence bit0/bit1 on D2/D4.
// ---------------------------------------------------------------------------
#define PIN_PWR_BIT0         2       // was A7 (analog-only, never worked)
#define PIN_PWR_BIT1         4       // was A6 (analog-only, never worked)
#define PIN_PWR_BIT2         A4
#define PIN_PWR_BIT3         A2
#define PIN_PWR_BIT4         A1
#define PIN_PWR_BIT5         13
#define PIN_PWR_BIT6         3
#define PIN_PWR_BIT7         5

#define PIN_EMISSION_ENABLE  A0      // arm / MO
#define PIN_LATCH            6
#define PIN_GUIDE_LASER      7
#define PIN_AUX_OFF          8       // held HIGH
#define PIN_5V_DETECT        A5
#define PIN_STAT_16          A3
#define PIN_STAT_11          9
#define PIN_STAT_12          10
#define PIN_RED_LED          11
#define PIN_GREEN_LED        12

// ---------------------------------------------------------------------------
// Protocol
// ---------------------------------------------------------------------------
#define FRAME_SOF            0xA5
#define MAX_PAYLOAD          8
#define CMD_SET_ARM          0x01
#define CMD_SET_GUIDE        0x02
#define CMD_SET_POWER        0x03
#define CMD_GET_STATUS       0x04    // also serves as the heartbeat
#define CMD_NAK              0xFF
#define CMD_ACK_FLAG         0x80

#define ERR_NONE             0
#define ERR_CRC              1
#define ERR_UNKNOWN_CMD      2
#define ERR_BAD_LEN          3
#define ERR_BAD_ARG          4

#define FLAG_ARMED           (1 << 0)
#define FLAG_READY           (1 << 1)
#define FLAG_GUIDE           (1 << 2)
#define FLAG_HB_TRIPPED      (1 << 3)
#define FLAG_5V_PRESENT      (1 << 4)

static uint8_t crc8_update(uint8_t crc, uint8_t b) {
    crc ^= b;
    for (uint8_t i = 0; i < 8; i++)
        crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    return crc;
}

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
static const uint8_t POWER_PINS[8] = {
    PIN_PWR_BIT0, PIN_PWR_BIT1, PIN_PWR_BIT2, PIN_PWR_BIT3,
    PIN_PWR_BIT4, PIN_PWR_BIT5, PIN_PWR_BIT6, PIN_PWR_BIT7
};

// Pre-resolved output registers for fast power-word writes.
static volatile uint8_t *power_reg[8];
static uint8_t power_mask[8];

static bool     armed;
static uint32_t arm_t0;
static bool     guide_on;
static uint8_t  power_word;
static bool     hb_tripped;
static uint8_t  last_error;
static uint32_t last_valid_rx;
static bool     v5_present;
static uint16_t vdetect_mv;

// ---------------------------------------------------------------------------
// Hardware helpers
// ---------------------------------------------------------------------------
// Runs right after reset (before main/Arduino init): turn the watchdog off so
// a WDT reset does not leave a short timeout running through init.
void wdt_early_off(void) __attribute__((naked, used, section(".init3")));
void wdt_early_off(void) {
    MCUSR = 0;
    wdt_disable();
}

static void outputs_safe(void) {
    // Drive the level before the direction so pins never glitch high.
    for (uint8_t i = 0; i < 8; i++) {
        digitalWrite(POWER_PINS[i], LOW);
        pinMode(POWER_PINS[i], OUTPUT);
    }
    digitalWrite(PIN_LATCH, LOW);           pinMode(PIN_LATCH, OUTPUT);
    digitalWrite(PIN_EMISSION_ENABLE, LOW); pinMode(PIN_EMISSION_ENABLE, OUTPUT);
    digitalWrite(PIN_GUIDE_LASER, LOW);     pinMode(PIN_GUIDE_LASER, OUTPUT);
    digitalWrite(PIN_AUX_OFF, HIGH);        pinMode(PIN_AUX_OFF, OUTPUT);
    digitalWrite(PIN_RED_LED, LOW);         pinMode(PIN_RED_LED, OUTPUT);
    digitalWrite(PIN_GREEN_LED, LOW);       pinMode(PIN_GREEN_LED, OUTPUT);
    pinMode(PIN_5V_DETECT, INPUT);
    pinMode(PIN_STAT_11, INPUT);
    pinMode(PIN_STAT_12, INPUT);
    pinMode(PIN_STAT_16, INPUT);
}

// Write all 8 bits via direct port access (~2 us total), then strobe LATCH.
static void write_power(uint8_t value) {
    for (uint8_t i = 0; i < 8; i++) {
        if ((value >> i) & 1) *power_reg[i] |= power_mask[i];
        else                  *power_reg[i] &= (uint8_t)~power_mask[i];
    }
    delayMicroseconds(LATCH_SETUP_US);
    digitalWrite(PIN_LATCH, HIGH);
    delayMicroseconds(LATCH_PULSE_US);
    digitalWrite(PIN_LATCH, LOW);
    power_word = value;
}

static void set_armed(bool on) {
    if (on) {
        if (!armed) {
            armed = true;
            arm_t0 = millis();
            digitalWrite(PIN_EMISSION_ENABLE, HIGH);
        }
    } else {
        digitalWrite(PIN_EMISSION_ENABLE, LOW);
        armed = false;
        write_power(0);   // guide is deliberately left alone
    }
}

static void set_guide(bool on) {
    guide_on = on;
    digitalWrite(PIN_GUIDE_LASER, on ? HIGH : LOW);
}

static bool is_ready(void) {
    return armed && (uint32_t)(millis() - arm_t0) >= ARM_SETTLE_MS;
}

// Periodic analog read + LED update (one ~110 us conversion every 100 ms).
static void update_analog(void) {
    static uint32_t last = 0;
    uint32_t now = millis();
    if ((uint32_t)(now - last) < STATUS_PERIOD_MS) return;
    last = now;

    uint16_t raw = analogRead(PIN_5V_DETECT);
    vdetect_mv = (uint16_t)(((uint32_t)raw * 5000UL) / 1024UL);
    if (raw > 700) {            // ~3.4 V: 5 V present
        v5_present = true;
        digitalWrite(PIN_RED_LED, HIGH);
        digitalWrite(PIN_GREEN_LED, LOW);
    } else if (raw < 300) {     // ~1.5 V: 0 V present
        v5_present = false;
        digitalWrite(PIN_RED_LED, LOW);
        digitalWrite(PIN_GREEN_LED, HIGH);
    } else {                    // ambiguous
        v5_present = false;
        digitalWrite(PIN_RED_LED, LOW);
        digitalWrite(PIN_GREEN_LED, LOW);
    }
}

// ---------------------------------------------------------------------------
// Framing
// ---------------------------------------------------------------------------
static void send_frame(uint8_t cmd, const uint8_t *payload, uint8_t len) {
    uint8_t buf[3 + MAX_PAYLOAD + 1];
    uint8_t crc = 0;
    buf[0] = FRAME_SOF;
    buf[1] = cmd;
    buf[2] = len;
    for (uint8_t i = 0; i < len; i++) buf[3 + i] = payload[i];
    for (uint8_t i = 1; i < 3 + len; i++) crc = crc8_update(crc, buf[i]);
    buf[3 + len] = crc;
    Serial.write(buf, 4 + len);
}

static void send_nak(uint8_t err) {
    last_error = err;
    send_frame(CMD_NAK, &err, 1);
}

static void send_status(uint8_t cmd) {
    uint8_t p[8];
    uint8_t stat = (digitalRead(PIN_STAT_11) ? 1 : 0) |
                   (digitalRead(PIN_STAT_12) ? 2 : 0) |
                   (digitalRead(PIN_STAT_16) ? 4 : 0);
    p[0] = (armed ? FLAG_ARMED : 0) | (is_ready() ? FLAG_READY : 0) |
           (guide_on ? FLAG_GUIDE : 0) | (hb_tripped ? FLAG_HB_TRIPPED : 0) |
           (v5_present ? FLAG_5V_PRESENT : 0);
    p[1] = power_word;
    p[2] = stat;
    p[3] = (uint8_t)(vdetect_mv >> 8);
    p[4] = (uint8_t)(vdetect_mv & 0xFF);
    p[5] = PROTOCOL_VERSION;
    p[6] = last_error;
    p[7] = 0;
    hb_tripped = false;     // latched flag is cleared once reported
    send_frame(cmd | CMD_ACK_FLAG, p, 8);
}

static void handle_frame(uint8_t cmd, const uint8_t *pl, uint8_t len) {
    switch (cmd) {
    case CMD_SET_ARM:
    case CMD_SET_GUIDE:
        if (len != 1) { send_nak(ERR_BAD_LEN); return; }
        if (pl[0] > 1) { send_nak(ERR_BAD_ARG); return; }
        if (cmd == CMD_SET_ARM) set_armed(pl[0] != 0);
        else                    set_guide(pl[0] != 0);
        break;
    case CMD_SET_POWER:
        if (len != 1) { send_nak(ERR_BAD_LEN); return; }
        write_power(pl[0]);
        break;
    case CMD_GET_STATUS:
        if (len != 0) { send_nak(ERR_BAD_LEN); return; }
        break;
    default:
        send_nak(ERR_UNKNOWN_CMD);
        return;
    }
    last_valid_rx = millis();
    send_status(cmd);   // reply only after the action completed (power latched)
}

// Byte-wise parser: hunts for SOF, drops partial frames after FRAME_TIMEOUT_MS.
static void parse_serial(void) {
    enum { S_SOF, S_CMD, S_LEN, S_PAYLOAD, S_CRC };
    static uint8_t state = S_SOF;
    static uint8_t cmd, len, idx, crc;
    static uint8_t payload[MAX_PAYLOAD];
    static uint32_t last_byte_ms;

    if (state != S_SOF && (uint32_t)(millis() - last_byte_ms) > FRAME_TIMEOUT_MS)
        state = S_SOF;

    while (Serial.available() > 0) {
        uint8_t b = (uint8_t)Serial.read();
        last_byte_ms = millis();
        switch (state) {
        case S_SOF:
            if (b == FRAME_SOF) state = S_CMD;
            break;
        case S_CMD:
            cmd = b; crc = crc8_update(0, b); state = S_LEN;
            break;
        case S_LEN:
            if (b > MAX_PAYLOAD) {
                send_nak(ERR_BAD_LEN);
                state = S_SOF;
                break;
            }
            len = b; idx = 0; crc = crc8_update(crc, b);
            state = len ? S_PAYLOAD : S_CRC;
            break;
        case S_PAYLOAD:
            payload[idx++] = b; crc = crc8_update(crc, b);
            if (idx >= len) state = S_CRC;
            break;
        case S_CRC:
            state = S_SOF;
            if (b == crc) handle_frame(cmd, payload, len);
            else          send_nak(ERR_CRC);
            break;
        }
    }
}

// ---------------------------------------------------------------------------
void setup() {
    // Safe state first, before anything else.
    for (uint8_t i = 0; i < 8; i++) {
        power_reg[i]  = portOutputRegister(digitalPinToPort(POWER_PINS[i]));
        power_mask[i] = digitalPinToBitMask(POWER_PINS[i]);
    }
    outputs_safe();
    armed = false;
    guide_on = false;
    power_word = 0;

    Serial.begin(SERIAL_BAUD);
    last_valid_rx = millis();
    wdt_enable(WDTO_250MS);
}

void loop() {
    wdt_reset();
    parse_serial();
    update_analog();

    if (armed && (uint32_t)(millis() - last_valid_rx) > HEARTBEAT_TIMEOUT_MS) {
        set_armed(false);       // also zeroes + latches power
        hb_tripped = true;
    }
}
