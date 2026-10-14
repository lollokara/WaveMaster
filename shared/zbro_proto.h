#ifndef ZBRO_PROTO_H_
#define ZBRO_PROTO_H_

/* Z-Bro COM protocol v1 (docs/ZBRO_PLAN.md section 3). Pure C99, no dependencies beyond libc, shared by
 * the C3 firmware (CompanionZ/) and the S3 client (src/zbro_link.c). Host tested in test/host/zbro. */

#include <stdbool.h>
#include <stddef.h>

#define ZBRO_PORT_TCP   7700
#define ZBRO_PORT_UDP   7701
#define ZBRO_PROTO_VER  1
#define ZBRO_MAX_LINE   128          /* max request length, bytes excluding the terminating '\n' */
#define ZBRO_MAX_REPLY  512          /* max reply/line the assembler accepts (CAL json replies are long) */
#define ZBRO_KEY_MAX    24
#define ZBRO_CAL_TESTS  8
#define ZBRO_CAL_NAME   16

/* Error codes (same numbers as enum zb_err in zb_motion.h). */
enum {
    ZBRO_E_NONE = 0, ZBRO_E_BUSY = 1, ZBRO_E_NOT_HOMED = 2, ZBRO_E_RANGE = 3, ZBRO_E_ARG = 4,
    ZBRO_E_ENDSTOP = 5, ZBRO_E_HOME_FAIL = 6, ZBRO_E_FAULT = 7, ZBRO_E_UNKNOWN = 8
};

/* ---- requests (S3 -> C3) ---- */
enum zbro_cmd {
    ZBRO_C_HELLO, ZBRO_C_PING, ZBRO_C_POS, ZBRO_C_HOME, ZBRO_C_MOVE, ZBRO_C_JOG, ZBRO_C_STOP,
    ZBRO_C_ESTOP, ZBRO_C_GET, ZBRO_C_SET, ZBRO_C_SAVE, ZBRO_C_CAL
};
enum zbro_cal_op { ZBRO_CAL_START, ZBRO_CAL_ABORT, ZBRO_CAL_STATUS };

struct zbro_req {
    enum zbro_cmd cmd;
    float a;                         /* MOVE target / JOG delta / SET value */
    float b;                         /* speed mm/s, valid if has_b */
    bool  has_b;
    char  key[ZBRO_KEY_MAX];         /* GET / SET */
    enum zbro_cal_op cal_op;
    int   ntests;
    char  tests[ZBRO_CAL_TESTS][ZBRO_CAL_NAME];
};

/* Parse one request line (no newline; a trailing '\r' is ignored). Returns ZBRO_E_NONE, ZBRO_E_ARG (known
 * command, bad/missing/extra arguments, overlong, NaN/inf) or ZBRO_E_UNKNOWN (unknown/empty command). */
int zbro_parse_request(const char *line, size_t len, struct zbro_req *out);

/* Incremental line assembler for a byte stream. */
struct zbro_lines { char buf[ZBRO_MAX_REPLY + 1]; size_t n; bool overflow; };
void zbro_lines_init(struct zbro_lines *l);
/* Feed one byte. Returns 0 (need more), 1 (a line is ready: *line is NUL-terminated, valid until the next
 * feed; *len set) or -1 (an overlong line just ended and was discarded). Empty lines are skipped. */
int zbro_lines_feed(struct zbro_lines *l, char c, const char **line, size_t *len);

/* ---- replies / events (C3 -> S3). All return the length written (including the trailing '\n') or -1
 * when the buffer is too small. ---- */
const char *zbro_state_name(int state);          /* 0..4: IDLE MOVING HOMING CAL FAULT */
int zbro_state_from_name(const char *s);         /* -1 if unknown */
int zbro_fmt_ok(char *b, size_t n);
int zbro_fmt_hello(char *b, size_t n, const char *fw);
int zbro_fmt_pos(char *b, size_t n, float mm, int state, bool homed, bool endstop);
int zbro_fmt_evt_homed(char *b, size_t n, float mm);
int zbro_fmt_evt_done(char *b, size_t n, float mm);
int zbro_fmt_evt_err(char *b, size_t n, int code);
int zbro_fmt_val(char *b, size_t n, const char *key, float v);
int zbro_fmt_err(char *b, size_t n, int code, const char *text);
int zbro_fmt_cal(char *b, size_t n, const char *json);   /* "CAL <json>\n", json must not contain '\n' */
int zbro_fmt_beacon(char *b, size_t n, const char *ip, int port, const char *hostname);

/* ---- request formatters (S3 side). Same return convention. ---- */
int zbro_fmt_req_simple(char *b, size_t n, const char *word);           /* HELLO PING POS HOME STOP ESTOP SAVE */
int zbro_fmt_req_move(char *b, size_t n, bool rel, float mm, float speed); /* speed <= 0 omitted */
int zbro_fmt_req_get(char *b, size_t n, const char *key);
int zbro_fmt_req_set(char *b, size_t n, const char *key, float v);

/* ---- reply parsing (S3 side) ---- */
enum zbro_rkind { ZBRO_R_OK, ZBRO_R_HELLO, ZBRO_R_POS, ZBRO_R_EVT_HOMED, ZBRO_R_EVT_DONE, ZBRO_R_EVT_ERR,
                  ZBRO_R_VAL, ZBRO_R_ERR, ZBRO_R_CAL, ZBRO_R_EVT_CAL };
struct zbro_reply {
    enum zbro_rkind kind;
    float pos;                       /* POS / HOMED / DONE */
    int   state;                     /* POS */
    bool  homed, endstop;            /* POS */
    int   code;                      /* ERR / EVT ERR / HELLO protocol version */
    float val;                       /* VAL */
    char  key[ZBRO_KEY_MAX];         /* VAL */
    char  text[ZBRO_MAX_REPLY];      /* ERR text, HELLO fw, CAL json */
};
/* Returns true on success. Line without newline. */
bool zbro_parse_reply(const char *line, size_t len, struct zbro_reply *out);

/* UDP beacon "ZBRO 1 <ip> <port> <hostname>". */
bool zbro_parse_beacon(const char *line, size_t len, char *ip, size_t ipn, int *port, char *host, size_t hn);

#endif
