#include "zbro_proto.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#define MAX_TOK 12

/* Split a writable string in place on blanks. Returns the token count, or -1 if there are more than max. */
static int tokenize(char *s, char **tok, int max)
{
    int n = 0;
    while (*s) {
        while (*s == ' ' || *s == '\t') s++;
        if (!*s) break;
        if (n == max) return -1;
        tok[n++] = s;
        while (*s && *s != ' ' && *s != '\t') s++;
        if (*s) *s++ = '\0';
    }
    return n;
}

static bool parse_float(const char *s, float *out)
{
    if (!s || !*s) return false;
    char *end;
    double v = strtod(s, &end);
    if (*end != '\0' || !isfinite(v) || fabs(v) > 1e6) return false;
    *out = (float)v;
    return true;
}

static bool ieq(const char *a, const char *b)
{
    for (; *a && *b; a++, b++)
        if (toupper((unsigned char)*a) != toupper((unsigned char)*b)) return false;
    return *a == *b;
}

static bool copy_name(char *dst, size_t n, const char *src)
{
    size_t l = strlen(src);
    if (l == 0 || l >= n) return false;
    memcpy(dst, src, l + 1);
    return true;
}

int zbro_parse_request(const char *line, size_t len, struct zbro_req *out)
{
    char buf[ZBRO_MAX_LINE + 1];
    char *tok[MAX_TOK];
    if (!line || !out) return ZBRO_E_ARG;
    while (len > 0 && (line[len - 1] == '\r' || line[len - 1] == '\n')) len--;
    memset(out, 0, sizeof(*out));
    if (len > ZBRO_MAX_LINE) return ZBRO_E_ARG;
    memcpy(buf, line, len);
    buf[len] = '\0';
    if (memchr(buf, '\0', len)) return ZBRO_E_ARG;   /* embedded NUL: len mismatch */
    int nt = tokenize(buf, tok, MAX_TOK);
    if (nt == 0) return ZBRO_E_UNKNOWN;
    if (nt < 0) {
        /* known command with too many words is a bad argument, anything else unknown */
        return ZBRO_E_ARG;
    }
    const char *c = tok[0];
    int na = nt - 1;
    static const struct { const char *w; enum zbro_cmd c; } simple[] = {
        {"HELLO", ZBRO_C_HELLO}, {"PING", ZBRO_C_PING}, {"POS", ZBRO_C_POS}, {"HOME", ZBRO_C_HOME},
        {"STOP", ZBRO_C_STOP}, {"ESTOP", ZBRO_C_ESTOP}, {"SAVE", ZBRO_C_SAVE},
    };
    for (size_t i = 0; i < sizeof(simple) / sizeof(simple[0]); i++) {
        if (ieq(c, simple[i].w)) {
            out->cmd = simple[i].c;
            return na == 0 ? ZBRO_E_NONE : ZBRO_E_ARG;
        }
    }
    if (ieq(c, "MOVE") || ieq(c, "JOG")) {
        out->cmd = ieq(c, "MOVE") ? ZBRO_C_MOVE : ZBRO_C_JOG;
        if (na < 1 || na > 2) return ZBRO_E_ARG;
        if (!parse_float(tok[1], &out->a)) return ZBRO_E_ARG;
        if (na == 2) {
            if (!parse_float(tok[2], &out->b) || out->b < 0) return ZBRO_E_ARG;
            out->has_b = true;
        }
        return ZBRO_E_NONE;
    }
    if (ieq(c, "GET")) {
        out->cmd = ZBRO_C_GET;
        if (na != 1 || !copy_name(out->key, sizeof(out->key), tok[1])) return ZBRO_E_ARG;
        return ZBRO_E_NONE;
    }
    if (ieq(c, "SET")) {
        out->cmd = ZBRO_C_SET;
        if (na != 2 || !copy_name(out->key, sizeof(out->key), tok[1])) return ZBRO_E_ARG;
        if (!parse_float(tok[2], &out->a)) return ZBRO_E_ARG;
        return ZBRO_E_NONE;
    }
    if (ieq(c, "CAL")) {
        out->cmd = ZBRO_C_CAL;
        if (na < 1) return ZBRO_E_ARG;
        if (ieq(tok[1], "START")) {
            out->cal_op = ZBRO_CAL_START;
            if (na - 1 > ZBRO_CAL_TESTS) return ZBRO_E_ARG;
            for (int i = 2; i < nt; i++)
                if (!copy_name(out->tests[out->ntests++], ZBRO_CAL_NAME, tok[i])) return ZBRO_E_ARG;
            return ZBRO_E_NONE;
        }
        if (na != 1) return ZBRO_E_ARG;
        if (ieq(tok[1], "ABORT")) out->cal_op = ZBRO_CAL_ABORT;
        else if (ieq(tok[1], "STATUS")) out->cal_op = ZBRO_CAL_STATUS;
        else return ZBRO_E_ARG;
        return ZBRO_E_NONE;
    }
    return ZBRO_E_UNKNOWN;
}

void zbro_lines_init(struct zbro_lines *l)
{
    l->n = 0;
    l->overflow = false;
}

int zbro_lines_feed(struct zbro_lines *l, char c, const char **line, size_t *len)
{
    if (c != '\n') {
        if (l->overflow) return 0;
        if (l->n >= ZBRO_MAX_REPLY) { l->overflow = true; return 0; }
        l->buf[l->n++] = c;
        return 0;
    }
    if (l->overflow) {
        l->overflow = false;
        l->n = 0;
        return -1;
    }
    size_t n = l->n;
    l->n = 0;
    while (n > 0 && l->buf[n - 1] == '\r') n--;
    if (n == 0) return 0;
    l->buf[n] = '\0';
    *line = l->buf;
    *len = n;
    return 1;
}

/* ---- formatters ---- */

static const char *const state_names[] = {"IDLE", "MOVING", "HOMING", "CAL", "FAULT"};

const char *zbro_state_name(int state)
{
    if (state < 0 || state > 4) return "FAULT";
    return state_names[state];
}

int zbro_state_from_name(const char *s)
{
    for (int i = 0; i < 5; i++)
        if (strcmp(s, state_names[i]) == 0) return i;
    return -1;
}

static int fin(int r, size_t n)
{
    return (r < 0 || (size_t)r >= n) ? -1 : r;
}

int zbro_fmt_ok(char *b, size_t n) { return fin(snprintf(b, n, "OK\n"), n); }

int zbro_fmt_hello(char *b, size_t n, const char *fw)
{
    return fin(snprintf(b, n, "OK ZBRO %d %s\n", ZBRO_PROTO_VER, fw), n);
}

int zbro_fmt_pos(char *b, size_t n, float mm, int state, bool homed, bool endstop)
{
    return fin(snprintf(b, n, "POS %.3f %s %d %d\n", (double)mm, zbro_state_name(state), homed ? 1 : 0,
                        endstop ? 1 : 0), n);
}

int zbro_fmt_evt_homed(char *b, size_t n, float mm) { return fin(snprintf(b, n, "EVT HOMED %.3f\n", (double)mm), n); }
int zbro_fmt_evt_done(char *b, size_t n, float mm) { return fin(snprintf(b, n, "EVT DONE %.3f\n", (double)mm), n); }
int zbro_fmt_evt_err(char *b, size_t n, int code) { return fin(snprintf(b, n, "EVT ERR %d\n", code), n); }

int zbro_fmt_val(char *b, size_t n, const char *key, float v)
{
    return fin(snprintf(b, n, "VAL %s %.6g\n", key, (double)v), n);
}

int zbro_fmt_err(char *b, size_t n, int code, const char *text)
{
    return fin(snprintf(b, n, "ERR %d %s\n", code, text ? text : ""), n);
}

int zbro_fmt_cal(char *b, size_t n, const char *json) { return fin(snprintf(b, n, "CAL %s\n", json), n); }

int zbro_fmt_beacon(char *b, size_t n, const char *ip, int port, const char *hostname)
{
    return fin(snprintf(b, n, "ZBRO %d %s %d %s\n", ZBRO_PROTO_VER, ip, port, hostname), n);
}

int zbro_fmt_req_simple(char *b, size_t n, const char *word) { return fin(snprintf(b, n, "%s\n", word), n); }

int zbro_fmt_req_move(char *b, size_t n, bool rel, float mm, float speed)
{
    if (speed > 0)
        return fin(snprintf(b, n, "%s %.4f %.3f\n", rel ? "JOG" : "MOVE", (double)mm, (double)speed), n);
    return fin(snprintf(b, n, "%s %.4f\n", rel ? "JOG" : "MOVE", (double)mm), n);
}

int zbro_fmt_req_get(char *b, size_t n, const char *key) { return fin(snprintf(b, n, "GET %s\n", key), n); }

int zbro_fmt_req_set(char *b, size_t n, const char *key, float v)
{
    return fin(snprintf(b, n, "SET %s %.6g\n", key, (double)v), n);
}

/* ---- reply parsing ---- */

static bool set_text(char *dst, size_t n, const char *s)
{
    size_t l = strlen(s);
    if (l >= n) return false;
    memcpy(dst, s, l + 1);
    return true;
}

bool zbro_parse_reply(const char *line, size_t len, struct zbro_reply *out)
{
    char buf[ZBRO_MAX_REPLY + 1];
    char *tok[MAX_TOK];
    if (!line || !out) return false;
    while (len > 0 && (line[len - 1] == '\r' || line[len - 1] == '\n')) len--;
    if (len == 0 || len > ZBRO_MAX_REPLY) return false;
    memcpy(buf, line, len);
    buf[len] = '\0';
    memset(out, 0, sizeof(*out));

    /* Free-text tails (ERR text, CAL json) are taken verbatim after the keyword. */
    if (strncmp(buf, "CAL ", 4) == 0) {
        out->kind = ZBRO_R_CAL;
        return set_text(out->text, sizeof(out->text), buf + 4);
    }
    if (strncmp(buf, "EVT CAL", 7) == 0) {
        out->kind = ZBRO_R_EVT_CAL;
        return set_text(out->text, sizeof(out->text), buf[7] == ' ' ? buf + 8 : buf + 7);
    }
    if (strncmp(buf, "ERR ", 4) == 0) {
        char *e;
        long code = strtol(buf + 4, &e, 10);
        if (e == buf + 4 || (*e != '\0' && *e != ' ')) return false;
        out->kind = ZBRO_R_ERR;
        out->code = (int)code;
        return set_text(out->text, sizeof(out->text), *e == ' ' ? e + 1 : e);
    }

    int nt = tokenize(buf, tok, MAX_TOK);
    if (nt <= 0) return false;
    if (strcmp(tok[0], "OK") == 0) {
        if (nt == 1) { out->kind = ZBRO_R_OK; return true; }
        if (nt == 4 && strcmp(tok[1], "ZBRO") == 0) {
            out->kind = ZBRO_R_HELLO;
            out->code = atoi(tok[2]);
            return set_text(out->text, sizeof(out->text), tok[3]);
        }
        return false;
    }
    if (strcmp(tok[0], "POS") == 0) {
        if (nt != 5) return false;
        out->kind = ZBRO_R_POS;
        if (!parse_float(tok[1], &out->pos)) return false;
        out->state = zbro_state_from_name(tok[2]);
        if (out->state < 0) return false;
        out->homed = tok[3][0] == '1';
        out->endstop = tok[4][0] == '1';
        return true;
    }
    if (strcmp(tok[0], "VAL") == 0) {
        if (nt != 3) return false;
        out->kind = ZBRO_R_VAL;
        return copy_name(out->key, sizeof(out->key), tok[1]) && parse_float(tok[2], &out->val);
    }
    if (strcmp(tok[0], "EVT") == 0 && nt == 3) {
        if (strcmp(tok[1], "HOMED") == 0 || strcmp(tok[1], "DONE") == 0) {
            out->kind = tok[1][0] == 'H' ? ZBRO_R_EVT_HOMED : ZBRO_R_EVT_DONE;
            return parse_float(tok[2], &out->pos);
        }
        if (strcmp(tok[1], "ERR") == 0) {
            out->kind = ZBRO_R_EVT_ERR;
            out->code = atoi(tok[2]);
            return true;
        }
    }
    return false;
}

bool zbro_parse_beacon(const char *line, size_t len, char *ip, size_t ipn, int *port, char *host, size_t hn)
{
    char buf[ZBRO_MAX_LINE + 1];
    char *tok[MAX_TOK];
    while (len > 0 && (line[len - 1] == '\r' || line[len - 1] == '\n')) len--;
    if (len == 0 || len > ZBRO_MAX_LINE) return false;
    memcpy(buf, line, len);
    buf[len] = '\0';
    if (tokenize(buf, tok, MAX_TOK) != 5 || strcmp(tok[0], "ZBRO") != 0) return false;
    if (atoi(tok[1]) != ZBRO_PROTO_VER) return false;
    int p = atoi(tok[3]);
    if (p <= 0 || p > 65535) return false;
    if (strlen(tok[2]) >= ipn || strlen(tok[4]) >= hn) return false;
    strcpy(ip, tok[2]);
    strcpy(host, tok[4]);
    *port = p;
    return true;
}
