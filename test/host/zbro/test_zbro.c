#include "zbro_proto.h"

#include <stdio.h>
#include <string.h>

static int g_fail, g_checks;
#define CHECK(c) do { g_checks++; if (!(c)) { g_fail++; printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); } } while (0)

static int P(const char *s, struct zbro_req *r) { return zbro_parse_request(s, strlen(s), r); }

static void test_parse(void)
{
    struct zbro_req r;
    CHECK(P("HELLO", &r) == 0 && r.cmd == ZBRO_C_HELLO);
    CHECK(P("ping\r", &r) == 0 && r.cmd == ZBRO_C_PING);
    CHECK(P("  POS  ", &r) == 0 && r.cmd == ZBRO_C_POS);
    CHECK(P("HOME", &r) == 0 && r.cmd == ZBRO_C_HOME);
    CHECK(P("STOP", &r) == 0 && r.cmd == ZBRO_C_STOP);
    CHECK(P("ESTOP", &r) == 0 && r.cmd == ZBRO_C_ESTOP);
    CHECK(P("SAVE", &r) == 0 && r.cmd == ZBRO_C_SAVE);
    CHECK(P("HOME 1", &r) == ZBRO_E_ARG);
    CHECK(P("PING x", &r) == ZBRO_E_ARG);

    CHECK(P("MOVE 12.5", &r) == 0 && r.cmd == ZBRO_C_MOVE && r.a == 12.5f && !r.has_b);
    CHECK(P("MOVE -3 7.5", &r) == 0 && r.a == -3.0f && r.has_b && r.b == 7.5f);
    CHECK(P("JOG -0.25 2", &r) == 0 && r.cmd == ZBRO_C_JOG && r.a == -0.25f && r.b == 2.0f);
    CHECK(P("MOVE 1e2", &r) == 0 && r.a == 100.0f);
    CHECK(P("MOVE", &r) == ZBRO_E_ARG);
    CHECK(P("MOVE 1 2 3", &r) == ZBRO_E_ARG);
    CHECK(P("MOVE abc", &r) == ZBRO_E_ARG);
    CHECK(P("MOVE 1x", &r) == ZBRO_E_ARG);
    CHECK(P("MOVE 1,5", &r) == ZBRO_E_ARG);
    CHECK(P("MOVE nan", &r) == ZBRO_E_ARG);
    CHECK(P("MOVE NaN", &r) == ZBRO_E_ARG);
    CHECK(P("MOVE inf", &r) == ZBRO_E_ARG);
    CHECK(P("MOVE -inf", &r) == ZBRO_E_ARG);
    CHECK(P("MOVE 1e999", &r) == ZBRO_E_ARG);
    CHECK(P("MOVE 1e30", &r) == ZBRO_E_ARG);
    CHECK(P("MOVE 5 nan", &r) == ZBRO_E_ARG);
    CHECK(P("MOVE 5 -1", &r) == ZBRO_E_ARG);
    CHECK(P("JOG 0x10", &r) == 0 && r.a == 16.0f);   /* strtod accepts hex; documented, finite */

    CHECK(P("GET max_speed", &r) == 0 && r.cmd == ZBRO_C_GET && strcmp(r.key, "max_speed") == 0);
    CHECK(P("GET", &r) == ZBRO_E_ARG);
    CHECK(P("GET a b", &r) == ZBRO_E_ARG);
    CHECK(P("GET abcdefghijklmnopqrstuvwxyz", &r) == ZBRO_E_ARG);
    CHECK(P("SET accel 150", &r) == 0 && r.cmd == ZBRO_C_SET && r.a == 150.0f && strcmp(r.key, "accel") == 0);
    CHECK(P("SET accel", &r) == ZBRO_E_ARG);
    CHECK(P("SET accel nan", &r) == ZBRO_E_ARG);
    CHECK(P("SET accel 1 2", &r) == ZBRO_E_ARG);

    CHECK(P("CAL start", &r) == 0 && r.cal_op == ZBRO_CAL_START && r.ntests == 0);
    CHECK(P("CAL START speed accel", &r) == 0 && r.ntests == 2 && strcmp(r.tests[1], "accel") == 0);
    CHECK(P("CAL abort", &r) == 0 && r.cal_op == ZBRO_CAL_ABORT);
    CHECK(P("CAL status", &r) == 0 && r.cal_op == ZBRO_CAL_STATUS);
    CHECK(P("CAL", &r) == ZBRO_E_ARG);
    CHECK(P("CAL bogus", &r) == ZBRO_E_ARG);
    CHECK(P("CAL abort now", &r) == ZBRO_E_ARG);
    CHECK(P("CAL start a b c d e f g h i", &r) == ZBRO_E_ARG);
    CHECK(P("CAL start abcdefghijklmnopqrstuvwxyz", &r) == ZBRO_E_ARG);

    CHECK(P("", &r) == ZBRO_E_UNKNOWN);
    CHECK(P("   ", &r) == ZBRO_E_UNKNOWN);
    CHECK(P("FOO 1", &r) == ZBRO_E_UNKNOWN);
    CHECK(P("MOVES 1", &r) == ZBRO_E_UNKNOWN);

    /* overlong */
    char big[400];
    memset(big, 'A', sizeof(big));
    CHECK(zbro_parse_request(big, ZBRO_MAX_LINE + 1, &r) == ZBRO_E_ARG);
    memcpy(big, "PING", 4);
    memset(big + 4, ' ', 124);                    /* exactly 128 bytes */
    CHECK(zbro_parse_request(big, ZBRO_MAX_LINE, &r) == 0);
    CHECK(zbro_parse_request(big, ZBRO_MAX_LINE + 1, &r) == ZBRO_E_ARG);
    CHECK(zbro_parse_request("PI\0NG", 5, &r) == ZBRO_E_ARG);   /* embedded NUL */
    CHECK(zbro_parse_request(NULL, 0, &r) == ZBRO_E_ARG);
    CHECK(P("MOVE 1 2 3 4 5 6 7 8 9 10 11 12 13", &r) == ZBRO_E_ARG);
}

static void test_lines(void)
{
    struct zbro_lines l;
    const char *line = NULL;
    size_t len = 0;
    const char *in = "PING\r\n\n\nPOS\nMO";
    int got = 0;
    zbro_lines_init(&l);
    for (const char *p = in; *p; p++) {
        int r = zbro_lines_feed(&l, *p, &line, &len);
        if (r == 1) {
            got++;
            if (got == 1) CHECK(len == 4 && strcmp(line, "PING") == 0);
            if (got == 2) CHECK(len == 3 && strcmp(line, "POS") == 0);
        }
    }
    CHECK(got == 2);
    CHECK(zbro_lines_feed(&l, 'V', &line, &len) == 0);
    CHECK(zbro_lines_feed(&l, 'E', &line, &len) == 0);
    CHECK(zbro_lines_feed(&l, '\n', &line, &len) == 1 && strcmp(line, "MOVE") == 0);

    /* overlong line is dropped, next one is fine */
    int res = 0;
    for (int i = 0; i < ZBRO_MAX_REPLY + 50; i++) zbro_lines_feed(&l, 'x', &line, &len);
    res = zbro_lines_feed(&l, '\n', &line, &len);
    CHECK(res == -1);
    CHECK(zbro_lines_feed(&l, 'P', &line, &len) == 0);
    CHECK(zbro_lines_feed(&l, '\n', &line, &len) == 1 && strcmp(line, "P") == 0);
    /* exactly at the limit is accepted */
    for (int i = 0; i < ZBRO_MAX_REPLY; i++) zbro_lines_feed(&l, 'y', &line, &len);
    CHECK(zbro_lines_feed(&l, '\n', &line, &len) == 1 && len == ZBRO_MAX_REPLY);
    /* a 129..512 byte line passes the assembler but the request parser rejects it */
    for (int i = 0; i < 200; i++) zbro_lines_feed(&l, 'P', &line, &len);
    CHECK(zbro_lines_feed(&l, '\n', &line, &len) == 1);
    struct zbro_req r;
    CHECK(zbro_parse_request(line, len, &r) == ZBRO_E_ARG);
}

static void test_fmt(void)
{
    char b[64];
    CHECK(zbro_fmt_ok(b, sizeof b) == 3 && strcmp(b, "OK\n") == 0);
    zbro_fmt_hello(b, sizeof b, "1.0.0");
    CHECK(strcmp(b, "OK ZBRO 1 1.0.0\n") == 0);
    zbro_fmt_pos(b, sizeof b, 12.3456f, 1, true, false);
    CHECK(strcmp(b, "POS 12.346 MOVING 1 0\n") == 0);
    zbro_fmt_evt_homed(b, sizeof b, 0.0f);
    CHECK(strcmp(b, "EVT HOMED 0.000\n") == 0);
    zbro_fmt_evt_done(b, sizeof b, -1.5f);
    CHECK(strcmp(b, "EVT DONE -1.500\n") == 0);
    zbro_fmt_evt_err(b, sizeof b, 3);
    CHECK(strcmp(b, "EVT ERR 3\n") == 0);
    zbro_fmt_val(b, sizeof b, "accel", 100.0f);
    CHECK(strcmp(b, "VAL accel 100\n") == 0);
    zbro_fmt_err(b, sizeof b, 4, "bad argument");
    CHECK(strcmp(b, "ERR 4 bad argument\n") == 0);
    zbro_fmt_beacon(b, sizeof b, "192.168.1.5", 7700, "zbro");
    CHECK(strcmp(b, "ZBRO 1 192.168.1.5 7700 zbro\n") == 0);
    CHECK(zbro_fmt_err(b, 8, 4, "much too long for it") == -1);
    CHECK(zbro_fmt_pos(b, 5, 1.0f, 0, false, false) == -1);
    CHECK(zbro_fmt_ok(b, 3) == -1);
    CHECK(zbro_fmt_ok(b, 4) == 3);
    CHECK(strcmp(zbro_state_name(3), "CAL") == 0 && strcmp(zbro_state_name(99), "FAULT") == 0);
    CHECK(zbro_state_from_name("HOMING") == 2 && zbro_state_from_name("x") == -1);
    zbro_fmt_req_move(b, sizeof b, false, 5.0f, 0.0f);
    CHECK(strcmp(b, "MOVE 5.0000\n") == 0);
    zbro_fmt_req_move(b, sizeof b, true, -1.0f, 2.5f);
    CHECK(strcmp(b, "JOG -1.0000 2.500\n") == 0);
    zbro_fmt_req_set(b, sizeof b, "accel", 12.5f);
    CHECK(strcmp(b, "SET accel 12.5\n") == 0);
    zbro_fmt_req_get(b, sizeof b, "accel");
    CHECK(strcmp(b, "GET accel\n") == 0);

    /* formatted requests parse back */
    struct zbro_req r;
    zbro_fmt_req_move(b, sizeof b, true, -1.25f, 2.5f);
    CHECK(zbro_parse_request(b, strlen(b), &r) == 0 && r.cmd == ZBRO_C_JOG && r.a == -1.25f && r.b == 2.5f);
}

static bool R(const char *s, struct zbro_reply *r) { return zbro_parse_reply(s, strlen(s), r); }

static void test_reply(void)
{
    struct zbro_reply r;
    CHECK(R("OK", &r) && r.kind == ZBRO_R_OK);
    CHECK(R("OK ZBRO 1 1.0.0\r", &r) && r.kind == ZBRO_R_HELLO && r.code == 1 && strcmp(r.text, "1.0.0") == 0);
    CHECK(R("POS 3.5 HOMING 1 0", &r) && r.kind == ZBRO_R_POS && r.pos == 3.5f && r.state == 2 && r.homed && !r.endstop);
    CHECK(!R("POS 3.5 BOGUS 1 0", &r));
    CHECK(!R("POS nan IDLE 1 0", &r));
    CHECK(!R("POS 3.5 IDLE 1", &r));
    CHECK(R("EVT HOMED 0.000", &r) && r.kind == ZBRO_R_EVT_HOMED);
    CHECK(R("EVT DONE 4.25", &r) && r.kind == ZBRO_R_EVT_DONE && r.pos == 4.25f);
    CHECK(R("EVT ERR 5", &r) && r.kind == ZBRO_R_EVT_ERR && r.code == 5);
    CHECK(R("VAL accel 100", &r) && r.kind == ZBRO_R_VAL && r.val == 100.0f && strcmp(r.key, "accel") == 0);
    CHECK(R("ERR 3 out of range", &r) && r.kind == ZBRO_R_ERR && r.code == 3 && strcmp(r.text, "out of range") == 0);
    CHECK(R("ERR 8", &r) && r.code == 8 && r.text[0] == 0);
    CHECK(!R("ERR x", &r));
    CHECK(R("CAL {\"running\":false}", &r) && r.kind == ZBRO_R_CAL && strcmp(r.text, "{\"running\":false}") == 0);
    CHECK(R("EVT CAL {\"t\":1}", &r) && r.kind == ZBRO_R_EVT_CAL);
    CHECK(!R("", &r));
    CHECK(!R("WAT", &r));

    char ip[16], host[32];
    int port;
    const char *bc = "ZBRO 1 192.168.1.5 7700 zbro\n";
    CHECK(zbro_parse_beacon(bc, strlen(bc), ip, sizeof ip, &port, host, sizeof host));
    CHECK(strcmp(ip, "192.168.1.5") == 0 && port == 7700 && strcmp(host, "zbro") == 0);
    bc = "ZBRO 2 1.2.3.4 7700 zbro";
    CHECK(!zbro_parse_beacon(bc, strlen(bc), ip, sizeof ip, &port, host, sizeof host));
    bc = "ZBRO 1 1.2.3.4 0 zbro";
    CHECK(!zbro_parse_beacon(bc, strlen(bc), ip, sizeof ip, &port, host, sizeof host));
    bc = "ZBRO 1 1.2.3.4 7700";
    CHECK(!zbro_parse_beacon(bc, strlen(bc), ip, sizeof ip, &port, host, sizeof host));
}

int main(void)
{
    test_parse();
    test_lines();
    test_fmt();
    test_reply();
    printf("%d checks, %d failures\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
