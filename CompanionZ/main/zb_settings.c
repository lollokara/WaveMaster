#include "zb_settings.h"

#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "zb_setcore.c"   /* pure table/validation/JSON, also built by the host test */

#define NVS_NS "zbro"
#define HOST_KEY "hostname"
#define HOST_DEF "zbro"

static float s_val[ZS_COUNT];
static uint32_t s_dirty;
static unsigned s_gen;
static nvs_handle_t s_nvs;
static bool s_nvs_ok;
static SemaphoreHandle_t s_mtx;
static char s_host[32] = HOST_DEF;

#define FLEET_KEY "fleet_hub"
#define FLEET_MAX 127
static char s_fleet[FLEET_MAX + 1] = ZB_FLEET_HUB_DEFAULT;

static void lock(void)   { if (s_mtx) xSemaphoreTake(s_mtx, portMAX_DELAY); }
static void unlock(void) { if (s_mtx) xSemaphoreGive(s_mtx); }

static void nvs_put(int id)
{
    uint32_t bits;
    memcpy(&bits, &s_val[id], sizeof bits);
    if (s_nvs_ok && nvs_set_u32(s_nvs, zbc_defs[id].key, bits) == ESP_OK) s_dirty &= ~(1u << id);
}

/* "" (discover via mDNS), "off", or http(s)://host[:port][/] without whitespace or quotes. */
static bool fleet_valid(const char *u)
{
    if (!u) return false;
    size_t n = strlen(u);
    if (n == 0 || strcmp(u, "off") == 0) return true;
    if (n > FLEET_MAX) return false;
    if (strncmp(u, "http://", 7) != 0 && strncmp(u, "https://", 8) != 0) return false;
    for (size_t i = 0; i < n; i++)
        if ((unsigned char)u[i] <= ' ' || u[i] == '"' || u[i] == '\\' || u[i] == 0x7f) return false;
    return n > (u[4] == 's' ? 8u : 7u);
}

void zb_settings_init(void)
{
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        e = nvs_flash_init();
    }
    s_nvs_ok = (e == ESP_OK) && nvs_open(NVS_NS, NVS_READWRITE, &s_nvs) == ESP_OK;
    for (int i = 0; i < ZS_COUNT; i++) s_val[i] = zbc_defs[i].def;
    if (s_nvs_ok) {
        for (int i = 0; i < ZS_COUNT; i++) {
            uint32_t bits;
            if (nvs_get_u32(s_nvs, zbc_defs[i].key, &bits) != ESP_OK) continue;
            float v;
            memcpy(&v, &bits, sizeof v);
            /* validate against the plain table range: caps depend on values that may load later */
            float base[ZS_COUNT];
            for (int k = 0; k < ZS_COUNT; k++) base[k] = 0.0f;
            if (zbc_validate(base, (enum zb_set_id)i, v) == 0) s_val[i] = v;
        }
        size_t n = sizeof s_host;
        char tmp[32];
        if (nvs_get_str(s_nvs, HOST_KEY, tmp, &n) == ESP_OK && zbc_hostname_valid(tmp)) strcpy(s_host, tmp);
        n = sizeof s_fleet;
        char ftmp[sizeof s_fleet];
        if (nvs_get_str(s_nvs, FLEET_KEY, ftmp, &n) == ESP_OK && fleet_valid(ftmp)) strcpy(s_fleet, ftmp);
    }
    s_mtx = xSemaphoreCreateMutex();
}

float zb_set_get(enum zb_set_id id)
{
    return (id >= 0 && id < ZS_COUNT) ? s_val[id] : 0.0f;
}

int zb_set_set(enum zb_set_id id, float v, bool persist)
{
    lock();
    int r = zbc_validate(s_val, id, v);
    if (r == 0 && s_val[id] != v) {
        s_val[id] = v;
        s_dirty |= 1u << id;
        s_gen++;
        if (persist) {
            nvs_put(id);
            if (s_nvs_ok) nvs_commit(s_nvs);
        }
    }
    unlock();
    return r;
}

int zb_set_find(const char *key)
{
    if (!key) return -1;
    for (int i = 0; i < ZS_COUNT; i++)
        if (strcmp(zbc_defs[i].key, key) == 0) return i;
    return -1;
}

int zb_set_set_by_key(const char *key, float v, bool persist)
{
    int id = zb_set_find(key);
    return id < 0 ? 4 : zb_set_set((enum zb_set_id)id, v, persist);
}

const char *zb_set_key(enum zb_set_id id)
{
    return (id >= 0 && id < ZS_COUNT) ? zbc_defs[id].key : NULL;
}

bool zb_set_get_by_key(const char *key, float *v)
{
    int id = zb_set_find(key);
    if (id < 0) return false;
    if (v) *v = s_val[id];
    return true;
}

void zb_set_save(void)
{
    lock();
    for (int i = 0; i < ZS_COUNT; i++)
        if (s_dirty & (1u << i)) nvs_put(i);
    if (s_nvs_ok) nvs_commit(s_nvs);
    unlock();
}

void zb_set_range(enum zb_set_id id, float *lo, float *hi)
{
    if (id < 0 || id >= ZS_COUNT) { if (lo) *lo = 0; if (hi) *hi = 0; return; }
    zbc_range(s_val, id, lo, hi);
}

char *zb_set_to_json(void)
{
    lock();
    size_t n = zbc_json(s_val, NULL, 0);
    char *buf = malloc(n + 1);
    if (buf) zbc_json(s_val, buf, n + 1);
    unlock();
    return buf;
}

unsigned zb_set_generation(void) { return s_gen; }

void zb_hostname_get(char *buf, size_t n)
{
    if (!buf || !n) return;
    lock();
    strncpy(buf, s_host, n - 1);
    buf[n - 1] = 0;
    unlock();
}

int zb_hostname_set(const char *name)
{
    if (!zbc_hostname_valid(name)) return 4;
    lock();
    strcpy(s_host, name);
    s_gen++;
    if (s_nvs_ok && nvs_set_str(s_nvs, HOST_KEY, name) == ESP_OK) nvs_commit(s_nvs);
    unlock();
    return 0;
}

void zb_fleet_hub_get(char *buf, size_t n)
{
    if (!buf || !n) return;
    lock();
    strncpy(buf, s_fleet, n - 1);
    buf[n - 1] = 0;
    unlock();
}

int zb_fleet_hub_set(const char *url)
{
    if (!fleet_valid(url)) return 4;
    lock();
    strcpy(s_fleet, url);
    s_gen++;
    if (s_nvs_ok && nvs_set_str(s_nvs, FLEET_KEY, url) == ESP_OK) nvs_commit(s_nvs);
    unlock();
    return 0;
}

float zb_steps_per_mm(void)
{
    return s_val[ZS_STEPS_REV] * s_val[ZS_MICROSTEP] / s_val[ZS_LEAD_MM];
}
