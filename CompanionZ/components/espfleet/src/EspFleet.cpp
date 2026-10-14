#include "EspFleet.h"

#include <fcntl.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "esp_app_desc.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_heap_caps.h"
#include "esp_http_client.h"
#include "esp_idf_version.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "mbedtls/sha256.h"
#include "nvs.h"
#include "nvs_flash.h"

#if ESPFLEET_ENABLE_MDNS && __has_include("mdns.h")
#include "mdns.h"
#define ESPFLEET_HAVE_MDNS 1
#endif

#define ESPFLEET_AGENT_VERSION "1.1.0"

static const char* TAG = "espfleet";

EspFleetClass EspFleet;

// ----------------------------------------------------------------- log ring
#define LOG_RING_BYTES 8192
#define LOG_MAX_BATCH 32
#define LOG_SHIP_INTERVAL_MS 5000
#define LOG_LINE_MAX 384
#define LOG_TAG_MAX 32
#define LOG_MSG_MAX 400

struct LogRing {
  uint8_t* buf = nullptr;
  size_t cap = 0, head = 0, tail = 0, used = 0;
  uint32_t count = 0;
  uint32_t dropped = 0;

  void init(uint8_t* b, size_t c) {
    buf = b; cap = c;
    head = tail = used = 0;
    count = dropped = 0;
  }

  uint16_t lenAt(size_t pos) const {
    return (uint16_t)(buf[pos] | (buf[(pos + 1) % cap] << 8));
  }

  void popOldest(bool overflow) {
    if (!count) return;
    size_t total = (size_t)lenAt(tail) + 2;
    tail = (tail + total) % cap;
    used -= total;
    count--;
    if (overflow) dropped++;
  }

  void putByte(uint8_t b) {
    buf[head] = b;
    head = (head + 1) % cap;
  }

  bool push(const uint8_t* rec, uint16_t len) {
    size_t need = (size_t)len + 2;
    if (need > cap) return false;
    while (cap - used < need) popOldest(true);
    putByte((uint8_t)len);
    putByte((uint8_t)(len >> 8));
    for (uint16_t k = 0; k < len; k++) putByte(rec[k]);
    used += need;
    count++;
    return true;
  }

  int peek(uint32_t index, uint8_t* out, uint16_t out_cap) const {
    if (index >= count) return -1;
    size_t pos = tail;
    for (uint32_t k = 0; k < index; k++) pos = (pos + lenAt(pos) + 2) % cap;
    uint16_t len = lenAt(pos);
    if (len > out_cap) return -1;
    pos = (pos + 2) % cap;
    for (uint16_t k = 0; k < len; k++) out[k] = buf[(pos + k) % cap];
    return len;
  }

  void drop(uint32_t n) {
    while (n-- && count) popOldest(false);
  }
};

static LogRing s_log_ring;
static uint8_t s_log_buf[LOG_RING_BYTES];
static vprintf_like_t s_prev_vprintf = nullptr;

// VENDORED PATCH (WaveMaster): application hooks, see EspFleet.h.
static espfleet_gate_cb s_gate_cb = nullptr;
static espfleet_health_cb s_health_cb = nullptr;
static int64_t s_next_discover_us = 0;

static bool gate(const char* what) { return s_gate_cb ? s_gate_cb(what) : true; }

// ----------------------------------------------------------------- utilities

// ----------------------------------------------------------------- log parsing
static int logRank(char l) {
  switch (l) {
    case 'E': return 0;
    case 'W': return 1;
    case 'I': return 2;
    case 'D': return 3;
    case 'V': return 4;
    default: return 99;
  }
}

static bool parseEspLogLine(const char* line, char* level, char* tag, size_t tag_cap, const char** msg, size_t* msg_len) {
  const char* p = line;
  if (p[0] == 0x1b && p[1] == '[') {
    while (*p && *p != 'm') p++;
    if (*p == 'm') p++;
  }
  char l = p[0];
  if (logRank(l) == 99 || p[1] != ' ' || p[2] != '(') return false;
  p += 3;
  while (*p >= '0' && *p <= '9') p++;
  if (p[0] != ')' || p[1] != ' ') return false;
  p += 2;
  const char* colon = strstr(p, ": ");
  if (!colon) return false;
  size_t tl = (size_t)(colon - p);
  if (tl >= tag_cap) tl = tag_cap - 1;
  memcpy(tag, p, tl);
  tag[tl] = '\0';
  const char* m = colon + 2;
  size_t ml = strlen(m);
  while (ml && (m[ml - 1] == '\n' || m[ml - 1] == '\r')) ml--;
  if (ml >= 4 && memcmp(m + ml - 4, "\x1b[0m", 4) == 0) ml -= 4;
  *level = l;
  *msg = m;
  *msg_len = ml;
  return true;
}

// VENDORED PATCH (WaveMaster): the hook must be safe from any caller. From an ISR, or while the
// scheduler is not running, it only chains to the previous handler (no mutex, no big stack use). The
// line is formatted once into a small stack buffer and parsed in place (no second copy), and a
// per-task recursion guard keeps a log call made while shipping from re-entering the hook.
static volatile TaskHandle_t s_in_hook = nullptr;

int EspFleetClass::vprintfHook(const char* fmt, va_list ap) {
  if (xPortInIsrContext() || xTaskGetSchedulerState() != taskSCHEDULER_RUNNING) {
    return s_prev_vprintf ? s_prev_vprintf(fmt, ap) : vprintf(fmt, ap);
  }
  va_list copy;
  va_copy(copy, ap);
  int n = s_prev_vprintf ? s_prev_vprintf(fmt, ap) : vprintf(fmt, ap);

  TaskHandle_t self = xTaskGetCurrentTaskHandle();
  if (s_in_hook == self) {
    va_end(copy);
    return n;
  }
  char line[LOG_LINE_MAX];
  vsnprintf(line, sizeof(line), fmt, copy);
  va_end(copy);

  char lvl = 0;
  char tag[LOG_TAG_MAX + 1] = {0};
  const char* msg = nullptr;
  size_t ml = 0;
  if (parseEspLogLine(line, &lvl, tag, sizeof(tag), &msg, &ml) && ml > 0) {
    if (ml > LOG_MSG_MAX) ml = LOG_MSG_MAX;
    line[(msg - line) + ml] = '\0';  // terminate in place, strips the colour reset / newline
    s_in_hook = self;
    EspFleet.pushLog(lvl, tag, msg);
    s_in_hook = nullptr;
  }
  return n;
}

static void copyTrim(char* dst, size_t dst_len, const char* src) {
  if (!src) {
    dst[0] = 0;
    return;
  }
  strncpy(dst, src, dst_len - 1);
  dst[dst_len - 1] = 0;
}

static const char* resetReasonName() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON: return "poweron";
    case ESP_RST_EXT: return "external";
    case ESP_RST_SW: return "software";
    case ESP_RST_PANIC: return "panic";
    case ESP_RST_INT_WDT: return "int_wdt";
    case ESP_RST_TASK_WDT: return "task_wdt";
    case ESP_RST_WDT: return "wdt";
    case ESP_RST_DEEPSLEEP: return "deepsleep";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_SDIO: return "sdio";
    default: return "unknown";
  }
}

static bool currentIp(char* out, size_t len) {
  esp_netif_t* netif = esp_netif_get_default_netif();
  if (!netif) netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
  if (!netif) return false;
  esp_netif_ip_info_t info;
  if (esp_netif_get_ip_info(netif, &info) != ESP_OK || info.ip.addr == 0) return false;
  snprintf(out, len, IPSTR, IP2STR(&info.ip));
  return true;
}

/// Escapes the few characters that can appear in user-supplied strings.
static size_t appendEscaped(char* out, size_t out_len, const char* value) {
  size_t written = 0;
  for (const char* p = value; *p && written + 2 < out_len; ++p) {
    char c = *p;
    if (c == '"' || c == '\\') {
      out[written++] = '\\';
      out[written++] = c;
    } else if ((unsigned char)c < 0x20) {
      continue;
    } else {
      out[written++] = c;
    }
  }
  out[written] = 0;
  return written;
}

/// snprintf that never lets the cursor run past the buffer.
static size_t appendf(char* out, size_t out_len, size_t used, const char* fmt, ...) {
  if (used + 1 >= out_len) return used;
  va_list args;
  va_start(args, fmt);
  int n = vsnprintf(out + used, out_len - used, fmt, args);
  va_end(args);
  if (n < 0) return used;
  used += (size_t)n;
  return used >= out_len ? out_len - 1 : used;
}

static bool jsonStr(const char* json, const char* key, char* out, size_t out_len) {
  char pattern[24];
  snprintf(pattern, sizeof(pattern), "\"%s\"", key);
  const char* p = strstr(json, pattern);
  if (!p) return false;
  p = strchr(p + strlen(pattern), ':');
  if (!p) return false;
  ++p;
  while (*p == ' ') ++p;
  if (*p != '"') return false;
  ++p;
  size_t i = 0;
  while (*p && *p != '"' && i + 1 < out_len) {
    if (*p == '\\' && p[1]) ++p;
    out[i++] = *p++;
  }
  out[i] = 0;
  return true;
}

static long jsonInt(const char* json, const char* key, long fallback) {
  char pattern[24];
  snprintf(pattern, sizeof(pattern), "\"%s\"", key);
  const char* p = strstr(json, pattern);
  if (!p) return fallback;
  p = strchr(p + strlen(pattern), ':');
  if (!p) return fallback;
  ++p;
  while (*p == ' ' || *p == '"') ++p;
  char* end = nullptr;
  long value = strtol(p, &end, 10);
  return end == p ? fallback : value;
}

// --------------------------------------------------------------- lifecycle

void EspFleetClass::resolveIdentity() {
  uint8_t mac[6] = {0};
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  snprintf(device_id_, sizeof(device_id_), "%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  if (name_buf_[0] == 0) {
    snprintf(name_buf_, sizeof(name_buf_), "espfleet-%02x%02x%02x", mac[3], mac[4], mac[5]);
  }

  nvs_handle_t nvs;
  if (nvs_open("espfleet", NVS_READWRITE, &nvs) == ESP_OK) {
    nvs_get_u32(nvs, "boots", &boot_count_);
    ++boot_count_;
    nvs_set_u32(nvs, "boots", boot_count_);
    nvs_commit(nvs);
    nvs_close(nvs);
  }
}

bool EspFleetClass::begin(const char* hub_url) {
  EspFleetConfig config;
  if (hub_url && hub_url[0]) config.hub = hub_url;
  return begin(config);
}

bool EspFleetClass::begin(const EspFleetConfig& config) {
  if (running_) return true;

  // Setters may have been called before begin(); those win over build flags.
  const uint16_t user_web_port = cfg_.web_port;
  const bool user_web_set = web_set_;
  const uint32_t user_interval = interval_set_ ? cfg_.interval_ms : 0;

  cfg_ = config;
  if (user_web_set) cfg_.web_port = user_web_port;
  if (user_interval) cfg_.interval_ms = user_interval;

  if ((!cfg_.hub || !cfg_.hub[0]) && !cfg_.enable_mdns) {
    ESP_LOGE(TAG, "no hub URL: pass one to begin(), define ESPFLEET_HUB, or enable mDNS");
    return false;
  }
  if (name_buf_[0] == 0) copyTrim(name_buf_, sizeof(name_buf_), cfg_.name);

  if (project_buf_[0] == 0) copyTrim(project_buf_, sizeof(project_buf_), cfg_.project);
  if (version_buf_[0] == 0) copyTrim(version_buf_, sizeof(version_buf_), cfg_.version);
  if (web_path_buf_[0] == 0) copyTrim(web_path_buf_, sizeof(web_path_buf_), cfg_.web_path);
  if (version_buf_[0] == 0) {
    const esp_app_desc_t* desc = esp_app_get_description();
    if (desc) copyTrim(version_buf_, sizeof(version_buf_), desc->version);
  }

  if (cfg_.hub && cfg_.hub[0]) {
    copyTrim(hub_url_, sizeof(hub_url_), cfg_.hub);
  }

  if (!log_lock_) {
    log_lock_ = xSemaphoreCreateMutex();
    s_log_ring.init(s_log_buf, sizeof(s_log_buf));
    s_prev_vprintf = esp_log_set_vprintf(&EspFleetClass::vprintfHook);
  }

  resolveIdentity();
  running_ = true;
  next_beat_us_ = 0;
  next_log_ship_us_ = 0;
  next_health_us_ = 0;

  // Claiming OTA support on a single-app partition table would let the hub
  // queue an update that can only ever fail, so verify a slot actually exists.
  if (cfg_.enable_ota && esp_ota_get_next_update_partition(nullptr) == nullptr) {
    cfg_.enable_ota = false;
    ESP_LOGW(TAG, "no OTA partition found, reporting updates as unsupported "
                  "(use a two-app partition table to enable them)");
  }

  if (cfg_.info_port) startInfoServer();

  ESP_LOGI(TAG, "agent %s up: id=%s name=%s project=%s version=%s hub=%s",
           ESPFLEET_AGENT_VERSION, device_id_, name_buf_,
           project_buf_[0] ? project_buf_ : "-", version_buf_[0] ? version_buf_ : "-", hub_url_[0] ? hub_url_ : "-");

  esp_reset_reason_t reason = esp_reset_reason();
  bool abnormal = (reason == ESP_RST_PANIC || reason == ESP_RST_INT_WDT ||
                   reason == ESP_RST_TASK_WDT || reason == ESP_RST_WDT ||
                   reason == ESP_RST_BROWNOUT);
  if (abnormal) {
    ESP_LOGE(TAG, "abnormal reset: %s (code %d)", resetReasonName(), (int)reason);
    char crash_msg[128];
    snprintf(crash_msg, sizeof(crash_msg), "Unexpected reset: %s (code %d)", resetReasonName(), (int)reason);
    pushLog('E', "crash", crash_msg);
  }

  if (cfg_.spawn_task) {
    TaskHandle_t handle = nullptr;
    // NOTE: on ESP-IDF the stack depth argument is in BYTES, not words.
    if (xTaskCreate(&EspFleetClass::taskEntry, "espfleet", ESPFLEET_STACK_SIZE,
                    this, ESPFLEET_TASK_PRIORITY, &handle) != pdPASS) {
      ESP_LOGE(TAG, "could not start agent task");
      running_ = false;
      return false;
    }
    task_ = handle;
  }
  return true;
}

void EspFleetClass::taskEntry(void* arg) {
  auto* self = static_cast<EspFleetClass*>(arg);
  while (true) {
    self->loop();
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void EspFleetClass::end() {
  running_ = false;
  if (s_prev_vprintf) {
    esp_log_set_vprintf(s_prev_vprintf);
    s_prev_vprintf = nullptr;
  }
  if (task_) {
    vTaskDelete(static_cast<TaskHandle_t>(task_));
    task_ = nullptr;
  }
  if (info_sock_ >= 0) {
    close(info_sock_);
    info_sock_ = -1;
  }
  if (log_lock_) {
    vSemaphoreDelete(static_cast<SemaphoreHandle_t>(log_lock_));
    log_lock_ = nullptr;
  }
}

void EspFleetClass::setName(const char* name) { copyTrim(name_buf_, sizeof(name_buf_), name); }
void EspFleetClass::setProject(const char* slug) { copyTrim(project_buf_, sizeof(project_buf_), slug); }
void EspFleetClass::setVersion(const char* version) { copyTrim(version_buf_, sizeof(version_buf_), version); }

void EspFleetClass::setInterval(uint32_t ms) {
  cfg_.interval_ms = ms < 5000 ? 5000 : ms;
  interval_set_ = true;
}

void EspFleetClass::setWebUi(uint16_t port, const char* path) {
  cfg_.web_port = port;
  web_set_ = true;
  copyTrim(web_path_buf_, sizeof(web_path_buf_), path && path[0] ? path : "/");
}

void EspFleetClass::setExtra(const char* key, const char* value) {
  if (!key || !value) return;
  for (uint8_t i = 0; i < extra_count_; ++i) {
    if (strncmp(extras_[i].key, key, sizeof(extras_[i].key)) == 0) {
      copyTrim(extras_[i].value, sizeof(extras_[i].value), value);
      return;
    }
  }
  if (extra_count_ >= ESPFLEET_MAX_EXTRA) return;
  copyTrim(extras_[extra_count_].key, sizeof(extras_[extra_count_].key), key);
  copyTrim(extras_[extra_count_].value, sizeof(extras_[extra_count_].value), value);
  ++extra_count_;
}

void EspFleetClass::setExtra(const char* key, long value) {
  char buffer[16];
  snprintf(buffer, sizeof(buffer), "%ld", value);
  setExtra(key, buffer);
}

void EspFleetClass::setMetric(const char* key, double value) {
  if (!key || !key[0]) return;
  for (uint8_t i = 0; i < metric_count_; ++i) {
    if (strcmp(metrics_[i].key, key) == 0) {
      metrics_[i].value = value;
      metrics_[i].is_int = false;
      return;
    }
  }
  if (metric_count_ >= ESPFLEET_MAX_METRICS) return;
  copyTrim(metrics_[metric_count_].key, sizeof(metrics_[metric_count_].key), key);
  metrics_[metric_count_].value = value;
  metrics_[metric_count_].is_int = false;
  ++metric_count_;
}

void EspFleetClass::setMetric(const char* key, int64_t value) {
  if (!key || !key[0]) return;
  for (uint8_t i = 0; i < metric_count_; ++i) {
    if (strcmp(metrics_[i].key, key) == 0) {
      metrics_[i].int_val = value;
      metrics_[i].value = (double)value;
      metrics_[i].is_int = true;
      return;
    }
  }
  if (metric_count_ >= ESPFLEET_MAX_METRICS) return;
  copyTrim(metrics_[metric_count_].key, sizeof(metrics_[metric_count_].key), key);
  metrics_[metric_count_].int_val = value;
  metrics_[metric_count_].value = (double)value;
  metrics_[metric_count_].is_int = true;
  ++metric_count_;
}

// ---------------------------------------------------------------- payload

size_t EspFleetClass::infoJson(char* out, size_t out_len) const {
  char ip[16] = {0};
  currentIp(ip, sizeof(ip));

  wifi_ap_record_t ap = {};
  bool have_ap = esp_wifi_sta_get_ap_info(&ap) == ESP_OK;

  esp_chip_info_t chip = {};
  esp_chip_info(&chip);

  uint32_t flash_size = 0;
  esp_flash_get_size(nullptr, &flash_size);

  uint8_t mac[6] = {0};
  esp_read_mac(mac, ESP_MAC_WIFI_STA);

  size_t heap_total = heap_caps_get_total_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  size_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);

  const esp_app_desc_t* desc = esp_app_get_description();
  const char* fw_name = desc ? desc->project_name : "";

  int n = snprintf(
      out, out_len,
      "{\"id\":\"%s\",\"name\":\"%s\",\"mac\":\"%02x:%02x:%02x:%02x:%02x:%02x\",\"ip\":\"%s\","
      "\"chip\":\"%s\",\"chip_rev\":\"%d.%d\",\"project\":\"%s\",\"fw\":\"%s\",\"version\":\"%s\","
      "\"commit\":\"%s\",\"branch\":\"%s\",\"dirty\":%d,\"built\":\"%s\",\"sdk\":\"%s\",\"agent\":\"%s\","
      "\"web_port\":%u,\"web_path\":\"%s\",\"ota\":%d,\"flash\":%u,\"heap\":%u,\"free\":%u,"
      "\"minfree\":%u,\"psram\":%u,\"up\":%lld,\"rssi\":%d,\"ssid\":\"%s\",\"reset\":\"%s\","
      "\"boots\":%u,\"hb\":%u",
      device_id_, name_buf_, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], ip,
      CONFIG_IDF_TARGET, chip.revision / 100, chip.revision % 100, project_buf_,
      fw_name, version_buf_,
      ESPFLEET_GIT_COMMIT, ESPFLEET_GIT_BRANCH, ESPFLEET_GIT_DIRTY ? 1 : 0,
      ESPFLEET_BUILD_TIME, esp_get_idf_version(), ESPFLEET_AGENT_VERSION,
      (unsigned)cfg_.web_port, web_path_buf_, cfg_.enable_ota ? 1 : 0,
      (unsigned)flash_size, (unsigned)heap_total, (unsigned)esp_get_free_heap_size(),
      (unsigned)esp_get_minimum_free_heap_size(), (unsigned)psram_free,
      (long long)(esp_timer_get_time() / 1000000), have_ap ? ap.rssi : 0,
      have_ap ? (const char*)ap.ssid : "", resetReasonName(),
      (unsigned)boot_count_, (unsigned)(cfg_.interval_ms / 1000));

  if (n < 0 || (size_t)n >= out_len) return 0;
  size_t used = (size_t)n;

  if (ota_result_[0]) used = appendf(out, out_len, used, ",\"ota_result\":\"%s\"", ota_result_);

  if (extra_count_) {
    used = appendf(out, out_len, used, ",\"extra\":{");
    for (uint8_t i = 0; i < extra_count_; ++i) {
      used = appendf(out, out_len, used, "%s\"%s\":\"", i ? "," : "", extras_[i].key);
      used += appendEscaped(out + used, out_len - used, extras_[i].value);
      used = appendf(out, out_len, used, "\"");
    }
    used = appendf(out, out_len, used, "}");
  }

  if (metric_count_) {
    used = appendf(out, out_len, used, ",\"metrics\":{");
    for (uint8_t i = 0; i < metric_count_; ++i) {
      if (metrics_[i].is_int) {
        used = appendf(out, out_len, used, "%s\"%s\":%lld", i ? "," : "", metrics_[i].key, (long long)metrics_[i].int_val);
      } else {
        used = appendf(out, out_len, used, "%s\"%s\":%.3f", i ? "," : "", metrics_[i].key, metrics_[i].value);
      }
    }
    used = appendf(out, out_len, used, "}");
  }

  if (used + 2 >= out_len) return 0;
  out[used++] = '}';
  out[used] = 0;
  return used;
}

// -------------------------------------------------------------- heartbeat

bool EspFleetClass::sendHeartbeat() {
  if (!hub_url_[0]) return false;  // hub not discovered/configured yet
  char ip[16];
  if (!currentIp(ip, sizeof(ip))) return false;  // no network yet

  // VENDORED PATCH: body on the heap (many metrics/extras do not fit the old 896 byte stack buffer).
  char* body = static_cast<char*>(malloc(ESPFLEET_BODY_MAX));
  if (!body) return false;
  size_t len = infoJson(body, ESPFLEET_BODY_MAX);
  if (!len) {
    ESP_LOGW(TAG, "status payload did not fit");
    free(body);
    return false;
  }

  char url[192];
  snprintf(url, sizeof(url), "%s/api/v1/agent/heartbeat", hub_url_);

  esp_http_client_config_t http = {};
  http.url = url;
  http.method = HTTP_METHOD_POST;
  http.timeout_ms = 6000;
  http.buffer_size = 512;
  http.buffer_size_tx = 512;
  http.keep_alive_enable = false;

  esp_http_client_handle_t client = esp_http_client_init(&http);
  if (!client) {
    free(body);
    return false;
  }

  bool ok = false;
  char response[320] = {0};
  esp_http_client_set_header(client, "Content-Type", "application/json");

  if (esp_http_client_open(client, len) == ESP_OK) {
    if (esp_http_client_write(client, body, len) == (int)len) {
      esp_http_client_fetch_headers(client);
      int status = esp_http_client_get_status_code(client);
      int read = esp_http_client_read_response(client, response, sizeof(response) - 1);
      if (read > 0) response[read] = 0;
      ok = status == 200;
      if (!ok) ESP_LOGW(TAG, "hub replied %d", status);
    }
  }
  esp_http_client_cleanup(client);
  free(body);

  if (ok) {
    ++heartbeats_;
    failures_ = 0;
    if (!online_) ESP_LOGI(TAG, "hub reachable at %s", hub_url_);
    online_ = true;
    if (!image_confirmed_) confirmOtaImage();
    if (!mdns_done_ && cfg_.enable_mdns) advertiseMdns();

    long hb = jsonInt(response, "hb", 0);
    if (hb >= 5 && (uint32_t)hb * 1000 != cfg_.interval_ms) setInterval((uint32_t)hb * 1000);
    handleCommand(response);
  } else {
    online_ = false;
    if (++failures_ % 10 == 1) ESP_LOGW(TAG, "heartbeat failed (%lu in a row)", (unsigned long)failures_);
  }
  return ok;
}

void EspFleetClass::handleCommand(const char* body) {
  char cmd[24] = {0};
  if (!jsonStr(body, "cmd", cmd, sizeof(cmd)) || strcmp(cmd, "none") == 0) return;

  long cid = jsonInt(body, "cid", -1);
  char arg[128] = {0};
  jsonStr(body, "arg", arg, sizeof(arg));
  ESP_LOGI(TAG, "command from hub: %s (cid %ld)", cmd, cid);

  if (strcmp(cmd, "ota") == 0) {
    if (!cfg_.enable_ota) {
      reportResult(cid, false, "OTA disabled in this build");
      return;
    }
    char url[192] = {0};
    char sha[72] = {0};
    if (!jsonStr(body, "url", url, sizeof(url))) {
      reportResult(cid, false, "no url in command");
      return;
    }
    jsonStr(body, "sha256", sha, sizeof(sha));
    performOta(url, sha, cid);
    return;
  }

  if (strcmp(cmd, "ping") == 0) {
    reportResult(cid, true, "pong");
    return;
  }

  bool handled = command_cb_ ? command_cb_(cmd, arg) : false;

  if (strcmp(cmd, "reboot") == 0) {
    if (!gate("reboot")) {
      reportResult(cid, false, "refused: device busy");
      return;
    }
    reportResult(cid, true, "rebooting");
    vTaskDelay(pdMS_TO_TICKS(400));
    esp_restart();
  }
  reportResult(cid, handled, handled ? "ok" : "unsupported command");
}

void EspFleetClass::reportResult(long cid, bool ok, const char* message) {
  if (cid < 0 || !hub_url_[0]) return;
  char url[192];
  snprintf(url, sizeof(url), "%s/api/v1/agent/result", hub_url_);

  char body[224];
  int len = snprintf(body, sizeof(body), "{\"cid\":%ld,\"status\":\"%s\",\"msg\":\"%s\"}",
                     cid, ok ? "ok" : "failed", message ? message : "");

  esp_http_client_config_t http = {};
  http.url = url;
  http.method = HTTP_METHOD_POST;
  http.timeout_ms = 5000;
  http.buffer_size = 256;
  http.buffer_size_tx = 256;

  esp_http_client_handle_t client = esp_http_client_init(&http);
  if (!client) return;
  esp_http_client_set_header(client, "Content-Type", "application/json");
  if (esp_http_client_open(client, len) == ESP_OK) {
    esp_http_client_write(client, body, len);
    esp_http_client_fetch_headers(client);
  }
  esp_http_client_cleanup(client);
}

// -------------------------------------------------------------------- OTA

bool EspFleetClass::performOta(const char* url, const char* expected_sha, long cid) {
  const esp_partition_t* target = esp_ota_get_next_update_partition(nullptr);
  if (!target) {
    reportResult(cid, false, "no OTA partition (single-app partition table?)");
    return false;
  }
  if (!gate("ota")) {  // VENDORED PATCH: let the application quiesce (or refuse) before flash writes
    reportResult(cid, false, "refused: device busy");
    return false;
  }
  ESP_LOGI(TAG, "OTA from %s into %s", url, target->label);

  esp_http_client_config_t http = {};
  http.url = url;
  http.method = HTTP_METHOD_GET;
  http.timeout_ms = 15000;
  http.buffer_size = 1024;
  http.keep_alive_enable = true;

  esp_http_client_handle_t client = esp_http_client_init(&http);
  if (!client) {
    reportResult(cid, false, "http init failed");
    return false;
  }

  char message[64] = {0};
  bool ok = false;
  esp_ota_handle_t ota = 0;
  bool ota_open = false;
  mbedtls_sha256_context sha_ctx;
  mbedtls_sha256_init(&sha_ctx);
  mbedtls_sha256_starts(&sha_ctx, 0);

  do {
    if (esp_http_client_open(client, 0) != ESP_OK) {
      snprintf(message, sizeof(message), "cannot reach firmware url");
      break;
    }
    int64_t total = esp_http_client_fetch_headers(client);
    if (esp_http_client_get_status_code(client) != 200) {
      snprintf(message, sizeof(message), "http %d", esp_http_client_get_status_code(client));
      break;
    }
    if (total > 0 && (size_t)total > target->size) {
      snprintf(message, sizeof(message), "image too large for partition");
      break;
    }
    if (esp_ota_begin(target, OTA_SIZE_UNKNOWN, &ota) != ESP_OK) {
      snprintf(message, sizeof(message), "esp_ota_begin failed");
      break;
    }
    ota_open = true;

    static uint8_t chunk[1460];
    size_t written = 0;
    bool checked_magic = false;
    while (true) {
      int n = esp_http_client_read(client, (char*)chunk, sizeof(chunk));
      if (n < 0) {
        snprintf(message, sizeof(message), "socket error after %u bytes", (unsigned)written);
        break;
      }
      if (n == 0) break;
      if (!checked_magic) {
        if (chunk[0] != 0xE9) {
          snprintf(message, sizeof(message), "not an ESP app image");
          break;
        }
        checked_magic = true;
      }
      mbedtls_sha256_update(&sha_ctx, chunk, n);
      if (esp_ota_write(ota, chunk, n) != ESP_OK) {
        snprintf(message, sizeof(message), "flash write failed at %u", (unsigned)written);
        break;
      }
      written += n;
    }
    if (message[0]) break;
    if (total > 0 && written != (size_t)total) {
      snprintf(message, sizeof(message), "short read %u/%lld", (unsigned)written, (long long)total);
      break;
    }

    unsigned char digest[32];
    mbedtls_sha256_finish(&sha_ctx, digest);
    if (expected_sha && strlen(expected_sha) == 64) {
      char hex[65];
      for (int i = 0; i < 32; ++i) snprintf(hex + i * 2, 3, "%02x", digest[i]);
      if (strcasecmp(hex, expected_sha) != 0) {
        snprintf(message, sizeof(message), "sha256 mismatch");
        break;
      }
    }

    if (esp_ota_end(ota) != ESP_OK) {
      ota_open = false;
      snprintf(message, sizeof(message), "image validation failed");
      break;
    }
    ota_open = false;
    if (esp_ota_set_boot_partition(target) != ESP_OK) {
      snprintf(message, sizeof(message), "set_boot_partition failed");
      break;
    }
    snprintf(message, sizeof(message), "flashed %u bytes, rebooting", (unsigned)written);
    ok = true;
  } while (false);

  if (ota_open) esp_ota_abort(ota);
  mbedtls_sha256_free(&sha_ctx);
  esp_http_client_cleanup(client);

  copyTrim(ota_result_, sizeof(ota_result_), ok ? "ok" : message);
  reportResult(cid, ok, message);
  ESP_LOGI(TAG, "OTA %s: %s", ok ? "succeeded" : "failed", message);

  if (ok) {
    vTaskDelay(pdMS_TO_TICKS(600));
    gate("restart");
    esp_restart();
  }
  return ok;
}

void EspFleetClass::confirmOtaImage() {
  image_confirmed_ = true;
  const esp_partition_t* running = esp_ota_get_running_partition();
  if (!running) return;
  esp_ota_img_states_t state;
  if (esp_ota_get_state_partition(running, &state) != ESP_OK) return;
  if (state == ESP_OTA_IMG_PENDING_VERIFY) {
    esp_ota_mark_app_valid_cancel_rollback();
    ESP_LOGI(TAG, "new image confirmed, rollback cancelled");
    copyTrim(ota_result_, sizeof(ota_result_), "confirmed");
  }
}

// ------------------------------------------------------------------- mDNS
bool EspFleetClass::discoverHub(char* url_out, size_t cap) {
#ifdef ESPFLEET_HAVE_MDNS
  esp_err_t err = mdns_init();
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return false;

  mdns_result_t* res = nullptr;
  if (mdns_query_ptr("_espfleet", "_tcp", 2000, 8, &res) != ESP_OK || !res) return false;
  bool found = false;
  for (mdns_result_t* r = res; r && !found; r = r->next) {
    bool has_api = false, has_id = false;
    for (size_t k = 0; k < r->txt_count; ++k) {
      if (strcmp(r->txt[k].key, "api") == 0) has_api = true;
      if (strcmp(r->txt[k].key, "id") == 0) has_id = true;
    }
    if (!has_api || has_id) continue;
    for (mdns_ip_addr_t* a = r->addr; a; a = a->next) {
      if (a->addr.type != ESP_IPADDR_TYPE_V4) continue;
      snprintf(url_out, cap, "http://" IPSTR ":%u", IP2STR(&a->addr.u_addr.ip4), (unsigned)r->port);
      found = true;
      break;
    }
  }
  mdns_query_results_free(res);
  return found;
#else
  (void)url_out;
  (void)cap;
  return false;
#endif
}

void EspFleetClass::advertiseMdns() {
  mdns_done_ = true;
#ifdef ESPFLEET_HAVE_MDNS
  esp_err_t err = mdns_init();
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return;

  mdns_txt_item_t txt[] = {
      {"id", device_id_},
      {"project", project_buf_},
      {"version", version_buf_},
      {"agent", ESPFLEET_AGENT_VERSION},
  };
  uint16_t port = cfg_.info_port ? cfg_.info_port : (cfg_.web_port ? cfg_.web_port : 80);
  mdns_service_add(nullptr, "_espfleet", "_tcp", port, txt, sizeof(txt) / sizeof(txt[0]));
  ESP_LOGI(TAG, "advertising _espfleet._tcp on port %u", (unsigned)port);
#endif
}

// ------------------------------------------------------------- log shipper
void EspFleetClass::pushLog(char level, const char* tag, const char* msg) {
  if (!log_lock_ || !msg) return;
  auto* lock = static_cast<SemaphoreHandle_t>(log_lock_);
  if (xSemaphoreTake(lock, 0) != pdTRUE) {
    ++logs_dropped_;
    return;
  }

  size_t tl = tag ? strlen(tag) : 0;
  if (tl > LOG_TAG_MAX) tl = LOG_TAG_MAX;
  size_t ml = strlen(msg);
  if (ml > LOG_MSG_MAX) ml = LOG_MSG_MAX;

  uint32_t ms = (uint32_t)(esp_timer_get_time() / 1000LL);
  uint8_t rec[9 + LOG_TAG_MAX + LOG_MSG_MAX];
  rec[0] = (uint8_t)ms; rec[1] = (uint8_t)(ms >> 8); rec[2] = (uint8_t)(ms >> 16); rec[3] = (uint8_t)(ms >> 24);
  rec[4] = 0; // source: default s3/c3 (will be serialized with CONFIG_IDF_TARGET or s3)
  rec[5] = (uint8_t)level;
  rec[6] = (uint8_t)tl;
  rec[7] = (uint8_t)ml; rec[8] = (uint8_t)(ml >> 8);
  if (tl && tag) memcpy(rec + 9, tag, tl);
  if (ml) memcpy(rec + 9 + tl, msg, ml);

  s_log_ring.push(rec, 9 + tl + ml);
  xSemaphoreGive(lock);
}

void EspFleetClass::shipLogs() {
  if (!log_lock_ || !hub_url_[0] || !online_) return;
  auto* lock = static_cast<SemaphoreHandle_t>(log_lock_);
  if (xSemaphoreTake(lock, pdMS_TO_TICKS(20)) != pdTRUE) return;
  if (!s_log_ring.count) {
    xSemaphoreGive(lock);
    return;
  }

  char* body = static_cast<char*>(malloc(4096));
  if (!body) {
    xSemaphoreGive(lock);
    return;
  }

  size_t used = 0;
  body[used++] = '[';

  uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000LL);
  time_t now_wall = 0;
  time(&now_wall);

  uint8_t rec[9 + LOG_TAG_MAX + LOG_MSG_MAX];
  uint32_t n = 0;
  const char* src_name = "s3";
#if defined(CONFIG_IDF_TARGET_ESP32C6) || defined(CONFIG_IDF_TARGET_ESP32C3)
  src_name = "c6";
#endif

  for (uint32_t k = 0; k < s_log_ring.count && n < LOG_MAX_BATCH; ++k) {
    int len = s_log_ring.peek(k, rec, sizeof(rec));
    if (len < 9) break;
    uint32_t ms = (uint32_t)rec[0] | (uint32_t)rec[1] << 8 | (uint32_t)rec[2] << 16 | (uint32_t)rec[3] << 24;
    char lvl = (char)rec[5];
    uint8_t tl = rec[6];
    uint16_t ml = (uint16_t)(rec[7] | rec[8] << 8);

    char tag_buf[LOG_TAG_MAX + 1] = {0};
    if (tl) memcpy(tag_buf, rec + 9, tl);

    char msg_buf[LOG_MSG_MAX + 1] = {0};
    if (ml) memcpy(msg_buf, rec + 9 + tl, ml);

    double ts = now_wall ? (double)now_wall - (double)(now_ms - ms) / 1000.0 : (double)ms / 1000.0;

    char entry[640];
    char escaped[512];
    appendEscaped(escaped, sizeof(escaped), msg_buf);
    int elen = snprintf(entry, sizeof(entry),
                        "%s{\"ts\":%.3f,\"source\":\"%s\",\"level\":\"%c\",\"tag\":\"%s\",\"msg\":\"%s\"}",
                        n ? "," : "", ts, src_name, lvl, tag_buf, escaped);
    if (elen < 0 || used + elen + 2 >= 4096) break;
    memcpy(body + used, entry, elen);
    used += elen;
    n++;
  }
  xSemaphoreGive(lock);

  if (!n) {
    free(body);
    return;
  }
  body[used++] = ']';
  body[used] = '\0';

  char url[192];
  snprintf(url, sizeof(url), "%s/api/v1/ingest/logs", hub_url_);

  esp_http_client_config_t http = {};
  http.url = url;
  http.method = HTTP_METHOD_POST;
  http.timeout_ms = 4000;
  http.buffer_size = 512;
  http.buffer_size_tx = 512;

  esp_http_client_handle_t client = esp_http_client_init(&http);
  if (!client) {
    free(body);
    return;
  }
  esp_http_client_set_header(client, "Content-Type", "application/json");
  esp_http_client_set_header(client, "X-Device-Id", device_id_);

  bool ok = false;
  if (esp_http_client_open(client, used) == ESP_OK) {
    if (esp_http_client_write(client, body, used) == (int)used) {
      esp_http_client_fetch_headers(client);
      int status = esp_http_client_get_status_code(client);
      ok = (status == 200);
    }
  }
  esp_http_client_cleanup(client);
  free(body);

  if (ok && xSemaphoreTake(lock, pdMS_TO_TICKS(20)) == pdTRUE) {
    s_log_ring.drop(n);
    xSemaphoreGive(lock);
  }
}


// ----------------------------------------------------------- health reporting
void EspFleetClass::emitHealth() {
  time_t wall = 0;
  time(&wall);
  uint32_t up_ms = (uint32_t)(esp_timer_get_time() / 1000LL);
  size_t int_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  size_t int_min = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  size_t int_big = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  size_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
  size_t psram_min = heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM);
  size_t psram_big = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);

  const esp_app_desc_t* desc = esp_app_get_description();
  const char* fw = desc ? desc->version : version_buf_;
  const char* idf = esp_get_idf_version();
  char elf[17] = {0};
  const char* elf_full = esp_app_get_elf_sha256_str();
  if (elf_full) snprintf(elf, sizeof(elf), "%.16s", elf_full);

  uint32_t seq = ++health_seq_;
  const char* trig = (seq == 1) ? "boot" : "hourly";

  esp_reset_reason_t reason = esp_reset_reason();
  bool abnormal = (reason == ESP_RST_PANIC || reason == ESP_RST_INT_WDT ||
                   reason == ESP_RST_TASK_WDT || reason == ESP_RST_WDT ||
                   reason == ESP_RST_BROWNOUT);

  char custom_buf[ESPFLEET_HEALTH_CUSTOM_MAX] = {0};
  if (metric_count_) {
    size_t c_used = snprintf(custom_buf, sizeof(custom_buf), ",\"custom\":{");
    for (uint8_t i = 0; i < metric_count_ && c_used + 40 < sizeof(custom_buf); ++i) {
      if (metrics_[i].is_int) {
        c_used += snprintf(custom_buf + c_used, sizeof(custom_buf) - c_used, "%s\"%s\":%lld",
                           i ? "," : "", metrics_[i].key, (long long)metrics_[i].int_val);
      } else {
        c_used += snprintf(custom_buf + c_used, sizeof(custom_buf) - c_used, "%s\"%s\":%.3f",
                           i ? "," : "", metrics_[i].key, metrics_[i].value);
      }
    }
    if (c_used + 2 < sizeof(custom_buf)) {
      custom_buf[c_used++] = '}';
      custom_buf[c_used] = 0;
    }
  }

  char stacks_ext[160] = {0};
  char cnt_ext[160] = {0};
  if (s_health_cb) s_health_cb(stacks_ext, sizeof(stacks_ext), cnt_ext, sizeof(cnt_ext));

  char json[ESPFLEET_HEALTH_JSON_MAX];
  snprintf(json, sizeof(json),
           "{\"kind\":\"health\",\"v\":1,\"seq\":%u,\"trig\":\"%s\",\"ts\":%u,\"up_ms\":%u,"
           "\"dev\":\"%s\",\"fw\":\"%s\",\"elf\":\"%s\",\"idf\":\"%s\",\"boot\":%u,"
           "\"reset\":{\"reason\":\"%s\",\"code\":%u,\"abnormal\":%s},"
           "\"heap\":{\"int\":{\"free\":%u,\"min\":%u,\"big\":%u},\"psram\":{\"free\":%u,\"min\":%u,\"big\":%u}},"
           "\"stacks\":{\"espfleet\":%u%s},\"cnt\":{\"coredump\":0,"
           "\"hub_fail\":%u,\"log_drop\":%u%s}%s}",
           (unsigned)seq, trig, (unsigned)wall, (unsigned)up_ms,
           device_id_, fw, elf, idf, (unsigned)boot_count_,
           resetReasonName(), (unsigned)reason, abnormal ? "true" : "false",
           (unsigned)int_free, (unsigned)int_min, (unsigned)int_big,
           (unsigned)psram_free, (unsigned)psram_min, (unsigned)psram_big,
           task_ ? (unsigned)uxTaskGetStackHighWaterMark(static_cast<TaskHandle_t>(task_)) : 0,
           stacks_ext, (unsigned)failures_, (unsigned)logs_dropped_,
           s_health_cb ? cnt_ext : ",\"wifi_reconn\":0", custom_buf);

  // Chunk into 360-byte payload parts for HEALTH line format
  size_t jlen = strlen(json);
  size_t chunk_size = 360;
  size_t parts = (jlen + chunk_size - 1) / chunk_size;
  if (!parts) parts = 1;

  for (size_t i = 0; i < parts; ++i) {
    size_t off = i * chunk_size;
    size_t n = jlen - off < chunk_size ? jlen - off : chunk_size;
    char line[440];
    int pre = snprintf(line, sizeof(line), "HEALTH %u %u %u/%u ", (unsigned)boot_count_, (unsigned)seq, (unsigned)(i + 1), (unsigned)parts);
    if (pre > 0 && pre + n < sizeof(line)) {
      memcpy(line + pre, json + off, n);
      line[pre + n] = '\0';
      pushLog('I', "health", line);
    }
  }
}

void EspFleetClass::checkHealthReport() {
  int64_t now_us = esp_timer_get_time();
  if (!next_health_us_) {
    // Schedule first health report 10s after link is online, then every hour
    next_health_us_ = now_us + 10000000LL;
    return;
  }
  if (now_us >= next_health_us_) {
    emitHealth();
    next_health_us_ = now_us + 3600000000LL; // every 1 hour
  }
}

// ------------------------------------------------------- optional info HTTP

void EspFleetClass::startInfoServer() {
  info_sock_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (info_sock_ < 0) return;

  int reuse = 1;
  setsockopt(info_sock_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

  struct sockaddr_in addr = {};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(cfg_.info_port);

  if (bind(info_sock_, (struct sockaddr*)&addr, sizeof(addr)) != 0 || listen(info_sock_, 1) != 0) {
    ESP_LOGW(TAG, "cannot listen on port %u", (unsigned)cfg_.info_port);
    close(info_sock_);
    info_sock_ = -1;
    return;
  }
  fcntl(info_sock_, F_SETFL, O_NONBLOCK);
  ESP_LOGI(TAG, "serving /fleet/info on port %u", (unsigned)cfg_.info_port);
}

void EspFleetClass::serviceInfoServer() {
  if (info_sock_ < 0) return;
  int client = accept(info_sock_, nullptr, nullptr);
  if (client < 0) return;

  struct timeval tv;
  tv.tv_sec = 1;
  tv.tv_usec = 0;
  setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  char request[128];
  int n = recv(client, request, sizeof(request) - 1, 0);
  if (n > 0) {
    request[n] = 0;
    char payload[896];
    size_t len = infoJson(payload, sizeof(payload));
    char header[128];
    int hl = snprintf(header, sizeof(header),
                      "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n"
                      "Access-Control-Allow-Origin: *\r\nContent-Length: %u\r\nConnection: close\r\n\r\n",
                      (unsigned)len);
    send(client, header, hl, 0);
    send(client, payload, len, 0);
  }
  close(client);
}

// -------------------------------------------------------------------- loop

void EspFleetClass::loop() {
  if (!running_) return;
  serviceInfoServer();

  int64_t now = esp_timer_get_time();

  // If hub URL is not known or empty, try mDNS discovery
  if (!hub_url_[0] && cfg_.enable_mdns && now >= s_next_discover_us) {
    s_next_discover_us = now + 30000000LL;  // VENDORED PATCH: a 2 s mDNS query every 200 ms loop is too chatty
    char discovered[128] = {0};
    if (discoverHub(discovered, sizeof(discovered))) {
      copyTrim(hub_url_, sizeof(hub_url_), discovered);
      ESP_LOGI(TAG, "hub %s (mdns)", hub_url_);
    }
  }

  if (online_) {
    checkHealthReport();
    if (now >= next_log_ship_us_) {
      shipLogs();
      next_log_ship_us_ = now + ((int64_t)LOG_SHIP_INTERVAL_MS * 1000);
    }
  }

  if (!poke_ && now < next_beat_us_) return;
  poke_ = false;

  bool ok = sendHeartbeat();
  // Back off gently while the hub or the network is unreachable.
  uint32_t interval = cfg_.interval_ms;
  if (!ok) interval = failures_ > 5 ? cfg_.interval_ms * 2 : 5000;
  next_beat_us_ = esp_timer_get_time() + (int64_t)interval * 1000;
}

// VENDORED PATCH (WaveMaster): setters for the application hooks.
void espfleet_gate_set_impl(espfleet_gate_cb cb) { s_gate_cb = cb; }
void espfleet_health_set_impl(espfleet_health_cb cb) { s_health_cb = cb; }
