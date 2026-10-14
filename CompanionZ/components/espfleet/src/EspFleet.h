// ESPFleet agent - a tiny fleet-management client for ESP32 family chips.
//
// Framework agnostic: uses only ESP-IDF APIs, so it builds under both
// framework = arduino and framework = espidf, on esp32, esp32s2, esp32s3,
// esp32c3, esp32c5 and esp32c6.
//
// Cost at rest: one task (6 KB stack by default, priority 1), ~1.2 KB of
// static state and one short HTTP POST every ESPFLEET_INTERVAL_MS.

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdarg.h>


// --------------------------------------------------- ESP-IDF menuconfig bridge
//
// Under bare ESP-IDF the options come from Kconfig. Explicit -D flags still win,
// which is how the PlatformIO build script injects the git identity.

#ifdef CONFIG_ESPFLEET_HUB_URL
#  ifndef ESPFLEET_HUB
#    define ESPFLEET_HUB CONFIG_ESPFLEET_HUB_URL
#  endif
#endif
#ifdef CONFIG_ESPFLEET_PROJECT_SLUG
#  ifndef ESPFLEET_PROJECT
#    define ESPFLEET_PROJECT CONFIG_ESPFLEET_PROJECT_SLUG
#  endif
#endif
#ifdef CONFIG_ESPFLEET_DEVICE_NAME
#  ifndef ESPFLEET_NAME
#    define ESPFLEET_NAME CONFIG_ESPFLEET_DEVICE_NAME
#  endif
#endif
#ifdef CONFIG_ESPFLEET_HEARTBEAT_MS
#  ifndef ESPFLEET_INTERVAL_MS
#    define ESPFLEET_INTERVAL_MS CONFIG_ESPFLEET_HEARTBEAT_MS
#  endif
#endif
#ifndef ESPFLEET_ENABLE_OTA
#  ifdef CONFIG_ESPFLEET_OTA
#    define ESPFLEET_ENABLE_OTA 1
#  elif defined(CONFIG_ESPFLEET_HUB_URL)
#    define ESPFLEET_ENABLE_OTA 0
#  endif
#endif
#ifndef ESPFLEET_ENABLE_MDNS
#  ifdef CONFIG_ESPFLEET_MDNS
#    define ESPFLEET_ENABLE_MDNS 1
#  elif defined(CONFIG_ESPFLEET_HUB_URL)
#    define ESPFLEET_ENABLE_MDNS 0
#  endif
#endif
#ifdef CONFIG_ESPFLEET_WEB_UI_PORT
#  ifndef ESPFLEET_WEB_PORT
#    define ESPFLEET_WEB_PORT CONFIG_ESPFLEET_WEB_UI_PORT
#  endif
#endif
#ifdef CONFIG_ESPFLEET_STATUS_PORT
#  ifndef ESPFLEET_INFO_PORT
#    define ESPFLEET_INFO_PORT CONFIG_ESPFLEET_STATUS_PORT
#  endif
#endif
#ifdef CONFIG_ESPFLEET_AGENT_STACK
#  ifndef ESPFLEET_STACK_SIZE
#    define ESPFLEET_STACK_SIZE CONFIG_ESPFLEET_AGENT_STACK
#  endif
#endif
#ifdef CONFIG_ESPFLEET_AGENT_PRIO
#  ifndef ESPFLEET_TASK_PRIORITY
#    define ESPFLEET_TASK_PRIORITY CONFIG_ESPFLEET_AGENT_PRIO
#  endif
#endif

// --------------------------------------------------------------- build flags

#ifndef ESPFLEET_HUB
#define ESPFLEET_HUB ""              // e.g. -DESPFLEET_HUB='"http://192.168.3.87:8080"'
#endif
#ifndef ESPFLEET_PROJECT
#define ESPFLEET_PROJECT ""          // Gogs slug, e.g. "lorenzo/sensor-node"
#endif
#ifndef ESPFLEET_NAME
#define ESPFLEET_NAME ""             // falls back to espfleet-<mac suffix>
#endif
#ifndef ESPFLEET_VERSION
#define ESPFLEET_VERSION ""          // injected by espfleet_build.py from git describe
#endif
#ifndef ESPFLEET_GIT_COMMIT
#define ESPFLEET_GIT_COMMIT ""
#endif
#ifndef ESPFLEET_GIT_BRANCH
#define ESPFLEET_GIT_BRANCH ""
#endif
#ifndef ESPFLEET_GIT_DIRTY
#define ESPFLEET_GIT_DIRTY 0
#endif
#ifndef ESPFLEET_BUILD_TIME
#define ESPFLEET_BUILD_TIME ""
#endif
#ifndef ESPFLEET_INTERVAL_MS
#define ESPFLEET_INTERVAL_MS 30000
#endif
#ifndef ESPFLEET_ENABLE_OTA
#define ESPFLEET_ENABLE_OTA 1
#endif
#ifndef ESPFLEET_ENABLE_MDNS
#define ESPFLEET_ENABLE_MDNS 1
#endif
#ifndef ESPFLEET_INFO_PORT
#define ESPFLEET_INFO_PORT 0         // set to e.g. 8266 to serve /fleet/info yourself
#endif
#ifndef ESPFLEET_WEB_PORT
#define ESPFLEET_WEB_PORT 0          // port of the web UI your project already serves
#endif
#ifndef ESPFLEET_WEB_PATH
#define ESPFLEET_WEB_PATH "/"
#endif
#ifndef ESPFLEET_STACK_SIZE
#define ESPFLEET_STACK_SIZE 8192
#endif

#ifndef ESPFLEET_TASK_PRIORITY
#define ESPFLEET_TASK_PRIORITY 1
#endif
#ifndef ESPFLEET_MAX_EXTRA
#define ESPFLEET_MAX_EXTRA 8        // upstream 4 (vendored patch)
#endif
#ifndef ESPFLEET_BODY_MAX
#define ESPFLEET_BODY_MAX 1792      // heartbeat JSON buffer (heap), upstream 896 on the stack
#endif
#ifndef ESPFLEET_HEALTH_CUSTOM_MAX
#define ESPFLEET_HEALTH_CUSTOM_MAX 800
#endif
#ifndef ESPFLEET_HEALTH_JSON_MAX
#define ESPFLEET_HEALTH_JSON_MAX 1900
#endif
#ifndef ESPFLEET_EXTRA_VALUE_LEN
#define ESPFLEET_EXTRA_VALUE_LEN 32
#endif
#ifndef ESPFLEET_MAX_METRICS
#define ESPFLEET_MAX_METRICS 28     // upstream 16
#endif

// --------------------------------------------------------------------- types

/// Return true if the command was handled. Unhandled "reboot"/"ota" are
/// handled internally; anything else is reported back as unsupported.
typedef bool (*espfleet_command_cb)(const char* cmd, const char* arg);

// ------------------------------------------------------------------- C API
//
// Usable from plain C, which is what most bare ESP-IDF projects are:
//
//     #include "EspFleet.h"          // or "espfleet_api.h"
//     void app_main(void) { ...wifi up...; espfleet_start(NULL); }

#ifdef __cplusplus
extern "C" {
#endif

/// VENDORED PATCH. Called by the agent before it does something that needs the device quiet:
/// what = "ota" (before any flash write), "reboot" (hub reboot command), "restart" (after a good
/// OTA, just before esp_restart(); result ignored). Return false to refuse "ota" / "reboot".
typedef bool (*espfleet_gate_cb)(const char* what);
void espfleet_set_gate(espfleet_gate_cb cb);

/// VENDORED PATCH. Fills the application part of a HEALTH record. `stacks` receives members of the
/// "stacks" object, `cnt` members of "cnt", each as comma-prefixed JSON members (e.g. `,"zb_tcp":812`).
/// If a callback is set, the agent no longer emits its own `"wifi_reconn":0`; supply it in `cnt`.
typedef void (*espfleet_health_cb)(char* stacks, size_t stacks_cap, char* cnt, size_t cnt_cap);
void espfleet_set_health_cb(espfleet_health_cb cb);

typedef struct {
  const char* hub;       ///< hub base URL, NULL = CONFIG_ESPFLEET_HUB_URL
  const char* project;   ///< Gogs slug "owner/repo", NULL = build-time value
  const char* name;      ///< device name, NULL = espfleet-<mac suffix>
  const char* version;   ///< NULL = git describe, else the app descriptor
  uint32_t interval_ms;  ///< 0 = build-time default
  uint16_t web_port;     ///< port of a web UI this project already serves
  const char* web_path;  ///< path of that UI, NULL = "/"
  uint16_t info_port;    ///< serve /fleet/info here, 0 = disabled
  bool enable_ota;
  bool enable_mdns;
  bool spawn_task;       ///< false = call espfleet_loop() yourself
} espfleet_config_t;

/// Fills `out` with the values chosen in menuconfig / build flags.
void espfleet_config_default(espfleet_config_t* out);

/// Starts the agent. Safe to call before the network is up; it waits for an IP.
/// Pass NULL for the default configuration. Returns ESP_OK on success.
int espfleet_start(const espfleet_config_t* config);

/// Convenience: start with everything default except the hub URL.
int espfleet_start_with_hub(const char* hub_url);

void espfleet_stop(void);

void espfleet_set_name(const char* name);
void espfleet_set_project(const char* slug);
void espfleet_set_version(const char* version);
void espfleet_set_web_ui(uint16_t port, const char* path);
void espfleet_set_interval(uint32_t ms);
void espfleet_on_command(espfleet_command_cb callback);

/// Attaches a small key/value to every heartbeat. Copied, max 31 characters.
void espfleet_set_extra(const char* key, const char* value);
void espfleet_set_extra_int(const char* key, long value);

/// Sets a software-defined telemetry metric tracked dynamically by the agent.
void espfleet_set_metric(const char* key, double value);
void espfleet_set_metric_int(const char* key, int64_t value);

/// Renders the status document, the same one /fleet/info serves. Hand it to
/// your own HTTP server if you already run one. Returns 0 if it did not fit.
size_t espfleet_info_json(char* out, size_t out_len);

/// Sends a heartbeat on the next agent iteration.
void espfleet_poke(void);

/// Runs one agent iteration. Only needed when spawn_task was false.
void espfleet_loop(void);

bool espfleet_online(void);
const char* espfleet_device_id(void);
const char* espfleet_hub_url(void);  // VENDORED PATCH
uint32_t espfleet_heartbeat_count(void);

/// Log shipper and health functions
void espfleet_log_push(char level, const char* tag, const char* msg);
void espfleet_health_emit(void);
bool espfleet_discover_hub(char* url_out, size_t cap);

#ifdef __cplusplus
}  // extern "C"
#endif

#ifdef __cplusplus

struct EspFleetConfig {
  const char* hub = ESPFLEET_HUB;
  const char* project = ESPFLEET_PROJECT;
  const char* name = ESPFLEET_NAME;
  const char* version = ESPFLEET_VERSION;
  uint32_t interval_ms = ESPFLEET_INTERVAL_MS;
  uint16_t web_port = ESPFLEET_WEB_PORT;
  const char* web_path = ESPFLEET_WEB_PATH;
  uint16_t info_port = ESPFLEET_INFO_PORT;
  bool enable_ota = ESPFLEET_ENABLE_OTA;
  bool enable_mdns = ESPFLEET_ENABLE_MDNS;
  bool spawn_task = true;  ///< false = you must call loop() yourself
};

class EspFleetClass {
 public:
  /// Start the agent. Safe to call before Wi-Fi is up; it waits for an IP.
  bool begin(const EspFleetConfig& config);
  bool begin(const char* hub_url = nullptr);
  void end();

  void setName(const char* name);
  void setProject(const char* slug);
  void setVersion(const char* version);
  void setWebUi(uint16_t port, const char* path = "/");
  void setInterval(uint32_t ms);
  void onCommand(espfleet_command_cb cb) { command_cb_ = cb; }

  /// Attach a small key/value to every heartbeat (value is copied, <=31 chars).
  void setExtra(const char* key, const char* value);
  void setExtra(const char* key, long value);

  /// Declare / update software-defined telemetry metrics.
  void setMetric(const char* key, double value);
  void setMetric(const char* key, int64_t value);

  /// Render the status document. Also what /fleet/info serves.
  /// Returns the number of bytes written (excluding the terminator).
  size_t infoJson(char* out, size_t out_len) const;

  /// Run one agent iteration. Only needed when spawn_task = false.
  void loop();

  /// Force a heartbeat on the next iteration (e.g. after a state change).
  void poke() { poke_ = true; }

  /// Push a log message into the shipper queue
  void pushLog(char level, const char* tag, const char* msg);

  /// Queue a health report immediately
  void emitHealth();

  /// Query mDNS for an ESPFleet hub
  bool discoverHub(char* url_out, size_t cap);

  const char* deviceId() const { return device_id_; }
  const char* hubUrl() const { return hub_url_; }  // VENDORED PATCH: resolved hub (configured or mDNS)
  bool online() const { return online_; }
  uint32_t heartbeats() const { return heartbeats_; }

 private:
  static void taskEntry(void* arg);
  static int vprintfHook(const char* fmt, va_list ap);

  bool sendHeartbeat();
  void handleCommand(const char* body);
  void reportResult(long cid, bool ok, const char* message);
  bool performOta(const char* url, const char* sha256, long cid);
  void startInfoServer();
  void serviceInfoServer();
  void advertiseMdns();
  void confirmOtaImage();
  void resolveIdentity();
  void shipLogs();
  void checkHealthReport();

  EspFleetConfig cfg_{};
  char hub_url_[128] = {0};
  char device_id_[13] = {0};
  char name_buf_[33] = {0};
  char project_buf_[65] = {0};
  char version_buf_[33] = {0};
  char web_path_buf_[33] = {0};
  char ota_result_[40] = {0};

  struct Extra {
    char key[16];
    char value[ESPFLEET_EXTRA_VALUE_LEN];
  } extras_[ESPFLEET_MAX_EXTRA] = {};
  uint8_t extra_count_ = 0;

  struct Metric {
    char key[24];
    double value;
    bool is_int;
    int64_t int_val;
  } metrics_[ESPFLEET_MAX_METRICS] = {};
  uint8_t metric_count_ = 0;

  espfleet_command_cb command_cb_ = nullptr;
  void* task_ = nullptr;
  void* log_lock_ = nullptr;
  int info_sock_ = -1;
  uint32_t boot_count_ = 0;
  uint32_t heartbeats_ = 0;
  uint32_t failures_ = 0;
  int64_t next_beat_us_ = 0;
  bool running_ = false;
  bool online_ = false;
  bool poke_ = false;
  bool mdns_done_ = false;
  bool image_confirmed_ = false;
  bool web_set_ = false;
  bool interval_set_ = false;

  // Logging & Health state
  uint32_t health_seq_ = 0;
  int64_t next_health_us_ = 0;
  int64_t next_log_ship_us_ = 0;
  uint32_t logs_dropped_ = 0;
};

extern EspFleetClass EspFleet;

#endif  // __cplusplus
