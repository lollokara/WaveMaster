// C API shim: forwards to the C++ singleton so plain-C ESP-IDF apps can use it.

#include "EspFleet.h"

#include "esp_err.h"

// Implemented in EspFleet.cpp (file-scope state there).
void espfleet_gate_set_impl(espfleet_gate_cb cb);
void espfleet_health_set_impl(espfleet_health_cb cb);

extern "C" {

void espfleet_config_default(espfleet_config_t* out) {
  if (!out) return;
  const EspFleetConfig defaults;
  out->hub = defaults.hub;
  out->project = defaults.project;
  out->name = defaults.name;
  out->version = defaults.version;
  out->interval_ms = defaults.interval_ms;
  out->web_port = defaults.web_port;
  out->web_path = defaults.web_path;
  out->info_port = defaults.info_port;
  out->enable_ota = defaults.enable_ota;
  out->enable_mdns = defaults.enable_mdns;
  out->spawn_task = defaults.spawn_task;
}

int espfleet_start(const espfleet_config_t* config) {
  EspFleetConfig cfg;  // build-flag / Kconfig defaults
  if (config) {
    if (config->hub) cfg.hub = config->hub;
    if (config->project) cfg.project = config->project;
    if (config->name) cfg.name = config->name;
    if (config->version) cfg.version = config->version;
    if (config->interval_ms) cfg.interval_ms = config->interval_ms;
    if (config->web_path) cfg.web_path = config->web_path;
    cfg.web_port = config->web_port;
    cfg.info_port = config->info_port;
    cfg.enable_ota = config->enable_ota;
    cfg.enable_mdns = config->enable_mdns;
    cfg.spawn_task = config->spawn_task;
  }
  return EspFleet.begin(cfg) ? ESP_OK : ESP_FAIL;
}

int espfleet_start_with_hub(const char* hub_url) {
  return EspFleet.begin(hub_url) ? ESP_OK : ESP_FAIL;
}

void espfleet_stop(void) { EspFleet.end(); }

void espfleet_set_name(const char* name) { EspFleet.setName(name); }
void espfleet_set_project(const char* slug) { EspFleet.setProject(slug); }
void espfleet_set_version(const char* version) { EspFleet.setVersion(version); }
void espfleet_set_interval(uint32_t ms) { EspFleet.setInterval(ms); }
void espfleet_on_command(espfleet_command_cb callback) { EspFleet.onCommand(callback); }
void espfleet_set_gate(espfleet_gate_cb cb) { espfleet_gate_set_impl(cb); }
void espfleet_set_health_cb(espfleet_health_cb cb) { espfleet_health_set_impl(cb); }
void espfleet_poke(void) { EspFleet.poke(); }
void espfleet_loop(void) { EspFleet.loop(); }
bool espfleet_online(void) { return EspFleet.online(); }
const char* espfleet_device_id(void) { return EspFleet.deviceId(); }
const char* espfleet_hub_url(void) { return EspFleet.hubUrl(); }
uint32_t espfleet_heartbeat_count(void) { return EspFleet.heartbeats(); }

void espfleet_set_web_ui(uint16_t port, const char* path) {
  EspFleet.setWebUi(port, path ? path : "/");
}

void espfleet_set_extra(const char* key, const char* value) {
  EspFleet.setExtra(key, value);
}

void espfleet_set_extra_int(const char* key, long value) {
  EspFleet.setExtra(key, value);
}

void espfleet_set_metric(const char* key, double value) {
  EspFleet.setMetric(key, value);
}

void espfleet_set_metric_int(const char* key, int64_t value) {
  EspFleet.setMetric(key, value);
}

size_t espfleet_info_json(char* out, size_t out_len) {
  return EspFleet.infoJson(out, out_len);
}

void espfleet_log_push(char level, const char* tag, const char* msg) {
  EspFleet.pushLog(level, tag, msg);
}

void espfleet_health_emit(void) {
  EspFleet.emitHealth();
}

bool espfleet_discover_hub(char* url_out, size_t cap) {
  return EspFleet.discoverHub(url_out, cap);
}

}  // extern "C"
