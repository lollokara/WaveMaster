#include "calib.h"
#include <stdio.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

static const char *TAG = "calib";
#define NVS_NS "wmcal"

/* Defaults - placeholders until the real galvo/lens working-area
 * calibration is known. 1.0 V/mm, no offset, is not calibrated to any
 * actual optical setup. */
static float s_x_volts_per_mm = 1.0f;
static float s_x_offset_volts = 0.0f;
static float s_y_volts_per_mm = 1.0f;
static float s_y_offset_volts = 0.0f;

/* Marking-quality defaults - conservative placeholders, not tuned against
 * any real laser/galvo yet (same caveat as the gain/offset defaults
 * above). 0 for wobble/skywrite/radial-k1 means "disabled", matching
 * this firmware's prior (pre-this-feature) behavior exactly until tuned. */
static float s_jump_delay_us = 200.0f;
static float s_mark_delay_us = 50.0f;
static float s_polygon_delay_us = 20.0f;
static float s_laser_on_delay_us = 100.0f;
static float s_laser_off_delay_us = 100.0f;
static float s_radial_k1 = 0.0f;
static float s_wobble_diameter_mm = 0.0f;
static float s_wobble_pitch_mm = 1.0f;
static float s_skywrite_margin_mm = 0.0f;
static float s_hatch_spacing_mm = 0.2f;
static float s_hatch_angle_deg = 0.0f;
static float s_preview_edge_subdivisions = 8.0f;

/* Motion-planner defaults - conservative placeholders (see motion_planner.c),
 * not tuned against any real galvo's actual mechanical accel/speed limits. */
static float s_max_accel_mm_s2 = 1000.0f;
static float s_junction_deviation_mm = 0.01f;
static float s_max_velocity_mm_s = 200.0f;

static void load_float(nvs_handle_t h, const char *key, float *inout)
{
    size_t sz = sizeof(*inout);
    esp_err_t err = nvs_get_blob(h, key, inout, &sz);

    if (err != ESP_OK)
        ESP_LOGW(TAG, "no stored '%s', using default %.4f", key, (double)*inout);
}

static void save_float(const char *key, float val)
{
    nvs_handle_t h;

    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open (write) failed for '%s'", key);
        return;
    }
    nvs_set_blob(h, key, &val, sizeof(val));
    nvs_commit(h);
    nvs_close(h);
}

void calib_init(void)
{
    esp_err_t err = nvs_flash_init();

    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_flash_init failed: %s, calibration will not persist",
                 esp_err_to_name(err));
        return;
    }

    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        load_float(h, "xvpm", &s_x_volts_per_mm);
        load_float(h, "xoff", &s_x_offset_volts);
        load_float(h, "yvpm", &s_y_volts_per_mm);
        load_float(h, "yoff", &s_y_offset_volts);
        load_float(h, "jumpdly", &s_jump_delay_us);
        load_float(h, "markdly", &s_mark_delay_us);
        load_float(h, "polydly", &s_polygon_delay_us);
        load_float(h, "laserond", &s_laser_on_delay_us);
        load_float(h, "laseroffd", &s_laser_off_delay_us);
        load_float(h, "radk1", &s_radial_k1);
        load_float(h, "wobd", &s_wobble_diameter_mm);
        load_float(h, "wobp", &s_wobble_pitch_mm);
        load_float(h, "skym", &s_skywrite_margin_mm);
        load_float(h, "hspc", &s_hatch_spacing_mm);
        load_float(h, "hang", &s_hatch_angle_deg);
        load_float(h, "prevsub", &s_preview_edge_subdivisions);
        load_float(h, "accel", &s_max_accel_mm_s2);
        load_float(h, "jdev", &s_junction_deviation_mm);
        load_float(h, "maxvel", &s_max_velocity_mm_s);
        nvs_close(h);
    } else {
        ESP_LOGW(TAG, "no calibration stored yet, using defaults");
    }

    ESP_LOGI(TAG, "X: %.4f V/mm + %.4f V offset", (double)s_x_volts_per_mm,
             (double)s_x_offset_volts);
    ESP_LOGI(TAG, "Y: %.4f V/mm + %.4f V offset", (double)s_y_volts_per_mm,
             (double)s_y_offset_volts);
}

float calib_x_volts_per_mm(void) { return s_x_volts_per_mm; }
float calib_x_offset_volts(void) { return s_x_offset_volts; }
float calib_y_volts_per_mm(void) { return s_y_volts_per_mm; }
float calib_y_offset_volts(void) { return s_y_offset_volts; }

float calib_jump_delay_us(void) { return s_jump_delay_us; }
float calib_mark_delay_us(void) { return s_mark_delay_us; }
float calib_polygon_delay_us(void) { return s_polygon_delay_us; }
float calib_laser_on_delay_us(void) { return s_laser_on_delay_us; }
float calib_laser_off_delay_us(void) { return s_laser_off_delay_us; }
float calib_radial_k1(void) { return s_radial_k1; }
float calib_wobble_diameter_mm(void) { return s_wobble_diameter_mm; }
float calib_wobble_pitch_mm(void) { return s_wobble_pitch_mm; }
float calib_skywrite_margin_mm(void) { return s_skywrite_margin_mm; }
float calib_hatch_spacing_mm(void) { return s_hatch_spacing_mm; }
float calib_hatch_angle_deg(void) { return s_hatch_angle_deg; }
float calib_preview_edge_subdivisions(void) { return s_preview_edge_subdivisions; }

float calib_max_accel_mm_s2(void) { return s_max_accel_mm_s2; }
float calib_junction_deviation_mm(void) { return s_junction_deviation_mm; }
float calib_max_velocity_mm_s(void) { return s_max_velocity_mm_s; }

bool calib_set_param(int number, float value)
{
    switch (number) {
    case 100:
        s_x_volts_per_mm = value;
        save_float("xvpm", value);
        return true;
    case 101:
        s_y_volts_per_mm = value;
        save_float("yvpm", value);
        return true;
    case 110:
        s_x_offset_volts = value;
        save_float("xoff", value);
        return true;
    case 111:
        s_y_offset_volts = value;
        save_float("yoff", value);
        return true;
    case 130:
        s_jump_delay_us = value;
        save_float("jumpdly", value);
        return true;
    case 131:
        s_mark_delay_us = value;
        save_float("markdly", value);
        return true;
    case 132:
        s_polygon_delay_us = value;
        save_float("polydly", value);
        return true;
    case 133:
        s_laser_on_delay_us = value;
        save_float("laserond", value);
        return true;
    case 134:
        s_laser_off_delay_us = value;
        save_float("laseroffd", value);
        return true;
    case 140:
        s_radial_k1 = value;
        save_float("radk1", value);
        return true;
    case 150:
        s_wobble_diameter_mm = value;
        save_float("wobd", value);
        return true;
    case 151:
        s_wobble_pitch_mm = value;
        save_float("wobp", value);
        return true;
    case 160:
        s_skywrite_margin_mm = value;
        save_float("skym", value);
        return true;
    case 170:
        s_hatch_spacing_mm = value;
        save_float("hspc", value);
        return true;
    case 171:
        s_hatch_angle_deg = value;
        save_float("hang", value);
        return true;
    case 190:
        s_preview_edge_subdivisions = value;
        save_float("prevsub", value);
        return true;
    case 180:
        s_max_accel_mm_s2 = value;
        save_float("accel", value);
        return true;
    case 181:
        s_junction_deviation_mm = value;
        save_float("jdev", value);
        return true;
    case 182:
        s_max_velocity_mm_s = value;
        save_float("maxvel", value);
        return true;
    default:
        return false;
    }
}

void calib_dump(void (*write_line)(const char *line))
{
    char buf[48];

    snprintf(buf, sizeof(buf), "$100=%.4f\r\n", (double)s_x_volts_per_mm);
    write_line(buf);
    snprintf(buf, sizeof(buf), "$101=%.4f\r\n", (double)s_y_volts_per_mm);
    write_line(buf);
    snprintf(buf, sizeof(buf), "$110=%.4f\r\n", (double)s_x_offset_volts);
    write_line(buf);
    snprintf(buf, sizeof(buf), "$111=%.4f\r\n", (double)s_y_offset_volts);
    write_line(buf);
    snprintf(buf, sizeof(buf), "$130=%.1f\r\n", (double)s_jump_delay_us);
    write_line(buf);
    snprintf(buf, sizeof(buf), "$131=%.1f\r\n", (double)s_mark_delay_us);
    write_line(buf);
    snprintf(buf, sizeof(buf), "$132=%.1f\r\n", (double)s_polygon_delay_us);
    write_line(buf);
    snprintf(buf, sizeof(buf), "$133=%.1f\r\n", (double)s_laser_on_delay_us);
    write_line(buf);
    snprintf(buf, sizeof(buf), "$134=%.1f\r\n", (double)s_laser_off_delay_us);
    write_line(buf);
    snprintf(buf, sizeof(buf), "$140=%.6f\r\n", (double)s_radial_k1);
    write_line(buf);
    snprintf(buf, sizeof(buf), "$150=%.4f\r\n", (double)s_wobble_diameter_mm);
    write_line(buf);
    snprintf(buf, sizeof(buf), "$151=%.4f\r\n", (double)s_wobble_pitch_mm);
    write_line(buf);
    snprintf(buf, sizeof(buf), "$160=%.4f\r\n", (double)s_skywrite_margin_mm);
    write_line(buf);
    snprintf(buf, sizeof(buf), "$170=%.4f\r\n", (double)s_hatch_spacing_mm);
    write_line(buf);
    snprintf(buf, sizeof(buf), "$171=%.4f\r\n", (double)s_hatch_angle_deg);
    write_line(buf);
    snprintf(buf, sizeof(buf), "$190=%.1f\r\n", (double)s_preview_edge_subdivisions);
    write_line(buf);
    snprintf(buf, sizeof(buf), "$180=%.1f\r\n", (double)s_max_accel_mm_s2);
    write_line(buf);
    snprintf(buf, sizeof(buf), "$181=%.4f\r\n", (double)s_junction_deviation_mm);
    write_line(buf);
    snprintf(buf, sizeof(buf), "$182=%.1f\r\n", (double)s_max_velocity_mm_s);
    write_line(buf);
}
