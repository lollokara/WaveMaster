#include "spi_probe.h"
#include "ad3552r_board.h"
#include "no_os_spi.h"
#include "no_os_gpio.h"
#include "no_os_delay.h"
#include "esp_log.h"

static const char *TAG = "spi_probe";

static void read_reg_raw(struct no_os_spi_desc *spi, uint8_t addr, const char *name)
{
    uint8_t instr = 0x80 | addr; /* read bit + address, matches AD3552R_READ_BIT */
    uint8_t data = 0xAA;
    int32_t err;
    struct no_os_spi_msg msgs[2] = {0};

    msgs[0].tx_buff = &instr;
    msgs[0].bytes_number = 1;
    msgs[0].cs_change = 0; /* hold CS for the data phase */

    msgs[1].rx_buff = &data;
    msgs[1].bytes_number = 1;
    msgs[1].cs_change = 1; /* release CS after */

    err = no_os_spi_transfer(spi, msgs, 2);
    ESP_LOGI(TAG, "reg[0x%02x] (%s) = 0x%02x (err=%ld)", addr, name, data, (long)err);
}

void spi_probe_run(void)
{
    struct no_os_gpio_desc *rst = NULL;
    struct no_os_gpio_init_param rst_param = { .number = BOARD_PIN_RST };
    struct no_os_gpio_desc *qspi_pin = NULL;
    struct no_os_gpio_init_param qspi_pin_param = { .number = BOARD_PIN_SPI_OP1 };
    struct no_os_spi_desc *spi = NULL;
    struct ad3552r_esp32_spi_pins pins = {
        .sck = BOARD_PIN_SCK, .cs = BOARD_PIN_CS,
        .d0 = BOARD_PIN_D0, .d1 = BOARD_PIN_D1,
        .d2 = BOARD_PIN_D2, .d3 = BOARD_PIN_D3,
        .dir = BOARD_PIN_DIR,
    };
    struct no_os_spi_init_param spi_param = {
        .device_id = 0,
        .chip_select = 0,
        .mode = NO_OS_SPI_MODE_0,
        .max_speed_hz = 1000000, /* slow, for clean bring-up */
        .bit_order = NO_OS_SPI_BIT_ORDER_MSB_FIRST,
        .extra = &pins,
    };
    int32_t err;

    ESP_LOGI(TAG, "--- raw SPI probe start ---");

    /* Force classic/dual SPI mode on the chip; without this the QSPI pin
     * floats and the chip may be expecting quad-line framing instead. */
    no_os_gpio_get_optional(&qspi_pin, &qspi_pin_param);
    if (qspi_pin)
        no_os_gpio_direction_output(qspi_pin, NO_OS_GPIO_LOW);
    ESP_LOGI(TAG, "QSPI mode pin (IO%d) held low", BOARD_PIN_SPI_OP1);

    no_os_gpio_get_optional(&rst, &rst_param);
    if (rst) {
        no_os_gpio_direction_output(rst, NO_OS_GPIO_LOW);
        no_os_mdelay(1);
        no_os_gpio_set_value(rst, NO_OS_GPIO_HIGH);
        no_os_mdelay(100); /* t18: up to 100ms before register access */
        ESP_LOGI(TAG, "RST pulsed low->high on IO%d, waited 100ms", BOARD_PIN_RST);
    } else {
        ESP_LOGW(TAG, "RST gpio desc was NULL");
    }

    err = no_os_spi_init(&spi, &spi_param);
    if (err) {
        ESP_LOGE(TAG, "no_os_spi_init failed: %ld", (long)err);
        return;
    }

    /* Expected power-up values per AD3552R datasheet / ref driver:
     * INTERFACE_CONFIG_B (0x01) = 0x08 (AD3552R_DEFAULT_CONFIG_B_VALUE)
     * CHIP_TYPE (0x03) = 0x08
     * PRODUCT_ID_L/H (0x04/0x05) = 0x08 / 0x40 (ID 0x4008) */
    read_reg_raw(spi, 0x01, "INTERFACE_CONFIG_B, expect 0x08");
    read_reg_raw(spi, 0x03, "CHIP_TYPE, expect 0x08");
    read_reg_raw(spi, 0x04, "PRODUCT_ID_L, expect 0x08");
    read_reg_raw(spi, 0x05, "PRODUCT_ID_H, expect 0x40");

    no_os_spi_remove(spi);
    if (rst)
        no_os_gpio_remove(rst);

    ESP_LOGI(TAG, "--- raw SPI probe end ---");
}
