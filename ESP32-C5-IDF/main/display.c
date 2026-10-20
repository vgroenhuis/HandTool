// SH1106 128x64 on I2C (address 0x3C or 0x3D), as U8G2_SH1106_128X64_NONAME in the Arduino version: a 132-column
// controller showing columns 2..129, written page by page (8 rows of 8 pixels). Text: 5x7 font, 21 x 8 characters.
#include "display.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "closer_demo.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "font5x7.h"
#include "sensors.h"

static const char *TAG = "display";

static i2c_master_bus_handle_t bus;
static i2c_master_dev_handle_t oled;
static bool present;
static uint8_t fb[8][128];

static bool cmds(const uint8_t *c, size_t n) {
    uint8_t buf[24];
    if (n + 1 > sizeof buf) return false;
    buf[0] = 0x00;   // control byte: commands follow
    memcpy(buf + 1, c, n);
    return i2c_master_transmit(oled, buf, n + 1, 50) == ESP_OK;
}

static bool flush(void) {
    uint8_t buf[129];
    buf[0] = 0x40;   // control byte: data follows
    for (int page = 0; page < 8; page++) {
        const uint8_t addr[] = {(uint8_t)(0xB0 | page), 0x02, 0x10};   // page, column 2 (low nibble, high nibble)
        if (!cmds(addr, sizeof addr)) return false;
        memcpy(buf + 1, fb[page], 128);
        if (i2c_master_transmit(oled, buf, sizeof buf, 50) != ESP_OK) return false;
    }
    return true;
}

static void clear(void) {
    memset(fb, 0, sizeof fb);
}

static void text(int line, const char *fmt, ...) {
    char s[24];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(s, sizeof s, fmt, ap);
    va_end(ap);
    if (line < 0 || line > 7) return;
    for (int i = 0; s[i] && i < 21; i++) {
        const unsigned char *g = &font5x7[(unsigned char)s[i] * 5];
        memcpy(&fb[line][i * 6], g, 5);
        fb[line][i * 6 + 5] = 0;
    }
}

bool display_present(void) {
    return present;
}

void display_start(void) {
    i2c_master_bus_config_t bcfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = CONFIG_HANDTOOL_OLED_SDA_GPIO,
        .scl_io_num = CONFIG_HANDTOOL_OLED_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    if (i2c_new_master_bus(&bcfg, &bus) != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus did not start");
        return;
    }
    uint16_t address = 0;
    if (i2c_master_probe(bus, 0x3C, 50) == ESP_OK) address = 0x3C;
    else if (i2c_master_probe(bus, 0x3D, 50) == ESP_OK) address = 0x3D;
    if (!address) {
        ESP_LOGW(TAG, "no display found at 0x3C or 0x3D: carrying on without");
        return;
    }
    i2c_device_config_t dcfg = {.dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = address, .scl_speed_hz = 400000};
    if (i2c_master_bus_add_device(bus, &dcfg, &oled) != ESP_OK) return;
    static const uint8_t init[] = {
        0xAE,         // display off
        0xD5, 0x80,   // clock divide
        0xA8, 0x3F,   // multiplex 64
        0xD3, 0x00,   // display offset
        0x40,         // start line 0
        0xAD, 0x8B,   // DC-DC on
        0xA1,         // segment remap
        0xC8,         // COM scan direction: remapped
        0xDA, 0x12,   // COM pins
        0x81, 0xCF,   // contrast
        0xD9, 0x22,   // precharge
        0xDB, 0x40,   // VCOM deselect
        0xA4,         // show the RAM
        0xA6,         // not inverted
    };
    if (!cmds(init, sizeof init)) {
        ESP_LOGW(TAG, "display did not take its settings");
        return;
    }
    clear();
    flush();
    static const uint8_t on[] = {0xAF};
    cmds(on, 1);
    present = true;
    ESP_LOGI(TAG, "SH1106 at 0x%02X", address);
}

void display_boot_message(const char *msg) {
    if (!present) return;
    clear();
    text(0, "HandTool %s", closer_demo_version());
    text(2, "%s", msg);
    flush();
}

static void display_task(void *arg) {
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(100));
        sensor_snapshot_t s;
        sensors_get(&s);
        closer_demo_status_t st;
        closer_demo_get_status(&st);
        clear();
        text(0, "HandTool %s", closer_demo_version());
        if (st.updating) text(1, "updating firmware...");
        else if (st.connected) text(1, "IP %s", st.ip);
        else text(1, "Wi-Fi: connecting...");
        text(2, "%.21s", CONFIG_HANDTOOL_WEB_ADDRESS);
        if (s.valid) {
            text(3, "J1-3 %4.0f %4.0f %4.0f", s.angles_deg[0], s.angles_deg[1], s.angles_deg[2]);
            text(4, "J4-6 %4.0f %4.0f %4.0f", s.angles_deg[3], s.angles_deg[4], s.angles_deg[5]);
            text(5, "X%4.0f Y%4.0f Z%4.0f mm", s.T[0][3] * 1000, s.T[1][3] * 1000, s.T[2][3] * 1000);
        } else {
            text(4, "MCP3008 not found!");
        }
        if (s.battery_v > 0.1f) text(7, "Bat %.2fV %s", s.battery_v, s.button ? " [button]" : "");
        else text(7, "%s", s.button ? "[button]" : "");
        if (!flush()) ESP_LOGW(TAG, "display write failed");
    }
}

void display_run(void) {
    if (present) xTaskCreate(display_task, "display", 3072, NULL, 2, NULL);
}
