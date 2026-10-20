// The 1.3" SH1106 OLED (128x64, I2C at 0x3C or 0x3D), drawn with u8g2 (components/u8g2) as in the Arduino version, now
// with proportional fonts in a few sizes:
//
//   connected                          connecting
//   HandTool          [wifi] [bat]     HandTool           [x]  [bat]
//   145.126.38.161   (bold, large)     Searching for          (bold)
//   -47 dBm  ch 124  5 GHz             iotroam ...
//   ---------------------------------  ch 124 · try 2
//   J1   13   J2   -1   J3  -21        ---------------------------------
//   J4   26   J5    5   J6 -135        x 95  y 13  z 120 mm
//   x 95  y 13  z 120 mm      [BTN]
//
// Updating firmware and a missing MCP3008 get screens of their own.
#include "display.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "closer_demo.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "sensors.h"
#include "u8g2.h"

static const char *TAG = "display";

static i2c_master_bus_handle_t bus;
static i2c_master_dev_handle_t oled;
static bool present;
static u8g2_t u8g2;

// ---------------------------------------------------------------- u8g2 on the ESP-IDF I2C master driver

static uint8_t tx[160];
static size_t tx_len;

static uint8_t byte_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    switch (msg) {
        case U8X8_MSG_BYTE_START_TRANSFER:
            tx_len = 0;
            break;
        case U8X8_MSG_BYTE_SEND:
            if (tx_len + arg_int > sizeof tx) return 0;
            memcpy(tx + tx_len, arg_ptr, arg_int);
            tx_len += arg_int;
            break;
        case U8X8_MSG_BYTE_END_TRANSFER:
            if (i2c_master_transmit(oled, tx, tx_len, 50) != ESP_OK) return 0;
            break;
        default:
            break;
    }
    return 1;
}

static uint8_t delay_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    switch (msg) {
        case U8X8_MSG_DELAY_MILLI: vTaskDelay(pdMS_TO_TICKS(arg_int) ? pdMS_TO_TICKS(arg_int) : 1); break;
        case U8X8_MSG_DELAY_10MICRO: esp_rom_delay_us(10 * arg_int); break;
        case U8X8_MSG_DELAY_100NANO: esp_rom_delay_us(1); break;
        default: break;
    }
    return 1;
}

// ---------------------------------------------------------------- drawing helpers

static void textf(int x, int y, const uint8_t *font, const char *fmt, ...) {
    char s[48];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(s, sizeof s, fmt, ap);
    va_end(ap);
    u8g2_SetFont(&u8g2, font);
    u8g2_DrawStr(&u8g2, x, y, s);
}

static void text_right(int x_right, int y, const uint8_t *font, const char *s) {
    u8g2_SetFont(&u8g2, font);
    u8g2_DrawStr(&u8g2, x_right - u8g2_GetStrWidth(&u8g2, s), y, s);
}

// Signal bars (4) at x..x+12, bottom at y; rssi 0: not connected (a small cross).
static void wifi_icon(int x, int y, int rssi) {
    int bars = rssi == 0 ? 0 : rssi > -55 ? 4 : rssi > -65 ? 3 : rssi > -75 ? 2 : 1;
    for (int i = 0; i < 4; i++) {
        int h = 3 + 2 * i;
        if (i < bars) u8g2_DrawBox(&u8g2, x + 3 * i, y - h, 2, h);
        else u8g2_DrawPixel(&u8g2, x + 3 * i, y - 1);
    }
    if (!bars) {
        u8g2_DrawLine(&u8g2, x + 6, y - 9, x + 11, y - 4);
        u8g2_DrawLine(&u8g2, x + 11, y - 9, x + 6, y - 4);
    }
}

// Battery outline with a fill for 3.3..4.2 V, at x..x+13, y-7..y.
static void battery_icon(int x, int y, float v) {
    u8g2_DrawFrame(&u8g2, x, y - 7, 12, 7);
    u8g2_DrawBox(&u8g2, x + 12, y - 5, 2, 3);
    float f = (v - 3.3f) / 0.9f;
    if (f < 0) f = 0;
    if (f > 1) f = 1;
    int w = (int)(f * 8 + 0.5f);
    if (w) u8g2_DrawBox(&u8g2, x + 2, y - 5, w, 3);
}

// Wraps `s` into lines of at most `width` pixels at spaces; returns the number of lines drawn (at most max_lines).
static int wrapped(int x, int y, int line_h, int width, int max_lines, const uint8_t *font, const char *s) {
    u8g2_SetFont(&u8g2, font);
    char line[48];
    int lines = 0;
    while (*s && lines < max_lines) {
        size_t best = 0, n = 0;
        while (s[n]) {
            size_t m = n;
            while (s[m] && s[m] != ' ') m++;          // the next word end
            memcpy(line, s, m < sizeof line - 1 ? m : sizeof line - 1);
            line[m < sizeof line - 1 ? m : sizeof line - 1] = 0;
            if (best && u8g2_GetStrWidth(&u8g2, line) > width) break;
            best = m;
            if (!s[m]) break;
            n = m + 1;
        }
        memcpy(line, s, best);
        line[best] = 0;
        u8g2_DrawStr(&u8g2, x, y + lines * line_h, line);
        lines++;
        s += best;
        while (*s == ' ') s++;
    }
    return lines;
}

static void header(const closer_demo_status_t *st, const sensor_snapshot_t *s) {
    textf(0, 9, u8g2_font_helvB08_tr, "HandTool");
    int x = 127;
    if (s->battery_v > 0.1f) {
        char v[8];
        snprintf(v, sizeof v, "%.2fV", s->battery_v);
        text_right(x, 8, u8g2_font_5x8_tr, v);
        u8g2_SetFont(&u8g2, u8g2_font_5x8_tr);
        x -= u8g2_GetStrWidth(&u8g2, v) + 16;
        battery_icon(x, 8, s->battery_v);
        x -= 4;
    }
    wifi_icon(x - 12, 9, st->connected ? (st->rssi ? st->rssi : -60) : 0);
}

static void joints(int y, const sensor_snapshot_t *s) {
    for (int i = 0; i < 6; i++) {
        int col = i % 3, row = i / 3;
        int x = col * 45, yy = y + row * 9;      // "J1" then the angle right-aligned, 9 px before the next column
        textf(x, yy, u8g2_font_5x8_tr, "J%d", i + 1);
        char v[8];
        snprintf(v, sizeof v, "%.0f", s->angles_deg[i]);
        text_right(x + 36, yy, u8g2_font_6x10_tr, v);
    }
}

static void position_line(int y, const sensor_snapshot_t *s) {
    textf(0, y, u8g2_font_6x10_tr, "x%.0f y%.0f z%.0f", s->T[0][3] * 1000, s->T[1][3] * 1000, s->T[2][3] * 1000);
    if (s->button) {
        u8g2_DrawRBox(&u8g2, 104, y - 8, 24, 10, 2);
        u8g2_SetDrawColor(&u8g2, 0);
        textf(107, y - 1, u8g2_font_5x8_tr, "BTN");
        u8g2_SetDrawColor(&u8g2, 1);
    } else {
        text_right(127, y, u8g2_font_5x8_tr, "mm");
    }
}

static void draw(const closer_demo_status_t *st, const sensor_snapshot_t *s, int tick) {
    u8g2_ClearBuffer(&u8g2);
    header(st, s);
    if (st->updating) {
        textf(0, 30, u8g2_font_helvB10_tr, "Updating");
        textf(0, 44, u8g2_font_helvB10_tr, "firmware%.*s", tick / 3 % 4, "...");
        textf(0, 60, u8g2_font_6x10_tr, "Don't switch off");
    } else if (st->connected) {
        textf(0, 23, u8g2_font_helvB10_tr, "%s", st->ip);
        textf(0, 32, u8g2_font_5x8_tr, "%d dBm  ch %d  %s", st->rssi, st->channel, st->channel > 14 ? "5 GHz" : "2.4 GHz");
        u8g2_DrawHLine(&u8g2, 0, 34, 128);
        if (s->valid) {
            joints(43, s);
            position_line(61, s);      // (the 6x10 font's "y" reaches 2 rows below the baseline: row 63)
        } else {
            textf(0, 48, u8g2_font_helvB08_tr, "MCP3008 not found");
            textf(0, 60, u8g2_font_5x8_tr, "check the ADC wires");
        }
    } else {
        // What Wi-Fi is doing right now, with a few moving dots while it works.
        char state[64];
        bool busy = strncmp(st->wifi, "Searching", 9) == 0 || strncmp(st->wifi, "Getting", 7) == 0 ||
                    strncmp(st->wifi, "Starting", 8) == 0 || strncmp(st->wifi, "Moving", 6) == 0;
        snprintf(state, sizeof state, "%s%.*s", st->wifi, busy ? tick / 3 % 4 : 0, "...");
        int lines = wrapped(0, 22, 11, 127, 2, u8g2_font_helvB08_tr, state);
        char detail[48] = "";
        if (st->channel) snprintf(detail, sizeof detail, "ch %d  ", st->channel);
        if (st->attempts > 1) snprintf(detail + strlen(detail), sizeof detail - strlen(detail), "attempt %d", st->attempts);
        if (!detail[0] && strstr(st->wifi, st->ssid)) snprintf(detail, sizeof detail, "5 GHz access points");
        else if (!detail[0]) snprintf(detail, sizeof detail, "network %s", st->ssid);
        textf(0, 22 + lines * 11, u8g2_font_5x8_tr, "%s", detail);
        u8g2_DrawHLine(&u8g2, 0, 46, 128);
        if (s->valid) position_line(60, s);
        else textf(0, 60, u8g2_font_helvB08_tr, "MCP3008 not found");
    }
    u8g2_SendBuffer(&u8g2);
}

// ---------------------------------------------------------------- start

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
    u8g2_Setup_sh1106_i2c_128x64_noname_f(&u8g2, U8G2_R0, byte_cb, delay_cb);
    u8x8_SetI2CAddress(&u8g2.u8x8, address << 1);
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);
    u8g2_SetFontMode(&u8g2, 1);
    present = true;
    ESP_LOGI(TAG, "SH1106 at 0x%02X", address);
}

void display_boot_message(const char *msg) {
    if (!present) return;
    u8g2_ClearBuffer(&u8g2);
    textf(0, 12, u8g2_font_helvB10_tr, "HandTool");
    textf(0, 26, u8g2_font_6x10_tr, "firmware %s", closer_demo_version());
    textf(0, 44, u8g2_font_helvB08_tr, "%s", msg);
    u8g2_SendBuffer(&u8g2);
}

static void display_task(void *arg) {
    int tick = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(100));
        sensor_snapshot_t s;
        sensors_get(&s);
        closer_demo_status_t st;
        closer_demo_get_status(&st);
        draw(&st, &s, tick++);
    }
}

void display_run(void) {
    if (present) xTaskCreate(display_task, "display", 4096, NULL, 2, NULL);
}
