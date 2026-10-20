// MCP3008 (8 x 10-bit ADC on SPI): channels 0..5 the joint potentiometers, channel 6 the drag button. Sampled every
// HANDTOOL_SAMPLE_MS and low-pass filtered (exponential moving average, HANDTOOL_FILTER_MS), as in the Arduino version.
// Angle = direction x (filtered - zero) x 270 / 1024 degrees (the potentiometers turn 270 degrees).
#include "sensors.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>

#include "driver/spi_master.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "kinematics.h"
#include "nvs.h"

static const char *TAG = "sensors";

const int16_t SENSORS_DEFAULT_ZERO[6] = {470, 488, 215, 530, 95, 380};
const int8_t SENSORS_JOINT_DIRECTIONS[6] = {1, -1, -1, 1, 1, -1};

static spi_device_handle_t mcp;
static SemaphoreHandle_t lock;
static sensor_snapshot_t snap;
static int16_t zero[6];
static bool default_zero = true;

// ---------------------------------------------------------------- MCP3008

static bool mcp_read(int ch, uint16_t *out) {
    spi_transaction_t t = {.length = 24, .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA};
    t.tx_data[0] = 0x01;                          // start bit
    t.tx_data[1] = (uint8_t)(0x80 | (ch << 4));   // single-ended, channel
    t.tx_data[2] = 0x00;
    if (spi_device_polling_transmit(mcp, &t) != ESP_OK) return false;
    *out = (uint16_t)(((t.rx_data[1] & 0x03) << 8) | t.rx_data[2]);
    return true;
}

static bool mcp_read_all(uint16_t v[8]) {
    for (int ch = 0; ch < 8; ch++) {
        if (!mcp_read(ch, &v[ch])) return false;
    }
    return true;
}

// Without a chip MISO floats or is pulled to one level: every channel reads exactly 0 or exactly 1023.
static bool looks_absent(const uint16_t v[8]) {
    bool all0 = true, all1023 = true;
    for (int ch = 0; ch < 8; ch++) {
        all0 &= v[ch] == 0;
        all1023 &= v[ch] == 1023;
    }
    return all0 || all1023;
}

static bool mcp_init(void) {
    spi_bus_config_t bus = {
        .mosi_io_num = CONFIG_HANDTOOL_MCP_MOSI_GPIO,
        .miso_io_num = CONFIG_HANDTOOL_MCP_MISO_GPIO,
        .sclk_io_num = CONFIG_HANDTOOL_MCP_SCK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    esp_err_t e = spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_DISABLED);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus: %s", esp_err_to_name(e));
        return false;
    }
    spi_device_interface_config_t dev = {
        .clock_speed_hz = CONFIG_HANDTOOL_MCP_CLOCK_KHZ * 1000,
        .mode = 0,
        .spics_io_num = CONFIG_HANDTOOL_MCP_CS_GPIO,
        .queue_size = 1,
    };
    e = spi_bus_add_device(SPI2_HOST, &dev, &mcp);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "SPI device: %s", esp_err_to_name(e));
        return false;
    }
    return true;
}

// ---------------------------------------------------------------- battery

static adc_oneshot_unit_handle_t adc_unit;
static adc_cali_handle_t adc_cali;
static adc_channel_t battery_channel;
static bool battery_ok;

static void battery_init(void) {
#if CONFIG_HANDTOOL_BATTERY_GPIO >= 0
    adc_unit_t unit;
    if (adc_oneshot_io_to_channel(CONFIG_HANDTOOL_BATTERY_GPIO, &unit, &battery_channel) != ESP_OK) {
        ESP_LOGW(TAG, "GPIO%d is not an ADC pin: no battery voltage", CONFIG_HANDTOOL_BATTERY_GPIO);
        return;
    }
    adc_oneshot_unit_init_cfg_t ucfg = {.unit_id = unit};
    if (adc_oneshot_new_unit(&ucfg, &adc_unit) != ESP_OK) return;
    adc_oneshot_chan_cfg_t ccfg = {.atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT};
    if (adc_oneshot_config_channel(adc_unit, battery_channel, &ccfg) != ESP_OK) return;
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cal = {.unit_id = unit, .chan = battery_channel, .atten = ADC_ATTEN_DB_12,
                                           .bitwidth = ADC_BITWIDTH_DEFAULT};
    if (adc_cali_create_scheme_curve_fitting(&cal, &adc_cali) != ESP_OK) adc_cali = NULL;
#endif
    battery_ok = true;
#endif
}

static float battery_read(void) {
    if (!battery_ok) return 0.0f;
    int sum = 0, n = 0;
    for (int i = 0; i < 8; i++) {
        int raw, mv;
        if (adc_oneshot_read(adc_unit, battery_channel, &raw) != ESP_OK) continue;
        if (adc_cali && adc_cali_raw_to_voltage(adc_cali, raw, &mv) == ESP_OK) sum += mv;
        else sum += raw * 3300 / 4095;   // uncalibrated estimate
        n++;
    }
    return n ? (float)sum / n * CONFIG_HANDTOOL_BATTERY_DIVIDER_X100 / 100.0f / 1000.0f : 0.0f;
}

// ---------------------------------------------------------------- zero calibration (NVS "handtool"/"zero")

static void zero_load(void) {
    memcpy(zero, SENSORS_DEFAULT_ZERO, sizeof zero);
    default_zero = true;
    nvs_handle_t h;
    if (nvs_open("handtool", NVS_READONLY, &h) != ESP_OK) return;
    int16_t z[6];
    size_t len = sizeof z;
    if (nvs_get_blob(h, "zero", z, &len) == ESP_OK && len == sizeof z) {
        memcpy(zero, z, sizeof z);
        default_zero = false;
    }
    nvs_close(h);
}

static bool zero_save(const int16_t z[6], bool is_default) {
    nvs_handle_t h;
    if (nvs_open("handtool", NVS_READWRITE, &h) != ESP_OK) return false;
    esp_err_t e = is_default ? nvs_erase_key(h, "zero") : nvs_set_blob(h, "zero", z, 6 * sizeof(int16_t));
    if (e == ESP_ERR_NVS_NOT_FOUND) e = ESP_OK;
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);
    return e == ESP_OK;
}

void sensors_get_zero(int16_t z[6]) {
    xSemaphoreTake(lock, portMAX_DELAY);
    memcpy(z, zero, sizeof zero);
    xSemaphoreGive(lock);
}

bool sensors_set_zero(const int16_t z[6]) {
    bool is_default = memcmp(z, SENSORS_DEFAULT_ZERO, sizeof zero) == 0;
    if (!zero_save(z, is_default)) return false;
    xSemaphoreTake(lock, portMAX_DELAY);
    memcpy(zero, z, sizeof zero);
    default_zero = is_default;
    xSemaphoreGive(lock);
    return true;
}

bool sensors_zero_here(void) {
    sensor_snapshot_t s;
    sensors_get(&s);
    if (!s.valid) return false;
    int16_t z[6];
    for (int i = 0; i < 6; i++) z[i] = (int16_t)lroundf(s.filtered[i]);
    return sensors_set_zero(z);
}

bool sensors_using_default_zero(void) {
    return default_zero;
}

// ---------------------------------------------------------------- sampling

static void sensor_task(void *arg) {
    const float dt = CONFIG_HANDTOOL_SAMPLE_MS / 1000.0f;
    const float alpha = dt / (CONFIG_HANDTOOL_FILTER_MS / 1000.0f + dt);
    float filtered[8] = {0};
    bool present = false, filter_ready = false;
    int bad_rounds = 0;
    int64_t next_battery = 0, next_probe = 0, rate_t0 = esp_timer_get_time();
    uint32_t rate_n = 0, seq = 0;
    float battery = 0.0f, rate = 0.0f;
    TickType_t wake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(CONFIG_HANDTOOL_SAMPLE_MS));
        int64_t now = esp_timer_get_time();
        if (now >= next_battery) {
            battery = battery_read();
            next_battery = now + 500000;
        }
        uint16_t raw[8];
        bool ok = mcp != NULL && mcp_read_all(raw);
        if (!present) {
            // Not found (yet): look again every 2 s, so a loose wire that is fixed is picked up without a reset.
            if (now < next_probe) continue;
            next_probe = now + 2000000;
            if (ok && !looks_absent(raw)) {
                present = true;
                filter_ready = false;
                bad_rounds = 0;
                ESP_LOGI(TAG, "MCP3008 found");
            } else {
                xSemaphoreTake(lock, portMAX_DELAY);
                snap.valid = false;
                snap.battery_v = battery;
                snap.rate_hz = 0;
                kinematics_identity(snap.T);
                xSemaphoreGive(lock);
                continue;
            }
        }
        if (!ok || looks_absent(raw)) {
            if (++bad_rounds >= 50) {           // half a second of nonsense: the chip is gone
                present = false;
                ESP_LOGW(TAG, "MCP3008 lost");
            }
            continue;
        }
        bad_rounds = 0;
        for (int ch = 0; ch < 8; ch++) {
            filtered[ch] = filter_ready ? alpha * raw[ch] + (1.0f - alpha) * filtered[ch] : raw[ch];
        }
        filter_ready = true;
        seq++;
        rate_n++;
        if (now - rate_t0 >= 1000000) {
            rate = rate_n * 1e6f / (float)(now - rate_t0);
            rate_n = 0;
            rate_t0 = now;
        }

        float angles[6], T[4][4];
        xSemaphoreTake(lock, portMAX_DELAY);
        for (int i = 0; i < 6; i++) {
            angles[i] = SENSORS_JOINT_DIRECTIONS[i] * (filtered[i] - zero[i]) * 270.0f / 1024.0f;
        }
        xSemaphoreGive(lock);
        kinematics_fk(angles, T);

        xSemaphoreTake(lock, portMAX_DELAY);
        snap.valid = true;
        memcpy(snap.raw, raw, sizeof raw);
        memcpy(snap.filtered, filtered, sizeof filtered);
        memcpy(snap.angles_deg, angles, sizeof angles);
        snap.button = raw[CONFIG_HANDTOOL_BUTTON_CHANNEL] < 512;
        memcpy(snap.T, T, sizeof T);
        snap.battery_v = battery;
        snap.rate_hz = rate;
        snap.seq = seq;
        xSemaphoreGive(lock);
    }
}

void sensors_get(sensor_snapshot_t *out) {
    xSemaphoreTake(lock, portMAX_DELAY);
    *out = snap;
    xSemaphoreGive(lock);
}

void sensors_start(void) {
    lock = xSemaphoreCreateMutex();
    kinematics_identity(snap.T);
    zero_load();
    battery_init();
    if (!mcp_init()) mcp = NULL;
    xTaskCreate(sensor_task, "sensors", 4096, NULL, 6, NULL);
}
