// HandTool: a handheld 6-DoF input device (six potentiometers on an MCP3008, a drag button, an SH1106 OLED) on an
// ESP32-C5-DevKitC-1, ESP-IDF version of the Arduino firmware in ../ESP32-C5. Same sensors, maths, pages and endpoints;
// new: reporting to the CloSer lab demos page, over-the-air updates with rollback, Wi-Fi settings in flash (no
// RoamingWiFiManager), a 50 Hz WebSocket stream, zero calibration saved in flash, no halt when a part is missing.
#include "closer_demo.h"
#include "display.h"
#include "esp_log.h"
#include "sensors.h"
#include "web.h"

void app_main(void) {
    display_start();
    display_boot_message("starting...");
    closer_demo_start("handtool");     // NVS, Wi-Fi, clock, reporting, updates (in the background)
    sensors_start();                   // after closer_demo_start: reads its calibration from NVS
    web_start();
    display_run();
    ESP_LOGI("handtool", "running, firmware %s", closer_demo_version());
}
