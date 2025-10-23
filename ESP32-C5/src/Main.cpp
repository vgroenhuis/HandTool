#include <Arduino.h>
#include "HandWifi.h"
#include "ToolModel.h"
#include "Display.h"
#include "Files.h"
#include "Kinematics.h"

void setup() {
    Serial.begin(115200);
    delay(100);

    init_file_system();
    display_setup();
    robot_setup();
    kinematics_init();
    wifi_setup();
}

void printValuesPeriodically() {
    static unsigned long lastPrinted = 0;
    if (millis() - lastPrinted >= 1000) {
        lastPrinted = millis();
        // Print ADC readings to Serial
        Serial.print("ADC:");
        for (uint8_t ch = 0; ch < 8; ++ch) {
            Serial.printf(" CH%u=%u", ch, adc_values[ch]);
        }
        Serial.println();
    }
}

void loop() {
    sensors_loop();
    //display_loop(); // 30 ms
    //printValuesPeriodically();
    wifi_loop();
    delay(1);
}