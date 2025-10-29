#include <Arduino.h>
#include "WifiServer.h"
#include "Adc.h"
#include "Display.h"
#include "Kinematics.h"
#include <LittleFS.h>

void init_file_system() {
    if (!LittleFS.begin(true)) {
        Serial.println("LittleFS Mount Failed");
    } else {
        Serial.println("LittleFS Mounted Successfully");
    }
}

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
        Serial.print("Raw ADC:");
        for (uint8_t ch = 0; ch < 8; ++ch) {
            Serial.printf(" CH%u=%u", ch, raw_adc[ch]);
        }
        Serial.println();
    }
}

void loop() {
    sensors_loop();
    display_loop();
    //printValuesPeriodically();
    wifi_loop();
    //delay(1);
}