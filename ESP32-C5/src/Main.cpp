#include <Arduino.h>
#include "WifiServer.h"
#include "Adc.h"
#include "Display.h"
#include "Kinematics.h"
#include <LittleFS.h>
#include <esp_sleep.h>

void sleep_setup() {
    esp_sleep_enable_ext1_wakeup_io(1<<4, ESP_EXT1_WAKEUP_ANY_LOW);
}

void init_file_system() {
    if (!LittleFS.begin(true, "/littlefs", 10, "littlefs")) {
        Serial.println("LittleFS Mount Failed");
    } else {
        Serial.println("LittleFS Mounted Successfully");
    }
}

void setupHandTool() {
    Serial.begin(115200);
    delay(100);
    rgbLedWrite(LED_BUILTIN, 30, 10, 0); // Orange during setup
    sleep_setup();
    init_file_system();
    display_setup();
    robot_setup();
    kinematics_init();
    wifi_setup();
    rgbLedWrite(LED_BUILTIN, 0, 10, 0); // Green when setup complete
}



void setup() {
    setupHandTool();
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

void startDeepSleep() {
    rgbLedWrite(LED_BUILTIN, 0, 0, 0);
    display_off();
    delay(10);
    esp_deep_sleep_start();
}

void testDeepSleep() {
    if (digitalRead(4) == HIGH) {
        startDeepSleep();
    }

/*
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 1000) {
        lastPrint = millis();
        Serial.println("Awake!");
    }
*/
}

void loopHandTool() {
    sensors_loop();
    display_loop();
    wifi_loop();
    testDeepSleep();
    //printValuesPeriodically();
    //delay(1);
}

void loop() {
    loopHandTool();
}
