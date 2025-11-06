#include <Arduino.h>
#include "WifiServer.h"
#include "Adc.h"
#include "Display.h"
#include "Kinematics.h"
#include <LittleFS.h>
#include <esp_sleep.h>

void sleep_setup() {
    esp_sleep_enable_ext1_wakeup_io(1<<4, ESP_EXT1_WAKEUP_ANY_LOW);
    esp_sleep_enable_timer_wakeup(30 * 60 * 1000000); // 30 minutes
}

bool isPowerSwitchOn() {
    return digitalRead(4) == LOW;
}

void init_file_system() {
    if (!LittleFS.begin(true, "/littlefs", 10, "littlefs")) {
        Serial.println("LittleFS Mount Failed");
    } else {
        //Serial.println("LittleFS Mounted Successfully");
    }
}

void setupHandTool() {
    rgbLedWrite(LED_BUILTIN, 255, 255, 255);
    Serial.begin(115200);
    delay(100);
    sleep_setup();
    init_file_system();
    display_setup();
    robot_setup();
    kinematics_init();
    if (isPowerSwitchOn()) {
        rgbLedWrite(LED_BUILTIN, 30, 10, 0); // Orange during setup
        wifi_setup();
    }
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
    delay(10);
    rgbLedWrite(LED_BUILTIN, 0, 0, 0);
    display_off();
    delay(10);
    esp_deep_sleep_start();
}

void testDeepSleep() {
    if (!isPowerSwitchOn()) {
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
    testDeepSleep();
    sensors_loop();
    display_loop();
    wifi_loop();
    //printValuesPeriodically();
    //delay(1);
}

void loop() {
    loopHandTool();
}
