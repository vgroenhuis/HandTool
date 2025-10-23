#include "Display.h"
#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include "HandWifi.h"
#include "ToolModel.h"

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, 3, 2);

void display_setup() {
    u8g2.begin();
    u8g2.enableUTF8Print(); // Enable UTF-8 support for Arduino print functions
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenR08_tr);
    u8g2.drawStr(0,0,"HandTool Display");
    u8g2.sendBuffer();
    Serial.printf("Display initialized\n");
}

void update_display() {
    u8g2.clearBuffer();
    u8g2.setCursor(0, 10);
    u8g2.printf("IP: %s", wifi_ip());        
    u8g2.sendBuffer();
}


void display_loop() {
    static int lastUpdate = 0;
    if (millis() - lastUpdate >= 100) {    
        lastUpdate = millis();

        u8g2.clearBuffer();
        u8g2.setCursor(0, 10);
        u8g2.printf("IP: %s", wifi_ip());        

        u8g2.setCursor(0, u8g2.getCursorY() + 12);
        for (uint8_t ch = 0; ch < 6; ++ch) {
            u8g2.printf("%u ", adc_values[ch]);
        }
        u8g2.setCursor(0, u8g2.getCursorY() + 12);
        u8g2.printf("Time: %0.1f s", millis() / 1000.0f);
        u8g2.sendBuffer();
    }
}

void display_message(String msg) {
    u8g2.clearBuffer();
    u8g2.setCursor(0, 10);
    u8g2.print(msg);
    u8g2.sendBuffer();
}
