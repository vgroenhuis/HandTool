#include "Display.h"
#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include "HandWifi.h"
#include "ToolModel.h"

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, /*SCL=*/3, /*SDA=*/2);

bool displayConnected = false;

bool display_is_connected() {
    // SH1106 typically has address 0x3C or 0x3D
    Wire.begin(2, 3); // SDA=GPIO2, SCL=GPIO3 (matching the u8g2 initialization)
    Wire.beginTransmission(0x3C);
    byte error = Wire.endTransmission();
    
    if (error == 0) {
        Serial.println("Display found at address 0x3C");
        return true;
    }
    
    // Try alternate address
    Wire.beginTransmission(0x3D);
    error = Wire.endTransmission();
    
    if (error == 0) {
        Serial.println("Display found at address 0x3D");
        return true;
    }
    
    Serial.println("Display NOT found on I2C bus");
    return false;
}

void display_setup() {
    // Check if display is connected
    displayConnected = display_is_connected();

    if (!displayConnected) {
        Serial.printf("Display not connected, skipping initialization\n");
        return;
    }
    
    u8g2.begin();
    u8g2.enableUTF8Print(); // Enable UTF-8 support for Arduino print functions
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenR08_tr);
    u8g2.drawStr(0,0,"HandTool Display");
    u8g2.sendBuffer();
    Serial.printf("Display initialized\n");
}

void update_display() {
    if (!displayConnected) {
        return;
    }
    u8g2.clearBuffer();
    u8g2.setCursor(0, 10);
    u8g2.printf("IP: %s", wifi_ip());        
    u8g2.sendBuffer();
}


void display_loop() {
    if (!displayConnected) {
        return;
    }
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
    if (!displayConnected) {
        return;
    }
    u8g2.clearBuffer();
    u8g2.setCursor(0, 10);
    u8g2.print(msg);
    u8g2.sendBuffer();
}
