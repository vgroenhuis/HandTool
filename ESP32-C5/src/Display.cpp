#include "Display.h"
#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include "WifiServer.h"
#include "Adc.h"
#include "Kinematics.h"

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, /*SCL=*/3, /*SDA=*/2);

bool displayConnected = false;

void display_off() {
    if (!displayConnected) {
        return;
    }
    u8g2.clearBuffer();
    u8g2.sendBuffer();
}

bool display_is_connected() {
    // SH1106 typically has address 0x3C or 0x3D
    Wire.begin(2, 3); // SDA=GPIO2, SCL=GPIO3 (matching the u8g2 initialization)
    Wire.beginTransmission(0x3C);
    byte error = Wire.endTransmission();
    
    if (error == 0) {
        //Serial.println("Display found at address 0x3C");
        return true;
    }
    
    // Try alternate address
    Wire.beginTransmission(0x3D);
    error = Wire.endTransmission();
    
    if (error == 0) {
        //Serial.println("Display found at address 0x3D");
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
    // u8g2.enableUTF8Print(); // Not needed for ASCII-only output; saves flash
    u8g2.clearBuffer();
    // Use a very compact built-in font to reduce flash footprint
    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.setCursor(0, 10);
    u8g2.printf("HandTool Display");
    u8g2.setCursor(0, 20);
#ifndef DISABLE_WIFI
    u8g2.printf("%s", manager.getMAC().c_str());
#endif
    u8g2.sendBuffer();
    //Serial.printf("Display initialized\n");
}

void update_display() {
    static long lastUpdate = 0;
    //long tm = millis();
    //int interval = tm - lastUpdate;

    if (!displayConnected) {
        return;
    }
    u8g2.clearBuffer();
    u8g2.setCursor(0, 10);
#ifndef DISABLE_WIFI
    u8g2.printf("IP: %s", manager.getConnectedIp().c_str());
#endif
    u8g2.setCursor(0,20);
    u8g2.printf("%s", WEB_ADDRESS);
    for (int i = 0; i < 6; i++) {
        u8g2.setCursor(i*20,30);
        u8g2.printf("%d", raw_adc[i]);
    }

    for (int i = 0; i < 6; i++) {
        u8g2.setCursor(i*20,40);
        u8g2.printf("%.0f", angles_deg[i]);
    }

    // Compute forward kinematics
    Matrix4x4 T = IDENTITY_MATRIX;

    if (mcp3008_present) {
        T = compute_forward_kinematics(angles_deg);
    }

    u8g2.setCursor(0,50);
    u8g2.printf("X:%.1f Y:%.1f Z:%.1f", T.m[0][3]*1000, T.m[1][3]*1000, T.m[2][3]*1000);

    u8g2.setCursor(0,60);
    if (!mcp3008_present) {
        u8g2.printf("MCP3008 fail! ");
    }
    u8g2.printf("Btn:%s", dragButtonPressed ? "P" : "");
    u8g2.sendBuffer();
}


void display_loop() {
    if (!displayConnected) {
        return;
    }
    static int lastUpdate = 0;
    if (millis() - lastUpdate >= 100) {    
        lastUpdate = millis();
        update_display();
    }
}

void display_message(String msg, String msg2) {
    if (!displayConnected) {
        return;
    }
    u8g2.clearBuffer();
    u8g2.setCursor(0, 10);
    u8g2.print(msg);
    if (msg2 != "") {
        u8g2.setCursor(0, 20);
        u8g2.print(msg2);
    }
    u8g2.sendBuffer();
}
