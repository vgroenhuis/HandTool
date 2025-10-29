#include "Display.h"
#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include "HandWifi.h"
#include "ToolModel.h"
#include "Kinematics.h"

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
    static long lastUpdate = 0;
    //long tm = millis();
    //int interval = tm - lastUpdate;

    if (!displayConnected) {
        return;
    }
    u8g2.clearBuffer();
    u8g2.setCursor(0, 10);
    u8g2.printf("IP: %s", wifi_ip());
    u8g2.setCursor(0,20);
    u8g2.printf("%s", WEB_ADDRESS);
    for (int i = 0; i < 6; i++) {
        u8g2.setCursor(i*20,30);
        u8g2.printf("%.1f", angles_deg[i]);
    }

    // Prepare joint angles array to display end-effector XYZ
    float joint_angles[6];
    for (int i = 0; i < 6; ++i) {
        joint_angles[i] = angles_deg[i];
    }
    
    // Compute forward kinematics
    Matrix4x4 T;
    // Initialize to identity matrix
    for (int i = 0; i < 16; ++i) T.m[i] = 0.0f;
    T.m[0] = 1.0f;
    T.m[5] = 1.0f;
    T.m[10] = 1.0f;
    T.m[15] = 1.0f;

    if (mcp3008_present) {
        T = compute_forward_kinematics(joint_angles);
    }

    u8g2.setCursor(0,40);
    u8g2.printf("X:%.1f Y:%.1f Z:%.1f", T.m[3]*1000, T.m[7]*1000, T.m[11]*1000);

    u8g2.setCursor(0,50);

    // u8g2.setCursor(0,30);
    // u8g2.printf("%.0f  %.0f  %.0f  %.0f  %.0f  %.0f",
    //     angles_deg[0], angles_deg[1], angles_deg[2],
    //     angles_deg[3], angles_deg[4], angles_deg[5]);
    //u8g2.setCursor(0,40);
    //u8g2.printf("%.1f Hz", 1000.0f / interval);
    u8g2.sendBuffer();

    //lastUpdate = tm;
}


void display_loop() {
    if (!displayConnected) {
        return;
    }
    static int lastUpdate = 0;
    if (millis() - lastUpdate >= 100) {    
        lastUpdate = millis();

        update_display();
/*
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
*/
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
