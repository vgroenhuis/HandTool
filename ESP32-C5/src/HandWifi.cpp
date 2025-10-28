#include "HandWifi.h"
#include <WiFi.h>
#include <WebServer.h>
#include <Arduino.h>
#include "ToolModel.h"
#include <LittleFS.h>
#include "Files.h"
#include "Display.h"
#include "Kinematics.h"

static const char* SSID = "iotroam";
static const char* PASS = "handtool";
static const char* HOSTNAME = "HandTool";

static unsigned long s_lastReconnect = 0;

WebServer server(80);

const char* wifi_mac() {
    static String m;
    m = WiFi.macAddress();
    return m.c_str();
}

const char* wifi_ip() {
    static String ip;
    ip = WiFi.localIP().toString();
    return ip.c_str();
}

bool wifi_is_connected() {
    return WiFi.status() == WL_CONNECTED;
}

// HTTP handler for /rawAdcValues - returns JSON with raw ADC readings
void handleRawAdcValues() {
    String json = "{";
    json += "\"valid\": ";
    json += mcp3008_present ? "true" : "false";
    json += ",";
    json += "\"values\": [";
    for (int i = 0; i < 8; ++i) {
        json += String(adc_values[i]);
        if (i < 7) json += ",";
    }
    json += "]";
    json += "}";
    server.send(200, "application/json", json);
}

// HTTP handler for /angles - returns JSON with six joint angles in degrees
void handleAngles() {
    String json = "{";
    json += "\"valid\": ";
    json += mcp3008_present ? "true" : "false";
    json += ",";
    json += "\"angles\": [";
    for (int i = 0; i < 6; ++i) {
        // angles_deg may be volatile floats
        json += String(angles_deg[i], 2);
        if (i < 5) json += ",";
    }
    json += "]";
    json += "}";
    server.send(200, "application/json", json);
}

// HTTP handler for /fk - returns JSON with 4x4 transformation matrix from base to end-effector
void handleForwardKinematics() {
    // Copy volatile angles to local array
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

    // Build JSON response with 4x4 matrix structure
    String json = "{";
    json += "\"valid\": ";
    json += mcp3008_present ? "true" : "false";
    json += ",";
    json += "\"matrix\": [";
    for (int row = 0; row < 4; ++row) {
        json += "[";
        for (int col = 0; col < 4; ++col) {
            int idx = row * 4 + col;  // row-major indexing
            json += String(T.m[idx], 6);  // 6 decimal places
            if (col < 3) json += ",";
        }
        json += "]";
        if (row < 3) json += ",";
    }
    json += "]";
    json += "}";
    
    server.send(200, "application/json", json);
}

void handleRoot() {
    static String cachedIndexHtml = "";
    static bool isLoaded = false;
    
    // Load the file only once
    if (!isLoaded) {
        if (LittleFS.exists("/index.html")) {
            File f = LittleFS.open("/index.html", "r");
            if (f) {
                // read entire file into cachedIndexHtml
                cachedIndexHtml = "";
                size_t bytes = 0;
                while (f.available()) {
                    char c = (char)f.read();
                    cachedIndexHtml += c;
                    bytes++;
                }
                f.close();
                Serial.printf("LittleFS: read /index.html %u bytes (cached)\n");
                isLoaded = true;
            } else {
                Serial.println("Failed to open LittleFS /index.html!");
            }
        } else {
            Serial.println("LittleFS /index.html not found!");
        }
    }
    
    // Serve the cached content
    if (isLoaded) {
        server.send(200, "text/html", cachedIndexHtml);
    } else {
        server.send(404, "text/plain", "index.html not found");
    }
}

// forward declaration so wifi_setup can register the handler
void handleRobotView();

void wifi_setup() {
    WiFi.mode(WIFI_STA);

    Serial.printf("MAC address: %s\n", wifi_mac());
    Serial.printf("Connecting to SSID: %s\n", SSID);
    display_message("Connecting to " + String(SSID));

    WiFi.setSleep(false);
    WiFi.setBandMode(WIFI_BAND_MODE_5G_ONLY);
    WiFi.setHostname(HOSTNAME);

    if (SSID && PASS) WiFi.begin(SSID, PASS);

    // small wait for initial connect
    unsigned long start = millis();
    const unsigned long timeout = 10000;
    while (!wifi_is_connected() && (millis() - start) < timeout) {
        Serial.print('.');
        delay(500);
    }
    Serial.println();

    if (wifi_is_connected()) {
        Serial.printf("Connected! IP address: %s\n", wifi_ip());
        update_display();
    } else {
        Serial.println("Failed to connect within timeout.");
        display_message("Failed to connect!");
    }

    // Start web server regardless (will still serve local info even if not connected)
    server.on("/", handleRoot);
    server.on("/index.html", handleRoot);
    server.on("/rawAdcValues", handleRawAdcValues);
    server.on("/robotView.html", handleRobotView);
    server.on("/angles", handleAngles);
    server.on("/fk", handleForwardKinematics);
    server.begin();
    Serial.printf("HTTP server started on port 80\n");
}

void wifi_loop() {
    server.handleClient();

    // simple auto-reconnect logic
    if (WiFi.status() != WL_CONNECTED) {
        if (millis() - s_lastReconnect > 5000) {
            s_lastReconnect = millis();
            if (SSID && PASS) WiFi.begin(SSID, PASS);
            update_display();
        }
    }
}

// HTTP handler to serve the robotView.html file from LittleFS
void handleRobotView() {
    if (LittleFS.exists("/robotView.html")) {
        File f = LittleFS.open("/robotView.html", "r");
        if (f) {
            String body = "";
            unsigned long t0 = millis();
            size_t bytes = 0;
            while (f.available()) {
                char c = (char)f.read();
                body += c;
                bytes++;
            }
            unsigned long t1 = millis();
            f.close();
            unsigned long elapsed = t1 - t0;
            Serial.printf("LittleFS: read /robotView.html %u bytes in %lu ms\n", (unsigned int)bytes, elapsed);
            server.send(200, "text/html", body);
            return;
        } else {
            Serial.println("Failed to open LittleFS /robotView.html!");
        }
    } else {
        Serial.println("LittleFS /robotView.html not found!");
    }

    // fallback: try to serve a minimal message
    server.send(404, "text/plain", "robotView.html not found on LittleFS");
}

