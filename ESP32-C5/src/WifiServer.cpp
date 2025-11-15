#include "WifiServer.h"
#include <WiFi.h>
#include <WebServer.h>
#include <Arduino.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include "Adc.h"
#include <LittleFS.h>
#include "Display.h"
#include "Kinematics.h"

static unsigned long s_lastReconnect = 0;
static unsigned long s_lastStatusPing = 0;

WebServer server(80);
WiFiClientSecure client; // TLS required by server
HTTPClient heartbeatHttp;
bool heartbeatInitialized = false;

// Serial output control flags
bool serialOutputAngles = false;
bool serialOutputFK = false;
unsigned long lastSerialOutput = 0;
unsigned long serialOutputInterval = 100; // ms between outputs (default 10Hz)

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
        json += String(raw_adc[i]);
        if (i < 7) json += ",";
    }
    json += "]";
    json += "}";
    server.send(200, "application/json", json);
}

void handleFilteredAdcValues() {
    String json = "{";
    json += "\"valid\": ";
    json += mcp3008_present ? "true" : "false";
    json += ",";
    json += "\"values\": [";
    for (int i = 0; i < 8; ++i) {
        json += String(filtered_adc[i]);
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
    json += "\"dragButtonPressed\": ";
    json += dragButtonPressed ? "true" : "false";
    json += ",";
    json += "\"angles\": [";
    for (int i = 0; i < 6; ++i) {
        json += String(angles_deg[i], 2);
        if (i < 5) json += ",";
    }
    json += "]";
    json += "}";
    server.send(200, "application/json", json);
}

// HTTP handler for /fk - returns JSON with 4x4 transformation matrix from base to end-effector
void handleForwardKinematics() {
    //float joint_angles[6];
    //for (int i = 0; i < 6; ++i) {
    //   joint_angles[i] = angles_deg[i];
    //}
    
    // Compute forward kinematics
    Matrix4x4 T = IDENTITY_MATRIX;
    //matrix_identity(T);

    if (mcp3008_present) {
        T = compute_forward_kinematics(angles_deg);
    }

    // Build JSON response with 4x4 matrix structure
    String json = "{";
    json += "\"valid\": ";
    json += mcp3008_present ? "true" : "false";
    json += ",";
    json += "\"dragButtonPressed\": ";
    json += dragButtonPressed ? "true" : "false";
    json += ",";
    json += "\"matrix\": [";
    for (int row = 0; row < 4; ++row) {
        json += "[";
        for (int col = 0; col < 4; ++col) {
            json += String(T.m[row][col], 6);  // 6 decimal places
            if (col < 3) json += ",";
        }
        json += "]";
        if (row < 3) json += ",";
    }
    json += "]";
    json += "}";
    
    server.send(200, "application/json", json);
}

void handleAllData() {
    // Compute forward kinematics
    Matrix4x4 T = IDENTITY_MATRIX;
    //matrix_identity(T);

    if (mcp3008_present) {
        T = compute_forward_kinematics(angles_deg);
    }

    // Build JSON response with 4x4 matrix structure
    String json = "{";
    json += "\"valid\": ";
    json += mcp3008_present ? "true" : "false";
    json += ",";
    json += "\"dragButtonPressed\": ";
    json += dragButtonPressed ? "true" : "false";
    json += ",";
    json += "\"rawAdcValues\": [";
    for (int i = 0; i < 8; ++i) {
        json += String(raw_adc[i]);
        if (i < 7) json += ",";
    }
    json += "],";
    json += "\"filteredAdcValues\": [";
    for (int i = 0; i < 8; ++i) {
        json += String(filtered_adc[i]);
        if (i < 7) json += ",";
    }
    json += "],";
    json += "\"jointAngles\": [";
    for (int i = 0; i < 6; ++i) {
        json += String(angles_deg[i], 2);
        if (i < 5) json += ",";
    }
    json += "],";
    json += "\"matrix\": [";
    for (int row = 0; row < 4; ++row) {
        json += "[";
        for (int col = 0; col < 4; ++col) {
            json += String(T.m[row][col], 6);  // 6 decimal places
            if (col < 3) json += ",";
        }
        json += "]";
        if (row < 3) json += ",";
    }
    json += "]";
    json += "}";
    
    server.send(200, "application/json", json);    
}

// Handler to enable/disable serial output of joint angles
void handleSerialAngles() {
    if (server.hasArg("enable")) {
        serialOutputAngles = (server.arg("enable") == "1" || server.arg("enable") == "true");
        server.send(200, "text/plain", serialOutputAngles ? "Angles output enabled" : "Angles output disabled");
    } else {
        server.send(400, "text/plain", "Missing 'enable' parameter");
    }
}

// Handler to enable/disable serial output of FK data
void handleSerialFK() {
    if (server.hasArg("enable")) {
        serialOutputFK = (server.arg("enable") == "1" || server.arg("enable") == "true");
        server.send(200, "text/plain", serialOutputFK ? "FK output enabled" : "FK output disabled");
    } else {
        server.send(400, "text/plain", "Missing 'enable' parameter");
    }
}

// Handler to set serial output frequency
void handleSerialFrequency() {
    if (server.hasArg("hz")) {
        float hz = server.arg("hz").toFloat();
        if (hz > 0 && hz <= 1000) {
            serialOutputInterval = (unsigned long)(1000.0 / hz);
            String response = "Serial output frequency set to " + String(hz, 1) + " Hz (" + String(serialOutputInterval) + " ms)";
            server.send(200, "text/plain", response);
        } else {
            server.send(400, "text/plain", "Frequency must be between 0.1 and 1000 Hz");
        }
    } else {
        String response = "Current: " + String(1000.0 / serialOutputInterval, 1) + " Hz (" + String(serialOutputInterval) + " ms)";
        server.send(200, "text/plain", response);
    }
}

// Handler to return D-H parameters as JSON
void handleDHParams() {
    String json = "{";
    json += "\"dhParams\": [";
    for (int i = 0; i < 6; i++) {
        json += "{";
        json += "\"joint\": " + String(i + 1) + ",";
        json += "\"a\": " + String(dh_params[i].a, 6) + ",";
        json += "\"alpha_deg\": " + String(dh_params[i].alpha_deg, 2) + ",";
        json += "\"d\": " + String(dh_params[i].d, 6) + ",";
        json += "\"theta_deg\": " + String(dh_params[i].theta_deg, 2);
        json += "}";
        if (i < 5) json += ",";
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
                //Serial.printf("LittleFS: read /index.html %u bytes (cached)\n");
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

// HTTP handler to serve the robotView.html file from LittleFS
void handleRobotView() {
    static String cachedRobotViewHtml = "";
    static bool robotViewLoaded = false;

    if (!robotViewLoaded) {
        if (LittleFS.exists("/robotView.html")) {
            File f = LittleFS.open("/robotView.html", "r");
            if (f) {
                cachedRobotViewHtml = "";
                unsigned long t0 = millis();
                size_t bytes = 0;
                while (f.available()) {
                    char c = (char)f.read();
                    cachedRobotViewHtml += c;
                    bytes++;
                }
                unsigned long t1 = millis();
                f.close();
                unsigned long elapsed = t1 - t0;
                //Serial.printf("LittleFS: read /robotView.html %u bytes in %lu ms (cached)\n", (unsigned int)bytes, elapsed);
                robotViewLoaded = true;
            } else {
                Serial.println("Failed to open LittleFS /robotView.html!");
            }
        } else {
            Serial.println("LittleFS /robotView.html not found!");
        }
    }

    if (robotViewLoaded) {
        server.send(200, "text/html", cachedRobotViewHtml);
    } else {
        server.send(404, "text/plain", "robotView.html not found on LittleFS");
    }
}

bool sendHeartBeat() {
    //WiFiClientSecure client; // TLS required by server
    //client.setInsecure();    // Skip certificate validation to keep code small; still uses HTTPS transport
    //HTTPClient http;
    //if (http.begin(client, HEARTBEAT_URL)) {
    int code = heartbeatHttp.GET();
    if (code > 0) {
        // Optionally read body (not required). Keep minimal to avoid heap churn.
        // String payload = http.getString();
        //Serial.printf("Heartbeat ping OK (%d)\n", code);
        return true;
    } else {
        Serial.printf("Heartbeat GET failed: %s\n", heartbeatHttp.errorToString(code).c_str());
        return false;
    }
    //heartbeatHttp.end();
    //} else {
    //    Serial.println("Heartbeat GET begin() failed");
    //}
}


void wifi_setup() {
    WiFi.mode(WIFI_STA);

    //Serial.printf("MAC address: %s\n", wifi_mac());
    //Serial.printf("Connecting to SSID: %s\n", SSID);
    display_message("MAC: " + String(wifi_mac()), "Connecting to " + String(SSID));

    WiFi.setSleep(false);
    WiFi.setBandMode(WIFI_BAND_MODE_5G_ONLY);
    WiFi.setHostname(HOSTNAME);

    if (SSID && PASS) WiFi.begin(SSID, PASS);

    // small wait for initial connect
    unsigned long start = millis();
    const unsigned long timeout = 10000;
    while (!wifi_is_connected() && (millis() - start) < timeout) {
        //Serial.print('.');
        delay(500);
    }
    Serial.println();

    if (wifi_is_connected()) {
        //Serial.printf("Connected! IP address: %s\n", wifi_ip());
        update_display();
    } else {
        Serial.println("Failed to connect within timeout.");
        display_message("Failed to connect!");
    }

    // Start web server regardless (will still serve local info even if not connected)
    server.on("/", handleRoot);
    server.on("/index.html", handleRoot);
    server.on("/robotView.html", handleRobotView);
    server.on("/allData", handleAllData);
    server.on("/angles", handleAngles);
    server.on("/rawAdcValues", handleRawAdcValues);
    server.on("/filteredAdcValues", handleFilteredAdcValues);
    server.on("/fk", handleForwardKinematics);
    server.on("/serialAngles", handleSerialAngles);
    server.on("/serialFK", handleSerialFK);
    server.on("/serialFrequency", handleSerialFrequency);
    server.on("/dhParams", handleDHParams);
    server.begin();
    //Serial.printf("HTTP server started on port 80\n");

    client.setInsecure();
    if (heartbeatHttp.begin(client, HEARTBEAT_URL)) {
        //Serial.println("heartbeatHttp begin() success");
        heartbeatInitialized = true;
        heartbeatHttp.setReuse(true);
    } else {
        Serial.println("heartbeatHttp begin() failed!");
        heartbeatInitialized = false;
    }
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

    // Serial output of joint angles and/or FK data
    if ((serialOutputAngles || serialOutputFK) && mcp3008_present) {
        unsigned long now = millis();
        if (now - lastSerialOutput >= serialOutputInterval) {
            lastSerialOutput = now;
            
            if (serialOutputAngles) {
                Serial.print("$ANGLES: ");
                Serial.print(millis());
                Serial.print(", ");
                for (int i = 0; i < 6; i++) {
                    Serial.print(angles_deg[i], 3);
                    if (i < 5) Serial.print(", ");
                }
                Serial.println();
            }
            
            if (serialOutputFK) {
                Matrix4x4 T = compute_forward_kinematics(angles_deg);
                Serial.print("$FK: ");
                Serial.print(millis());
                Serial.print(", ");
                for (int row = 0; row < 4; row++) {
                    for (int col = 0; col < 4; col++) {
                        Serial.print(T.m[row][col], 6);
                        if (row < 3 || col < 3) Serial.print(", ");
                    }
                }
                Serial.println();
            }
        }
    }

    // Periodically report device status every 10 seconds while connected
    if (SEND_HEARTBEAT && heartbeatInitialized && wifi_is_connected()) {
        unsigned long now = millis();
        if (now - s_lastStatusPing >= 10000UL || s_lastStatusPing == 0) {
            //Serial.println("WiFi connected - sending heartbeat ping");
            
            bool success = sendHeartBeat();
            if (success) {
                s_lastStatusPing = now;
            }
            unsigned long tm2 = millis();
            if (success) {
                Serial.printf("Heartbeat OK, ping took %lu ms\n", tm2 - now);
            }
        }
    }
}
