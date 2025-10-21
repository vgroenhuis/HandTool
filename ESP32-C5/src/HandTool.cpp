// HandTool.cpp
// Minimal ESP32-C5 web server: print MAC to Serial, connect to SSID "iotroam" and serve a simple page.

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

static const char* SSID = "iotroam";
static const char* PASS = "esproam1"; // set password if required

WebServer server(80);

void handleRoot() {
    String mac = WiFi.macAddress();
    String ip = WiFi.localIP().toString();
    String body = "<!doctype html><html><head><meta charset='utf-8'><title>ESP32-C5</title></head><body>";
    body += "<h1>ESP32-C5 Web Server</h1>";
    body += "<p>MAC: " + mac + "</p>";
    body += "<p>IP: " + ip + "</p>";
    body += "</body></html>";
    server.send(200, "text/html", body);
}

void setup() {
    Serial.begin(115200);
    //delay(100);

    // Start WiFi in station mode and connect
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.setBandMode(WIFI_BAND_MODE_5G_ONLY);

    // WiFi.disconnect(true); // clear prior configs
    //delay(100);

    // Print MAC address to Serial
    String mac = WiFi.macAddress();
    Serial.printf("MAC address: %s\n", mac.c_str());

    Serial.printf("Connecting to SSID: %s\n", SSID);
    WiFi.begin(SSID, PASS);

    unsigned long start = millis();
    const unsigned long timeout = 20000; // 20 seconds
    while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeout) {
        Serial.print(".");
        delay(500);
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("Connected! IP address: %s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("Failed to connect within timeout.");
    }

    // Start web server regardless (will still serve local info even if not connected)
    server.on("/", handleRoot);
    server.begin();
    Serial.println("HTTP server started on port 80");
}

void loop() {
    server.handleClient();

    // Simple auto-reconnect if disconnected
    if (WiFi.status() != WL_CONNECTED) {
        static unsigned long lastAttempt = 0;
        if (millis() - lastAttempt > 5000) { // try every 5s
            lastAttempt = millis();
            Serial.println("Reconnecting to WiFi...");
            WiFi.begin(SSID, PASS);
        }
    }

    delay(1);
}