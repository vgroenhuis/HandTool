// HandTool.cpp
// Minimal ESP32-C5 web server: print MAC to Serial, connect to SSID "iotroam" and serve a simple page.

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SPI.h>
#include <Adafruit_MCP3008.h>

// MCP3008 configuration (SPI)
const int MCP_MOSI_PIN = 8;
const int MCP_MISO_PIN = 7;
const int MCP_SCK_PIN  = 6;
const int MCP_CS_PIN = 10;

// Read buffer for channels
volatile uint16_t adc_values[8] = {0};

// Adafruit MCP3008 instance
Adafruit_MCP3008 mcp;

static const char* SSID = "iotroam";
static const char* PASS = "esproam1"; // set password if required

WebServer server(80);


// HTTP handler for /values - returns JSON with adc readings
void handleValues() {
    String json = "{";
    json += "\"values\": [";
    for (int i = 0; i < 8; ++i) {
        json += String(adc_values[i]);
        if (i < 7) json += ",";
    }
    json += "]";
    json += "}";
    server.send(200, "application/json", json);
}

void handleRoot() {
    String mac = WiFi.macAddress();
    String ip = WiFi.localIP().toString();
        // Use a raw string literal for the HTML and replace placeholders with runtime values
        static const char html_template[] = R"MY_STRING(<!doctype html>
<html>
    <head>
        <meta charset='utf-8'>
        <title>ESP32-C5</title>
        <meta name="viewport" content="width=device-width,initial-scale=1">
        <style>body{font-family:Arial,Helvetica,sans-serif;margin:12px;}img{border:1px solid #ddd;padding:4px;background:#fff}</style>
    </head>
    <body>
        <h1>ESP32-C5 Web Server</h1>
        <p>MAC: {{MAC}}</p>
        <p>IP: {{IP}}</p>
        <h2>HandTool</h2>
        <p><img src="https://raw.githubusercontent.com/vgroenhuis/HandTool/refs/heads/main/pics/HandTool_500px.jpg" alt="HandTool" style="max-width:100%;height:auto;"></p>
        <h3>Analog inputs (MCP3008)</h3>
        <div id="adc">Loading...</div>
        <script>
        async function fetchValues(){
            try{
                const r = await fetch('/values');
                const j = await r.json();
                const el = document.getElementById('adc');
                let html = '<ul>';
                for(let i=0;i<8;i++) html += `<li>CH${i}: ${j.values[i]}</li>`;
                html += '</ul>';
                el.innerHTML = html;
            }catch(e){ console.error(e); }
        }
        fetchValues();
        setInterval(fetchValues,1000);
        </script>
    </body>
</html>
)MY_STRING";

    String body = String(html_template);
    body.replace("{{MAC}}", mac);
    body.replace("{{IP}}", ip);
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
    server.on("/values", handleValues);
    server.begin();
    Serial.println("HTTP server started on port 80");

    // Initialize SPI and Adafruit MCP3008 (use hardware SPI pins)
    SPI.begin(MCP_SCK_PIN, MCP_MISO_PIN, MCP_MOSI_PIN, MCP_CS_PIN);
    //if (!mcp.begin(MCP_SCK_PIN, MCP_MOSI_PIN, MCP_MISO_PIN, MCP_CS_PIN)) {
    if (!mcp.begin(MCP_CS_PIN, &SPI)) {
        Serial.println("Failed to initialize MCP3008");
    } else {
        Serial.println("MCP3008 initialized");
    }
}

void loop() {
    server.handleClient();

    static unsigned long lastRead = 0;
    if (millis() - lastRead >= 1000) {
        lastRead = millis();
        // Read all 8 MCP3008 channels
        for (uint8_t ch = 0; ch < 8; ++ch) {
            adc_values[ch] = mcp.readADC(ch);
        }
        // Print ADC readings to Serial
        Serial.print("ADC:");
        for (uint8_t ch = 0; ch < 8; ++ch) {
            Serial.printf(" CH%u=%u", ch, adc_values[ch]);
        }
        Serial.println();
    }

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