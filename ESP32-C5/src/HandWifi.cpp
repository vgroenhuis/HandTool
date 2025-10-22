#include "HandWifi.h"
#include <WiFi.h>
#include <WebServer.h>
#include <Arduino.h>
#include "ToolModel.h"

static const char* SSID = "iotroam";
static const char* PASS = "esproam1";
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
        setInterval(fetchValues,100);
        </script>
    </body>
</html>
)MY_STRING";

    String body = String(html_template);
    body.replace("{{MAC}}", mac);
    body.replace("{{IP}}", ip);
    server.send(200, "text/html", body);
}

void wifi_setup() {
    Serial.printf("MAC address: %s\n", wifi_mac());
    Serial.printf("Connecting to SSID: %s\n", SSID);

    WiFi.mode(WIFI_STA);
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
    } else {
        Serial.println("Failed to connect within timeout.");
    }

    // Start web server regardless (will still serve local info even if not connected)
    server.on("/", handleRoot);
    server.on("/values", handleValues);
    server.begin();
    Serial.println("HTTP server started on port 80");    
}

void wifi_loop() {
    server.handleClient();

    // simple auto-reconnect logic
    if (WiFi.status() != WL_CONNECTED) {
        if (millis() - s_lastReconnect > 5000) {
            s_lastReconnect = millis();
            if (SSID && PASS) WiFi.begin(SSID, PASS);
        }
    }
}