#include "ToolModel.h"

volatile uint16_t adc_values[8] = {0};

// Adafruit MCP3008 instance
Adafruit_MCP3008 mcp;

void robot_setup() {
    // Initialize SPI and Adafruit MCP3008 (use hardware SPI pins)
    SPI.begin(MCP_SCK_PIN, MCP_MISO_PIN, MCP_MOSI_PIN, MCP_CS_PIN);
    //if (!mcp.begin(MCP_SCK_PIN, MCP_MOSI_PIN, MCP_MISO_PIN, MCP_CS_PIN)) {
    if (!mcp.begin(MCP_CS_PIN, &SPI)) {
        Serial.println("Failed to initialize MCP3008");
    } else {
        Serial.println("MCP3008 initialized");
    }
}

void sensors_loop() {
    static unsigned long lastRead = 0;
    if (millis() - lastRead >= 10) {
        lastRead = millis();
        // Read all 8 MCP3008 channels
        for (uint8_t ch = 0; ch < 8; ++ch) {
            adc_values[ch] = mcp.readADC(ch);
        }
    }    
}
