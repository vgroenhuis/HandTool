#include "ToolModel.h"

volatile uint16_t adc_values[8] = {0};
volatile float filtered_adc[8] = {0};
volatile float angles_deg[6] = {0.0};

// Low-pass filter state (exponential moving average)
bool filter_initialized = false;

// Adafruit MCP3008 instance
Adafruit_MCP3008 mcp;

void robot_setup() {
    // Initialize SPI and Adafruit MCP3008 (use hardware SPI pins)
    SPI.begin(MCP_SCK_PIN, MCP_MISO_PIN, MCP_MOSI_PIN, MCP_CS_PIN);
    if (!mcp.begin(MCP_CS_PIN, &SPI)) {
        Serial.println("Failed to initialize MCP3008");
    } else {
        Serial.println("MCP3008 initialized");
    }
}

void sensors_loop() {
    static unsigned long lastRead = 0;
    const unsigned long SAMPLE_INTERVAL_MS = 10;
    const float TIME_CONSTANT = 0.1;  // 0.1 seconds
    
    if (millis() - lastRead >= SAMPLE_INTERVAL_MS) {
        lastRead = millis();
        
        // Calculate filter coefficient (alpha) for exponential moving average
        // alpha = dt / (tau + dt), where tau is the time constant and dt is sample interval
        float dt = SAMPLE_INTERVAL_MS / 1000.0;  // Convert to seconds
        float alpha = dt / (TIME_CONSTANT + dt);
        
        // Read all 8 MCP3008 channels and apply low-pass filter
        for (uint8_t ch = 0; ch < 8; ++ch) {
            uint16_t raw_value = mcp.readADC(ch);
            
            // Initialize filter on first read
            if (!filter_initialized) {
                filtered_adc[ch] = raw_value;
            } else {
                // Exponential moving average: y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
                filtered_adc[ch] = alpha * raw_value + (1.0 - alpha) * filtered_adc[ch];
            }
            
            // Store filtered value
            adc_values[ch] = (uint16_t)(filtered_adc[ch] + 0.5);  // Round to nearest integer
            
            if (ch < 6) {
                angles_deg[ch] = JOINT_DIRECTIONS[ch] * (filtered_adc[ch] - ZERO_OFFSET[ch]) * 270.0 / 1024.0;  // Convert to degrees
            }
        }
        
        filter_initialized = true;
    }    
}
