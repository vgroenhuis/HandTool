#include "ToolModel.h"

volatile uint16_t adc_values[8] = {0};
volatile float filtered_adc[8] = {0};
volatile float angles_deg[6] = {0.0};
volatile bool mcp3008_present = false;

// Low-pass filter state (exponential moving average)
bool filter_initialized = false;

// Adafruit MCP3008 instance
Adafruit_MCP3008 mcp;

bool is_mcp3008_present() {
    return mcp3008_present;
}

// Heuristic probe: perform multiple reads on all channels and check for clearly invalid patterns
// Returns true if readings look plausible (not all 0x000 or 0x3FF across channels and time)
bool probe_mcp3008_presence() {
    const int attempts = 4;
    uint16_t minVal = 1023;
    uint16_t maxVal = 0;
    bool allSameAcrossChannels = true;
    uint16_t firstSample[8];

    // Initial fill
    for (uint8_t ch = 0; ch < 8; ++ch) {
        firstSample[ch] = mcp.readADC(ch);
        minVal = min(minVal, firstSample[ch]);
        maxVal = max(maxVal, firstSample[ch]);
        if (ch > 0 && firstSample[ch] != firstSample[ch - 1]) {
            allSameAcrossChannels = false;
        }
    }

    // Subsequent attempts
    for (int a = 1; a < attempts; ++a) {
        delay(2);
        for (uint8_t ch = 0; ch < 8; ++ch) {
            uint16_t v = mcp.readADC(ch);
            minVal = min(minVal, v);
            maxVal = max(maxVal, v);
            if (v != firstSample[ch]) {
                allSameAcrossChannels = false; // readings change over time or differ per channel
            }
        }
    }

    // If everything reads exactly 0x000 or exactly 0x3FF, highly likely no chip (floating MISO)
    if (minVal == 0 && maxVal == 0) return false;
    if (minVal == 1023 && maxVal == 1023) return false;

    // If all 8 channels match exactly and never change over several attempts, likely not present
    // (real sensors and noise usually cause small differences)
    // if (allSameAcrossChannels) return false;

    return true;
}

void robot_setup() {
    // Initialize SPI and Adafruit MCP3008 (use hardware SPI pins)
    SPI.begin(MCP_SCK_PIN, MCP_MISO_PIN, MCP_MOSI_PIN, MCP_CS_PIN);
    if (!mcp.begin(MCP_CS_PIN, &SPI)) {
        Serial.println("Failed to initialize MCP3008");
        mcp3008_present = false;
    } else {
        // Perform a few reads to verify presence over SPI
        bool ok = probe_mcp3008_presence();
        mcp3008_present = ok;
        if (ok) {
            Serial.println("MCP3008 detected via SPI probe");
        } else {
            Serial.println("WARNING: MCP3008 not detected (SPI reads look invalid)");
        }
    }
}

void sensors_loop() {
    // Skip reading if MCP3008 is not present
    if (!mcp3008_present) {
        return;
    }
    
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
