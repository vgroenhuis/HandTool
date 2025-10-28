#pragma once
#include <SPI.h>
#include <Adafruit_MCP3008.h>

// MCP3008 configuration (SPI)
const int MCP_MOSI_PIN = 8;
const int MCP_MISO_PIN = 7;
const int MCP_SCK_PIN  = 6;
const int MCP_CS_PIN = 10;

const int ZERO_OFFSET[8] = {273, 488, 215, 530, 100, 485, 0, 0};
const int JOINT_DIRECTIONS[6] = {1, -1, -1, 1, 1, -1};

extern volatile uint16_t adc_values[8]; // raw
extern volatile float filtered_adc[8]; // low-pass filtered
extern volatile float angles_deg[6]; // joint angles in degrees

void robot_setup();
void sensors_loop();
