#pragma once
#include <SPI.h>
#include <Adafruit_MCP3008.h>

// MCP3008 configuration (SPI)

const int MCP_CS_PIN = 9;   // red
const int MCP_SCK_PIN  = 0; // green // 6 does not work on Waveshare, because of BAT ADC
const int MCP_MOSI_PIN = 8; // yellow
const int MCP_MISO_PIN = 7; // orange

const int ZERO_OFFSET[8] = {470, 488, 215, 530, 95, 380, 0, 0};
const int JOINT_DIRECTIONS[6] = {1, -1, -1, 1, 1, -1};

extern uint16_t raw_adc[8]; // raw values
extern float filtered_adc[8]; // low-pass filtered
extern float angles_deg[6]; // joint angles in degrees
extern bool mcp3008_present; // whether MCP3008 is detected and working
extern bool dragButtonPressed; // state of drag button

void robot_setup();
void sensors_loop();
bool is_mcp3008_present();
bool probe_mcp3008_presence();
float getBatteryLevel(); // in volt, by reading ADC1_CH5
