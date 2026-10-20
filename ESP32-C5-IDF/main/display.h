// The 1.3" SH1106 OLED (128x64, I2C): address, Wi-Fi, joint angles, end-effector position, battery, problems.
#pragma once
#include <stdbool.h>

void display_start(void);                       // finds the display (carries on without one)
void display_boot_message(const char *text);
void display_run(void);                         // starts the refresh task (10 Hz)
bool display_present(void);
