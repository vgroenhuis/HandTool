#pragma once
#include <Arduino.h>

void display_off();
void display_setup();
void display_loop();
void update_display();
void display_message(String msg, String msg2="");
bool display_is_connected();