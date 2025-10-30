#pragma once

#define SEND_HEARTBEAT false

static const char* SSID = "iotroam";
static const char* PASS = "handtool";
static const char* HOSTNAME = "HandTool";
static const char* WEB_ADDRESS = "handtool.roaming.utwente.nl";

void wifi_setup();
void wifi_loop();

const char* wifi_ip();
