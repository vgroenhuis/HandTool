#pragma once

#include "RoamingWiFiManager.h"

#define SEND_HEARTBEAT false

static const char* SSID = "iotroam";
static const char* PASS = "handtool";
static const char* HOSTNAME = "HandTool";
static const char* WEB_ADDRESS = "handtool.roaming.utwente.nl";
static const char* HEARTBEAT_URL = "https://www.vincentgroenhuis.nl/devices/device_heartbeat.php?id=HandTool";

extern RoamingWiFiManager manager;


void wifi_setup();
void wifi_loop();

//const char* wifi_ip();
