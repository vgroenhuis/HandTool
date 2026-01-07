#pragma once

#include "RoamingWiFiManager.h"

#if __has_include("WiFiCredentials.h")
    #include "WiFiCredentials.h"
#else
    #define WIFI_CREDENTIALS {{"your-ssid","your-password"},{"your-ssid2","your-password2"}}
    #define ADMIN_CREDENTIALS {"",""}
    #define ALIAS_URL ""
#endif

#define SEND_HEARTBEAT false

static const char* HOSTNAME = "HandTool";
static const char* WEB_ADDRESS = "handtool.roaming.utwente.nl";
static const char* HEARTBEAT_URL = "https://www.vincentgroenhuis.nl/devices/device_heartbeat.php?id=HandTool";

#ifndef DISABLE_WIFI
extern RoamingWiFiManager manager;
#endif

void wifi_setup();
void wifi_loop();

//const char* wifi_ip();
