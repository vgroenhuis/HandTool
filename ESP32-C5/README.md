# HandTool (ESP32-C5)

This repository contains firmware and resources for the HandTool project targeting the ESP32-C5.

Below is an image of the Hand Tool:

![HandTool](https://raw.githubusercontent.com/vgroenhuis/HandTool/refs/heads/main/pics/HandTool_500px.jpg)

The file `src/WifiCredentials.h` is not under version control. You can make your own. Example contents:

```
{
#pragma once
#define WIFI_CREDENTIALS {{"ssid","password"},{"ssid2","pasword2"}}
#define ADMIN_CREDENTIALS {"admin",""}
#define ALIAS_URL "https://raw.githubusercontent.com/vgroenhuis/roaming-wifi-manager/refs/heads/main/aliases/bssid_info.json"
}
```
