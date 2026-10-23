# HandTool firmware (ESP-IDF)

ESP-IDF version of the Arduino/PlatformIO firmware in `../ESP32-C5`, for the ESP32-C5-DevKitC-1. Same hardware, same maths, same
pages and HTTP endpoints. What changed:

* **Wi-Fi without RoamingWiFiManager.** The network, password and the lab demos key are stored in flash (NVS) by a one-time
  settings image (`tools/provision.py`), not in the source. The shared `components/closer_demo` joins the network (5 GHz only,
  as before), keeps the time, reconnects, and moves to a clearly stronger access point of the same network.
* **Lab demos page.** The HandTool reports to https://closer.ram.eemcs.utwente.nl/labdemos/ about once a minute: address,
  signal, access point, firmware, battery, sensor state, end-effector position, viewers, problems. The page shows its status,
  history and the set-up instructions, and links to its own page while it is active.
* **Over-the-air updates.** A new release on the server is installed at the next report; one that cannot report within two
  minutes is rolled back. The web pages are embedded (gzipped) in the firmware image, so one image is everything (no LittleFS).
* **Live stream.** `ws://<address>/ws` pushes `/allData` 50 times a second (send any message to start). The 3D view and the
  main page use it, falling back to polling. The lab demos server relays it (`/labdemos/api/live/handtool`), so pages on that
  site (a 3D object game and a buzz-wire game) can use the HandTool from anywhere.
* **Zero calibration in flash** (from the roadmap): `POST /api/zero` makes the current pose the zero pose (`?reset=1`: the
  factory values of the Arduino version); `GET/POST /api/calibration` `{"zero": [6 raw readings]}`. A button on the main page.
* **No halt on missing parts.** A missing MCP3008 or display is reported (display, page, lab demos page) and the MCP3008 is
  looked for again every 2 seconds. three.js is served from the device (no internet needed for the 3D view).

Endpoints kept from the Arduino version (same JSON): `/`, `/index.html`, `/robotView.html`, `/allData` (now also `battery`,
`hz`, `seq`), `/angles`, `/rawAdcValues`, `/filteredAdcValues`, `/fk`, `/dhParams`, `/serialAngles?enable=`, `/serialFK?enable=`,
`/serialFrequency?hz=`. New: `/ws`, `/api/state`, `/api/zero`, `/api/calibration`. The Wi-Fi manager page `/wifi` is gone.

Pins (menuconfig > HandTool): MCP3008 SCK=0, MOSI=8, MISO=7, CS=9 (1 MHz); SH1106 OLED SDA=2, SCL=3; battery divider on GPIO6
(ADC1 channel 5, x3); drag button on MCP3008 channel 6.

## Build

    . $IDF_PATH/export.sh            # ESP-IDF 6.1 (5.5 builds too)
    idf.py build

## First flash (USB)

On the lab demos server, `python -m labdemos.cli add handtool` gives the device key (`local/device.key`). Put the Wi-Fi settings
in `local/provision.json` (`{"ssid": "iotroam", "password": "..."}`), then

    python tools/provision.py        # local/handtool-factory.bin: bootloader, partitions, firmware, settings
    python -m esptool --chip esp32c5 -p <port> write-flash 0x0 handtool-factory.bin

`local/` holds the Wi-Fi password and the key and is not in git. To change only the Wi-Fi settings later, flash
`local/handtool-settings.bin` at 0x9000. Over-the-air updates never touch the settings.

## Updates over the air

Raise `VERSION`, `idf.py build`, and on the lab demos server:

    python -m labdemos.cli publish handtool <path>/build/handtool.bin

## Files

    main/main.c          start-up order
    main/sensors.c       MCP3008 sampling (100 Hz, low-pass filter), angles, zero calibration (NVS), battery
    main/kinematics.c    forward kinematics (Denavit-Hartenberg, the same parameters as the 3D view)
    main/display.c       SH1106 driver and the status screen
    main/web.c           pages, JSON endpoints, WebSocket stream, calibration API, reports
    web/                 index.html, robotView.html, photo, three.js (embedded into the firmware at build time)
    components/closer_demo   Wi-Fi, reports, over-the-air updates (shared by the CloSer lab demos; copy of the one in
                             github.com/vgroenhuis/LedDemo)
    tools/provision.py   the first-flash image;  tools/pack_web.py  embeds web/
