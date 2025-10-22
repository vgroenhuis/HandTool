#include "Files.h"
#include <LittleFS.h>
#include <Arduino.h>

void init_file_system() {
    if (!LittleFS.begin(true)) {
        Serial.println("LittleFS Mount Failed");
    } else {
        Serial.println("LittleFS Mounted Successfully");
    }
}
