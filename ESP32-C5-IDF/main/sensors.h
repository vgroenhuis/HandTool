// The six joint potentiometers and the drag button (MCP3008 on SPI), the battery voltage, and the joint angles and
// end-effector pose computed from them.
#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool valid;              // the MCP3008 answers
    uint16_t raw[8];         // raw readings, 0..1023
    float filtered[8];       // low-pass filtered
    float angles_deg[6];     // joint angles
    bool button;             // the drag button in the tip
    float T[4][4];           // base to end-effector (m), identity while not valid
    float battery_v;         // 0 when not measured
    float rate_hz;           // samples per second
    uint32_t seq;            // counts samples
} sensor_snapshot_t;

void sensors_start(void);
void sensors_get(sensor_snapshot_t *out);

// Zero calibration: the raw reading at which each joint is at 0 degrees (stored in flash).
extern const int16_t SENSORS_DEFAULT_ZERO[6];
extern const int8_t SENSORS_JOINT_DIRECTIONS[6];
void sensors_get_zero(int16_t zero[6]);
bool sensors_set_zero(const int16_t zero[6]);   // also saves it
bool sensors_zero_here(void);                   // the current pose becomes all zeros
bool sensors_using_default_zero(void);
