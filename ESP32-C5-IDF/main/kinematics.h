// Forward kinematics of the HandTool (standard Denavit-Hartenberg, the same parameters as the 3D view).
#pragma once

typedef struct {
    float a;          // link length (m)
    float alpha_deg;  // link twist (degrees)
    float d;          // link offset (m)
    float theta_deg;  // joint angle offset (degrees)
} dh_param_t;

extern const dh_param_t DH_PARAMS[6];

// T (row-major 4x4): base to end-effector, for joint angles in degrees.
void kinematics_fk(const float angles_deg[6], float T[4][4]);
void kinematics_identity(float T[4][4]);
