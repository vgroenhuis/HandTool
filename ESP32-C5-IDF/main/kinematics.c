#include "kinematics.h"

#include <math.h>
#include <string.h>

// The same values as the Arduino firmware and robotView.html.
const dh_param_t DH_PARAMS[6] = {
    {0.0f, -90.0f, 0.0625f, 0.0f},        // joint 1
    {0.09375f, 0.0f, 0.0f, -90.0f},       // joint 2
    {0.09375f, 0.0f, 0.0f, 90.0f},        // joint 3
    {0.0f, -90.0f, -0.01533f, -90.0f},    // joint 4
    {0.0f, 90.0f, 0.01533f, 90.0f},       // joint 5
    {0.0f, 0.0f, -0.072f, 90.0f},         // joint 6
};

void kinematics_identity(float T[4][4]) {
    memset(T, 0, 16 * sizeof(float));
    for (int i = 0; i < 4; i++) T[i][i] = 1.0f;
}

// T = Rz(theta) Tz(d) Tx(a) Rx(alpha)
static void dh_transform(float a, float alpha, float d, float theta, float A[4][4]) {
    float ct = cosf(theta), st = sinf(theta), ca = cosf(alpha), sa = sinf(alpha);
    const float m[4][4] = {
        {ct, -st * ca, st * sa, a * ct},
        {st, ct * ca, -ct * sa, a * st},
        {0, sa, ca, d},
        {0, 0, 0, 1},
    };
    memcpy(A, m, sizeof m);
}

void kinematics_fk(const float angles_deg[6], float T[4][4]) {
    const float rad = (float)M_PI / 180.0f;
    kinematics_identity(T);
    for (int i = 0; i < 6; i++) {
        float A[4][4], R[4][4];
        dh_transform(DH_PARAMS[i].a, DH_PARAMS[i].alpha_deg * rad, DH_PARAMS[i].d,
                     (DH_PARAMS[i].theta_deg + angles_deg[i]) * rad, A);
        for (int r = 0; r < 4; r++) {
            for (int c = 0; c < 4; c++) {
                R[r][c] = T[r][0] * A[0][c] + T[r][1] * A[1][c] + T[r][2] * A[2][c] + T[r][3] * A[3][c];
            }
        }
        memcpy(T, R, sizeof R);
    }
}
