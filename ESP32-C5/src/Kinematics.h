#pragma once
#include <Arduino.h>

// DH parameters structure
struct DHParams {
    float a;          // link length (m)
    float alpha_deg;  // link twist (degrees)
    float d;          // link offset (m)
    float theta_deg;  // joint angle offset (degrees)
};

// 4x4 transformation matrix (row-major storage)
struct Matrix4x4 {
    float m[4][4];  // 2D array: m[row][col]
};

const Matrix4x4 IDENTITY_MATRIX = {
    {
        {1.0, 0.0, 0.0, 0.0},
        {0.0, 1.0, 0.0, 0.0},
        {0.0, 0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0, 1.0}
    }
};

// DH parameters matching robotView.html
const DHParams dh_params[6] = {
    { 0.0,     -90.0, 0.0625,   0.0   },  // Joint 1
    { 0.09375,   0.0, 0.0,     -90.0   },  // Joint 2
    { 0.09375,   0.0, 0.0,      90.0   },  // Joint 3
    { 0.0,     -90.0, -0.01533, -90.0  },  // Joint 4
    { 0.0,      90.0, 0.01533,  90.0   },  // Joint 5
    { 0.0,       0.0, -0.072, 90.0   }   // Joint 6
};

//void matrix_identity(Matrix4x4& mat);

// Initialize kinematics with DH parameters
void kinematics_init();

// Compute forward kinematics: returns 4x4 transformation matrix from base to end-effector
// joint_angles: array of 6 joint angles in degrees
Matrix4x4 compute_forward_kinematics(const float joint_angles[6]);
