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

// Initialize kinematics with DH parameters
void kinematics_init();

// Compute forward kinematics: returns 4x4 transformation matrix from base to end-effector
// joint_angles: array of 6 joint angles in degrees
Matrix4x4 compute_forward_kinematics(const float joint_angles[6]);
