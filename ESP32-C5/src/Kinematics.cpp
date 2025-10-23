#include "Kinematics.h"
#include <math.h>

// DH parameters matching robotView.html
static DHParams dh_params[6] = {
    { 0.0,     -90.0, 0.0625,   0.0   },  // Joint 1
    { 0.09375,   0.0, 0.0,     -90.0   },  // Joint 2
    { 0.09375,   0.0, 0.0,      90.0   },  // Joint 3
    { 0.0,     -90.0, -0.01533, -90.0  },  // Joint 4
    { 0.0,      90.0, 0.01533,  90.0   },  // Joint 5
    { 0.0,       0.0, -0.04073, 90.0   }   // Joint 6
};

void kinematics_init() {
    // DH parameters are statically initialized, nothing to do here
}

// Helper: multiply two 4x4 matrices (C = A * B)
static void matrix_multiply(const float A[16], const float B[16], float C[16]) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            C[i*4 + j] = 0.0;
            for (int k = 0; k < 4; k++) {
                C[i*4 + j] += A[i*4 + k] * B[k*4 + j];
            }
        }
    }
}

// Helper: create 4x4 identity matrix
static void matrix_identity(float M[16]) {
    for (int i = 0; i < 16; i++) M[i] = 0.0;
    M[0] = M[5] = M[10] = M[15] = 1.0;
}

// Helper: compute DH transformation matrix
// Standard DH: T = Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
static void dh_transform(float a, float alpha_rad, float d, float theta_rad, float T[16]) {
    float ca = cos(alpha_rad);
    float sa = sin(alpha_rad);
    float ct = cos(theta_rad);
    float st = sin(theta_rad);
    
    // Row-major 4x4 matrix
    T[0]  = ct;
    T[1]  = -st * ca;
    T[2]  = st * sa;
    T[3]  = a * ct;
    
    T[4]  = st;
    T[5]  = ct * ca;
    T[6]  = -ct * sa;
    T[7]  = a * st;
    
    T[8]  = 0.0;
    T[9]  = sa;
    T[10] = ca;
    T[11] = d;
    
    T[12] = 0.0;
    T[13] = 0.0;
    T[14] = 0.0;
    T[15] = 1.0;
}

Matrix4x4 compute_forward_kinematics(const float joint_angles[6]) {
    Matrix4x4 result;
    float T[16], A[16], temp[16];
    
    // Initialize with identity matrix
    matrix_identity(T);
    
    // Multiply DH transformations for each joint
    for (int i = 0; i < 6; i++) {
        // Convert angles to radians
        float alpha_rad = dh_params[i].alpha_deg * M_PI / 180.0;
        float theta_rad = (dh_params[i].theta_deg + joint_angles[i]) * M_PI / 180.0;
        
        // Compute DH transformation for this joint
        dh_transform(dh_params[i].a, alpha_rad, dh_params[i].d, theta_rad, A);
        
        // Multiply: T = T * A
        matrix_multiply(T, A, temp);
        
        // Copy result back to T
        for (int j = 0; j < 16; j++) {
            T[j] = temp[j];
        }
    }
    
    // Copy final transformation to result
    for (int i = 0; i < 16; i++) {
        result.m[i] = T[i];
    }
    
    return result;
}
