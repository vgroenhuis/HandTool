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

// Helper: multiply two 4x4 matrices (row-major)
static void matrix_multiply(const Matrix4x4& A, const Matrix4x4& B, Matrix4x4& result) {
    float temp[16];
    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
            temp[row * 4 + col] = 0.0;
            for (int k = 0; k < 4; k++) {
                temp[row * 4 + col] += A.m[row * 4 + k] * B.m[k * 4 + col];
            }
        }
    }
    memcpy(result.m, temp, sizeof(temp));
}

// Helper: create identity matrix
static void matrix_identity(Matrix4x4& mat) {
    for (int i = 0; i < 16; i++) {
        mat.m[i] = 0.0;
    }
    mat.m[0] = mat.m[5] = mat.m[10] = mat.m[15] = 1.0;
}

// Helper: compute DH transformation matrix
// Standard DH: T = Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
static Matrix4x4 dh_transform(float a, float alpha_rad, float d, float theta_rad) {
    Matrix4x4 T;
    
    float ct = cos(theta_rad);
    float st = sin(theta_rad);
    float ca = cos(alpha_rad);
    float sa = sin(alpha_rad);
    
    // Combined DH matrix (row-major)
    T.m[0] = ct;           T.m[1] = -st * ca;      T.m[2] = st * sa;       T.m[3] = a * ct;
    T.m[4] = st;           T.m[5] = ct * ca;       T.m[6] = -ct * sa;      T.m[7] = a * st;
    T.m[8] = 0;            T.m[9] = sa;            T.m[10] = ca;           T.m[11] = d;
    T.m[12] = 0;           T.m[13] = 0;            T.m[14] = 0;            T.m[15] = 1;
    
    return T;
}

Matrix4x4 compute_forward_kinematics(const float joint_angles[6]) {
    Matrix4x4 result, temp;
    
    // Start with identity matrix
    matrix_identity(result);
    
    // Multiply DH transformations for each joint
    for (int i = 0; i < 6; i++) {
        // Convert angles to radians
        float alpha_rad = dh_params[i].alpha_deg * M_PI / 180.0;
        float theta_rad = (dh_params[i].theta_deg + joint_angles[i]) * M_PI / 180.0;
        
        // Compute DH transformation for this joint
        Matrix4x4 A = dh_transform(dh_params[i].a, alpha_rad, dh_params[i].d, theta_rad);
        
        // Multiply: result = result * A
        matrix_multiply(result, A, temp);
        result = temp;
    }
    
    return result;
}
