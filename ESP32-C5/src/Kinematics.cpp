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

// Helper: create identity matrix
/*
void matrix_identity(Matrix4x4& mat) {
    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
            mat.m[row][col] = (row == col) ? 1.0 : 0.0;
        }
    }
}*/

// Helper: multiply two 4x4 matrices (row-major)
static void matrix_multiply(const Matrix4x4& A, const Matrix4x4& B, Matrix4x4& result) {
    float temp[4][4];
    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
            temp[row][col] = 0.0;
            for (int k = 0; k < 4; k++) {
                temp[row][col] += A.m[row][k] * B.m[k][col];
            }
        }
    }
    memcpy(result.m, temp, sizeof(temp));
}


// Helper: compute DH transformation matrix
// Standard DH: T = Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
Matrix4x4 dh_transform(float a, float alpha_rad, float d, float theta_rad) {
    Matrix4x4 T;
    
    float ct = cos(theta_rad);
    float st = sin(theta_rad);
    float ca = cos(alpha_rad);
    float sa = sin(alpha_rad);
    
    // Combined DH matrix
    T.m[0][0] = ct;        T.m[0][1] = -st * ca;   T.m[0][2] = st * sa;    T.m[0][3] = a * ct;
    T.m[1][0] = st;        T.m[1][1] = ct * ca;    T.m[1][2] = -ct * sa;   T.m[1][3] = a * st;
    T.m[2][0] = 0;         T.m[2][1] = sa;         T.m[2][2] = ca;         T.m[2][3] = d;
    T.m[3][0] = 0;         T.m[3][1] = 0;          T.m[3][2] = 0;          T.m[3][3] = 1;
    
    return T;
}

Matrix4x4 compute_forward_kinematics(const float joint_angles[6]) {
    Matrix4x4 result, temp;
    
    // Start with identity matrix
    //matrix_identity(result);
    result = IDENTITY_MATRIX;
    
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
