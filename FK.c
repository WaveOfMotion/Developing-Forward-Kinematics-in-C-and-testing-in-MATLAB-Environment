#include <stdio.h>
#include <math.h>
#include <mex.h>

#define PI 3.14159265358979323846

// ================================ STRUCTS ================================

// ================ Define 4x4 matrix
typedef struct {
    double m[4][4];
} TransMatrix;

// ================================ Rotation matrices ================================

// ================ Function --->>> Rotation around X axes
TransMatrix RotX(double q) {

    TransMatrix T = {0};

    T.m[0][0] = 1.0;

    T.m[1][1] = cos(q);
    T.m[1][2] = -sin(q);

    T.m[2][1] = sin(q);
    T.m[2][2] = cos(q);

    T.m[3][3] = 1.0;

    return T;
}

// ================ Function --->>> Rotation around Y axes
TransMatrix RotY(double q) {

    TransMatrix T = {0};

    T.m[0][0] = cos(q);
    T.m[0][2] = sin(q);

    T.m[1][1] = 1.0;

    T.m[2][0] = -sin(q);
    T.m[2][2] = cos(q);

    T.m[3][3] = 1.0;

    return T;
}

// ================ Function --->>> Rotation around Z axes
TransMatrix RotZ(double q) {

    TransMatrix T = {0};

    T.m[0][0] = cos(q);
    T.m[0][1] = -sin(q);

    T.m[1][0] = sin(q);
    T.m[1][1] = cos(q);

    T.m[2][2] = 1.0;

    T.m[3][3] = 1.0;

    return T;
}

// ================ Function --->>> Translation matrix which shifts a coordinate frame
TransMatrix Translate(
    double x,
    double y,
    double z)
{
    TransMatrix T = {0};

    T.m[0][0] = 1.0;
    T.m[1][1] = 1.0;
    T.m[2][2] = 1.0;
    T.m[3][3] = 1.0;

    T.m[0][3] = x;
    T.m[1][3] = y;
    T.m[2][3] = z;

    return T;
}

// ================================ Create function for matrix multilication ================================
TransMatrix multiply_T_Matrix(TransMatrix A, TransMatrix B) {

    TransMatrix C;

    for(int i = 0; i < 4; i++)
    {
        for(int j = 0; j < 4; j++)
        {
            C.m[i][j] = 0.0;

            for(int k = 0; k < 4; k++)
            {
                C.m[i][j] += A.m[i][k] * B.m[k][j];
            }
        }
    }

    return C;
}
// ================================ Compute forward kinematics ================================

TransMatrix ForwardKinematics(
    double q1,
    double q2,
    double q3,
    double q4,
    double q5,
    double q6) {

    TransMatrix T;

    // Use URDF Kinematic Chain: T = T01*Rz(q1) * T12*Ry(q2) * T23*Ry(q3) * T34*Rx(q4) * T45*Ry(q5) * T56*Rx(q6);
    // All matrices multiplied together give psotion, orinetation of Tool0 (end-effector) respecctive to Base (BodyBase)

    T = multiply_T_Matrix(
            RotZ(q1),
            Translate(0.0,0.0,0.290));

    T = multiply_T_Matrix(
            T,
            multiply_T_Matrix(
                RotY(q2),
                Translate(0.0,0.0,0.270)));

    T = multiply_T_Matrix(
            T,
            multiply_T_Matrix(
                RotY(q3),
                Translate(0.0,0.0,0.070)));

    T = multiply_T_Matrix(
            T,
            multiply_T_Matrix(
                RotX(q4),
                Translate(0.302,0.0,0.0)));

    T = multiply_T_Matrix(
            T,
            multiply_T_Matrix(
                RotY(q5),
                Translate(0.072,0.0,0.0)));

    T = multiply_T_Matrix(
            T,
            RotX(q6));

    return T;
}


// ================================== MEX FUNCTION ================================== //

// mexFunction is the entry-point to MATLAB
void mexFunction(
    int nlhs,              // Number of expected mxArray output arguments, specified as an integer
    mxArray *plhs[],       // Array of pointers to the expected mxArray output arguments
    int nrhs,              // Number of input arguments
    const mxArray *prhs[]) // Array of pointers to the mxArray input arguments
    
{
    if (nrhs != 6)
    {
        mexErrMsgTxt("Six joint angles required.");
    }

    plhs[0] = mxCreateDoubleMatrix(4,4,mxREAL); // allocates a MATLAB matrix

    // Convert MATLAB data into C doubles
    double q1 = mxGetScalar(prhs[0]);
    double q2 = mxGetScalar(prhs[1]);
    double q3 = mxGetScalar(prhs[2]);
    double q4 = mxGetScalar(prhs[3]);
    double q5 = mxGetScalar(prhs[4]);
    double q6 = mxGetScalar(prhs[5]);

    TransMatrix T = ForwardKinematics(q1,q2,q3,q4,q5,q6);

    double *out = mxGetPr(plhs[0]); // returns a pointer to MATLAB's matrix data

    for(int col=0; col<4; col++)
    {
        for(int row=0; row<4; row++)
        {
            out[row + col*4] = T.m[row][col]; // MATLAB stores matrices in column-major order
        }
    }
}