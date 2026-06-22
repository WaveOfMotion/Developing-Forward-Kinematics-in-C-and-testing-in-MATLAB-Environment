#include <stdio.h>
#include "robot_constants.h"
#include "forward_kinematics.h"
#include "mex.h"

Matrix_4x4 T;
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
    double jointA1 = mxGetScalar(prhs[0]);
    double jointA2 = mxGetScalar(prhs[1]);
    double jointA3 = mxGetScalar(prhs[2]);
    double jointA4 = mxGetScalar(prhs[3]);
    double jointA5 = mxGetScalar(prhs[4]);
    double jointA6 = mxGetScalar(prhs[5]);

    forward_kinematics(jointA1,jointA2,jointA3,jointA4,jointA5,jointA6, &T);

    double *out = mxGetPr(plhs[0]); // returns a pointer to MATLAB's matrix data

    for(int col=0; col<4; col++)
    {
        for(int row=0; row<4; row++)
        {
            out[row + col*4] = T.m[row][col]; // MATLAB stores matrices in column-major order
        }
    }
}