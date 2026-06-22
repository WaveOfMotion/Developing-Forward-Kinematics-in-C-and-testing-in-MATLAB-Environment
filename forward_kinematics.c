#include <stdio.h>
#include <math.h>

#include "transforms.h"
#include "matrix_operations.h"
#include "forward_kinematics.h"

void forward_kinematics(
    double jointA1,
    double jointA2,
    double jointA3,
    double jointA4,
    double jointA5,
    double jointA6,
    Matrix_4x4 *result)
{
    Matrix_4x4 Rotation_M;
    Matrix_4x4 Translation_M;
    Matrix_4x4 local;
    Matrix_4x4 temp;

    //============================ T01 
    RotZ(jointA1, &Rotation_M);
    Translate(0.0,0.0,0.290, &Translation_M);

    multiply_matrix(&Rotation_M, &Translation_M, result);

    //============================ T02
    RotY(jointA2, &Rotation_M);
    Translate(0.0,0.0,0.270, &Translation_M);

    multiply_matrix(&Rotation_M, &Translation_M, &local);
    multiply_matrix(result, &local, &temp);
    *result = temp;

    //============================ T03
    RotY(jointA3, &Rotation_M);
    Translate(0.0,0.0,0.070, &Translation_M);

    multiply_matrix(&Rotation_M, &Translation_M, &local);
    multiply_matrix(result, &local, &temp);
    *result = temp;

    //============================ T04
    RotX(jointA4, &Rotation_M);
    Translate(0.302,0.0,0.0, &Translation_M);

    multiply_matrix(&Rotation_M, &Translation_M, &local);
    multiply_matrix(result, &local, &temp);
    *result = temp;

    //============================ T05
    RotY(jointA5, &Rotation_M);
    Translate(0.072,0.0,0.0, &Translation_M);

    multiply_matrix(&Rotation_M, &Translation_M, &local);
    multiply_matrix(result, &local, &temp);
    *result = temp;

    //============================ T06
    RotX(jointA6, &Rotation_M);
    multiply_matrix(result, &Rotation_M, &temp);
    *result = temp;
}