#include <math.h>
#include "transforms.h"

void RotX(double jointAngle, Matrix_4x4 *out)
{
    out-> m[0][0] = 1.0;
    out-> m[0][1] = 0.0;
    out-> m[0][2] = 0.0;
    out-> m[0][3] = 0.0;

    out-> m[1][0] = 0.0;
    out-> m[1][1] = cos(jointAngle);
    out-> m[1][2] = -sin(jointAngle);
    out-> m[1][3] = 0.0;

    out-> m[2][0] = 0.0;
    out-> m[2][1] = sin(jointAngle);
    out-> m[2][2] = cos(jointAngle);
    out-> m[2][3] = 0.0;

    out-> m[3][0] = 0.0;
    out-> m[3][1] = 0.0;
    out-> m[3][2] = 0.0;
    out-> m[3][3] = 1.0;
}

void RotY(double jointAngle, Matrix_4x4 *out)
{
    out-> m[0][0] = cos(jointAngle);
    out-> m[0][1] = 0.0;
    out-> m[0][2] = sin(jointAngle);
    out-> m[0][3] = 0.0;

    out-> m[1][0] = 0.0;
    out-> m[1][1] = 1.0;
    out-> m[1][2] = 0.0;
    out-> m[1][3] = 0.0;

    out-> m[2][0] = -sin(jointAngle);
    out-> m[2][1] = 0.0;
    out-> m[2][2] = cos(jointAngle);
    out-> m[2][3] = 0.0;

    out-> m[3][0] = 0.0;
    out-> m[3][1] = 0.0;
    out-> m[3][2] = 0.0;
    out-> m[3][3] = 1.0;
}

void RotZ(double jointAngle, Matrix_4x4 *out)
{
    out-> m[0][0] = cos(jointAngle);
    out-> m[0][1] = -sin(jointAngle);
    out-> m[0][2] = 0.0;
    out-> m[0][3] = 0.0;

    out-> m[1][0] = sin(jointAngle);
    out-> m[1][1] = cos(jointAngle);
    out-> m[1][2] = 0.0;
    out-> m[1][3] = 0.0;

    out-> m[2][0] = 0.0;
    out-> m[2][1] = 0.0;
    out-> m[2][2] = 1.0;
    out-> m[2][3] = 0.0;

    out-> m[3][0] = 0.0;
    out-> m[3][1] = 0.0;
    out-> m[3][2] = 0.0;
    out-> m[3][3] = 1.0;
}

void Translate(double x, double y, double z, Matrix_4x4 *out)
{
    out->m[0][0] = 1.0;
    out->m[0][1] = 0.0;
    out->m[0][2] = 0.0;
    out->m[0][3] = x;

    out->m[1][0] = 0.0;
    out->m[1][1] = 1.0;
    out->m[1][2] = 0.0;
    out->m[1][3] = y;

    out->m[2][0] = 0.0;
    out->m[2][1] = 0.0;
    out->m[2][2] = 1.0;
    out->m[2][3] = z;

    out->m[3][0] = 0.0;
    out->m[3][1] = 0.0;
    out->m[3][2] = 0.0;
    out->m[3][3] = 1.0;
}