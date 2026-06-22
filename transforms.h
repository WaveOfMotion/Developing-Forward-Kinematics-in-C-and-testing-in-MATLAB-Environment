#ifndef TRANSFORMS_H
#define TRANSFORMS_H

//============================ Define storage for rotation, translation matrix
typedef struct
{
    double m[4][4];
} Matrix_4x4;

void RotX(double jointAngle, Matrix_4x4 *out);
void RotY(double jointAngle, Matrix_4x4 *out);
void RotZ(double jointAngle, Matrix_4x4 *out);
void Translate(double x, double y, double z, Matrix_4x4 *out);

#endif