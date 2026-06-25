#include <stdio.h>
#include "robot_constants.h"
#include "forward_kinematics.h"

int main(void)
{
    double jointA1 = PI/4;
    double jointA2 = PI/2;
    double jointA3 = 0;
    double jointA4 = 0;
    double jointA5 = 0;
    double jointA6 = 0;

    Matrix_4x4 T;

    forward_kinematics(jointA1, jointA2, jointA3, jointA4, jointA5, jointA6, &T);

    printf("Transformation Matrix:\n\n");

    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            printf("%12.6f ", T.m[i][j]);
        }
        printf("\n");
    }

    return 0;
}