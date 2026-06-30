#include <stdio.h>
#include "robot_constants.h"
#include "FK.h"

Matrix_4x4 TransMatrix;

int main(void)
{
    double jointAngle[6] =
    {
        PI/8,
        PI/7,
        PI/7,
        PI/6,
        PI/12,
        -PI/12
    };

    // ============================ Run forward kinematics
    forward_kinematics(jointAngle, &TransMatrix);

    // ============================ Print data 
    printf("Transformation Matrix:\n\n");
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            printf("%12.6f ", TransMatrix.m[i][j]);
        }
        printf("\n");
    }

    return 0;
}