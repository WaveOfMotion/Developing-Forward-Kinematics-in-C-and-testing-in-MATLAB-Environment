#include <stdio.h>
#include "robot_constants.h"
#include "FK.h"

Matrix_4x4 TransMatrix;
Pose body_pose;

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

    // ============================ Run Rotm2Eul
    body_pose = robot_pose(&TransMatrix);

    // ============================ Print data 
    printf("Transformation Matrix:\n\n");
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            printf("%12.6f ", TransMatrix.m[i][j]);
        }
        printf("\n\n");
    }

    printf("\nPose:\n");
    printf("X     = %12.6f\n", body_pose.x);
    printf("Y     = %12.6f\n", body_pose.y);
    printf("Z     = %12.6f\n", body_pose.z);

    printf("Roll  = %12.6f rad\n", body_pose.roll);
    printf("Pitch = %12.6f rad\n", body_pose.pitch);
    printf("Yaw   = %12.6f rad\n", body_pose.yaw);

    return 0;
}