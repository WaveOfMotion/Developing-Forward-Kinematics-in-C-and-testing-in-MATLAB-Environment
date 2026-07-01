#include <stdio.h>
#include <math.h>
#include "robot_constants.h"
#include "FK.h"

// T01 = RotZ * Translation: [x,y,z] = [0.0, 0.0, 0.290]

void Link1(double q, Matrix_4x4 *T)
{
    double c = cos(q);
    double s = sin(q);

    T-> m[0][0] = c;
    T-> m[0][1] = -s;
    T-> m[0][2] = 0.0;
    T-> m[0][3] = 0.0;

    T-> m[1][0] = s;
    T-> m[1][1] = c;
    T-> m[1][2] = 0.0;
    T-> m[1][3] = 0.0;

    T-> m[2][0] = 0.0;
    T-> m[2][1] = 0.0;
    T-> m[2][2] = 1.0;
    T-> m[2][3] = 0.290;

    T-> m[3][0] = 0.0;
    T-> m[3][1] = 0.0;
    T-> m[3][2] = 0.0;
    T-> m[3][3] = 1.0;
}

// T02 = RotY * Translation: [x,y,z] = [0.0, 0.0, 0.270]

void Link2(double q, Matrix_4x4 *T)
{
    double c = cos(q);
    double s = sin(q);
    
    T-> m[0][0] = c;
    T-> m[0][1] = 0.0;
    T-> m[0][2] = s;
    T-> m[0][3] = 0.27*s;
	
    T-> m[1][0] = 0.0;
    T-> m[1][1] = 1.0;
    T-> m[1][2] = 0.0;
    T-> m[1][3] = 0.0;
	
    T-> m[2][0] = -s;
    T-> m[2][1] = 0.0;
    T-> m[2][2] = c;
    T-> m[2][3] = 0.27*c;
	
    T-> m[3][0] = 0.0;
    T-> m[3][1] = 0.0;
    T-> m[3][2] = 0.0;
    T-> m[3][3] = 1.0;
}

// T03 = RotY * Translation: [x,y,z] = [0.0, 0.0, 0.07]

void Link3(double q, Matrix_4x4 *T)
{
    double c = cos(q);
    double s = sin(q);

    T-> m[0][0] = c;
    T-> m[0][1] = 0.0;
    T-> m[0][2] = s;
    T-> m[0][3] = 0.07*s;
	
    T-> m[1][0] = 0.0;
    T-> m[1][1] = 1.0;
    T-> m[1][2] = 0.0;
    T-> m[1][3] = 0.0;
	
    T-> m[2][0] = -s;
    T-> m[2][1] = 0.0;
    T-> m[2][2] = c;
    T-> m[2][3] = 0.07*c;
	
    T-> m[3][0] = 0.0;
    T-> m[3][1] = 0.0;
    T-> m[3][2] = 0.0;
    T-> m[3][3] = 1.0;
}

// T04 = RotX * Translation: [x,y,z] = [0.302, 0.0, 0.0]

void Link4(double q, Matrix_4x4 *T)
{
    double c = cos(q);
    double s = sin(q);

    T-> m[0][0] = 1.0;
    T-> m[0][1] = 0.0;
    T-> m[0][2] = 0.0;
    T-> m[0][3] = 0.302;
	
    T-> m[1][0] = 0.0;
    T-> m[1][1] = c;
    T-> m[1][2] = -s;
    T-> m[1][3] = 0.0;
	
    T-> m[2][0] = 0.0;
    T-> m[2][1] = s;
    T-> m[2][2] = c;
    T-> m[2][3] = 0.0;
	
    T-> m[3][0] = 0.0;
    T-> m[3][1] = 0.0;
    T-> m[3][2] = 0.0;
    T-> m[3][3] = 1.0;
}

// T05 = RotY * Translation: [x,y,z] = [0.072, 0.0, 0.0]

void Link5(double q, Matrix_4x4 *T)
{
    double c = cos(q);
    double s = sin(q);

    T-> m[0][0] = c;
    T-> m[0][1] = 0.0;
    T-> m[0][2] = s;
    T-> m[0][3] = 0.072*c;
	
    T-> m[1][0] = 0.0;
    T-> m[1][1] = 1.0;
    T-> m[1][2] = 0.0;
    T-> m[1][3] = 0.0;
	
    T-> m[2][0] = -s;
    T-> m[2][1] = 0.0;
    T-> m[2][2] = c;
    T-> m[2][3] = -0.072*s;
	
    T-> m[3][0] = 0.0;
    T-> m[3][1] = 0.0;
    T-> m[3][2] = 0.0;
    T-> m[3][3] = 1.0;
}

// T05 = RotX

void Link6(double q, Matrix_4x4 *T)
{
    double c = cos(q);
    double s = sin(q);

    T-> m[0][0] = 1.0;
    T-> m[0][1] = 0.0;
    T-> m[0][2] = 0.0;
    T-> m[0][3] = 0.0;
	
    T-> m[1][0] = 0.0;
    T-> m[1][1] = c;
    T-> m[1][2] = -s;
    T-> m[1][3] = 0.0;
	
    T-> m[2][0] = 0.0;
    T-> m[2][1] = s;
    T-> m[2][2] = c;
    T-> m[2][3] = 0.0;
	
    T-> m[3][0] = 0.0;
    T-> m[3][1] = 0.0;
    T-> m[3][2] = 0.0;
    T-> m[3][3] = 1.0;
}

// ============================ Define function multiplying matrices

Matrix_4x4 multiply_matrix(const Matrix_4x4 *A, const Matrix_4x4 *B) 
{
    Matrix_4x4 C;

    for(int i = 0; i < 4; i++)
    {
        for(int j = 0; j < 4; j++)
        {
            C.m[i][j] = 0.0;

            for(int k = 0; k < 4; k++)
            {
                C.m[i][j] += A->m[i][k] * B->m[k][j];
            }
        }
    }

    return C;
}

// ============================ Define void function multiplying matrices

void forward_kinematics(
    double jointAngle[6],
    Matrix_4x4 *T)
{
    Matrix_4x4 Link;

    Link1(jointAngle[0], T);

    Link2(jointAngle[1], &Link);
    *T = multiply_matrix(T, &Link);

    Link3(jointAngle[2], &Link);
    *T = multiply_matrix(T, &Link);

    Link4(jointAngle[3], &Link);
    *T = multiply_matrix(T, &Link);

    Link5(jointAngle[4], &Link);
    *T = multiply_matrix(T, &Link);

    Link6(jointAngle[5], &Link);
    *T = multiply_matrix(T, &Link);
}

// ============================ Compute Euler Angles (ZYX convection)
Pose robot_pose(Matrix_4x4 *T)
{
    Pose pose;

    pose.x = T->m[0][3];
    pose.y = T->m[1][3];
    pose.z = T->m[2][3];

    // Calculate Pitch -> clamp sin value to avoid precision errors outside [-1, 1]
    double sin_pitch = -T->m[2][0];

    if(sin_pitch > 1.0)
    {
        sin_pitch = 1;
    } else { if(sin_pitch < -1)
        sin_pitch = -1;
    }
    
    pose.pitch = asin(sin_pitch);

    // Check for Gimbal-Lock -> cos(pitch) close to zero
    double cos_pitch  = cos(pose.pitch);

    if(fabs(cos_pitch) < 1e-8)
    {
        pose.roll = 0.0;
        pose.yaw = atan2(-T->m[0][1], T->m[1][1]);
    } else {
        
        pose.roll = atan2(T->m[2][1], T->m[2][2]);
        pose.yaw = atan2(T->m[1][0], T->m[0][0]);
    }
    
    return pose;
}