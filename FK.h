#ifndef FK
#define FK

// ============================ Create 4x4 matrix
typedef struct
{
    double m[4][4];
} Matrix_4x4;

typedef struct
{
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
} Pose;

// ============================ Define robot links
void Link1(double q, Matrix_4x4 *T);
void Link2(double q, Matrix_4x4 *T);
void Link3(double q, Matrix_4x4 *T);
void Link4(double q, Matrix_4x4 *T);
void Link5(double q, Matrix_4x4 *T);
void Link6(double q, Matrix_4x4 *T);

// ============================ Define void function performing matrix multiplication
Matrix_4x4 multiply_matrix(const Matrix_4x4 *A, const Matrix_4x4 *B);

// ============================ Define void function computing transformation matrix
void forward_kinematics(double jointAngle[6], Matrix_4x4 *T);

Pose robot_pose(Matrix_4x4 *T);

#endif