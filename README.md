# Developing-Forward-Kinematics-in-C-and-testing-in-MATLAB-Environment
The application in C language computes forward kinematics using MATLAB URDF chain transform. Since MATLAB doesn't use DH parameters for specifying robot rigidBodyTree, a method to extract fixed-transforms, joint-axis rotations is proposed to manually compute robot transformation matrix in C. For C code to be executed, a mex function is called. The following URDF Transform chain equation is used
```iecst
T = T01∙Rz∙(q1)∙T12∙Ry∙(q2)∙T23∙Ry∙(q3)∙T34∙Rx∙(q4)∙T45∙Ry∙(q5)∙T56∙Rx(q6);
```
Before using MEX, if you want to use Microsoft Visuual C++ as your C/C++ compiler in MATLAB, install VS Installer with Microsoft Visual C++ 2022. Link:
```iecst
https://aka.ms/vs/17/release/vs_community.exe
```
, and configure your MATLAB Compiler with the following commands in the command window:
```iecst
mex -setup C
mex -setup C++
```
For this application I'm using ABB IRB 120 robot urdf model.

For your following robot urdf parameters extraction, to build transform matrix, first you need to import urdf:
```iecst
robot = importrobot('abbIrb120.urdf');
robot.DataFormat = 'row';
showdetails(robot);
```
Next is to get fixed-transforms between links, in which you can get joint rotation along axis 'JointAxis' and the position limits 'PositionLimits':
```iecst
for i = 1:numel(robot.Bodies)
    body = robot.Bodies{i};

    fprintf('\nBody: %s\n', body.Name);

    tform = body.Joint.JointToParentTransform;

    disp(body.Joint);
end
```
Next is to get urdf geometry transform matrices, to get rotation and translation part of each link.
Because the urdf defines the robot as: 
Parent link ---> Fixed Transform ---> Joint Axis ---> Child Link   , a joint in urdf consists of fixed transform (origin) and rotation axis (axis).
In short words, parent link is the first, child is the next, where again child link is a parent to the its next. To get the transforms, simply write:
```iecst
for i = 1:numel(robot.Bodies)
    body = robot.Bodies{i};

    fprintf('\n%s\n', body.Name);

    disp(body.Joint.JointToParentTransform);
end
```
And again to print just the joint rotation matrices, use:
```iecst
for i = 1:numel(robot.Bodies)

    body = robot.Bodies{i};

    fprintf('\n%s\n', body.Name);

    disp(body.Joint.JointAxis);

end
```
For the MEX function integration in your C code, you must include library:
```iecst
#include <mex.h>
```
, where the mex function arguments shoud be in precise order as below:
```iecst
void mexFunction(
    int nlhs,              // Number of expected mxArray output arguments, specified as an integer
    mxArray *plhs[],       // Array of pointers to the expected mxArray output arguments
    int nrhs,              // Number of input arguments
    const mxArray *prhs[]) // Array of pointers to the mxArray input arguments
```
To create a mex function from C code in MATLAB, make sure .c file is in current directory, and in command window simply write:
```iecst
mex nameofapplication.c
```
In my situation:
```iecst
mex Call_MEX_func.c transforms.c matrix_operations.c forward_kinematics.c
```
# Optional testing in Visual Studio
You can also test this also in Visual Studio Code by calling the main.c. Simply in terminal write:
```iecst
gcc transforms.c matrix_operations.c forward_kinematics.c main.c -lm -o main
```
, which will create an .exe file in your folder. Then simply in terminal write:
```iecst
.\main.exe
```
# Conclusion
After specifying joint angles, computing the transform matrix for forward kinematics and comparing with MATLAB result:
```iecst
q = [pi/2, -pi/2, 0, 0, 0, 0];
Transform_urdf = getTransform(robot,q,'tool0');
```
, machine precision was returned:
```iecst
Error = 1.734723475976807e-18
```
, and it proved that forward kinematics in C programing can be written with high precision.
