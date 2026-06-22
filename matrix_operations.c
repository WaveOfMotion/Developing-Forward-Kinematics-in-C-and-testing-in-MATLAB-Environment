#include <stdio.h>
#include <math.h>

#include "matrix_operations.h"

void multiply_matrix(const Matrix_4x4 *A, const Matrix_4x4 *B, Matrix_4x4 *C) 
{

    for(int i = 0; i < 4; i++)
    {
        for(int j = 0; j < 4; j++)
        {
            C->m[i][j] = 0.0;

            for(int k = 0; k < 4; k++)
            {
                C->m[i][j] += A->m[i][k] * B->m[k][j];
            }
        }
    }

    return;
}