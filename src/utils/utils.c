#include "utils.h"
#include <stdio.h>

void print_matrix(const char* matName, arm_matrix_instance_f32 *mat) {
    printf("Matrix: %s (%dx%d)\n", matName, mat->numRows, mat->numCols);
    for (int i = 0; i < mat->numRows; i++) {
        for (int j = 0; j < mat->numCols; j++) {
            printf("%10.4f ", mat->pData[i * mat->numCols + j]);
        }
        printf("\n");
    }
    printf("\n");
}
