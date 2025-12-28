#include <stdio.h>
#include "arm_math.h"

void print_matrix(const char* name, arm_matrix_instance_f32 *S) {
    printf("Matrix %s (%dx%d):\n", name, S->numRows, S->numCols);
    for (uint16_t i = 0; i < S->numRows; i++) {
        for (uint16_t j = 0; j < S->numCols; j++) {
            printf("%8.4f ", S->pData[i * S->numCols + j]);
        }
        printf("\n");
    }
    printf("\n");
}

int main() {
    float32_t a_data[6] = {
        1.0f, 2.0f,
        3.0f, 4.0f,
        5.0f, 6.0f
    };
    
    float32_t b_data[6] = {
        7.0f, 8.0f, 9.0f,
        10.0f, 11.0f, 12.0f
    };

    float32_t c_data[9];
    
    float32_t inv_data[9];

    arm_matrix_instance_f32 A, B, C, Inv;
    
    arm_mat_init_f32(&A, 3, 2, a_data);
    arm_mat_init_f32(&B, 2, 3, b_data);
    arm_mat_init_f32(&C, 3, 3, c_data);
    arm_mat_init_f32(&Inv, 3, 3, inv_data);

    arm_status status;
    status = arm_mat_mult_f32(&A, &B, &C);
    
    if (status == ARM_MATH_SUCCESS) {
        print_matrix("C = A * B", &C);
    } else {
        printf("Error: Matrix Multiplication failed! Status: %d\n", status);
    }

    status = arm_mat_inverse_f32(&C, &Inv);
    
    if (status == ARM_MATH_SUCCESS) {
        print_matrix("C_inverse", &Inv);
    } else if (status == ARM_MATH_SINGULAR) {
        printf("Error: Matrix C is singular and cannot be inverted!\n");
    } else {
        printf("Error: Matrix Inverse failed! Status: %d\n", status);
    }

    return 0;
}
