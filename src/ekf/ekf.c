#include "ekf.h"

#include <string.h>

#include "dsp/matrix_functions.h"

void EKF_Init(EKF_Context *ctx, uint16_t n, uint16_t m, float32_t *work_mem,
              float32_t *x_ptr, float32_t *P_ptr, float32_t *A_ptr,
              float32_t *Q_ptr, float32_t *R_ptr) {
    ctx->n = n;
    ctx->m = m;

    arm_mat_init_f32(&ctx->x, n, 1, x_ptr);
    arm_mat_init_f32(&ctx->P, n, n, P_ptr);
    arm_mat_init_f32(&ctx->A, n, n, A_ptr);
    arm_mat_init_f32(&ctx->Q, n, n, Q_ptr);
    arm_mat_init_f32(&ctx->R, m, m, R_ptr);

    float32_t *p = work_mem;
    arm_mat_init_f32(&ctx->T1, n, n, p);
    p += n * n;
    arm_mat_init_f32(&ctx->T2, n, n, p);
    p += n * n;
    arm_mat_init_f32(&ctx->K, n, m, p);
    p += n * m;
    arm_mat_init_f32(&ctx->S, m, m, p);
    p += m * m;
    arm_mat_init_f32(&ctx->invS, m, m, p);
    p += m * m;
    arm_mat_init_f32(&ctx->HT, n, m, p);
    p += n * m;
    arm_mat_init_f32(&ctx->HP, m, n, p);

    ctx->f = NULL;
    ctx->h = NULL;
}

void EKF_Predict(EKF_Context *ctx, arm_matrix_instance_f32 *u) {
    // x = f(x, u) or x = A * x
    if (ctx->f) {
        ctx->f(&ctx->x, u, &ctx->x);
    } else {
        arm_mat_mult_f32(&ctx->A, &ctx->x, &ctx->T1);
        memcpy(ctx->x.pData, ctx->T1.pData, ctx->n * sizeof(float32_t));
    }

    // P = A * P * A' + Q
    arm_mat_trans_f32(&ctx->A, &ctx->T1);          // T1 = A'
    arm_mat_mult_f32(&ctx->A, &ctx->P, &ctx->T2);  // T2 = A * P
    arm_mat_mult_f32(&ctx->T2, &ctx->T1, &ctx->P);
    arm_mat_add_f32(&ctx->P, &ctx->Q, &ctx->P);
}

void EKF_Update(EKF_Context *ctx, arm_matrix_instance_f32 *H,
                arm_matrix_instance_f32 *z) {
    uint16_t m = z->numRows;
    uint16_t n = ctx->n;

    ctx->R.numRows = ctx->R.numCols = m;
    ctx->S.numRows = ctx->S.numCols = m;
    ctx->invS.numRows = ctx->invS.numCols = m;
    ctx->K.numCols = ctx->HT.numCols = ctx->HP.numRows = m;

    // dy = z - h(x)
    float32_t dy_data[m];
    arm_matrix_instance_f32 dy;
    arm_mat_init_f32(&dy, m, 1, dy_data);
    if (ctx->h) {
        ctx->h(&ctx->x, &dy);
    } else {
        arm_mat_mult_f32(H, &ctx->x, &dy);
    }
    arm_mat_sub_f32(z, &dy, &dy);

    // K = P * H' * (H * P * H' + R)^-1
    arm_mat_trans_f32(H, &ctx->HT);
    arm_mat_mult_f32(H, &ctx->P, &ctx->HP);
    arm_mat_mult_f32(&ctx->HP, &ctx->HT, &ctx->S);
    arm_mat_add_f32(&ctx->S, &ctx->R, &ctx->S);
    arm_mat_inverse_f32(&ctx->S, &ctx->invS);
    arm_mat_mult_f32(&ctx->P, &ctx->HT, &ctx->K);
    arm_mat_mult_f32(&ctx->K, &ctx->invS, &ctx->K);

    // x = x + K * dy
    arm_mat_init_f32(&ctx->T1, n, 1, ctx->T1.pData);
    arm_mat_mult_f32(&ctx->K, &dy, &ctx->T1);  // T1 = K * dy
    arm_mat_add_f32(&ctx->x, &ctx->T1, &ctx->x);

    // P = (I - KH) * P
    arm_mat_init_f32(&ctx->T1, n, n, ctx->T1.pData);
    arm_mat_mult_f32(&ctx->K, H, &ctx->T1);  // T1 = KH
    arm_mat_scale_f32(&ctx->T1, -1.0f, &ctx->T1);
    for (int i = 0; i < n; i++)  // I think doing like this is faster
        ctx->T1.pData[i * n + i] += 1.0f;
    arm_mat_mult_f32(&ctx->T1, &ctx->P, &ctx->T2);
    memcpy(ctx->P.pData, ctx->T2.pData, n * n * sizeof(float32_t));
}
