#include "Inc/ekf.h"

/**
 * nearestPSD - Make a symmetric matrix positive semi-definite
 *
 * @param P           Input symmetric matrix (21x21)
 * @param PCorrect    Output corrected matrix (21x21)
 * @param PCorrData   Preallocated buffer for PCorrect (21*21 floats)
 */
void nearestPSD(arm_matrix_instance_f32* P,
                arm_matrix_instance_f32* PCorrect,
                float32_t PCorrData[21*21])
{
    // Step 1: Symmetrize P -> P = (P + P')/2
    float32_t PTData[21*21];
    arm_matrix_instance_f32 PTrans;
    arm_mat_init_f32(&PTrans, 21, 21, PTData);
    arm_mat_trans_f32(P, &PTrans);

    arm_mat_add_f32(P, &PTrans, P);
    arm_mat_scale_f32(P, 0.5f, P);

    // Step 2: Eigen-decomposition: [V, D] = eig(P)
    arm_matrix_instance_f32 D, V;
    float32_t DBuff[21], VBuff[21*21];

    qr_eigenvalues_vectors(P, &D, DBuff, &V, VBuff, 1e-6f, 100);

    // Step 3: Check for small/negative eigenvalues and clamp
    bool corrected = false;
    for (uint8_t i = 0; i < D.numCols; i++) {
        if (D.pData[i] < 1e-10f) {
            corrected = true;
            D.pData[i] = 1e-8f;
        }
    }

    if (corrected) {
        // Step 4: Reconstruct P_corrected = sum_i d(i) * v_i * v_i'
        memset(PCorrData, 0, 21*21*sizeof(float32_t));

        for (uint8_t i = 0; i < D.numCols; i++) {
            // Extract column i of V
            float32_t viData[21];
            for (uint8_t row = 0; row < 21; row++) {
                viData[row] = VBuff[row*21 + i]; // column-major access
            }
            arm_matrix_instance_f32 vi;
            arm_mat_init_f32(&vi, 21, 1, viData);

            // Compute outer product vi*vi'
            float32_t viOPData[21*21];
            arm_matrix_instance_f32 viOP;
            arm_mat_init_f32(&viOP, 21, 21, viOPData);
            arm_mat_outer_product_f32(&vi, &viOP, viOPData);

            // Scale by eigenvalue d(i)
            arm_mat_scale_f32(&viOP, D.pData[i], &viOP);

            // Add to PCorrData
            arm_matrix_instance_f32 PCorrTemp;
            arm_mat_init_f32(&PCorrTemp, 21, 21, PCorrData);
            arm_mat_add_f32(&PCorrTemp, &viOP, &PCorrTemp);
            memcpy(PCorrData, PCorrTemp.pData, 21*21*sizeof(float32_t));
        }
        // Initialize output CMSIS matrix
        arm_mat_init_f32(PCorrect, 21, 21, PCorrData);
    } else {
        // Matrix already PSD
        memcpy(PCorrData, P->pData, 21*21*sizeof(float32_t));
        arm_mat_init_f32(PCorrect, 21, 21, PCorrData);
    }
}
