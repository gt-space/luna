#include "ekf.h"

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
	arm_matrix_instance_f64 PDouble;
    float64_t PTData[21*21];
    float64_t PDataCopy[21*21];
    arm_mat_init_f64(&PDouble, 21, 21, PDataCopy);
    copyMatrixDouble(P, &PDouble);

    arm_matrix_instance_f64 PTrans;
    arm_mat_init_f64(&PTrans, 21, 21, PTData);
    arm_mat_trans_f64(&PDouble, &PTrans);

    arm_mat_add_f64(&PDouble, &PTrans, &PDouble);
    arm_mat_scale_f64(&PDouble, 0.5f, &PDouble);
    //printMatrixDouble(&PDouble);

    // Step 2: Eigen-decomposition: [V, D] = eig(P)
    arm_matrix_instance_f64 d, V;
    float64_t DRealBuff[21], VRealBuff[21*21];
    float64_t DImagBuff[21], VImagBuff[21*21];

    arm_matrix_instance_f64 PDoubleCol;
    float64_t PDoubleColData[21*21];
    arm_mat_to_colmajor(&PDouble, &PDoubleCol, PDoubleColData);

    eig(PDoubleColData, DRealBuff, DImagBuff, VRealBuff, VImagBuff, 21);

    // Step 3: Check for small/negative eigenvalues and clamp
    float64_t DRealBuffRow[21], VRealBuffRow[21*21];
    arm_mat_to_rowmajor(&(arm_matrix_instance_f64){21, 1, DRealBuff}, &d, DRealBuffRow);
    arm_mat_to_rowmajor(&(arm_matrix_instance_f64){21, 21, VRealBuff}, &V, VRealBuffRow);

    //printMatrixDouble(&d);
    //printMatrixDouble(&V);

    bool corrected = false;
    for (uint8_t i = 0; i < d.numRows; i++) {
        if (d.pData[i] < 1e-10f) {
            corrected = true;
            d.pData[i] = 1e-8f;
        }
    }

    if (corrected) {
        // Step 4: Reconstruct P_corrected = sum_i d(i) * v_i * v_i'
        memset(PCorrData, 0, 21*21*sizeof(float32_t));

        for (uint8_t i = 0; i < d.numCols; i++) {
            // Extract column i of V
            float64_t viData[21];
            for (uint8_t row = 0; row < 21; row++) {
                viData[row] = V.pData[row*21 + i]; // column-major access
            }
            arm_matrix_instance_f64 vi;
            arm_mat_init_f64(&vi, 21, 1, viData);

            // Compute outer product vi*vi'
            float64_t viOPData[21*21];
            arm_matrix_instance_f64 viOP;
            arm_mat_init_f64(&viOP, 21, 21, viOPData);
            arm_mat_outer_product_f64(&vi, &viOP, viOPData);

            // Scale by eigenvalue d(i)
            arm_mat_scale_f64(&viOP, d.pData[i], &viOP);

            // Add to PCorrData
            arm_matrix_instance_f64 PCorrTemp;
            float64_t PCorrDataTemp[21*21];
            arm_mat_init_f64(&PCorrTemp, 21, 21, PCorrDataTemp);

            arm_mat_add_f64(&PCorrTemp, &viOP, &PCorrTemp);
            arm_mat_init_f32(PCorrect, 21, 21, PCorrData);
            copyMatrixFloat(&PCorrTemp, PCorrect);
        }
    } else {
        // Matrix already PSD
        memcpy(PCorrData, P->pData, 21*21*sizeof(float32_t));
        arm_mat_init_f32(PCorrect, 21, 21, PCorrData);
    }
}
