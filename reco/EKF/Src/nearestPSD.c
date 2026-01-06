#include "ekf.h"

/**
 * @brief Project a covariance matrix to the nearest positive semi-definite (PSD) matrix.
 *
 * This function ensures that a covariance matrix remains symmetric and
 * positive semi-definite (PSD), which is required for numerical stability
 * and physical validity in Kalman filtering.
 *
 * The procedure is:
 * 1. Symmetrize the input matrix:
 *    \f[
 *      \mathbf{P} \leftarrow \frac{1}{2}(\mathbf{P} + \mathbf{P}^T)
 *    \f]
 * 2. Perform eigenvalue decomposition:
 *    \f[
 *      \mathbf{P} = \mathbf{V}\boldsymbol{\Lambda}\mathbf{V}^T
 *    \f]
 * 3. Clamp negative eigenvalues to a small positive value proportional
 *    to the largest eigenvalue.
 * 4. Reconstruct the corrected covariance matrix:
 *    \f[
 *      \mathbf{P}_{\text{PSD}} =
 *      \mathbf{V}\boldsymbol{\Lambda}_{\text{clamped}}\mathbf{V}^T
 *    \f]
 *
 * If no negative eigenvalues are detected, the original covariance
 * matrix is returned unchanged.
 *
 * @param[in]  P            Input covariance matrix (21×21).
 * @param[out] PCorrect     Output covariance matrix guaranteed to be
 *                          symmetric and positive semi-definite.
 * @param[out] PCorrData    User-provided buffer backing @p PCorrect
 *                          (size = 21×21 floats).
 *
 * @note Eigenvalue decomposition is performed in double precision
 *       to improve numerical robustness.
 *
 * @note Negative eigenvalues are replaced with:
 *       \f[
 *           \lambda_i = 10^{-8} \cdot \max_j |\lambda_j|
 *       \f]
 *       and capped to avoid excessive scaling.
 *
 * @warning This function modifies the covariance eigenstructure and
 *          may slightly inflate uncertainty. It should be used only
 *          as a safeguard against numerical instability.
 *
 * @warning The computational cost is significant (\f$O(n^3)\f$) due to
 *          eigenvalue decomposition and should not be called at high rates.
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

    // Step 2: Eigen-decomposition: [V, D] = eig(P)
    arm_matrix_instance_f64 D, V, VT;
    float64_t realD[21], realV[21*21];
    float64_t imagD[21], imagV[21*21];

    bool test = eig(PDouble.pData, realD, imagD, realV, imagV, 21);

    arm_mat_init_f64(&V, 21, 21, realV);
    arm_mat_init_f64(&D, 21, 1, realD);
    arm_mat_init_f64(&VT, 21, 21, imagV);

    arm_mat_trans_f64(&V, &VT);

    arm_matrix_instance_f64 eigvalDiag;
    float64_t eigvalDiagData[21*21] = {0};
    arm_mat_get_diag_f64(&D, &eigvalDiag, eigvalDiagData);

//    arm_matrix_instance_f64 temp;
//    float64_t tempData[21*21];
//    arm_mat_init_f64(&temp, 21, 21, tempData);
//    arm_mat_mult_f64(&V, &eigvalDiag, &temp);
//
//    arm_mat_mult_f64(&temp, &VT, &V);

    bool corrected = false;
    float64_t largestValue = 0;

    for (uint8_t i = 0; i < D.numRows; i++) {
        if (D.pData[i] < 0) {
        	corrected = true;
        }

        if (fabs(D.pData[i]) > fabs(largestValue)) {
        	largestValue = fabs(D.pData[i]);
        }
    }

    if (largestValue >= 100) {
    	largestValue = 100;
    }

    if (corrected) {

        // printf("Negative Eigenvalues Detected.\n");

    	for (uint8_t i = 0; i < D.numRows; i++) {
    		if (D.pData[i] < 0) {
    			D.pData[i] = 1e-8 * largestValue;
    		}
    	}

    	// Eigenvalues (Lambda) on diagonal matrix
//    	memset(PDataCopy, 0, 21*21*sizeof(float64_t));
//    	for (uint8_t i = 0; i < 21; i++) {
//    	    PDataCopy[i*21 + i] = D.pData[21 - 1 - i]; // place reversed eigenvalues on the diagonal
//    	}

    	arm_mat_get_diag_f64(&D, &PDouble, PDouble.pData);

    	// V*Lambda
    	arm_mat_mult_f64(&V, &PDouble,  &PTrans);

    	//V*Lambda*V'
    	arm_mat_mult_f64(&PTrans, &VT, &V);

    	// Copy Matrix to Float
        arm_mat_init_f32(PCorrect, 21, 21, PCorrData);
    	copyMatrixFloat(&V, PCorrect);

    } else {
        memcpy(PCorrData, P->pData, 21*21*sizeof(float32_t));
        arm_mat_init_f32(PCorrect, 21, 21, PCorrData);
    }

    return;
}
