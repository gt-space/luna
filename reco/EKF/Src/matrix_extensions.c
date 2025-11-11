#include "Inc/matrix_extensions.h"

/**
 * @brief Creates an identity matrix of given dimension.
 *
 * This function generates a square identity matrix of size dim x dim, where
 * all diagonal elements are set to 1.0 and all off-diagonal elements are 0.0.
 *
 * @param[out] outputMatrix   Pointer to the output matrix instance.
 * @param[out] outMatrixData  Pointer to a preallocated float32_t array of size dim*dim
 *                            to store the matrix data.
 * @param[in]  dim            Dimension of the square identity matrix.
 *
 * @return None
*/
void arm_mat_eye_f32(arm_matrix_instance_f32* outputMatrix, float32_t* outMatrixData, uint32_t dim) {
    memset(outMatrixData, 0, sizeof(float32_t) * dim * dim);

    for (uint32_t i = 0; i < dim; i++) {
        outMatrixData[i*dim + i] = 1.0f;
    }

    arm_mat_init_f32(outputMatrix, dim, dim, outMatrixData);
}

/**
 * @brief  Create a 3x3 skew-symmetric matrix from a 3D vector
 * @param[in]  inputVector   Pointer to 3-element input vector [v1, v2, v3]
 * @param[out]  outputMatrix   Pointer to an initialized arm_matrix_instance_f32 struct
 * @param[out]  outMatrixData Pointer to a 9-element float32_t array used as matrix storage
 * @note   The matrix will be initialized with the skew-symmetric form of v.
 *         The 'data' buffer must remain valid while using S in other CMSIS-DSP functions.
 */
void arm_mat_skew_f32(const arm_matrix_instance_f32* inputVector, arm_matrix_instance_f32* outputMatrix, float32_t outMatrixData[9]) {
    float32_t* v = inputVector->pData;

    outMatrixData[0] =  0.0f;   outMatrixData[1] = -v[2];  outMatrixData[2] =  v[1];
    outMatrixData[3] =  v[2];   outMatrixData[4] =  0.0f;  outMatrixData[5] = -v[0];
    outMatrixData[6] = -v[1];   outMatrixData[7] =  v[0];  outMatrixData[8] =  0.0f;

    arm_mat_init_f32(outputMatrix, 3, 3, outMatrixData);
}

/**
 * @brief Computes the outer product of a floating-point vector.
 *        outputMatrix = inputVector * inputVector'
 *
 * This function generates a square matrix where each element (i,j) is the
 * product of the i-th and j-th elements of the input vector:
 * \f$ M_{ij} = v_i \cdot v_j \f$.
 *
 * @param[in]  inputVector     Pointer to the input vector (arm_matrix_instance_f32).
 *                             Must be a column vector (n x 1).
 * @param[out] outputMatrix    Pointer to the output n x n matrix instance.
 * @param[out] outMatrixData   Pointer to a preallocated float32_t array of size n*n
 *                             to store the output matrix data.
 * @return None
 */
void arm_mat_outer_product_f32(const arm_matrix_instance_f32* inputVector, arm_matrix_instance_f32* outputMatrix, float32_t* outMatrixData) {
    uint16_t n = inputVector->numRows;

    arm_mat_init_f32(outputMatrix, n, n, outMatrixData);

    const float32_t* v_data = inputVector->pData;

    for (uint16_t i = 0; i < n; i++) {
        for (uint16_t j = 0; j < n; j++) {
            outMatrixData[i * n + j] = v_data[i] * v_data[j];
        }
    }
}

/**
 * @brief Creates a square diagonal matrix from a floating-point input matrix.
 *
 * This function generates a square matrix with the diagonal elements taken
 * from the input matrix. Non-diagonal elements are set to zero. The output
 * matrix is of size n x n, where n = max(numRows, numCols) of the input matrix.
 *
 * @param[in]  inputMatrix   Pointer to the input matrix (arm_matrix_instance_f32).
 * @param[out] outputMatrix  Pointer to the output square diagonal matrix instance.
 * @param[out] outputData    Pointer to a preallocated float32_t array of size n*n
 *                           to store the output matrix data.
 *
 * @return None
 */
void arm_mat_get_diag_f32(const arm_matrix_instance_f32* inputMatrix, arm_matrix_instance_f32* outputMatrix, float32_t* outputData) {
    uint16_t rows = inputMatrix->numRows;
    uint16_t cols = inputMatrix->numCols;
    float32_t *pIn = inputMatrix->pData;
    float32_t *pOut = outputData;

    uint16_t n = (rows > cols) ? rows : cols;

    // Clear output matrix
    memset(pOut, 0, n * n * sizeof(float32_t));


    for (uint16_t i = 0; i < n; i++) {
        pOut[i * n + i] = pIn[i];
    }

    arm_mat_init_f32(outputMatrix, n, n, outputData);
}

/**
 * @brief Extracts the main diagonal elements from a floating-point matrix.
 *
 * This function copies the elements along the main diagonal of the input matrix
 * into a separate output matrix represented as a column vector. The output
 * matrix is initialized with dimensions (n x 1), where n = min(numRows, numCols)
 * of the input matrix.
 *
 * The function does not allocate memory; instead, the caller must provide
 * a pointer to a preallocated array `outputData` large enough to hold n elements.
 *
 * @param[in]  inputMatrix   Pointer to the input matrix of type arm_matrix_instance_f32.
 * @param[out] outputMatrix  Pointer to the output matrix instance (column vector)
 *                           that will be initialized by this function.
 * @param[out] outputData    Pointer to a preallocated float32_t array of length n
 *                           to store the diagonal elements.
 *
 * @return None
 */
void arm_mat_extract_diag(const arm_matrix_instance_f32* inputMatrix, arm_matrix_instance_f32* outputMatrix, float32_t* outputData) {
    uint16_t rows = inputMatrix->numRows;
    uint16_t cols = inputMatrix->numCols;
    float32_t *pIn = inputMatrix->pData;
    float32_t *pOut = outputData;

    uint16_t n = (rows < cols) ? rows : cols;

    for (uint16_t i = 0; i < n; i++) {
        pOut[i] = pIn[i * cols + i];
    }

    arm_mat_init_f32(outputMatrix, n, 1, outputData);
}

/**
 * @brief Places a submatrix inside a larger parent matrix at the specified offset.
 *
 * @param subMatrix Pointer to the smaller matrix (to be inserted)
 * @param destMatrix Pointer to the larger matrix (destination)
 * @param rowOffset Starting row index in destMatrix where subMatrix[0][0] will be placed
 * @param colOffset Starting column index in destMatrix where subMatrix[0][0] will be placed
 * @retval arm_status ARM_MATH_SUCCESS if successful, ARM_MATH_ARGUMENT_ERROR if out of bounds
 */
arm_status arm_mat_place_f32(const arm_matrix_instance_f32* subMatrix,
                                arm_matrix_instance_f32* destMatrix,
                                uint16_t rowOffset,
                                uint16_t colOffset) {
    // Check that submatrix fits inside destination
    if ((rowOffset + subMatrix->numRows > destMatrix->numRows) ||
        (colOffset + subMatrix->numCols > destMatrix->numCols)) {
        return ARM_MATH_ARGUMENT_ERROR; // Out of bounds
    }

    // Insert submatrix into destination
    for (uint16_t r = 0; r < subMatrix->numRows; r++) {
        for (uint16_t c = 0; c < subMatrix->numCols; c++) {
            uint32_t destIndex = (r + rowOffset) * destMatrix->numCols + (c + colOffset);
            uint32_t subIndex  = r * subMatrix->numCols + c;
            destMatrix->pData[destIndex] = subMatrix->pData[subIndex];
        }
    }

    return ARM_MATH_SUCCESS;
}

/**
 * @brief Fill a matrix with ones.
 */
void arm_mat_ones_f32(arm_matrix_instance_f32* outputMatrix, float32_t* outMatrixData, uint32_t dim) {
    for (uint32_t i = 0; i < dim * dim; i++) {
        outMatrixData[i] = 1.0f;
    }
    arm_mat_init_f32(outputMatrix, dim, dim, outMatrixData);
}

/**
 * Solves A*X = B (least-squares) using QR factorization with CMSIS-DSP.
 * Supports tall matrices (m >= n) and multiple RHS columns (k >= 1).
 *
 * @param A[in]   Pointer to input matrix A (size m*n)
 * @param B[in]   Pointer to input matrix B (size m*k)
 * @param X[out]  Pointer to output matrix X (size n*k)
 * @param m            Number of rows of A
 * @param n            Number of columns of A
 * @param k            Number of columns of B
 * @return ARM_MATH_SUCCESS if successful, otherwise error code
 */
arm_status arm_mat_lin_qr_solve_right(arm_matrix_instance_f32* A, arm_matrix_instance_f32* B, arm_matrix_instance_f32* x, float32_t* xData) {
	arm_status status;

	// Initialize CMSIS matrix instances
	arm_matrix_instance_f32 Q, R, Y;

	uint8_t m = A->numRows;
	uint8_t k = B->numCols;
	uint8_t n = A->numCols;

	// Temporary buffers (stack allocation)
	float32_t Q_data[m*m];    // full Q
	float32_t R_data[m*n];    // full R
	float32_t Y_data[m*k];    // Y = Q^T * B
	float32_t tau[n];         // Householder scaling factors
	float32_t tmpA[m];        // workspace
	float32_t tmpB[m];        // workspace

	arm_mat_init_f32(&Q, m, m, Q_data);
	arm_mat_init_f32(&R, m, n, R_data);
	arm_mat_init_f32(&Y, m, k, Y_data);

	// Compute QR decomposition: A = Q*R
	status = arm_mat_qr_f32(&A, 1e-6f, &R, &Q, tau, tmpA, tmpB);
	if (status != ARM_MATH_SUCCESS) return status;

	// Compute Y = Q^T * B
	arm_matrix_instance_f32 Q_T;
	float32_t Q_T_data[m*m];
	arm_mat_init_f32(&Q_T, m, m, Q_T_data);
	status = arm_mat_trans_f32(&Q, &Q_T);
	if (status != ARM_MATH_SUCCESS) return status;

	status = arm_mat_mult_f32(&Q_T, &B, &Y);
	if (status != ARM_MATH_SUCCESS) return status;

	// Solve R_top * X = Y_top using CMSIS upper-triangular solver
	// R_top = top n x n block of R
	// Y_top = top n x k block of Y
	arm_matrix_instance_f32 R_top, Y_top, X_sol;
	arm_mat_init_f32(&R_top, n, n, R_data);  // top n x n of R_data
	arm_mat_init_f32(&Y_top, n, k, Y_data);  // top n x k of Y_data
	arm_mat_init_f32(x, n, k, xData);  // solution output

	status = arm_mat_solve_upper_triangular_f32(&R_top, &Y_top, &X_sol);
	if (status != ARM_MATH_SUCCESS) return status;

	return ARM_MATH_SUCCESS;
}


/**
 * @brief  Solves the matrix equation X * A = B using QR decomposition.
 *
 * This function computes the solution X for the left-multiplied linear system:
 *
 *      X * A = B
 *
 * where:
 * - A is an m x n matrix,
 * - B is a p x n matrix,
 * - X is the resulting p x m solution matrix.
 *
 * The function performs a QR decomposition of the transpose of A (A^T) and
 * solves the system efficiently using upper-triangular back-substitution.
 * This avoids explicit matrix inversion and is suitable for STM32/CMSIS DSP
 * applications.
 *
 * @param[in]   A       Pointer to an arm_matrix_instance_f32 representing the m x n matrix A.
 * @param[in]   B       Pointer to an arm_matrix_instance_f32 representing the p x n matrix B.
 * @param[out]  X       Pointer to an arm_matrix_instance_f32 where the p x m solution will be stored.
 * @param[in]   XData   Pointer to a float32_t buffer of size p * m to hold the solution data.
 *
 * @return  ARM_MATH_SUCCESS if the solution was computed successfully.
 * @return  ARM_MATH_SIZE_MISMATCH if matrix dimensions are incompatible.
 * @return  ARM_MATH_ARGUMENT_ERROR if a NULL pointer or invalid matrix instance is passed.
 * @return  ARM_MATH_SINGULAR if the system is singular and cannot be solved.
 *
 * @note    The function requires temporary buffers for QR decomposition (Q, R, Y)
 *          and transposes of the input matrices. Ensure sufficient stack memory is available.
 *
 * @warning This function is designed for single-precision floating-point matrices
 *          (arm_matrix_instance_f32) and is not suitable for integer or double matrices.
 */
arm_status arm_mat_lin_qr_solve_left(arm_matrix_instance_f32* A,
                                     arm_matrix_instance_f32* B,
                                     arm_matrix_instance_f32* X,
                                     float32_t* XData)
{
    arm_status status;

    uint8_t m = A->numRows;
    uint8_t n = A->numCols;
    uint8_t p = B->numRows;  // number of rows in X

    // Buffers for QR decomposition of A^T (n x m)
    float32_t A_T_data[n * m];
    arm_matrix_instance_f32 A_T;
    arm_mat_init_f32(&A_T, n, m, A_T_data);

    // Transpose A -> A_T
    status = arm_mat_trans_f32(A, &A_T);
    if (status != ARM_MATH_SUCCESS) return status;

    // Buffers for Q, R, Y
    float32_t Q_data[n * n];
    float32_t R_data[n * m];  // upper triangular
    float32_t Y_data[n * p];
    float32_t tau[m];
    float32_t tmpA[n];
    float32_t tmpB[n];

    arm_matrix_instance_f32 Q, R, Y;
    arm_mat_init_f32(&Q, n, n, Q_data);
    arm_mat_init_f32(&R, n, m, R_data);
    arm_mat_init_f32(&Y, n, p, Y_data);

    // Compute QR decomposition of A_T: A_T = Q * R
    status = arm_mat_qr_f32(&A_T, 1e-6f, &R, &Q, tau, tmpA, tmpB);
    if (status != ARM_MATH_SUCCESS) return status;

    // Compute Y = Q^T * B^T
    float32_t B_T_data[B->numCols * B->numRows];
    arm_matrix_instance_f32 B_T;
    arm_mat_init_f32(&B_T, n, p, B_T_data);

    // Transpose B -> B_T (B is p x n, B_T is n x p)
    status = arm_mat_trans_f32(B, &B_T);
    if (status != ARM_MATH_SUCCESS) return status;

    status = arm_mat_mult_f32(&Q, &B_T, &Y); // Q^T * B_T
    if (status != ARM_MATH_SUCCESS) return status;

    // Solve R_top * X_T = Y_top
    arm_matrix_instance_f32 R_top, Y_top, X_T;
    arm_mat_init_f32(&R_top, n, n, R_data);  // top n x n of R
    arm_mat_init_f32(&Y_top, n, p, Y_data);  // top n x p of Y
    arm_mat_init_f32(&X_T, n, p, XData);     // solution X^T

    status = arm_mat_solve_upper_triangular_f32(&R_top, &Y_top, &X_T);
    if (status != ARM_MATH_SUCCESS) return status;

    // Transpose solution X_T (n x p) -> X (p x m)
    status = arm_mat_trans_f32(&X_T, X);
    if (status != ARM_MATH_SUCCESS) return status;

    return ARM_MATH_SUCCESS;
}

/**
 * Compute eigenvalues and eigenvectors of a square matrix using shifted QR iteration.
 *
 * @param A        Pointer to input square matrix (n x n), will be overwritten.
 * @param D        Pointer to output CMSIS matrix (1 x n) of eigenvalues.
 * @param D_buff   float32_t buffer of size n for eigenvalues.
 * @param V        Pointer to output CMSIS matrix (n x n) of eigenvectors (columns).
 * @param V_buff   float32_t buffer of size n*n for eigenvectors.
 * @param tolerance Convergence tolerance.
 * @param maxIter  Maximum number of iterations.
 * @return ARM_MATH_SUCCESS on success.
 */
arm_status qr_eigenvalues_vectors(const arm_matrix_instance_f32 *A,
                                  arm_matrix_instance_f32 *D, float32_t *D_buff,
                                  arm_matrix_instance_f32 *V, float32_t *V_buff,
                                  float32_t tolerance, uint32_t maxIter) {

    if (!A || !D || !V || !D_buff || !V_buff) return ARM_MATH_ARGUMENT_ERROR;
    uint32_t n = A->numRows;
    if (A->numRows != A->numCols) return ARM_MATH_ARGUMENT_ERROR;

    // Copy input matrix into local workspace (overwritten during iteration)
    float32_t A_data[n*n];
    memcpy(A_data, A->pData, n*n*sizeof(float32_t));

    // Initialize V as identity matrix
    for (uint32_t i = 0; i < n; i++)
        for (uint32_t j = 0; j < n; j++)
            V_buff[i*n + j] = (i == j) ? 1.0f : 0.0f;

    arm_matrix_instance_f32 V_mat;
    arm_mat_init_f32(&V_mat, n, n, V_buff);

    // Workspace for QR decomposition
    float32_t Q_data[n*n], R_data[n*n], tau[n], tmpA[n], tmpB[n];
    arm_matrix_instance_f32 Q_mat, R_mat;
    arm_mat_init_f32(&Q_mat, n, n, Q_data);
    arm_mat_init_f32(&R_mat, n, n, R_data);

    for (uint32_t iter = 0; iter < maxIter; iter++)
    {
        // Step 1: Shift
        float32_t mu = A_data[(n-1)*n + (n-1)];

        // Step 2: Form A_shifted = A - mu*I
        float32_t A_shifted[n*n];
        for (uint32_t i = 0; i < n; i++)
            for (uint32_t j = 0; j < n; j++)
                A_shifted[i*n + j] = A_data[i*n + j] - ((i==j) ? mu : 0.0f);

        arm_matrix_instance_f32 A_shifted_mat;
        arm_mat_init_f32(&A_shifted_mat, n, n, A_shifted);

        // Step 3: QR decomposition
        arm_status status = arm_mat_qr_f32(&A_shifted_mat, tolerance,
                                           &R_mat, &Q_mat, tau, tmpA, tmpB);
        if (status != ARM_MATH_SUCCESS) return status;

        // Step 4: Compute A_next = R*Q + mu*I
        float32_t A_next[n*n];
        for (uint32_t i = 0; i < n; i++)
        {
            for (uint32_t j = 0; j < n; j++)
            {
                float32_t sum = 0.0f;
                for (uint32_t k = 0; k < n; k++)
                    sum += R_data[i*n + k] * Q_data[k*n + j];
                A_next[i*n + j] = sum + ((i==j) ? mu : 0.0f);
            }
        }

        // Step 5: Accumulate eigenvectors: V = V*Q
        float32_t V_next[n*n];
        arm_matrix_instance_f32 V_next_mat;
        arm_mat_init_f32(&V_next_mat, n, n, V_next);
        status = arm_mat_mult_f32(&V_mat, &Q_mat, &V_next_mat);
        if (status != ARM_MATH_SUCCESS) return status;
        memcpy(V_buff, V_next, n*n*sizeof(float32_t));
        arm_mat_init_f32(&V_mat, n, n, V_buff); // update V_mat

        // Copy A_next back to A_data
        memcpy(A_data, A_next, n*n*sizeof(float32_t));

        // Step 6: Check convergence (off-diagonal)
        bool converged = true;
        for (uint32_t i = 0; i < n; i++)
        {
            for (uint32_t j = 0; j < n; j++)
            {
                if (i != j && fabsf(A_data[i*n + j]) > tolerance)
                {
                    converged = false;
                    break;
                }
            }
            if (!converged) break;
        }
        if (converged) break;
    }

    // Step 7: Copy eigenvalues (diagonal) into D_buff
    for (uint32_t i = 0; i < n; i++)
        D_buff[i] = A_data[i*n + i];

    // Step 8: Initialize CMSIS matrices
    arm_mat_init_f32(D, 1, n, D_buff);
    arm_mat_init_f32(V, n, n, V_buff);

    return ARM_MATH_SUCCESS;
}

