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


