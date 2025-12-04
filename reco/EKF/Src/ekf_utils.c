#include "ekf_utils.h"

// attitude, pos, vel, g bias, a bias, g sf, a sf
// x0 = [q0; lla0; zeros(3,1); zeros(3,1); zeros(3,1); zeros(3,1); zeros(3,1)];
inline void getStateQuaternion(const arm_matrix_instance_f32* x, arm_matrix_instance_f32* quaternion, float32_t quaternionData[4]) {
	quaternionData[0] = x->pData[0];
	quaternionData[1] = x->pData[1];
	quaternionData[2] = x->pData[2];
	quaternionData[3] = x->pData[3];

	arm_mat_init_f32(quaternion, 4, 1, quaternionData);
}

inline void getStatePosition(const arm_matrix_instance_f32* x, arm_matrix_instance_f32* position, float32_t posData[3]) {
	posData[0] = x->pData[4];
	posData[1] = x->pData[5];
	posData[2] = x->pData[6];

	arm_mat_init_f32(position, 3, 1, posData);
}

inline void getStateVelocity(const arm_matrix_instance_f32* x, arm_matrix_instance_f32* vel, float32_t velData[3]) {
	velData[0] = x->pData[7];
	velData[1] = x->pData[8];
	velData[2] = x->pData[9];

	arm_mat_init_f32(vel, 3, 1, velData);
}

inline void getStateGBias(const arm_matrix_instance_f32* x, arm_matrix_instance_f32* gBias, float32_t gData[3]) {
	gData[0] = x->pData[10];
	gData[1] = x->pData[11];
	gData[2] = x->pData[12];

	arm_mat_init_f32(gBias, 3, 1, gData);
}

inline void getStateABias(const arm_matrix_instance_f32* x, arm_matrix_instance_f32* aBias, float32_t aData[3]) {
	aData[0] = x->pData[13];
	aData[1] = x->pData[14];
	aData[2] = x->pData[15];

	arm_mat_init_f32(aBias, 3, 1, aData);
}

inline void getStateGSF(const arm_matrix_instance_f32* x, arm_matrix_instance_f32* g_sf, float32_t g_sf_data[3]) {
	g_sf_data[0] = x->pData[16];
	g_sf_data[1] = x->pData[17];
	g_sf_data[2] = x->pData[18];

	arm_mat_init_f32(g_sf, 3, 1, g_sf_data);
}

inline void getStateASF(const arm_matrix_instance_f32* x, arm_matrix_instance_f32* a_sf, float32_t a_sf_data[3]) {
	a_sf_data[0] = x->pData[19];
	a_sf_data[1] = x->pData[20];
	a_sf_data[2] = x->pData[21];

	arm_mat_init_f32(a_sf, 3, 1, a_sf_data);
}

// Doesn't implement the check to make sure that is 1x4 vector (lines 6 through 9 in the respective .m file)
void quaternion2DCM(const arm_matrix_instance_f32* quaternion, arm_matrix_instance_f32* CB2I, float32_t CB2IBuffer[9]) {
    // Matrices and buffers
    arm_matrix_instance_f32 qNormMat, scalarQ, vectorQ;
    arm_matrix_instance_f32 identityMat, eyeDot, outerProduct, skewMat, term1;

    // Normalize quaternion
    float32_t qNorm[4];
    arm_quaternion_normalize_f32(quaternion->pData, qNorm, 1);
    arm_mat_init_f32(&qNormMat, 4, 1, qNorm);

    // Extract scalar (s) and vector (v)
    float32_t scalarVal, vectorVal[3];
    arm_quaternion_scalar_f32(&qNormMat, &scalarQ, &scalarVal);
    arm_quaternion_vector_f32(&qNormMat, &vectorQ, vectorVal);

    // Compute v·v
    float32_t vDotProd;
    arm_dot_prod_f32(vectorVal, vectorVal, 3, &vDotProd);

    // Identity matrix I
    float32_t identityData[9];
    arm_mat_eye_f32(&identityMat, identityData, 3);

    // (s^2 - dot(v,v)) * I
    float32_t eyeDotData[9];
    arm_mat_init_f32(&eyeDot, 3, 3, eyeDotData);
    float32_t firstPart = scalarVal * scalarVal - vDotProd;
    arm_mat_scale_f32(&identityMat, firstPart, &eyeDot);

    // 2 * v * v'
    float32_t outerProductData[9];
    arm_mat_outer_product_f32(&vectorQ, &outerProduct, outerProductData);
    arm_mat_scale_f32(&outerProduct, 2.0f, &outerProduct);

    // 2 * s * skew(v)
    float32_t skewData[9];
    arm_mat_skew_f32(&vectorQ, &skewMat, skewData);
    arm_mat_scale_f32(&skewMat, 2.0f * scalarVal, &skewMat);

    // Sum: CB2I = (s^2 - v·v)I + 2vv' + 2s*skew(v)
    float32_t term1Data[9];
    arm_mat_init_f32(&term1, 3, 3, term1Data);
    arm_mat_add_f32(&eyeDot, &outerProduct, &term1);

    arm_mat_init_f32(CB2I, 3, 3, CB2IBuffer);
    arm_mat_add_f32(&term1, &skewMat, CB2I);
    return;
}

void compute_radii(float32_t phi, float32_t returnVector[4]) {
	float32_t a = 6378137; // semi-major axis
	float32_t b = 6356752.31425; // semi-minor axis
	float32_t ecc = 1 - ((b / a) * (b / a)); // eccentricity

	float32_t num1 = a * (1 - ecc);
	float32_t den1 = powf((1 - ecc * arm_sind_f32(phi) * arm_sind_f32(phi)), 1.5f);

	float32_t den2;
	arm_sqrt_f32(1 - ecc * arm_sind_f32(phi) * arm_sind_f32(phi), &den2);

	float32_t R_phi = num1 / den1;
	float32_t R_lamb = a / den2;

	float32_t num3 = 3 * a * (1 - ecc) * ecc * arm_sind_f32(phi) * arm_cosd_f32(phi);
	float32_t den3 = powf((1 - ecc * arm_sind_f32(phi) * arm_sind_f32(phi)), 2.5f);

	float32_t num4 = a * ecc * arm_sind_f32(phi) * arm_cosd_f32(phi);
	float32_t den4 = powf((1 - ecc * arm_sind_f32(phi) * arm_sind_f32(phi)), 1.5f);

	float32_t dR_lamb_dphi = num4 / den4;
	float32_t dR_phi_dphi = num3 / den3;

	returnVector[0] = R_phi;
	returnVector[1] = R_lamb;
	returnVector[2] = dR_phi_dphi;
	returnVector[3] = dR_lamb_dphi;
}

void compute_g_dg(float32_t phi, float32_t h, float32_t gDgResult[3]) {
	float32_t sin_phi = arm_sind_f32(phi);
	float32_t cos_phi = arm_cosd_f32(phi);
	float32_t sin_phi_sq = sin_phi * sin_phi;
	float32_t sin_2phi = arm_sind_f32(2.0f * phi);

	// Compute dg_dphi - Eqn 7.84a
	float32_t term1 = 1.06048e-2f * sin_phi * cos_phi;
	float32_t term2 = 4.64e-5f * (sin_phi * cos_phi * cos_phi * cos_phi - sin_phi * sin_phi * sin_phi * cos_phi);
	float32_t term3 = 8.8e-9f * h * sin_phi * cos_phi;

	// g
	gDgResult[0] = 9.780327f * (1.0f + 5.3024e-3f * sin_phi * sin_phi - 5.8e-6f * sin_2phi * sin_2phi)
	        	   - (3.0877e-6f - 4.4e-9f * sin_phi * sin_phi) * h
				   + 7.2e-14f * h * h;

	// dg_dphi
	gDgResult[1] = 9.780327f * (term1 - term2) + term3;

	// Compute dg_dh - Eqn 7.84b
	gDgResult[2] = -3.0877e-6f + 4.4e-9f * sin_phi_sq + 1.44e-13f * h;
}

__attribute__((used))
void printMatrix(arm_matrix_instance_f32* matrix) {
    printf("[\n");
    for (uint16_t i = 0; i < matrix->numRows; i++) {
        for (uint16_t j = 0; j < matrix->numCols; j++) {
            // % .8e → scientific notation with 8 digits after the decimal
            printf("%15.9e ", matrix->pData[i * matrix->numCols + j]);
        }
        printf("\n");
    }
    printf("]\n\n");
}

__attribute__((used))
void printMatrixDouble(arm_matrix_instance_f64* matrix) {
    printf("[\n");
    for (uint16_t i = 0; i < matrix->numRows; i++) {
        for (uint16_t j = 0; j < matrix->numCols; j++) {
            // % .8e → scientific notation with 8 digits after the decimal
            printf("%15.9e ", matrix->pData[i * matrix->numCols + j]);
        }
        printf("\n");
    }
    printf("]\n\n");
}


bool areMatricesEqual(arm_matrix_instance_f32* A, arm_matrix_instance_f32* B) {
    // Check dimensions first
    if (A->numRows != B->numRows || A->numCols != B->numCols) {
        return false;
    }

    // Compare each element using the constant tolerance
    for (uint16_t i = 0; i < A->numRows; i++) {
        for (uint16_t j = 0; j < A->numCols; j++) {
            float32_t diff = A->pData[i * A->numCols + j] - B->pData[i * B->numCols + j];
            if (diff < -1e-6f || diff > 1e-6f) {
            	printf("Failed at [%d,%d]\n", i, j);
                return false;
            }
        }
    }

    return true;
}

void copyMatrixDouble(arm_matrix_instance_f32* src, arm_matrix_instance_f64* dest) {

	uint32_t total = src->numRows * src->numCols;
	for (uint32_t i = 0; i < total; i++) {
	    dest->pData[i] = (float64_t) src->pData[i];
	}

}

void copyMatrixFloat(arm_matrix_instance_f64* src, arm_matrix_instance_f32* dest) {

	uint32_t total = src->numRows * src->numCols;
	for (uint32_t i = 0; i < total; i++) {
	    dest->pData[i] = (float32_t) src->pData[i];
	}

}

void arm_mat_to_colmajor(arm_matrix_instance_f64 *src, arm_matrix_instance_f64* dst, float64_t* destData) {
    uint16_t m = src->numRows;
    uint16_t n = src->numCols;

    dst->numRows = m;
    dst->numCols = n;

    // Allocate memory for the column-major buffer
    dst->pData = destData;

    const float64_t *row = src->pData;
    float64_t *col = dst->pData;

    // Convert row-major → column-major
    for (uint16_t i = 0; i < m; i++) {
        for (uint16_t j = 0; j < n; j++) {
            col[j * m + i] = row[i * n + j];
        }
    }
}

void arm_mat_to_rowmajor(arm_matrix_instance_f64* src, arm_matrix_instance_f64* dst, float64_t* data) {
    uint16_t m = src->numRows;
    uint16_t n = src->numCols;

    dst->numRows = m;
    dst->numCols = n;

    // Allocate output buffer (row-major)
    dst->pData = data;

    const float64_t *col = src->pData;  // input is column-major
    float64_t *row = dst->pData;         // output is row-major

    // Convert column-major → row-major
    for (uint16_t i = 0; i < m; i++) {
        for (uint16_t j = 0; j < n; j++) {
            // row-major index:    i*n + j
            // column-major index: j*m + i
            row[i * n + j] = col[j * m + i];
        }
    }
}

void copyMatrix(float32_t* src, float32_t* dest, uint16_t total) {
	for (uint32_t i = 0; i < total; i++) {
	    dest[i] = src[i];
	}
}

void calculateEigSym(arm_matrix_instance_f32* A) {
	arm_matrix_instance_f64 PDouble;
    float64_t PTData[21*21];
    float64_t PDataCopy[21*21];
    arm_mat_init_f64(&PDouble, 21, 21, PDataCopy);
    copyMatrixDouble(A, &PDouble);

    arm_matrix_instance_f64 PTrans;
    arm_mat_init_f64(&PTrans, 21, 21, PTData);
    arm_mat_trans_f64(&PDouble, &PTrans);

    arm_mat_add_f64(&PDouble, &PTrans, &PDouble);
    arm_mat_scale_f64(&PDouble, 0.5f, &PDouble);
    printMatrixDouble(&PDouble);

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

    printMatrixDouble(&d);
}


