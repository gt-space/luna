#include "ekf_utils.h"
#include "float.h"

static const float32_t a = 6378137.0f; // semi-major axis
static const float32_t b = 6356752.31425f;
static const float32_t ecc = 1.0 - ((b/a) * (b/a)); // eccentricity

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
    float32_t scalarVal = qNorm[0];
	float32_t vectorVal[3] = {qNorm[1], qNorm[2], qNorm[3]};

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
    arm_mat_outer_product_f32(&(arm_matrix_instance_f32){3, 1, vectorVal}, &outerProduct, outerProductData);
    arm_mat_scale_f32(&outerProduct, 2.0f, &outerProduct);

    // 2 * s * skew(v)
    float32_t skewData[9];
    arm_mat_skew_f32(&(arm_matrix_instance_f32){3, 1, vectorVal}, &skewMat, skewData);
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
	float32_t phiRad = deg2rad(phi);
	float32_t sinphi = arm_sin_f32(phiRad);
	float32_t cosphi = arm_cos_f32(phiRad);
	float32_t sinphi2 = sinphi * sinphi;

	float32_t sqrtF;
	arm_sqrt_f32(1.0f - ecc * sinphi2, &sqrtF);

    // Radii of curvature
	float32_t R_phi = a * (1.0f - ecc) / (sqrtF * sqrtF * sqrtF); // Eqn 7.69a (meridian)
	float32_t R_lambda = a / sqrtF; // Eqn 7.69b (prime vertical)

    // Derivatives w.r.t phi (radians) - Eqns 7.75a, 7.75b
	float32_t dR_phi_dphi = 3.0f * a * (1.0f - ecc) * ecc * sinphi * cosphi / powf(sqrtF, 5);
	float32_t dR_lambda_dphi = a * ecc * sinphi * cosphi / (sqrtF * sqrtF * sqrtF);

	returnVector[0] = R_phi;
	returnVector[1] = R_lambda;
	returnVector[2] = dR_phi_dphi;
	returnVector[3] = dR_lambda_dphi;
}

/*
WGS84 gravity model and derivatives
Returns:
    g, dg_dphi, dg_dh where phi is in radians and h is altitude in meters
*/
void compute_g_dg(float32_t phi, float32_t h, float32_t gDgResult[3]) {
	float32_t sin_phi = arm_sind_f32(phi);
	float32_t cos_phi = arm_cosd_f32(phi);
	float32_t sin_phi_sq = sin_phi * sin_phi;
	float32_t sin_2phi = arm_sind_f32(2.0f * phi);

	// Compute dg_dphi - Eqn 7.84a
	float32_t term1 = 1.06048e-2f * sin_phi * cos_phi;
	float32_t term2 = 4.64e-5f * (sin_phi * cos_phi * cos_phi * cos_phi - sin_phi * sin_phi * sin_phi * cos_phi);
	float32_t term3 = 8.8e-9f * h * sin_phi * cos_phi;

	// Surface Gravity (g)
	gDgResult[0] = 9.780327f * (1.0f + 5.3024e-3f * sin_phi * sin_phi - 5.8e-6f * sin_2phi * sin_2phi)
	        	   - (3.0877e-6f - 4.4e-9f * sin_phi * sin_phi) * h
				   + 7.2e-14f * h * h;

	// dg_dph
	gDgResult[1] = 9.780327f * (term1 - term2) + term3;

	// Compute dg_dh - Eqn 7.84b
	gDgResult[2] = -3.0877e-6f + 4.4e-9f * sin_phi_sq + 1.44e-13f * h;
}

/*
WGS84 gravity model and derivatives
Returns:
    g, dg_dphi, dg_dh where phi is in radians and h is altitude in meters
*/
void compute_g_dg2(float32_t phi_rad, float32_t h, float32_t gDgResult[3])
{
    // Precompute trig terms
    float32_t sin_phi     = arm_sin_f32(phi_rad);
    float32_t cos_phi     = arm_cos_f32(phi_rad);
    float32_t sin_phi_sq  = sin_phi * sin_phi;
    float32_t sin_2phi    = arm_sin_f32(2.0f * phi_rad);
    float32_t sin_2phi_sq = sin_2phi * sin_2phi;
    float32_t sin_4phi    = arm_sin_f32(4.0f * phi_rad);

    // Surface Gravity (g)
    float32_t g = 9.780327f * (1.0f + 5.3024e-3f * sin_phi_sq - 5.8e-6f   * sin_2phi_sq) - (3.0877e-6f - 4.4e-9f * sin_phi_sq) * h + 7.2e-14f * h * h;

    // Derivative w.r.t latitude (radians)
    float32_t dg_dphi =
        9.780327f * (5.3024e-3f * sin_2phi - 4.64e-5f * 0.25f * sin_4phi) + 4.4e-9f * h * sin_2phi;

    // Derivative w.r.t altitude
    float32_t dg_dh =
        -3.0877e-6f +
        4.4e-9f * sin_phi_sq +
        1.44e-13f * h;

    // Output
    gDgResult[0] = g;
    gDgResult[1] = dg_dphi;
    gDgResult[2] = dg_dh;
}

// Prints a given single precision matrix out using ITM printf()
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

// Prints a given double precision matrix out using ITM printf()
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


//bool areMatricesEqual(arm_matrix_instance_f32* A, arm_matrix_instance_f32* B) {
//    // Check dimensions first
//    if (A->numRows != B->numRows || A->numCols != B->numCols) {
//        return false;
//    }
//
//
//    // Compare each element using the constant tolerance
//    for (uint16_t i = 0; i < A->numRows; i++) {
//        for (uint16_t j = 0; j < A->numCols; j++) {
//
//            float32_t diff = fabsf(A->pData[i * A->numCols + j] - B->pData[i * B->numCols + j]);
//
//            if (diff < FLT_EPSILON) {
//            	continue;
//            }
//
//
//            float32_t AVal = fabsf(A->pData[i * A->numCols + j]);
//            float32_t BVal = fabsf(B->pData[i * B->numCols + j]);
//            float32_t largest = (BVal > AVal) ? BVal : AVal;
//
//            if (diff > largest * FLT_EPSILON * 10) {
//            	printf("Failed at [%d,%d]\n", i, j);
//                return false;
//            }
//        }
//    }
//
//    return true;
//}

// Checks if two matrices are equal by checking if each index if within a set number of ULPs
// Read this for the motivation behind this function:
// https://randomascii.wordpress.com/2012/02/25/comparing-floating-point-numbers-2012-edition/
bool areMatricesEqual(arm_matrix_instance_f32* A, arm_matrix_instance_f32* B) {
    // Check dimensions first
    if (A->numRows != B->numRows || A->numCols != B->numCols) {
        return false;
    }

    // Compare each element
    for (uint16_t i = 0; i < A->numRows; i++) {
        for (uint16_t j = 0; j < A->numCols; j++) {
            float aVal = A->pData[i * A->numCols + j];
            float bVal = B->pData[i * B->numCols + j];

            // Handle special cases
            if (isnan(aVal) || isnan(bVal)) {
                printf("Failed at [%d,%d]: NaN detected\n", i, j);
                return false;
            }

            // Handle infinities
            if (isinf(aVal) || isinf(bVal)) {
                if (aVal != bVal) {
                    printf("Failed at [%d,%d]: Infinity mismatch\n", i, j);
                    return false;
                }
                continue;
            }

            // Cast to unsigned for ULP comparison
            uint32_t aInt = *(uint32_t*)&aVal;
            uint32_t bInt = *(uint32_t*)&bVal;

            // Handle sign bit for two's complement-like comparison
            if (aInt & 0x80000000) aInt = 0x80000000 - aInt;
            if (bInt & 0x80000000) bInt = 0x80000000 - bInt;

            // Compare ULP distance
            uint32_t ulpDiff = (aInt > bInt) ? (aInt - bInt) : (bInt - aInt);

            if (ulpDiff >= 50) {
                printf("Failed at [%d,%d]: %.9f vs %.9f (ULP diff: %u)\n",
                       i, j, aVal, bVal, ulpDiff);
                return false;
            }
        }
    }

    return true;
}

// Copies one matrix to another matrix (single-precision)
void copyMatrixDouble(arm_matrix_instance_f32* src, arm_matrix_instance_f64* dest) {

	uint32_t total = src->numRows * src->numCols;
	for (uint32_t i = 0; i < total; i++) {
	    dest->pData[i] = (float64_t) src->pData[i];
	}

}

// Copies one matrix to another matrix (double-precision)
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


