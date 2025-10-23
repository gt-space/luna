#include "compute_common.h"

// Doesn't implement the check to make sure that is 1x4 vector (lines 6 through 9 in the respective .m file)
void quaternion2DCM(const arm_matrix_instance_f32* quaternion, arm_matrix_instance_f32* CB2I, float32_t* CB2IBuffer) {
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
    arm_mat_skew_f32(&skewMat, vectorVal, skewData);
    arm_mat_scale_f32(&skewMat, 2.0f * scalarVal, &skewMat);

    // Sum: CB2I = (s^2 - v·v)I + 2vv' + 2s*skew(v)
    float32_t term1Data[9];
    arm_mat_init_f32(&term1, 3, 3, term1Data);
    arm_mat_add_f32(&eyeDot, &outerProduct, &term1);

    arm_mat_init_f32(CB2I, 3, 3, CB2IBuffer);
    arm_mat_add_f32(&term1, &skewMat, CB2I);
    return;
}

void compute_radii(float32_t phi, float32_t* returnVector) {
	float32_t a = 6378137; // semi-major axis
	float32_t b = 6356752.31425; // semi-minor axis
	float32_t ecc = 1 - ((a / b) * (a / b)); // eccentricity

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
	returnVector[3] = dR_phi_dphi;
}

void compute_g_dg() {

}
