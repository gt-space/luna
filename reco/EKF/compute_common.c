#include "compute_common.h"

// attitude, pos, vel, g bias, a bias, g sf, a sf
// x0 = [q0; lla0; zeros(3,1); zeros(3,1); zeros(3,1); zeros(3,1); zeros(3,1)];
inline void getStateQuaternion(arm_matrix_instance_f32* x, arm_matrix_instance_f32* quaternion, float32_t* quaternionData) {
	quaternionData[0] = x->pData[0];
	quaternionData[1] = x->pData[1];
	quaternionData[2] = x->pData[2];
	quaternionData[3] = x->pData[3];

	arm_mat_init_f32(quaternion, 4, 1, quaternionData);
}

inline void getStatePosition(arm_matrix_instance_f32* x, arm_matrix_instance_f32* position, float32_t* posData) {
	posData[0] = x->pData[4];
	posData[1] = x->pData[5];
	posData[2] = x->pData[6];

	arm_mat_init_f32(position, 3, 1, posData);
}

inline void getStateVelocity(arm_matrix_instance_f32* x, arm_matrix_instance_f32* vel, float32_t* velData) {
	velData[0] = x->pData[7];
	velData[1] = x->pData[8];
	velData[2] = x->pData[9];

	arm_mat_init_f32(vel, 3, 1, velData);
}

inline void getStateGBias(arm_matrix_instance_f32* x, arm_matrix_instance_f32* gBias, float32_t* gData) {
	gData[0] = x->pData[10];
	gData[1] = x->pData[11];
	gData[2] = x->pData[12];

	arm_mat_init_f32(gBias, 3, 1, gData);
}

inline void getStateABias(arm_matrix_instance_f32* x, arm_matrix_instance_f32* aBias, float32_t* aData) {
	aData[0] = x->pData[13];
	aData[1] = x->pData[14];
	aData[2] = x->pData[15];

	arm_mat_init_f32(aBias, 3, 1, aData);
}

inline void getStateGSF(arm_matrix_instance_f32* x, arm_matrix_instance_f32* g_sf, float32_t* g_sf_data) {
	g_sf_data[0] = x->pData[16];
	g_sf_data[1] = x->pData[17];
	g_sf_data[2] = x->pData[18];

	arm_mat_init_f32(g_sf, 3, 1, g_sf_data);
}

inline void getStateASF(arm_matrix_instance_f32* x, arm_matrix_instance_f32* a_sf, float32_t* a_sf_data) {
	a_sf_data[0] = x->pData[19];
	a_sf_data[1] = x->pData[20];
	a_sf_data[2] = x->pData[21];

	arm_mat_init_f32(a_sf, 3, 1, a_sf_data);
}

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

void compute_g_dg(arm_matrix_instance_f32* x, float32_t* result) {

}

