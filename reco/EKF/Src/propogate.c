#include "ekf.h"

void compute_qdot(arm_matrix_instance_f32* q, arm_matrix_instance_f32* what, arm_matrix_instance_f32* qdot, float32_t qDotBuff[4]) {

	float32_t wQuatBuff[4] = {0, what->pData[0], what->pData[1], what->pData[2]};
	arm_quaternion_product_single_f32(q->pData, wQuatBuff, qDotBuff);
	arm_scale_f32(qDotBuff, 0.5f, qDotBuff, 4);
	arm_mat_init_f32(qdot, 4, 1, qDotBuff);

}

void compute_lla_dot(float32_t phi, float32_t h, float32_t vn, float32_t ve, float32_t vd, arm_matrix_instance_f32* llaDot, float32_t llaDotBuff[3]) {
	float32_t computeRadiiResult[4];
	compute_radii(phi, computeRadiiResult);
	float32_t phiRad = deg2rad(phi);

	float32_t R_phi = computeRadiiResult[0];
	float32_t R_lamb = computeRadiiResult[1];

	float32_t phidot = vn / (R_phi + h);
	float32_t lambdot = ve / ((R_lamb + h) * arm_cos_f32(phiRad));

	llaDotBuff[0] = rad2deg(phidot);
	llaDotBuff[1] = rad2deg(lambdot);
	llaDotBuff[2] = -vd;

	arm_mat_init_f32(llaDot, 3, 1, llaDotBuff);
}

void compute_vdot(float32_t phi, float32_t h, float32_t vn, float32_t ve, float32_t vd, float32_t ahat_n[3], float32_t we, arm_matrix_instance_f32* vdot, float32_t vdotBuff[3]) {
	float32_t an = ahat_n[0];
	float32_t ae = ahat_n[1];
	float32_t ad = ahat_n[2];

	float32_t computeRadiiResult[4];
	compute_radii(phi, computeRadiiResult);

	float32_t R_phi = computeRadiiResult[0];
	float32_t R_lamb = computeRadiiResult[1];

	// Compute gravity - Eqn 7.69c
	float32_t sin_phi = arm_sind_f32(phi);
	float32_t cos_phi = arm_cosd_f32(phi);
	float32_t sin_phi_sq = sin_phi * sin_phi;
	float32_t sin_2phi = arm_sind_f32(2.0f * phi);
	float32_t sin_2phi_sq = sin_2phi * sin_2phi;
	float32_t ghat = 9.780327f * (1.0f + 5.3024e-3f * sin_phi_sq - 5.8e-6f * sin_2phi_sq) - (3.0877e-6f - 4.4e-9f * sin_phi_sq) * h + 7.2e-14f * h * h;

	float32_t R_phi_h = R_phi + h;
	float32_t R_lamb_h = R_lamb + h;

	float32_t vndot = -(ve / (R_lamb_h * cos_phi) + 2.0f * we) * ve * sin_phi + (vn * vd) / R_phi_h + an;

	float32_t vedot = (ve / (R_lamb_h * cos_phi) + 2.0f * we) * vn * sin_phi + (ve * vd) / R_lamb_h + 2.0f * we * vd * cos_phi + ae;

	float32_t vddot = -ve * ve / R_lamb_h - vn * vn / R_phi_h - 2.0f * we * ve * cos_phi + ghat + ad;

	vdotBuff[0] = vndot;
	vdotBuff[1] = vedot;
	vdotBuff[2] = vddot;

	arm_mat_init_f32(vdot, 3, 1, vdotBuff);
}

void compute_Pdot(arm_matrix_instance_f32* q, arm_matrix_instance_f32* sf_a, arm_matrix_instance_f32* sf_g,
		  	  	  arm_matrix_instance_f32* bias_g, arm_matrix_instance_f32* bias_a, arm_matrix_instance_f32* a_meas,
				  arm_matrix_instance_f32* w_meas, arm_matrix_instance_f32* P, arm_matrix_instance_f32* Q,
				  float32_t phi, float32_t h, float32_t vn, float32_t ve, float32_t vd, float32_t we,
				  arm_matrix_instance_f32* Pdot, float32_t PdotBuff[21*21]) {
	// F (21 x 21) / G (21 x 12) / P (21 x 21) / Q (12 x 12) / FP (21 x 21) / PF' (21 x 21)

	arm_matrix_instance_f32 F, G;
	float32_t FBuff[21*21], GBuff[21*12]; // G is 21x12

	compute_F(q, sf_a, sf_g, bias_g, bias_a, phi, h, vn, ve, vd, a_meas, w_meas, we, &F, FBuff);
	compute_G(sf_g, sf_a, q, &G, GBuff);

	arm_matrix_instance_f32 FTrans, GTrans;
	float32_t FTransBuff[21*21], GTransBuff[12*21]; // G' is 12x21

	arm_mat_init_f32(&FTrans, 21, 21, FTransBuff);
	arm_mat_init_f32(&GTrans, 12, 21, GTransBuff);

	arm_mat_trans_f32(&F, &FTrans);    // FTrans = F'
	arm_mat_trans_f32(&G, &GTrans);    // GTrans = G'

	arm_matrix_instance_f32 FP, PF, GQ, term3;
	float32_t FPBuff[21*21], PFBuff[21*21], GQBuff[21*12], term3Buff[21*21];

	arm_mat_init_f32(&FP, 21, 21, FPBuff);
	arm_mat_init_f32(&PF, 21, 21, PFBuff);
	arm_mat_init_f32(&GQ, 21, 12, GQBuff);
	arm_mat_init_f32(&term3, 21, 21, term3Buff);
	arm_mat_init_f32(Pdot, 21, 21, PdotBuff);

	arm_mat_mult_f32(&F, P, &FP);          // FP = F * P
	arm_mat_mult_f32(P, &FTrans, &PF);     // PF = P * F'
	arm_mat_mult_f32(&G, Q, &GQ);          // GQ = G * Q
	arm_mat_mult_f32(&GQ, &GTrans, &term3);// term3 = G * Q * G'

	arm_mat_add_f32(&FP, &PF, &FP);        // FP = F*P + P*F'
	arm_mat_add_f32(&FP, &term3, Pdot);    // Pdot = F*P + P*F' + G*Q*G'
}

void integrate(arm_matrix_instance_f32* x, arm_matrix_instance_f32* P, arm_matrix_instance_f32* qdot,
			   arm_matrix_instance_f32* pdot, arm_matrix_instance_f32* vdot, arm_matrix_instance_f32* Pdot,
			   float32_t dt, arm_matrix_instance_f32* xMinus, arm_matrix_instance_f32* Pminus,
			   float32_t xMinusBuff[22], float32_t PMinusBuff[21*21]) {


	// Assemble xDot = [qdot; pdot; vdot; zeros(12,1)] ---
	arm_matrix_instance_f32 xDot;
	float32_t xDotBuff[22] = {0}; // initialize to zero
	arm_mat_init_f32(&xDot, 22, 1, xDotBuff);

	// Copy qdot (4x1)
	for (uint8_t i = 0; i < 4; i++) {
		xDotBuff[i] = qdot->pData[i];
	}
	// Copy pdot (3x1)
	for (uint8_t i = 0; i < 3; i++) {
		xDotBuff[4 + i] = pdot->pData[i];
	}
	// Copy vdot (3x1)
	for (uint8_t i = 0; i < 3; i++) {
		xDotBuff[7 + i] = vdot->pData[i];
	}

	// Compute xMinus = x + dt * xDot ---
	arm_matrix_instance_f32 xDotScaled;
	float32_t xDotScaledBuff[22];
	arm_mat_init_f32(&xDotScaled, 22, 1, xDotScaledBuff);
	arm_mat_scale_f32(&xDot, dt, &xDotScaled);

	arm_mat_init_f32(xMinus, 22, 1, xMinusBuff);
	arm_mat_add_f32(x, &xDotScaled, xMinus);

	// Normalize quaternion (first 4 elements) ---
	arm_quaternion_normalize_f32(xMinus->pData, xMinus->pData, 1);  // in-place

	// Compute Pminus = P + dt * Pdot safely ---
	arm_matrix_instance_f32 PdotScaled;
	arm_mat_init_f32(&PdotScaled, P->numRows, P->numCols, PMinusBuff); // reuse buffer
	arm_mat_scale_f32(Pdot, dt, &PdotScaled);

	arm_mat_init_f32(Pminus, P->numRows, P->numCols, PMinusBuff);
	arm_mat_add_f32(P, &PdotScaled, Pminus);
}


void propogate(arm_matrix_instance_f32* xMinus, arm_matrix_instance_f32* PMinus, arm_matrix_instance_f32* what,
			   arm_matrix_instance_f32* aHatN, arm_matrix_instance_f32* wMeas, arm_matrix_instance_f32* aMeas,
			   arm_matrix_instance_f32* Q, float32_t dt, float32_t we,
			   arm_matrix_instance_f32* xPlus, arm_matrix_instance_f32* PPlus, float32_t xPlusBuff[22],
			   float32_t PPlusBuff[21*21]) {

	arm_matrix_instance_f32 q, gBias, aBias, g_sf, a_sf;
	float32_t quatBuff[4], gBiasBuff[3], aBiasBuff[3], gSFBias[3], aSFBias[3];

	getStateQuaternion(xMinus, &q, quatBuff);
	getStateGBias(xMinus, &gBias, gBiasBuff);
	getStateABias(xMinus, &aBias, aBiasBuff);
	getStateGSF(xMinus, &g_sf, gSFBias);
	getStateASF(xMinus, &a_sf, aSFBias);

	arm_matrix_instance_f32 qdot, pdot, vdot, Pdot;
	float32_t qDotBuff[4], pDotBuff[3], vDotBuff[3], PdotBuff[21*21];

	float32_t phi = xMinus->pData[4];
	float32_t h = xMinus->pData[6];
	float32_t vn = xMinus->pData[7];
	float32_t ve = xMinus->pData[8];
	float32_t vd = xMinus->pData[9];

	compute_qdot(&q, what, &qdot, qDotBuff);
	compute_lla_dot(phi, h, vn, ve, vd, &pdot, pDotBuff);
	compute_vdot(phi, h, vn, ve, vd, aHatN->pData, we, &vdot, vDotBuff);
	compute_Pdot(&q, &a_sf, &g_sf, &gBias, &aBias, aMeas, wMeas, PMinus, Q,
				 phi, h, vn, ve, vd, we, &Pdot, PdotBuff);

	integrate(xMinus, PMinus, &qdot, &pdot, &vdot, &Pdot, dt,
			  xPlus, PPlus, xPlusBuff, PPlusBuff);
}

