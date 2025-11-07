#include "Inc/propogate.h"

void compute_qdot(arm_matrix_instance_f32* q, arm_matrix_instance_f32* what, arm_matrix_instance_f32* qdot, float32_t qDotBuff[4]) {

	float32_t wQuatBuff[4] = {0, what->pData[0], what->pData[1], what->pData[2]};
	arm_quaternion_product_single_f32(q->pData, wQuatBuff, qDotBuff);
	arm_scale_f32(qDotBuff, 0.5f, qDotBuff, 3);
	arm_mat_init_f32(qdot, 4, 1, qDotBuff);

}

void compute_lla_dot(float32_t phi, float32_t h, float32_t vn, float32_t ve, float32_t vd, arm_matrix_instance_f32* llaDot, float32_t llaDotBuff[3]) {
	float32_t computeRadiiResult[4];
	compute_radii(phi, computeRadiiResult);

	float32_t R_phi = computeRadiiResult[0];
	float32_t R_lamb = computeRadiiResult[1];

	float32_t phidot = vn / (R_phi + h);
	float32_t lambdot = ve / ((R_lamb + h) * arm_cosd_f32(phi));
	float32_t hdot = -vd;

	llaDotBuff[0] = phidot;
	llaDotBuff[1] = lambdot;
	llaDotBuff[2] = hdot;

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

void compute_Pqdot(float32_t *x, float32_t *Pq, float32_t *Qq, float32_t *w_meas,
                   arm_matrix_instance_f32* Pqdot, float32_t PqdotBuff[6*6]) {

    // --- Extract quaternion and convert to matrix format ---
    arm_matrix_instance_f32 q_mat;
    float32_t q_data[4];
    for (int i = 0; i < 4; i++)
        q_data[i] = x[i];
    arm_mat_init_f32(&q_mat, 4, 1, q_data);

    // --- Compute DCM from quaternion ---
    arm_matrix_instance_f32 CB2I_mat;
    float32_t CB2I_data[9];
    quaternion2DCM(&q_mat, &CB2I_mat, CB2I_data);

    // --- Extract gyro bias ---
    float32_t bg_minus[3];
    for (int i = 0; i < 3; i++)
        bg_minus[i] = x[10 + i];  // MATLAB x(11:13)

    // --- Compute I_omega = CB2I * (w_meas - bg_minus) ---
    float32_t w_diff[3];
    for (int i = 0; i < 3; i++)
        w_diff[i] = w_meas[i] - bg_minus[i];

    arm_matrix_instance_f32 w_diff_mat, I_omega_mat;
    float32_t I_omega_data[3];
    arm_mat_init_f32(&w_diff_mat, 3, 1, w_diff);
    arm_mat_init_f32(&I_omega_mat, 3, 1, I_omega_data);
    arm_mat_mult_f32(&CB2I_mat, &w_diff_mat, &I_omega_mat);

    // --- Build Fq matrix (6x6) ---
    arm_matrix_instance_f32 Fq_mat;
    float32_t Fq_data[36];
    arm_mat_init_f32(&Fq_mat, 6, 6, Fq_data);
    memset(Fq_data, 0, sizeof(float32_t) * 36);

    // Top-right 3x3: -eye(3)
    for (int i = 0; i < 3; i++)
        Fq_data[i * 6 + i + 3] = -1.0f;

    // Bottom-right 3x3: skew(I_omega)
    arm_matrix_instance_f32 skew_mat;
    float32_t skew_data[9];
    arm_mat_skew_f32(&I_omega_mat, &skew_mat, skew_data);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            Fq_data[(i + 3) * 6 + j + 3] = skew_data[i * 3 + j];

    // --- Build Gq matrix (6x6) ---
    arm_matrix_instance_f32 Gq_mat;
    float32_t Gq_data[36];
    arm_mat_init_f32(&Gq_mat, 6, 6, Gq_data);
    memset(Gq_data, 0, sizeof(float32_t) * 36);

    // Top-left 3x3: CB2I
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            Gq_data[i * 6 + j] = CB2I_data[i * 3 + j];

    // Bottom-right 3x3: -CB2I
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            Gq_data[(i + 3) * 6 + j + 3] = -CB2I_data[i * 3 + j];

    // --- Initialize input matrices ---
    arm_matrix_instance_f32 Pq_mat, Qq_mat;
    arm_mat_init_f32(&Pq_mat, 6, 6, Pq);
    arm_mat_init_f32(&Qq_mat, 6, 6, Qq);

    // --- Temporary matrices for calculation ---
    arm_matrix_instance_f32 temp1, temp2, temp3, temp4, FqT, GqT;
    float32_t temp1_data[36], temp2_data[36], temp3_data[36], temp4_data[36];
    float32_t FqT_data[36], GqT_data[36];
    arm_mat_init_f32(&temp1, 6, 6, temp1_data);
    arm_mat_init_f32(&temp2, 6, 6, temp2_data);
    arm_mat_init_f32(&temp3, 6, 6, temp3_data);
    arm_mat_init_f32(&temp4, 6, 6, temp4_data);
    arm_mat_init_f32(&FqT, 6, 6, FqT_data);
    arm_mat_init_f32(&GqT, 6, 6, GqT_data);

    // --- Compute transpose matrices ---
    arm_mat_trans_f32(&Fq_mat, &FqT);
    arm_mat_trans_f32(&Gq_mat, &GqT);

    // --- Compute products ---
    arm_mat_mult_f32(&Fq_mat, &Pq_mat, &temp1);   // Fq*Pq
    arm_mat_mult_f32(&Pq_mat, &FqT, &temp2);      // Pq*Fq'
    arm_mat_mult_f32(&Gq_mat, &Qq_mat, &temp3);   // Gq*Qq
    arm_mat_mult_f32(&temp3, &GqT, &temp4);       // Gq*Qq*Gq'

    // --- Sum terms to get Pqdot ---
    arm_mat_init_f32(Pqdot, 6, 6, PqdotBuff);
    arm_mat_add_f32(&temp1, &temp2, Pqdot);
    arm_mat_add_f32(Pqdot, &temp4, Pqdot);
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

void integrate(arm_matrix_instance_f32* x, arm_matrix_instance_f32* P, arm_matrix_instance_f32* Pq,
			   arm_matrix_instance_f32* qdot, arm_matrix_instance_f32* pdot, arm_matrix_instance_f32* vdot,
			   arm_matrix_instance_f32* Pdot, arm_matrix_instance_f32* Pqdot, float32_t dt,
			   arm_matrix_instance_f32* xMinus, arm_matrix_instance_f32* Pminus, arm_matrix_instance_f32* Pqminus,
			   float32_t xMinusBuff[22], float32_t PMinusBuff[21*21], float32_t PqMinusBuff[6*6]) {


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

	// Compute Pqminus = Pq + dt * Pqdot safely ---
	arm_matrix_instance_f32 PqdotScaled;
	arm_mat_init_f32(&PqdotScaled, Pq->numRows, Pq->numCols, PqMinusBuff); // reuse buffer
	arm_mat_scale_f32(Pqdot, dt, &PqdotScaled);

	arm_mat_init_f32(Pqminus, Pq->numRows, Pq->numCols, PqMinusBuff);
	arm_mat_add_f32(Pq, &PqdotScaled, Pqminus);
}


void propogate(arm_matrix_instance_f32* xPlus, arm_matrix_instance_f32* Pplus, arm_matrix_instance_f32* PqPlus,
			   arm_matrix_instance_f32* what, arm_matrix_instance_f32* aHatN, arm_matrix_instance_f32* wMeas,
			   arm_matrix_instance_f32* aMeas, arm_matrix_instance_f32* Q, arm_matrix_instance_f32* Qq, float32_t dt,
			   float32_t we, arm_matrix_instance_f32* xMinus, arm_matrix_instance_f32* PMinus, arm_matrix_instance_f32* PqMinus,
			   float32_t xMinusBuff[22], float32_t PMinusBuff[22*22], float32_t PqMinusBuff[6*6]) {

	arm_matrix_instance_f32 q, gBias, aBias, g_sf, a_sf;
	float32_t quatBuff[4], gBiasBuff[3], aBiasBuff[3], gSFBias[3], aSFBias[3];

	getStateQuaternion(xPlus, &q, quatBuff);
	getStateGBias(xPlus, &gBias, gBiasBuff);
	getStateABias(xPlus, &aBias, aBiasBuff);
	getStateGSF(xPlus, &g_sf, gSFBias);
	getStateASF(xPlus, &a_sf, aSFBias);

	arm_matrix_instance_f32 qdot, llaDot, pdot, vdot, Pdot, Pqdot;
	float32_t qDotBuff[4], llaDotBuff[3], vDotBuff[3], PqDotBuff[6*6], PdotBuff[21*21];

	float32_t phi = xPlus->pData[4];
	float32_t h = xPlus->pData[6];
	float32_t vn = xPlus->pData[7];
	float32_t ve = xPlus->pData[8];
	float32_t vd = xPlus->pData[9];

	compute_qdot(&q, what, &qdot, qDotBuff);
	compute_lla_dot(phi, h, vn, ve, vd, &llaDot, llaDotBuff);
	compute_vdot(phi, h, vn, ve, vd, aHatN->pData, we, &vdot, vDotBuff);
	compute_Pqdot(xPlus->pData, PqPlus->pData, Qq->pData, wMeas->pData, &Pqdot, PqDotBuff);
	compute_Pdot(&q, &a_sf, &g_sf, &gBias, &aBias, aMeas, wMeas, Pplus, Q,
				 phi, h, vn, ve, vd, we, &Pdot, PdotBuff);

	integrate(xPlus, Pplus, PqPlus, &qdot, &pdot, &vdot, &Pdot, &Pqdot, dt,
			  xMinus, PMinus, PqMinus, xMinusBuff, PMinusBuff, PqMinusBuff);
}

