#include "compute.h"

#define COPY_BLOCK(Fdst, Fsrc, rows, cols, rowOffset, colOffset, Fcols) \
	for (uint32_t i = 0; i < (rows); i++)                               \
		for (uint32_t j = 0; j < (cols); j++)                           \
			(Fdst)[(rowOffset + i) * (Fcols) + (colOffset + j)] = (Fsrc)[i * (cols) + j];

void compute_wn(float32_t phi, float32_t h, float32_t vn, float32_t ve, arm_matrix_instance_f32 *wn, float32_t we, float32_t *buffer)
{

	float32_t computeRadiiVec[4];
	compute_radii(phi, computeRadiiVec);

	float32_t R_phi = computeRadiiVec[0];
	float32_t R_lamb = computeRadiiVec[1];

	arm_matrix_instance_f32 vec1;
	float32_t term1[3] = {arm_sind_f32(phi), 0, -arm_sind_f32(phi)};
	arm_mat_init_f32(&vec1, 3, 1, term1);

	arm_mat_scale_f32(&vec1, we, &vec1);

	arm_matrix_instance_f32 vec2;
	float32_t term2[3] = {ve / (R_lamb + h), -vn / (R_phi + h), -(ve * arm_tand_f32(phi)) / (R_lamb + h)};
	arm_mat_init_f32(&vec2, 3, 1, term2);

	float32_t finalTerm[3];
	arm_mat_init_f32(wn, 3, 1, finalTerm);
	arm_mat_add_f32(&vec1, &vec2, wn);
}

void compute_what(arm_matrix_instance_f32 *q, arm_matrix_instance_f32 *bias_g, arm_matrix_instance_f32 *sf_g,
				  float32_t phi, float32_t h, float32_t vn, float32_t ve, arm_matrix_instance_f32 *w_meas,
				  arm_matrix_instance_f32 *what, float32_t *whatBuffer, float32_t we)
{

	arm_matrix_instance_f32 D_bn, wn, product;
	float32_t D_bn_buff[9], wn_buff[3], productBuff[3];

	quaternion2DCM(&q, &D_bn, D_bn_buff);
	compute_wn(phi, h, vn, ve, &wn, we, wn_buff);

	arm_offset_f32(sf_g->pData, 1.0f, sf_g->pData, 3);

	for (uint32_t i = 0; i < sf_g->numRows * sf_g->numCols; i++)
	{
		sf_g->pData[i] = 1.0f / sf_g->pData[i];
	}

	arm_sub_f32(w_meas->pData, bias_g->pData, w_meas->pData, 3);
	arm_mult_f32(w_meas->pData, sf_g->pData, w_meas->pData, 3);

	arm_mat_init_f32(&product, 3, 1, productBuff);
	arm_mat_mult_f32(&D_bn, &wn, &product);

	arm_mat_init_f32(what, 3, 1, whatBuffer);
	arm_sub_f32(sf_g->pData, productBuff, what->pData, 3);
}

void compute_ahat(arm_matrix_instance_f32 *q, arm_matrix_instance_f32 *sf_a, arm_matrix_instance_f32 *bias_a, arm_matrix_instance_f32 *a_meas, arm_matrix_instance_f32 *ahat_n, float32_t *ahatBuff)
{
	arm_matrix_instance_f32 D_bn;
	float32_t D_bn_buff[9];

	quaternion2DCM(q, &D_bn, D_bn_buff);

	arm_offset_f32(sf_a->pData, 1.0f, sf_a->pData, 3);

	for (uint8_t i = 0; i < sf_a->numRows; i++)
	{
		sf_a->pData[i] = 1.0f / sf_a->pData[i];
	}

	arm_sub_f32(a_meas->pData, bias_a->pData, a_meas->pData, 3);
	arm_mult_f32(a_meas->pData, sf_a->pData, a_meas->pData, 3);

	arm_mat_init_f32(ahat_n, 3, 1, ahatBuff);
	arm_mat_mult_f32(&D_bn, a_meas, ahat_n);
}

void compute_dpdot_dp(float32_t phi, float32_t h, float32_t vn, float32_t ve, arm_matrix_instance_f32 *dpdot_dp, float32_t *dpDotBuff)
{

	float32_t computeRadiiResult[4];
	compute_radii(phi, computeRadiiResult);

	float32_t R_phi = computeRadiiResult[0], R_lamb = computeRadiiResult[1];
	float32_t dR_phi_dphi = computeRadiiResult[2], dR_lamb_dphi = computeRadiiResult[3];

	float32_t square_phi = (R_phi + h) * (R_phi + h);
	float32_t square_lamb = (R_lamb + h) * (R_lamb + h);

	float32_t m11 = -vn / square_phi * dR_phi_dphi;
	float32_t m13 = -vn / square_phi;
	float32_t m21 = -(ve * arm_secd_f32(phi)) / square_lamb * dR_lamb_dphi + (ve * arm_secd_f32(phi) * arm_tand_f32(phi)) / (R_lamb + h);
	float32_t m23 = -ve * arm_secd_f32(phi) / square_lamb;

	dpDotBuff[0] = m11;
	dpDotBuff[1] = 0;
	dpDotBuff[2] = m13;
	dpDotBuff[3] = m21;
	dpDotBuff[4] = 0;
	dpDotBuff[5] = m23;
	dpDotBuff[6] = 0;
	dpDotBuff[7] = 0;
	dpDotBuff[8] = 0;

	arm_mat_init_f32(dpdot_dp, 3, 3, dpDotBuff);
}

void compute_dpdot_dv(float32_t phi, float32_t h, arm_matrix_instance_f32 *dpdot_dv, float32_t *dpDotBuff)
{

	float32_t computeRadiiResult[4];
	compute_radii(phi, computeRadiiResult);

	float32_t R_phi = computeRadiiResult[0];
	float32_t R_lamb = computeRadiiResult[1];

	float32_t m11 = 1.0f / (R_phi + h);
	float32_t m22 = arm_secd_f32(phi) / (R_lamb + h);

	dpDotBuff[0] = m11;
	dpDotBuff[1] = 0;
	dpDotBuff[2] = 0;
	dpDotBuff[3] = 0;
	dpDotBuff[4] = m22;
	dpDotBuff[5] = 0;
	dpDotBuff[6] = 0;
	dpDotBuff[7] = 0;
	dpDotBuff[8] = -1;

	arm_mat_init_f32(dpdot_dv, 3, 3, dpDotBuff);
}

void compute_dvdot_dp(float32_t phi, float32_t h, float32_t vn, float32_t ve, float32_t vd, arm_matrix_instance_f32 *dvdot_dp, float32_t *dvdotBuff, float32_t we)
{
	// Compute radii and derivatives
	float32_t computeRadiiResult[4];
	compute_radii(phi, computeRadiiResult);

	float32_t R_phi = computeRadiiResult[0], R_lamb = computeRadiiResult[1];
	float32_t dR_phi_dphi = computeRadiiResult[2], dR_lamb_dphi = computeRadiiResult[3];

	// Compute gravity derivatives
	float32_t dg_dphi, dg_dh;
	compute_g_dg(phi, h, &dg_dphi, &dg_dh); // adjust depending on your C function signature

	// Precompute frequently used terms
	float32_t secphi = arm_secd_f32(phi);
	float32_t secphi2 = secphi * secphi;
	float32_t tanphi = arm_tand_f32(phi);
	float32_t sinphi = arm_sind_f32(phi);
	float32_t cosphi = arm_cosd_f32(phi);

	float32_t Rphi_h = R_phi + h;
	float32_t Rlamb_h = R_lamb + h;
	float32_t Rphi_h2 = Rphi_h * Rphi_h;
	float32_t Rlamb_h2 = Rlamb_h * Rlamb_h;

	// Compute matrix elements
	float32_t Y11 = -(ve * ve * secphi2) / (Rlamb_h) + (ve * ve * tanphi) / (Rlamb_h2)*dR_lamb_dphi - 2.0f * we * ve * cosphi - (vn * vd) / (Rphi_h2)*dR_phi_dphi;

	float32_t Y13 = (ve * ve * tanphi) / (Rlamb_h2) - (vn * vd) / (Rphi_h2);

	float32_t Y21 = (ve * vn * secphi2) / (Rlamb_h) - (ve * vn * tanphi) / (Rlamb_h2)*dR_lamb_dphi + 2.0f * we * vn * cosphi - (ve * vd) / (Rlamb_h2)*dR_lamb_dphi - 2.0f * we * vd * sinphi;

	float32_t Y23 = -ve * ((vn * tanphi + vd) / Rlamb_h2);

	float32_t Y31 = (ve * ve) / (Rlamb_h2)*dR_lamb_dphi + (vn * vn) / (Rphi_h2)*dR_phi_dphi + 2.0f * we * ve * sinphi + dg_dphi;

	float32_t Y33 = (ve * ve) / (Rlamb_h2) + (vn * vn) / (Rphi_h2) + dg_dh;

	// Fill CMSIS-DSP buffer (row-major order)
	dvdotBuff[0] = Y11;
	dvdotBuff[1] = 0.0f;
	dvdotBuff[2] = Y13;
	dvdotBuff[3] = Y21;
	dvdotBuff[4] = 0.0f;
	dvdotBuff[5] = Y23;
	dvdotBuff[6] = Y31;
	dvdotBuff[7] = 0.0f;
	dvdotBuff[8] = Y33;

	// Initialize CMSIS-DSP matrix
	arm_mat_init_f32(dvdot_dp, 3, 3, dvdotBuff);
}

void compute_dvdot_dv(float32_t phi, float32_t h, float32_t vn, float32_t ve, float32_t vd, arm_matrix_instance_f32 *dvdot_dv, float32_t *dvdotBuff, float32_t we)
{
	// Compute radii
	float32_t computeRadiiResult[4];
	compute_radii(phi, computeRadiiResult);

	float32_t R_phi = computeRadiiResult[0], R_lamb = computeRadiiResult[1];
	float32_t dR_phi_dphi = computeRadiiResult[2], dR_lamb_dphi = computeRadiiResult[3];

	// Precompute frequently used terms
	float32_t tanphi = arm_tand_f32(phi);
	float32_t sinphi = arm_sind_f32(phi);
	float32_t cosphi = arm_cosd_f32(phi);

	float32_t Rphi_h = R_phi + h;
	float32_t Rlamb_h = R_lamb + h;

	// Compute matrix elements
	float32_t Z11 = vd / Rphi_h;
	float32_t Z12 = (-2.0f * ve * tanphi) / Rlamb_h + 2.0f * we * sinphi;
	float32_t Z13 = vn / Rphi_h;

	float32_t Z21 = (ve * tanphi) / Rlamb_h + 2.0f * we * sinphi;
	float32_t Z22 = (vd + vn * tanphi) / Rlamb_h;
	float32_t Z23 = ve / Rlamb_h + 2.0f * we * cosphi;

	float32_t Z31 = (-2.0f * vn) / Rphi_h;
	float32_t Z32 = (-2.0f * ve) / Rlamb_h - 2.0f * we * cosphi;

	// Fill CMSIS-DSP buffer (row-major order)
	dvdotBuff[0] = Z11;
	dvdotBuff[1] = Z12;
	dvdotBuff[2] = Z13;
	dvdotBuff[3] = Z21;
	dvdotBuff[4] = Z22;
	dvdotBuff[5] = Z23;
	dvdotBuff[6] = Z31;
	dvdotBuff[7] = Z32;
	dvdotBuff[8] = 0.0f;

	// Initialize CMSIS-DSP matrix
	arm_mat_init_f32(dvdot_dv, 3, 3, dvdotBuff);
}

void compute_dwdp(float32_t phi, float32_t h, float32_t ve, float32_t vn, arm_matrix_instance_f32 *dwdp, float32_t *dwdpBuffer, float32_t we)
{
	float32_t computeRadiiResult[4];
	compute_radii(phi, computeRadiiResult);

	float32_t R_phi = computeRadiiResult[0], R_lamb = computeRadiiResult[1];
	float32_t dR_phi_dphi = computeRadiiResult[2], dR_lamb_dphi = computeRadiiResult[3];

	float32_t sin_phi = arm_sind_f32(phi);
	float32_t cos_phi = arm_cosd_f32(phi);
	float32_t tan_phi = arm_tand_f32(phi);
	float32_t sec_phi = arm_secd_f32(phi);
	float32_t sec_phi2 = sec_phi * sec_phi;

	float32_t RLh = R_lamb + h;
	float32_t RPh = R_phi + h;

	float32_t m11 = -we * sin_phi - ve / (RLh * RLh) * dR_lamb_dphi;
	float32_t m13 = -ve / (RLh * RLh);
	float32_t m21 = vn / (RPh * RPh) * dR_phi_dphi;
	float32_t m23 = vn / (RPh * RPh);
	float32_t m31 = -we * cos_phi - (ve * sec_phi2) / RLh + (ve * tan_phi / (RLh * RLh)) * dR_lamb_dphi;
	float32_t m33 = (ve * tan_phi) / (RLh * RLh);

	dwdpBuffer[0] = m11;
	dwdpBuffer[1] = 0.0f;
	dwdpBuffer[2] = m13;
	dwdpBuffer[3] = m21;
	dwdpBuffer[4] = 0.0f;
	dwdpBuffer[5] = m23;
	dwdpBuffer[6] = m31;
	dwdpBuffer[7] = 0.0f;
	dwdpBuffer[8] = m33;

	arm_mat_init_f32(dwdp, 3, 3, dwdpBuffer);
}

void compute_dwdv(float32_t phi, float32_t h, arm_matrix_instance_f32 *dwdv, float32_t *dwdvBuffer)
{
	float32_t computeRadiiResult[4];
	compute_radii(phi, computeRadiiResult);

	float32_t R_lamb = computeRadiiResult[0];
	float32_t R_phi = computeRadiiResult[1];

	float32_t m12 = 1 / (R_lamb + h);
	float32_t m21 = -1 / (R_phi + h);
	float32_t m32 = -arm_tand_f32(phi) / (R_lamb + h);

	dwdvBuffer[0] = 0.0f;
	dwdvBuffer[1] = m12;
	dwdvBuffer[2] = 0.0f;
	dwdvBuffer[3] = m21;
	dwdvBuffer[4] = 0.0f;
	dwdvBuffer[5] = 0.0f;
	dwdvBuffer[6] = 0.0f;
	dwdvBuffer[7] = m32f;
	dwdvBuffer[8] = 0.0f;

	arm_mat_init_f32(dwdv, 3, 3, dwdvBuffer);
}

void assemble_F(float32_t F[24 * 24],
				float32_t F11[9], float32_t F12[9], float32_t F13[9], float32_t F14[9], float32_t F16[9],
				float32_t F22[9], float32_t F23[36], // 3x12
				float32_t F31[9], float32_t F32[9], float32_t F33[9],
				float32_t F35[9], float32_t F37[9]) // 3x3
{
	memset(F, 0, 24 * 24 * sizeof(float32_t));

	COPY_BLOCK(F, F11, 3, 3, 0, 0, 24);
	COPY_BLOCK(F, F12, 3, 3, 0, 3, 24);
	COPY_BLOCK(F, F13, 3, 3, 0, 6, 24);
	COPY_BLOCK(F, F14, 3, 3, 0, 9, 24);
	COPY_BLOCK(F, F16, 3, 3, 0, 15, 24);

	COPY_BLOCK(F, F22, 3, 3, 3, 3, 24);
	COPY_BLOCK(F, F23, 3, 12, 3, 6, 24);

	COPY_BLOCK(F, F31, 3, 3, 6, 0, 24);
	COPY_BLOCK(F, F32, 3, 3, 6, 3, 24);
	COPY_BLOCK(F, F33, 3, 3, 6, 6, 24);
	COPY_BLOCK(F, F35, 3, 3, 6, 12, 24);
	COPY_BLOCK(F, F37, 3, 3, 6, 18, 24);
}

void compute_vdot(float32_t phi, float32_t h, float32_t vn, float32_t ve, float32_t vd, float32_t ahat_n[3], float32_t we, float32_t vdot[3])
{
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

	float32_t vndot = -(ve / (R_lamb_h * arm_cosd_f32(phi)) + 2.0f * we) * ve * arm_sind_f32(phi) + (vn * vd) / R_phi_h + an;

	float32_t vedot = (ve / (R_lamb_h * arm_cosd_f32(phi)) + 2.0f * we) * vn * arm_sind_f32(phi) + (ve * vd) / R_lamb_h + 2.0f * we * vd * arm_cosd_f32(phi) + ae;

	float32_t vddot = -ve * ve / R_lamb_h - vn * vn / R_phi_h - 2.0f * we * ve * arm_cosd_f32(phi) + ghat + ad;

	vdot[0] = vndot;
	vdot[1] = vedot;
	vdot[2] = vddot;
}

void compute_Pqdot(float32_t *Pqdot, float32_t *x, float32_t *Pq, float32_t *Qq, float32_t *w_meas)
{
	// Extract quaternion and convert to matrix format
	arm_matrix_instance_f32 q_mat;
	float32_t q_data[4];
	for (int i = 0; i < 4; i++)
	{
		q_data[i] = x[i];
	}
	arm_mat_init_f32(&q_mat, 4, 1, q_data);

	// Compute DCM from quaternion
	arm_matrix_instance_f32 CB2I_mat;
	float32_t CB2I_data[9];
	quaternion2DCM(&q_mat, &CB2I_mat, CB2I_data);

	// Extract gyro bias
	float32_t bg_minus[3];
	for (int i = 0; i < 3; i++)
	{
		bg_minus[i] = x[10 + i];
	}

	// Compute w_meas - bg_minus
	float32_t w_diff[3];
	for (int i = 0; i < 3; i++)
	{
		w_diff[i] = w_meas[i] - bg_minus[i];
	}

	// Compute I_omega = CB2I * (w_meas - bg_minus)
	arm_matrix_instance_f32 w_diff_mat, I_omega_mat;
	float32_t I_omega_data[3];
	arm_mat_init_f32(&w_diff_mat, 3, 1, w_diff);
	arm_mat_init_f32(&I_omega_mat, 3, 1, I_omega_data);
	arm_mat_mult_f32(&CB2I_mat, &w_diff_mat, &I_omega_mat);

	// Build Fq matrix (6x6)
	// Fq = [0 -I; 0 skew(I_omega)]
	arm_matrix_instance_f32 Fq_mat;
	float32_t Fq_data[36];
	arm_mat_init_f32(&Fq_mat, 6, 6, Fq_data);

	// Initialize Fq to zeros
	memset(Fq_data, 0, sizeof(float32_t) * 36);

	// Top-right 3x3: -eye(3)
	for (int i = 0; i < 3; i++)
	{
		Fq_data[i * 6 + i + 3] = -1.0f;
	}

	// Bottom-right 3x3: skew(I_omega)
	arm_matrix_instance_f32 skew_mat;
	float32_t skew_data[9];
	arm_mat_skew_f32(&I_omega_mat, &skew_mat, skew_data);

	// Copy skew matrix to bottom-right of Fq
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			Fq_data[(i + 3) * 6 + j + 3] = skew_data[i * 3 + j];
		}
	}

	// Build Gq matrix (6x6)
	// Gq = [CB2I 0; 0 -CB2I]
	arm_matrix_instance_f32 Gq_mat;
	float32_t Gq_data[36];
	arm_mat_init_f32(&Gq_mat, 6, 6, Gq_data);

	// Initialize Gq to zeros
	memset(Gq_data, 0, sizeof(float32_t) * 36);

	// Top-left 3x3: CB2I
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			Gq_data[i * 6 + j] = CB2I_data[i * 3 + j];
		}
	}

	// Bottom-right 3x3: -CB2I
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			Gq_data[(i + 3) * 6 + j + 3] = -CB2I_data[i * 3 + j];
		}
	}

	// Compute Pqdot = Fq*Pq + Pq*Fq' + Gq*Qq*Gq'
	// Initialize input matrices
	arm_matrix_instance_f32 Pq_mat, Qq_mat;
	arm_mat_init_f32(&Pq_mat, 6, 6, Pq);
	arm_mat_init_f32(&Qq_mat, 6, 6, Qq);

	// Temporary matrices
	arm_matrix_instance_f32 temp1_mat, temp2_mat, temp3_mat, temp4_mat;
	arm_matrix_instance_f32 FqT_mat, GqT_mat;
	float32_t temp1_data[36], temp2_data[36], temp3_data[36], temp4_data[36];
	float32_t FqT_data[36], GqT_data[36];

	arm_mat_init_f32(&temp1_mat, 6, 6, temp1_data);
	arm_mat_init_f32(&temp2_mat, 6, 6, temp2_data);
	arm_mat_init_f32(&temp3_mat, 6, 6, temp3_data);
	arm_mat_init_f32(&temp4_mat, 6, 6, temp4_data);
	arm_mat_init_f32(&FqT_mat, 6, 6, FqT_data);
	arm_mat_init_f32(&GqT_mat, 6, 6, GqT_data);

	// Transpose Fq
	arm_mat_trans_f32(&Fq_mat, &FqT_mat);

	// Compute Fq*Pq
	arm_mat_mult_f32(&Fq_mat, &Pq_mat, &temp1_mat);

	// Compute Pq*Fq'
	arm_mat_mult_f32(&Pq_mat, &FqT_mat, &temp2_mat);

	// Compute Gq*Qq
	arm_mat_mult_f32(&Gq_mat, &Qq_mat, &temp3_mat);

	// Transpose Gq
	arm_mat_trans_f32(&Gq_mat, &GqT_mat);

	// Compute Gq*Qq*Gq'
	arm_mat_mult_f32(&temp3_mat, &GqT_mat, &temp4_mat);

	// Sum all terms: Pqdot = Fq*Pq + Pq*Fq' + Gq*Qq*Gq'
	arm_matrix_instance_f32 Pqdot_mat;
	arm_mat_init_f32(&Pqdot_mat, 6, 6, Pqdot);

	// Pqdot = temp1 + temp2
	arm_mat_add_f32(&temp1_mat, &temp2_mat, &Pqdot_mat);

	// Pqdot = (temp1 + temp2) + temp4
	arm_mat_add_f32(&Pqdot_mat, &temp4_mat, &Pqdot_mat);
}

void compute_lla_dot(float32_t phi, float32_t h, float32_t vn, float32_t ve, float32_t vd, float32_t lla_dot[3])
{
	float32_t computeRadiiResult[4];
	compute_radii(phi, computeRadiiResult);

	float32_t R_phi = computeRadiiResult[0];
	float32_t R_lamb = computeRadiiResult[1];

	float32_t phidot = vn / (R_phi + h);
	float32_t lambdot = ve / ((R_lamb + h) * arm_cosd_f32(phi));
	float32_t hdot = -vd;

	lla_dot[0] = phidot;
	lla_dot[1] = lambdot;
	lla_dot[2] = hdot;
}

void compute_g_dg(float32_t phi, float32_t h, float32_t *dg_dphi, float32_t *dg_dh)
{
	float32_t sin_phi = arm_sind_f32(phi);
	float32_t cos_phi = arm_cosd_f32(phi);
	float32_t sin_phi_sq = sin_phi * sin_phi;
	float32_t sin_2phi = arm_sind_f32(2.0f * phi);
	float32_t sin_2phi_sq = sin_2phi * sin_2phi;

	// Compute dg_dphi - Eqn 7.84a
	float32_t term1 = 1.06048e-2f * sin_phi * cos_phi;
	float32_t term2 = 4.64e-5f * (sin_phi * cos_phi * cos_phi * cos_phi - sin_phi * sin_phi * sin_phi * cos_phi);
	float32_t term3 = 8.8e-9f * h * sin_phi * cos_phi;

	*dg_dphi = 9.780327f * (term1 - term2) + term3;

	// Compute dg_dh - Eqn 7.84b
	*dg_dh = -3.0877e-6f + 4.4e-9f * sin_phi_sq + 1.44e-13f * h;
}

void computeF(arm_matrix_instance_f32 *q, arm_matrix_instance_f32 *sf_a, arm_matrix_instance_f32 *sf_g,
			  arm_matrix_instance_f32 *bias_g, arm_matrix_instance_f32 *bias_a, float32_t phi, float32_t h,
			  float32_t vn, float32_t ve, float32_t vd, arm_matrix_instance_f32 *a_meas, arm_matrix_instance_f32 *w_meas,
			  arm_matrix_instance_f32 *F, arm_matrix_instance_f32 *we, float32_t FBuff[24 * 24])
{

	float32_t dnbBuff[9], dbnBuff[9], F11VecResult[3], offsetResultData[3], F11Data[9];
	arm_matrix_instance_f32 D_nb, D_bn, offsetResult, F11, finalF11, dwdp;

	arm_mat_init_f32(&D_bn, 3, 3, dbnBuff);
	arm_mat_init_f32(&D_nb, 3, 3, dnbBuff);

	quaternion2DCM(q, &D_nb, dnbBuff);
	arm_mat_trans_f32(&D_nb, &D_bn);

	float32_t F11_vec[3] = {sf_g->pData[0], sf_g->pData[1], sf_g->pData[2]};
	arm_sub_f32(F11_vec, w_meas->pData, F11VecResult, 3);
	arm_offset_f32(sf_g->pData, 1.0f, offsetResultData, 3);

	for (uint8_t i = 0; i < sizeof(offsetResultData) / sizeof(float32_t); i++)
	{
		offsetResultData[i] = -1 / offsetResultData[i];
	}

	float32_t finalF11Vec[3];
	arm_mult_f32(&F11VecResult, &offsetResultData, &finalF11Vec);

	arm_mat_init_f32(&finalF11, 3, 3, finalF11Vec);
	arm_mat_skew_f32(&offsetResult, &F11, F11Data);

	float32_t dwdpBuff[9];
	compute_dwdp(phi, h, ve, vn, &dwdp, dwdpBuff, we);

	arm_matrix_instance_f32 F12;
	float32_t F12Data[9];
	arm_mat_init_f32(&F12, 3, 3, F12Data);
	arm_mat_scale_f32(&D_bn, -1.0f, &D_bn);
	arm_mat_mult_f32(&D_bn, &dwdp, &F12);

	arm_matrix_instance_f32 dwdv, F13;
	float32_t dwdvBuff[9], F13Data[9];
	compute_dwdv(phi, h, &dwdv, dwdvBuff);
	arm_mat_init_f32(&F13, 3, 3, F13Data);
	arm_mat_mult_f32(&D_bn, &dwdv, &F13);

	arm_matrix_instance_f32 F14, Omega, Bg, F16;
	float32_t F14Data[9], OmegaData[9], BgData[9], F16Data[9];

	arm_mat_get_diag_f32(sf_g, &F14, F14Data);
	arm_mat_get_diag_f32(w_meas, &Omega, OmegaData);

	arm_mat_sub_f32(&Omega, &Bg, &F16);
	arm_mat_scale_f32(&F16, -1.0f, &F16);

	arm_matrix_instance_f32 F22, F23;
	float32_t dpdot_dpData[9], dpdot_dvData[9];
	compute_dpdot_dp(phi, h, vn, ve, &F22, dpdot_dpData);
	compute_dpdot_dv(phi, h, &F23, dpdot_dvData);

	arm_matrix_instance_f32 ahat_n, ahatB, ahatBSkew, F31;
	float32_t ahat_nData[9], ahatBData[9], ahatBSkewVar[9], F31Data[9];
	arm_mat_init_f32(&ahatB, 3, 1, ahatBData);
	arm_mat_init_f32(&ahatBSkew, 3, 3, ahatBSkewVar);

	compute_ahat(q, sf_a, sf_g, bias_a, a_meas, &ahat_n, ahat_nData);
	arm_mat_scale_f32(&D_bn, -1.0f, &D_bn);
	arm_mat_mult_f32(&D_bn, &ahat_n, &ahatB);

	arm_mat_skew_f32(&ahatB, &ahatBSkew, ahatBSkewVar);
	arm_mat_scale_f32(&D_nb, -1.0f, &D_nb);
	arm_mat_mult_f32(&ahatBSkew, &D_nb, &F31);

	arm_matrix_instance_f32 F31, F32, F33;
	float32_t dvdot_dpData[9], dvdot_dvData[9], F31Data[9];

	arm_mat_mult_f32(&D_nb, &ahatBSkew, &F31);
	compute_dvdot_dp(phi, h, vn, ve, vd, &F32, dvdot_dpData, we);
	compute_dvdot_dv(phi, h, vn, ve, vd, &F33, dvdot_dvData, we);

	arm_matrix_instance_f32 F35, F37, measDiff;
	float32_t F35Data[9], F37Data[9], measDiffBuff[9];
	arm_mat_init_f32(&F35, 3, 3, F35Data);
	arm_mat_init_f32(&F37, 3, 3, F37Data);
	arm_mat_init_f32(&measDiff, 3, 3, measDiffBuff);

	arm_mat_scale_f32(sf_a, -1.0f, sf_a);
	arm_mat_mult_f32(&D_nb, &F14, &F35);
	arm_mat_sub_f32(a_meas, bias_a, &measDiff);
	arm_mat_mult_f32(&D_nb, &measDiff, &F37);
	arm_mat_scale_f32(sf_a, -1.0f, sf_a);

	assemble_F(FBuff, F11, F12, F13, F14, F16, F22, F23, F31, F32, F33, F35, F37);
	arm_mat_init_f32(F, 24, 24, FBuff);
}

void update(arm_matrix_instance_f32 *x)
{
	float32_t phi = x->pData[4], h = x->pData[6], vn = x->pData[7], ve = x->pData[8], vd = x->pData[9];
	arm_matrix_instance_f32 sf_g, sf_a, bias_g, bias_a, q, D_nb;

	getStateQuaternion(x, q, qBuff);
	getStateASF(x, sf_a, sfaBuff);
	getStateGSF(x, sf_g, sfgBuff);
	getStateABias(x, bias_a, biasABuff);
	getStateGBias(x, bias_g, biasGBuff);
}

void update_GPS(float32_t *x_plus, float32_t *P_plus, float32_t *Pq_plus, float32_t *x_minus, float32_t *P_minus, float32_t *Pq_minus, float32_t *H, float32_t *R, float32_t *lla_meas)
{
	// STEP 2: KALMAN GAIN - Adaptive underweighting based on position uncertainty
	float32_t trace_sub = P_minus[3 * 21 + 3] + P_minus[4 * 21 + 4] + P_minus[5 * 21 + 5];
	float32_t beta = (trace_sub > 1000.0f) ? 0.25f : 0.0f;

	// Compute W = (1+beta)*H*P_minus*H' + R
	arm_matrix_instance_f32 H_mat, P_minus_mat, HPT_mat;
	float32_t HP_data[63];
	arm_mat_init_f32(&H_mat, 3, 21, H);
	arm_mat_init_f32(&P_minus_mat, 21, 21, P_minus);
	arm_mat_init_f32(&HPT_mat, 3, 21, HP_data);
	arm_mat_mult_f32(&H_mat, &P_minus_mat, &HPT_mat);

	arm_matrix_instance_f32 HT_mat;
	float32_t HT_data[63], W_temp_data[9];
	arm_mat_init_f32(&HT_mat, 21, 3, HT_data);
	arm_mat_trans_f32(&H_mat, &HT_mat);

	arm_matrix_instance_f32 W_temp_mat;
	arm_mat_init_f32(&W_temp_mat, 3, 3, W_temp_data);
	arm_mat_mult_f32(&HPT_mat, &HT_mat, &W_temp_mat);

	float32_t W_data[9];
	for (int i = 0; i < 9; i++)
	{
		W_data[i] = (1.0f + beta) * W_temp_data[i] + R[i];
	}

	// Compute PH = P_minus*H'
	arm_matrix_instance_f32 PH_mat;
	float32_t PH_data[63];
	arm_mat_init_f32(&PH_mat, 21, 3, PH_data);
	arm_mat_mult_f32(&P_minus_mat, &HT_mat, &PH_mat);

	// Compute K = PH * W^(-1) using library function
	arm_matrix_instance_f32 W_mat, W_inv_mat;
	float32_t W_inv[9];
	arm_mat_init_f32(&W_mat, 3, 3, W_data);
	arm_mat_init_f32(&W_inv_mat, 3, 3, W_inv);
	arm_mat_inverse_f32(&W_mat, &W_inv_mat);

	arm_matrix_instance_f32 K_mat;
	float32_t K_data[63];
	arm_mat_init_f32(&K_mat, 21, 3, K_data);
	arm_mat_mult_f32(&PH_mat, &W_inv_mat, &K_mat);

	// STEP 3: MEASUREMENT UPDATE - Compute measurement residual
	float32_t residual_data[3];
	for (int i = 0; i < 3; i++)
	{
		residual_data[i] = lla_meas[i] - x_minus[4 + i];
	}

	// Compute Delta_x = K * residual
	arm_matrix_instance_f32 residual_vec, Delta_x_vec;
	float32_t Delta_x_data[21];
	arm_mat_init_f32(&residual_vec, 3, 1, residual_data);
	arm_mat_init_f32(&Delta_x_vec, 21, 1, Delta_x_data);
	arm_mat_mult_f32(&K_mat, &residual_vec, &Delta_x_vec);

	// Update state components
	// Quaternion: unchanged
	for (int i = 0; i < 4; i++)
	{
		x_plus[i] = x_minus[i];
	}

	// Position: p_plus = x_minus(5:7) + Delta_x(4:6)
	for (int i = 0; i < 3; i++)
	{
		x_plus[4 + i] = x_minus[4 + i] + Delta_x_data[3 + i];
	}

	// Velocity: v_plus = x_minus(8:10) + Delta_x(7:9)
	for (int i = 0; i < 3; i++)
	{
		x_plus[7 + i] = x_minus[7 + i] + Delta_x_data[6 + i];
	}

	// Gyro bias: unchanged
	for (int i = 0; i < 3; i++)
	{
		x_plus[10 + i] = x_minus[10 + i];
	}

	// Accel bias: ba_plus = x_minus(14:16) + Delta_x(13:15)
	for (int i = 0; i < 3; i++)
	{
		x_plus[13 + i] = x_minus[13 + i] + Delta_x_data[12 + i];
	}

	// Gyro scale factor: kg_plus = x_minus(17:19) + Delta_x(16:18)
	for (int i = 0; i < 3; i++)
	{
		x_plus[16 + i] = x_minus[16 + i] + Delta_x_data[15 + i];
	}

	// Accel scale factor: ka_plus = x_minus(20:22) + Delta_x(19:21)
	for (int i = 0; i < 3; i++)
	{
		x_plus[19 + i] = x_minus[19 + i] + Delta_x_data[18 + i];
	}

	// Compute P_plus = (I - K*H)*P_minus*(I - K*H)' + K*R*K'
	// Compute KH = K*H
	arm_matrix_instance_f32 KH_mat;
	float32_t KH_data[441];
	arm_mat_init_f32(&KH_mat, 21, 21, KH_data);
	arm_mat_mult_f32(&K_mat, &H_mat, &KH_mat);

	// Create identity matrix I
	float32_t I_data[441];
	arm_matrix_instance_f32 I_mat;
	arm_mat_eye_f32(&I_mat, I_data, 21);

	// Compute I_KH = I - K*H
	arm_matrix_instance_f32 I_KH_mat;
	float32_t I_KH_data[441];
	arm_mat_init_f32(&I_KH_mat, 21, 21, I_KH_data);
	arm_mat_sub_f32(&I_mat, &KH_mat, &I_KH_mat);

	// Compute I_KH_T (transpose of I_KH)
	arm_matrix_instance_f32 I_KH_T_mat;
	float32_t I_KH_T_data[441];
	arm_mat_init_f32(&I_KH_T_mat, 21, 21, I_KH_T_data);
	arm_mat_trans_f32(&I_KH_mat, &I_KH_T_mat);

	// Compute temp1 = I_KH * P_minus
	arm_matrix_instance_f32 temp1_mat;
	float32_t temp1_data[441];
	arm_mat_init_f32(&temp1_mat, 21, 21, temp1_data);
	arm_mat_mult_f32(&I_KH_mat, &P_minus_mat, &temp1_mat);

	// Compute temp2 = (I_KH * P_minus) * I_KH_T
	arm_matrix_instance_f32 temp2_mat;
	float32_t temp2_data[441];
	arm_mat_init_f32(&temp2_mat, 21, 21, temp2_data);
	arm_mat_mult_f32(&temp1_mat, &I_KH_T_mat, &temp2_mat);

	// Compute K*R
	arm_matrix_instance_f32 KR_mat;
	float32_t KR_data[63];
	arm_mat_init_f32(&KR_mat, 21, 3, KR_data);
	arm_matrix_instance_f32 R_mat;
	arm_mat_init_f32(&R_mat, 3, 3, R);
	arm_mat_mult_f32(&K_mat, &R_mat, &KR_mat);

	// Compute K_T = K'
	arm_matrix_instance_f32 KT_mat;
	float32_t KT_data[63];
	arm_mat_init_f32(&KT_mat, 3, 21, KT_data);
	arm_mat_trans_f32(&K_mat, &KT_mat);

	// Compute KRK = KR * K_T
	arm_matrix_instance_f32 KRK_mat;
	float32_t KRK_data[441];
	arm_mat_init_f32(&KRK_mat, 21, 21, KRK_data);
	arm_mat_mult_f32(&KR_mat, &KT_mat, &KRK_mat);

	// Sum: P_plus = temp2 + KRK
	arm_matrix_instance_f32 P_plus_mat;
	arm_mat_init_f32(&P_plus_mat, 21, 21, P_plus);
	arm_mat_add_f32(&temp2_mat, &KRK_mat, &P_plus_mat);

	// Pq_plus = Pq_minus (unchanged)
	memcpy(Pq_plus, Pq_minus, 36 * sizeof(float32_t));
}

void propogate()
