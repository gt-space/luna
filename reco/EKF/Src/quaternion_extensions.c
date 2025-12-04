#include "quaternion_extensions.h"
#include "trig_extensions.h"

void arm_quaternion_sandwich_f32(arm_matrix_instance_f32* q, arm_matrix_instance_f32* x, arm_matrix_instance_f32* y, float32_t yBuff[4]) {
	arm_matrix_instance_f32 qConj;
	float32_t qConjBuff[4], term1Buff[4];

	// qconj(q)
	arm_quaternion_qconj_f32(q, &qConj, qConjBuff);

	// qmult(x,qconj(q))
	arm_quaternion_product_single_f32(x->pData, qConj.pData, term1Buff);

	// qmult(q,qmult(x,qconj(q)))
	arm_quaternion_product_single_f32(q->pData, term1Buff, yBuff);

	arm_mat_init_f32(y, 4, 1, yBuff);
}

void arm_quaternion_exp_f32(const arm_matrix_instance_f32* v,
                            arm_matrix_instance_f32* dq,
                            float32_t dqBuff[4]) {
    // v is a 3x1 vector
    float32_t vNorm;

    float32_t vx = v->pData[0];
    float32_t vy = v->pData[1];
    float32_t vz = v->pData[2];

    arm_sqrt_f32(vx * vx + vy * vy + vz * vz, &vNorm);

    // Compute norm(v)
    if (vNorm < 1e-6f) {
        // dq = [1; v]
        dqBuff[0] = 1.0f;
        dqBuff[1] = vx;
        dqBuff[2] = vy;
        dqBuff[3] = vz;

        // Normalize dq
        float32_t dqnorm;
        arm_sqrt_f32(dqBuff[0]*dqBuff[0] +
                     dqBuff[1]*dqBuff[1] +
                     dqBuff[2]*dqBuff[2] +
                     dqBuff[3]*dqBuff[3], &dqnorm);

        dqBuff[0] /= dqnorm;
        dqBuff[1] /= dqnorm;
        dqBuff[2] /= dqnorm;
        dqBuff[3] /= dqnorm;

    } else {
        // dq = [cos(vnorm); sin(vnorm)*(v/vnorm)]
        float32_t s, c;
        arm_sin_cos_f32(rad2deg(vNorm), &s, &c);

        dqBuff[0] = c;
        dqBuff[1] = s * (vx / vNorm);
        dqBuff[2] = s * (vy / vNorm);
        dqBuff[3] = s * (vz / vNorm);
    }

    // Initialize output quaternion
    arm_mat_init_f32(dq, 4, 1, dqBuff);
}

