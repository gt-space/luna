#include "ekf.h"

const float32_t a[8] = {5.0122185, -4.9929004e-5, -5.415637e-10,
						-3.837231e-14, 2.55155e-18, -5.321706e-23,
						4.813401e-28, -1.6294356e-33};

const float32_t A_p_ini = -8529.674f;
const float32_t p_0_inv = 9.7356315e-6f;

const float32_t p0 = 102715.47296217596;
const float32_t hOffset = 111.79493450041844;

static inline float32_t invert_measurement_guess(float32_t press) {
    return A_p_ini * logf(press * p_0_inv);
}

float32_t pressure_to_altitude(float32_t press) {

    float32_t h;
    if ((press < 105240.625f) && (press > 100510.31f))
    {
        float32_t a = -200.0f;
        float32_t b =  200.0f;
        float32_t c;

        for (uint8_t i = 0; i < 16; i++)
        {
            c = 0.5f * (a + b);
            h = c;

            if (fabsf(b - a) < 1.0e-5f)
                break;

            // foo is residual in the code
            float32_t residual = pressure_function(c) - log10f(press);

            if (residual == 0.0f)
                break;
            else if (residual > 0.0f)
                a = c;
            else
                b = c;
        }
    }
    else
    {
        h = invert_measurement_guess(press);
        float32_t logp = log10f(press);

        for (int i = 0; i < 16; i++)
        {
            float delta_h =
                (pressure_function(h) - logp) / pressure_derivative(h);

            h -= delta_h;

            if (fabsf(delta_h) < 1.0e-5f)
                break;
        }
    }

    return h;
}


float32_t pressure_function(float32_t alt) {

	float32_t h = alt + hOffset;

    float32_t poly =
        a[0] +
        a[1]*h +
        a[2]*h*h +
        a[3]*h*h*h +
        a[4]*h*h*h*h +
        a[5]*h*h*h*h*h +
        a[6]*h*h*h*h*h*h +
        a[7]*h*h*h*h*h*h*h;

	return powf(10.0, poly);
}

float32_t pressure_derivative(float32_t alt) {

	float32_t h = alt + hOffset;

    // Compute the polynomial f(h)
    float32_t poly =
        a[0] +
        a[1]*h +
        a[2]*h*h +
        a[3]*h*h*h +
        a[4]*h*h*h*h +
        a[5]*h*h*h*h*h +
        a[6]*h*h*h*h*h*h +
        a[7]*h*h*h*h*h*h*h;

    // Compute the derivative f'(h)
    float32_t dpoly =
        a[1] +
        2.0f*a[2]*h +
        3.0f*a[3]*h*h +
        4.0f*a[4]*h*h*h +
        5.0f*a[5]*h*h*h*h +
        6.0f*a[6]*h*h*h*h*h +
        7.0f*a[7]*h*h*h*h*h*h;

    // dp/dh = log(10) * f'(h) * 10^f(h)
    return logf(10.0f) * dpoly * powf(10.0f, poly);
}

void initialize_Hb(arm_matrix_instance_f32* x, arm_matrix_instance_f32* Hb, float32_t HbBuff[1*21]) {
	memset(HbBuff, 0, 21*sizeof(float32_t));
	HbBuff[5] = pressure_derivative(x->pData[6]);
}
