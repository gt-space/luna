#include "ekf.h"

// interval in geodetic altitude over which the interpolant is valid
const float32_t h_base = -50.0f;
const float32_t h_ceil = 50000.0f;

// slope of log normalized pressure at either extrema of the valid interval
const float32_t m_base = -0.00011927191f;
const float32_t m_ceil = -0.00012193789f;

// value of the log normalized pressure at either extrema of the valid interval
const float32_t b_base = 0.0059635397f;
const float32_t b_ceil = -6.9525123f;

// leading coefficient of the derivative of the atmospheric pressure with respect 
// to geodetic altitude at either extrema of the valid interval
const float32_t C_base = -11.999518f;
const float32_t C_ceil = -12.267735f;

// atmospheric pressure at the surface of the reference ellipsoid
const float32_t P_0 = 100606.414f;

// seventh order polynomial interpolant coefficients (alpha)
const float32_t poly_consts[7] = {-0.00011927925f,
                                  -1.8147103e-10f,
                                  -2.445637e-13f,
                                   1.7510401e-17f,
                                  -5.290156e-22f,
                                   7.715311e-27f,
                                  -4.4337637e-32f};

/**
 * @brief Calculates the ambient atmospheric pressure (in Pascals) as a 
 * function of the geodetic altitude over the WGS 84 reference ellipsoid.
 * 
 * The logarithm of the pressure is interpolated by a 7th-order polynomial. 
 * If the provided geodetic altitude exceeds the bounds in which the polynomial 
 * interpolant is valid, backup linear fits are used instead.
 * 
 * @param[in] h     Geodetic altitude above WGS84 reference ellipsoid (meters)
 * 
 * @warning Δh (hOffset) MUST be set prior to flight. Note the positive sign 
 * (which is opposite to the altimeter function).
 */
inline float32_t filter_P(float32_t h) {

    // Apply offset
    float32_t hBuff = h + hOffset;

    /*
    Exponentiating the log normalized pressure returns the normalized pressure
    we then multiply by P_0 in order to get the actual pressure
    */

    return P_0 * expf(filter_lognormP(hBuff));
}

/** 
 * @brief  the logarithm derivative of the normalized ambient atmospheric pressure 
 * as a function of the geodetic altitude over the WGS 84 reference ellipsoid. 
 * 7th-order polynomial interpolant used. 
 * 
 * @param[in] h     Geodetic altitude above the WGS 84 reference ellipsoid (metres)
 * 
 */
inline float32_t filter_dLogNorm_dH(float32_t h) {

    float32_t h2 = h * h;
    float32_t h3 = h2 * h;
    float32_t h4 = h3 * h;
    float32_t h5 = h4 * h;
    float32_t h6 = h5 * h;
    float32_t h7 = h6 * h;

    return poly_consts[0] +
           2 * poly_consts[1] * h +
           3 * poly_consts[2] * h2 +
           4 * poly_consts[3] * h3 +
           5 * poly_consts[4] * h4 +
           6 * poly_consts[5] * h5 +
           7 * poly_consts[6] * h6;
}

/**
 * @brief Calculates the logarithm of the normalized ambient atmospheric pressure 
 * as a function of the geodetic altitude over the WGS 84 reference ellipsoid. 
 * 7th-order polynomial interpolant used. 
 * 
 * @param[in] h     Geodetic altitude above the WGS 84 reference ellipsoid (metres)
 * 
 * @note If the provided geodetic altitude exceeds the bounds in which the polynomial interpolant
 * is valid, backup linear fits are used instead.
 */
float32_t filter_lognormP(float32_t h) {

    if (h < h_base) {
        return m_base * (h - h_base) + b_base;
    } else if (h > h_ceil) {
        return m_ceil * (h - h_ceil) + b_ceil;
    } else {

        float32_t h2 = h * h;
        float32_t h3 = h2 * h;
        float32_t h4 = h3 * h;
        float32_t h5 = h4 * h;
        float32_t h6 = h5 * h;
        float32_t h7 = h6 * h;

        return poly_consts[0] * h +
               poly_consts[1] * h2 +
               poly_consts[2] * h3 +
               poly_consts[3] * h4 +
               poly_consts[4] * h5 +
               poly_consts[5] * h6 +
               poly_consts[6] * h7;
    }
}

float32_t filter_dP_dH(float32_t h) {

    // Apply offset
    float32_t hBuff = h + hOffset;
    float32_t partialP = expf(filter_lognormP(h));

    // 
    if (hBuff < h_base) {
        return partialP * C_base;
    } else if (hBuff > h_ceil) {
        return partialP * C_ceil;
    } else {
        return partialP * P_0 * filter_dLogNorm_dH(hBuff);
    }
}

void initialize_Hb(arm_matrix_instance_f32* x, arm_matrix_instance_f32* Hb, float32_t HbBuff[1*21]) {
	memset(HbBuff, 0, 21*sizeof(float32_t));
	HbBuff[5] = filter_dP_dH(x->pData[6]);
}


