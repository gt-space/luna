#include "ekf.h"

// interval in geodetic altitude over which the interpolant is valid
static const float32_t h_base = -1000.0f;
static const float32_t h_ceil = 12000.0f;

// slope of log normalized pressure at either extrema of the valid interval
static const float32_t m_base = -0.00011841112f;
static const float32_t m_ceil = -0.00015328368f;

// value of the log normalized pressure at either extrema of the valid interval
static const float32_t b_base = 0.11881527f;
static const float32_t b_ceil = -1.6251616f;

// log atmospheric pressure at the surface of the reference ellipsoid
static const float32_t b_0 = 11.518971f;

// fifth order polynomial interpolant coefficients (Beta)
static const float32_t poly_consts[5] = {-0.00011933408,
										  -6.295912e-10f,
										  -1.06790716e-13f,
										   3.986928e-18f,
										  -2.5322159e-24f};

/**
 * @brief Estimate the geodetic altitude (in metres) by lerping (linear interpolate) between the 
 * ends of the valid interval, using the logarithm of the normalized 
 * atmospheric pressure as the lerp parameter.
 * 
 * @param[in] logP Log normalized atmospheric pressure (Pascals)
 * @return Geodetic altitude (meters)
 */
inline float32_t lerp(float32_t logP) {
    return (h_ceil * (b_base - logP) + h_base * (logP - b_ceil)) / (b_base - b_ceil);
}

/**
 * @brief Calculate the geodetic altitude (in metres) as a function of the atmospheric pressure.
 * 
 * @param[in] P     Measured atmospheric pressure (pascals)
 * @return          Geodetic altitude (meters)
 */
inline float32_t pressure_altimeter_uncorrected(float32_t P) {
    return logP2alt(logf(P) - b_0);
}

/**
 * @brief Calculate the geodetic altitude (in metres) as a function of the atmospheric pressure.
 * 
 * @param[in] P         Atmospheric pressure (pascals)
 */
inline float32_t pressure_altimeter_corrected(float32_t P) {
    return pressure_altimeter_uncorrected(P) - hOffset;
}

/**
 * @brief Nonlinear root-finding solver employing Laguerre's method.
 * 
 * Suppose we have value `y` described by `y = f(x)` and its nominal measured 
 * value `ŷ`, and wish to find `x` such that `f(x) = ŷ`. This is equivalent to 
 * solving the nonlinear root-finding problem `f(x) - ŷ = 0` in `x`. In cases 
 * where `f(x)` is a polynomial function, we may employ Laguerre's method.
 * 
 * @param[in] x0    The initial guess for the root
 * @param[in] yHat  The nominal value we wish to match
 */
float32_t laguerre_solve(float32_t x0, float32_t yHat) {

    float32_t x = x0;

    float32_t x2, x3, x4, x5;
    float32_t f, fPrime, fDoublePrime;
    float32_t G, H, lambda, a;

    int32_t n = 5;
    float32_t epsilon = 1e-7f;

    // Max Number of Iterations = 2
    // Testing shows that the max amount of iterations needed to converge
    // was two 
    for (uint8_t i = 0; i < 2; i++) {
        // buffer for storing the powers in x
        x2 = x * x;
        x3 = x * x2;
        x4 = x * x3;
        x5 = x * x4;

        // Objective Function Value
        f = (poly_consts[0] * x) + (poly_consts[1] * x2) + (poly_consts[2] * x3) 
            + (poly_consts[3] * x4) + (poly_consts[4] * x5) - yHat;
        
        if (fabs(f) < epsilon) {
            break;
        }

        // Objective Function Derivative
        fPrime = (poly_consts[0]) + (2 * poly_consts[1] * x) + (3 * poly_consts[2] * x2) 
                 + (4 * poly_consts[3] * x3) + (5 * poly_consts[4] * x4); 
        
        // Objective Function Second Derivative
        fDoublePrime = (2 * poly_consts[1]) + (6 * poly_consts[2] * x) 
                        + (12 * poly_consts[3] * x2) + (20 * poly_consts[4] * x3);

        // G, H, and a are values used in Laguerre's method
        G = fPrime / f;
        H = G * G - (fDoublePrime / f);

        /*
        λ is just a buffer to store the denominator of a factoring the n 
        over the G^2 in H in this expression is inadvisable as this approximately 
        doubles the floating point noise and round-off errors
        */ 
        lambda = (n - 1) * (n * H - G * G);

        if (lambda < 0) {
        	break;
        }

        lambda = sqrtf(lambda);

        /*
         this upcoming one liner expression for a is identical to the following conditional block:
         if G < 0.0
            a = n / (G - λ)
         else
            a = n / (G + λ)
         end
         essentially, when calculating a, we want to minimize the step (because we are finding the closest root)
         additionally, we want to maximize the denominator of a, which is G ± λ
         in order to avoid catastrophic cancellation, we pick the sign of ± such that G ± λ is maximized, which of course also minimizes a
        */
        a = (G < 0) ? (n / (G - lambda)) : (n / (G + lambda));

        // Update Step
        x -= a;

        // if the change just made was small, have arrived at the solution
        if (fabs(a) < epsilon) {
            break;
        }
    }
    return x;
}

/**
 * @brief Calculate the geodetic altitude (metres) as a function of the logarithm of 
 * the normalized atmospheric pressure.
 * 
 * @param[in] logP Log normalized atmospheric pressure (pascals)
 * @return Geodetic altitude (meters)
 * 
 * @note  The normalized atmospheric pressure is the pressure divided 
 *        by the pressure at the surface of the WGS 84 reference ellipsoid.
 */
float32_t logP2alt(float32_t logP) {
    
    float32_t h;

    /*
    First two cases happen if logP is out of bounds
    if logP is greater than the maximum interpolant pressure, we are below h_base
    */

    if (logP > b_base) {
        h = h_base + (logP - b_base) / m_base;
        // if logP is smaller than the minimum interpolant pressure, we are above h_ceil
    } else if (logP < b_ceil) {
        h = h_ceil + (logP - b_ceil) / m_ceil;
        // otherwise we will call Laguerre's method
    } else {
        /*
            In order to initialize Laguerre's method, we need an 
            initial guess for our altitude we will use a single linsolve
             in order to achieve this
        */

        h = laguerre_solve(lerp(logP), logP);
    }

    return h;
}





