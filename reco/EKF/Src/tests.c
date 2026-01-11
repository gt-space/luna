#include "tests.h"
#include "float.h"
#include "ekf.h"
#include "ekf_utils.h"
#include "../CControl/ccontrol.h"

// Test Suite for the EKF code to ensure it matches with the known good
// Python version located at the gt-space/EKF/run_MEKF.inbpy

// Test statistics
static test_stats_t g_test_stats = {0, 0, 0};

// Tolerance for floating point comparisons
#define TEST_TOLERANCE 1e-6f

// Helper function to check matrix dimensions
bool test_matrix_dimensions(arm_matrix_instance_f32 *matrix, uint16_t expected_rows, uint16_t expected_cols)
{
    return (matrix->numRows == expected_rows && matrix->numCols == expected_cols);
}

// Helper function to check if a value is finite (portable version)
static inline bool is_finite_f32(float32_t x)
{
    // Check for NaN and infinity
    // NaN: x != x is true
    // Infinity: |x| > FLT_MAX or x is inf
    return (x == x) && (x <= FLT_MAX) && (x >= -FLT_MAX);
}

// Helper function to check if all matrix elements are finite
bool test_matrix_finite(arm_matrix_instance_f32 *matrix)
{
    for (uint16_t i = 0; i < matrix->numRows * matrix->numCols; i++)
    {
        if (!is_finite_f32(matrix->pData[i]))
        {
            return false;
        }
    }
    return true;
}

// Helper function to check if all vector elements are finite
bool test_vector_finite(float32_t *vector, uint32_t length)
{
    for (uint32_t i = 0; i < length; i++)
    {
        if (!is_finite_f32(vector[i]))
        {
            return false;
        }
    }
    return true;
}

// Helper function to check if matrix is not all zeros
bool test_matrix_not_all_zero(arm_matrix_instance_f32 *matrix)
{
    for (uint16_t i = 0; i < matrix->numRows * matrix->numCols; i++)
    {
        if (fabsf(matrix->pData[i]) > TEST_TOLERANCE)
        {
            return true;
        }
    }
    return false;
}

// Print test result
void print_test_result(const char *test_name, bool passed, const char *message)
{
    if (passed)
    {
        printf("[PASS] %s\n", test_name);
        g_test_stats.passed_tests++;
    }
    else
    {
        printf("[FAIL] %s", test_name);
        if (message != NULL)
        {
            printf(": %s", message);
        }
        printf("\n");
        g_test_stats.failed_tests++;
    }
    g_test_stats.total_tests++;
}

// ============================================================================
// Tests for compute_wn
// ============================================================================

bool test_compute_wn_basic(void)
{
    float32_t phi = 45.0f;       // 45 degrees latitude
    float32_t h = 1000.0f;       // 1000m altitude
    float32_t vn = 10.0f;        // 10 m/s north velocity
    float32_t ve = 5.0f;         // 5 m/s east velocity
    float32_t we = 7.292115e-5f; // Earth rotation rate (rad/s)

    arm_matrix_instance_f32 wn;
    float32_t wn_buffer[3];

    compute_wn(phi, h, vn, ve, &wn, we, wn_buffer);

    // Check dimensions
    if (!test_matrix_dimensions(&wn, 3, 1))
    {
        print_test_result("test_compute_wn_basic", false, "Wrong dimensions");
        return false;
    }

    // Check finite values
    if (!test_vector_finite(wn_buffer, 3))
    {
        print_test_result("test_compute_wn_basic", false, "Non-finite values");
        return false;
    }

    // Check that result is not all zeros
    bool all_zero = true;
    for (int i = 0; i < 3; i++)
    {
        if (fabsf(wn_buffer[i]) > TEST_TOLERANCE)
        {
            all_zero = false;
            break;
        }
    }
    if (all_zero)
    {
        print_test_result("test_compute_wn_basic", false, "Result is all zeros");
        return false;
    }

    print_test_result("test_compute_wn_basic", true, NULL);
    return true;
}

bool test_compute_wn_zero_velocity(void)
{
    float32_t phi = 0.0f;
    float32_t h = 0.0f;
    float32_t vn = 0.0f;
    float32_t ve = 0.0f;
    float32_t we = 7.292115e-5f;

    arm_matrix_instance_f32 wn;
    float32_t wn_buffer[3];

    compute_wn(phi, h, vn, ve, &wn, we, wn_buffer);

    // With zero velocity, only Earth rotation should contribute
    if (!test_matrix_dimensions(&wn, 3, 1))
    {
        print_test_result("test_compute_wn_zero_velocity", false, "Wrong dimensions");
        return false;
    }

    if (!test_vector_finite(wn_buffer, 3))
    {
        print_test_result("test_compute_wn_zero_velocity", false, "Non-finite values");
        return false;
    }

    print_test_result("test_compute_wn_zero_velocity", true, NULL);
    return true;
}

bool test_compute_wn_edge_cases(void)
{
    // Test at equator (phi = 0)
    float32_t phi = 0.0f;
    float32_t h = 10000.0f; // High altitude
    float32_t vn = 100.0f;
    float32_t ve = 100.0f;
    float32_t we = 7.292115e-5f;

    arm_matrix_instance_f32 wn;
    float32_t wn_buffer[3];

    compute_wn(phi, h, vn, ve, &wn, we, wn_buffer);

    if (!test_matrix_dimensions(&wn, 3, 1) || !test_vector_finite(wn_buffer, 3))
    {
        print_test_result("test_compute_wn_edge_cases", false, "Failed at equator");
        return false;
    }

    // Test at pole (phi = 90)
    phi = 90.0f;
    compute_wn(phi, h, vn, ve, &wn, we, wn_buffer);

    if (!test_matrix_dimensions(&wn, 3, 1) || !test_vector_finite(wn_buffer, 3))
    {
        print_test_result("test_compute_wn_edge_cases", false, "Failed at pole");
        return false;
    }

    print_test_result("test_compute_wn_edge_cases", true, NULL);
    return true;
}

// ============================================================================
// Tests for compute_what
// ============================================================================

bool test_compute_what_basic(void)
{
    // Setup quaternion (identity quaternion: [1, 0, 0, 0])
    arm_matrix_instance_f32 q;
    float32_t q_data[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    arm_mat_init_f32(&q, 4, 1, q_data);

    // Setup bias_g (zero bias)
    arm_matrix_instance_f32 bias_g;
    float32_t bias_g_data[3] = {0.0f, 0.0f, 0.0f};
    arm_mat_init_f32(&bias_g, 3, 1, bias_g_data);

    // Setup sf_g (scale factor, small values)
    arm_matrix_instance_f32 sf_g;
    float32_t sf_g_data[3] = {0.01f, 0.01f, 0.01f};
    arm_mat_init_f32(&sf_g, 3, 1, sf_g_data);

    // Setup w_meas (measured angular velocity)
    arm_matrix_instance_f32 w_meas;
    float32_t w_meas_data[3] = {0.1f, 0.1f, 0.1f};
    arm_mat_init_f32(&w_meas, 3, 1, w_meas_data);

    float32_t phi = 45.0f;
    float32_t h = 1000.0f;
    float32_t vn = 10.0f;
    float32_t ve = 5.0f;
    float32_t we = 7.292115e-5f;

    arm_matrix_instance_f32 what;
    float32_t what_buffer[3];

    // Make copies since compute_what modifies input matrices
    float32_t sf_g_copy[3], w_meas_copy[3];
    memcpy(sf_g_copy, sf_g_data, 3 * sizeof(float32_t));
    memcpy(w_meas_copy, w_meas_data, 3 * sizeof(float32_t));

    arm_matrix_instance_f32 sf_g_work, w_meas_work;
    arm_mat_init_f32(&sf_g_work, 3, 1, sf_g_copy);
    arm_mat_init_f32(&w_meas_work, 3, 1, w_meas_copy);

    compute_what(&q, &bias_g, &sf_g_work, phi, h, vn, ve, we, &w_meas_work, &what, what_buffer);

    if (!test_matrix_dimensions(&what, 3, 1))
    {
        print_test_result("test_compute_what_basic", false, "Wrong dimensions");
        return false;
    }

    if (!test_vector_finite(what_buffer, 3))
    {
        print_test_result("test_compute_what_basic", false, "Non-finite values");
        return false;
    }

    print_test_result("test_compute_what_basic", true, NULL);
    return true;
}

bool test_compute_what_zero_inputs(void)
{
    arm_matrix_instance_f32 q;
    float32_t q_data[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    arm_mat_init_f32(&q, 4, 1, q_data);

    arm_matrix_instance_f32 bias_g;
    float32_t bias_g_data[3] = {0.0f, 0.0f, 0.0f};
    arm_mat_init_f32(&bias_g, 3, 1, bias_g_data);

    arm_matrix_instance_f32 sf_g;
    float32_t sf_g_data[3] = {0.0f, 0.0f, 0.0f};
    arm_mat_init_f32(&sf_g, 3, 1, sf_g_data);

    arm_matrix_instance_f32 w_meas;
    float32_t w_meas_data[3] = {0.0f, 0.0f, 0.0f};
    arm_mat_init_f32(&w_meas, 3, 1, w_meas_data);

    float32_t phi = 0.0f;
    float32_t h = 0.0f;
    float32_t vn = 0.0f;
    float32_t ve = 0.0f;
    float32_t we = 7.292115e-5f;

    arm_matrix_instance_f32 what;
    float32_t what_buffer[3];

    float32_t sf_g_copy[3], w_meas_copy[3];
    memcpy(sf_g_copy, sf_g_data, 3 * sizeof(float32_t));
    memcpy(w_meas_copy, w_meas_data, 3 * sizeof(float32_t));

    arm_matrix_instance_f32 sf_g_work, w_meas_work;
    arm_mat_init_f32(&sf_g_work, 3, 1, sf_g_copy);
    arm_mat_init_f32(&w_meas_work, 3, 1, w_meas_copy);

    compute_what(&q, &bias_g, &sf_g_work, phi, h, vn, ve, we, &w_meas_work, &what, what_buffer);

    if (!test_matrix_dimensions(&what, 3, 1) || !test_vector_finite(what_buffer, 3))
    {
        print_test_result("test_compute_what_zero_inputs", false, "Failed with zero inputs");
        return false;
    }

    print_test_result("test_compute_what_zero_inputs", true, NULL);
    return true;
}

// ============================================================================
// Tests for compute_ahat
// ============================================================================

bool test_compute_ahat_basic(void)
{
    // Setup quaternion (identity)
    arm_matrix_instance_f32 q;
    float32_t q_data[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    arm_mat_init_f32(&q, 4, 1, q_data);

    // Setup sf_a (scale factor)
    arm_matrix_instance_f32 sf_a;
    float32_t sf_a_data[3] = {0.01f, 0.01f, 0.01f};
    arm_mat_init_f32(&sf_a, 3, 1, sf_a_data);

    // Setup bias_a (zero bias)
    arm_matrix_instance_f32 bias_a;
    float32_t bias_a_data[3] = {0.0f, 0.0f, 0.0f};
    arm_mat_init_f32(&bias_a, 3, 1, bias_a_data);

    // Setup a_meas (measured acceleration)
    arm_matrix_instance_f32 a_meas;
    float32_t a_meas_data[3] = {9.81f, 0.0f, 0.0f}; // Gravity in x direction
    arm_mat_init_f32(&a_meas, 3, 1, a_meas_data);

    arm_matrix_instance_f32 ahat_n;
    float32_t ahat_buffer[3];

    // Make copies since compute_ahat modifies input matrices
    float32_t sf_a_copy[3], a_meas_copy[3];
    memcpy(sf_a_copy, sf_a_data, 3 * sizeof(float32_t));
    memcpy(a_meas_copy, a_meas_data, 3 * sizeof(float32_t));

    arm_matrix_instance_f32 sf_a_work, a_meas_work;
    arm_mat_init_f32(&sf_a_work, 3, 1, sf_a_copy);
    arm_mat_init_f32(&a_meas_work, 3, 1, a_meas_copy);

    compute_ahat(&q, &sf_a_work, &bias_a, &a_meas_work, &ahat_n, ahat_buffer);

    if (!test_matrix_dimensions(&ahat_n, 3, 1))
    {
        print_test_result("test_compute_ahat_basic", false, "Wrong dimensions");
        return false;
    }

    if (!test_vector_finite(ahat_buffer, 3))
    {
        print_test_result("test_compute_ahat_basic", false, "Non-finite values");
        return false;
    }

    print_test_result("test_compute_ahat_basic", true, NULL);
    return true;
}

bool test_compute_ahat_zero_inputs(void)
{
    arm_matrix_instance_f32 q;
    float32_t q_data[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    arm_mat_init_f32(&q, 4, 1, q_data);

    arm_matrix_instance_f32 sf_a;
    float32_t sf_a_data[3] = {0.0f, 0.0f, 0.0f};
    arm_mat_init_f32(&sf_a, 3, 1, sf_a_data);

    arm_matrix_instance_f32 bias_a;
    float32_t bias_a_data[3] = {0.0f, 0.0f, 0.0f};
    arm_mat_init_f32(&bias_a, 3, 1, bias_a_data);

    arm_matrix_instance_f32 a_meas;
    float32_t a_meas_data[3] = {0.0f, 0.0f, 0.0f};
    arm_mat_init_f32(&a_meas, 3, 1, a_meas_data);

    arm_matrix_instance_f32 ahat_n;
    float32_t ahat_buffer[3];

    float32_t sf_a_copy[3], a_meas_copy[3];
    memcpy(sf_a_copy, sf_a_data, 3 * sizeof(float32_t));
    memcpy(a_meas_copy, a_meas_data, 3 * sizeof(float32_t));

    arm_matrix_instance_f32 sf_a_work, a_meas_work;
    arm_mat_init_f32(&sf_a_work, 3, 1, sf_a_copy);
    arm_mat_init_f32(&a_meas_work, 3, 1, a_meas_copy);

    compute_ahat(&q, &sf_a_work, &bias_a, &a_meas_work, &ahat_n, ahat_buffer);

    if (!test_matrix_dimensions(&ahat_n, 3, 1) || !test_vector_finite(ahat_buffer, 3))
    {
        print_test_result("test_compute_ahat_zero_inputs", false, "Failed with zero inputs");
        return false;
    }

    print_test_result("test_compute_ahat_zero_inputs", true, NULL);
    return true;
}

// ============================================================================
// Tests for compute_F
// ============================================================================

bool test_compute_F_dimensions(void)
{
    // Setup all required inputs
    arm_matrix_instance_f32 q;
    float32_t q_data[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    arm_mat_init_f32(&q, 4, 1, q_data);

    // Make copies since compute_F may modify inputs
    float32_t sf_a_data[3] = {0.01f, 0.01f, 0.01f};
    float32_t sf_a_copy[3];
    memcpy(sf_a_copy, sf_a_data, 3 * sizeof(float32_t));
    arm_matrix_instance_f32 sf_a;
    arm_mat_init_f32(&sf_a, 3, 1, sf_a_copy);

    float32_t sf_g_data[3] = {0.01f, 0.01f, 0.01f};
    float32_t sf_g_copy[3];
    memcpy(sf_g_copy, sf_g_data, 3 * sizeof(float32_t));
    arm_matrix_instance_f32 sf_g;
    arm_mat_init_f32(&sf_g, 3, 1, sf_g_copy);

    arm_matrix_instance_f32 bias_g;
    float32_t bias_g_data[3] = {0.0f, 0.0f, 0.0f};
    arm_mat_init_f32(&bias_g, 3, 1, bias_g_data);

    arm_matrix_instance_f32 bias_a;
    float32_t bias_a_data[3] = {0.0f, 0.0f, 0.0f};
    arm_mat_init_f32(&bias_a, 3, 1, bias_a_data);

    float32_t a_meas_data[3] = {9.81f, 0.0f, 0.0f};
    float32_t a_meas_copy[3];
    memcpy(a_meas_copy, a_meas_data, 3 * sizeof(float32_t));
    arm_matrix_instance_f32 a_meas;
    arm_mat_init_f32(&a_meas, 3, 1, a_meas_copy);

    arm_matrix_instance_f32 w_meas;
    float32_t w_meas_data[3] = {0.1f, 0.1f, 0.1f};
    arm_mat_init_f32(&w_meas, 3, 1, w_meas_data);

    float32_t phi = 45.0f;
    float32_t h = 1000.0f;
    float32_t vn = 10.0f;
    float32_t ve = 5.0f;
    float32_t vd = -1.0f;
    float32_t we = 7.292115e-5f;

    arm_matrix_instance_f32 F;
    float32_t F_buffer[21 * 21];

    compute_F(&q, &sf_a, &sf_g, &bias_g, &bias_a, phi, h, vn, ve, vd,
              &a_meas, &w_meas, we, &F, F_buffer);

    if (!test_matrix_dimensions(&F, 21, 21))
    {
        print_test_result("test_compute_F_dimensions", false, "Wrong dimensions (expected 21x21)");
        return false;
    }

    print_test_result("test_compute_F_dimensions", true, NULL);
    return true;
}

bool test_compute_F_finite_values(void)
{
    arm_matrix_instance_f32 q;
    float32_t q_data[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    arm_mat_init_f32(&q, 4, 1, q_data);

    float32_t sf_a_data[3] = {0.01f, 0.01f, 0.01f};
    float32_t sf_a_copy[3];
    memcpy(sf_a_copy, sf_a_data, 3 * sizeof(float32_t));
    arm_matrix_instance_f32 sf_a;
    arm_mat_init_f32(&sf_a, 3, 1, sf_a_copy);

    float32_t sf_g_data[3] = {0.01f, 0.01f, 0.01f};
    float32_t sf_g_copy[3];
    memcpy(sf_g_copy, sf_g_data, 3 * sizeof(float32_t));
    arm_matrix_instance_f32 sf_g;
    arm_mat_init_f32(&sf_g, 3, 1, sf_g_copy);

    arm_matrix_instance_f32 bias_g;
    float32_t bias_g_data[3] = {0.0f, 0.0f, 0.0f};
    arm_mat_init_f32(&bias_g, 3, 1, bias_g_data);

    arm_matrix_instance_f32 bias_a;
    float32_t bias_a_data[3] = {0.0f, 0.0f, 0.0f};
    arm_mat_init_f32(&bias_a, 3, 1, bias_a_data);

    float32_t a_meas_data[3] = {9.81f, 0.0f, 0.0f};
    float32_t a_meas_copy[3];
    memcpy(a_meas_copy, a_meas_data, 3 * sizeof(float32_t));
    arm_matrix_instance_f32 a_meas;
    arm_mat_init_f32(&a_meas, 3, 1, a_meas_copy);

    arm_matrix_instance_f32 w_meas;
    float32_t w_meas_data[3] = {0.1f, 0.1f, 0.1f};
    arm_mat_init_f32(&w_meas, 3, 1, w_meas_data);

    float32_t phi = 45.0f;
    float32_t h = 1000.0f;
    float32_t vn = 10.0f;
    float32_t ve = 5.0f;
    float32_t vd = -1.0f;
    float32_t we = 7.292115e-5f;

    arm_matrix_instance_f32 F;
    float32_t F_buffer[21 * 21];

    compute_F(&q, &sf_a, &sf_g, &bias_g, &bias_a, phi, h, vn, ve, vd,
              &a_meas, &w_meas, we, &F, F_buffer);

    if (!test_matrix_finite(&F))
    {
        print_test_result("test_compute_F_finite_values", false, "Non-finite values in F matrix");
        return false;
    }

    print_test_result("test_compute_F_finite_values", true, NULL);
    return true;
}

bool test_compute_F_zero_velocity(void)
{
    arm_matrix_instance_f32 q;
    float32_t q_data[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    arm_mat_init_f32(&q, 4, 1, q_data);

    float32_t sf_a_data[3] = {0.01f, 0.01f, 0.01f};
    float32_t sf_a_copy[3];
    memcpy(sf_a_copy, sf_a_data, 3 * sizeof(float32_t));
    arm_matrix_instance_f32 sf_a;
    arm_mat_init_f32(&sf_a, 3, 1, sf_a_copy);

    float32_t sf_g_data[3] = {0.01f, 0.01f, 0.01f};
    float32_t sf_g_copy[3];
    memcpy(sf_g_copy, sf_g_data, 3 * sizeof(float32_t));
    arm_matrix_instance_f32 sf_g;
    arm_mat_init_f32(&sf_g, 3, 1, sf_g_copy);

    arm_matrix_instance_f32 bias_g;
    float32_t bias_g_data[3] = {0.0f, 0.0f, 0.0f};
    arm_mat_init_f32(&bias_g, 3, 1, bias_g_data);

    arm_matrix_instance_f32 bias_a;
    float32_t bias_a_data[3] = {0.0f, 0.0f, 0.0f};
    arm_mat_init_f32(&bias_a, 3, 1, bias_a_data);

    float32_t a_meas_data[3] = {0.0f, 0.0f, 0.0f};
    float32_t a_meas_copy[3];
    memcpy(a_meas_copy, a_meas_data, 3 * sizeof(float32_t));
    arm_matrix_instance_f32 a_meas;
    arm_mat_init_f32(&a_meas, 3, 1, a_meas_copy);

    arm_matrix_instance_f32 w_meas;
    float32_t w_meas_data[3] = {0.0f, 0.0f, 0.0f};
    arm_mat_init_f32(&w_meas, 3, 1, w_meas_data);

    float32_t phi = 0.0f;
    float32_t h = 0.0f;
    float32_t vn = 0.0f;
    float32_t ve = 0.0f;
    float32_t vd = 0.0f;
    float32_t we = 7.292115e-5f;

    arm_matrix_instance_f32 F;
    float32_t F_buffer[21 * 21];

    compute_F(&q, &sf_a, &sf_g, &bias_g, &bias_a, phi, h, vn, ve, vd,
              &a_meas, &w_meas, we, &F, F_buffer);

    if (!test_matrix_dimensions(&F, 21, 21) || !test_matrix_finite(&F))
    {
        print_test_result("test_compute_F_zero_velocity", false, "Failed with zero velocity");
        return false;
    }

    print_test_result("test_compute_F_zero_velocity", true, NULL);
    return true;
}

// ============================================================================
// Tests for compute_G
// ============================================================================

bool test_compute_G_dimensions(void)
{
    arm_matrix_instance_f32 sf_g;
    float32_t sf_g_data[3] = {0.01f, 0.01f, 0.01f};
    arm_mat_init_f32(&sf_g, 3, 1, sf_g_data);

    arm_matrix_instance_f32 sf_a;
    float32_t sf_a_data[3] = {0.01f, 0.01f, 0.01f};
    arm_mat_init_f32(&sf_a, 3, 1, sf_a_data);

    arm_matrix_instance_f32 q;
    float32_t q_data[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    arm_mat_init_f32(&q, 4, 1, q_data);

    arm_matrix_instance_f32 G;
    float32_t G_buffer[21 * 12];

    compute_G(&sf_g, &sf_a, &q, &G, G_buffer);

    if (!test_matrix_dimensions(&G, 21, 12))
    {
        print_test_result("test_compute_G_dimensions", false, "Wrong dimensions (expected 21x12)");
        return false;
    }

    print_test_result("test_compute_G_dimensions", true, NULL);
    return true;
}

bool test_compute_G_finite_values(void)
{
    arm_matrix_instance_f32 sf_g;
    float32_t sf_g_data[3] = {0.01f, 0.01f, 0.01f};
    arm_mat_init_f32(&sf_g, 3, 1, sf_g_data);

    arm_matrix_instance_f32 sf_a;
    float32_t sf_a_data[3] = {0.01f, 0.01f, 0.01f};
    arm_mat_init_f32(&sf_a, 3, 1, sf_a_data);

    arm_matrix_instance_f32 q;
    float32_t q_data[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    arm_mat_init_f32(&q, 4, 1, q_data);

    arm_matrix_instance_f32 G;
    float32_t G_buffer[21 * 12];

    compute_G(&sf_g, &sf_a, &q, &G, G_buffer);

    if (!test_matrix_finite(&G))
    {
        print_test_result("test_compute_G_finite_values", false, "Non-finite values in G matrix");
        return false;
    }

    print_test_result("test_compute_G_finite_values", true, NULL);
    return true;
}

bool test_compute_G_structure(void)
{
    arm_matrix_instance_f32 sf_g;
    float32_t sf_g_data[3] = {0.01f, 0.01f, 0.01f};
    arm_mat_init_f32(&sf_g, 3, 1, sf_g_data);

    arm_matrix_instance_f32 sf_a;
    float32_t sf_a_data[3] = {0.01f, 0.01f, 0.01f};
    arm_mat_init_f32(&sf_a, 3, 1, sf_a_data);

    arm_matrix_instance_f32 q;
    float32_t q_data[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    arm_mat_init_f32(&q, 4, 1, q_data);

    arm_matrix_instance_f32 G;
    float32_t G_buffer[21 * 12];

    compute_G(&sf_g, &sf_a, &q, &G, G_buffer);

    // Check that G is not all zeros
    if (!test_matrix_not_all_zero(&G))
    {
        print_test_result("test_compute_G_structure", false, "G matrix is all zeros");
        return false;
    }

    // Check that matrix has correct dimensions
    if (!test_matrix_dimensions(&G, 21, 12))
    {
        print_test_result("test_compute_G_structure", false, "Wrong dimensions");
        return false;
    }

    print_test_result("test_compute_G_structure", true, NULL);
    return true;
}

// i = 25000
bool test_quaternion_to_DCM(void) {

	arm_matrix_instance_f32 q, DCM;
	float32_t qTest[4] = {-0.02337602,  0.91473126,  0.4009951,  -0.04385527};
	float32_t DCMData[9];

	arm_mat_init_f32(&q, 4, 1, qTest);

	quaternion2DCM(&q, &DCM, DCMData);

	arm_matrix_instance_f32 DCMTrue;
	float32_t DCMTrueData[9] = {0.6745594 ,  0.73155516, -0.09897891,  0.73565584, -0.67731297,
	        					0.00759405, -0.06148423, -0.07793704, -0.99506056};

	arm_mat_init_f32(&DCMTrue, 3, 3, DCMTrueData);

	bool test = areMatricesEqual(&DCM, &DCMTrue);
}

bool test_what(void) {

	arm_matrix_instance_f32 xPrev, wMeas, wHatTrue, wHatTest;

	arm_matrix_instance_f32 q, gBias, aBias, g_sf, a_sf;
	float32_t quatBuff[4], gBiasBuff[3], aBiasBuff[3], gSFBias[3], aSFBias[3];

	float32_t xPrevData[22*1] = {-2.2884607315063477e-02,  9.1512638330459595e-01,
	        4.0011137723922729e-01, -4.3943215161561966e-02,
	        3.5394687652587891e+01, -1.1787300109863281e+02,
	        2.8781792968750000e+04, -1.2077078819274902e+01,
	        5.7730107307434082e+00,  1.1333886718750000e+02,
	       -2.7942578890360892e-04, -2.1035106328781694e-04,
	       -2.6591881760396063e-04,  8.8350940495729446e-03,
	        1.6256757080554962e-03,  1.9927009998355061e-04,
	        2.1494920656550676e-04, -1.0350634111091495e-03,
	       -8.9672525064088404e-05,  1.5854457160457969e-03,
	        1.0850373655557632e-03,  4.6325451694428921e-04};

	float32_t wMeasData[3*1] = {0.1148542687296867,  0.0044058579951525, -0.0044308393262327};

	float32_t wHatDataTrue[3*1] = {0.1150641068816185,  0.0045749028213322, -0.0042020138353109};

	arm_mat_init_f32(&xPrev, 22, 1, xPrevData);
	arm_mat_init_f32(&wMeas, 3, 1, wMeasData);
	arm_mat_init_f32(&wHatTrue, 3, 1, wHatDataTrue);

	printMatrix(&xPrev);
	printMatrix(&wMeas);
	printMatrix(&wHatTrue);

	getStateQuaternion(&xPrev, &q, quatBuff);
	printMatrix(&q);

	getStateGBias(&xPrev, &gBias, gBiasBuff);
	printMatrix(&gBias);

	getStateABias(&xPrev, &aBias, aBiasBuff);
	printMatrix(&aBias);

	getStateGSF(&xPrev, &g_sf, gSFBias);
	printMatrix(&g_sf);

	getStateASF(&xPrev, &a_sf, aSFBias);
	printMatrix(&a_sf);

	float32_t phi = xPrev.pData[4];
	float32_t h = xPrev.pData[6];
	float32_t vn = xPrev.pData[7];
	float32_t ve = xPrev.pData[8];

	float32_t wHatDataTest[3];

	compute_what(&q, &gBias, &g_sf, phi, h, vn, ve, we, &wMeas, &wHatTest, wHatDataTest);

	printMatrix(&wHatTest);

	bool test = false;
	test = areMatricesEqual(&wHatTest, &wHatTrue);
}

bool test_ahat(void) {

	arm_matrix_instance_f32 xPrev, aMeas, aMeasTrue, aHatTest, aHatTrue;

	arm_matrix_instance_f32 q, gBias, aBias, g_sf, a_sf;
	float32_t quatBuff[4], gBiasBuff[3], aBiasBuff[3], gSFBias[3], aSFBias[3];

	float32_t xPrevData[22*1] = {-2.2884607315063477e-02,  9.1512638330459595e-01,
	        4.0011137723922729e-01, -4.3943215161561966e-02,
	        3.5394687652587891e+01, -1.1787300109863281e+02,
	        2.8781792968750000e+04, -1.2077078819274902e+01,
	        5.7730107307434082e+00,  1.1333886718750000e+02,
	       -2.7942578890360892e-04, -2.1035106328781694e-04,
	       -2.6591881760396063e-04,  8.8350940495729446e-03,
	        1.6256757080554962e-03,  1.9927009998355061e-04,
	        2.1494920656550676e-04, -1.0350634111091495e-03,
	       -8.9672525064088404e-05,  1.5854457160457969e-03,
	        1.0850373655557632e-03,  4.6325451694428921e-04};

	float32_t aMeasData[3*1] = {0.4731085002422333,  0.9613523483276367, 10.812639236450195};

	float32_t aHatDataTrue[3*1] = {-0.0536695718765259,  -0.237719401717186 , -10.857034683227539};

	arm_mat_init_f32(&xPrev, 22, 1, xPrevData);
	arm_mat_init_f32(&aMeas, 3, 1, aMeasData);
	arm_mat_init_f32(&aHatTrue, 3, 1, aHatDataTrue);

	printMatrix(&xPrev);
	printMatrix(&aMeas);

	getStateQuaternion(&xPrev, &q, quatBuff);
	printMatrix(&q);

	getStateGBias(&xPrev, &gBias, gBiasBuff);
	printMatrix(&gBias);

	getStateABias(&xPrev, &aBias, aBiasBuff);
	printMatrix(&aBias);

	getStateGSF(&xPrev, &g_sf, gSFBias);
	printMatrix(&g_sf);

	getStateASF(&xPrev, &a_sf, aSFBias);
	printMatrix(&a_sf);

	float32_t aHatDataTest[3];
	compute_ahat(&q, &a_sf, &aBias, &aMeas, &aHatTest, aHatDataTest);

	bool test = false;

	printMatrix(&aHatTest);
	test = areMatricesEqual(&aHatTest, &aHatTrue);
}

bool test_qdot() {

	arm_matrix_instance_f32 xPrev, wHat, qDotTest, qDotTrue;

	arm_matrix_instance_f32 q, gBias, aBias, g_sf, a_sf;
	float32_t quatBuff[4], gBiasBuff[3], aBiasBuff[3], gSFBias[3], aSFBias[3], qDotTestData[4];

	float32_t xPrevData[22*1] = {-2.2884607315063477e-02,  9.1512638330459595e-01,
	        4.0011137723922729e-01, -4.3943215161561966e-02,
	        3.5394687652587891e+01, -1.1787300109863281e+02,
	        2.8781792968750000e+04, -1.2077078819274902e+01,
	        5.7730107307434082e+00,  1.1333886718750000e+02,
	       -2.7942578890360892e-04, -2.1035106328781694e-04,
	       -2.6591881760396063e-04,  8.8350940495729446e-03,
	        1.6256757080554962e-03,  1.9927009998355061e-04,
	        2.1494920656550676e-04, -1.0350634111091495e-03,
	       -8.9672525064088404e-05,  1.5854457160457969e-03,
	        1.0850373655557632e-03,  4.6325451694428921e-04};

	float32_t wHatData[3*1] = { 0.1150641068816185,  0.0045749028213322, -0.0042020138353109};
	float32_t qDotTrueData[4*1] = {-0.0536566600203514, -0.0020567174069583, -0.0006578038446605,
		       -0.0208778418600559};

	arm_mat_init_f32(&xPrev, 22, 1, xPrevData);
	arm_mat_init_f32(&wHat, 3, 1, wHatData);

	printMatrix(&xPrev);
	printMatrix(&wHat);

	getStateQuaternion(&xPrev, &q, quatBuff);

	getStateGBias(&xPrev, &gBias, gBiasBuff);

	getStateABias(&xPrev, &aBias, aBiasBuff);

	getStateGSF(&xPrev, &g_sf, gSFBias);

	getStateASF(&xPrev, &a_sf, aSFBias);

	arm_mat_init_f32(&qDotTrue, 4, 1, qDotTrueData);
	compute_qdot(&q, &wHat, &qDotTest, qDotTestData);

	printf("Q Dot True\n");
	printMatrix(&qDotTrue);

	printf("Q Dot Test\n");
	printMatrix(&qDotTest);

	bool test1 = false;
	test1 = areMatricesEqual(&qDotTest, &qDotTrue);
	return true;
}

bool test_lla_dot(void) {

	float32_t xPrevData[22*1] = {-2.3083120584487915e-02,  9.1496688127517700e-01,
	        4.0046840906143188e-01, -4.3907660990953445e-02,
	        3.5394672393798828e+01, -1.1787238311767578e+02,
	        2.8782597656250000e+04, -1.2230652809143066e+01,
	        6.2059984207153320e+00,  1.1328311157226562e+02,
	       -2.7709139976650476e-04, -2.1110560919623822e-04,
	       -2.6525690918788314e-04,  8.8304430246353149e-03,
	        1.7384933307766914e-03,  2.6933639310300350e-04,
	        1.9250490004196763e-04, -9.9813647102564573e-04,
	       -2.6455882471054792e-04,  1.5813577920198441e-03,
	        9.9914241582155228e-04,  3.9185321656987071e-04};

	arm_matrix_instance_f32 llaDotTest, llaDotTrue, xPrev;
	float32_t llaDotTestData[3];

	float32_t llaDotTrueData[3] = {-1.0974099859595299e-04,  6.8006738729309291e-05,
		       -1.1328311157226562e+02};

	arm_mat_init_f32(&xPrev, 22, 1, xPrevData);

	float32_t phi = xPrev.pData[4];
	float32_t h = xPrev.pData[6];
	float32_t vn = xPrev.pData[7];
	float32_t ve = xPrev.pData[8];
	float32_t vd = xPrev.pData[9];

	arm_mat_init_f32(&llaDotTrue, 3, 1, llaDotTrueData);

	compute_lla_dot(phi, h, vn, ve, vd, &llaDotTest, llaDotTestData);

	bool test1 = false;
	test1 = areMatricesEqual(&llaDotTest, &llaDotTrue);
}

bool test_compute_vdot(void) {

	float32_t xPrevData[22*1] = {-2.2884607315063477e-02,  9.1512638330459595e-01,
	        4.0011137723922729e-01, -4.3943215161561966e-02,
	        3.5394687652587891e+01, -1.1787300109863281e+02,
	        2.8781792968750000e+04, -1.2077078819274902e+01,
	        5.7730107307434082e+00,  1.1333886718750000e+02,
	       -2.7942578890360892e-04, -2.1035106328781694e-04,
	       -2.6591881760396063e-04,  8.8350940495729446e-03,
	        1.6256757080554962e-03,  1.9927009998355061e-04,
	        2.1494920656550676e-04, -1.0350634111091495e-03,
	       -8.9672525064088404e-05,  1.5854457160457969e-03,
	        1.0850373655557632e-03,  4.6325451694428921e-04};

	float32_t ahatNData[3*1] = {-0.0536695718765259,  -0.237719401717186 , -10.857034683227539};

	arm_matrix_instance_f32 xPrev, vDotTest;
	float32_t vDotTestData[3];

	arm_mat_init_f32(&xPrev, 22, 1, xPrevData);

	float32_t phi = xPrev.pData[4];
	float32_t h = xPrev.pData[6];
	float32_t vn = xPrev.pData[7];
	float32_t ve = xPrev.pData[8];
	float32_t vd = xPrev.pData[9];

	/* vDot =
	 *  [-0.0002920179
         0.01316535
         -9.996122]
	 */

	compute_vdot(phi, h, vn, ve, vd, ahatNData, we, &vDotTest, vDotTestData);

	arm_matrix_instance_f32 vDotTrue;
	float32_t vDotTrueData[] = {-0.054375272244215 , -0.2251708954572678, -1.1488429307937622};
	arm_mat_init_f32(&vDotTrue, 3, 1, vDotTrueData);

	bool test = false;
	test = areMatricesEqual(&vDotTrue, &vDotTest);
	return true;
}

// i = 201
bool test_compute_dwdp(void) {

	arm_matrix_instance_f32 xPrev;

	float32_t xPrevData[22*1] = {-2.2884607315063477e-02,  9.1512638330459595e-01,
	        4.0011137723922729e-01, -4.3943215161561966e-02,
	        3.5394687652587891e+01, -1.1787300109863281e+02,
	        2.8781792968750000e+04, -1.2077078819274902e+01,
	        5.7730107307434082e+00,  1.1333886718750000e+02,
	       -2.7942578890360892e-04, -2.1035106328781694e-04,
	       -2.6591881760396063e-04,  8.8350940495729446e-03,
	        1.6256757080554962e-03,  1.9927009998355061e-04,
	        2.1494920656550676e-04, -1.0350634111091495e-03,
	       -8.9672525064088404e-05,  1.5854457160457969e-03,
	        1.0850373655557632e-03,  4.6325451694428921e-04};

	float32_t aMeasData[3] = {0.4731085002422333,  0.9613523483276367, 10.812639236450195};

	float32_t wMeasData[3] = {0.1148542687296867,  0.0044058579951525, -0.0044308393262327};

	arm_mat_init_f32(&xPrev, 22, 1, xPrevData);

	float32_t phi = xPrev.pData[4];
	float32_t h = xPrev.pData[6];
	float32_t vn = xPrev.pData[7];
	float32_t ve = xPrev.pData[8];

	arm_matrix_instance_f32 dwdpTest;
	float32_t dwdpTestData[9];

	compute_dwdp(phi, h, ve, vn, we, &dwdpTest, dwdpTestData);

	arm_matrix_instance_f32 dwdpTrue;
	float32_t dwdpTrueData[9] = {-4.2238429159624502e-05,  0.0000000000000000e+00,
		       -1.4032396392863605e-13, -1.7892485715265138e-08,
		        0.0000000000000000e+00, -2.9617969318426751e-13,
		       -6.0795398894697428e-05,  0.0000000000000000e+00,
		        9.9703455398465063e-14};

	arm_mat_init_f32(&dwdpTrue, 3, 3, dwdpTrueData);

	bool test1 = false;
	test1 = areMatricesEqual(&dwdpTest, &dwdpTrue);
	return true;
}

bool test_compute_dwdv(void) {

	arm_matrix_instance_f32 xPrev;

	float32_t xPrevData[22*1] = {-2.2884607315063477e-02,  9.1512638330459595e-01,
	        4.0011137723922729e-01, -4.3943215161561966e-02,
	        3.5394687652587891e+01, -1.1787300109863281e+02,
	        2.8781792968750000e+04, -1.2077078819274902e+01,
	        5.7730107307434082e+00,  1.1333886718750000e+02,
	       -2.7942578890360892e-04, -2.1035106328781694e-04,
	       -2.6591881760396063e-04,  8.8350940495729446e-03,
	        1.6256757080554962e-03,  1.9927009998355061e-04,
	        2.1494920656550676e-04, -1.0350634111091495e-03,
	       -8.9672525064088404e-05,  1.5854457160457969e-03,
	        1.0850373655557632e-03,  4.6325451694428921e-04};

	arm_mat_init_f32(&xPrev, 22, 1, xPrevData);

	float32_t phi = xPrev.pData[4];
	float32_t h = xPrev.pData[6];

	arm_matrix_instance_f32 dwdvTest;
	float32_t dwdvTestData[9];

	compute_dwdv(phi, h, &dwdvTest, dwdvTestData);

	arm_matrix_instance_f32 dwdvTrue;
	float32_t dwdvTrueData[9] = {0.0000000000000000e+00,  1.5590669022458314e-07,
	        0.0000000000000000e+00, -1.5660178576126782e-07,
	        0.0000000000000000e+00,  0.0000000000000000e+00,
	        0.0000000000000000e+00, -1.1077533912384752e-07,
	        0.0000000000000000e+00};

	arm_mat_init_f32(&dwdvTrue, 3, 3, dwdvTrueData);

	bool test = false;
	test = areMatricesEqual(&dwdvTrue, &dwdvTest);

	return true;
}

bool test_compute_dpdot_dp(void) {

	arm_matrix_instance_f32 xPrev;

	float32_t xPrevData[22*1] = {-2.2884607315063477e-02,  9.1512638330459595e-01,
	        4.0011137723922729e-01, -4.3943215161561966e-02,
	        3.5394687652587891e+01, -1.1787300109863281e+02,
	        2.8781792968750000e+04, -1.2077078819274902e+01,
	        5.7730107307434082e+00,  1.1333886718750000e+02,
	       -2.7942578890360892e-04, -2.1035106328781694e-04,
	       -2.6591881760396063e-04,  8.8350940495729446e-03,
	        1.6256757080554962e-03,  1.9927009998355061e-04,
	        2.1494920656550676e-04, -1.0350634111091495e-03,
	       -8.9672525064088404e-05,  1.5854457160457969e-03,
	        1.0850373655557632e-03,  4.6325451694428921e-04};

	arm_mat_init_f32(&xPrev, 22, 1, xPrevData);

	float32_t phi = xPrev.pData[4];
	float32_t h = xPrev.pData[6];
	float32_t vn = xPrev.pData[7];
	float32_t ve = xPrev.pData[8];

	arm_matrix_instance_f32 dpdot_dpTest, dpdot_dpTrue;
	float32_t dpDotTestData[9];
	float32_t dpDotTrueData[] = {1.7892485715265138e-08,  0.0000000000000000e+00,
	        1.6969845667569317e-11,  7.8102800671331352e-07,
	        0.0000000000000000e+00, -9.8629654790571841e-12,
	        0.0000000000000000e+00,  0.0000000000000000e+00,
	        0.0000000000000000e+00};

	arm_mat_init_f32(&dpdot_dpTrue, 3, 3, dpDotTrueData);

	compute_dpdot_dp(phi, h, vn, ve, &dpdot_dpTest, dpDotTestData);

	bool test = false;
	test = areMatricesEqual(&dpdot_dpTrue, &dpdot_dpTest);
	return true;
}

bool test_compute_dpdot_dv(void) {

	arm_matrix_instance_f32 xPrev;

	float32_t xPrevData[22*1] = {-2.2884607315063477e-02,  9.1512638330459595e-01,
	        4.0011137723922729e-01, -4.3943215161561966e-02,
	        3.5394687652587891e+01, -1.1787300109863281e+02,
	        2.8781792968750000e+04, -1.2077078819274902e+01,
	        5.7730107307434082e+00,  1.1333886718750000e+02,
	       -2.7942578890360892e-04, -2.1035106328781694e-04,
	       -2.6591881760396063e-04,  8.8350940495729446e-03,
	        1.6256757080554962e-03,  1.9927009998355061e-04,
	        2.1494920656550676e-04, -1.0350634111091495e-03,
	       -8.9672525064088404e-05,  1.5854457160457969e-03,
	        1.0850373655557632e-03,  4.6325451694428921e-04};

	arm_mat_init_f32(&xPrev, 22, 1, xPrevData);

	float32_t phi = xPrev.pData[4];
	float32_t h = xPrev.pData[6];

	arm_matrix_instance_f32 dpdot_dvTest, dpdot_dvTrue;
	float32_t dpDotTestData[9];
	float32_t dpDotTrueData[] = {8.972620889835525e-06,  0.000000000000000e+00,
	        0.000000000000000e+00,  0.000000000000000e+00,
	        1.095822881325148e-05,  0.000000000000000e+00,
	        0.000000000000000e+00,  0.000000000000000e+00,
	       -1.000000000000000e+00};

	compute_dpdot_dv(phi, h, &dpdot_dvTest, dpDotTestData);

	arm_mat_init_f32(&dpdot_dvTrue, 3, 3, dpDotTrueData);

	bool test = false;
	test = areMatricesEqual(&dpdot_dvTrue, &dpdot_dvTest);

	return true;
}

bool test_compute_dvdot_dp(void) {

	arm_matrix_instance_f32 xPrev;

	float32_t xPrevData[22*1] = {-2.2884607315063477e-02,  9.1512638330459595e-01,
	        4.0011137723922729e-01, -4.3943215161561966e-02,
	        3.5394687652587891e+01, -1.1787300109863281e+02,
	        2.8781792968750000e+04, -1.2077078819274902e+01,
	        5.7730107307434082e+00,  1.1333886718750000e+02,
	       -2.7942578890360892e-04, -2.1035106328781694e-04,
	       -2.6591881760396063e-04,  8.8350940495729446e-03,
	        1.6256757080554962e-03,  1.9927009998355061e-04,
	        2.1494920656550676e-04, -1.0350634111091495e-03,
	       -8.9672525064088404e-05,  1.5854457160457969e-03,
	        1.0850373655557632e-03,  4.6325451694428921e-04};

	arm_mat_init_f32(&xPrev, 22, 1, xPrevData);

	float32_t phi = xPrev.pData[4];
	float32_t h = xPrev.pData[6];
	float32_t vn = xPrev.pData[7];
	float32_t ve = xPrev.pData[8];
	float32_t vd = xPrev.pData[9];

	arm_matrix_instance_f32 dpdot_dvTest, dpdot_dvTrue;
	float32_t dpDotTestData[9];
	float32_t dpDotTrueData[] = {-6.9210928631946445e-04,  0.0000000000000000e+00,
	        3.4144260335766674e-11, -1.1026318185031414e-02,
	        0.0000000000000000e+00, -1.4700032857639656e-11,
	        4.9507610499858856e-02,  0.0000000000000000e+00,
	       -3.0820749543636339e-06};

	compute_dvdot_dp(phi, h, vn, ve, vd, we, &dpdot_dvTest, dpDotTestData);

	arm_mat_init_f32(&dpdot_dvTrue, 3, 3, dpDotTrueData);

	bool test = false;
	test = areMatricesEqual(&dpdot_dvTrue, &dpdot_dvTest);

	return true;
}

bool test_compute_dvdot_dv(void) {

	arm_matrix_instance_f32 xPrev;

	float32_t xPrevData[22*1] = {-2.2884607315063477e-02,  9.1512638330459595e-01,
	        4.0011137723922729e-01, -4.3943215161561966e-02,
	        3.5394687652587891e+01, -1.1787300109863281e+02,
	        2.8781792968750000e+04, -1.2077078819274902e+01,
	        5.7730107307434082e+00,  1.1333886718750000e+02,
	       -2.7942578890360892e-04, -2.1035106328781694e-04,
	       -2.6591881760396063e-04,  8.8350940495729446e-03,
	        1.6256757080554962e-03,  1.9927009998355061e-04,
	        2.1494920656550676e-04, -1.0350634111091495e-03,
	       -8.9672525064088404e-05,  1.5854457160457969e-03,
	        1.0850373655557632e-03,  4.6325451694428921e-04};

	arm_mat_init_f32(&xPrev, 22, 1, xPrevData);

	float32_t phi = xPrev.pData[4];
	float32_t h = xPrev.pData[6];
	float32_t vn = xPrev.pData[7];
	float32_t ve = xPrev.pData[8];
	float32_t vd = xPrev.pData[9];

	arm_matrix_instance_f32 dvdot_dvTest, dvdot_dvTrue;
	float32_t dvDotTestData[9];
	float32_t dvDotTrueData[] = {1.7749067410477437e-05,  8.3192171587143093e-05,
		       -1.8912920722868876e-06,  8.5110688814893365e-05,
		        1.6332445738953538e-05,  1.1978591646766290e-04,
		        3.7825841445737751e-06, -1.2068596697645262e-04,
		        0.0000000000000000e+00};

//  Expected Value (i == 201)
//
//	   -8.376576e-06    8.437293e-05   -3.322961e-09
//	    8.437677e-05   -8.341535e-06    0.0001189586
//	    6.645922e-09   -0.0001189604               0

	compute_dvdot_dv(phi, h, vn, ve, vd, we, &dvdot_dvTest, dvDotTestData);
	arm_mat_init_f32(&dvdot_dvTest, 3, 3, dvDotTrueData);

	bool test = false;
	test = areMatricesEqual(&dvdot_dvTrue, &dvdot_dvTest);

	return true;
}

bool test_compute_F(void) {

	arm_matrix_instance_f32 xPrev, aMeas, wMeas;

	float32_t xPrevData[22*1] = {-2.2884607315063477e-02,  9.1512638330459595e-01,
	        4.0011137723922729e-01, -4.3943215161561966e-02,
	        3.5394687652587891e+01, -1.1787300109863281e+02,
	        2.8781792968750000e+04, -1.2077078819274902e+01,
	        5.7730107307434082e+00,  1.1333886718750000e+02,
	       -2.7942578890360892e-04, -2.1035106328781694e-04,
	       -2.6591881760396063e-04,  8.8350940495729446e-03,
	        1.6256757080554962e-03,  1.9927009998355061e-04,
	        2.1494920656550676e-04, -1.0350634111091495e-03,
	       -8.9672525064088404e-05,  1.5854457160457969e-03,
	        1.0850373655557632e-03,  4.6325451694428921e-04};

	float32_t aMeasData[3] = {0.4731085002422333,  0.9613523483276367, 10.812639236450195};

	float32_t wMeasData[3] = {0.1148542687296867,  0.0044058579951525, -0.0044308393262327};

	arm_mat_init_f32(&xPrev, 22, 1, xPrevData);
	arm_mat_init_f32(&aMeas, 3, 1, aMeasData);
	arm_mat_init_f32(&wMeas, 3, 1, wMeasData);

	arm_matrix_instance_f32 q, gBias, aBias, g_sf, a_sf, FTest;
	float32_t quatBuff[4], gBiasBuff[3], aBiasBuff[3], gSFBias[3], aSFBias[3];
	float32_t FTestData[21*21] = {0};

	getStateQuaternion(&xPrev, &q, quatBuff);
	printMatrix(&q);

	getStateGBias(&xPrev, &gBias, gBiasBuff);
	printMatrix(&gBias);

	getStateABias(&xPrev, &aBias, aBiasBuff);
	printMatrix(&aBias);

	getStateGSF(&xPrev, &g_sf, gSFBias);
	printMatrix(&g_sf);

	getStateASF(&xPrev, &a_sf, aSFBias);
	printMatrix(&a_sf);

	float32_t phi = xPrev.pData[4];
	float32_t h = xPrev.pData[6];
	float32_t vn = xPrev.pData[7];
	float32_t ve = xPrev.pData[8];
	float32_t vd = xPrev.pData[9];

	compute_F(&q, &a_sf, &g_sf, &gBias, &aBias, phi, h, vn, ve, vd, &aMeas, &wMeas, we, &FTest, FTestData);

	arm_matrix_instance_f32 FTrue;
	float32_t FDataTrue[21*21] = {0.0000000000000000e+00, -4.1652941145002842e-03,
		       -4.6209916472434998e-03,  2.4788349037407897e-05,
		       -0.0000000000000000e+00,  3.1853591487586430e-13,
		        1.1499522400981732e-07, -1.1226740781467015e-07,
		       -0.0000000000000000e+00, -9.9978512525558472e-01,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00, -1.1513369530439377e-01,
		       -0.0000000000000000e+00, -0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  4.1652941145002842e-03,
		        0.0000000000000000e+00,  1.1510895937681198e-01,
		        2.6150091798626818e-05, -0.0000000000000000e+00,
		       -9.0879465953520866e-14, -1.0629729274569399e-07,
		       -1.2239280522408080e-07, -0.0000000000000000e+00,
		        0.0000000000000000e+00, -1.0010360479354858e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		       -0.0000000000000000e+00, -4.6162088401615620e-03,
		       -0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        4.6209916472434998e-03, -1.1510895937681198e-01,
		        0.0000000000000000e+00, -6.4667437982279807e-05,
		       -0.0000000000000000e+00,  8.7348802236403289e-14,
		        1.0524040527926104e-09, -9.4837290021132503e-08,
		       -0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00, -1.0000896453857422e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00, -0.0000000000000000e+00,
		       -0.0000000000000000e+00,  4.1649206541478634e-03,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        1.7892485715265138e-08,  0.0000000000000000e+00,
		        1.6969845667569317e-11,  8.9726208898355253e-06,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  7.8102800671331352e-07,
		        0.0000000000000000e+00, -9.8629654790571841e-12,
		        0.0000000000000000e+00,  1.0958228813251480e-05,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00, -1.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		       -7.9872636795043945e+00,  7.3511629104614258e+00,
		       -3.0951437354087830e-01, -6.9210928631946445e-04,
		        0.0000000000000000e+00,  3.4144260335766674e-11,
		        1.7749067410477437e-05,  8.3192171587143093e-05,
		       -1.8912920722868876e-06,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		       -6.7488980293273926e-01, -7.2950214147567749e-01,
		        9.8694257438182831e-02,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		       -3.1383016705513000e-01, -7.0088237524032593e-01,
		        1.0676200389862061e+00,  7.3422551155090332e+00,
		        7.9329605102539062e+00, -1.0186172723770142e+00,
		       -1.1026318185031414e-02,  0.0000000000000000e+00,
		       -1.4700032857639656e-11,  8.5110688814893365e-05,
		        1.6332445738953538e-05,  1.1978591646766290e-04,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00, -7.3315376043319702e-01,
		        6.7803877592086792e-01, -6.7171445116400719e-03,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00, -3.4092345833778381e-01,
		        6.5143799781799316e-01, -7.2662368416786194e-02,
		       -1.2127828598022461e-01, -2.1003460884094238e-01,
		        2.3833084851503372e-02,  4.9507610499858856e-02,
		        0.0000000000000000e+00, -3.0820749543636339e-06,
		        3.7825841445737751e-06, -1.2068596697645262e-04,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        6.2016084790229797e-02,  7.6965466141700745e-02,
		        9.9462997913360596e-01,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        2.8838068246841431e-02,  7.3945954442024231e-02,
		        1.0759358406066895e+01,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00,  0.0000000000000000e+00,
		        0.0000000000000000e+00};

	arm_mat_init_f32(&FTrue, 21, 21, FDataTrue);

	bool test = false;
	test = areMatricesEqual(&FTrue, &FTest);

	return true;
}

bool test_compute_G(void) {

	arm_matrix_instance_f32 xPrev, aMeas, wMeas;

	float32_t xPrevData[22*1] = {-2.2884607315063477e-02,  9.1512638330459595e-01,
	        4.0011137723922729e-01, -4.3943215161561966e-02,
	        3.5394687652587891e+01, -1.1787300109863281e+02,
	        2.8781792968750000e+04, -1.2077078819274902e+01,
	        5.7730107307434082e+00,  1.1333886718750000e+02,
	       -2.7942578890360892e-04, -2.1035106328781694e-04,
	       -2.6591881760396063e-04,  8.8350940495729446e-03,
	        1.6256757080554962e-03,  1.9927009998355061e-04,
	        2.1494920656550676e-04, -1.0350634111091495e-03,
	       -8.9672525064088404e-05,  1.5854457160457969e-03,
	        1.0850373655557632e-03,  4.6325451694428921e-04};


	arm_mat_init_f32(&xPrev, 22, 1, xPrevData);

	arm_matrix_instance_f32 q, gBias, aBias, g_sf, a_sf;
	float32_t quatBuff[4], gBiasBuff[3], aBiasBuff[3], gSFBias[3], aSFBias[3];

	getStateQuaternion(&xPrev, &q, quatBuff);
	printMatrix(&q);

	getStateGSF(&xPrev, &g_sf, gSFBias);
	printMatrix(&g_sf);

	getStateASF(&xPrev, &a_sf, aSFBias);
	printMatrix(&a_sf);

	arm_matrix_instance_f32 G;
	float32_t GBuff[21*12];

	compute_G(&g_sf, &a_sf, &q, &G, GBuff);

	arm_matrix_instance_f32 GTrue;
	float32_t GTrueData[21*12] = {-9.9978512525558472e-01f, -0.0000000000000000e+00f, -0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
			-0.0000000000000000e+00f, -1.0010360479354858e+00f, -0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
			-0.0000000000000000e+00f, -0.0000000000000000e+00f, -1.0000896453857422e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
			0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
			0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
			0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
			0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, -6.7488980293273926e-01f, -7.2950214147567749e-01f, 9.8694257438182831e-02f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
			0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, -7.3315376043319702e-01f, 6.7803877592086792e-01f, -6.7171445116400719e-03f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
			0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 6.2016084790229797e-02f, 7.6965466141700745e-02f, 9.9462997913360596e-01f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
			0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 1.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
			0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 1.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
			0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 1.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
			0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
			0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
			0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
			0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 1.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
			0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 1.0000000000000000e+00f, 0.0000000000000000e+00f,
			0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 1.0000000000000000e+00f,
			0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
			0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
			0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f};

	arm_mat_init_f32(&GTrue, 21, 12, GTrueData);

	bool test = false;
	test = areMatricesEqual(&GTrue, &G);

	return true;
}

bool test_compute_Pdot() {

	arm_matrix_instance_f32 xPrev, aMeas, wMeas;

	float32_t xPrevData[22*1] = {-2.2884607315063477e-02,  9.1512638330459595e-01,
	        4.0011137723922729e-01, -4.3943215161561966e-02,
	        3.5394687652587891e+01, -1.1787300109863281e+02,
	        2.8781792968750000e+04, -1.2077078819274902e+01,
	        5.7730107307434082e+00,  1.1333886718750000e+02,
	       -2.7942578890360892e-04, -2.1035106328781694e-04,
	       -2.6591881760396063e-04,  8.8350940495729446e-03,
	        1.6256757080554962e-03,  1.9927009998355061e-04,
	        2.1494920656550676e-04, -1.0350634111091495e-03,
	       -8.9672525064088404e-05,  1.5854457160457969e-03,
	        1.0850373655557632e-03,  4.6325451694428921e-04};

	float32_t aMeasData[3] = {0.4731085002422333,  0.9613523483276367, 10.812639236450195};

	float32_t wMeasData[3] = {0.1148542687296867,  0.0044058579951525, -0.0044308393262327};

	arm_mat_init_f32(&xPrev, 22, 1, xPrevData);
	arm_mat_init_f32(&aMeas, 3, 1, aMeasData);
	arm_mat_init_f32(&wMeas, 3, 1, wMeasData);

	arm_matrix_instance_f32 q, gBias, aBias, g_sf, a_sf, F;
	float32_t quatBuff[4], gBiasBuff[3], aBiasBuff[3], gSFBias[3], aSFBias[3], FBuff[21*21];

	getStateQuaternion(&xPrev, &q, quatBuff);
	printMatrix(&q);

	getStateGBias(&xPrev, &gBias, gBiasBuff);
	printMatrix(&gBias);

	getStateABias(&xPrev, &aBias, aBiasBuff);
	printMatrix(&aBias);

	getStateGSF(&xPrev, &g_sf, gSFBias);
	printMatrix(&g_sf);

	getStateASF(&xPrev, &a_sf, aSFBias);
	printMatrix(&a_sf);

	float32_t phi = xPrev.pData[4];
	float32_t h = xPrev.pData[6];
	float32_t vn = xPrev.pData[7];
	float32_t ve = xPrev.pData[8];
	float32_t vd = xPrev.pData[9];

	float32_t PBuff[21*21] = {5.0389925017952919e-03f, 3.1386734917759895e-03f, -1.2951527722179890e-02f, -8.0738391261547804e-04f, 2.5598644278943539e-03f, -2.1820283889770508e+01f, -1.5453934669494629e+00f, 5.9288525581359863e+00f, 1.3891488313674927e-01f, -2.0048216811119346e-06f, -3.2393774745287374e-05f, 4.9889800720848143e-05f, -1.5646855899831280e-05f, 3.2121343451763096e-07f, -1.5264936337189283e-06f, -2.1615074365399778e-04f, -7.4237369699403644e-04f, -7.4153754394501448e-04f, -1.0093436912939069e-06f, -1.1073028645114391e-06f, -6.2298909142555203e-07f,
			3.1386725604534149e-03f, 1.9648219458758831e-03f, -8.0804517492651939e-03f, -5.0261052092537284e-04f, 1.5936470590531826e-03f, -1.3582573890686035e+01f, -9.6213459968566895e-01f, 3.6917583942413330e+00f, 8.6432509124279022e-02f, -5.9811674191223574e-07f, -2.2610181986237876e-05f, 3.1977098842617124e-05f, -9.7411066235508770e-06f, 2.0000864253688633e-07f, -9.5021465540412464e-07f, -1.3153281179256737e-04f, -4.6551285777240992e-04f, -4.6530558029189706e-04f, -6.2809255041429424e-07f, -6.8936219577153679e-07f, -3.8803003121756774e-07f,
			-1.2951526790857315e-02f, -8.0804536119103432e-03f, 3.3347148448228836e-02f, 2.0787562243640423e-03f, -6.5908450633287430e-03f, 5.6180130004882812e+01f, 3.9788146018981934e+00f, -1.5264608383178711e+01f, -3.5766848921775818e-01f, 1.3322169252205640e-06f, 8.3474267739802599e-05f, -1.2983183842152357e-04f, 4.0285089198732749e-05f, -8.2711824234138476e-07f, 3.9301376091316342e-06f, 5.3943332750350237e-04f, 1.9106010440737009e-03f, 1.9155526533722878e-03f, 2.5984440981119405e-06f, 2.8512238259281730e-06f, 1.6040754644564004e-06f,
			-8.0738525139167905e-04f, -5.0261139404028654e-04f, 2.0787597168236971e-03f, 1.7619828577153385e-04f, -5.2187195979058743e-04f, 4.7090721130371094e+00f, 3.2039403915405273e-01f, -1.1370785236358643e+00f, -3.0653338879346848e-02f, -3.4618537938513327e-06f, -1.5313280243844929e-07f, -4.3224565615673782e-07f, 1.7139997225967818e-06f, 3.9339300883511896e-07f, -2.5401027414773125e-06f, 3.3012915082508698e-05f, 8.6762891442049295e-05f, 6.8411834945436567e-05f, 1.4002503689880541e-07f, 8.4199955381336622e-06f, 8.1725174823077396e-06f,
			2.5598646607249975e-03f, 1.5936469426378608e-03f, -6.5908445976674557e-03f, -5.2187172695994377e-04f, 1.6188059234991670e-03f, -1.4437356948852539e+01f, -9.6182018518447876e-01f, 3.5346310138702393e+00f, 1.0662219673395157e-01f, 9.6007197498693131e-06f, 3.1550774792776792e-07f, 1.9050106629947550e-06f, -1.5928870197967626e-05f, -1.2149911299275118e-06f, -4.5260462684382219e-06f, -9.4931870989967138e-05f, -2.9946854920126498e-04f, -2.4497826234437525e-04f, -1.7763567257134127e-06f, 1.0535148931012372e-06f, 2.5394670046807732e-06f,
			-2.1820224761962891e+01f, -1.3582537651062012e+01f, 5.6179962158203125e+01f, 4.7090630531311035e+00f, -1.4437336921691895e+01f, 2.3969992187500000e+05f, 8.5390068359375000e+03f, -3.1784367187500000e+04f, -3.0329980468750000e+03f, -1.0520824044942856e-01f, -6.2867989763617516e-03f, -6.0234428383409977e-03f, -1.2287986278533936e+00f, -2.1592376753687859e-02f, 2.8575158212333918e-03f, 9.7096240520477295e-01f, 2.1565728187561035e+00f, 1.5645617246627808e+00f, 3.3647991716861725e-02f, -1.8120876550674438e+00f, -1.8054577112197876e+00f,
			-1.5453902482986450e+00f, -9.6213251352310181e-01f, 3.9788067340850830e+00f, 3.2039386034011841e-01f, -9.6181970834732056e-01f, 8.5390214843750000e+03f, 5.9188671875000000e+02f, -2.1271850585937500e+03f, -5.3193809509277344e+01f, -6.0443156398832798e-03f, 2.8295663651078939e-04f, -1.5286422567442060e-03f, 2.2926249075680971e-03f, 1.8699679640121758e-04f, -1.1160597205162048e-03f, 6.1143510043621063e-02f, 1.7335088551044464e-01f, 1.4149878919124603e-01f, 3.1555970781482756e-04f, 1.7744706943631172e-02f, 1.4385745860636234e-02f,
			5.9288649559020996e+00f, 3.6917665004730225e+00f, -1.5264640808105469e+01f, -1.1370774507522583e+00f, 3.5346331596374512e+00f, -3.1784408203125000e+04f, -2.1271867675781250e+03f, 7.9166689453125000e+03f, 2.2918943786621094e+02f, 1.9937748089432716e-02f, -2.8586506377905607e-03f, 8.6001874879002571e-03f, -2.6711093261837959e-02f, 7.2020280640572309e-05f, -3.2150780316442251e-03f, -2.1821407973766327e-01f, -7.1389967203140259e-01f, -6.0691767930984497e-01f, -1.8647768301889300e-03f, 5.1091467030346394e-03f, 9.4506256282329559e-03f,
			1.3891413807868958e-01f, 8.6432047188282013e-02f, -3.5766661167144775e-01f, -3.0653275549411774e-02f, 1.0662207007408142e-01f, -3.0329960937500000e+03f, -5.3193630218505859e+01f, 2.2918931579589844e+02f, 5.3554500579833984e+01f, 9.2046259669587016e-04f, 1.8295941117685288e-04f, -1.8548936350271106e-04f, 1.0393148288130760e-02f, -6.6834740573540330e-04f, 6.3706625951454043e-05f, -7.4256197549402714e-03f, -1.0279154404997826e-02f, -5.3347381763160229e-03f, -2.0230085647199303e-04f, 4.4972959905862808e-02f, 4.6382255852222443e-02f,
			-2.0045276869495865e-06f, -5.9798333040816942e-07f, 1.3315027445059968e-06f, -3.4619006328284740e-06f, 9.6003741418826394e-06f, -1.0520680993795395e-01f, -6.0441894456744194e-03f, 1.9937917590141296e-02f, 9.2044891789555550e-04f, 6.7339438828639686e-05f, 1.6288674942188663e-06f, -2.6550769689492881e-06f, -2.3640905055799522e-07f, -5.8867838381715387e-10f, -1.5121122132200071e-08f, -5.2034913096576929e-04f, 6.8058338911214378e-06f, 1.0775735063361935e-05f, -2.5391910796201955e-08f, 9.6409458194557374e-09f, 2.6672950514949889e-09f,
			-3.2393250876339152e-05f, -2.2609890947933309e-05f, 8.3472943515516818e-05f, -1.5329075608860876e-07f, 3.1558107593809837e-07f, -6.2814927659928799e-03f, 2.8312802896834910e-04f, -2.8582934755831957e-03f, 1.8291932065039873e-04f, 1.6288963706756476e-06f, 9.1781666924362071e-06f, -5.4072875173005741e-06f, -6.4835772306537365e-09f, 1.3737452841944275e-10f, -3.4200661747085803e-10f, 6.8448762249317952e-06f, -3.6012610507896170e-05f, 2.1306108465068974e-05f, 1.0799910965531012e-10f, -3.4312130914315730e-10f, -6.5111810387818991e-10f,
			4.9890528316609561e-05f, 3.1977557227946818e-05f, -1.2983367196284235e-04f, -4.3230005530858762e-07f, 1.9047888599743601e-06f, -6.0257846489548683e-03f, -1.5284898690879345e-03f, 8.6009548977017403e-03f, -1.8544698832556605e-04f, -2.6551272185315611e-06f, -5.4073366300144698e-06f, 1.4393031051440630e-05f, -1.5971821198945690e-08f, 4.4223869011261741e-10f, -5.4078402866863939e-10f, -1.1097878086729906e-05f, -1.9307506590848789e-05f, -3.4532658901298419e-05f, 1.0265270855569497e-09f, -8.1956336162036791e-10f, -2.2865120907766823e-09f,
			-1.5646786778233945e-05f, -9.7410629678051919e-06f, 4.0284918213728815e-05f, 1.7139785768449656e-06f, -1.5928804714349099e-05f, -1.2287999391555786e+00f, 2.2925895173102617e-03f, -2.6710949838161469e-02f, 1.0393155738711357e-02f, -2.3640593838081259e-07f, -6.4811791489205461e-09f, -1.5972609901382384e-08f, 6.3321771449409425e-05f, 3.3199177096321364e-07f, -2.2320757580018835e-06f, 2.1212531464698259e-06f, 3.2101997931022197e-06f, 2.4232499526988249e-06f, -1.6747668496464030e-06f, 2.5793087843339890e-06f, 1.0641876997397048e-06f,
			3.2121482718139305e-07f, 2.0000949518816924e-07f, -8.2712182347677299e-07f, 3.9339244040093035e-07f, -1.2149910162406741e-06f, -2.1592328324913979e-02f, 1.8699542852118611e-04f, 7.2021044616121799e-05f, -6.6834874451160431e-04f, -5.8871690855610836e-10f, 1.3724940628456750e-10f, 4.4230644147269516e-10f, 3.3199174254150421e-07f, 9.9912736914120615e-05f, -4.0852743410368930e-08f, 3.9147689534502206e-09f, -6.9215992937188275e-08f, -7.0368152194077993e-08f, -3.8060299800690700e-08f, -7.5367509566603985e-08f, -2.3470493104582602e-08f,
			-1.5264979538187617e-06f, -9.5021732704481110e-07f, 3.9301489778154064e-06f, -2.5401029688509880e-06f, -4.5260453589435201e-06f, 2.8575679752975702e-03f, -1.1160590220242739e-03f, -3.2150766346603632e-03f, 6.3705869251862168e-05f, -1.5121218055469399e-08f, -3.4221950273582991e-10f, -5.4075111055595926e-10f, -2.2320757580018835e-06f, -4.0853198157719817e-08f, 9.9725519248750061e-05f, 1.3509078655715712e-07f, 2.0688474933194811e-07f, 1.5276552289833489e-07f, 1.1678861255859374e-07f, 1.4450354512973718e-07f, 6.5526265302651154e-08f,
			-2.1614816796500236e-04f, -1.3153137115295976e-04f, 5.3942616796121001e-04f, 3.3012838684953749e-05f, -9.4933151558507234e-05f, 9.7096288204193115e-01f, 6.1144329607486725e-02f, -2.1821361780166626e-01f, -7.4256518855690956e-03f, -5.2034918917343020e-04f, 6.8443268901319243e-06f, -1.1097737115051132e-05f, 2.1212676983850542e-06f, 3.9149239405844583e-09f, 1.3508970653219876e-07f, 4.6292310580611229e-03f, 2.0611056243069470e-05f, 4.2525040043983608e-05f, 2.1577127995442424e-07f, -7.3872811867659038e-08f, -1.5092286531626087e-08f,
			-7.4237032094970345e-04f, -4.6551087871193886e-04f, 1.9105921965092421e-03f, 8.6761770944576710e-05f, -2.9946726863272488e-04f, 2.1565580368041992e+00f, 1.7335081100463867e-01f, -7.1389639377593994e-01f, -1.0278771631419659e-02f, 6.8058116085012443e-06f, -3.6012479540659115e-05f, -1.9307142792968079e-05f, 3.2102047953230795e-06f, -6.9215957410051487e-08f, 2.0688821678049862e-07f, 2.0613559172488749e-05f, 7.3290541768074036e-03f, 1.3586098793894053e-04f, -6.8022117716282082e-08f, -2.7972839689027751e-07f, -1.3294845757627627e-07f,
			-7.4153725290670991e-04f, -4.6530526014976203e-04f, 1.9155518384650350e-03f, 6.8411332904361188e-05f, -2.4497651611454785e-04f, 1.5645618438720703e+00f, 1.4149840176105499e-01f, -6.0691231489181519e-01f, -5.3345630876719952e-03f, 1.0775971531984396e-05f, 2.1306335838744417e-05f, -3.4532517020124942e-05f, 2.4232460873463424e-06f, -7.0367747184718610e-08f, 1.5276349074611062e-07f, 4.2525905882939696e-05f, 1.3586239947471768e-04f, 7.2971871122717857e-03f, -1.0468691868936730e-07f, -2.2436098845446395e-07f, -1.0051029164515057e-07f,
			-1.0093333457916742e-06f, -6.2808601342112524e-07f, 2.5984172680182382e-06f, 1.4002334580709430e-07f, -1.7763507003110135e-06f, 3.3647935837507248e-02f, 3.1555656460113823e-04f, -1.8647639080882072e-03f, -2.0230076916050166e-04f, -2.5392036917537553e-08f, 1.0803027916672647e-10f, 1.0264853411712238e-09f, -1.6747667359595653e-06f, -3.8060306906118058e-08f, 1.1678861255859374e-07f, 2.1576983044724329e-07f, -6.8023823018847906e-08f, -1.0468918532069438e-07f, 7.5745538197224960e-07f, -6.3558040608313604e-08f, -2.3654614267343277e-08f,
			-1.1073016139562242e-06f, -6.8936111574657843e-07f, 2.8512201879493659e-06f, 8.4199946286389604e-06f, 1.0535139836065355e-06f, -1.8120877742767334e+00f, 1.7744705080986023e-02f, 5.1091457717120647e-03f, 4.4972959905862808e-02f, 9.6412726691141870e-09f, -3.4336727905426301e-10f, -8.1960782605250415e-10f, 2.5793096938286908e-06f, -7.5364297913438349e-08f, 1.4450402829879749e-07f, -7.3875582984328503e-08f, -2.7972924954156042e-07f, -2.2435951052557357e-07f, -6.3558047713740962e-08f, 9.9770812084898353e-05f, -8.9589754281860223e-08f,
			-6.2298846614794456e-07f, -3.8802963331363571e-07f, 1.6040736454669968e-06f, 8.1725138443289325e-06f, 2.5394695057912031e-06f, -1.8054578304290771e+00f, 1.4385740272700787e-02f, 9.4506312161684036e-03f, 4.6382255852222443e-02f, 2.6671636010888733e-09f, -6.5114730274373755e-10f, -2.2864601323391298e-09f, 1.0641880408002180e-06f, -2.3469929999464512e-08f, 6.5526677417437895e-08f, -1.5091567107106130e-08f, -1.3295007761371380e-07f, -1.0051021348544964e-07f, -2.3654633807268510e-08f, -8.9589192953098973e-08f, 9.9961427622474730e-05f};

	float32_t QBuff[12*12] = {2.0943951312801801e-05f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
			0.0000000000000000e+00f, 2.0943951312801801e-05f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
			0.0000000000000000e+00f, 0.0000000000000000e+00f, 2.0943951312801801e-05f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
			0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 1.4544409623340471e-06f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
			0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 1.4544409623340471e-06f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
			0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 1.4544409623340471e-06f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
			0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 1.9619999511633068e-04f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
			0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 1.9619999511633068e-04f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
			0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 1.9619999511633068e-04f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
			0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 3.9199996535899118e-05f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
			0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 3.9199996535899118e-05f, 0.0000000000000000e+00f,
			0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 3.9199996535899118e-05f};

	float32_t actualPdotBuff[21*21] = {1.6653972852509469e-04f, -1.3902165228500962e-03f, -5.7315267622470856e-04f, -2.1549873054027557e-05f, 8.9598863269202411e-05f, -3.4388661384582520e-01f, -2.7750030159950256e-02f, 1.3428026437759399e-01f, -1.0875484440475702e-03f, -7.4219278758391738e-06f, -2.7077362574345898e-06f, 4.3978357098239940e-06f, -1.5014911980415491e-07f, 3.1499867159112682e-09f, -1.4469070386269323e-08f, -1.4655641280114651e-05f, -1.5965037164278328e-05f, -2.2496997189591639e-05f, -8.5981657349520901e-09f, -9.7626031703157423e-09f, -5.9299489763020574e-09f,
			-1.3902166392654181e-03f, -1.7642914317548275e-03f, 3.4478278830647469e-03f, 2.2715004161000252e-04f, -7.0682534715160728e-04f, 6.2889542579650879e+00f, 4.4285455346107483e-01f, -1.6801494359970093e+00f, -4.2174980044364929e-02f, -1.5188898032647558e-06f, 4.5258826730787405e-07f, -9.2358868641895242e-06f, 4.5667425183637533e-06f, -9.3707171799906064e-08f, 4.4586880676433793e-07f, 5.4267231462290511e-05f, 2.1912378724664450e-04f, 1.9551423611119390e-04f, 2.9530420420087466e-07f, 3.2293272056449496e-07f, 1.8084207908941607e-07f,
			-5.7315488811582327e-04f, 3.4478260204195976e-03f, 2.0397950429469347e-03f, 9.0639485279098153e-05f, -3.4211372258141637e-04f, 1.8355742692947388e+00f, 1.3840644061565399e-01f, -6.0253530740737915e-01f, -3.2901912927627563e-03f, 2.7581572794588283e-06f, 7.9497849583276547e-06f, -1.7989246771321632e-05f, 1.0774750762720942e-06f, -2.2305892244389725e-08f, 1.0396938421308732e-07f, 2.5436389478272758e-05f, 7.0091569796204567e-05f, 1.1511545017128810e-04f, 6.6340412274712435e-08f, 7.3109596598897042e-08f, 4.2245222431347429e-08f,
			-2.1549913071794435e-05f, 2.2715030354447663e-04f, 9.0639456175267696e-05f, 5.7497127272654325e-06f, -2.1090574591653422e-05f, 1.0727488994598389e-01f, 7.3336521163582802e-03f, -3.1112715601921082e-02f, -5.4939882829785347e-06f, -5.4235201218943985e-08f, 2.5387532076592834e-09f, -1.3716037194910768e-08f, 2.0550031010202474e-08f, 1.6774920341688926e-09f, -1.0013978091194531e-08f, 5.4863460263732122e-07f, 1.5554500123471371e-06f, 1.2696427802438848e-06f, 2.8319711020685645e-09f, 1.5918593021524430e-07f, 1.2904735058327788e-07f,
			8.9598499471321702e-05f, -7.0682511432096362e-04f, -3.4211430465802550e-04f, -2.1090567315695807e-05f, 7.7466087532229722e-05f, -4.5492169260978699e-01f, -2.9710367321968079e-02f, 1.2490954995155334e-01f, 1.3194676721468568e-03f, 2.1848074993613409e-07f, -3.1325804172865901e-08f, 9.4242544435019227e-08f, -2.9269278911669971e-07f, 7.8973488859546137e-10f, -3.5233572504012045e-08f, -2.3912236883916194e-06f, -7.8230295912362635e-06f, -6.6507045630714856e-06f, -2.0434873349017835e-08f, 5.6011646876186205e-08f, 1.0358630220252962e-07f,
			-3.4388375282287598e-01f, 6.2889404296875000e+00f, 1.8355662822723389e+00f, 1.0727469623088837e-01f, -4.5492112636566162e-01f, 6.0659941406250000e+03f, 1.0792869567871094e+02f, -5.5475665283203125e+02f, -6.2987468719482422e+01f, -9.2046259669587016e-04f, -1.8295941117685288e-04f, 1.8548936350271106e-04f, -1.0393148288130760e-02f, 6.6834740573540330e-04f, -6.3706625951454043e-05f, 7.4256197549402714e-03f, 1.0279154404997826e-02f, 5.3347381763160229e-03f, 2.0230085647199303e-04f, -4.4972959905862808e-02f, -4.6382255852222443e-02f,
			-2.7749788016080856e-02f, 4.4285380840301514e-01f, 1.3840639591217041e-01f, 7.3336493223905563e-03f, -2.9710344970226288e-02f, 1.0792903137207031e+02f, 7.7473964691162109e+00f, -3.7863525390625000e+01f, 5.5410069227218628e-01f, 1.2918424545205198e-05f, 6.6460706875659525e-05f, -1.2253208842594177e-04f, -4.6478180593112484e-06f, -7.3903946031350642e-05f, 1.5015818462416064e-05f, 5.7404104154556990e-04f, 1.8577404553070664e-03f, 1.8599247559905052e-03f, 3.4423349006829085e-06f, -6.8131557782180607e-05f, 1.0867154924198985e-04f,
			1.3428059220314026e-01f, -1.6801526546478271e+00f, -6.0253542661666870e-01f, -3.1112711876630783e-02f, 1.2490954995155334e-01f, -5.5475762939453125e+02f, -3.7863601684570312e+01f, 1.7675575256347656e+02f, -5.8906364440917969e-01f, -2.0674371626228094e-05f, -5.0223170546814799e-04f, 7.5222982559353113e-04f, -2.7621979825198650e-04f, 7.2187467594631016e-05f, -2.1872647266718559e-05f, -3.1812421511858702e-03f, -1.1091409251093864e-02f, -1.1088214814662933e-02f, -1.4166337678034324e-05f, 5.3464507800526917e-05f, -1.0552022104093339e-05f,
			-1.0875741718336940e-03f, -4.2174808681011200e-02f, -3.2900949008762836e-03f, -5.4929987527430058e-06f, 1.3194660423323512e-03f, -6.2987445831298828e+01f, 5.5410391092300415e-01f, -5.8907103538513184e-01f, 8.7912267446517944e-01f, -1.8767492520055384e-06f, 1.1017135875590611e-05f, -1.6933805454755202e-05f, 2.5332985387649387e-05f, 7.3879573392332532e-06f, 1.0049511911347508e-04f, 9.2010261141695082e-05f, 3.1675162608735263e-04f, 3.0485849129036069e-04f, 2.1773888647658168e-07f, 1.2509175576269627e-05f, 1.0807212674990296e-03f,
			-7.4219278758391738e-06f, -1.5189419855232700e-06f, 2.7580922505876515e-06f, -5.4234067903280447e-08f, 2.1848259734724707e-07f, -9.2044891789555550e-04f, 1.2917295862280298e-05f, -2.0670411686296575e-05f, -1.8768554355119704e-06f, 1.4544409623340471e-06f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
			-2.7078235689259600e-06f, 4.5243970703268133e-07f, 7.9497040132991970e-06f, 2.5402913106375991e-09f, -3.1321889082391863e-08f, -1.8291932065039873e-04f, 6.6459106164984405e-05f, -5.0222419667989016e-04f, 1.1016914868378080e-05f, 0.0000000000000000e+00f, 1.4544409623340471e-06f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
			4.3979084694001358e-06f, -9.2360442067729309e-06f, -1.7989295884035528e-05f, -1.3714670288322850e-08f, 9.4250957261010626e-08f, 1.8544698832556605e-04f, -1.2253390741534531e-04f, 7.5224071042612195e-04f, -1.6934123777900822e-05f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 1.4544409623340471e-06f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
			-1.5014995824458310e-07f, 4.5667211452382617e-06f, 1.0774712109196116e-06f, 2.0549714818685061e-08f, -2.9269122592268104e-07f, -1.0393155738711357e-02f, -4.6479854063363746e-06f, -2.7621875051409006e-04f, 2.5332945369882509e-05f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
			3.1500559938280048e-09f, -9.3707456017000368e-08f, -2.2306052116505271e-08f, 1.6774797106933192e-09f, 7.8974327077929729e-10f, 6.6834874451160431e-04f, -7.3903953307308257e-05f, 7.2187489422503859e-05f, 7.3879491537809372e-06f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
			-1.4469141440542899e-08f, 4.4587031311493774e-07f, 1.0396964711389955e-07f, -1.0013971873945593e-08f, -3.5233554740443651e-08f, -6.3705869251862168e-05f, 1.5015830285847187e-05f, -2.1872709112358280e-05f, 1.0049511183751747e-04f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
			-1.4655583072453737e-05f, 5.4266980441752821e-05f, 2.5436091164010577e-05f, 5.4864193543835427e-07f, -2.3912186861707596e-06f, 7.4256518855690956e-03f, 5.7403329992666841e-04f, -3.1812046654522419e-03f, 9.2009402578696609e-05f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 3.9199996535899118e-05f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
			-1.5965271813911386e-05f, 2.1912265219725668e-04f, 7.0090987719595432e-05f, 1.5554493302261108e-06f, -7.8229932114481926e-06f, 1.0278771631419659e-02f, 1.8577309092506766e-03f, -1.1091358028352261e-02f, 3.1675017089582980e-04f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 3.9199996535899118e-05f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
			-2.2497331883641891e-05f, 1.9551391596905887e-04f, 1.1511526827234775e-04f, 1.2696392559519154e-06f, -6.6506459006632213e-06f, 5.3345630876719952e-03f, 1.8599254544824362e-03f, -1.1088209226727486e-02f, 3.0485767638310790e-04f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 3.9199996535899118e-05f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
			-8.5977767128042615e-09f, 2.9530116307796561e-07f, 6.6339730153686105e-08f, 2.8319429024037390e-09f, -2.0434731240470683e-08f, 2.0230076916050166e-04f, 3.4423103443259606e-06f, -1.4166182154440321e-05f, 2.1773436742478225e-07f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
			-9.7625987294236438e-09f, 3.2293257845594781e-07f, 7.3109532650050824e-08f, 1.5918591600438958e-07f, 5.6011639770758848e-08f, -4.4972959905862808e-02f, -6.8131565058138222e-05f, 5.3464529628399760e-05f, 1.2509170119301416e-05f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
			-5.9298947974184557e-09f, 1.8084189434830478e-07f, 4.2245126508078101e-08f, 1.2904729373985901e-07f, 1.0358636615137584e-07f, -4.6382255852222443e-02f, 1.0867154924198985e-04f, -1.0552012099651620e-05f, 1.0807212674990296e-03f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f};

	arm_matrix_instance_f32 P, Q, Pdot, actualPdot;
	float32_t PdotBuff[21*21];
	arm_mat_init_f32(&P, 21, 21, PBuff);
	arm_mat_init_f32(&Q, 12, 12, QBuff);
	arm_mat_init_f32(&actualPdot, 21, 21, actualPdotBuff);

	compute_Pdot(&q, &a_sf, &g_sf, &gBias, &aBias, &aMeas, &wMeas, &P, &Q,
				 phi, h, vn, ve, vd, we, &Pdot, PdotBuff);

	bool test = false;
	test = areMatricesEqual(&Pdot, &actualPdot);
	return test;
}

bool test_integrate(void) {

	arm_matrix_instance_f32 x, P, qdot, pdot, vdot, Pdot, xMinus, PMinus;

	float32_t xData[22*1] = {-2.2884607315063477e-02,  9.1512638330459595e-01,
	        4.0011137723922729e-01, -4.3943215161561966e-02,
	        3.5394687652587891e+01, -1.1787300109863281e+02,
	        2.8781792968750000e+04, -1.2077078819274902e+01,
	        5.7730107307434082e+00,  1.1333886718750000e+02,
	       -2.7942578890360892e-04, -2.1035106328781694e-04,
	       -2.6591881760396063e-04,  8.8350940495729446e-03,
	        1.6256757080554962e-03,  1.9927009998355061e-04,
	        2.1494920656550676e-04, -1.0350634111091495e-03,
	       -8.9672525064088404e-05,  1.5854457160457969e-03,
	        1.0850373655557632e-03,  4.6325451694428921e-04};

	float32_t PData[21*21] = {5.0389925017952919e-03f, 3.1386734917759895e-03f, -1.2951527722179890e-02f, -8.0738391261547804e-04f, 2.5598644278943539e-03f, -2.1820283889770508e+01f, -1.5453934669494629e+00f, 5.9288525581359863e+00f, 1.3891488313674927e-01f, -2.0048216811119346e-06f, -3.2393774745287374e-05f, 4.9889800720848143e-05f, -1.5646855899831280e-05f, 3.2121343451763096e-07f, -1.5264936337189283e-06f, -2.1615074365399778e-04f, -7.4237369699403644e-04f, -7.4153754394501448e-04f, -1.0093436912939069e-06f, -1.1073028645114391e-06f, -6.2298909142555203e-07f,
			3.1386725604534149e-03f, 1.9648219458758831e-03f, -8.0804517492651939e-03f, -5.0261052092537284e-04f, 1.5936470590531826e-03f, -1.3582573890686035e+01f, -9.6213459968566895e-01f, 3.6917583942413330e+00f, 8.6432509124279022e-02f, -5.9811674191223574e-07f, -2.2610181986237876e-05f, 3.1977098842617124e-05f, -9.7411066235508770e-06f, 2.0000864253688633e-07f, -9.5021465540412464e-07f, -1.3153281179256737e-04f, -4.6551285777240992e-04f, -4.6530558029189706e-04f, -6.2809255041429424e-07f, -6.8936219577153679e-07f, -3.8803003121756774e-07f,
			-1.2951526790857315e-02f, -8.0804536119103432e-03f, 3.3347148448228836e-02f, 2.0787562243640423e-03f, -6.5908450633287430e-03f, 5.6180130004882812e+01f, 3.9788146018981934e+00f, -1.5264608383178711e+01f, -3.5766848921775818e-01f, 1.3322169252205640e-06f, 8.3474267739802599e-05f, -1.2983183842152357e-04f, 4.0285089198732749e-05f, -8.2711824234138476e-07f, 3.9301376091316342e-06f, 5.3943332750350237e-04f, 1.9106010440737009e-03f, 1.9155526533722878e-03f, 2.5984440981119405e-06f, 2.8512238259281730e-06f, 1.6040754644564004e-06f,
			-8.0738525139167905e-04f, -5.0261139404028654e-04f, 2.0787597168236971e-03f, 1.7619828577153385e-04f, -5.2187195979058743e-04f, 4.7090721130371094e+00f, 3.2039403915405273e-01f, -1.1370785236358643e+00f, -3.0653338879346848e-02f, -3.4618537938513327e-06f, -1.5313280243844929e-07f, -4.3224565615673782e-07f, 1.7139997225967818e-06f, 3.9339300883511896e-07f, -2.5401027414773125e-06f, 3.3012915082508698e-05f, 8.6762891442049295e-05f, 6.8411834945436567e-05f, 1.4002503689880541e-07f, 8.4199955381336622e-06f, 8.1725174823077396e-06f,
			2.5598646607249975e-03f, 1.5936469426378608e-03f, -6.5908445976674557e-03f, -5.2187172695994377e-04f, 1.6188059234991670e-03f, -1.4437356948852539e+01f, -9.6182018518447876e-01f, 3.5346310138702393e+00f, 1.0662219673395157e-01f, 9.6007197498693131e-06f, 3.1550774792776792e-07f, 1.9050106629947550e-06f, -1.5928870197967626e-05f, -1.2149911299275118e-06f, -4.5260462684382219e-06f, -9.4931870989967138e-05f, -2.9946854920126498e-04f, -2.4497826234437525e-04f, -1.7763567257134127e-06f, 1.0535148931012372e-06f, 2.5394670046807732e-06f,
			-2.1820224761962891e+01f, -1.3582537651062012e+01f, 5.6179962158203125e+01f, 4.7090630531311035e+00f, -1.4437336921691895e+01f, 2.3969992187500000e+05f, 8.5390068359375000e+03f, -3.1784367187500000e+04f, -3.0329980468750000e+03f, -1.0520824044942856e-01f, -6.2867989763617516e-03f, -6.0234428383409977e-03f, -1.2287986278533936e+00f, -2.1592376753687859e-02f, 2.8575158212333918e-03f, 9.7096240520477295e-01f, 2.1565728187561035e+00f, 1.5645617246627808e+00f, 3.3647991716861725e-02f, -1.8120876550674438e+00f, -1.8054577112197876e+00f,
			-1.5453902482986450e+00f, -9.6213251352310181e-01f, 3.9788067340850830e+00f, 3.2039386034011841e-01f, -9.6181970834732056e-01f, 8.5390214843750000e+03f, 5.9188671875000000e+02f, -2.1271850585937500e+03f, -5.3193809509277344e+01f, -6.0443156398832798e-03f, 2.8295663651078939e-04f, -1.5286422567442060e-03f, 2.2926249075680971e-03f, 1.8699679640121758e-04f, -1.1160597205162048e-03f, 6.1143510043621063e-02f, 1.7335088551044464e-01f, 1.4149878919124603e-01f, 3.1555970781482756e-04f, 1.7744706943631172e-02f, 1.4385745860636234e-02f,
			5.9288649559020996e+00f, 3.6917665004730225e+00f, -1.5264640808105469e+01f, -1.1370774507522583e+00f, 3.5346331596374512e+00f, -3.1784408203125000e+04f, -2.1271867675781250e+03f, 7.9166689453125000e+03f, 2.2918943786621094e+02f, 1.9937748089432716e-02f, -2.8586506377905607e-03f, 8.6001874879002571e-03f, -2.6711093261837959e-02f, 7.2020280640572309e-05f, -3.2150780316442251e-03f, -2.1821407973766327e-01f, -7.1389967203140259e-01f, -6.0691767930984497e-01f, -1.8647768301889300e-03f, 5.1091467030346394e-03f, 9.4506256282329559e-03f,
			1.3891413807868958e-01f, 8.6432047188282013e-02f, -3.5766661167144775e-01f, -3.0653275549411774e-02f, 1.0662207007408142e-01f, -3.0329960937500000e+03f, -5.3193630218505859e+01f, 2.2918931579589844e+02f, 5.3554500579833984e+01f, 9.2046259669587016e-04f, 1.8295941117685288e-04f, -1.8548936350271106e-04f, 1.0393148288130760e-02f, -6.6834740573540330e-04f, 6.3706625951454043e-05f, -7.4256197549402714e-03f, -1.0279154404997826e-02f, -5.3347381763160229e-03f, -2.0230085647199303e-04f, 4.4972959905862808e-02f, 4.6382255852222443e-02f,
			-2.0045276869495865e-06f, -5.9798333040816942e-07f, 1.3315027445059968e-06f, -3.4619006328284740e-06f, 9.6003741418826394e-06f, -1.0520680993795395e-01f, -6.0441894456744194e-03f, 1.9937917590141296e-02f, 9.2044891789555550e-04f, 6.7339438828639686e-05f, 1.6288674942188663e-06f, -2.6550769689492881e-06f, -2.3640905055799522e-07f, -5.8867838381715387e-10f, -1.5121122132200071e-08f, -5.2034913096576929e-04f, 6.8058338911214378e-06f, 1.0775735063361935e-05f, -2.5391910796201955e-08f, 9.6409458194557374e-09f, 2.6672950514949889e-09f,
			-3.2393250876339152e-05f, -2.2609890947933309e-05f, 8.3472943515516818e-05f, -1.5329075608860876e-07f, 3.1558107593809837e-07f, -6.2814927659928799e-03f, 2.8312802896834910e-04f, -2.8582934755831957e-03f, 1.8291932065039873e-04f, 1.6288963706756476e-06f, 9.1781666924362071e-06f, -5.4072875173005741e-06f, -6.4835772306537365e-09f, 1.3737452841944275e-10f, -3.4200661747085803e-10f, 6.8448762249317952e-06f, -3.6012610507896170e-05f, 2.1306108465068974e-05f, 1.0799910965531012e-10f, -3.4312130914315730e-10f, -6.5111810387818991e-10f,
			4.9890528316609561e-05f, 3.1977557227946818e-05f, -1.2983367196284235e-04f, -4.3230005530858762e-07f, 1.9047888599743601e-06f, -6.0257846489548683e-03f, -1.5284898690879345e-03f, 8.6009548977017403e-03f, -1.8544698832556605e-04f, -2.6551272185315611e-06f, -5.4073366300144698e-06f, 1.4393031051440630e-05f, -1.5971821198945690e-08f, 4.4223869011261741e-10f, -5.4078402866863939e-10f, -1.1097878086729906e-05f, -1.9307506590848789e-05f, -3.4532658901298419e-05f, 1.0265270855569497e-09f, -8.1956336162036791e-10f, -2.2865120907766823e-09f,
			-1.5646786778233945e-05f, -9.7410629678051919e-06f, 4.0284918213728815e-05f, 1.7139785768449656e-06f, -1.5928804714349099e-05f, -1.2287999391555786e+00f, 2.2925895173102617e-03f, -2.6710949838161469e-02f, 1.0393155738711357e-02f, -2.3640593838081259e-07f, -6.4811791489205461e-09f, -1.5972609901382384e-08f, 6.3321771449409425e-05f, 3.3199177096321364e-07f, -2.2320757580018835e-06f, 2.1212531464698259e-06f, 3.2101997931022197e-06f, 2.4232499526988249e-06f, -1.6747668496464030e-06f, 2.5793087843339890e-06f, 1.0641876997397048e-06f,
			3.2121482718139305e-07f, 2.0000949518816924e-07f, -8.2712182347677299e-07f, 3.9339244040093035e-07f, -1.2149910162406741e-06f, -2.1592328324913979e-02f, 1.8699542852118611e-04f, 7.2021044616121799e-05f, -6.6834874451160431e-04f, -5.8871690855610836e-10f, 1.3724940628456750e-10f, 4.4230644147269516e-10f, 3.3199174254150421e-07f, 9.9912736914120615e-05f, -4.0852743410368930e-08f, 3.9147689534502206e-09f, -6.9215992937188275e-08f, -7.0368152194077993e-08f, -3.8060299800690700e-08f, -7.5367509566603985e-08f, -2.3470493104582602e-08f,
			-1.5264979538187617e-06f, -9.5021732704481110e-07f, 3.9301489778154064e-06f, -2.5401029688509880e-06f, -4.5260453589435201e-06f, 2.8575679752975702e-03f, -1.1160590220242739e-03f, -3.2150766346603632e-03f, 6.3705869251862168e-05f, -1.5121218055469399e-08f, -3.4221950273582991e-10f, -5.4075111055595926e-10f, -2.2320757580018835e-06f, -4.0853198157719817e-08f, 9.9725519248750061e-05f, 1.3509078655715712e-07f, 2.0688474933194811e-07f, 1.5276552289833489e-07f, 1.1678861255859374e-07f, 1.4450354512973718e-07f, 6.5526265302651154e-08f,
			-2.1614816796500236e-04f, -1.3153137115295976e-04f, 5.3942616796121001e-04f, 3.3012838684953749e-05f, -9.4933151558507234e-05f, 9.7096288204193115e-01f, 6.1144329607486725e-02f, -2.1821361780166626e-01f, -7.4256518855690956e-03f, -5.2034918917343020e-04f, 6.8443268901319243e-06f, -1.1097737115051132e-05f, 2.1212676983850542e-06f, 3.9149239405844583e-09f, 1.3508970653219876e-07f, 4.6292310580611229e-03f, 2.0611056243069470e-05f, 4.2525040043983608e-05f, 2.1577127995442424e-07f, -7.3872811867659038e-08f, -1.5092286531626087e-08f,
			-7.4237032094970345e-04f, -4.6551087871193886e-04f, 1.9105921965092421e-03f, 8.6761770944576710e-05f, -2.9946726863272488e-04f, 2.1565580368041992e+00f, 1.7335081100463867e-01f, -7.1389639377593994e-01f, -1.0278771631419659e-02f, 6.8058116085012443e-06f, -3.6012479540659115e-05f, -1.9307142792968079e-05f, 3.2102047953230795e-06f, -6.9215957410051487e-08f, 2.0688821678049862e-07f, 2.0613559172488749e-05f, 7.3290541768074036e-03f, 1.3586098793894053e-04f, -6.8022117716282082e-08f, -2.7972839689027751e-07f, -1.3294845757627627e-07f,
			-7.4153725290670991e-04f, -4.6530526014976203e-04f, 1.9155518384650350e-03f, 6.8411332904361188e-05f, -2.4497651611454785e-04f, 1.5645618438720703e+00f, 1.4149840176105499e-01f, -6.0691231489181519e-01f, -5.3345630876719952e-03f, 1.0775971531984396e-05f, 2.1306335838744417e-05f, -3.4532517020124942e-05f, 2.4232460873463424e-06f, -7.0367747184718610e-08f, 1.5276349074611062e-07f, 4.2525905882939696e-05f, 1.3586239947471768e-04f, 7.2971871122717857e-03f, -1.0468691868936730e-07f, -2.2436098845446395e-07f, -1.0051029164515057e-07f,
			-1.0093333457916742e-06f, -6.2808601342112524e-07f, 2.5984172680182382e-06f, 1.4002334580709430e-07f, -1.7763507003110135e-06f, 3.3647935837507248e-02f, 3.1555656460113823e-04f, -1.8647639080882072e-03f, -2.0230076916050166e-04f, -2.5392036917537553e-08f, 1.0803027916672647e-10f, 1.0264853411712238e-09f, -1.6747667359595653e-06f, -3.8060306906118058e-08f, 1.1678861255859374e-07f, 2.1576983044724329e-07f, -6.8023823018847906e-08f, -1.0468918532069438e-07f, 7.5745538197224960e-07f, -6.3558040608313604e-08f, -2.3654614267343277e-08f,
			-1.1073016139562242e-06f, -6.8936111574657843e-07f, 2.8512201879493659e-06f, 8.4199946286389604e-06f, 1.0535139836065355e-06f, -1.8120877742767334e+00f, 1.7744705080986023e-02f, 5.1091457717120647e-03f, 4.4972959905862808e-02f, 9.6412726691141870e-09f, -3.4336727905426301e-10f, -8.1960782605250415e-10f, 2.5793096938286908e-06f, -7.5364297913438349e-08f, 1.4450402829879749e-07f, -7.3875582984328503e-08f, -2.7972924954156042e-07f, -2.2435951052557357e-07f, -6.3558047713740962e-08f, 9.9770812084898353e-05f, -8.9589754281860223e-08f,
			-6.2298846614794456e-07f, -3.8802963331363571e-07f, 1.6040736454669968e-06f, 8.1725138443289325e-06f, 2.5394695057912031e-06f, -1.8054578304290771e+00f, 1.4385740272700787e-02f, 9.4506312161684036e-03f, 4.6382255852222443e-02f, 2.6671636010888733e-09f, -6.5114730274373755e-10f, -2.2864601323391298e-09f, 1.0641880408002180e-06f, -2.3469929999464512e-08f, 6.5526677417437895e-08f, -1.5091567107106130e-08f, -1.3295007761371380e-07f, -1.0051021348544964e-07f, -2.3654633807268510e-08f, -8.9589192953098973e-08f, 9.9961427622474730e-05f};

	float32_t dt = 0.0100;

	float32_t qDotData[4*1] = {-0.0536566600203514, -0.0020567174069583, -0.0006578038446605,
		       -0.0208778418600559};

	float32_t pDotData[3*1] = {-1.0836305591510609e-04,  6.3261977629736066e-05,
		       -1.1333886718750000e+02};

	float32_t vDotData[3*1] = {-0.054375272244215 , -0.2251708954572678, -1.1488429307937622};

	float32_t PDotData[21*21] = {1.6653972852509469e-04f, -1.3902165228500962e-03f, -5.7315267622470856e-04f, -2.1549873054027557e-05f, 8.9598863269202411e-05f, -3.4388661384582520e-01f, -2.7750030159950256e-02f, 1.3428026437759399e-01f, -1.0875484440475702e-03f, -7.4219278758391738e-06f, -2.7077362574345898e-06f, 4.3978357098239940e-06f, -1.5014911980415491e-07f, 3.1499867159112682e-09f, -1.4469070386269323e-08f, -1.4655641280114651e-05f, -1.5965037164278328e-05f, -2.2496997189591639e-05f, -8.5981657349520901e-09f, -9.7626031703157423e-09f, -5.9299489763020574e-09f,
			-1.3902166392654181e-03f, -1.7642914317548275e-03f, 3.4478278830647469e-03f, 2.2715004161000252e-04f, -7.0682534715160728e-04f, 6.2889542579650879e+00f, 4.4285455346107483e-01f, -1.6801494359970093e+00f, -4.2174980044364929e-02f, -1.5188898032647558e-06f, 4.5258826730787405e-07f, -9.2358868641895242e-06f, 4.5667425183637533e-06f, -9.3707171799906064e-08f, 4.4586880676433793e-07f, 5.4267231462290511e-05f, 2.1912378724664450e-04f, 1.9551423611119390e-04f, 2.9530420420087466e-07f, 3.2293272056449496e-07f, 1.8084207908941607e-07f,
			-5.7315488811582327e-04f, 3.4478260204195976e-03f, 2.0397950429469347e-03f, 9.0639485279098153e-05f, -3.4211372258141637e-04f, 1.8355742692947388e+00f, 1.3840644061565399e-01f, -6.0253530740737915e-01f, -3.2901912927627563e-03f, 2.7581572794588283e-06f, 7.9497849583276547e-06f, -1.7989246771321632e-05f, 1.0774750762720942e-06f, -2.2305892244389725e-08f, 1.0396938421308732e-07f, 2.5436389478272758e-05f, 7.0091569796204567e-05f, 1.1511545017128810e-04f, 6.6340412274712435e-08f, 7.3109596598897042e-08f, 4.2245222431347429e-08f,
			-2.1549913071794435e-05f, 2.2715030354447663e-04f, 9.0639456175267696e-05f, 5.7497127272654325e-06f, -2.1090574591653422e-05f, 1.0727488994598389e-01f, 7.3336521163582802e-03f, -3.1112715601921082e-02f, -5.4939882829785347e-06f, -5.4235201218943985e-08f, 2.5387532076592834e-09f, -1.3716037194910768e-08f, 2.0550031010202474e-08f, 1.6774920341688926e-09f, -1.0013978091194531e-08f, 5.4863460263732122e-07f, 1.5554500123471371e-06f, 1.2696427802438848e-06f, 2.8319711020685645e-09f, 1.5918593021524430e-07f, 1.2904735058327788e-07f,
			8.9598499471321702e-05f, -7.0682511432096362e-04f, -3.4211430465802550e-04f, -2.1090567315695807e-05f, 7.7466087532229722e-05f, -4.5492169260978699e-01f, -2.9710367321968079e-02f, 1.2490954995155334e-01f, 1.3194676721468568e-03f, 2.1848074993613409e-07f, -3.1325804172865901e-08f, 9.4242544435019227e-08f, -2.9269278911669971e-07f, 7.8973488859546137e-10f, -3.5233572504012045e-08f, -2.3912236883916194e-06f, -7.8230295912362635e-06f, -6.6507045630714856e-06f, -2.0434873349017835e-08f, 5.6011646876186205e-08f, 1.0358630220252962e-07f,
			-3.4388375282287598e-01f, 6.2889404296875000e+00f, 1.8355662822723389e+00f, 1.0727469623088837e-01f, -4.5492112636566162e-01f, 6.0659941406250000e+03f, 1.0792869567871094e+02f, -5.5475665283203125e+02f, -6.2987468719482422e+01f, -9.2046259669587016e-04f, -1.8295941117685288e-04f, 1.8548936350271106e-04f, -1.0393148288130760e-02f, 6.6834740573540330e-04f, -6.3706625951454043e-05f, 7.4256197549402714e-03f, 1.0279154404997826e-02f, 5.3347381763160229e-03f, 2.0230085647199303e-04f, -4.4972959905862808e-02f, -4.6382255852222443e-02f,
			-2.7749788016080856e-02f, 4.4285380840301514e-01f, 1.3840639591217041e-01f, 7.3336493223905563e-03f, -2.9710344970226288e-02f, 1.0792903137207031e+02f, 7.7473964691162109e+00f, -3.7863525390625000e+01f, 5.5410069227218628e-01f, 1.2918424545205198e-05f, 6.6460706875659525e-05f, -1.2253208842594177e-04f, -4.6478180593112484e-06f, -7.3903946031350642e-05f, 1.5015818462416064e-05f, 5.7404104154556990e-04f, 1.8577404553070664e-03f, 1.8599247559905052e-03f, 3.4423349006829085e-06f, -6.8131557782180607e-05f, 1.0867154924198985e-04f,
			1.3428059220314026e-01f, -1.6801526546478271e+00f, -6.0253542661666870e-01f, -3.1112711876630783e-02f, 1.2490954995155334e-01f, -5.5475762939453125e+02f, -3.7863601684570312e+01f, 1.7675575256347656e+02f, -5.8906364440917969e-01f, -2.0674371626228094e-05f, -5.0223170546814799e-04f, 7.5222982559353113e-04f, -2.7621979825198650e-04f, 7.2187467594631016e-05f, -2.1872647266718559e-05f, -3.1812421511858702e-03f, -1.1091409251093864e-02f, -1.1088214814662933e-02f, -1.4166337678034324e-05f, 5.3464507800526917e-05f, -1.0552022104093339e-05f,
			-1.0875741718336940e-03f, -4.2174808681011200e-02f, -3.2900949008762836e-03f, -5.4929987527430058e-06f, 1.3194660423323512e-03f, -6.2987445831298828e+01f, 5.5410391092300415e-01f, -5.8907103538513184e-01f, 8.7912267446517944e-01f, -1.8767492520055384e-06f, 1.1017135875590611e-05f, -1.6933805454755202e-05f, 2.5332985387649387e-05f, 7.3879573392332532e-06f, 1.0049511911347508e-04f, 9.2010261141695082e-05f, 3.1675162608735263e-04f, 3.0485849129036069e-04f, 2.1773888647658168e-07f, 1.2509175576269627e-05f, 1.0807212674990296e-03f,
			-7.4219278758391738e-06f, -1.5189419855232700e-06f, 2.7580922505876515e-06f, -5.4234067903280447e-08f, 2.1848259734724707e-07f, -9.2044891789555550e-04f, 1.2917295862280298e-05f, -2.0670411686296575e-05f, -1.8768554355119704e-06f, 1.4544409623340471e-06f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
			-2.7078235689259600e-06f, 4.5243970703268133e-07f, 7.9497040132991970e-06f, 2.5402913106375991e-09f, -3.1321889082391863e-08f, -1.8291932065039873e-04f, 6.6459106164984405e-05f, -5.0222419667989016e-04f, 1.1016914868378080e-05f, 0.0000000000000000e+00f, 1.4544409623340471e-06f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
			4.3979084694001358e-06f, -9.2360442067729309e-06f, -1.7989295884035528e-05f, -1.3714670288322850e-08f, 9.4250957261010626e-08f, 1.8544698832556605e-04f, -1.2253390741534531e-04f, 7.5224071042612195e-04f, -1.6934123777900822e-05f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 1.4544409623340471e-06f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
			-1.5014995824458310e-07f, 4.5667211452382617e-06f, 1.0774712109196116e-06f, 2.0549714818685061e-08f, -2.9269122592268104e-07f, -1.0393155738711357e-02f, -4.6479854063363746e-06f, -2.7621875051409006e-04f, 2.5332945369882509e-05f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
			3.1500559938280048e-09f, -9.3707456017000368e-08f, -2.2306052116505271e-08f, 1.6774797106933192e-09f, 7.8974327077929729e-10f, 6.6834874451160431e-04f, -7.3903953307308257e-05f, 7.2187489422503859e-05f, 7.3879491537809372e-06f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
			-1.4469141440542899e-08f, 4.4587031311493774e-07f, 1.0396964711389955e-07f, -1.0013971873945593e-08f, -3.5233554740443651e-08f, -6.3705869251862168e-05f, 1.5015830285847187e-05f, -2.1872709112358280e-05f, 1.0049511183751747e-04f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
			-1.4655583072453737e-05f, 5.4266980441752821e-05f, 2.5436091164010577e-05f, 5.4864193543835427e-07f, -2.3912186861707596e-06f, 7.4256518855690956e-03f, 5.7403329992666841e-04f, -3.1812046654522419e-03f, 9.2009402578696609e-05f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 3.9199996535899118e-05f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
			-1.5965271813911386e-05f, 2.1912265219725668e-04f, 7.0090987719595432e-05f, 1.5554493302261108e-06f, -7.8229932114481926e-06f, 1.0278771631419659e-02f, 1.8577309092506766e-03f, -1.1091358028352261e-02f, 3.1675017089582980e-04f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 3.9199996535899118e-05f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
			-2.2497331883641891e-05f, 1.9551391596905887e-04f, 1.1511526827234775e-04f, 1.2696392559519154e-06f, -6.6506459006632213e-06f, 5.3345630876719952e-03f, 1.8599254544824362e-03f, -1.1088209226727486e-02f, 3.0485767638310790e-04f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 3.9199996535899118e-05f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
			-8.5977767128042615e-09f, 2.9530116307796561e-07f, 6.6339730153686105e-08f, 2.8319429024037390e-09f, -2.0434731240470683e-08f, 2.0230076916050166e-04f, 3.4423103443259606e-06f, -1.4166182154440321e-05f, 2.1773436742478225e-07f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
			-9.7625987294236438e-09f, 3.2293257845594781e-07f, 7.3109532650050824e-08f, 1.5918591600438958e-07f, 5.6011639770758848e-08f, -4.4972959905862808e-02f, -6.8131565058138222e-05f, 5.3464529628399760e-05f, 1.2509170119301416e-05f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
			-5.9298947974184557e-09f, 1.8084189434830478e-07f, 4.2245126508078101e-08f, 1.2904729373985901e-07f, 1.0358636615137584e-07f, -4.6382255852222443e-02f, 1.0867154924198985e-04f, -1.0552012099651620e-05f, 1.0807212674990296e-03f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f};

	float32_t xMinusData[22];
	float32_t PMinusData[21*21];

	arm_mat_init_f32(&x, 22, 1, xData);
	arm_mat_init_f32(&P, 21, 21, PData);
	arm_mat_init_f32(&qdot, 4, 1, qDotData);
	arm_mat_init_f32(&pdot, 3, 1, pDotData);
	arm_mat_init_f32(&vdot, 3, 1, vDotData);
	arm_mat_init_f32(&Pdot, 21, 21, PDotData);

	integrate(&x, &P, &qdot, &pdot,
			  &vdot, &Pdot, dt, &xMinus,
			  &PMinus, xMinusData,
			  PMinusData);

	arm_matrix_instance_f32 xMinusTrue, PMinusTrue;

	float32_t xMinusTrueData[22*1] = {-2.3421168327331543e-02,  9.1510558128356934e-01,
	        4.0010470151901245e-01, -4.4151984155178070e-02,
	        3.5394687652587891e+01, -1.1787300109863281e+02,
	        2.8780660156250000e+04, -1.2077622413635254e+01,
	        5.7707591056823730e+00,  1.1332737731933594e+02,
	       -2.7942578890360892e-04, -2.1035106328781694e-04,
	       -2.6591881760396063e-04,  8.8350940495729446e-03,
	        1.6256757080554962e-03,  1.9927009998355061e-04,
	        2.1494920656550676e-04, -1.0350634111091495e-03,
	       -8.9672525064088404e-05,  1.5854457160457969e-03,
	        1.0850373655557632e-03,  4.6325451694428921e-04};

	float32_t PMinusDataTrue[21*21] = {5.0406577065587044e-03f, 3.1247714068740606e-03f, -1.2957259081304073e-02f, -8.0759939737617970e-04f, 2.5607603602111340e-03f, -2.1823722839355469e+01f, -1.5456709861755371e+00f, 5.9301953315734863e+00f, 1.3890400528907776e-01f, -2.0790409962501144e-06f, -3.2420852221548557e-05f, 4.9933780246647075e-05f, -1.5648356566089205e-05f, 3.2124492577167985e-07f, -1.5266383570633479e-06f, -2.1629729599226266e-04f, -7.4253336060792208e-04f, -7.4176251655444503e-04f, -1.0094296385432244e-06f, -1.1074005215050420e-06f, -6.2304837911142386e-07f,
			3.1247704755514860e-03f, 1.9471790874376893e-03f, -8.0459732562303543e-03f, -5.0033902516588569e-04f, 1.5865787863731384e-03f, -1.3519684791564941e+01f, -9.5770603418350220e-01f, 3.6749567985534668e+00f, 8.6010761559009552e-02f, -6.1330564449235681e-07f, -2.2605656340601854e-05f, 3.1884741474641487e-05f, -9.6954390755854547e-06f, 1.9907156456611119e-07f, -9.4575597131552058e-07f, -1.3099014176987112e-04f, -4.6332163037732244e-04f, -4.6335044316947460e-04f, -6.2513953480447526e-07f, -6.8613286430263543e-07f, -3.8622161468992999e-07f,
			-1.2957258149981499e-02f, -8.0459751188755035e-03f, 3.3367548137903214e-02f, 2.0796626340597868e-03f, -6.5942662768065929e-03f, 5.6198486328125000e+01f, 3.9801986217498779e+00f, -1.5270633697509766e+01f, -3.5770139098167419e-01f, 1.3597984889202053e-06f, 8.3553764852695167e-05f, -1.3001172919757664e-04f, 4.0295864891959354e-05f, -8.2734129591699457e-07f, 3.9311771615757607e-06f, 5.3968769498169422e-04f, 1.9113019807264209e-03f, 1.9167037680745125e-03f, 2.5991075744968839e-06f, 2.8519548322947230e-06f, 1.6044979247453739e-06f,
			-8.0760073615238070e-04f, -5.0033989828079939e-04f, 2.0796661265194416e-03f, 1.7625578038860112e-04f, -5.2208284614607692e-04f, 4.7101449966430664e+00f, 3.2046738266944885e-01f, -1.1373896598815918e+00f, -3.0653392896056175e-02f, -3.4623960800672648e-06f, -1.5310742185192794e-07f, -4.3238281932644895e-07f, 1.7142052683993825e-06f, 3.9340977764368290e-07f, -2.5402027858945075e-06f, 3.3018401154549792e-05f, 8.6778447439428419e-05f, 6.8424531491473317e-05f, 1.4005335913225281e-07f, 8.4215871538617648e-06f, 8.1738080552895553e-06f,
			2.5607605930417776e-03f, 1.5865786699578166e-03f, -6.5942658111453056e-03f, -5.2208261331543326e-04f, 1.6195805510506034e-03f, -1.4441905975341797e+01f, -9.6211731433868408e-01f, 3.5358800888061523e+00f, 1.0663539171218872e-01f, 9.6029043561429717e-06f, 3.1519448384642601e-07f, 1.9059531268794672e-06f, -1.5931796951917931e-05f, -1.2149832855357090e-06f, -4.5263986976351589e-06f, -9.4955779786687344e-05f, -2.9954678029753268e-04f, -2.4504476459696889e-04f, -1.7765610209607985e-06f, 1.0540750281506917e-06f, 2.5405029191460926e-06f,
			-2.1823663711547852e+01f, -1.3519648551940918e+01f, 5.6198318481445312e+01f, 4.7101359367370605e+00f, -1.4441885948181152e+01f, 2.3976057812500000e+05f, 8.5400859375000000e+03f, -3.1789914062500000e+04f, -3.0336279296875000e+03f, -1.0521744191646576e-01f, -6.2886285595595837e-03f, -6.0215881094336510e-03f, -1.2289025783538818e+00f, -2.1585693582892418e-02f, 2.8568787965923548e-03f, 9.7103667259216309e-01f, 2.1566755771636963e+00f, 1.5646151304244995e+00f, 3.3650014549493790e-02f, -1.8125374317169189e+00f, -1.8059215545654297e+00f,
			-1.5456677675247192e+00f, -9.5770394802093506e-01f, 3.9801907539367676e+00f, 3.2046720385551453e-01f, -9.6211683750152588e-01f, 8.5401005859375000e+03f, 5.9196417236328125e+02f, -2.1275637207031250e+03f, -5.3188266754150391e+01f, -6.0441866517066956e-03f, 2.8362125158309937e-04f, -1.5298675280064344e-03f, 2.2925783414393663e-03f, 1.8625776283442974e-04f, -1.1159095447510481e-03f, 6.1149250715970993e-02f, 1.7336946725845337e-01f, 1.4151738584041595e-01f, 3.1559413764625788e-04f, 1.7744025215506554e-02f, 1.4386832714080811e-02f,
			5.9302077293395996e+00f, 3.6749649047851562e+00f, -1.5270666122436523e+01f, -1.1373885869979858e+00f, 3.5358822345733643e+00f, -3.1789955078125000e+04f, -2.1275654296875000e+03f, 7.9184365234375000e+03f, 2.2918354797363281e+02f, 1.9937541335821152e-02f, -2.8636730276048183e-03f, 8.6077097803354263e-03f, -2.6713855564594269e-02f, 7.2742157499305904e-05f, -3.2152966596186161e-03f, -2.1824589371681213e-01f, -7.1401059627532959e-01f, -6.0702854394912720e-01f, -1.8649185076355934e-03f, 5.1096812821924686e-03f, 9.4505203887820244e-03f,
			1.3890326023101807e-01f, 8.6010299623012543e-02f, -3.5769951343536377e-01f, -3.0653329566121101e-02f, 1.0663526505231857e-01f, -3.0336259765625000e+03f, -5.3188087463378906e+01f, 2.2918342590332031e+02f, 5.3563293457031250e+01f, 9.2044385382905602e-04f, 1.8306958372704685e-04f, -1.8565870414022356e-04f, 1.0393401607871056e-02f, -6.6827354021370411e-04f, 6.4711573941167444e-05f, -7.4246996082365513e-03f, -1.0275986976921558e-02f, -5.3316894918680191e-03f, -2.0229867368470877e-04f, 4.4973086565732956e-02f, 4.6393062919378281e-02f,
			-2.0787470020877663e-06f, -6.1317274457906024e-07f, 1.3590836260846118e-06f, -3.4624429190444062e-06f, 9.6025587481562980e-06f, -1.0521601140499115e-01f, -6.0440604574978352e-03f, 1.9937710836529732e-02f, 9.2043017502874136e-04f, 6.7353983467910439e-05f, 1.6288674942188663e-06f, -2.6550769689492881e-06f, -2.3640905055799522e-07f, -5.8867838381715387e-10f, -1.5121122132200071e-08f, -5.2034913096576929e-04f, 6.8058338911214378e-06f, 1.0775735063361935e-05f, -2.5391910796201955e-08f, 9.6409458194557374e-09f, 2.6672950514949889e-09f,
			-3.2420328352600336e-05f, -2.2605367121286690e-05f, 8.3552440628409386e-05f, -1.5326534708037798e-07f, 3.1526786870017531e-07f, -6.2833218835294247e-03f, 2.8379261493682861e-04f, -2.8633156325668097e-03f, 1.8302949320059270e-04f, 1.6288963706756476e-06f, 9.1927113317069598e-06f, -5.4072875173005741e-06f, -6.4835772306537365e-09f, 1.3737452841944275e-10f, -3.4200661747085803e-10f, 6.8448762249317952e-06f, -3.6012610507896170e-05f, 2.1306108465068974e-05f, 1.0799910965531012e-10f, -3.4312130914315730e-10f, -6.5111810387818991e-10f,
			4.9934507842408493e-05f, 3.1885196221992373e-05f, -1.3001356273889542e-04f, -4.3243719005658932e-07f, 1.9057313238590723e-06f, -6.0239303857088089e-03f, -1.5297152567654848e-03f, 8.6084771901369095e-03f, -1.8561632896307856e-04f, -2.6551272185315611e-06f, -5.4073366300144698e-06f, 1.4407575690711383e-05f, -1.5971821198945690e-08f, 4.4223869011261741e-10f, -5.4078402866863939e-10f, -1.1097878086729906e-05f, -1.9307506590848789e-05f, -3.4532658901298419e-05f, 1.0265270855569497e-09f, -8.1956336162036791e-10f, -2.2865120907766823e-09f,
			-1.5648287444491871e-05f, -9.6953954198397696e-06f, 4.0295693906955421e-05f, 1.7141841226475663e-06f, -1.5931731468299404e-05f, -1.2289038896560669e+00f, 2.2925429511815310e-03f, -2.6713712140917778e-02f, 1.0393409058451653e-02f, -2.3640593838081259e-07f, -6.4811791489205461e-09f, -1.5972609901382384e-08f, 6.3321771449409425e-05f, 3.3199177096321364e-07f, -2.2320757580018835e-06f, 2.1212531464698259e-06f, 3.2101997931022197e-06f, 2.4232499526988249e-06f, -1.6747668496464030e-06f, 2.5793087843339890e-06f, 1.0641876997397048e-06f,
			3.2124631843544194e-07f, 1.9907241721739410e-07f, -8.2734487705238280e-07f, 3.9340920920949429e-07f, -1.2149831718488713e-06f, -2.1585645154118538e-02f, 1.8625639495439827e-04f, 7.2742921474855393e-05f, -6.6827487898990512e-04f, -5.8871690855610836e-10f, 1.3724940628456750e-10f, 4.4230644147269516e-10f, 3.3199174254150421e-07f, 9.9912736914120615e-05f, -4.0852743410368930e-08f, 3.9147689534502206e-09f, -6.9215992937188275e-08f, -7.0368152194077993e-08f, -3.8060299800690700e-08f, -7.5367509566603985e-08f, -2.3470493104582602e-08f,
			-1.5266426771631814e-06f, -9.4575864295620704e-07f, 3.9311885302595329e-06f, -2.5402030132681830e-06f, -4.5263977881404571e-06f, 2.8569309506565332e-03f, -1.1159088462591171e-03f, -3.2152952626347542e-03f, 6.4710817241575569e-05f, -1.5121218055469399e-08f, -3.4221950273582991e-10f, -5.4075111055595926e-10f, -2.2320757580018835e-06f, -4.0853198157719817e-08f, 9.9725519248750061e-05f, 1.3509078655715712e-07f, 2.0688474933194811e-07f, 1.5276552289833489e-07f, 1.1678861255859374e-07f, 1.4450354512973718e-07f, 6.5526265302651154e-08f,
			-2.1629472030326724e-04f, -1.3098870113026351e-04f, 5.3968053543940187e-04f, 3.3018324756994843e-05f, -9.4957060355227441e-05f, 9.7103714942932129e-01f, 6.1150070279836655e-02f, -2.1824543178081512e-01f, -7.4247317388653755e-03f, -5.2034918917343020e-04f, 6.8443268901319243e-06f, -1.1097737115051132e-05f, 2.1212676983850542e-06f, 3.9149239405844583e-09f, 1.3508970653219876e-07f, 4.6296231448650360e-03f, 2.0611056243069470e-05f, 4.2525040043983608e-05f, 2.1577127995442424e-07f, -7.3872811867659038e-08f, -1.5092286531626087e-08f,
			-7.4252998456358910e-04f, -4.6331965131685138e-04f, 1.9112931331619620e-03f, 8.6777326941955835e-05f, -2.9954549972899258e-04f, 2.1566607952117920e+00f, 1.7336939275264740e-01f, -7.1400731801986694e-01f, -1.0275604203343391e-02f, 6.8058116085012443e-06f, -3.6012479540659115e-05f, -1.9307142792968079e-05f, 3.2102047953230795e-06f, -6.9215957410051487e-08f, 2.0688821678049862e-07f, 2.0613559172488749e-05f, 7.3294462636113167e-03f, 1.3586098793894053e-04f, -6.8022117716282082e-08f, -2.7972839689027751e-07f, -1.3294845757627627e-07f,
			-7.4176222551614046e-04f, -4.6335012302733958e-04f, 1.9167029531672597e-03f, 6.8424029450397938e-05f, -2.4504301836714149e-04f, 1.5646151304244995e+00f, 1.4151699841022491e-01f, -6.0702317953109741e-01f, -5.3315144032239914e-03f, 1.0775971531984396e-05f, 2.1306335838744417e-05f, -3.4532517020124942e-05f, 2.4232460873463424e-06f, -7.0367747184718610e-08f, 1.5276349074611062e-07f, 4.2525905882939696e-05f, 1.3586239947471768e-04f, 7.2975791990756989e-03f, -1.0468691868936730e-07f, -2.2436098845446395e-07f, -1.0051029164515057e-07f,
			-1.0094192930409918e-06f, -6.2513299781130627e-07f, 2.5990807444031816e-06f, 1.4005166804054170e-07f, -1.7765549955583992e-06f, 3.3649958670139313e-02f, 3.1559099443256855e-04f, -1.8649055855348706e-03f, -2.0229858637321740e-04f, -2.5392036917537553e-08f, 1.0803027916672647e-10f, 1.0264853411712238e-09f, -1.6747667359595653e-06f, -3.8060306906118058e-08f, 1.1678861255859374e-07f, 2.1576983044724329e-07f, -6.8023823018847906e-08f, -1.0468918532069438e-07f, 7.5745538197224960e-07f, -6.3558040608313604e-08f, -2.3654614267343277e-08f,
			-1.1073992709498270e-06f, -6.8613178427767707e-07f, 2.8519511943159159e-06f, 8.4215862443670630e-06f, 1.0540741186559899e-06f, -1.8125375509262085e+00f, 1.7744023352861404e-02f, 5.1096803508698940e-03f, 4.4973086565732956e-02f, 9.6412726691141870e-09f, -3.4336727905426301e-10f, -8.1960782605250415e-10f, 2.5793096938286908e-06f, -7.5364297913438349e-08f, 1.4450402829879749e-07f, -7.3875582984328503e-08f, -2.7972924954156042e-07f, -2.2435951052557357e-07f, -6.3558047713740962e-08f, 9.9770812084898353e-05f, -8.9589754281860223e-08f,
			-6.2304775383381639e-07f, -3.8622121678599797e-07f, 1.6044961057559703e-06f, 8.1738044173107482e-06f, 2.5405054202565225e-06f, -1.8059216737747192e+00f, 1.4386827126145363e-02f, 9.4505259767174721e-03f, 4.6393062919378281e-02f, 2.6671636010888733e-09f, -6.5114730274373755e-10f, -2.2864601323391298e-09f, 1.0641880408002180e-06f, -2.3469929999464512e-08f, 6.5526677417437895e-08f, -1.5091567107106130e-08f, -1.3295007761371380e-07f, -1.0051021348544964e-07f, -2.3654633807268510e-08f, -8.9589192953098973e-08f, 9.9961427622474730e-05f};

	arm_mat_init_f32(&xMinusTrue, 22, 1, xMinusTrueData);
	arm_mat_init_f32(&PMinusTrue, 21, 21, PMinusDataTrue);

	bool test1 = false;
	bool test2 = false;

	test1 = areMatricesEqual(&xMinus, &xMinusTrue);
	test2 = areMatricesEqual(&PMinus, &PMinusTrue);

	bool test = test1 && test2;
	return test;
}

bool test_propogate(void) {

	arm_matrix_instance_f32 xPlus, P_plus, what, aHatN, wMeas, aMeas, Q;

		float32_t xData[22*1] = {-2.2884607315063477e-02,  9.1512638330459595e-01,
		        4.0011137723922729e-01, -4.3943215161561966e-02,
		        3.5394687652587891e+01, -1.1787300109863281e+02,
		        2.8781792968750000e+04, -1.2077078819274902e+01,
		        5.7730107307434082e+00,  1.1333886718750000e+02,
		       -2.7942578890360892e-04, -2.1035106328781694e-04,
		       -2.6591881760396063e-04,  8.8350940495729446e-03,
		        1.6256757080554962e-03,  1.9927009998355061e-04,
		        2.1494920656550676e-04, -1.0350634111091495e-03,
		       -8.9672525064088404e-05,  1.5854457160457969e-03,
		        1.0850373655557632e-03,  4.6325451694428921e-04};

		float32_t PData[21*21] = {5.0389925017952919e-03f, 3.1386734917759895e-03f, -1.2951527722179890e-02f, -8.0738391261547804e-04f, 2.5598644278943539e-03f, -2.1820283889770508e+01f, -1.5453934669494629e+00f, 5.9288525581359863e+00f, 1.3891488313674927e-01f, -2.0048216811119346e-06f, -3.2393774745287374e-05f, 4.9889800720848143e-05f, -1.5646855899831280e-05f, 3.2121343451763096e-07f, -1.5264936337189283e-06f, -2.1615074365399778e-04f, -7.4237369699403644e-04f, -7.4153754394501448e-04f, -1.0093436912939069e-06f, -1.1073028645114391e-06f, -6.2298909142555203e-07f,
				3.1386725604534149e-03f, 1.9648219458758831e-03f, -8.0804517492651939e-03f, -5.0261052092537284e-04f, 1.5936470590531826e-03f, -1.3582573890686035e+01f, -9.6213459968566895e-01f, 3.6917583942413330e+00f, 8.6432509124279022e-02f, -5.9811674191223574e-07f, -2.2610181986237876e-05f, 3.1977098842617124e-05f, -9.7411066235508770e-06f, 2.0000864253688633e-07f, -9.5021465540412464e-07f, -1.3153281179256737e-04f, -4.6551285777240992e-04f, -4.6530558029189706e-04f, -6.2809255041429424e-07f, -6.8936219577153679e-07f, -3.8803003121756774e-07f,
				-1.2951526790857315e-02f, -8.0804536119103432e-03f, 3.3347148448228836e-02f, 2.0787562243640423e-03f, -6.5908450633287430e-03f, 5.6180130004882812e+01f, 3.9788146018981934e+00f, -1.5264608383178711e+01f, -3.5766848921775818e-01f, 1.3322169252205640e-06f, 8.3474267739802599e-05f, -1.2983183842152357e-04f, 4.0285089198732749e-05f, -8.2711824234138476e-07f, 3.9301376091316342e-06f, 5.3943332750350237e-04f, 1.9106010440737009e-03f, 1.9155526533722878e-03f, 2.5984440981119405e-06f, 2.8512238259281730e-06f, 1.6040754644564004e-06f,
				-8.0738525139167905e-04f, -5.0261139404028654e-04f, 2.0787597168236971e-03f, 1.7619828577153385e-04f, -5.2187195979058743e-04f, 4.7090721130371094e+00f, 3.2039403915405273e-01f, -1.1370785236358643e+00f, -3.0653338879346848e-02f, -3.4618537938513327e-06f, -1.5313280243844929e-07f, -4.3224565615673782e-07f, 1.7139997225967818e-06f, 3.9339300883511896e-07f, -2.5401027414773125e-06f, 3.3012915082508698e-05f, 8.6762891442049295e-05f, 6.8411834945436567e-05f, 1.4002503689880541e-07f, 8.4199955381336622e-06f, 8.1725174823077396e-06f,
				2.5598646607249975e-03f, 1.5936469426378608e-03f, -6.5908445976674557e-03f, -5.2187172695994377e-04f, 1.6188059234991670e-03f, -1.4437356948852539e+01f, -9.6182018518447876e-01f, 3.5346310138702393e+00f, 1.0662219673395157e-01f, 9.6007197498693131e-06f, 3.1550774792776792e-07f, 1.9050106629947550e-06f, -1.5928870197967626e-05f, -1.2149911299275118e-06f, -4.5260462684382219e-06f, -9.4931870989967138e-05f, -2.9946854920126498e-04f, -2.4497826234437525e-04f, -1.7763567257134127e-06f, 1.0535148931012372e-06f, 2.5394670046807732e-06f,
				-2.1820224761962891e+01f, -1.3582537651062012e+01f, 5.6179962158203125e+01f, 4.7090630531311035e+00f, -1.4437336921691895e+01f, 2.3969992187500000e+05f, 8.5390068359375000e+03f, -3.1784367187500000e+04f, -3.0329980468750000e+03f, -1.0520824044942856e-01f, -6.2867989763617516e-03f, -6.0234428383409977e-03f, -1.2287986278533936e+00f, -2.1592376753687859e-02f, 2.8575158212333918e-03f, 9.7096240520477295e-01f, 2.1565728187561035e+00f, 1.5645617246627808e+00f, 3.3647991716861725e-02f, -1.8120876550674438e+00f, -1.8054577112197876e+00f,
				-1.5453902482986450e+00f, -9.6213251352310181e-01f, 3.9788067340850830e+00f, 3.2039386034011841e-01f, -9.6181970834732056e-01f, 8.5390214843750000e+03f, 5.9188671875000000e+02f, -2.1271850585937500e+03f, -5.3193809509277344e+01f, -6.0443156398832798e-03f, 2.8295663651078939e-04f, -1.5286422567442060e-03f, 2.2926249075680971e-03f, 1.8699679640121758e-04f, -1.1160597205162048e-03f, 6.1143510043621063e-02f, 1.7335088551044464e-01f, 1.4149878919124603e-01f, 3.1555970781482756e-04f, 1.7744706943631172e-02f, 1.4385745860636234e-02f,
				5.9288649559020996e+00f, 3.6917665004730225e+00f, -1.5264640808105469e+01f, -1.1370774507522583e+00f, 3.5346331596374512e+00f, -3.1784408203125000e+04f, -2.1271867675781250e+03f, 7.9166689453125000e+03f, 2.2918943786621094e+02f, 1.9937748089432716e-02f, -2.8586506377905607e-03f, 8.6001874879002571e-03f, -2.6711093261837959e-02f, 7.2020280640572309e-05f, -3.2150780316442251e-03f, -2.1821407973766327e-01f, -7.1389967203140259e-01f, -6.0691767930984497e-01f, -1.8647768301889300e-03f, 5.1091467030346394e-03f, 9.4506256282329559e-03f,
				1.3891413807868958e-01f, 8.6432047188282013e-02f, -3.5766661167144775e-01f, -3.0653275549411774e-02f, 1.0662207007408142e-01f, -3.0329960937500000e+03f, -5.3193630218505859e+01f, 2.2918931579589844e+02f, 5.3554500579833984e+01f, 9.2046259669587016e-04f, 1.8295941117685288e-04f, -1.8548936350271106e-04f, 1.0393148288130760e-02f, -6.6834740573540330e-04f, 6.3706625951454043e-05f, -7.4256197549402714e-03f, -1.0279154404997826e-02f, -5.3347381763160229e-03f, -2.0230085647199303e-04f, 4.4972959905862808e-02f, 4.6382255852222443e-02f,
				-2.0045276869495865e-06f, -5.9798333040816942e-07f, 1.3315027445059968e-06f, -3.4619006328284740e-06f, 9.6003741418826394e-06f, -1.0520680993795395e-01f, -6.0441894456744194e-03f, 1.9937917590141296e-02f, 9.2044891789555550e-04f, 6.7339438828639686e-05f, 1.6288674942188663e-06f, -2.6550769689492881e-06f, -2.3640905055799522e-07f, -5.8867838381715387e-10f, -1.5121122132200071e-08f, -5.2034913096576929e-04f, 6.8058338911214378e-06f, 1.0775735063361935e-05f, -2.5391910796201955e-08f, 9.6409458194557374e-09f, 2.6672950514949889e-09f,
				-3.2393250876339152e-05f, -2.2609890947933309e-05f, 8.3472943515516818e-05f, -1.5329075608860876e-07f, 3.1558107593809837e-07f, -6.2814927659928799e-03f, 2.8312802896834910e-04f, -2.8582934755831957e-03f, 1.8291932065039873e-04f, 1.6288963706756476e-06f, 9.1781666924362071e-06f, -5.4072875173005741e-06f, -6.4835772306537365e-09f, 1.3737452841944275e-10f, -3.4200661747085803e-10f, 6.8448762249317952e-06f, -3.6012610507896170e-05f, 2.1306108465068974e-05f, 1.0799910965531012e-10f, -3.4312130914315730e-10f, -6.5111810387818991e-10f,
				4.9890528316609561e-05f, 3.1977557227946818e-05f, -1.2983367196284235e-04f, -4.3230005530858762e-07f, 1.9047888599743601e-06f, -6.0257846489548683e-03f, -1.5284898690879345e-03f, 8.6009548977017403e-03f, -1.8544698832556605e-04f, -2.6551272185315611e-06f, -5.4073366300144698e-06f, 1.4393031051440630e-05f, -1.5971821198945690e-08f, 4.4223869011261741e-10f, -5.4078402866863939e-10f, -1.1097878086729906e-05f, -1.9307506590848789e-05f, -3.4532658901298419e-05f, 1.0265270855569497e-09f, -8.1956336162036791e-10f, -2.2865120907766823e-09f,
				-1.5646786778233945e-05f, -9.7410629678051919e-06f, 4.0284918213728815e-05f, 1.7139785768449656e-06f, -1.5928804714349099e-05f, -1.2287999391555786e+00f, 2.2925895173102617e-03f, -2.6710949838161469e-02f, 1.0393155738711357e-02f, -2.3640593838081259e-07f, -6.4811791489205461e-09f, -1.5972609901382384e-08f, 6.3321771449409425e-05f, 3.3199177096321364e-07f, -2.2320757580018835e-06f, 2.1212531464698259e-06f, 3.2101997931022197e-06f, 2.4232499526988249e-06f, -1.6747668496464030e-06f, 2.5793087843339890e-06f, 1.0641876997397048e-06f,
				3.2121482718139305e-07f, 2.0000949518816924e-07f, -8.2712182347677299e-07f, 3.9339244040093035e-07f, -1.2149910162406741e-06f, -2.1592328324913979e-02f, 1.8699542852118611e-04f, 7.2021044616121799e-05f, -6.6834874451160431e-04f, -5.8871690855610836e-10f, 1.3724940628456750e-10f, 4.4230644147269516e-10f, 3.3199174254150421e-07f, 9.9912736914120615e-05f, -4.0852743410368930e-08f, 3.9147689534502206e-09f, -6.9215992937188275e-08f, -7.0368152194077993e-08f, -3.8060299800690700e-08f, -7.5367509566603985e-08f, -2.3470493104582602e-08f,
				-1.5264979538187617e-06f, -9.5021732704481110e-07f, 3.9301489778154064e-06f, -2.5401029688509880e-06f, -4.5260453589435201e-06f, 2.8575679752975702e-03f, -1.1160590220242739e-03f, -3.2150766346603632e-03f, 6.3705869251862168e-05f, -1.5121218055469399e-08f, -3.4221950273582991e-10f, -5.4075111055595926e-10f, -2.2320757580018835e-06f, -4.0853198157719817e-08f, 9.9725519248750061e-05f, 1.3509078655715712e-07f, 2.0688474933194811e-07f, 1.5276552289833489e-07f, 1.1678861255859374e-07f, 1.4450354512973718e-07f, 6.5526265302651154e-08f,
				-2.1614816796500236e-04f, -1.3153137115295976e-04f, 5.3942616796121001e-04f, 3.3012838684953749e-05f, -9.4933151558507234e-05f, 9.7096288204193115e-01f, 6.1144329607486725e-02f, -2.1821361780166626e-01f, -7.4256518855690956e-03f, -5.2034918917343020e-04f, 6.8443268901319243e-06f, -1.1097737115051132e-05f, 2.1212676983850542e-06f, 3.9149239405844583e-09f, 1.3508970653219876e-07f, 4.6292310580611229e-03f, 2.0611056243069470e-05f, 4.2525040043983608e-05f, 2.1577127995442424e-07f, -7.3872811867659038e-08f, -1.5092286531626087e-08f,
				-7.4237032094970345e-04f, -4.6551087871193886e-04f, 1.9105921965092421e-03f, 8.6761770944576710e-05f, -2.9946726863272488e-04f, 2.1565580368041992e+00f, 1.7335081100463867e-01f, -7.1389639377593994e-01f, -1.0278771631419659e-02f, 6.8058116085012443e-06f, -3.6012479540659115e-05f, -1.9307142792968079e-05f, 3.2102047953230795e-06f, -6.9215957410051487e-08f, 2.0688821678049862e-07f, 2.0613559172488749e-05f, 7.3290541768074036e-03f, 1.3586098793894053e-04f, -6.8022117716282082e-08f, -2.7972839689027751e-07f, -1.3294845757627627e-07f,
				-7.4153725290670991e-04f, -4.6530526014976203e-04f, 1.9155518384650350e-03f, 6.8411332904361188e-05f, -2.4497651611454785e-04f, 1.5645618438720703e+00f, 1.4149840176105499e-01f, -6.0691231489181519e-01f, -5.3345630876719952e-03f, 1.0775971531984396e-05f, 2.1306335838744417e-05f, -3.4532517020124942e-05f, 2.4232460873463424e-06f, -7.0367747184718610e-08f, 1.5276349074611062e-07f, 4.2525905882939696e-05f, 1.3586239947471768e-04f, 7.2971871122717857e-03f, -1.0468691868936730e-07f, -2.2436098845446395e-07f, -1.0051029164515057e-07f,
				-1.0093333457916742e-06f, -6.2808601342112524e-07f, 2.5984172680182382e-06f, 1.4002334580709430e-07f, -1.7763507003110135e-06f, 3.3647935837507248e-02f, 3.1555656460113823e-04f, -1.8647639080882072e-03f, -2.0230076916050166e-04f, -2.5392036917537553e-08f, 1.0803027916672647e-10f, 1.0264853411712238e-09f, -1.6747667359595653e-06f, -3.8060306906118058e-08f, 1.1678861255859374e-07f, 2.1576983044724329e-07f, -6.8023823018847906e-08f, -1.0468918532069438e-07f, 7.5745538197224960e-07f, -6.3558040608313604e-08f, -2.3654614267343277e-08f,
				-1.1073016139562242e-06f, -6.8936111574657843e-07f, 2.8512201879493659e-06f, 8.4199946286389604e-06f, 1.0535139836065355e-06f, -1.8120877742767334e+00f, 1.7744705080986023e-02f, 5.1091457717120647e-03f, 4.4972959905862808e-02f, 9.6412726691141870e-09f, -3.4336727905426301e-10f, -8.1960782605250415e-10f, 2.5793096938286908e-06f, -7.5364297913438349e-08f, 1.4450402829879749e-07f, -7.3875582984328503e-08f, -2.7972924954156042e-07f, -2.2435951052557357e-07f, -6.3558047713740962e-08f, 9.9770812084898353e-05f, -8.9589754281860223e-08f,
				-6.2298846614794456e-07f, -3.8802963331363571e-07f, 1.6040736454669968e-06f, 8.1725138443289325e-06f, 2.5394695057912031e-06f, -1.8054578304290771e+00f, 1.4385740272700787e-02f, 9.4506312161684036e-03f, 4.6382255852222443e-02f, 2.6671636010888733e-09f, -6.5114730274373755e-10f, -2.2864601323391298e-09f, 1.0641880408002180e-06f, -2.3469929999464512e-08f, 6.5526677417437895e-08f, -1.5091567107106130e-08f, -1.3295007761371380e-07f, -1.0051021348544964e-07f, -2.3654633807268510e-08f, -8.9589192953098973e-08f, 9.9961427622474730e-05f};

		float32_t aMeasData[3] = {0.4731085002422333,  0.9613523483276367, 10.812639236450195};

		float32_t dt = 0.0100;

		float32_t wHatData[3] = {0.1150641068816185,  0.0045749028213322, -0.0042020138353109};

		float32_t aHatNData[3] = {-0.0536695718765259,  -0.237719401717186 , -10.857034683227539};

		float32_t wMeasData[3] = {0.1148542687296867,  0.0044058579951525, -0.0044308393262327};

		float32_t QBuff[12*12] = {2.0943951312801801e-05f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
				0.0000000000000000e+00f, 2.0943951312801801e-05f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
				0.0000000000000000e+00f, 0.0000000000000000e+00f, 2.0943951312801801e-05f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
				0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 1.4544409623340471e-06f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
				0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 1.4544409623340471e-06f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
				0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 1.4544409623340471e-06f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
				0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 1.9619999511633068e-04f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
				0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 1.9619999511633068e-04f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
				0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 1.9619999511633068e-04f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
				0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 3.9199996535899118e-05f, 0.0000000000000000e+00f, 0.0000000000000000e+00f,
				0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 3.9199996535899118e-05f, 0.0000000000000000e+00f,
				0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 0.0000000000000000e+00f, 3.9199996535899118e-05f};

		arm_matrix_instance_f32 xMinus, PMinus;

		float32_t xMinusData[22];
		float32_t PMinusData[21*21];

		arm_mat_init_f32(&xPlus, 22, 1, xData);
		arm_mat_init_f32(&P_plus, 21, 21, PData);
		arm_mat_init_f32(&what, 3, 1, wHatData);
		arm_mat_init_f32(&aHatN, 3, 1, aHatNData);
		arm_mat_init_f32(&wMeas, 3, 1, wMeasData);
		arm_mat_init_f32(&aMeas, 3, 1, aMeasData);
		arm_mat_init_f32(&Q, 12, 12, QBuff);

		propogate(&xPlus, &P_plus, &what,
				  &aHatN, &wMeas, &aMeas, &Q,
				  dt, we, &xMinus, &PMinus,
				  xMinusData, PMinusData);

		arm_matrix_instance_f32 xMinusTrue, PMinusTrue;

		float32_t xMinusTrueData[22*1] = {-2.3421168327331543e-02,  9.1510558128356934e-01,
		        4.0010470151901245e-01, -4.4151984155178070e-02,
		        3.5394687652587891e+01, -1.1787300109863281e+02,
		        2.8780660156250000e+04, -1.2077622413635254e+01,
		        5.7707591056823730e+00,  1.1332737731933594e+02,
		       -2.7942578890360892e-04, -2.1035106328781694e-04,
		       -2.6591881760396063e-04,  8.8350940495729446e-03,
		        1.6256757080554962e-03,  1.9927009998355061e-04,
		        2.1494920656550676e-04, -1.0350634111091495e-03,
		       -8.9672525064088404e-05,  1.5854457160457969e-03,
		        1.0850373655557632e-03,  4.6325451694428921e-04};

		float32_t PMinusDataTrue[21*21] = {5.0406577065587044e-03f, 3.1247714068740606e-03f, -1.2957259081304073e-02f, -8.0759939737617970e-04f, 2.5607603602111340e-03f, -2.1823722839355469e+01f, -1.5456709861755371e+00f, 5.9301953315734863e+00f, 1.3890400528907776e-01f, -2.0790409962501144e-06f, -3.2420852221548557e-05f, 4.9933780246647075e-05f, -1.5648356566089205e-05f, 3.2124492577167985e-07f, -1.5266383570633479e-06f, -2.1629729599226266e-04f, -7.4253336060792208e-04f, -7.4176251655444503e-04f, -1.0094296385432244e-06f, -1.1074005215050420e-06f, -6.2304837911142386e-07f,
				3.1247704755514860e-03f, 1.9471790874376893e-03f, -8.0459732562303543e-03f, -5.0033902516588569e-04f, 1.5865787863731384e-03f, -1.3519684791564941e+01f, -9.5770603418350220e-01f, 3.6749567985534668e+00f, 8.6010761559009552e-02f, -6.1330564449235681e-07f, -2.2605656340601854e-05f, 3.1884741474641487e-05f, -9.6954390755854547e-06f, 1.9907156456611119e-07f, -9.4575597131552058e-07f, -1.3099014176987112e-04f, -4.6332163037732244e-04f, -4.6335044316947460e-04f, -6.2513953480447526e-07f, -6.8613286430263543e-07f, -3.8622161468992999e-07f,
				-1.2957258149981499e-02f, -8.0459751188755035e-03f, 3.3367548137903214e-02f, 2.0796626340597868e-03f, -6.5942662768065929e-03f, 5.6198486328125000e+01f, 3.9801986217498779e+00f, -1.5270633697509766e+01f, -3.5770139098167419e-01f, 1.3597984889202053e-06f, 8.3553764852695167e-05f, -1.3001172919757664e-04f, 4.0295864891959354e-05f, -8.2734129591699457e-07f, 3.9311771615757607e-06f, 5.3968769498169422e-04f, 1.9113019807264209e-03f, 1.9167037680745125e-03f, 2.5991075744968839e-06f, 2.8519548322947230e-06f, 1.6044979247453739e-06f,
				-8.0760073615238070e-04f, -5.0033989828079939e-04f, 2.0796661265194416e-03f, 1.7625578038860112e-04f, -5.2208284614607692e-04f, 4.7101449966430664e+00f, 3.2046738266944885e-01f, -1.1373896598815918e+00f, -3.0653392896056175e-02f, -3.4623960800672648e-06f, -1.5310742185192794e-07f, -4.3238281932644895e-07f, 1.7142052683993825e-06f, 3.9340977764368290e-07f, -2.5402027858945075e-06f, 3.3018401154549792e-05f, 8.6778447439428419e-05f, 6.8424531491473317e-05f, 1.4005335913225281e-07f, 8.4215871538617648e-06f, 8.1738080552895553e-06f,
				2.5607605930417776e-03f, 1.5865786699578166e-03f, -6.5942658111453056e-03f, -5.2208261331543326e-04f, 1.6195805510506034e-03f, -1.4441905975341797e+01f, -9.6211731433868408e-01f, 3.5358800888061523e+00f, 1.0663539171218872e-01f, 9.6029043561429717e-06f, 3.1519448384642601e-07f, 1.9059531268794672e-06f, -1.5931796951917931e-05f, -1.2149832855357090e-06f, -4.5263986976351589e-06f, -9.4955779786687344e-05f, -2.9954678029753268e-04f, -2.4504476459696889e-04f, -1.7765610209607985e-06f, 1.0540750281506917e-06f, 2.5405029191460926e-06f,
				-2.1823663711547852e+01f, -1.3519648551940918e+01f, 5.6198318481445312e+01f, 4.7101359367370605e+00f, -1.4441885948181152e+01f, 2.3976057812500000e+05f, 8.5400859375000000e+03f, -3.1789914062500000e+04f, -3.0336279296875000e+03f, -1.0521744191646576e-01f, -6.2886285595595837e-03f, -6.0215881094336510e-03f, -1.2289025783538818e+00f, -2.1585693582892418e-02f, 2.8568787965923548e-03f, 9.7103667259216309e-01f, 2.1566755771636963e+00f, 1.5646151304244995e+00f, 3.3650014549493790e-02f, -1.8125374317169189e+00f, -1.8059215545654297e+00f,
				-1.5456677675247192e+00f, -9.5770394802093506e-01f, 3.9801907539367676e+00f, 3.2046720385551453e-01f, -9.6211683750152588e-01f, 8.5401005859375000e+03f, 5.9196417236328125e+02f, -2.1275637207031250e+03f, -5.3188266754150391e+01f, -6.0441866517066956e-03f, 2.8362125158309937e-04f, -1.5298675280064344e-03f, 2.2925783414393663e-03f, 1.8625776283442974e-04f, -1.1159095447510481e-03f, 6.1149250715970993e-02f, 1.7336946725845337e-01f, 1.4151738584041595e-01f, 3.1559413764625788e-04f, 1.7744025215506554e-02f, 1.4386832714080811e-02f,
				5.9302077293395996e+00f, 3.6749649047851562e+00f, -1.5270666122436523e+01f, -1.1373885869979858e+00f, 3.5358822345733643e+00f, -3.1789955078125000e+04f, -2.1275654296875000e+03f, 7.9184365234375000e+03f, 2.2918354797363281e+02f, 1.9937541335821152e-02f, -2.8636730276048183e-03f, 8.6077097803354263e-03f, -2.6713855564594269e-02f, 7.2742157499305904e-05f, -3.2152966596186161e-03f, -2.1824589371681213e-01f, -7.1401059627532959e-01f, -6.0702854394912720e-01f, -1.8649185076355934e-03f, 5.1096812821924686e-03f, 9.4505203887820244e-03f,
				1.3890326023101807e-01f, 8.6010299623012543e-02f, -3.5769951343536377e-01f, -3.0653329566121101e-02f, 1.0663526505231857e-01f, -3.0336259765625000e+03f, -5.3188087463378906e+01f, 2.2918342590332031e+02f, 5.3563293457031250e+01f, 9.2044385382905602e-04f, 1.8306958372704685e-04f, -1.8565870414022356e-04f, 1.0393401607871056e-02f, -6.6827354021370411e-04f, 6.4711573941167444e-05f, -7.4246996082365513e-03f, -1.0275986976921558e-02f, -5.3316894918680191e-03f, -2.0229867368470877e-04f, 4.4973086565732956e-02f, 4.6393062919378281e-02f,
				-2.0787470020877663e-06f, -6.1317274457906024e-07f, 1.3590836260846118e-06f, -3.4624429190444062e-06f, 9.6025587481562980e-06f, -1.0521601140499115e-01f, -6.0440604574978352e-03f, 1.9937710836529732e-02f, 9.2043017502874136e-04f, 6.7353983467910439e-05f, 1.6288674942188663e-06f, -2.6550769689492881e-06f, -2.3640905055799522e-07f, -5.8867838381715387e-10f, -1.5121122132200071e-08f, -5.2034913096576929e-04f, 6.8058338911214378e-06f, 1.0775735063361935e-05f, -2.5391910796201955e-08f, 9.6409458194557374e-09f, 2.6672950514949889e-09f,
				-3.2420328352600336e-05f, -2.2605367121286690e-05f, 8.3552440628409386e-05f, -1.5326534708037798e-07f, 3.1526786870017531e-07f, -6.2833218835294247e-03f, 2.8379261493682861e-04f, -2.8633156325668097e-03f, 1.8302949320059270e-04f, 1.6288963706756476e-06f, 9.1927113317069598e-06f, -5.4072875173005741e-06f, -6.4835772306537365e-09f, 1.3737452841944275e-10f, -3.4200661747085803e-10f, 6.8448762249317952e-06f, -3.6012610507896170e-05f, 2.1306108465068974e-05f, 1.0799910965531012e-10f, -3.4312130914315730e-10f, -6.5111810387818991e-10f,
				4.9934507842408493e-05f, 3.1885196221992373e-05f, -1.3001356273889542e-04f, -4.3243719005658932e-07f, 1.9057313238590723e-06f, -6.0239303857088089e-03f, -1.5297152567654848e-03f, 8.6084771901369095e-03f, -1.8561632896307856e-04f, -2.6551272185315611e-06f, -5.4073366300144698e-06f, 1.4407575690711383e-05f, -1.5971821198945690e-08f, 4.4223869011261741e-10f, -5.4078402866863939e-10f, -1.1097878086729906e-05f, -1.9307506590848789e-05f, -3.4532658901298419e-05f, 1.0265270855569497e-09f, -8.1956336162036791e-10f, -2.2865120907766823e-09f,
				-1.5648287444491871e-05f, -9.6953954198397696e-06f, 4.0295693906955421e-05f, 1.7141841226475663e-06f, -1.5931731468299404e-05f, -1.2289038896560669e+00f, 2.2925429511815310e-03f, -2.6713712140917778e-02f, 1.0393409058451653e-02f, -2.3640593838081259e-07f, -6.4811791489205461e-09f, -1.5972609901382384e-08f, 6.3321771449409425e-05f, 3.3199177096321364e-07f, -2.2320757580018835e-06f, 2.1212531464698259e-06f, 3.2101997931022197e-06f, 2.4232499526988249e-06f, -1.6747668496464030e-06f, 2.5793087843339890e-06f, 1.0641876997397048e-06f,
				3.2124631843544194e-07f, 1.9907241721739410e-07f, -8.2734487705238280e-07f, 3.9340920920949429e-07f, -1.2149831718488713e-06f, -2.1585645154118538e-02f, 1.8625639495439827e-04f, 7.2742921474855393e-05f, -6.6827487898990512e-04f, -5.8871690855610836e-10f, 1.3724940628456750e-10f, 4.4230644147269516e-10f, 3.3199174254150421e-07f, 9.9912736914120615e-05f, -4.0852743410368930e-08f, 3.9147689534502206e-09f, -6.9215992937188275e-08f, -7.0368152194077993e-08f, -3.8060299800690700e-08f, -7.5367509566603985e-08f, -2.3470493104582602e-08f,
				-1.5266426771631814e-06f, -9.4575864295620704e-07f, 3.9311885302595329e-06f, -2.5402030132681830e-06f, -4.5263977881404571e-06f, 2.8569309506565332e-03f, -1.1159088462591171e-03f, -3.2152952626347542e-03f, 6.4710817241575569e-05f, -1.5121218055469399e-08f, -3.4221950273582991e-10f, -5.4075111055595926e-10f, -2.2320757580018835e-06f, -4.0853198157719817e-08f, 9.9725519248750061e-05f, 1.3509078655715712e-07f, 2.0688474933194811e-07f, 1.5276552289833489e-07f, 1.1678861255859374e-07f, 1.4450354512973718e-07f, 6.5526265302651154e-08f,
				-2.1629472030326724e-04f, -1.3098870113026351e-04f, 5.3968053543940187e-04f, 3.3018324756994843e-05f, -9.4957060355227441e-05f, 9.7103714942932129e-01f, 6.1150070279836655e-02f, -2.1824543178081512e-01f, -7.4247317388653755e-03f, -5.2034918917343020e-04f, 6.8443268901319243e-06f, -1.1097737115051132e-05f, 2.1212676983850542e-06f, 3.9149239405844583e-09f, 1.3508970653219876e-07f, 4.6296231448650360e-03f, 2.0611056243069470e-05f, 4.2525040043983608e-05f, 2.1577127995442424e-07f, -7.3872811867659038e-08f, -1.5092286531626087e-08f,
				-7.4252998456358910e-04f, -4.6331965131685138e-04f, 1.9112931331619620e-03f, 8.6777326941955835e-05f, -2.9954549972899258e-04f, 2.1566607952117920e+00f, 1.7336939275264740e-01f, -7.1400731801986694e-01f, -1.0275604203343391e-02f, 6.8058116085012443e-06f, -3.6012479540659115e-05f, -1.9307142792968079e-05f, 3.2102047953230795e-06f, -6.9215957410051487e-08f, 2.0688821678049862e-07f, 2.0613559172488749e-05f, 7.3294462636113167e-03f, 1.3586098793894053e-04f, -6.8022117716282082e-08f, -2.7972839689027751e-07f, -1.3294845757627627e-07f,
				-7.4176222551614046e-04f, -4.6335012302733958e-04f, 1.9167029531672597e-03f, 6.8424029450397938e-05f, -2.4504301836714149e-04f, 1.5646151304244995e+00f, 1.4151699841022491e-01f, -6.0702317953109741e-01f, -5.3315144032239914e-03f, 1.0775971531984396e-05f, 2.1306335838744417e-05f, -3.4532517020124942e-05f, 2.4232460873463424e-06f, -7.0367747184718610e-08f, 1.5276349074611062e-07f, 4.2525905882939696e-05f, 1.3586239947471768e-04f, 7.2975791990756989e-03f, -1.0468691868936730e-07f, -2.2436098845446395e-07f, -1.0051029164515057e-07f,
				-1.0094192930409918e-06f, -6.2513299781130627e-07f, 2.5990807444031816e-06f, 1.4005166804054170e-07f, -1.7765549955583992e-06f, 3.3649958670139313e-02f, 3.1559099443256855e-04f, -1.8649055855348706e-03f, -2.0229858637321740e-04f, -2.5392036917537553e-08f, 1.0803027916672647e-10f, 1.0264853411712238e-09f, -1.6747667359595653e-06f, -3.8060306906118058e-08f, 1.1678861255859374e-07f, 2.1576983044724329e-07f, -6.8023823018847906e-08f, -1.0468918532069438e-07f, 7.5745538197224960e-07f, -6.3558040608313604e-08f, -2.3654614267343277e-08f,
				-1.1073992709498270e-06f, -6.8613178427767707e-07f, 2.8519511943159159e-06f, 8.4215862443670630e-06f, 1.0540741186559899e-06f, -1.8125375509262085e+00f, 1.7744023352861404e-02f, 5.1096803508698940e-03f, 4.4973086565732956e-02f, 9.6412726691141870e-09f, -3.4336727905426301e-10f, -8.1960782605250415e-10f, 2.5793096938286908e-06f, -7.5364297913438349e-08f, 1.4450402829879749e-07f, -7.3875582984328503e-08f, -2.7972924954156042e-07f, -2.2435951052557357e-07f, -6.3558047713740962e-08f, 9.9770812084898353e-05f, -8.9589754281860223e-08f,
				-6.2304775383381639e-07f, -3.8622121678599797e-07f, 1.6044961057559703e-06f, 8.1738044173107482e-06f, 2.5405054202565225e-06f, -1.8059216737747192e+00f, 1.4386827126145363e-02f, 9.4505259767174721e-03f, 4.6393062919378281e-02f, 2.6671636010888733e-09f, -6.5114730274373755e-10f, -2.2864601323391298e-09f, 1.0641880408002180e-06f, -2.3469929999464512e-08f, 6.5526677417437895e-08f, -1.5091567107106130e-08f, -1.3295007761371380e-07f, -1.0051021348544964e-07f, -2.3654633807268510e-08f, -8.9589192953098973e-08f, 9.9961427622474730e-05f};

		arm_mat_init_f32(&xMinusTrue, 22, 1, xMinusTrueData);
		arm_mat_init_f32(&PMinusTrue, 21, 21, PMinusDataTrue);

		bool test1 = false;
		bool test2 = false;

		test1 = areMatricesEqual(&xMinus, &xMinusTrue);
		test2 = areMatricesEqual(&PMinus, &PMinusTrue);

		bool test = (test1 && test2);
		return test;
}

bool test_right_divide(void) {

	float32_t BData[21*3] = {
	     0.064f,   0.1975f,   0.2860f,
	     0.0675f, -0.0845f,   0.1455f,
	    -0.1575f,  0.0635f,   0.0810f,
	     0.4125f, -0.3350f,  -0.0215f,
	     0.1224f,  0.1260f,  -0.0762f,
	    -0.1695f,  0.0665f,   0.2025f,
	    -0.1035f,  0.1520f,  -0.0945f,
	     0.0730f, -0.1185f,   0.1520f,
	     0.2245f, -0.0100f,   0.0625f,
	    -0.0715f,  0.1880f,  -0.0165f,
	     0.1930f, -0.1475f,   0.2090f,
	     0.0070f,  0.0175f,  -0.1565f,
	    -0.0915f,  0.0895f,   0.0730f,
	     0.1105f, -0.0750f,   0.1455f,
	    -0.1620f,  0.0375f,  -0.0285f,
	     0.2885f, -0.2260f,   0.0845f,
	    -0.0280f,  0.0825f,   0.1180f,
	     0.0145f,  0.0095f,  -0.0800f,
	     0.1610f, -0.0430f,   0.0305f,
	    -0.0115f,  0.0600f,   0.0085f,
	     0.1405f, -0.0890f,  -0.0100f
	};


	float32_t AData[9] = {
	     1.0f,   0.2f,  -0.1f,
	    -0.3f,   0.9f,   0.05f,
	     0.15f, -0.25f,  0.8f
	};


	float32_t xRealData[21*3] = {0.099747779,0.29492703,0.35153553,
			0.025126904,-0.047258883,0.18796954,
			-0.13263008,0.12144036,0.077081218,
			0.27986516,-0.42478744,0.03465736,
			0.15915609,0.082271574,-0.080497462,
			-0.15209391,0.16977157,0.22350254,
			-0.041273794,0.14136104,-0.13211929,
			0.019065673,-0.081056472,0.19744924,
			0.20078839,-0.026595812,0.10488579,
			-0.0061595812,0.20082805,-0.033946701,
			0.11716529,-0.1113547,0.28285533,
			0.023919734,-0.038708756,-0.19021574,
			-0.062503173,0.13418147,0.075050761,
			0.06808217,-0.044800127,0.19318528,
			-0.13664181,0.056411802,-0.056230964,
			0.19076301,-0.25314404,0.14529188,
			-0.0091005711,0.13205266,0.13810914,
			0.022553934,-0.021085025,-0.095862944,
			0.13383249,-0.061218274,0.058680203,
			0.0075079315,0.06704632,0.0073730964,
			0.10341212,-0.11967322,0.0079060914
			};

	float32_t xTestData[21*3];

	arm_matrix_instance_f32 A, B, xReal, xTest;

	arm_mat_init_f32(&A, 3, 3, AData);
	arm_mat_init_f32(&B, 21, 3, BData);
	arm_mat_init_f32(&xReal, 21, 3, xRealData);

	arm_mat_linsolve_right_f32(&A, &B, &xTest, xTestData);

	bool test = areMatricesEqual(&xTest, &xReal);
	return test;
}

// Something wrong here
bool test_update_GPS(void) {

	arm_matrix_instance_f32 xMinus, P_minus, Pq_minus, H, R, lla_meas;

	float32_t xMinusData[22*1] = {-2.3421935737133026e-02,  9.1510498523712158e-01,
									4.0010610222816467e-01, -4.4151850044727325e-02,
									3.5394710540771484e+01, -1.1787292480468750e+02,
									2.8780652343750000e+04, -1.2077791213989258e+01,
									5.7740802764892578e+00,  1.1332697296142578e+02,
								   -2.7941286680288613e-04, -2.1034452947787941e-04,
								   -2.6590019115246832e-04,  8.8367471471428871e-03,
									1.6264109872281551e-03,  2.0100758410990238e-04,
									2.1478341659530997e-04, -1.0353781981393695e-03,
								   -8.9991277491208166e-05,  1.5823085559532046e-03,
									1.0846852092072368e-03,  4.6369922347366810e-04};

	float32_t PMinusData[21*21] = {5.0406595692038536e-03,  3.1247735023498535e-03,
		       -1.2957267463207245e-02, -8.0784893361851573e-04,
		        2.5607252027839422e-03, -2.1824132919311523e+01,
		       -1.5456910133361816e+00,  5.9301991462707520e+00,
		        1.3891421258449554e-01, -2.0804461655643536e-06,
		       -3.2422674848930910e-05,  4.9934082198888063e-05,
		       -1.5633544535376132e-05,  3.1727938676340273e-07,
		       -1.5371331301139435e-06, -2.1628673130180687e-04,
		       -7.4254663195461035e-04, -7.4174418114125729e-04,
		       -1.0179843457081006e-06, -1.1088407063652994e-06,
		       -6.2235011455413769e-07,  3.1247735023498535e-03,
		        1.9471816485747695e-03, -8.0459835007786751e-03,
		       -5.0049374112859368e-04,  1.5865571331232786e-03,
		       -1.3519937515258789e+01, -9.5771872997283936e-01,
		        3.6749596595764160e+00,  8.6017094552516937e-02,
		       -6.1427283526427345e-07, -2.2606980564887635e-05,
		        3.1884988857200369e-05, -9.6862640930339694e-06,
		        1.9661543149140925e-07, -9.5225658469644259e-07,
		       -1.3098385534249246e-04, -4.6333027421496809e-04,
		       -4.6333920909091830e-04, -6.3043819409358548e-07,
		       -6.8702468070114264e-07, -3.8578920680265583e-07,
		       -1.2957267463207245e-02, -8.0459816381335258e-03,
		        3.3367574214935303e-02,  2.0803057122975588e-03,
		       -6.5941778011620045e-03,  5.6199539184570312e+01,
		        3.9802510738372803e+00, -1.5270647048950195e+01,
		       -3.5772767663002014e-01,  1.3634202105095028e-06,
		        8.3558348705992103e-05, -1.3001257320865989e-04,
		        4.0257731598103419e-05, -8.1712988730942016e-07,
		        3.9582046156283468e-06,  5.3965987171977758e-04,
		        1.9113363232463598e-03,  1.9166570855304599e-03,
		        2.6211371277895523e-06,  2.8556644338095794e-06,
		        1.6027000810936443e-06, -8.0785021418705583e-04,
		       -5.0049449782818556e-04,  2.0803087390959263e-03,
		        1.7635760013945401e-04, -5.2224291721358895e-04,
		        4.7127633094787598e+00,  3.2056900858879089e-01,
		       -1.1377519369125366e+00, -3.0671397224068642e-02,
		       -3.4587753816595068e-06, -1.5335669445448730e-07,
		       -4.3208055444665661e-07,  1.6512942693225341e-06,
		        3.8705209703948640e-07, -2.5500023639324354e-06,
		        3.2987401937134564e-05,  8.6790947534609586e-05,
		        6.8387627834454179e-05,  1.8710242954966816e-07,
		        8.4281946328701451e-06,  8.1654989116941579e-06,
		        2.5607200805097818e-03,  1.5865541063249111e-03,
		       -6.5941647626459599e-03, -5.2224250975996256e-04,
		        1.6195350326597691e-03, -1.4441775321960449e+01,
		       -9.6211594343185425e-01,  3.5358335971832275e+00,
		        1.0663956403732300e-01,  9.6024959930218756e-06,
		        3.1587600801685767e-07,  1.9062437104366836e-06,
		       -1.5932222595438361e-05, -1.2188560276626959e-06,
		       -4.5354895519267302e-06, -9.4954404630698264e-05,
		       -2.9954785713925958e-04, -2.4503649910911918e-04,
		       -1.7738763062880025e-06,  1.0542772770349984e-06,
		        2.5395804641448194e-06, -2.1824117660522461e+01,
		       -1.3519928932189941e+01,  5.6199508666992188e+01,
		        4.7127614021301270e+00, -1.4441769599914551e+01,
		        2.3976639062500000e+05,  8.5403515625000000e+03,
		       -3.1790466796875000e+04, -3.0337165527343750e+03,
		       -1.0520769655704498e-01, -6.2783779576420784e-03,
		       -6.0279117897152901e-03, -1.2290066480636597e+00,
		       -2.1524276584386826e-02,  2.9753465205430984e-03,
		        9.7097933292388916e-01,  2.1567680835723877e+00,
		        1.5645829439163208e+00,  3.3686764538288116e-02,
		       -1.8125399351119995e+00, -1.8059346675872803e+00,
		       -1.5456923246383667e+00, -9.5771932601928711e-01,
		        3.9802539348602295e+00,  3.2056912779808044e-01,
		       -9.6211665868759155e-01,  8.5403564453125000e+03,
		        5.9197888183593750e+02, -2.1275939941406250e+03,
		       -5.3192577362060547e+01, -6.0434597544372082e-03,
		        2.8401683084666729e-04, -1.5295977937057614e-03,
		        2.2874367423355579e-03,  1.8771526811178774e-04,
		       -1.1117374524474144e-03,  6.1146728694438934e-02,
		        1.7337624728679657e-01,  1.4151421189308167e-01,
		        3.1831508385948837e-04,  1.7744606360793114e-02,
		        1.4386754482984543e-02,  5.9301943778991699e+00,
		        3.6749567985534668e+00, -1.5270636558532715e+01,
		       -1.1377514600753784e+00,  3.5358345508575439e+00,
		       -3.1790500000000000e+04, -2.1275908203125000e+03,
		        7.9184433593750000e+03,  2.2919744873046875e+02,
		        1.9936045631766319e-02, -2.8638883959501982e-03,
		        8.6086159572005272e-03, -2.6694038882851601e-02,
		        6.6810629505198449e-05, -3.2306576613336802e-03,
		       -2.1822887659072876e-01, -7.1402388811111450e-01,
		       -6.0699760913848877e-01, -1.8768141744658351e-03,
		        5.1077166572213173e-03,  9.4514116644859314e-03,
		        1.3891354203224182e-01,  8.6016692221164703e-02,
		       -3.5772603750228882e-01, -3.0671393498778343e-02,
		        1.0663928836584091e-01, -3.0337148437500000e+03,
		       -5.3192520141601562e+01,  2.2919682312011719e+02,
		        5.3564392089843750e+01,  9.2047511134296656e-04,
		        1.8302891112398356e-04, -1.8561437900643796e-04,
		        1.0393768548965454e-02, -6.6892825998365879e-04,
		        6.3916908402461559e-05, -7.4246353469789028e-03,
		       -1.0277233086526394e-02, -5.3329993970692158e-03,
		       -2.0220861188136041e-04,  4.4973399490118027e-02,
		        4.6393368393182755e-02, -2.0801739992748480e-06,
		       -6.1407018847603467e-07,  1.3628047099700780e-06,
		       -3.4588776998134563e-06,  9.6024459708132781e-06,
		       -1.0521101206541061e-01, -6.0435268096625805e-03,
		        1.9936244934797287e-02,  9.2046248028054833e-04,
		        6.7353896156419069e-05,  1.6289303630401264e-06,
		       -2.6550476377451560e-06, -2.3660300030314829e-07,
		       -5.3149790124606966e-10, -1.4971234918448317e-08,
		       -5.2034953841939569e-04,  6.8057779571972787e-06,
		        1.0775746886793058e-05, -2.5247942403439083e-08,
		        9.6646450842285958e-09,  2.6561663979407513e-09,
		       -3.2423278753412887e-05, -2.2607320715906098e-05,
		        8.3559913036879152e-05, -1.5345192139193387e-07,
		        3.1539013889414491e-07, -6.2862057238817215e-03,
		        2.8403362375684083e-04, -2.8635994531214237e-03,
		        1.8298531358595937e-04,  1.6289745872200001e-06,
		        9.1931506176479161e-06, -5.4073275350674521e-06,
		       -6.4790133258441074e-09,  1.3441231461186476e-10,
		       -3.4910968560453171e-10,  6.8449480750132352e-06,
		       -3.6011097108712420e-05,  2.1306448616087437e-05,
		        1.0106315090352425e-10, -3.4424893491369346e-10,
		       -6.5065325349777936e-10,  4.9933663831325248e-05,
		        3.1884719646768644e-05, -1.3001142360735685e-04,
		       -4.3214993183937622e-07,  1.9060918248214875e-06,
		       -6.0232491232454777e-03, -1.5293824253603816e-03,
		        8.6080813780426979e-03, -1.8564770289231092e-04,
		       -2.6550624170340598e-06, -5.4073411774879787e-06,
		        1.4407381968339905e-05, -1.5992986490687144e-08,
		        4.4633391427240099e-10, -5.3058074600542682e-10,
		       -1.1097689821326640e-05, -1.9308148694108240e-05,
		       -3.4532076824689284e-05,  1.0356631108265901e-09,
		       -8.1799911288982230e-10, -2.2872854721356362e-09,
		       -1.5633559087291360e-05, -9.6862713689915836e-06,
		        4.0257767977891490e-05,  1.6513005220986088e-06,
		       -1.5932248061290011e-05, -1.2290062904357910e+00,
		        2.2874404676258564e-03, -2.6694055646657944e-02,
		        1.0393761098384857e-02, -2.3659755754579237e-07,
		       -6.4793401755025570e-09, -1.5993460777963264e-08,
		        6.3325715018436313e-05,  3.3095616913669801e-07,
		       -2.2346005152940052e-06,  2.1228504465398146e-06,
		        3.2094581001729239e-06,  2.4251464765256969e-06,
		       -1.6772364688222297e-06,  2.5788790480874013e-06,
		        1.0644635040080175e-06,  3.1728043836665165e-07,
		        1.9661614203414501e-07, -8.1713255895010661e-07,
		        3.8705212546119583e-07, -1.2188550044811564e-06,
		       -2.1524289622902870e-02,  1.8771532631944865e-04,
		        6.6811953729484230e-05, -6.6892825998365879e-04,
		       -5.3154403101274283e-10,  1.3442083557357876e-10,
		        4.4628631346022019e-10,  3.3095611229327915e-07,
		        9.9911951110698283e-05, -4.1221817070891120e-08,
		        3.4194831410871984e-09, -6.9054813423008454e-08,
		       -7.0966017062801257e-08, -3.7264367591660630e-08,
		       -7.5937251153845864e-08, -2.3685815975227342e-08,
		       -1.5371335848612944e-06, -9.5225692575695575e-07,
		        3.9582064346177503e-06, -2.5500025913061108e-06,
		       -4.5354886424320284e-06,  2.9753756243735552e-03,
		       -1.1117375688627362e-03, -3.2306581269949675e-03,
		        6.3916195358615369e-05, -1.4970897410648831e-08,
		       -3.4907274293338730e-10, -5.3058984983422874e-10,
		       -2.2346002879203297e-06, -4.1221813518177441e-08,
		        9.9724238680209965e-05,  1.3378999597080110e-07,
		        2.0729609673253435e-07,  1.5124976471270202e-07,
		        1.1879767924938278e-07,  1.4488556132619124e-07,
		        6.5032786267238407e-08, -2.1628639660775661e-04,
		       -1.3098372437525541e-04,  5.3965934785082936e-04,
		        3.2987249142024666e-05, -9.4955285021569580e-05,
		        9.7097063064575195e-01,  6.1146754771471024e-02,
		       -2.1822890639305115e-01, -7.4246642179787159e-03,
		       -5.2034942200407386e-04,  6.8450153776211664e-06,
		       -1.1097456081188284e-05,  2.1228981950116577e-06,
		        3.4189218123259479e-09,  1.3378932806062949e-07,
		        4.6296245418488979e-03,  2.0615703760995530e-05,
		        4.2524781747488305e-05,  2.1452336795846350e-07,
		       -7.4076503153719386e-08, -1.4997752373346884e-08,
		       -7.4254669016227126e-04, -4.6333004138432443e-04,
		        1.9113363232463598e-03,  8.6790525529067963e-05,
		       -2.9954864294268191e-04,  2.1567702293395996e+00,
		        1.7337484657764435e-01, -7.1401971578598022e-01,
		       -1.0277320630848408e-02,  6.8061062847846188e-06,
		       -3.6011766496812925e-05, -1.9307992261019535e-05,
		        3.2094428661366692e-06, -6.9055197116085765e-08,
		        2.0729569882860233e-07,  2.0612736989278346e-05,
		        7.3294457979500294e-03,  1.3586199202109128e-04,
		       -6.7493282074337912e-08, -2.7965032245447219e-07,
		       -1.3299953138812270e-07, -7.4174633482471108e-04,
		       -4.6334046055562794e-04,  1.9166623242199421e-03,
		        6.8388064391911030e-05, -2.4503964232280850e-04,
		        1.5646095275878906e+00,  1.4151482284069061e-01,
		       -6.0700702667236328e-01, -5.3333295509219170e-03,
		        1.0775933333206922e-05,  2.1306263079168275e-05,
		       -3.4532135032350197e-05,  2.4251335162261967e-06,
		       -7.0965704423997522e-08,  1.5125164054552442e-07,
		        4.2524945456534624e-05,  1.3586155546363443e-04,
		        7.2975964285433292e-03, -1.0599505628761108e-07,
		       -2.2456107728885399e-07, -1.0043542886251089e-07,
		       -1.0179841183344251e-06, -6.3043813725016662e-07,
		        2.6211369004158769e-06,  1.8710237270624930e-07,
		       -1.7738767610353534e-06,  3.3686779439449310e-02,
		        3.1831490923650563e-04, -1.8768147565424442e-03,
		       -2.0220868464093655e-04, -2.5247830492958201e-08,
		        1.0109300896399276e-10,  1.0358600643911586e-09,
		       -1.6772371509432560e-06, -3.7264360486233272e-08,
		        1.1879767924938278e-07,  2.1452270004829188e-07,
		       -6.7492294419935206e-08, -1.0599586630632984e-07,
		        7.5987145464750938e-07, -6.3270626071698643e-08,
		       -2.3801639770226757e-08, -1.1088407063652994e-06,
		       -6.8702479438798036e-07,  2.8556642064359039e-06,
		        8.4281955423648469e-06,  1.0542764812271344e-06,
		       -1.8125399351119995e+00,  1.7744606360793114e-02,
		        5.1077161915600300e-03,  4.4973399490118027e-02,
		        9.6646530778343731e-09, -3.4421010486340720e-10,
		       -8.1821804887027838e-10,  2.5788790480874013e-06,
		       -7.5937251153845864e-08,  1.4488551869362709e-07,
		       -7.4077632916669245e-08, -2.7964969717686472e-07,
		       -2.2456114834312757e-07, -6.3270647387980716e-08,
		        9.9771008535753936e-05, -8.9145949289104465e-08,
		       -6.2235034192781313e-07, -3.8578937733291241e-07,
		        1.6027006495278329e-06,  8.1654989116941579e-06,
		        2.5395806915184949e-06, -1.8059346675872803e+00,
		        1.4386752620339394e-02,  9.4514125958085060e-03,
		        4.6393368393182755e-02,  2.6560516008800050e-09,
		       -6.5057520481914821e-10, -2.2873198890493995e-09,
		        1.0644635040080175e-06, -2.3685805317086306e-08,
		        6.5032800478093122e-08, -1.4996867747640863e-08,
		       -1.3299940349043027e-07, -1.0043576992302405e-07,
		       -2.3801653981081472e-08, -8.9145942183677107e-08,
		        9.9961725936736912e-05};

	float32_t HData[3*21] = {0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
							 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
							 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
							 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

	float32_t RData[3*3] = {1.3500000022759195e-05, 0.0000000000000000e+00, 0.0000000000000000e+00,
							0.0000000000000000e+00, 1.6500000128871761e-05, 0.0000000000000000e+00,
							0.0000000000000000e+00, 0.0000000000000000e+00, 2.0000000000000000e+00};

	float32_t llaMeasData[3] = {35.38190460205078,  -117.90266418457031, 28954.111328125};

	arm_mat_init_f32(&xMinus, 22, 1, xMinusData);
	arm_mat_init_f32(&P_minus, 21, 21, PMinusData);
	arm_mat_init_f32(&H, 3, 21, HData);
	arm_mat_init_f32(&R, 3, 3, RData);
	arm_mat_init_f32(&lla_meas, 3, 1, llaMeasData);

	float32_t xPlusData[22*1];
	float32_t PPlusData[21*21];

	arm_matrix_instance_f32 xPlus, P_plus;

	update_GPS(&xMinus, &P_minus, &H,
			   &R, &lla_meas, &xPlus,
			   &P_plus, xPlusData, PPlusData);

	arm_matrix_instance_f32 xPlusTrue, PPlusTrue;

	float32_t xPlusTrueData[22*1] = {-2.3421935737133026e-02,  9.1510498523712158e-01,
									4.0010610222816467e-01, -4.4151850044727325e-02,
									3.5394058227539062e+01, -1.1789278411865234e+02,
									2.8919419921875000e+04, -9.4489202499389648e+00,
								   -3.9771766662597656e+01,  1.0850834655761719e+02,
								   -2.7941286680288613e-04, -2.1034452947787941e-04,
								   -2.6590019115246832e-04,  1.1989779770374298e-02,
									1.6433990094810724e-03,  3.8164916913956404e-03,
									2.1478341659530997e-04, -1.0353781981393695e-03,
								   -8.9991277491208166e-05,  1.9477189052850008e-03,
								   -6.8978127092123032e-03, -7.7270744368433952e-03};

	float32_t PPlusDataTrue[21*21] = {1.1813067831099033e-03, 7.3359895031899214e-04, -3.0189633835107088e-03, -2.7086314730695449e-05, 1.2851053907070309e-04, -8.7295323610305786e-01, -1.0466296225786209e-01, 6.2079751491546631e-01, -9.3027343973517418e-03, -1.6230371329584159e-05, -3.2854029996087775e-05, 4.6927594667067751e-05, 1.8522323443903588e-05, 2.3588731892232317e-06, 7.1205317908606958e-06, -7.5431824370753020e-05, -2.8545100940391421e-04, -3.6549934884533286e-04, 1.6848315453898977e-06, 5.4194379117689095e-06, 3.4609010981512256e-06,
			7.3359924135729671e-04, 4.6565974480472505e-04, -1.8884178716689348e-03, -1.6766256521805190e-05, 7.9629251558799297e-05, -5.4078996181488037e-01, -6.4911834895610809e-02, 3.8541331887245178e-01, -5.7894876226782799e-03, -9.3804655989515595e-06, -2.2874128262628801e-05, 3.0021899874554947e-05, 1.1498406820464879e-05, 1.4619756711908849e-06, 4.4168609747430310e-06, -4.3718744564102963e-05, -1.8010922940447927e-04, -2.3020849039312452e-04, 1.0443405926707783e-06, 3.3723433716659201e-06, 2.1582000044872984e-06,
			-3.0189647804945707e-03, -1.8884162418544292e-03, 7.7752312645316124e-03, 6.9748821260873228e-05, -3.3093098318204284e-04, 2.2479498386383057e+00, 2.6943096518516541e-01, -1.5982985496520996e+00, 2.3945134133100510e-02, 3.7801102735102177e-05, 8.4669110947288573e-05, -1.2227044499013573e-04, -4.7700916184112430e-05, -6.0745464907085989e-06, -1.8336892026127316e-05, 1.7694201960694045e-04, 7.3425786104053259e-04, 9.4777846243232489e-04, -4.3389513848524075e-06, -1.3957958799437620e-05, -8.9146142272511497e-06,
			-2.7087507987744175e-05, -1.6766964108683169e-05, 6.9751622504554689e-05, 1.2260843504918739e-05, -2.3514765416621231e-05, 1.8851137161254883e-01, 2.0094782114028931e-02, -4.9773525446653366e-02, 1.6673016361892223e-03, -3.4422717476445541e-07, -3.3379102148956008e-08, 7.5010504474448680e-08, -1.1619492852332769e-06, 2.9971957360430679e-08, -2.2899473606230458e-06, 2.7190417313249782e-06, -1.1995266504527535e-06, -2.4276482690765988e-06, -2.1070850664273166e-07, 6.0529582697199658e-06, 6.1496575654018670e-06,
			1.2850898201577365e-04, 7.9628334788139910e-05, -3.3092731609940529e-04, -2.3514705389970914e-05, 7.8273464168887585e-05, -5.7767289876937866e-01, -4.4369418174028397e-02, 1.7114523053169250e-01, 3.8970818277448416e-03, 3.6881297660329437e-07, 1.1207763606080334e-09, 1.4125421898825152e-07, -2.8439876587071922e-06, -9.1858979089920467e-08, -1.0989759857693571e-06, -3.9699948501947802e-06, -1.7040285456459969e-05, -1.4682546861877199e-05, -1.4912748724782432e-07, 1.2915677416458493e-07, 2.6580613621263183e-07,
			-8.7295198440551758e-01, -5.4078924655914307e-01, 2.2479486465454102e+00, 1.8851131200790405e-01, -5.7767254114151001e-01, 9.5924501953125000e+03, 3.4161352539062500e+02, -1.2716237792968750e+03, -1.2138269042968750e+02, -4.2080800049006939e-03, -2.4993458646349609e-04, -2.4170349934138358e-04, -4.9181926995515823e-02, -8.6148449918255210e-04, 1.1873408220708370e-04, 3.8842547684907913e-02, 8.6263485252857208e-02, 6.2570348381996155e-02, 1.3477851171046495e-03, -7.2531051933765411e-02, -7.2266608476638794e-02,
			-1.0466463118791580e-01, -6.4912557601928711e-02, 2.6943480968475342e-01, 2.0094865933060646e-02, -4.4369496405124664e-02, 3.4161389160156250e+02, 4.0873443603515625e+01, -1.2535884857177734e+02, 4.7979211807250977e+00, -3.9804505649954081e-04, 4.9186899559572339e-04, -5.5201002396643162e-04, -4.9585914239287376e-03, -5.0448952242732048e-04, -1.3960197102278471e-03, 6.0051227919757366e-03, 9.1504901647567749e-03, 8.6335372179746628e-03, -4.6435144031420350e-04, 1.3060096651315689e-02, 1.0423442348837852e-02,
			6.2080061435699463e-01, 3.8541507720947266e-01, -1.5983058214187622e+00, -4.9773510545492172e-02, 1.7114529013633728e-01, -1.2716286621093750e+03, -1.2535662078857422e+02, 5.7219067382812500e+02, -3.8198947906494141e-01, -2.2588757565245032e-04, -3.5531925968825817e-03, 4.7601577825844288e-03, -9.1989478096365929e-04, 2.4522508028894663e-03, 4.6443967148661613e-03, -1.9584918394684792e-02, -9.7364164888858795e-02, -1.0434590280056000e-01, 1.7559715779498219e-03, -2.1239845082163811e-03, -7.0301513187587261e-04,
			-9.3028191477060318e-03, -5.7895490899682045e-03, 2.3945456370711327e-02, 1.6672767233103514e-03, 3.8970534224063158e-03, -1.2138240814208984e+02, 4.7979145050048828e+00, -3.8182926177978516e-01, 8.2595462799072266e+00, 3.8997746742097661e-05, 1.1197845014976338e-04, -1.4934926002752036e-04, -1.2621519155800343e-02, -1.1765514500439167e-03, 5.8270001318305731e-04, 3.1146549736149609e-04, 1.6372350510209799e-03, 1.3900778722018003e-03, 3.2096053473651409e-04, 1.0283561423420906e-02, 1.1919576674699783e-02,
			-1.6229982065851800e-05, -9.3801900220569223e-06, 3.7800185964442790e-05, -3.4429797324264655e-07, 3.6877938214274764e-07, -4.2087389156222343e-03, -3.9807904977351427e-04, -2.2564102255273610e-04, 3.8955633499426767e-05, 6.7290173319634050e-05, 1.6257455399681930e-06, -2.6612178771756589e-06, -4.2138682943004824e-07, 1.1363983087875340e-09, -5.9750803416136478e-08, -5.1975017413496971e-04, 8.2733795352396555e-06, 1.1902279766218271e-05, -1.8475025953534896e-08, -1.4370658618645393e-07, -1.5252227569817478e-07,
			-3.2853855373105034e-05, -2.2873988200444728e-05, 8.4668688941746950e-05, -3.3501805773994420e-08, 9.7062269333036966e-10, -2.5150028523057699e-04, 4.9178214976564050e-04, -3.5521674435585737e-03, 1.1182907473994419e-04, 1.6257870356639614e-06, 9.1928823167108931e-06, -5.4070001169748139e-06, -4.9277435465455710e-08, -5.4995208387254024e-10, -9.0945126984820490e-09, 6.8719268710992765e-06, -3.5988316085422412e-05, 2.1313910110620782e-05, 1.2381651259829596e-10, -3.2364425806008512e-08, -3.1940466271862533e-08,
			4.6927423682063818e-05, 3.0021783459233120e-05, -1.2226995022501796e-04, 7.4943919514680601e-08, 1.4120605840162170e-07, -2.4076887348201126e-04, -5.5180536583065987e-04, 4.7598676756024361e-03, -1.4930048200767487e-04, -2.6612344754539663e-06, -5.4070169426267967e-06, 1.4403047316591255e-05, 1.4447702767483861e-07, 4.6061368053074148e-09, 3.6504957279248629e-08, -1.1021944374078885e-05, -1.8866916434490122e-05, -3.4135173336835578e-05, 4.1997298971807595e-09, 9.0171290878515720e-08, 8.3818783025435550e-08,
			1.8521670426707715e-05, 1.1498030289658345e-05, -4.7699264541734010e-05, -1.1619231372606009e-06, -2.8439781090128236e-06, -4.9182027578353882e-02, -4.9585876986384392e-03, -9.1969966888427734e-04, -1.2621521949768066e-02, -4.2133308397751534e-07, -4.9133184631955373e-08, 1.4445613771840726e-07, 4.6913417463656515e-05, -5.6328467223920597e-08, -3.0896392217982793e-06, 3.0266749035945395e-06, -6.1299519984459039e-06, -7.7867089203209616e-06, -1.5203754628601018e-06, -1.7554684745846316e-05, -1.8805543732014485e-05,
			2.3588629574078368e-06, 1.4619696457884856e-06, -6.0745187511201948e-06, 2.9972660797739081e-08, -9.1858183282056416e-08, -8.6149101844057441e-04, -5.0448806723579764e-04, 2.4522552266716957e-03, -1.1765516828745604e-03, 1.1371437125262673e-09, -5.4708609864562163e-10, 4.6052868185597617e-09, -5.6328438802211167e-08, 9.9902121291961521e-05, -5.0157932918182269e-08, -3.0924006466648279e-08, -4.0887670138545218e-07, -4.0132695744432567e-07, -3.3101493102094537e-08, -5.8529241186988656e-07, -5.2675142114821938e-07,
			7.1201252467290033e-06, 4.4166281440993771e-06, -1.8335949789616279e-05, -2.2899434952705633e-06, -1.0989891734425328e-06, 1.1872513277921826e-04, -1.3960301876068115e-03, 4.6443976461887360e-03, 5.8270839508622885e-04, -5.9730865586971049e-08, -9.0520639872693209e-09, 3.6523680080335907e-08, -3.0896451335138408e-06, -5.0158153186430354e-08, 9.9022763606626540e-05, 3.5738599990509101e-07, -2.1880637177673634e-06, -2.2869269287184579e-06, 5.2196057254150219e-08, 1.3489579941960983e-06, 1.3124777069606353e-06,
			-7.5430099968798459e-05, -4.3717744119931012e-05, 1.7693791596684605e-04, 2.7188345939066494e-06, -3.9702490539639257e-06, 3.8840755820274353e-02, 6.0049858875572681e-03, -1.9583621993660927e-02, 3.1132416916079819e-04, -5.1975005771964788e-04, 6.8719800765393302e-06, -1.1021707905456424e-05, 3.0265193800005363e-06, -3.0928486438597247e-08, 3.5731454772758298e-07, 4.6239024959504604e-03, 5.3926596592646092e-06, 3.0564198823412880e-05, 1.3857427916263987e-07, 8.8769940020938520e-07, 9.8601753961702343e-07,
			-2.8545159148052335e-04, -1.8010922940447927e-04, 7.3425885057076812e-04, -1.1998481568298303e-06, -1.7040571037796326e-05, 8.6263619363307953e-02, 9.1491146013140678e-03, -9.7358860075473785e-02, 1.6372841782867908e-03, 8.2737624325091019e-06, -3.5988789022667333e-05, -1.8866699974751100e-05, -6.1300916058826260e-06, -4.0887888985707832e-07, -2.1881608063267777e-06, 5.3900484999758191e-06, 7.2716418653726578e-03, 8.6857151472941041e-05, -4.4797616283176467e-07, -3.8658745324937627e-06, -3.3048420391423861e-06,
			-3.6549902870319784e-04, -2.3020815569907427e-04, 9.4777724007144570e-04, -2.4277583179355133e-06, -1.4683304470963776e-05, 6.2575466930866241e-02, 8.6329774931073189e-03, -1.0435010492801666e-01, 1.3900420162826777e-03, 1.1902539881702978e-05, 2.1313926481525414e-05, -3.4135176974814385e-05, -7.7868235166533850e-06, -4.0132906065082352e-07, -2.2870124212204246e-06, 3.0564599001081660e-05, 8.6856423877179623e-05, 7.2555062361061573e-03, -4.2788821019712486e-07, -5.1520869419618975e-06, -4.6349596232175827e-06,
			1.6847951656018267e-06, 1.0443192195452866e-06, -4.3388672565924935e-06, -2.1070830769076565e-07, -1.4912920676124486e-07, 1.3477877946570516e-03, -4.6435283729806542e-04, 1.7559680854901671e-03, 3.2096164068207145e-04, -1.8473514273864566e-08, 1.2650547276393809e-10, 4.2024077551161554e-09, -1.5203769407889922e-06, -3.3101510865662931e-08, 5.2195979094449285e-08, 1.3857869873845630e-07, -4.4796612996833574e-07, -4.2787877418959397e-07, 7.4850339615295525e-07, 3.9271924379136181e-07, 4.3400200411269907e-07,
			5.4199017540668137e-06, 3.3726050787663553e-06, -1.3958953786641359e-05, 6.0529682741616853e-06, 1.2921753977934713e-07, -7.2531215846538544e-02, 1.3060167431831360e-02, -2.1237004548311234e-03, 1.0283533483743668e-02, -1.4370576195688045e-07, -3.2307063690950599e-08, 9.0078088987866067e-08, -1.7554662917973474e-05, -5.8529167290544137e-07, 1.3489642469721730e-06, 8.8774311279848916e-07, -3.8660309655824676e-06, -5.1522538342396729e-06, 3.9271958485187497e-07, 6.7308334109839052e-05, -3.2335192372556776e-05,
			3.4614201922522625e-06, 2.1584769456239883e-06, -8.9155710156774148e-06, 6.1496702983276919e-06, 2.6586968715491821e-07, -7.2266742587089539e-02, 1.0423495434224606e-02, -7.0274062454700470e-04, 1.1919547803699970e-02, -1.5252305729518412e-07, -3.1886987272855549e-08, 8.3725097965725581e-08, -1.8805521904141642e-05, -5.2675068218377419e-07, 1.3124854376656003e-06, 9.8605596576817334e-07, -3.3050034744519508e-06, -4.6351296987268142e-06, 4.3400248728175939e-07, -3.2335196010535583e-05, 6.7926055635325611e-05};

	arm_mat_init_f32(&xPlusTrue, 22, 1, xPlusTrueData);
	arm_mat_init_f32(&PPlusTrue, 21, 21, PPlusDataTrue);

	bool test1 = areMatricesEqual(&xPlus, &xPlusTrue);
	bool test2 = areMatricesEqual(&P_plus, &PPlusTrue);

	bool test = (test1 && test2);
	return test;
}

void test_update_mag(void) {

	arm_matrix_instance_f32 xMinus, PMinus, R, magI, magMeas;

	float32_t xMinusData[22*1] = {-2.3421935737133026e-02,  9.1510498523712158e-01,
	        4.0010610222816467e-01, -4.4151850044727325e-02,
	        3.5394058227539062e+01, -1.1789278411865234e+02,
	        2.8919419921875000e+04, -9.4489202499389648e+00,
	       -3.9771766662597656e+01,  1.0850834655761719e+02,
	       -2.7941286680288613e-04, -2.1034452947787941e-04,
	       -2.6590019115246832e-04,  1.1989779770374298e-02,
	        1.6433990094810724e-03,  3.8164916913956404e-03,
	        2.1478341659530997e-04, -1.0353781981393695e-03,
	       -8.9991277491208166e-05,  1.9477189052850008e-03,
	       -6.8978127092123032e-03, -7.7270744368433952e-03};

	float32_t PMinusData[21*21] = {1.1813067831099033e-03, 7.3359895031899214e-04, -3.0189633835107088e-03, -2.7086314730695449e-05, 1.2851053907070309e-04, -8.7295323610305786e-01, -1.0466296225786209e-01, 6.2079751491546631e-01, -9.3027343973517418e-03, -1.6230371329584159e-05, -3.2854029996087775e-05, 4.6927594667067751e-05, 1.8522323443903588e-05, 2.3588731892232317e-06, 7.1205317908606958e-06, -7.5431824370753020e-05, -2.8545100940391421e-04, -3.6549934884533286e-04, 1.6848315453898977e-06, 5.4194379117689095e-06, 3.4609010981512256e-06,
			7.3359924135729671e-04, 4.6565974480472505e-04, -1.8884178716689348e-03, -1.6766256521805190e-05, 7.9629251558799297e-05, -5.4078996181488037e-01, -6.4911834895610809e-02, 3.8541331887245178e-01, -5.7894876226782799e-03, -9.3804655989515595e-06, -2.2874128262628801e-05, 3.0021899874554947e-05, 1.1498406820464879e-05, 1.4619756711908849e-06, 4.4168609747430310e-06, -4.3718744564102963e-05, -1.8010922940447927e-04, -2.3020849039312452e-04, 1.0443405926707783e-06, 3.3723433716659201e-06, 2.1582000044872984e-06,
			-3.0189647804945707e-03, -1.8884162418544292e-03, 7.7752312645316124e-03, 6.9748821260873228e-05, -3.3093098318204284e-04, 2.2479498386383057e+00, 2.6943096518516541e-01, -1.5982985496520996e+00, 2.3945134133100510e-02, 3.7801102735102177e-05, 8.4669110947288573e-05, -1.2227044499013573e-04, -4.7700916184112430e-05, -6.0745464907085989e-06, -1.8336892026127316e-05, 1.7694201960694045e-04, 7.3425786104053259e-04, 9.4777846243232489e-04, -4.3389513848524075e-06, -1.3957958799437620e-05, -8.9146142272511497e-06,
			-2.7087507987744175e-05, -1.6766964108683169e-05, 6.9751622504554689e-05, 1.2260843504918739e-05, -2.3514765416621231e-05, 1.8851137161254883e-01, 2.0094782114028931e-02, -4.9773525446653366e-02, 1.6673016361892223e-03, -3.4422717476445541e-07, -3.3379102148956008e-08, 7.5010504474448680e-08, -1.1619492852332769e-06, 2.9971957360430679e-08, -2.2899473606230458e-06, 2.7190417313249782e-06, -1.1995266504527535e-06, -2.4276482690765988e-06, -2.1070850664273166e-07, 6.0529582697199658e-06, 6.1496575654018670e-06,
			1.2850898201577365e-04, 7.9628334788139910e-05, -3.3092731609940529e-04, -2.3514705389970914e-05, 7.8273464168887585e-05, -5.7767289876937866e-01, -4.4369418174028397e-02, 1.7114523053169250e-01, 3.8970818277448416e-03, 3.6881297660329437e-07, 1.1207763606080334e-09, 1.4125421898825152e-07, -2.8439876587071922e-06, -9.1858979089920467e-08, -1.0989759857693571e-06, -3.9699948501947802e-06, -1.7040285456459969e-05, -1.4682546861877199e-05, -1.4912748724782432e-07, 1.2915677416458493e-07, 2.6580613621263183e-07,
			-8.7295198440551758e-01, -5.4078924655914307e-01, 2.2479486465454102e+00, 1.8851131200790405e-01, -5.7767254114151001e-01, 9.5924501953125000e+03, 3.4161352539062500e+02, -1.2716237792968750e+03, -1.2138269042968750e+02, -4.2080800049006939e-03, -2.4993458646349609e-04, -2.4170349934138358e-04, -4.9181926995515823e-02, -8.6148449918255210e-04, 1.1873408220708370e-04, 3.8842547684907913e-02, 8.6263485252857208e-02, 6.2570348381996155e-02, 1.3477851171046495e-03, -7.2531051933765411e-02, -7.2266608476638794e-02,
			-1.0466463118791580e-01, -6.4912557601928711e-02, 2.6943480968475342e-01, 2.0094865933060646e-02, -4.4369496405124664e-02, 3.4161389160156250e+02, 4.0873443603515625e+01, -1.2535884857177734e+02, 4.7979211807250977e+00, -3.9804505649954081e-04, 4.9186899559572339e-04, -5.5201002396643162e-04, -4.9585914239287376e-03, -5.0448952242732048e-04, -1.3960197102278471e-03, 6.0051227919757366e-03, 9.1504901647567749e-03, 8.6335372179746628e-03, -4.6435144031420350e-04, 1.3060096651315689e-02, 1.0423442348837852e-02,
			6.2080061435699463e-01, 3.8541507720947266e-01, -1.5983058214187622e+00, -4.9773510545492172e-02, 1.7114529013633728e-01, -1.2716286621093750e+03, -1.2535662078857422e+02, 5.7219067382812500e+02, -3.8198947906494141e-01, -2.2588757565245032e-04, -3.5531925968825817e-03, 4.7601577825844288e-03, -9.1989478096365929e-04, 2.4522508028894663e-03, 4.6443967148661613e-03, -1.9584918394684792e-02, -9.7364164888858795e-02, -1.0434590280056000e-01, 1.7559715779498219e-03, -2.1239845082163811e-03, -7.0301513187587261e-04,
			-9.3028191477060318e-03, -5.7895490899682045e-03, 2.3945456370711327e-02, 1.6672767233103514e-03, 3.8970534224063158e-03, -1.2138240814208984e+02, 4.7979145050048828e+00, -3.8182926177978516e-01, 8.2595462799072266e+00, 3.8997746742097661e-05, 1.1197845014976338e-04, -1.4934926002752036e-04, -1.2621519155800343e-02, -1.1765514500439167e-03, 5.8270001318305731e-04, 3.1146549736149609e-04, 1.6372350510209799e-03, 1.3900778722018003e-03, 3.2096053473651409e-04, 1.0283561423420906e-02, 1.1919576674699783e-02,
			-1.6229982065851800e-05, -9.3801900220569223e-06, 3.7800185964442790e-05, -3.4429797324264655e-07, 3.6877938214274764e-07, -4.2087389156222343e-03, -3.9807904977351427e-04, -2.2564102255273610e-04, 3.8955633499426767e-05, 6.7290173319634050e-05, 1.6257455399681930e-06, -2.6612178771756589e-06, -4.2138682943004824e-07, 1.1363983087875340e-09, -5.9750803416136478e-08, -5.1975017413496971e-04, 8.2733795352396555e-06, 1.1902279766218271e-05, -1.8475025953534896e-08, -1.4370658618645393e-07, -1.5252227569817478e-07,
			-3.2853855373105034e-05, -2.2873988200444728e-05, 8.4668688941746950e-05, -3.3501805773994420e-08, 9.7062269333036966e-10, -2.5150028523057699e-04, 4.9178214976564050e-04, -3.5521674435585737e-03, 1.1182907473994419e-04, 1.6257870356639614e-06, 9.1928823167108931e-06, -5.4070001169748139e-06, -4.9277435465455710e-08, -5.4995208387254024e-10, -9.0945126984820490e-09, 6.8719268710992765e-06, -3.5988316085422412e-05, 2.1313910110620782e-05, 1.2381651259829596e-10, -3.2364425806008512e-08, -3.1940466271862533e-08,
			4.6927423682063818e-05, 3.0021783459233120e-05, -1.2226995022501796e-04, 7.4943919514680601e-08, 1.4120605840162170e-07, -2.4076887348201126e-04, -5.5180536583065987e-04, 4.7598676756024361e-03, -1.4930048200767487e-04, -2.6612344754539663e-06, -5.4070169426267967e-06, 1.4403047316591255e-05, 1.4447702767483861e-07, 4.6061368053074148e-09, 3.6504957279248629e-08, -1.1021944374078885e-05, -1.8866916434490122e-05, -3.4135173336835578e-05, 4.1997298971807595e-09, 9.0171290878515720e-08, 8.3818783025435550e-08,
			1.8521670426707715e-05, 1.1498030289658345e-05, -4.7699264541734010e-05, -1.1619231372606009e-06, -2.8439781090128236e-06, -4.9182027578353882e-02, -4.9585876986384392e-03, -9.1969966888427734e-04, -1.2621521949768066e-02, -4.2133308397751534e-07, -4.9133184631955373e-08, 1.4445613771840726e-07, 4.6913417463656515e-05, -5.6328467223920597e-08, -3.0896392217982793e-06, 3.0266749035945395e-06, -6.1299519984459039e-06, -7.7867089203209616e-06, -1.5203754628601018e-06, -1.7554684745846316e-05, -1.8805543732014485e-05,
			2.3588629574078368e-06, 1.4619696457884856e-06, -6.0745187511201948e-06, 2.9972660797739081e-08, -9.1858183282056416e-08, -8.6149101844057441e-04, -5.0448806723579764e-04, 2.4522552266716957e-03, -1.1765516828745604e-03, 1.1371437125262673e-09, -5.4708609864562163e-10, 4.6052868185597617e-09, -5.6328438802211167e-08, 9.9902121291961521e-05, -5.0157932918182269e-08, -3.0924006466648279e-08, -4.0887670138545218e-07, -4.0132695744432567e-07, -3.3101493102094537e-08, -5.8529241186988656e-07, -5.2675142114821938e-07,
			7.1201252467290033e-06, 4.4166281440993771e-06, -1.8335949789616279e-05, -2.2899434952705633e-06, -1.0989891734425328e-06, 1.1872513277921826e-04, -1.3960301876068115e-03, 4.6443976461887360e-03, 5.8270839508622885e-04, -5.9730865586971049e-08, -9.0520639872693209e-09, 3.6523680080335907e-08, -3.0896451335138408e-06, -5.0158153186430354e-08, 9.9022763606626540e-05, 3.5738599990509101e-07, -2.1880637177673634e-06, -2.2869269287184579e-06, 5.2196057254150219e-08, 1.3489579941960983e-06, 1.3124777069606353e-06,
			-7.5430099968798459e-05, -4.3717744119931012e-05, 1.7693791596684605e-04, 2.7188345939066494e-06, -3.9702490539639257e-06, 3.8840755820274353e-02, 6.0049858875572681e-03, -1.9583621993660927e-02, 3.1132416916079819e-04, -5.1975005771964788e-04, 6.8719800765393302e-06, -1.1021707905456424e-05, 3.0265193800005363e-06, -3.0928486438597247e-08, 3.5731454772758298e-07, 4.6239024959504604e-03, 5.3926596592646092e-06, 3.0564198823412880e-05, 1.3857427916263987e-07, 8.8769940020938520e-07, 9.8601753961702343e-07,
			-2.8545159148052335e-04, -1.8010922940447927e-04, 7.3425885057076812e-04, -1.1998481568298303e-06, -1.7040571037796326e-05, 8.6263619363307953e-02, 9.1491146013140678e-03, -9.7358860075473785e-02, 1.6372841782867908e-03, 8.2737624325091019e-06, -3.5988789022667333e-05, -1.8866699974751100e-05, -6.1300916058826260e-06, -4.0887888985707832e-07, -2.1881608063267777e-06, 5.3900484999758191e-06, 7.2716418653726578e-03, 8.6857151472941041e-05, -4.4797616283176467e-07, -3.8658745324937627e-06, -3.3048420391423861e-06,
			-3.6549902870319784e-04, -2.3020815569907427e-04, 9.4777724007144570e-04, -2.4277583179355133e-06, -1.4683304470963776e-05, 6.2575466930866241e-02, 8.6329774931073189e-03, -1.0435010492801666e-01, 1.3900420162826777e-03, 1.1902539881702978e-05, 2.1313926481525414e-05, -3.4135176974814385e-05, -7.7868235166533850e-06, -4.0132906065082352e-07, -2.2870124212204246e-06, 3.0564599001081660e-05, 8.6856423877179623e-05, 7.2555062361061573e-03, -4.2788821019712486e-07, -5.1520869419618975e-06, -4.6349596232175827e-06,
			1.6847951656018267e-06, 1.0443192195452866e-06, -4.3388672565924935e-06, -2.1070830769076565e-07, -1.4912920676124486e-07, 1.3477877946570516e-03, -4.6435283729806542e-04, 1.7559680854901671e-03, 3.2096164068207145e-04, -1.8473514273864566e-08, 1.2650547276393809e-10, 4.2024077551161554e-09, -1.5203769407889922e-06, -3.3101510865662931e-08, 5.2195979094449285e-08, 1.3857869873845630e-07, -4.4796612996833574e-07, -4.2787877418959397e-07, 7.4850339615295525e-07, 3.9271924379136181e-07, 4.3400200411269907e-07,
			5.4199017540668137e-06, 3.3726050787663553e-06, -1.3958953786641359e-05, 6.0529682741616853e-06, 1.2921753977934713e-07, -7.2531215846538544e-02, 1.3060167431831360e-02, -2.1237004548311234e-03, 1.0283533483743668e-02, -1.4370576195688045e-07, -3.2307063690950599e-08, 9.0078088987866067e-08, -1.7554662917973474e-05, -5.8529167290544137e-07, 1.3489642469721730e-06, 8.8774311279848916e-07, -3.8660309655824676e-06, -5.1522538342396729e-06, 3.9271958485187497e-07, 6.7308334109839052e-05, -3.2335192372556776e-05,
			3.4614201922522625e-06, 2.1584769456239883e-06, -8.9155710156774148e-06, 6.1496702983276919e-06, 2.6586968715491821e-07, -7.2266742587089539e-02, 1.0423495434224606e-02, -7.0274062454700470e-04, 1.1919547803699970e-02, -1.5252305729518412e-07, -3.1886987272855549e-08, 8.3725097965725581e-08, -1.8805521904141642e-05, -5.2675068218377419e-07, 1.3124854376656003e-06, 9.8605596576817334e-07, -3.3050034744519508e-06, -4.6351296987268142e-06, 4.3400248728175939e-07, -3.2335196010535583e-05, 6.7926055635325611e-05};

	float32_t RData[3*3] = {1.3500000022759195e-05, 0.0000000000000000e+00,
							   0.0000000000000000e+00, 0.0000000000000000e+00,
							   1.6500000128871761e-05, 0.0000000000000000e+00,
							   0.0000000000000000e+00, 0.0000000000000000e+00,
							   2.0000000000000000e+00};

	float32_t magIData[3*1] = {0.4891000092029572, 0.1040000021457672, 0.8659999966621399};

	float32_t magMeasData[3*1] = { 0.3532234728336334,  0.2187790721654892, -0.9095926284790039};

	arm_mat_init_f32(&xMinus, 22, 1, xMinusData);
	arm_mat_init_f32(&PMinus, 21, 21, PMinusData);
	arm_mat_init_f32(&R, 3, 3, RData);
	arm_mat_init_f32(&magI, 3, 1, magIData);
	arm_mat_init_f32(&magMeas, 3, 1, magMeasData);

	arm_matrix_instance_f32 xPlus, Pplus;
	float32_t xPlusData[22*1], PPlusData[21*21];

	update_mag(&xMinus, &PMinus, &R,
			   &magI, &magMeas, &xPlus,
			   &Pplus, xPlusData, PPlusData);

	float32_t xPlusDataTrue[22*1] = {-2.3431327193975449e-02,  9.1510486602783203e-01,
	        4.0010544657707214e-01, -4.4155474752187729e-02,
	        3.5394058227539062e+01, -1.1789278411865234e+02,
	        2.8919419921875000e+04, -9.4489974975585938e+00,
	       -3.9771469116210938e+01,  1.0850833892822266e+02,
	       -2.8288309113122523e-04, -2.1035737881902605e-04,
	       -2.6712063117884099e-04,  1.1989776976406574e-02,
	        1.6433987766504288e-03,  3.8164909929037094e-03,
	        1.9975155009888113e-04, -1.0361741296947002e-03,
	       -8.4345454524736851e-05,  1.9477185560390353e-03,
	       -6.8978141061961651e-03, -7.7270758338272572e-03};

	float32_t PPlusDataTrue[21*21] = {1.1763718212023377e-03, 7.3005689773708582e-04, -3.0136641580611467e-03, -2.7090729417977855e-05, 1.2850810890085995e-04, -8.7297034263610840e-01, -1.0462568700313568e-01, 6.2049317359924316e-01, -9.2928158119320869e-03, -1.5671810615458526e-05, -3.2096133509185165e-05, 4.6911070967325941e-05, 1.8516115233069286e-05, 2.3587522264278959e-06, 7.1190615926752798e-06, -7.3039613198488951e-05, -2.8430001111701131e-04, -3.6560921580530703e-04, 1.6847445749590406e-06, 5.4153892961039674e-06, 3.4570907700981479e-06,
			7.3005742160603404e-04, 4.6017495333217084e-04, -1.8798181554302573e-03, -1.6773421521065757e-05, 7.9624558566138148e-05, -5.4081428050994873e-01, -6.4868748188018799e-02, 3.8498932123184204e-01, -5.7756863534450531e-03, -9.2375894382712431e-06, -2.1661502614733763e-05, 2.9730630558333360e-05, 1.1487793017295189e-05, 1.4617410215578275e-06, 4.4143716877442785e-06, -4.3143521907040849e-05, -1.7843599198386073e-04, -2.2915974841453135e-04, 1.0441369795444189e-06, 3.3654855542408768e-06, 2.1517066670639906e-06,
			-3.0136655550450087e-03, -1.8798169912770391e-03, 7.7617238275706768e-03, 6.9760069891344756e-05, -3.3092353260144591e-04, 2.2479875087738037e+00, 2.6936429738998413e-01, -1.5976365804672241e+00, 2.3923590779304504e-02, 3.7621834053425118e-05, 8.2765516708604991e-05, -1.2179731129435822e-04, -4.7684210585430264e-05, -6.0741754168702755e-06, -1.8332973922952078e-05, 1.7623411258682609e-04, 7.3164125205948949e-04, 9.4605860067531466e-04, -4.3386276047385763e-06, -1.3947169463790487e-05, -8.9043969637714326e-06,
			-2.7091879019280896e-05, -1.6774114556028508e-05, 6.9762849307153374e-05, 1.2260834409971721e-05, -2.3514770873589441e-05, 1.8851134181022644e-01, 2.0094837993383408e-02, -4.9774076789617538e-02, 1.6673195641487837e-03, -3.4408373039696016e-07, -3.1796698607422513e-08, 7.4615137179989688e-08, -1.1619631550274789e-06, 2.9971648274340623e-08, -2.2899505438545020e-06, 2.7196056180400774e-06, -1.1973527307418408e-06, -2.4262094484583940e-06, -2.1070877664897125e-07, 6.0529491747729480e-06, 6.1496489252022002e-06,
			1.2850658094976097e-04, 7.9623656347393990e-05, -3.3091989462263882e-04, -2.3514710846939124e-05, 7.8273456892929971e-05, -5.7767289876937866e-01, -4.4369384646415710e-02, 1.7114487290382385e-01, 3.8970934692770243e-03, 3.6882283893646672e-07, 2.1623116630564709e-09, 1.4096433176291612e-07, -2.8439969810278853e-06, -9.1859185147313838e-08, -1.0989781458192738e-06, -3.9699934859527275e-06, -1.7038875739672221e-05, -1.4681461834697984e-05, -1.4912767198893562e-07, 1.2915081981645926e-07, 2.6580050871416461e-07,
			-8.7296944856643677e-01, -5.4081374406814575e-01, 2.2479865550994873e+00, 1.8851128220558167e-01, -5.7767254114151001e-01, 9.5924501953125000e+03, 3.4161370849609375e+02, -1.2716257324218750e+03, -1.2138262939453125e+02, -4.2071468196809292e-03, -2.4453891091980040e-04, -2.4289781867992133e-04, -4.9181975424289703e-02, -8.6148554692044854e-04, 1.1872305913129821e-04, 3.8846369832754135e-02, 8.6270995438098907e-02, 6.2574543058872223e-02, 1.3477841857820749e-03, -7.2531081736087799e-02, -7.2266638278961182e-02,
			-1.0462737083435059e-01, -6.4869567751884460e-02, 2.6936829090118408e-01, 2.0094921812415123e-02, -4.4369462877511978e-02, 3.4161407470703125e+02, 4.0873073577880859e+01, -1.2535541534423828e+02, 4.7978096008300781e+00, -4.0083730709739029e-04, 4.8245457583107054e-04, -5.5034266551956534e-04, -4.9585103988647461e-03, -5.0448777619749308e-04, -1.3960007345303893e-03, 5.9933662414550781e-03, 9.1371238231658936e-03, 8.6281411349773407e-03, -4.6435001422651112e-04, 1.3060148805379868e-02, 1.0423491708934307e-02,
			6.2049615383148193e-01, 3.8499107956886292e-01, -1.5976438522338867e+00, -4.9774061888456345e-02, 1.7114493250846863e-01, -1.2716306152343750e+03, -1.2535317993164062e+02, 5.7215753173828125e+02, -3.8091096282005310e-01, -2.0944610878359526e-04, -3.4597364719957113e-03, 4.7396267764270306e-03, -9.2070788377895951e-04, 2.4522331077605486e-03, 4.6442057937383652e-03, -1.9517084583640099e-02, -9.7233995795249939e-02, -1.0427393764257431e-01, 1.7559563275426626e-03, -2.1245100069791079e-03, -7.0351269096136093e-04,
			-9.2929173260927200e-03, -5.7757557369768620e-03, 2.3923924192786217e-02, 1.6672946512699127e-03, 3.8970650639384985e-03, -1.2138234710693359e+02, 4.7978019714355469e+00, -3.8075143098831177e-01, 8.2595109939575195e+00, 3.8463313103420660e-05, 1.0893827129621059e-04, -1.4868118159938604e-04, -1.2621493078768253e-02, -1.1765508679673076e-03, 5.8270624140277505e-04, 3.0926006729714572e-04, 1.6330006765201688e-03, 1.3877359451726079e-03, 3.2096102950163186e-04, 1.0283578187227249e-02, 1.1919592507183552e-02,
			-1.5671495930291712e-05, -9.2373311417759396e-06, 3.7621037336066365e-05, -3.4414858873788035e-07, 3.6879575304737955e-07, -4.2078560218214989e-03, -4.0086414082907140e-04, -2.0922004478052258e-04, 3.8419017073465511e-05, 6.7204338847659528e-05, 1.5985965546860825e-06, -2.6838456506084185e-06, -4.2122459831261949e-07, 1.1371229513557068e-09, -5.9710188793360430e-08, -5.2012101514264941e-04, 8.2173910413985141e-06, 1.2013570994895417e-05, -1.8477576801956275e-08, -1.4359494571181131e-07, -1.5242062545439694e-07,
			-3.2095947972266003e-05, -2.1661338905687444e-05, 8.2764949183911085e-05, -3.1915764253653833e-08, 2.0156321056674642e-09, -2.4614704307168722e-04, 4.8234598943963647e-04, -3.4587148111313581e-03, 1.0878737521125004e-04, 1.5986292964953464e-06, 8.9245422714157030e-06, -5.3409721658681519e-06, -4.6924572671969145e-08, -4.9776982535831849e-10, -8.5428188967284768e-09, 6.7639557528309524e-06, -3.6357585486257449e-05, 2.1074558389955200e-05, 1.6925570667236656e-10, -3.0844603315927088e-08, -3.0501187353593195e-08,
			4.6910874516470358e-05, 2.9730503229075111e-05, -1.2179675104562193e-04, 7.4549710404880898e-08, 1.4091786226799741e-07, -2.4196725280489773e-04, -5.5012939264997840e-04, 4.7393301501870155e-03, -1.4863276737742126e-04, -2.6838592930289451e-06, -5.3409871725307312e-06, 1.4376461876963731e-05, 1.4387141789029556e-07, 4.5917065705225468e-09, 3.6363857702781388e-08, -1.1122329851787072e-05, -1.8782620827551000e-05, -3.4028478694381192e-05, 4.1860324095921442e-09, 8.9782446366371005e-08, 8.3449123167156358e-08,
			1.8515462215873413e-05, 1.1487405572552234e-05, -4.7682544391136616e-05, -1.1619371207416407e-06, -2.8439874313335167e-06, -4.9182076007127762e-02, -4.9585066735744476e-03, -9.2051347019150853e-04, -1.2621495872735977e-02, -4.2117113707718090e-07, -4.6778023232718624e-08, 1.4384970370429073e-07, 4.6913395635783672e-05, -5.6328929076698842e-08, -3.0896439966454636e-06, 3.0272915410023415e-06, -6.1267282944754697e-06, -7.7844842962804250e-06, -1.5203759176074527e-06, -1.7554697478772141e-05, -1.8805556464940310e-05,
			2.3587413124914747e-06, 1.4617346550949151e-06, -6.0741476772818714e-06, 2.9972351711649026e-08, -9.1858389339449786e-08, -8.6149206617847085e-04, -5.0448632100597024e-04, 2.4522372987121344e-03, -1.1765511007979512e-03, 1.1379196473981779e-09, -4.9485171516039372e-10, 4.5908468138122771e-09, -5.6328900654989411e-08, 9.9902121291961521e-05, -5.0158039499592633e-08, -3.0922517879616862e-08, -4.0880584606384218e-07, -4.0127304146153620e-07, -3.3101503760235573e-08, -5.8529269608698087e-07, -5.2675170536531368e-07,
			7.1186691457114648e-06, 4.4141420403320808e-06, -1.8332035324419849e-05, -2.2899466785020195e-06, -1.0989913334924495e-06, 1.1871420429088175e-04, -1.3960112119093537e-03, 4.6442071907222271e-03, 5.8271462330594659e-04, -5.9692446541248501e-08, -8.5008888817128536e-09, 3.6381955226261198e-08, -3.0896499083610252e-06, -5.0158259767840718e-08, 9.9022763606626540e-05, 3.5753259908233304e-07, -2.1873092919122428e-06, -2.2864071524963947e-06, 5.2195961330880891e-08, 1.3489548109646421e-06, 1.3124747511028545e-06,
			-7.3038085247389972e-05, -4.3142510548932478e-05, 1.7623035819269717e-04, 2.7194248559681000e-06, -3.9702126741758548e-06, 3.8844391703605652e-02, 5.9932605363428593e-03, -1.9515851512551308e-02, 3.0910881469026208e-04, -5.2012089872732759e-04, 6.7640221459441818e-06, -1.1122100659122225e-05, 3.0271382911450928e-06, -3.0927182592677127e-08, 3.5747075344261248e-07, 4.6223001554608345e-03, 5.1634033297887072e-06, 3.1054903956828639e-05, 1.3856163150194334e-07, 8.8812834064810886e-07, 9.8640612122835591e-07,
			-2.8430059319362044e-04, -1.7843605019152164e-04, 7.3164218338206410e-04, -1.1976680980296806e-06, -1.7039154045050964e-05, 8.6271062493324280e-02, 9.1357184574007988e-03, -9.7228705883026123e-02, 1.6330472426488996e-03, 8.2177530202898197e-06, -3.6358040233608335e-05, -1.8782417100737803e-05, -6.1268710851436481e-06, -4.0880814822230604e-07, -2.1874050162296044e-06, 5.1607376008178107e-06, 7.2711296379566193e-03, 8.6558189650531858e-05, -4.4791522668674588e-07, -3.8637926991214044e-06, -3.3028713914973196e-06,
			-3.6560875014401972e-04, -2.2915938461665064e-04, 9.4605714548379183e-04, -2.4263256364065455e-06, -1.4682227629236877e-05, 6.2579691410064697e-02, 8.6275460198521614e-03, -1.0427810996770859e-01, 1.3877020683139563e-03, 1.2013816558464896e-05, 2.1074569303891622e-05, -3.4028482332359999e-05, -7.7846016210969537e-06, -4.0127514466803405e-07, -2.2864956008561421e-06, 3.1055249564815313e-05, 8.6557418399024755e-05, 7.2550717741250992e-03, -4.2783602793861064e-07, -5.1506626732589211e-06, -4.6336040213645902e-06,
			1.6847096730998601e-06, 1.0441162885399535e-06, -4.3385439312260132e-06, -2.1070857769700524e-07, -1.4912939150235616e-07, 1.3477868633344769e-03, -4.6435141121037304e-04, 1.7559529514983296e-03, 3.2096213544718921e-04, -1.8476269403322476e-08, 1.7183057754444064e-10, 4.1886685231418141e-09, -1.5203773955363431e-06, -3.3101521523803967e-08, 5.2195883171179958e-08, 1.3856514158305799e-07, -4.4790539277528296e-07, -4.2782639297911373e-07, 7.4850339615295525e-07, 3.9271898799597693e-07, 4.3400174831731420e-07,
			5.4158231250767130e-06, 3.3657361200312153e-06, -1.3948149899078999e-05, 6.0529591792146675e-06, 1.2921157122036675e-07, -7.2531245648860931e-02, 1.3060220517218113e-02, -2.1242271177470684e-03, 1.0283550247550011e-02, -1.4358968769556668e-07, -3.0784995885824173e-08, 8.9690175286705198e-08, -1.7554675650899298e-05, -5.8529195712253568e-07, 1.3489610637407168e-06, 8.8819092525227461e-07, -3.8639450394839514e-06, -5.1508341130102053e-06, 3.9271932905649010e-07, 6.7308326833881438e-05, -3.2335199648514390e-05,
			3.4575505196698941e-06, 2.1519601887121098e-06, -8.9053210103884339e-06, 6.1496616581280250e-06, 2.6586403123474156e-07, -7.2266772389411926e-02, 1.0423545725643635e-02, -7.0324051193892956e-04, 1.1919563636183739e-02, -1.5241298001456016e-07, -3.0443008114389158e-08, 8.3357079461165995e-08, -1.8805534637067467e-05, -5.2675096640086849e-07, 1.3124824818078196e-06, 9.8648069979390129e-07, -3.3030244139808929e-06, -4.6337827370734885e-06, 4.3400223148637451e-07, -3.2335203286493197e-05, 6.7926048359367996e-05};

	arm_matrix_instance_f32 xPlusTrue, PplusTrue;

	arm_mat_init_f32(&xPlusTrue, 22, 1, xPlusDataTrue);
	arm_mat_init_f32(&PplusTrue, 21, 21, PPlusDataTrue);

	bool test1 = false;
	bool test2 = false;

	test1 = areMatricesEqual(&xPlusTrue, &xPlus);
	test2 = areMatricesEqual(&PplusTrue, &Pplus);

	bool test = test1 && test2;
}

void test_update_baro(void) {

	// Iteration Number: i = 20003 in Python Sim

	arm_matrix_instance_f32 xMinus, PMinus;

	float32_t xMinusData[22*1] = {2.6710262894630432e-01, -8.8066107034683228e-01,
		       -2.3144841194152832e-01,  3.1547418236732483e-01,
		        3.7686931610107422e+01, -1.1979366302490234e+02,
		        1.5799787500000000e+05,  6.3558452148437500e+03,
		       -6.7667036132812500e+03, -2.7864328613281250e+03,
		        1.0489179985597730e-03, -3.9222507621161640e-04,
		       -3.7785505992360413e-04,  3.2356657087802887e-02,
		        1.9722206890583038e-01, -1.8388214707374573e-01,
		        3.8933895993977785e-03, -1.4267979422584176e-03,
		       -1.2040146393701434e-03,  1.9319592043757439e-02,
		        2.8289997577667236e-01,  6.0102105140686035e-01};

	float32_t PMinusData[21*21] = {1.3929616216046270e-05, -1.3860545004718006e-04, -2.4420372210443020e-04, -2.9681275464099599e-06, 5.2331852202769369e-06, -1.7384399473667145e-01, -1.9398845732212067e-02, 4.2044091969728470e-02, 9.2673562467098236e-03, -1.1877330052811885e-06, 1.1790974667746923e-06, 4.7397002163052093e-06, -1.0159933481190819e-06, 6.4348500927735586e-07, -6.9020160253785434e-07, -6.9855191213719081e-06, -1.5654071830795147e-05, -1.9957542463089339e-05, -1.5884278781186367e-08, 8.1398218299000291e-07, 1.8623716186993988e-06,
			-1.3860547915101051e-04, 2.7374250348657370e-03, 4.8120804131031036e-03, 5.8391709899296984e-05, -1.0299191490048543e-04, 3.4196028709411621e+00, 3.8149553537368774e-01, -8.2721936702728271e-01, -1.8234488368034363e-01, -4.1553757910151035e-06, -2.4781618776614778e-05, -9.3882241344545037e-05, 1.9975073882960714e-05, -1.2655009413720109e-05, 1.3572993339039385e-05, 4.1892781155183911e-05, 3.1137504265643656e-04, 3.9449782343581319e-04, 3.1136468692238850e-07, -1.6007265003281645e-05, -3.6622292100219056e-05,
			-2.4420372210443020e-04, 4.8120804131031036e-03, 8.4805702790617943e-03, 1.0255449160467833e-04, -1.8095281848218292e-04, 6.0048418045043945e+00, 6.7034006118774414e-01, -1.4540482759475708e+00, -3.2030403614044189e-01, -7.2757889029162470e-06, -4.1510480514261872e-05, -1.6796789714135230e-04, 3.5064662370132282e-05, -2.2214238924789242e-05, 2.3825616153771989e-05, 7.3593888373579830e-05, 5.4525991436094046e-04, 7.0150173269212246e-04, 5.4621744993710308e-07, -2.8099029805161990e-05, -6.4285632106475532e-05,
			-2.9681250452995300e-06, 5.8391651691636071e-05, 1.0255440429318696e-04, 3.4339457215537550e-06, -6.2685285229235888e-06, 1.0742063075304031e-01, 1.1546486988663673e-02, -2.4581113830208778e-02, -5.2484492771327496e-03, -1.3428200418275082e-07, -2.5354162858093332e-07, -3.5210763371651410e-07, 6.3404047523363261e-07, -3.0797335170973383e-07, -1.7085137642425252e-06, 1.3629206705445540e-06, 5.1405563681328204e-06, 4.7670582716818899e-06, 6.6599734616090700e-09, 1.3167177712603007e-06, -1.4109710377852025e-07,
			5.2331738515931647e-06, -1.0299170389771461e-04, -1.8095244013238698e-04, -6.2685303419129923e-06, 2.6922818506136537e-05, -7.8646354377269745e-02, -1.8147259950637817e-02, 4.9712829291820526e-02, 8.2399593666195869e-03, -6.4286240331057343e-08, 5.3594851578964153e-07, 9.1140350377827417e-07, -5.4432352953881491e-06, -3.5435843415143609e-07, -3.5694235975824995e-06, 4.9666130053083180e-07, -1.8166329027735628e-05, -1.7147533071693033e-05, -8.8482568116887705e-07, -5.2753802037841524e-07, 2.1422256395453587e-06,
			-1.7384359240531921e-01, 3.4195947647094727e+00, 6.0048270225524902e+00, 1.0742066800594330e-01, -7.8646361827850342e-02, 1.9095388671875000e+04, 6.4310803222656250e+02, -1.3053355712890625e+03, -5.3278527832031250e+02, -9.3218935653567314e-03, -1.4330357313156128e-02, -1.6045816242694855e-02, -7.6664221286773682e-01, -3.9898794144392014e-02, 3.1212612986564636e-02, 9.3251682817935944e-02, 2.5820371508598328e-01, 1.9049970805644989e-01, 2.1009372547268867e-02, -2.9497653245925903e-01, -3.2255056500434875e-01,
			-1.9398845732212067e-02, 3.8149562478065491e-01, 6.7034024000167847e-01, 1.1546495370566845e-02, -1.8147282302379608e-02, 6.4310815429687500e+02, 6.9187477111816406e+01, -1.3935247802734375e+02, -3.0477502822875977e+01, -8.8210147805511951e-04, -1.7501449910923839e-03, -3.9597572758793831e-03, 2.2480019833892584e-03, -2.7955272234976292e-03, 2.1279109641909599e-03, 8.7480731308460236e-03, 3.3117849379777908e-02, 3.2471697777509689e-02, -3.7240843084873632e-05, 6.1400602571666241e-03, 3.2843940425664186e-03,
			4.2044077068567276e-02, -8.2721894979476929e-01, -1.4540476799011230e+00, -2.4581121280789375e-02, 4.9712870270013809e-02, -1.3053359375000000e+03, -1.3935247802734375e+02, 3.0161370849609375e+02, 6.9772872924804688e+01, 1.6311180079355836e-03, 4.1507333517074585e-03, 1.0864116251468658e-02, -1.1432580649852753e-02, 5.0749508664011955e-03, -6.7013339139521122e-03, -1.6220448538661003e-02, -8.0861009657382965e-02, -8.2185909152030945e-02, -5.1261013140901923e-04, 7.1931919082999229e-03, 1.9447200000286102e-02,
			9.2673525214195251e-03, -1.8234479427337646e-01, -3.2030385732650757e-01, -5.2484539337456226e-03, 8.2399686798453331e-03, -5.3278540039062500e+02, -3.0477510452270508e+01, 6.9772850036621094e+01, 2.3378774642944336e+01, 4.4868854456581175e-04, 7.8240851871669292e-04, 1.3157939538359642e-03, 7.0214066654443741e-03, 1.0067186085507274e-03, -1.2295185588300228e-03, -4.4601811096072197e-03, -1.5179289504885674e-02, -1.3789841905236244e-02, -1.8114647537004203e-04, 1.4340979047119617e-02, 2.2893076762557030e-02,
			-1.1877295946760569e-06, -4.1554353629180696e-06, -7.2758944043016527e-06, -1.3428437739548826e-07, -6.4277585920535785e-08, -9.3219829723238945e-03, -8.8210683315992355e-04, 1.6311373328790069e-03, 4.4869145494885743e-04, 4.0363185689784586e-05, -1.2879199040582989e-08, -4.6418819010796142e-08, -2.1855224474620627e-07, 3.9948069741058134e-08, -5.3119684650937415e-08, -3.3763443934731185e-04, 1.8951955098600592e-06, 1.3300901855473057e-06, -1.9367140779991132e-08, 6.8037671496767871e-08, 1.2607809196651942e-07,
			1.1790943972300738e-06, -2.4781616957625374e-05, -4.1510462324367836e-05, -2.5354137278554845e-07, 5.3594590099237394e-07, -1.4330195263028145e-02, -1.7501357942819595e-03, 4.1507505811750889e-03, 7.8242318704724312e-04, -1.2880674304938111e-08, 6.9281591095204931e-06, 3.2195084713748656e-06, -9.4060197852741112e-08, 5.1544507329026601e-08, -5.5796441245092865e-08, -4.7337330499885866e-08, -3.9693735743639991e-05, -1.0171554094995372e-05, 3.6282180038149647e-10, 6.7157188254896028e-08, 1.4819026716850203e-07,
			4.7396911213581916e-06, -9.3882015789858997e-05, -1.6796743148006499e-04, -3.5211127169532119e-07, 9.1142027258683811e-07, -1.6046158969402313e-02, -3.9597679860889912e-03, 1.0864128358662128e-02, 1.3157838257029653e-03, -4.6419067700753658e-08, 3.2194773211813299e-06, 1.9241546397097409e-05, -4.5661387559903233e-08, 2.5177508078400024e-08, -2.6876943337583725e-08, -2.0664036526341079e-07, -1.5911447917460464e-05, -1.3211495570430998e-05, 6.8590200186235961e-10, 3.3399057741689830e-08, 7.3457066207538446e-08,
			-1.0159919838770293e-06, 1.9975042960140854e-05, 3.5064615076407790e-05, 6.3404115735465894e-07, -5.4432339311460964e-06, -7.6664227247238159e-01, 2.2479998879134655e-03, -1.1432573199272156e-02, 7.0214071311056614e-03, -2.1854657461517490e-07, -9.4060602862100495e-08, -4.5660037528705288e-08, 6.3324834627564996e-05, 3.2801216320876847e-07, -2.2318586161418352e-06, 2.1350253973650979e-06, 3.1959334592102095e-06, 2.2989756871538702e-06, -1.6775186395534547e-06, 2.5746160190465162e-06, 1.0554041409704951e-06,
			6.4348512296419358e-07, -1.2655013051698916e-05, -2.2214248019736260e-05, -3.0797352223999042e-07, -3.5435809309092292e-07, -3.9898779243230820e-02, -2.7955265250056982e-03, 5.0749527290463448e-03, 1.0067182593047619e-03, 3.9947988028643522e-08, 5.1543704415735192e-08, 2.5177905982332049e-08, 3.2801204952193075e-07, 9.9881537607870996e-05, -1.2461036646982393e-08, -3.9942457874531101e-07, -9.8319833341520280e-07, -6.8131402031212929e-07, -4.0005534884812732e-08, -1.1984943171228224e-07, -1.1728769777619164e-07,
			-6.9019921511426219e-07, 1.3572947864304297e-05, 2.3825537937227637e-05, -1.7085138779293629e-06, -3.5694222333404468e-06, 3.1212611123919487e-02, 2.1279070060700178e-03, -6.7013199441134930e-03, -1.2295166961848736e-03, -5.3119624254804876e-08, -5.5796181896994312e-08, -2.6875278891225207e-08, -2.2318588435155107e-06, -1.2460993126239828e-08, 9.9697237601503730e-05, 5.1447364057821687e-07, 1.0645962902344763e-06, 7.1242260446524597e-07, 1.2138106342263200e-07, 1.8628136899678793e-07, 1.5327195512782055e-07,
			-6.9855414039921016e-06, 4.1893195884767920e-05, 7.3594608693383634e-05, 1.3629461363962037e-06, 4.9656944156595273e-07, 9.3252383172512054e-02, 8.7481364607810974e-03, -1.6220675781369209e-02, -4.4602290727198124e-03, -3.3763443934731185e-04, -4.7359559829374120e-08, -2.0664570854478370e-07, 2.1350792849261779e-06, -3.9942568719197880e-07, 5.1447409532556776e-07, 3.3912681974470615e-03, -1.8243730664835311e-05, -1.0668885806808248e-05, 1.8724450967511075e-07, -6.5438661067673820e-07, -1.2545430081445375e-06,
			-1.5654015442123637e-05, 3.1137399491854012e-04, 5.4525810992345214e-04, 5.1405149861238897e-06, -1.8166310837841593e-05, 2.5819978117942810e-01, 3.3117674291133881e-02, -8.0860443413257599e-02, -1.5179142355918884e-02, 1.8954998495246400e-06, -3.9693743019597605e-05, -1.5911542504909448e-05, 3.1959302759787533e-06, -9.8318946584186051e-07, 1.0645852626112173e-06, -1.8246655599796213e-05, 5.7736723683774471e-03, 7.3337927460670471e-05, -1.6252762691237876e-07, -1.6003112932594377e-06, -2.9257839742058422e-06,
			-1.9957527911174111e-05, 3.9449744508601725e-04, 7.0150109240785241e-04, 4.7670737330918200e-06, -1.7147578546428122e-05, 1.9050085544586182e-01, 3.2471716403961182e-02, -8.2185819745063782e-02, -1.3789799995720387e-02, 1.3301287253852934e-06, -1.0171463145525195e-05, -1.3211575605964754e-05, 2.2989768240222475e-06, -6.8131970465401537e-07, 7.1244176069740206e-07, -1.0669226867321413e-05, 7.3336574132554233e-05, 5.6617395021021366e-03, -1.7929770024238678e-07, -1.1132128747703973e-06, -1.9723493096535094e-06,
			-1.5884261017617973e-08, 3.1136437428358477e-07, 5.4621693834633334e-07, 6.6599823433932670e-09, -8.8482585169913364e-07, 2.1009376272559166e-02, -3.7240977690089494e-05, -5.1261001499369740e-04, -1.8114663544110954e-04, -1.9367073278431235e-08, 3.6281710968921743e-10, 6.8591676782858713e-10, -1.6775192079876433e-06, -4.0005531332099054e-08, 1.2138106342263200e-07, 1.8724382755408442e-07, -1.6252701584562601e-07, -1.7929775708580564e-07, 7.5962464052281575e-07, -6.7216433308203705e-08, -3.2213321787821769e-08,
			8.1398184192948975e-07, -1.6007259546313435e-05, -2.8099020710214972e-05, 1.3167178849471384e-06, -5.2753944146388676e-07, -2.9497650265693665e-01, 6.1400607228279114e-03, 7.1931886486709118e-03, 1.4340977184474468e-02, 6.8037699918477301e-08, 6.7156562977288559e-08, 3.3396144516473214e-08, 2.5746164737938670e-06, -1.1984943171228224e-07, 1.8628132636422379e-07, -6.5438666752015706e-07, -1.6003289147192845e-06, -1.1131944575026864e-06, -6.7216440413631062e-08, 9.9708020570687950e-05, -2.2387830256320740e-07,
			1.8623717323862365e-06, -3.6622292100219056e-05, -6.4285624830517918e-05, -1.4109659218775050e-07, 2.1422240479296306e-06, -3.2255062460899353e-01, 3.2844003289937973e-03, 1.9447183236479759e-02, 2.2893078625202179e-02, 1.2607834776190430e-07, 1.4818554916473659e-07, 7.3464313743443199e-08, 1.0554043683441705e-06, -1.1728777593589257e-07, 1.5327181301927340e-07, -1.2545442586997524e-06, -2.9257910227897810e-06, -1.9723327113752021e-06, -3.2213325340535448e-08, -2.2387811782209610e-07, 9.9674602097366005e-05};

	float32_t Rb = 0.0025f;
	float32_t pressMeas = 0.0060078953f;

	arm_mat_init_f32(&xMinus, 22, 1, xMinusData);
	arm_mat_init_f32(&PMinus, 21, 21, PMinusData);

	arm_matrix_instance_f32 xPlus, Pplus;
	float32_t xPlusData[22*1], PPlusData[21*21];

	update_baro(&xMinus, &PMinus, pressMeas, Rb, &xPlus, &Pplus, xPlusData, PPlusData);

	float32_t xPlusTrueData[22*1] = {2.6710262894630432e-01, -8.8066107034683228e-01,
								   -2.3144841194152832e-01,  3.1547418236732483e-01,
									3.7686931610107422e+01, -1.1979366302490234e+02,
									1.5799787500000000e+05,  6.3558452148437500e+03,
								   -6.7667036132812500e+03, -2.7864328613281250e+03,
									1.0489179985597730e-03, -3.9222507621161640e-04,
								   -3.7785505992360413e-04,  3.2356657087802887e-02,
									1.9722206890583038e-01, -1.8388214707374573e-01,
									3.8933895993977785e-03, -1.4267979422584176e-03,
								   -1.2040146393701434e-03,  1.9319592043757439e-02,
									2.8289997577667236e-01,  6.0102105140686035e-01};

	float32_t PPlusTrueData[21*21] = {1.3929616216046270e-05, -1.3860545004718006e-04, -2.4420372210443020e-04, -2.9681275464099599e-06, 5.2331852202769369e-06, -1.7384399473667145e-01, -1.9398845732212067e-02, 4.2044091969728470e-02, 9.2673562467098236e-03, -1.1877330052811885e-06, 1.1790974667746923e-06, 4.7397002163052093e-06, -1.0159933481190819e-06, 6.4348500927735586e-07, -6.9020160253785434e-07, -6.9855191213719081e-06, -1.5654071830795147e-05, -1.9957542463089339e-05, -1.5884278781186367e-08, 8.1398218299000291e-07, 1.8623716186993988e-06,
			-1.3860547915101051e-04, 2.7374250348657370e-03, 4.8120804131031036e-03, 5.8391709899296984e-05, -1.0299191490048543e-04, 3.4196028709411621e+00, 3.8149553537368774e-01, -8.2721936702728271e-01, -1.8234488368034363e-01, -4.1553757910151035e-06, -2.4781618776614778e-05, -9.3882241344545037e-05, 1.9975073882960714e-05, -1.2655009413720109e-05, 1.3572993339039385e-05, 4.1892781155183911e-05, 3.1137504265643656e-04, 3.9449782343581319e-04, 3.1136468692238850e-07, -1.6007265003281645e-05, -3.6622292100219056e-05,
			-2.4420372210443020e-04, 4.8120804131031036e-03, 8.4805702790617943e-03, 1.0255449160467833e-04, -1.8095281848218292e-04, 6.0048418045043945e+00, 6.7034006118774414e-01, -1.4540482759475708e+00, -3.2030403614044189e-01, -7.2757889029162470e-06, -4.1510480514261872e-05, -1.6796789714135230e-04, 3.5064662370132282e-05, -2.2214238924789242e-05, 2.3825616153771989e-05, 7.3593888373579830e-05, 5.4525991436094046e-04, 7.0150173269212246e-04, 5.4621744993710308e-07, -2.8099029805161990e-05, -6.4285632106475532e-05,
			-2.9681250452995300e-06, 5.8391651691636071e-05, 1.0255440429318696e-04, 3.4339457215537550e-06, -6.2685285229235888e-06, 1.0742063075304031e-01, 1.1546486988663673e-02, -2.4581113830208778e-02, -5.2484492771327496e-03, -1.3428200418275082e-07, -2.5354162858093332e-07, -3.5210763371651410e-07, 6.3404047523363261e-07, -3.0797335170973383e-07, -1.7085137642425252e-06, 1.3629206705445540e-06, 5.1405563681328204e-06, 4.7670582716818899e-06, 6.6599734616090700e-09, 1.3167177712603007e-06, -1.4109710377852025e-07,
			5.2331738515931647e-06, -1.0299170389771461e-04, -1.8095244013238698e-04, -6.2685303419129923e-06, 2.6922818506136537e-05, -7.8646354377269745e-02, -1.8147259950637817e-02, 4.9712829291820526e-02, 8.2399593666195869e-03, -6.4286240331057343e-08, 5.3594851578964153e-07, 9.1140350377827417e-07, -5.4432352953881491e-06, -3.5435843415143609e-07, -3.5694235975824995e-06, 4.9666130053083180e-07, -1.8166329027735628e-05, -1.7147533071693033e-05, -8.8482568116887705e-07, -5.2753802037841524e-07, 2.1422256395453587e-06,
			-1.7384359240531921e-01, 3.4195947647094727e+00, 6.0048270225524902e+00, 1.0742066800594330e-01, -7.8646361827850342e-02, 1.9095388671875000e+04, 6.4310803222656250e+02, -1.3053355712890625e+03, -5.3278527832031250e+02, -9.3218935653567314e-03, -1.4330357313156128e-02, -1.6045816242694855e-02, -7.6664221286773682e-01, -3.9898794144392014e-02, 3.1212612986564636e-02, 9.3251682817935944e-02, 2.5820371508598328e-01, 1.9049970805644989e-01, 2.1009372547268867e-02, -2.9497653245925903e-01, -3.2255056500434875e-01,
			-1.9398845732212067e-02, 3.8149562478065491e-01, 6.7034024000167847e-01, 1.1546495370566845e-02, -1.8147282302379608e-02, 6.4310815429687500e+02, 6.9187477111816406e+01, -1.3935247802734375e+02, -3.0477502822875977e+01, -8.8210147805511951e-04, -1.7501449910923839e-03, -3.9597572758793831e-03, 2.2480019833892584e-03, -2.7955272234976292e-03, 2.1279109641909599e-03, 8.7480731308460236e-03, 3.3117849379777908e-02, 3.2471697777509689e-02, -3.7240843084873632e-05, 6.1400602571666241e-03, 3.2843940425664186e-03,
			4.2044077068567276e-02, -8.2721894979476929e-01, -1.4540476799011230e+00, -2.4581121280789375e-02, 4.9712870270013809e-02, -1.3053359375000000e+03, -1.3935247802734375e+02, 3.0161370849609375e+02, 6.9772872924804688e+01, 1.6311180079355836e-03, 4.1507333517074585e-03, 1.0864116251468658e-02, -1.1432580649852753e-02, 5.0749508664011955e-03, -6.7013339139521122e-03, -1.6220448538661003e-02, -8.0861009657382965e-02, -8.2185909152030945e-02, -5.1261013140901923e-04, 7.1931919082999229e-03, 1.9447200000286102e-02,
			9.2673525214195251e-03, -1.8234479427337646e-01, -3.2030385732650757e-01, -5.2484539337456226e-03, 8.2399686798453331e-03, -5.3278540039062500e+02, -3.0477510452270508e+01, 6.9772850036621094e+01, 2.3378774642944336e+01, 4.4868854456581175e-04, 7.8240851871669292e-04, 1.3157939538359642e-03, 7.0214066654443741e-03, 1.0067186085507274e-03, -1.2295185588300228e-03, -4.4601811096072197e-03, -1.5179289504885674e-02, -1.3789841905236244e-02, -1.8114647537004203e-04, 1.4340979047119617e-02, 2.2893076762557030e-02,
			-1.1877295946760569e-06, -4.1554353629180696e-06, -7.2758944043016527e-06, -1.3428437739548826e-07, -6.4277585920535785e-08, -9.3219829723238945e-03, -8.8210683315992355e-04, 1.6311373328790069e-03, 4.4869145494885743e-04, 4.0363185689784586e-05, -1.2879199040582989e-08, -4.6418819010796142e-08, -2.1855224474620627e-07, 3.9948069741058134e-08, -5.3119684650937415e-08, -3.3763443934731185e-04, 1.8951955098600592e-06, 1.3300901855473057e-06, -1.9367140779991132e-08, 6.8037671496767871e-08, 1.2607809196651942e-07,
			1.1790943972300738e-06, -2.4781616957625374e-05, -4.1510462324367836e-05, -2.5354137278554845e-07, 5.3594590099237394e-07, -1.4330195263028145e-02, -1.7501357942819595e-03, 4.1507505811750889e-03, 7.8242318704724312e-04, -1.2880674304938111e-08, 6.9281591095204931e-06, 3.2195084713748656e-06, -9.4060197852741112e-08, 5.1544507329026601e-08, -5.5796441245092865e-08, -4.7337330499885866e-08, -3.9693735743639991e-05, -1.0171554094995372e-05, 3.6282180038149647e-10, 6.7157188254896028e-08, 1.4819026716850203e-07,
			4.7396911213581916e-06, -9.3882015789858997e-05, -1.6796743148006499e-04, -3.5211127169532119e-07, 9.1142027258683811e-07, -1.6046158969402313e-02, -3.9597679860889912e-03, 1.0864128358662128e-02, 1.3157838257029653e-03, -4.6419067700753658e-08, 3.2194773211813299e-06, 1.9241546397097409e-05, -4.5661387559903233e-08, 2.5177508078400024e-08, -2.6876943337583725e-08, -2.0664036526341079e-07, -1.5911447917460464e-05, -1.3211495570430998e-05, 6.8590200186235961e-10, 3.3399057741689830e-08, 7.3457066207538446e-08,
			-1.0159919838770293e-06, 1.9975042960140854e-05, 3.5064615076407790e-05, 6.3404115735465894e-07, -5.4432339311460964e-06, -7.6664227247238159e-01, 2.2479998879134655e-03, -1.1432573199272156e-02, 7.0214071311056614e-03, -2.1854657461517490e-07, -9.4060602862100495e-08, -4.5660037528705288e-08, 6.3324834627564996e-05, 3.2801216320876847e-07, -2.2318586161418352e-06, 2.1350253973650979e-06, 3.1959334592102095e-06, 2.2989756871538702e-06, -1.6775186395534547e-06, 2.5746160190465162e-06, 1.0554041409704951e-06,
			6.4348512296419358e-07, -1.2655013051698916e-05, -2.2214248019736260e-05, -3.0797352223999042e-07, -3.5435809309092292e-07, -3.9898779243230820e-02, -2.7955265250056982e-03, 5.0749527290463448e-03, 1.0067182593047619e-03, 3.9947988028643522e-08, 5.1543704415735192e-08, 2.5177905982332049e-08, 3.2801204952193075e-07, 9.9881537607870996e-05, -1.2461036646982393e-08, -3.9942457874531101e-07, -9.8319833341520280e-07, -6.8131402031212929e-07, -4.0005534884812732e-08, -1.1984943171228224e-07, -1.1728769777619164e-07,
			-6.9019921511426219e-07, 1.3572947864304297e-05, 2.3825537937227637e-05, -1.7085138779293629e-06, -3.5694222333404468e-06, 3.1212611123919487e-02, 2.1279070060700178e-03, -6.7013199441134930e-03, -1.2295166961848736e-03, -5.3119624254804876e-08, -5.5796181896994312e-08, -2.6875278891225207e-08, -2.2318588435155107e-06, -1.2460993126239828e-08, 9.9697237601503730e-05, 5.1447364057821687e-07, 1.0645962902344763e-06, 7.1242260446524597e-07, 1.2138106342263200e-07, 1.8628136899678793e-07, 1.5327195512782055e-07,
			-6.9855414039921016e-06, 4.1893195884767920e-05, 7.3594608693383634e-05, 1.3629461363962037e-06, 4.9656944156595273e-07, 9.3252383172512054e-02, 8.7481364607810974e-03, -1.6220675781369209e-02, -4.4602290727198124e-03, -3.3763443934731185e-04, -4.7359559829374120e-08, -2.0664570854478370e-07, 2.1350792849261779e-06, -3.9942568719197880e-07, 5.1447409532556776e-07, 3.3912681974470615e-03, -1.8243730664835311e-05, -1.0668885806808248e-05, 1.8724450967511075e-07, -6.5438661067673820e-07, -1.2545430081445375e-06,
			-1.5654015442123637e-05, 3.1137399491854012e-04, 5.4525810992345214e-04, 5.1405149861238897e-06, -1.8166310837841593e-05, 2.5819978117942810e-01, 3.3117674291133881e-02, -8.0860443413257599e-02, -1.5179142355918884e-02, 1.8954998495246400e-06, -3.9693743019597605e-05, -1.5911542504909448e-05, 3.1959302759787533e-06, -9.8318946584186051e-07, 1.0645852626112173e-06, -1.8246655599796213e-05, 5.7736723683774471e-03, 7.3337927460670471e-05, -1.6252762691237876e-07, -1.6003112932594377e-06, -2.9257839742058422e-06,
			-1.9957527911174111e-05, 3.9449744508601725e-04, 7.0150109240785241e-04, 4.7670737330918200e-06, -1.7147578546428122e-05, 1.9050085544586182e-01, 3.2471716403961182e-02, -8.2185819745063782e-02, -1.3789799995720387e-02, 1.3301287253852934e-06, -1.0171463145525195e-05, -1.3211575605964754e-05, 2.2989768240222475e-06, -6.8131970465401537e-07, 7.1244176069740206e-07, -1.0669226867321413e-05, 7.3336574132554233e-05, 5.6617395021021366e-03, -1.7929770024238678e-07, -1.1132128747703973e-06, -1.9723493096535094e-06,
			-1.5884261017617973e-08, 3.1136437428358477e-07, 5.4621693834633334e-07, 6.6599823433932670e-09, -8.8482585169913364e-07, 2.1009376272559166e-02, -3.7240977690089494e-05, -5.1261001499369740e-04, -1.8114663544110954e-04, -1.9367073278431235e-08, 3.6281710968921743e-10, 6.8591676782858713e-10, -1.6775192079876433e-06, -4.0005531332099054e-08, 1.2138106342263200e-07, 1.8724382755408442e-07, -1.6252701584562601e-07, -1.7929775708580564e-07, 7.5962464052281575e-07, -6.7216433308203705e-08, -3.2213321787821769e-08,
			8.1398184192948975e-07, -1.6007259546313435e-05, -2.8099020710214972e-05, 1.3167178849471384e-06, -5.2753944146388676e-07, -2.9497650265693665e-01, 6.1400607228279114e-03, 7.1931886486709118e-03, 1.4340977184474468e-02, 6.8037699918477301e-08, 6.7156562977288559e-08, 3.3396144516473214e-08, 2.5746164737938670e-06, -1.1984943171228224e-07, 1.8628132636422379e-07, -6.5438666752015706e-07, -1.6003289147192845e-06, -1.1131944575026864e-06, -6.7216440413631062e-08, 9.9708020570687950e-05, -2.2387830256320740e-07,
			1.8623717323862365e-06, -3.6622292100219056e-05, -6.4285624830517918e-05, -1.4109659218775050e-07, 2.1422240479296306e-06, -3.2255062460899353e-01, 3.2844003289937973e-03, 1.9447183236479759e-02, 2.2893078625202179e-02, 1.2607834776190430e-07, 1.4818554916473659e-07, 7.3464313743443199e-08, 1.0554043683441705e-06, -1.1728777593589257e-07, 1.5327181301927340e-07, -1.2545442586997524e-06, -2.9257910227897810e-06, -1.9723327113752021e-06, -3.2213325340535448e-08, -2.2387811782209610e-07, 9.9674602097366005e-05};

	arm_matrix_instance_f32 xPlusTrue, PplusTrue;
	arm_mat_init_f32(&xPlusTrue, 22, 1, xPlusTrueData);
	arm_mat_init_f32(&PplusTrue, 21, 21, PPlusTrueData);

	bool test1 = false;
	bool test2 = false;

	test1 = areMatricesEqual(&xPlusTrue, &xPlus);
	test2 = areMatricesEqual(&PplusTrue, &Pplus);
	bool test = test1 && test2;
}

void test_eig(void) {

	float64_t testA[] = {2.5107394903898239e-03, 1.3870508410036564e-03, 4.1348565719090402e-04, -1.2105195992262452e-06, 1.0964167813654058e-05, 3.6244294606149197e-03, -1.2846818193793297e-02, 7.9308077692985535e-02, 3.5281997406855226e-04, -1.4956512313801795e-04, -6.1641869251616299e-05, -4.9729995225789025e-05, 2.4503715394530445e-05, -6.9716980988232535e-07, 2.8424997253750917e-06, -9.2655711341649294e-05, -8.7311973402393050e-06, 2.7717702323570848e-05, 2.5163149075524416e-06, 3.5494674648361979e-06, 1.8817178215613239e-06,
			1.3870507245883346e-03, 8.1886450061574578e-04, 2.3266782227437943e-04, -6.7888652210967848e-07, 6.1673517848248594e-06, 2.0476393401622772e-03, -7.2076767683029175e-03, 4.4740043580532074e-02, 2.3241442977450788e-04, -7.8583412687294185e-05, -4.2562889575492591e-05, -2.8463964554248378e-05, 1.4163388186716475e-05, -4.1093636582445470e-07, 1.6388738686146098e-06, -5.0807924708351493e-05, 1.3446798220684286e-05, 1.4755168194824364e-05, 1.4184652172843926e-06, 1.9934200281568337e-06, 1.0558607073107851e-06,
			4.1348559898324311e-04, 2.3266780772246420e-04, 7.3661096394062042e-05, -1.9892927127784787e-07, 1.8006873006015667e-06, 6.0063292039558291e-04, -2.1141790784895420e-03, 1.3045616447925568e-02, 6.3182415033224970e-05, -2.4244429368991405e-05, -1.0794721674756147e-05, -9.6233252406818792e-06, 4.0759332478046417e-06, -1.1547382428034325e-07, 4.6977430656625074e-07, -1.5842888387851417e-05, -9.1125809831282822e-07, 1.1417474524932913e-05, 4.1246372006753518e-07, 5.7899666217053891e-07, 3.0758295110899780e-07,
			-1.2105170981158153e-06, -6.7888493049395038e-07, -1.9892888758477056e-07, 3.9228316950357112e-07, -6.2305929304784513e-08, 6.2159284652807401e-07, 1.0580982780084014e-04, -2.6186555624008179e-04, 1.8523078892940248e-07, 4.4635051210661914e-08, 1.5559155741584618e-08, 1.3357949413261849e-08, 1.1511580311207581e-07, -1.0933271710200643e-07, -3.2273410965899529e-07, 2.0881076423506784e-08, 1.2791484316210244e-08, 1.2963364603990613e-08, -4.9920871880715367e-09, -5.2465015665248416e-10, -1.6203637320799658e-09,
			1.0964507964672521e-05, 6.1675450524489861e-06, 1.8007427797783748e-06, -6.2304401637902629e-08, 8.2087535702157766e-07, 1.6809068256407045e-05, -3.4061339101754129e-04, 1.4484929852187634e-03, -5.0715198085526936e-07, -3.6733649722009432e-07, -1.2941723070980515e-07, -1.0716044585024065e-07, -7.4508307079668157e-07, -1.9485790403450665e-07, -1.2857272224664484e-07, -7.4732980692715500e-08, -5.1507078069334966e-07, -2.4529884967705584e-07, -4.2692278157119290e-08, -1.5160398447733314e-07, -5.6402740256089601e-08,
			3.6244466900825500e-03, 2.0476495847105980e-03, 6.0063553974032402e-04, 6.2165213421394583e-07, 1.6809141015983187e-05, 4.5943939685821533e-01, -5.7779666967689991e-03, 1.7302942276000977e-01, -5.7389553636312485e-02, -1.3826752547174692e-04, -4.3787229515146464e-05, -3.5405118978815153e-05, -2.5520569179207087e-03, 5.9971585869789124e-05, -1.3714710075873882e-04, 3.2266182824969292e-04, 5.8854091912508011e-04, 4.4556905049830675e-04, 5.8347060985397547e-05, 2.1537233260460198e-04, 6.6922919359058142e-05,
			-1.2846780940890312e-02, -7.2076544165611267e-03, -2.1141730248928070e-03, 1.0580992966424674e-04, -3.4062186023220420e-04, -5.7783816009759903e-03, 6.2996292114257812e-01, -1.4853866100311279e+00, -3.5820071934722364e-04, 5.5491639068350196e-04, 2.0981105626560748e-04, 1.7389358254149556e-04, 5.6143430992960930e-04, -1.6836579015944153e-04, -1.3457383029162884e-03, 1.9399459415581077e-04, 1.4092064520809799e-04, 1.0764015314634889e-04, -2.2570191504200920e-05, 1.3674369256477803e-04, -6.9707697548437864e-05,
			7.9310439527034760e-02, 4.4741448014974594e-02, 1.3046003878116608e-02, -2.6185708702541888e-04, 1.4484681887552142e-03, 1.7302660644054413e-01, -1.4853343963623047e+00, 5.7770266532897949e+00, -2.9659565188921988e-04, -2.7608654927462339e-03, -9.9651701748371124e-04, -8.0658157821744680e-04, -2.7222814969718456e-03, -4.0339294355362654e-04, -1.0821479372680187e-03, 4.8641275498084724e-04, -3.2360495533794165e-03, -1.1922345729544759e-03, -3.2302935142070055e-04, -9.0769183589145541e-04, -2.7783834957517684e-04,
			3.5281977034173906e-04, 2.3241416784003377e-04, 6.3182378653436899e-05, 1.8523188316521555e-07, -5.0712287702481262e-07, -5.7389538735151291e-02, -3.5820595803670585e-04, -2.9663741588592529e-04, 1.2771101668477058e-02, -4.2966996261384338e-05, -3.0355138733284548e-05, -1.8712098608375527e-05, 7.7484786743298173e-04, -2.0578119801939465e-05, 4.7269837523344904e-05, -1.1147045006509870e-04, -1.7707193910609931e-04, -9.6887917607091367e-05, -1.7920267055160366e-05, -5.9676600358216092e-05, -2.8807564376620576e-05,
			-1.4956507948227227e-04, -7.8583390859421343e-05, -2.4244423912023194e-05, 4.4634276719079935e-08, -3.6733470665240020e-07, -1.3826903887093067e-04, 5.5491190869361162e-04, -2.7608594391494989e-03, -4.2966923501808196e-05, 2.0496569050010294e-05, 4.9658983698463999e-06, 4.0205377445090562e-06, -5.5960828149181907e-07, -2.6980835343692888e-08, -2.3648182079227809e-08, -9.1610409072018228e-06, 5.5260265980905388e-06, -1.0604027238514391e-06, -5.0587885880304384e-08, -4.6675118881012168e-08, -3.5594727165744189e-08,
			-6.1641869251616299e-05, -4.2562878661556169e-05, -1.0794716217787936e-05, 1.5558889288058708e-08, -1.2941748650519003e-07, -4.3787898903246969e-05, 2.0980952831450850e-04, -9.9651748314499855e-04, -3.0355025955941528e-05, 4.9659051910566632e-06, 9.8640703072305769e-06, 2.0764571218023775e-06, -1.6215861364798911e-07, -1.0480809109481015e-08, -6.2002554201967541e-09, 9.3000808192300610e-06, -5.7644897424324881e-06, -1.5230972394419950e-07, -1.9156566111178108e-08, -1.9603312750859914e-08, -1.3858532099675358e-08,
			-4.9729962483979762e-05, -2.8463950002333149e-05, -9.6233188742189668e-06, 1.3357685624271198e-08, -1.0716085085960003e-07, -3.5405697417445481e-05, 1.7389224376529455e-04, -8.0657808575779200e-04, -1.8712038581725210e-05, 4.0205277400673367e-06, 2.0764637156389654e-06, 7.4262979978811927e-06, -5.8993389728811962e-08, -9.7563832568425823e-09, 1.3298082079948870e-10, 6.6884031184599735e-06, 5.3126564125705045e-07, -2.8473093607317423e-06, -1.7268568797135231e-08, -2.1258269811141872e-08, -1.4033791906342685e-08,
			2.4504384782630950e-05, 1.4163776540954132e-05, 4.0760405681794509e-06, 1.1511843922562548e-07, -7.4506027658571838e-07, -2.5520545896142721e-03, 5.6145124835893512e-04, -2.7220603078603745e-03, 7.7484676148742437e-04, -5.5957360700631398e-07, -1.6214436016070977e-07, -5.8981306949590362e-08, 7.2132883360609412e-05, 1.4755420352230431e-07, -1.7513864349893993e-06, 4.2617612052708864e-06, 7.2065454332914669e-06, 4.3029453991039190e-06, -2.0605182271538069e-06, 1.8851620779969380e-06, 8.1687727515600272e-07,
			-6.9711222749901935e-07, -4.1090302715929283e-07, -1.1546439537823971e-07, -1.0933236893606590e-07, -1.9485705138322373e-07, 5.9971629525534809e-05, -1.6836426220834255e-04, -4.0338345570489764e-04, -2.0578117982950062e-05, -2.6980929490605376e-08, -1.0480847301153062e-08, -9.7563237488884624e-09, 1.4755256927401206e-07, 9.9922661320306361e-05, -4.3617159661835103e-08, -7.3328088490143273e-08, -1.4194748132467794e-07, -1.1123331944418169e-07, -3.0141002582695364e-08, -5.6034750173239445e-08, -1.6472405661716039e-08,
			2.8425549771782244e-06, 1.6389063830501982e-06, 4.6978314571788360e-07, -3.2273388228531985e-07, -1.2857113063091674e-07, -1.3714688248001039e-04, -1.3457374880090356e-03, -1.0821323376148939e-03, 4.7269793867599219e-05, -2.3645359448210002e-08, -6.1992735389537756e-09, 1.3397868925402179e-10, -1.7513864349893993e-06, -4.3617124134698315e-08, 9.9766286439262331e-05, 2.7176085382052406e-07, 4.7291382543335203e-07, 2.6270365083291836e-07, 5.4778496405560873e-08, 1.0859947252583879e-07, 5.2211170498139836e-08,
			-9.2656009655911475e-05, -5.0808095693355426e-05, -1.5842939319554716e-05, 2.0877987338963067e-08, -7.4745905465078977e-08, 3.2266258494928479e-04, 1.9397563301026821e-04, 4.8632881953381002e-04, -1.1147020995849743e-04, -9.1610863819369115e-06, 9.3000644483254291e-06, 6.6884131229016930e-06, 4.2618012230377644e-06, -7.3329864846982673e-08, 2.7176145067642210e-07, 3.7262580008246005e-04, -6.0031734392396174e-06, 7.7199420047691092e-06, -1.1730424631650749e-07, -3.5823987332150864e-07, -1.5919896156901814e-07,
			-8.7308162619592622e-06, 1.3446997400023974e-05, -9.1119142098250450e-07, 1.2792117587423490e-08, -5.1507674925233005e-07, 5.8854307280853391e-04, 1.4092151832301170e-04, -3.2360393088310957e-03, -1.7707135702949017e-04, 5.5260106819332577e-06, -5.7644833759695757e-06, 5.3121925702726003e-07, 7.2066013672156259e-06, -1.4194836239767028e-07, 4.7291825922002317e-07, -6.0032489272998646e-06, 1.3652035268023610e-03, 3.4823813621187583e-05, -1.6015627579690772e-07, -6.0524831724251271e-07, -2.8109784011576266e-07,
			2.7718067940440960e-05, 1.4755489246454090e-05, 1.1417545465519652e-05, 1.2960233775061170e-08, -2.4526320885343011e-07, 4.4557641376741230e-04, 1.0760546865640208e-04, -1.1919669341295958e-03, -9.6888055850286037e-05, -1.0604258022794966e-06, -1.5230133953991754e-07, -2.8473086786107160e-06, 4.3031213863287121e-06, -1.1122734377977395e-07, 2.6271544584233197e-07, 7.7199410952744074e-06, 3.4824162867153063e-05, 5.9815496206283569e-04, -9.7324303283130575e-08, -3.6442099826672347e-07, -1.4917364410393930e-07,
			2.5163089958368801e-06, 1.4184622614266118e-06, 4.1246289583796170e-07, -4.9922954659109564e-09, -4.2691251422866117e-08, 5.8347246522316709e-05, -2.2571251975023188e-05, -3.2302382169291377e-04, -1.7920274331117980e-05, -5.0587942723723245e-08, -1.9156656705376918e-08, -1.7268661167690880e-08, -2.0605132249329472e-06, -3.0140814288870388e-08, 5.4778865887783468e-08, -1.1730421078937070e-07, -1.6015647474887373e-07, -9.7321361636204529e-08, -8.7882545685147306e-10, -3.8049517314675541e-08, -1.5173744927210464e-08,
			3.5494731491780840e-06, 1.9934229840146145e-06, 5.7899785588233499e-07, -5.2460952248978288e-10, -1.5160499344801792e-07, 2.1537224529311061e-04, 1.3674370711669326e-04, -9.0769876260310411e-04, -5.9676578530343249e-05, -4.6677758547275516e-08, -1.9604240009130081e-08, -2.1259115356997427e-08, 1.8851602590075345e-06, -5.6034878070931882e-08, 1.0859941568241993e-07, -3.5823461530526401e-07, -6.0524422451635473e-07, -3.6441554129851284e-07, -3.8049350337132637e-08, 9.9831479019485414e-05, -6.7397593284113100e-08,
			1.8817100908563589e-06, 1.0558570693319780e-06, 3.0758187108403945e-07, -1.6203666186598298e-09, -5.6403383297265464e-08, 6.6922788391821086e-05, -6.9707821239717305e-05, -2.7784364647231996e-04, -2.8807546186726540e-05, -3.5595824954270938e-08, -1.3859057013121401e-08, -1.4034136519569529e-08, 8.1687556985343690e-07, -1.6472480268703293e-08, 5.2211071022156830e-08, -1.5919671625397314e-07, -2.8109491267969133e-07, -1.4916990664914920e-07, -1.5173620582231706e-08, -6.7397550651548954e-08, 9.9969409347977489e-05};

	float64_t realD[21], imagD[21], realV[21*21], imagV[21*21];

	eig(testA, realD, imagD, realV, imagV, 21);

	arm_matrix_instance_f64 V, D;
	arm_mat_init_f64(&V, 21, 21, realV);
	arm_mat_init_f64(&D, 21, 1, realD);

	printMatrixDouble(&V);
	printMatrixDouble(&D);
}

void test_nearest_PSD(void) {

	// iteration in Python = 4300

	arm_matrix_instance_f32 P;

	float32_t PData[21*21] = {2.5107394903898239e-03, 1.3870508410036564e-03, 4.1348565719090402e-04, -1.2105195992262452e-06, 1.0964167813654058e-05, 3.6244294606149197e-03, -1.2846818193793297e-02, 7.9308077692985535e-02, 3.5281997406855226e-04, -1.4956512313801795e-04, -6.1641869251616299e-05, -4.9729995225789025e-05, 2.4503715394530445e-05, -6.9716980988232535e-07, 2.8424997253750917e-06, -9.2655711341649294e-05, -8.7311973402393050e-06, 2.7717702323570848e-05, 2.5163149075524416e-06, 3.5494674648361979e-06, 1.8817178215613239e-06,
			1.3870507245883346e-03, 8.1886450061574578e-04, 2.3266782227437943e-04, -6.7888652210967848e-07, 6.1673517848248594e-06, 2.0476393401622772e-03, -7.2076767683029175e-03, 4.4740043580532074e-02, 2.3241442977450788e-04, -7.8583412687294185e-05, -4.2562889575492591e-05, -2.8463964554248378e-05, 1.4163388186716475e-05, -4.1093636582445470e-07, 1.6388738686146098e-06, -5.0807924708351493e-05, 1.3446798220684286e-05, 1.4755168194824364e-05, 1.4184652172843926e-06, 1.9934200281568337e-06, 1.0558607073107851e-06,
			4.1348559898324311e-04, 2.3266780772246420e-04, 7.3661096394062042e-05, -1.9892927127784787e-07, 1.8006873006015667e-06, 6.0063292039558291e-04, -2.1141790784895420e-03, 1.3045616447925568e-02, 6.3182415033224970e-05, -2.4244429368991405e-05, -1.0794721674756147e-05, -9.6233252406818792e-06, 4.0759332478046417e-06, -1.1547382428034325e-07, 4.6977430656625074e-07, -1.5842888387851417e-05, -9.1125809831282822e-07, 1.1417474524932913e-05, 4.1246372006753518e-07, 5.7899666217053891e-07, 3.0758295110899780e-07,
			-1.2105170981158153e-06, -6.7888493049395038e-07, -1.9892888758477056e-07, 3.9228316950357112e-07, -6.2305929304784513e-08, 6.2159284652807401e-07, 1.0580982780084014e-04, -2.6186555624008179e-04, 1.8523078892940248e-07, 4.4635051210661914e-08, 1.5559155741584618e-08, 1.3357949413261849e-08, 1.1511580311207581e-07, -1.0933271710200643e-07, -3.2273410965899529e-07, 2.0881076423506784e-08, 1.2791484316210244e-08, 1.2963364603990613e-08, -4.9920871880715367e-09, -5.2465015665248416e-10, -1.6203637320799658e-09,
			1.0964507964672521e-05, 6.1675450524489861e-06, 1.8007427797783748e-06, -6.2304401637902629e-08, 8.2087535702157766e-07, 1.6809068256407045e-05, -3.4061339101754129e-04, 1.4484929852187634e-03, -5.0715198085526936e-07, -3.6733649722009432e-07, -1.2941723070980515e-07, -1.0716044585024065e-07, -7.4508307079668157e-07, -1.9485790403450665e-07, -1.2857272224664484e-07, -7.4732980692715500e-08, -5.1507078069334966e-07, -2.4529884967705584e-07, -4.2692278157119290e-08, -1.5160398447733314e-07, -5.6402740256089601e-08,
			3.6244466900825500e-03, 2.0476495847105980e-03, 6.0063553974032402e-04, 6.2165213421394583e-07, 1.6809141015983187e-05, 4.5943939685821533e-01, -5.7779666967689991e-03, 1.7302942276000977e-01, -5.7389553636312485e-02, -1.3826752547174692e-04, -4.3787229515146464e-05, -3.5405118978815153e-05, -2.5520569179207087e-03, 5.9971585869789124e-05, -1.3714710075873882e-04, 3.2266182824969292e-04, 5.8854091912508011e-04, 4.4556905049830675e-04, 5.8347060985397547e-05, 2.1537233260460198e-04, 6.6922919359058142e-05,
			-1.2846780940890312e-02, -7.2076544165611267e-03, -2.1141730248928070e-03, 1.0580992966424674e-04, -3.4062186023220420e-04, -5.7783816009759903e-03, 6.2996292114257812e-01, -1.4853866100311279e+00, -3.5820071934722364e-04, 5.5491639068350196e-04, 2.0981105626560748e-04, 1.7389358254149556e-04, 5.6143430992960930e-04, -1.6836579015944153e-04, -1.3457383029162884e-03, 1.9399459415581077e-04, 1.4092064520809799e-04, 1.0764015314634889e-04, -2.2570191504200920e-05, 1.3674369256477803e-04, -6.9707697548437864e-05,
			7.9310439527034760e-02, 4.4741448014974594e-02, 1.3046003878116608e-02, -2.6185708702541888e-04, 1.4484681887552142e-03, 1.7302660644054413e-01, -1.4853343963623047e+00, 5.7770266532897949e+00, -2.9659565188921988e-04, -2.7608654927462339e-03, -9.9651701748371124e-04, -8.0658157821744680e-04, -2.7222814969718456e-03, -4.0339294355362654e-04, -1.0821479372680187e-03, 4.8641275498084724e-04, -3.2360495533794165e-03, -1.1922345729544759e-03, -3.2302935142070055e-04, -9.0769183589145541e-04, -2.7783834957517684e-04,
			3.5281977034173906e-04, 2.3241416784003377e-04, 6.3182378653436899e-05, 1.8523188316521555e-07, -5.0712287702481262e-07, -5.7389538735151291e-02, -3.5820595803670585e-04, -2.9663741588592529e-04, 1.2771101668477058e-02, -4.2966996261384338e-05, -3.0355138733284548e-05, -1.8712098608375527e-05, 7.7484786743298173e-04, -2.0578119801939465e-05, 4.7269837523344904e-05, -1.1147045006509870e-04, -1.7707193910609931e-04, -9.6887917607091367e-05, -1.7920267055160366e-05, -5.9676600358216092e-05, -2.8807564376620576e-05,
			-1.4956507948227227e-04, -7.8583390859421343e-05, -2.4244423912023194e-05, 4.4634276719079935e-08, -3.6733470665240020e-07, -1.3826903887093067e-04, 5.5491190869361162e-04, -2.7608594391494989e-03, -4.2966923501808196e-05, 2.0496569050010294e-05, 4.9658983698463999e-06, 4.0205377445090562e-06, -5.5960828149181907e-07, -2.6980835343692888e-08, -2.3648182079227809e-08, -9.1610409072018228e-06, 5.5260265980905388e-06, -1.0604027238514391e-06, -5.0587885880304384e-08, -4.6675118881012168e-08, -3.5594727165744189e-08,
			-6.1641869251616299e-05, -4.2562878661556169e-05, -1.0794716217787936e-05, 1.5558889288058708e-08, -1.2941748650519003e-07, -4.3787898903246969e-05, 2.0980952831450850e-04, -9.9651748314499855e-04, -3.0355025955941528e-05, 4.9659051910566632e-06, 9.8640703072305769e-06, 2.0764571218023775e-06, -1.6215861364798911e-07, -1.0480809109481015e-08, -6.2002554201967541e-09, 9.3000808192300610e-06, -5.7644897424324881e-06, -1.5230972394419950e-07, -1.9156566111178108e-08, -1.9603312750859914e-08, -1.3858532099675358e-08,
			-4.9729962483979762e-05, -2.8463950002333149e-05, -9.6233188742189668e-06, 1.3357685624271198e-08, -1.0716085085960003e-07, -3.5405697417445481e-05, 1.7389224376529455e-04, -8.0657808575779200e-04, -1.8712038581725210e-05, 4.0205277400673367e-06, 2.0764637156389654e-06, 7.4262979978811927e-06, -5.8993389728811962e-08, -9.7563832568425823e-09, 1.3298082079948870e-10, 6.6884031184599735e-06, 5.3126564125705045e-07, -2.8473093607317423e-06, -1.7268568797135231e-08, -2.1258269811141872e-08, -1.4033791906342685e-08,
			2.4504384782630950e-05, 1.4163776540954132e-05, 4.0760405681794509e-06, 1.1511843922562548e-07, -7.4506027658571838e-07, -2.5520545896142721e-03, 5.6145124835893512e-04, -2.7220603078603745e-03, 7.7484676148742437e-04, -5.5957360700631398e-07, -1.6214436016070977e-07, -5.8981306949590362e-08, 7.2132883360609412e-05, 1.4755420352230431e-07, -1.7513864349893993e-06, 4.2617612052708864e-06, 7.2065454332914669e-06, 4.3029453991039190e-06, -2.0605182271538069e-06, 1.8851620779969380e-06, 8.1687727515600272e-07,
			-6.9711222749901935e-07, -4.1090302715929283e-07, -1.1546439537823971e-07, -1.0933236893606590e-07, -1.9485705138322373e-07, 5.9971629525534809e-05, -1.6836426220834255e-04, -4.0338345570489764e-04, -2.0578117982950062e-05, -2.6980929490605376e-08, -1.0480847301153062e-08, -9.7563237488884624e-09, 1.4755256927401206e-07, 9.9922661320306361e-05, -4.3617159661835103e-08, -7.3328088490143273e-08, -1.4194748132467794e-07, -1.1123331944418169e-07, -3.0141002582695364e-08, -5.6034750173239445e-08, -1.6472405661716039e-08,
			2.8425549771782244e-06, 1.6389063830501982e-06, 4.6978314571788360e-07, -3.2273388228531985e-07, -1.2857113063091674e-07, -1.3714688248001039e-04, -1.3457374880090356e-03, -1.0821323376148939e-03, 4.7269793867599219e-05, -2.3645359448210002e-08, -6.1992735389537756e-09, 1.3397868925402179e-10, -1.7513864349893993e-06, -4.3617124134698315e-08, 9.9766286439262331e-05, 2.7176085382052406e-07, 4.7291382543335203e-07, 2.6270365083291836e-07, 5.4778496405560873e-08, 1.0859947252583879e-07, 5.2211170498139836e-08,
			-9.2656009655911475e-05, -5.0808095693355426e-05, -1.5842939319554716e-05, 2.0877987338963067e-08, -7.4745905465078977e-08, 3.2266258494928479e-04, 1.9397563301026821e-04, 4.8632881953381002e-04, -1.1147020995849743e-04, -9.1610863819369115e-06, 9.3000644483254291e-06, 6.6884131229016930e-06, 4.2618012230377644e-06, -7.3329864846982673e-08, 2.7176145067642210e-07, 3.7262580008246005e-04, -6.0031734392396174e-06, 7.7199420047691092e-06, -1.1730424631650749e-07, -3.5823987332150864e-07, -1.5919896156901814e-07,
			-8.7308162619592622e-06, 1.3446997400023974e-05, -9.1119142098250450e-07, 1.2792117587423490e-08, -5.1507674925233005e-07, 5.8854307280853391e-04, 1.4092151832301170e-04, -3.2360393088310957e-03, -1.7707135702949017e-04, 5.5260106819332577e-06, -5.7644833759695757e-06, 5.3121925702726003e-07, 7.2066013672156259e-06, -1.4194836239767028e-07, 4.7291825922002317e-07, -6.0032489272998646e-06, 1.3652035268023610e-03, 3.4823813621187583e-05, -1.6015627579690772e-07, -6.0524831724251271e-07, -2.8109784011576266e-07,
			2.7718067940440960e-05, 1.4755489246454090e-05, 1.1417545465519652e-05, 1.2960233775061170e-08, -2.4526320885343011e-07, 4.4557641376741230e-04, 1.0760546865640208e-04, -1.1919669341295958e-03, -9.6888055850286037e-05, -1.0604258022794966e-06, -1.5230133953991754e-07, -2.8473086786107160e-06, 4.3031213863287121e-06, -1.1122734377977395e-07, 2.6271544584233197e-07, 7.7199410952744074e-06, 3.4824162867153063e-05, 5.9815496206283569e-04, -9.7324303283130575e-08, -3.6442099826672347e-07, -1.4917364410393930e-07,
			2.5163089958368801e-06, 1.4184622614266118e-06, 4.1246289583796170e-07, -4.9922954659109564e-09, -4.2691251422866117e-08, 5.8347246522316709e-05, -2.2571251975023188e-05, -3.2302382169291377e-04, -1.7920274331117980e-05, -5.0587942723723245e-08, -1.9156656705376918e-08, -1.7268661167690880e-08, -2.0605132249329472e-06, -3.0140814288870388e-08, 5.4778865887783468e-08, -1.1730421078937070e-07, -1.6015647474887373e-07, -9.7321361636204529e-08, -8.7882545685147306e-10, -3.8049517314675541e-08, -1.5173744927210464e-08,
			3.5494731491780840e-06, 1.9934229840146145e-06, 5.7899785588233499e-07, -5.2460952248978288e-10, -1.5160499344801792e-07, 2.1537224529311061e-04, 1.3674370711669326e-04, -9.0769876260310411e-04, -5.9676578530343249e-05, -4.6677758547275516e-08, -1.9604240009130081e-08, -2.1259115356997427e-08, 1.8851602590075345e-06, -5.6034878070931882e-08, 1.0859941568241993e-07, -3.5823461530526401e-07, -6.0524422451635473e-07, -3.6441554129851284e-07, -3.8049350337132637e-08, 9.9831479019485414e-05, -6.7397593284113100e-08,
			1.8817100908563589e-06, 1.0558570693319780e-06, 3.0758187108403945e-07, -1.6203666186598298e-09, -5.6403383297265464e-08, 6.6922788391821086e-05, -6.9707821239717305e-05, -2.7784364647231996e-04, -2.8807546186726540e-05, -3.5595824954270938e-08, -1.3859057013121401e-08, -1.4034136519569529e-08, 8.1687556985343690e-07, -1.6472480268703293e-08, 5.2211071022156830e-08, -1.5919671625397314e-07, -2.8109491267969133e-07, -1.4916990664914920e-07, -1.5173620582231706e-08, -6.7397550651548954e-08, 9.9969409347977489e-05};

	arm_mat_init_f32(&P, 21, 21, PData);

	arm_matrix_instance_f32 PCorrect;
	float32_t PCorrectData[21*21];
	nearestPSD(&P, &PCorrect, PCorrectData);

	float32_t PDataTrue[21*21] = {2.5107417334987252e-03, 1.3870514504377904e-03, 4.1348555943753260e-04, -1.2126775191626709e-06, 1.0981982675905102e-05, 3.6244381444420761e-03, -1.2846799695097090e-02, 7.9309258533828975e-02, 3.5282081897809568e-04, -1.4955324883539350e-04, -6.1635658608634171e-05, -4.9724924136941580e-05, 2.4492670549430152e-05, -6.9744882791964069e-07, 2.8394202803995727e-06, -9.2654659777810020e-05, -8.7310282262389662e-06, 2.7717855186206110e-05, 2.4743796202823709e-06, 3.5495171524189489e-06, 1.8816965739920804e-06,
			1.3870514504377904e-03, 8.1886469933355307e-04, 2.3266779456548401e-04, -6.7952838455816301e-07, 6.1727002354764877e-06, 2.0476444830016143e-03, -7.2076656304575510e-03, 4.4740745775079797e-02, 2.3241458060599393e-04, -7.8579873987635459e-05, -4.2561035574815007e-05, -2.8462452785592111e-05, 1.4160195343268230e-05, -4.1101131322327410e-07, 1.6379653332917909e-06, -5.0807652816738523e-05, 1.3446891433352409e-05, 1.4755319807536749e-05, 1.4059829466702330e-06, 1.9934354492151082e-06, 1.0558537146569788e-06,
			4.1348555943753260e-04, 2.3266779456548401e-04, 7.3661098495053854e-05, -1.9886299879026950e-07, 1.8001750278289355e-06, 6.0063422795344796e-04, -2.1141760477812340e-03, 1.3045810165352559e-02, 6.3182367867679819e-05, -2.4244789381235254e-05, -1.0794909020771228e-05, -9.6234767552758684e-06, 4.0763351746961359e-06, -1.1545968943850010e-07, 4.6987381692903356e-07, -1.5842950601337628e-05, -9.1122410393765333e-07, 1.1417510911707383e-05, 4.1374663187679601e-07, 5.7899582533969608e-07, 3.0758294307310258e-07,
			-1.2126775191626709e-06, -6.7952838455816354e-07, -1.9886299879026963e-07, 3.9436154261294012e-07, -7.9289674373255113e-08, 6.2155598268791643e-07, 1.0581000170690882e-04, -2.6186124830152160e-04, 1.8431999184410324e-07, 3.3225715477890014e-08, 9.5807852850203126e-09, 8.4922499650951928e-09, 1.2607083156981369e-07, -1.0903625218429216e-07, -3.1974319343240748e-07, 1.9723742565839753e-08, 1.2812424354042357e-08, 1.2990624401187085e-08, 3.5371008681788600e-08, -5.6972227036552548e-10, -1.6036334108816307e-09,
			1.0981982675905102e-05, 6.1727002354764885e-06, 1.8001750278289355e-06, -7.9289674373255100e-08, 9.5967312160807109e-07, 1.6809648139407532e-05, -3.4061863057407028e-04, 1.4484799877225799e-03, -4.9968990504422854e-07, -2.7410143455442553e-07, -8.0563078019119595e-08, -6.7399127801541512e-08, -8.3458562583348134e-07, -1.9727877265063025e-07, -1.5301282778260177e-07, -6.5294308453819270e-08, -5.1524229985190273e-07, -2.4551658949617708e-07, -3.7254068555235444e-07, -1.5123599266828230e-07, -5.6539794107073378e-08,
			3.6244381444420761e-03, 2.0476444830016143e-03, 6.0063422795344796e-04, 6.2155598268791728e-07, 1.6809648139407542e-05, 4.5943939686034385e-01, -5.7781741528075466e-03, 1.7302801459793063e-01, -5.7389546156568917e-02, -1.3826791708570268e-04, -4.3787372905983144e-05, -3.5405252500275196e-05, -2.5520561042855360e-03, 5.9971598216374703e-05, -1.3714708732486504e-04, 3.2266224358466636e-04, 5.8854199530686778e-04, 4.4557273121045957e-04, 5.8345862133972073e-05, 2.1537229039181292e-04, 6.6922853340024545e-05,
			-1.2846799695097090e-02, -7.2076656304575510e-03, -2.1141760477812340e-03, 1.0581000170690883e-04, -3.4061863057407028e-04, -5.7781741528075406e-03, 6.2996292114985453e-01, -1.4853605031923787e+00, -3.5820339261492042e-04, 5.5491347463732324e-04, 2.0980993856627560e-04, 1.7389262526469306e-04, 5.6144342725971514e-04, -1.6836500865278573e-04, -1.3457377185011371e-03, 1.9398504519663317e-04, 1.4092108298581282e-04, 1.0762281260692133e-04, -2.2568333506553030e-05, 1.3674369717268106e-04, -6.9707758404082985e-05,
			7.9309258533828975e-02, 4.4740745775079797e-02, 1.3045810165352558e-02, -2.6186124830152160e-04, 1.4484799877225799e-03, 1.7302801459793063e-01, -1.4853605031923787e+00, 5.7770266532923884e+00, -2.9661656604253916e-04, -2.7608628684897868e-03, -9.9651746124449414e-04, -8.0658000365943603e-04, -2.7221705159363604e-03, -4.0338818917523323e-04, -1.0821400319169748e-03, 4.8637074647761558e-04, -3.2360444303776090e-03, -1.1921007525249969e-03, -3.2302516242205720e-04, -9.0769530083827771e-04, -2.7784099743340183e-04,
			3.5282081897809568e-04, 2.3241458060599393e-04, 6.3182367867679819e-05, 1.8431999184410287e-07, -4.9968990504422832e-07, -5.7389546156568924e-02, -3.5820339261492123e-04, -2.9661656604253981e-04, 1.2771102068091653e-02, -4.2961957180779113e-05, -3.0352460952087129e-05, -1.8709935096173277e-05, 7.7484251137637341e-04, -2.0578248812778742e-05, 4.7268504260828225e-05, -1.1146982321065550e-04, -1.7707165711093413e-04, -9.6887999368231787e-05, -1.7937969520862211e-05, -5.9676569671735012e-05, -2.8807562618371390e-05,
			-1.4955324883539353e-04, -7.8579873987635459e-05, -2.4244789381235254e-05, 3.3225715477889994e-08, -2.7410143455442569e-07, -1.3826791708570265e-04, 5.5491347463732324e-04, -2.7608628684897868e-03, -4.2961957180779113e-05, 2.0559196931598020e-05, 4.9987185061974867e-06, 4.0472416181486788e-06, -6.1971985786070576e-07, -2.8607330930307961e-08, -4.0064377296096829e-08, -9.1547190952548986e-06, 5.5259054306306062e-06, -1.0605724951663743e-06, -2.7215624849192986e-07, -4.6428909906089235e-08, -3.5687122819929573e-08,
			-6.1635658608634171e-05, -4.2561035574815000e-05, -1.0794909020771228e-05, 9.5807852850203126e-09, -8.0563078019119648e-08, -4.3787372905983144e-05, 2.0980993856627565e-04, -9.9651746124449414e-04, -3.0352460952087129e-05, 4.9987185061974875e-06, 9.8812661226018087e-06, 2.0904557484352522e-06, -1.9365876476396804e-07, -1.1333079845099695e-08, -1.4802515797041433e-08, 9.3033971488266076e-06, -5.7645458804029150e-06, -1.5238844464486045e-07, -1.3525741299499249e-07, -1.9474072407801358e-08, -1.3906921841740124e-08,
			-4.9724924136941586e-05, -2.8462452785592111e-05, -9.6234767552758684e-06, 8.4922499650951928e-09, -6.7399127801541552e-08, -3.5405252500275196e-05, 1.7389262526469306e-04, -8.0658000365943603e-04, -1.8709935096173277e-05, 4.0472416181486788e-06, 2.0904557484352527e-06, 7.4376885164623210e-06, -8.4630490781595498e-08, -1.0449984044183468e-08, -6.8681272519754958e-09, 6.6911138768985750e-06, 5.3119416880390490e-07, -2.8473765008222631e-06, -1.1176073221168355e-07, -2.1153129118744056e-08, -1.4073134046643291e-08,
			2.4492670549430145e-05, 1.4160195343268227e-05, 4.0763351746961376e-06, 1.2607083156981369e-07, -8.3458562583348134e-07, -2.5520561042855360e-03, 5.6144342725971503e-04, -2.7221705159363599e-03, 7.7484251137637341e-04, -6.1971985786070597e-07, -1.9365876476396871e-07, -8.4630490781595365e-08, 7.2190613019669815e-05, 1.4911493661600175e-07, -1.7356239212376851e-06, 4.2556898240590129e-06, 7.2066820923711468e-06, 4.3031853110642910e-06, -1.8477883770431320e-06, 1.8849235165499579e-06, 8.1696460440941031e-07,
			-6.9744882791964323e-07, -4.1101131322327494e-07, -1.1545968943850018e-07, -1.0903625218429217e-07, -1.9727877265063033e-07, 5.9971598216374710e-05, -1.6836500865278573e-04, -4.0338818917523328e-04, -2.0578248812778745e-05, -2.8607330930308011e-08, -1.1333079845099699e-08, -1.0449984044183431e-08, 1.4911493661600196e-07, 9.9922703559236827e-05, -4.3190776020879561e-08, -7.3493744860368310e-08, -1.4194498180906261e-07, -1.1122622231811912e-07, -2.4386770207141899e-08, -5.6041242462204051e-08, -1.6470057683921132e-08,
			2.8394202803995592e-06, 1.6379653332917893e-06, 4.6987381692903398e-07, -3.1974319343240753e-07, -1.5301282778260201e-07, -1.3714708732486506e-04, -1.3457377185011371e-03, -1.0821400319169748e-03, 4.7268504260828218e-05, -4.0064377296097074e-08, -1.4802515797041782e-08, -6.8681272519750599e-09, -1.7356239212376830e-06, -4.3190776020879264e-08, 9.9770590238104068e-05, 2.7009795828993659e-07, 4.7294571963721552e-07, 2.6275102814371874e-07, 1.1286178467626743e-07, 1.0853455558495363e-07, 5.2235197956946150e-08,
			-9.2654659777810020e-05, -5.0807652816738509e-05, -1.5842950601337631e-05, 1.9723742565839746e-08, -6.5294308453819125e-08, 3.2266224358466636e-04, 1.9398504519663317e-04, 4.8637074647761558e-04, -1.1146982321065553e-04, -9.1547190952548969e-06, 9.3033971488266076e-06, 6.6911138768985742e-06, 4.2556898240590112e-06, -7.3493744860368310e-08, 2.7009795828993527e-07, 3.7262644282024784e-04, -6.0032226520042868e-06, 7.7199255202378772e-06, -1.3975032104254182e-07, -3.5821216828205091e-07, -1.5920714349419221e-07,
			-8.7310282262389730e-06, 1.3446891433352385e-05, -9.1122410393765471e-07, 1.2812424354042408e-08, -5.1524229985190315e-07, 5.8854199530686767e-04, 1.4092108298581282e-04, -3.2360444303776094e-03, -1.7707165711093413e-04, 5.5259054306306020e-06, -5.7645458804029192e-06, 5.3119416880390479e-07, 7.2066820923711468e-06, -1.4194498180906229e-07, 4.7294571963721764e-07, -6.0032226520042902e-06, 1.3652035270070085e-03, 3.4823988530199153e-05, -1.5975585695476135e-07, -6.0524671832510014e-07, -2.8109621037069869e-07,
			2.7717855186206110e-05, 1.4755319807536749e-05, 1.1417510911707385e-05, 1.2990624401187080e-08, -2.4551658949617729e-07, 4.4557273121045957e-04, 1.0762281260692133e-04, -1.1921007525249969e-03, -9.6887999368231774e-05, -1.0605724951663746e-06, -1.5238844464486071e-07, -2.8473765008222635e-06, 4.3031853110642910e-06, -1.1122622231811907e-07, 2.6275102814371954e-07, 7.7199255202378721e-06, 3.4823988530199153e-05, 5.9815496246261595e-04, -9.6763030307726177e-08, -3.6441889517487360e-07, -1.4917154332156223e-07,
			2.4743796202823705e-06, 1.4059829466702321e-06, 4.1374663187679606e-07, 3.5371008681788607e-08, -3.7254068555235444e-07, 5.8345862133972073e-05, -2.2568333506553030e-05, -3.2302516242205720e-04, -1.7937969520862211e-05, -2.7215624849192997e-07, -1.3525741299499249e-07, -1.1176073221168359e-07, -1.8477883770431318e-06, -2.4386770207141915e-08, 1.1286178467626738e-07, -1.3975032104254182e-07, -1.5975585695476164e-07, -9.6763030307726389e-08, 7.8299770784062419e-07, -3.8925154739842181e-08, -1.4848742280245814e-08,
			3.5495171524189523e-06, 1.9934354492151078e-06, 5.7899582533969630e-07, -5.6972227036553086e-10, -1.5123599266828230e-07, 2.1537229039181292e-04, 1.3674369717268106e-04, -9.0769530083827771e-04, -5.9676569671735006e-05, -4.6428909906089328e-08, -1.9474072407801405e-08, -2.1153129118743946e-08, 1.8849235165499596e-06, -5.6041242462201603e-08, 1.0853455558495295e-07, -3.5821216828205097e-07, -6.0524671832510120e-07, -3.6441889517487339e-07, -3.8925154739842174e-08, 9.9831479997816532e-05, -6.7397934984988334e-08,
			1.8816965739920804e-06, 1.0558537146569792e-06, 3.0758294307310285e-07, -1.6036334108816321e-09, -5.6539794107073384e-08, 6.6922853340024545e-05, -6.9707758404082985e-05, -2.7784099743340183e-04, -2.8807562618371390e-05, -3.5687122819929693e-08, -1.3906921841740157e-08, -1.4073134046643301e-08, 8.1696460440941031e-07, -1.6470057683921083e-08, 5.2235197956945680e-08, -1.5920714349419205e-07, -2.8109621037069922e-07, -1.4917154332156210e-07, -1.4848742280245840e-08, -6.7397934984990664e-08, 9.9969409482670622e-05};

	arm_matrix_instance_f32 PTrue;
	arm_mat_init_f32(&PTrue, 21, 21, PDataTrue);

	bool test1 = false;
	test1 = areMatricesEqual(&PCorrect, &PTrue);
}

// 12492

void test_update_EKF(void) {

	// Iteration Number 25000 from Python Simulation

	arm_matrix_instance_f32 xPrev, PPrev, Rq, Q, H,
							R, aMeas, wMeas, llaMeas,
							magMeas, magI, xPlus,
							Pplus;

	float32_t xPrevData[22*1] = {-1.4657696e-01, -3.4793302e-01, -1.4144270e-01,  9.1512394e-01,
        3.3863335e+01, -8.4334328e+01,  6.6638171e+02,  7.5325325e+01,
       -8.5498688e+01,  7.6204457e+00,  1.9777637e-02, -2.3959318e-02,
       -1.0602783e-02, -1.1829544e+00, -1.3212320e-01, -4.5035532e-01,
        3.2062292e-02,  9.9996477e-02,  2.3580007e-02,  2.8258219e-04,
        3.3405580e-04,  8.4102973e-03};

	float32_t PPrevData[21*21] =
{
  1.758770522e-04f, 2.340855644e-05f, -9.198215412e-05f, 1.936428617e-08f, -2.834561386e-08f, -5.824290565e-04f, 2.844173927e-03f, -6.835414097e-03f, 5.934080764e-05f, -1.389853651e-06f, 1.714433040e-07f, 1.900070856e-06f, -9.228786439e-06f, 1.168013114e-04f, -6.205762020e-06f, 2.264562863e-05f, 4.602628906e-06f, 2.070468781e-06f, 4.909798790e-09f, 1.222922208e-07f, -9.873876650e-08f,
  2.340854917e-05f, 1.560856181e-04f, -1.942452218e-04f, 7.158672766e-09f, -3.118372449e-09f, -6.807690021e-03f, 4.125323147e-03f, -1.225373941e-03f, 3.561239457e-03f, 1.526991014e-06f, -8.954704640e-07f, 3.238562613e-06f, -1.039604686e-04f, -3.614298112e-05f, 1.169108327e-05f, -1.881134449e-05f, -6.349071555e-06f, 1.259196438e-06f, 9.720501026e-08f, -6.430665334e-09f, -5.266873870e-08f,
  -9.198198677e-05f, -1.942451781e-04f, 9.511889657e-04f, -5.447484241e-09f, 2.580865299e-08f, 6.936177146e-03f, -6.980124395e-03f, 7.093988359e-03f, -4.364741035e-03f, -6.832329746e-06f, -2.730422466e-06f, -1.588753548e-05f, -4.731387526e-05f, 2.749036503e-05f, 1.641761082e-05f, 7.443734648e-05f, -5.640798554e-05f, -1.486342899e-05f, 4.404121867e-08f, -4.756466865e-08f, 1.125320921e-07f,
  1.936435190e-08f, 7.158669657e-09f, -5.447576612e-09f, 5.508959955e-10f, 9.458280513e-13f, -1.498054303e-06f, 3.981834652e-06f, -1.198968448e-06f, 2.253093072e-07f, -2.915414021e-10f, -1.488122126e-10f, 2.034726693e-10f, -5.664669178e-09f, -3.817111605e-08f, 1.391117621e-08f, 4.047176816e-09f, 3.374148960e-09f, 6.598492863e-10f, 1.908410756e-11f, -1.571135756e-11f, 2.439885793e-10f,
  -2.834561741e-08f, -3.118373781e-09f, 2.580867076e-08f, 9.458310871e-13f, 2.072879507e-10f, -5.232202938e-08f, -4.901243074e-07f, 2.846905545e-06f, 7.240536348e-08f, 1.240295922e-10f, -8.866787166e-11f, -4.912376617e-10f, 3.948227856e-10f, 2.994946735e-09f, -4.655083818e-10f, -1.961859120e-09f, -2.820302436e-09f, 7.209344927e-11f, -1.003563335e-12f, -5.934181098e-12f, -8.571948706e-12f,
  -5.824291147e-04f, -6.807688624e-03f, 6.936176214e-03f, -1.498054871e-06f, -5.232192635e-08f, 9.132284522e-01f, -2.686436474e-01f, 1.693462767e-02f, -2.696112692e-01f, -6.966126239e-05f, 4.634513971e-05f, -1.145349743e-04f, -1.197241072e-04f, 1.017447561e-03f, 8.945200534e-05f, 8.979155100e-04f, 5.792289739e-04f, 4.145005732e-05f, 5.137325161e-07f, 4.404619176e-07f, 3.942737123e-07f,
  2.844171366e-03f, 4.125321284e-03f, -6.980123930e-03f, 3.981833743e-06f, -4.901247621e-07f, -2.686437368e-01f, 2.079245448e-01f, -1.434509158e-01f, 1.161226928e-01f, 2.874721213e-05f, -1.527148197e-05f, 1.187083326e-04f, -4.423159771e-05f, -1.590152737e-03f, 1.925019023e-04f, -2.538234985e-04f, -6.598266191e-05f, 4.022673238e-05f, 2.126744505e-07f, 1.174014869e-06f, 6.136286629e-06f,
  -6.835413631e-03f, -1.225374639e-03f, 7.093997672e-03f, -1.198966856e-06f, 2.846905318e-06f, 1.693460718e-02f, -1.434505731e-01f, 3.736151457e-01f, -4.846471362e-03f, 2.407827196e-05f, -2.170569132e-05f, -1.288765925e-04f, 6.702024257e-05f, -8.683340857e-04f, 5.922623095e-04f, -5.778216873e-04f, -4.890731070e-04f, -6.234441389e-05f, 2.664309022e-07f, -5.518336820e-06f, 9.286663044e-06f,
  5.934159344e-05f, 3.561239224e-03f, -4.364743363e-03f, 2.253098614e-07f, 7.240527111e-08f, -2.696112692e-01f, 1.161227673e-01f, -4.846516997e-03f, 1.154875532e-01f, 4.312406236e-05f, -2.208870865e-05f, 7.023832586e-05f, 1.314047986e-04f, -1.006801380e-03f, 1.846696432e-05f, -5.476393271e-04f, -2.089536138e-04f, -3.333520681e-06f, -2.161244907e-07f, -7.481426110e-07f, 1.284430596e-06f,
  -1.389855129e-06f, 1.526990104e-06f, -6.832326108e-06f, -2.915400976e-10f, 1.240298003e-10f, -6.966127694e-05f, 2.874716847e-05f, 2.407842476e-05f, 4.312406600e-05f, 2.593062902e-07f, 2.787671782e-08f, 1.232591842e-07f, 4.118175525e-07f, -3.805875508e-07f, -2.189915627e-08f, -3.144075151e-07f, 4.066955910e-07f, -9.585875205e-08f, -3.314779007e-10f, -1.052655185e-09f, 9.777447740e-10f,
  1.714435740e-07f, -8.954702935e-07f, -2.730421556e-06f, -1.488117685e-10f, -8.866774676e-11f, 4.634515426e-05f, -1.527149834e-05f, -2.170564949e-05f, -2.208869955e-05f, 2.787673381e-08f, 2.171017002e-07f, 6.779606565e-08f, 2.736577471e-07f, 5.135078709e-07f, -8.514278704e-08f, -1.191274279e-07f, -8.095702810e-07f, -7.309443362e-08f, 1.172580505e-10f, 2.879821104e-10f, -9.902167974e-10f,
  1.900070515e-06f, 3.238565114e-06f, -1.588754094e-05f, 2.034697133e-10f, -4.912378837e-10f, -1.145350470e-04f, 1.187083981e-04f, -1.288766798e-04f, 7.023834041e-05f, 1.232592837e-07f, 6.779605144e-08f, 5.320613354e-07f, 6.688193110e-08f, 5.774842862e-07f, -1.856705296e-07f, -7.191034683e-07f, 4.046163724e-07f, 3.085647791e-07f, -2.115102121e-10f, 1.429851459e-09f, -2.082001460e-09f,
  -9.228781892e-06f, -1.039604758e-04f, -4.731388617e-05f, -5.664668734e-09f, 3.948255889e-10f, -1.197227684e-04f, -4.423223072e-05f, 6.701976963e-05f, 1.314042311e-04f, 4.118173820e-07f, 2.736586850e-07f, 6.688224374e-08f, 9.747481090e-04f, -6.094957826e-06f, -9.911194502e-05f, -1.080898755e-06f, -1.475044087e-06f, 7.877239483e-08f, 3.039491503e-08f, 1.227640434e-09f, 1.041969881e-06f,
  1.168013114e-04f, -3.614298475e-05f, 2.749047417e-05f, -3.817109118e-08f, 2.994944959e-09f, 1.017449540e-03f, -1.590153552e-03f, -8.683334454e-04f, -1.006802078e-03f, -3.805865845e-07f, 5.135070751e-07f, 5.774819556e-07f, -6.094935543e-06f, 9.439505520e-04f, 1.328031431e-05f, 3.030652124e-06f, 7.873991308e-06f, -5.239154532e-08f, 1.921698001e-08f, -1.751997658e-08f, 3.135014595e-07f,
  -6.205763384e-06f, 1.169107600e-05f, 1.641763811e-05f, 1.391117532e-08f, -4.655079655e-10f, 8.945210720e-05f, 1.925017277e-04f, 5.922617274e-04f, 1.846686791e-05f, -2.190004622e-08f, -8.514312100e-08f, -1.856706149e-07f, -9.911192319e-05f, 1.328029794e-05f, 1.045779718e-04f, -6.369329526e-07f, -1.630667043e-06f, -1.676141892e-08f, 1.052987244e-07f, 4.468437620e-09f, 8.493653695e-06f,
  2.264561772e-05f, -1.881134995e-05f, 7.443736104e-05f, 4.047172819e-09f, -1.961860008e-09f, 8.979152190e-04f, -2.538241097e-04f, -5.778216873e-04f, -5.476392689e-04f, -3.144070320e-07f, -1.191278685e-07f, -7.191030136e-07f, -1.080902734e-06f, 3.030669859e-06f, -6.369296557e-07f, 1.359689049e-04f, -8.128512491e-06f, 4.508465565e-07f, -8.518803440e-10f, 8.213497438e-09f, -1.438043284e-08f,
  4.602606168e-06f, -6.349074738e-06f, -5.640798918e-05f, 3.374151847e-09f, -2.820302214e-09f, 5.792291486e-04f, -6.598326581e-05f, -4.890722339e-04f, -2.089535556e-04f, 4.066960457e-07f, -8.095699400e-07f, 4.046165429e-07f, -1.475059321e-06f, 7.874007679e-06f, -1.630667953e-06f, -8.128515219e-06f, 1.305672777e-04f, -7.136636526e-08f, -1.039352604e-09f, 1.141349504e-08f, -3.631932444e-08f,
  2.070470146e-06f, 1.259195983e-06f, -1.486342626e-05f, 6.598486757e-10f, 7.209348396e-11f, 4.145006824e-05f, 4.022663779e-05f, -6.234452303e-05f, -3.333519544e-06f, -9.585881600e-08f, -7.309441941e-08f, 3.085647506e-07f, 7.877309827e-08f, -5.239420631e-08f, -1.676111872e-08f, 4.508472671e-07f, -7.136606683e-08f, 1.479234779e-04f, -3.099821441e-11f, -9.968383896e-10f, -1.042523068e-09f,
  4.909769036e-09f, 9.720496763e-08f, 4.404123999e-08f, 1.908415613e-11f, -1.003564419e-12f, 5.137338803e-07f, 2.126749621e-07f, 2.664325223e-07f, -2.161251018e-07f, -3.314773456e-10f, 1.172628522e-10f, -2.115107395e-10f, 3.039491503e-08f, 1.921702264e-08f, 1.052987670e-07f, -8.518790673e-10f, -1.039357156e-09f, -3.100016424e-11f, 9.999802160e-07f, 3.195627573e-12f, -1.283176565e-09f,
  1.222922492e-07f, -6.430654675e-09f, -4.756472904e-08f, -1.571134542e-11f, -5.934185435e-12f, 4.404624292e-07f, 1.174016347e-06f, -5.518339094e-06f, -7.481429520e-07f, -1.052655074e-09f, 2.879824712e-10f, 1.429851904e-09f, 1.227651536e-09f, -1.752002809e-08f, 4.468439396e-09f, 8.213499214e-09f, 1.141349237e-08f, -9.968379455e-10f, 3.195614780e-12f, 9.999691883e-07f, 7.364472920e-11f,
  -9.873897966e-08f, -5.266880621e-08f, 1.125319002e-07f, 2.439881353e-10f, -8.571914012e-12f, 3.942647879e-07f, 6.136287993e-06f, 9.286658496e-06f, 1.284432187e-06f, 9.777625376e-10f, -9.902125786e-10f, -2.081999906e-09f, 1.041969995e-06f, 3.135012321e-07f, 8.493652786e-06f, -1.438036623e-08f, -3.631924272e-08f, -1.042527731e-09f, -1.283176343e-09f, 7.364460430e-11f, 9.141562600e-07f
};


	float32_t QData[12*12] =
{
  9.999998838e-06f, 9.999999406e-08f, 9.999999406e-08f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f,
  9.999999406e-08f, 9.999998838e-06f, 9.999999406e-08f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f,
  9.999999406e-08f, 9.999999406e-08f, 9.999998838e-06f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f,
  0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 1.057699262e-09f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f,
  0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 1.057699262e-09f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f,
  0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 1.057699262e-09f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f,
  0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 4.999999655e-04f, 4.999999419e-06f, 4.999999419e-06f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f,
  0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 4.999999419e-06f, 4.999999655e-04f, 4.999999419e-06f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f,
  0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 4.999999419e-06f, 4.999999419e-06f, 4.999999655e-04f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f,
  0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 7.698887430e-07f, 0.000000000e+00f, 0.000000000e+00f,
  0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 7.698887430e-07f, 0.000000000e+00f,
  0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 7.698887430e-07f
};


	float32_t HData[3*21] = {
  0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 1.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f,
  0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 1.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f,
  0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 1.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f, 0.000000000e+00f
};

	float32_t RData[3*3] = {
  1.199999988e-09f, 0.000000000e+00f, 0.000000000e+00f,
  0.000000000e+00f, 4.000000053e-10f, 0.000000000e+00f,
  0.000000000e+00f, 0.000000000e+00f, 1.000000000e+02f
};

	float32_t RqData[3*3] = {
  2.499999937e-05f, 0.000000000e+00f, 0.000000000e+00f,
  0.000000000e+00f, 2.499999937e-05f, 0.000000000e+00f,
  0.000000000e+00f, 0.000000000e+00f, 2.499999937e-05f
};

	float32_t Rb = 1e+4;

	float32_t aMeasData[3*1] = {-2.20558429, -0.18305093, -9.41515923};

	float32_t wMeasData[3*1] = {-7.78853155e-06, -1.38971836e-05, -4.88692167e-05};

	float32_t llaMeasData[3*1] = {33.86387634, -84.33440399, 617.36602783};

	float32_t magMeasData[3*1] = {0.3532234728336334,  0.2187790721654892, -0.9095926284790039};

	float32_t magIData[3*1] = {0.4891000092029572, 0.1040000021457672, 0.8659999966621399};

	float32_t pressMeas = 93634.0f;

	float32_t dt = 1 / 200.0f;

	uint32_t vdStart = UINT32_MAX;
	uint32_t mainAltStart = UINT32_MAX;
	uint32_t drougeAltStart = UINT32_MAX;

	fc_message fcMess;
	fcMess.body.valid = 1;

	arm_mat_init_f32(&xPrev, 22, 1, xPrevData);
	arm_mat_init_f32(&PPrev, 21, 21, PPrevData);
	arm_mat_init_f32(&Q, 12, 12, QData);
	arm_mat_init_f32(&H, 3, 21, HData);
	arm_mat_init_f32(&R, 3, 3, RData);
	arm_mat_init_f32(&Rq, 3, 3, RqData);
	arm_mat_init_f32(&aMeas, 3, 1, aMeasData);
	arm_mat_init_f32(&wMeas, 3, 1, wMeasData);
	arm_mat_init_f32(&llaMeas, 3, 1, llaMeasData);
	arm_mat_init_f32(&magMeas, 3, 1, magMeasData);
	arm_mat_init_f32(&magI, 3, 1, magIData);

	float32_t xPlusBuff[22*1];
	float32_t PPlusBuff[21*21];

	bool fallbackDR = false;

//    update_EKF(&xPrev, &PPrev, &Q, &H,
//    		   &R, &Rq, Rb, &aMeas,
//			   &wMeas, &llaMeas, &magMeas,
//			   pressMeas, &magI, we, dt, &xPlus,
//			   &Pplus, xPlusBuff, PPlusBuff, &fcMess, &fallbackDR);

	float32_t xPlusDataTest[22*1] = {-1.46576956e-01, -3.47933024e-01, -1.41442701e-01,  9.15123940e-01,
        3.38633347e+01, -8.43343277e+01,  6.66381714e+02,  7.53253250e+01,
       -8.54986877e+01,  7.62044573e+00,  1.97776370e-02, -2.39593182e-02,
       -1.06027834e-02, -1.18295443e+00, -1.32123202e-01, -4.50355321e-01,
        3.20622921e-02,  9.99964774e-02,  2.35800073e-02,  2.82582187e-04,
        3.34055803e-04,  8.41029733e-03};

	float32_t PPlusDataTest[21*21] = {
  1.743820467e-04f, 2.311080061e-05f, -9.065445192e-05f, 1.337366662e-08f, -1.891317680e-08f, -5.577258999e-04f, 2.777093789e-03f, -6.694072392e-03f, 5.680938921e-05f, -1.382018809e-06f, 1.695380405e-07f, 1.873999622e-06f, -9.150588085e-06f, 1.173767960e-04f, -6.381432286e-06f, 2.252341255e-05f, 4.440642897e-06f, 2.069046786e-06f, 4.658316399e-09f, 1.221882684e-07f, -1.018589586e-07f,
  2.311079334e-05f, 1.551166642e-04f, -1.931217121e-04f, 4.882723115e-09f, -2.104029173e-09f, -6.685272325e-03f, 4.072266165e-03f, -1.204232918e-03f, 3.525852924e-03f, 1.519063062e-06f, -8.891112770e-07f, 3.219194468e-06f, -1.039491472e-04f, -3.583279977e-05f, 1.164468813e-05f, -1.871202949e-05f, -6.298851986e-06f, 1.264648176e-06f, 9.719326499e-08f, -6.338555014e-09f, -5.364466915e-08f,
  -9.065429185e-05f, -1.931216539e-04f, 9.491406381e-04f, -3.816544858e-09f, 1.724929533e-08f, 6.817334332e-03f, -6.914454047e-03f, 6.970832590e-03f, -4.331905395e-03f, -6.828736787e-06f, -2.734414238e-06f, -1.585018617e-05f, -4.734224785e-05f, 2.709405635e-05f, 1.646847340e-05f, 7.440464833e-05f, -5.636396600e-05f, -1.487949976e-05f, 4.407796084e-08f, -4.740684290e-08f, 1.135930106e-07f,
  1.337371103e-08f, 4.882721338e-09f, -3.816608363e-09f, 3.777087532e-10f, 3.867123906e-13f, -1.015013709e-06f, 2.729810603e-06f, -8.290442679e-07f, 1.518517792e-07f, -2.005766525e-10f, -1.013157058e-10f, 1.410003225e-10f, -3.886123867e-09f, -2.618551953e-08f, 9.540900514e-09f, 2.787579723e-09f, 2.325835968e-09f, 4.542626830e-10f, 1.309631483e-11f, -1.071384908e-11f, 1.673974431e-10f,
  -1.891317680e-08f, -2.104029617e-09f, 1.724930598e-08f, 3.867138001e-13f, 1.366667335e-10f, -3.292756290e-08f, -3.296536590e-07f, 1.888737643e-06f, 4.730063097e-08f, 8.255193751e-11f, -5.906306694e-11f, -3.281330485e-10f, 2.640475061e-10f, 1.953855744e-09f, -2.898142559e-10f, -1.312755016e-09f, -1.874759015e-09f, 4.506085782e-11f, -6.569812511e-13f, -4.097583817e-12f, -5.391316733e-12f,
  -5.577257834e-04f, -6.685270462e-03f, 6.817333866e-03f, -1.015014277e-06f, -3.292749895e-08f, 8.965032697e-01f, -2.607742846e-01f, 1.585984416e-02f, -2.648541927e-01f, -6.872824451e-05f, 4.540173541e-05f, -1.124753617e-04f, -1.227574539e-04f, 9.701278177e-04f, 9.932837565e-05f, 8.860668750e-04f, 5.713992869e-04f, 4.121085294e-05f, 5.206867968e-07f, 4.216481386e-07f, 5.861168120e-07f,
  2.777090995e-03f, 4.072264303e-03f, -6.914453115e-03f, 2.729809694e-06f, -3.296539717e-07f, -2.607743740e-01f, 1.972488463e-01f, -1.385351121e-01f, 1.143276542e-01f, 2.914539618e-05f, -1.475532190e-05f, 1.173165219e-04f, -3.157369065e-05f, -1.495732111e-03f, 1.608974562e-04f, -2.597908024e-04f, -7.266948523e-05f, 3.911820022e-05f, 1.709362891e-07f, 1.209121024e-06f, 5.576671811e-06f,
  -6.694071926e-03f, -1.204233617e-03f, 6.970841438e-03f, -8.290432447e-07f, 1.888737529e-06f, 1.585982740e-02f, -1.385347843e-01f, 3.598361313e-01f, -4.930881783e-03f, 2.334253077e-05f, -2.142336962e-05f, -1.264697639e-04f, 6.143404607e-05f, -9.095230489e-04f, 6.043165922e-04f, -5.666090292e-04f, -4.739585856e-04f, -6.237255002e-05f, 2.841265143e-07f, -5.505419722e-06f, 9.500363376e-06f,
  5.681015682e-05f, 3.525853157e-03f, -4.331908189e-03f, 1.518521628e-07f, 4.730057412e-08f, -2.648542225e-01f, 1.143277213e-01f, -4.930926487e-03f, 1.141039953e-01f, 4.279562563e-05f, -2.181898708e-05f, 6.968590606e-05f, 1.315770933e-04f, -9.976864094e-04f, 1.730448093e-05f, -5.433955230e-04f, -2.059257240e-04f, -3.164077725e-06f, -2.158979413e-07f, -7.440083891e-07f, 1.258701104e-06f,
  -1.382020287e-06f, 1.519062153e-06f, -6.828733603e-06f, -2.005757782e-10f, 8.255207629e-11f, -6.872825907e-05f, 2.914534889e-05f, 2.334268174e-05f, 4.279562927e-05f, 2.591311272e-07f, 2.794079990e-08f, 1.232224776e-07f, 4.106053950e-07f, -3.860357651e-07f, -1.934736815e-08f, -3.119542384e-07f, 4.087275443e-07f, -9.569938442e-08f, -3.273007143e-10f, -1.053311882e-09f, 1.020827201e-09f,
  1.695383247e-07f, -8.891110497e-07f, -2.734413556e-06f, -1.013154005e-10f, -5.906298367e-11f, 4.540175360e-05f, -1.475533827e-05f, -2.142332778e-05f, -2.181897798e-05f, 2.794081588e-08f, 2.170345113e-07f, 6.785535334e-08f, 2.733588929e-07f, 5.096888458e-07f, -8.412116870e-08f, -1.199842643e-07f, -8.102879292e-07f, -7.307001226e-08f, 1.182077491e-10f, 2.852943159e-10f, -9.711742521e-10f,
  1.873999395e-06f, 3.219197197e-06f, -1.585019345e-05f, 1.409983102e-10f, -3.281332150e-10f, -1.124754417e-04f, 1.173165947e-04f, -1.264698512e-04f, 6.968592788e-05f, 1.232225770e-07f, 6.785533913e-08f, 5.313479505e-07f, 6.757551319e-08f, 5.869365509e-07f, -1.874396958e-07f, -7.189748885e-07f, 4.033267373e-07f, 3.086464346e-07f, -2.132860416e-10f, 1.427682417e-09f, -2.116358866e-09f,
  -9.150582628e-06f, -1.039491617e-04f, -4.734226241e-05f, -3.886123867e-09f, 2.640493657e-10f, -1.227561297e-04f, -3.157431638e-05f, 6.143355131e-05f, 1.315765257e-04f, 4.106052245e-07f, 2.733598308e-07f, 6.757584714e-08f, 9.747291915e-04f, -6.217977898e-06f, -9.906633932e-05f, -1.064071284e-06f, -1.460674071e-06f, 8.098222537e-08f, 3.045879637e-08f, 1.182250187e-09f, 1.042767281e-06f,
  1.173767887e-04f, -3.583280341e-05f, 2.709416367e-05f, -2.618550177e-08f, 1.953854634e-09f, 9.701297968e-04f, -1.495732926e-03f, -9.095224086e-04f, -9.976871079e-04f, -3.860348272e-07f, 5.096880500e-07f, 5.869342772e-07f, -6.217955615e-06f, 9.430797072e-04f, 1.358426562e-05f, 3.109151294e-06f, 7.949023711e-06f, -3.917758207e-08f, 1.962742679e-08f, -1.784074044e-08f, 3.188627318e-07f,
  -6.381434559e-06f, 1.164468085e-05f, 1.646850069e-05f, 9.540900514e-09f, -2.898139784e-10f, 9.932847024e-05f, 1.608972671e-04f, 6.043160101e-04f, 1.730438635e-05f, -1.934824745e-08f, -8.412150265e-08f, -1.874397952e-07f, -9.906631749e-05f, 1.358424925e-05f, 1.044667879e-04f, -6.724403079e-07f, -1.660806220e-06f, -2.206332361e-08f, 1.051450695e-07f, 4.587443758e-09f, 8.491704648e-06f,
  2.252340164e-05f, -1.871203494e-05f, 7.440466288e-05f, 2.787577058e-09f, -1.312755460e-09f, 8.860666421e-04f, -2.597914136e-04f, -5.666090292e-04f, -5.433954648e-04f, -3.119537553e-07f, -1.199847048e-07f, -7.189744338e-07f, -1.064075263e-06f, 3.109169029e-06f, -6.724369541e-07f, 1.359392190e-04f, -8.157044249e-06f, 4.487234548e-07f, -9.095575404e-10f, 8.220723657e-09f, -1.498193214e-08f,
  4.440620160e-06f, -6.298854714e-06f, -5.636397327e-05f, 2.325837967e-09f, -1.874758571e-09f, 5.713995197e-04f, -7.267008914e-05f, -4.739577125e-04f, -2.059256658e-04f, 4.087279990e-07f, -8.102875881e-07f, 4.033269079e-07f, -1.460689305e-06f, 7.949040082e-06f, -1.660807129e-06f, -8.157046977e-06f, 1.305439364e-04f, -7.285009218e-08f, -1.087494206e-09f, 1.140902661e-08f, -3.683547689e-08f,
  2.069048151e-06f, 1.264647835e-06f, -1.487949612e-05f, 4.542622389e-10f, 4.506087864e-11f, 4.121085658e-05f, 3.911810563e-05f, -6.237265916e-05f, -3.164076361e-06f, -9.569944837e-08f, -7.306999805e-08f, 3.086464062e-07f, 8.098292881e-08f, -3.918024660e-08f, -2.206301808e-08f, 4.487241654e-07f, -7.284979375e-08f, 1.479270431e-04f, -3.858339342e-11f, -9.906131471e-10f, -1.134433991e-09f,
  4.658285313e-09f, 9.719322236e-08f, 4.407798215e-08f, 1.309634866e-11f, -6.569818474e-13f, 5.206881610e-07f, 1.709366870e-07f, 2.841281912e-07f, -2.158985382e-07f, -3.273001314e-10f, 1.182125647e-10f, -2.132866106e-10f, 3.045879993e-08f, 1.962747120e-08f, 1.051451122e-07f, -9.095564302e-10f, -1.087498758e-09f, -3.858537101e-11f, 9.999799886e-07f, 3.351172421e-12f, -1.285860196e-09f,
  1.221882968e-07f, -6.338544800e-09f, -4.740690684e-08f, -1.071384041e-11f, -4.097587287e-12f, 4.216485934e-07f, 1.209122502e-06f, -5.505421996e-06f, -7.440087302e-07f, -1.053311771e-09f, 2.852946768e-10f, 1.427682861e-09f, 1.182261289e-09f, -1.784079195e-08f, 4.587445535e-09f, 8.220725434e-09f, 1.140902395e-08f, -9.906127030e-10f, 3.351158977e-12f, 9.999689610e-07f, 7.573929678e-11f,
  -1.018591718e-07f, -5.364473310e-08f, 1.135928258e-07f, 1.673971239e-10f, -5.391294182e-12f, 5.861077170e-07f, 5.576674084e-06f, 9.500358829e-06f, 1.258702810e-06f, 1.020845075e-09f, -9.711700333e-10f, -2.116357534e-09f, 1.042767394e-06f, 3.188625044e-07f, 8.491703738e-06f, -1.498186464e-08f, -3.683539518e-08f, -1.134438654e-09f, -1.285859863e-09f, 7.573917188e-11f, 9.141220971e-07f
};


	arm_matrix_instance_f32 xPlusTrue;
	arm_matrix_instance_f32 PPlusTrue;

	arm_mat_init_f32(&xPlusTrue, 22, 1, xPlusDataTest);
	arm_mat_init_f32(&PPlusTrue, 21, 21, PPlusDataTest);

	bool test1 = false;
	bool test2 = false;

	printMatrix(&xPlus);
	printMatrix(&Pplus);

	test1 = areMatricesEqual(&xPlusTrue, &xPlus);
	test2 = areMatricesEqual(&PPlusTrue, &Pplus);
	bool test = test1 && test2;
}

void test_p2alt(void) {

	// Correct Value shown to the side
	float32_t test0 = pressure_altimeter_uncorrected(100950.51);  // -28.61435
	float32_t test1 = pressure_altimeter_uncorrected(89529.11828); //
	float32_t test2 = pressure_altimeter_uncorrected(79253.63168);
	float32_t test3 = pressure_altimeter_uncorrected(69993.38861);
	float32_t test4 = pressure_altimeter_uncorrected(61645.73585);
	float32_t test5 = pressure_altimeter_uncorrected(54128.19964);
	float32_t test6 = pressure_altimeter_uncorrected(47372.46839);
	float32_t test7 = pressure_altimeter_uncorrected(41319.83769);
	float32_t test8 = pressure_altimeter_uncorrected(35917.85531);
	float32_t test9 = pressure_altimeter_uncorrected(31117.95599);
	float32_t test10 = pressure_altimeter_uncorrected(26873.90612);
	float32_t test11 = pressure_altimeter_uncorrected(23140.89511);
	float32_t test12 = pressure_altimeter_uncorrected(19875.11719);
	float32_t test13 = pressure_altimeter_uncorrected(17033.72513);
	float32_t test14 = pressure_altimeter_uncorrected(14575.03714);
	float32_t test15 = pressure_altimeter_uncorrected(12458.8766);
	float32_t test16 = pressure_altimeter_uncorrected(10646.83425);
	float32_t test17 = pressure_altimeter_uncorrected(9101.552976);
	float32_t test18 = pressure_altimeter_uncorrected(7786.935401);
	float32_t test19 = pressure_altimeter_uncorrected(6669.498257);
	float32_t test20 = pressure_altimeter_uncorrected(5719.153996);
	float32_t test21 = pressure_altimeter_uncorrected(4909.537135);
	float32_t test22 = pressure_altimeter_uncorrected(4218.385292);
	float32_t test23 = pressure_altimeter_uncorrected(3627.365687);
	float32_t test24 = pressure_altimeter_uncorrected(3121.319115);
	float32_t test25 = pressure_altimeter_uncorrected(2687.626281);
	float32_t test26 = pressure_altimeter_uncorrected(2315.716266);
	float32_t test27 = pressure_altimeter_uncorrected(1996.683093);
	float32_t test28 = pressure_altimeter_uncorrected(1722.984816);
	float32_t test29 = pressure_altimeter_uncorrected(1488.205904);
	float32_t test30 = pressure_altimeter_uncorrected(1286.868305);
	float32_t test31 = pressure_altimeter_uncorrected(1114.280108);
	float32_t test32 = pressure_altimeter_uncorrected(966.4133053);
	float32_t test33 = pressure_altimeter_uncorrected(839.7967927);
	float32_t test34 = pressure_altimeter_uncorrected(731.2914148);
	float32_t test35 = pressure_altimeter_uncorrected(638.0972595);
	float32_t test36 = pressure_altimeter_uncorrected(557.8658981);
	float32_t test37 = pressure_altimeter_uncorrected(488.6305232);
	float32_t test38 = pressure_altimeter_uncorrected(428.7414456);
	float32_t test39 = pressure_altimeter_uncorrected(376.812803);
	float32_t test40 = pressure_altimeter_uncorrected(331.678501);
	float32_t test41 = pressure_altimeter_uncorrected(292.3557583);
	float32_t test42 = pressure_altimeter_uncorrected(258.014917);
	float32_t test43 = pressure_altimeter_uncorrected(227.9544129);
	float32_t test44 = pressure_altimeter_uncorrected(201.580003);
	float32_t test45 = pressure_altimeter_uncorrected(178.3875023);
	float32_t test46 = pressure_altimeter_uncorrected(157.9485107);

}

// ============================================================================
// Test Runner
// ============================================================================

void run_all_tests(void)
{
    printf("\n");
    printf("========================================\n");
    printf("EKF Test Suite\n");
    printf("========================================\n");
    printf("\n");

    // Reset statistics
    g_test_stats.total_tests = 0;
    g_test_stats.passed_tests = 0;
    g_test_stats.failed_tests = 0;

    printf("Testing compute_hats.c functions:\n");
    printf("-----------------------------------\n");
    test_compute_wn_basic();
    test_compute_wn_zero_velocity();
    test_compute_wn_edge_cases();
    test_compute_what_basic();
    test_compute_what_zero_inputs();
    test_compute_ahat_basic();
    test_compute_ahat_zero_inputs();

    printf("\n");
    printf("Testing compute_F.c functions:\n");
    printf("-----------------------------------\n");
    test_compute_F_dimensions();
    test_compute_F_finite_values();
    test_compute_F_zero_velocity();
    test_compute_G_dimensions();
    test_compute_G_finite_values();
    test_compute_G_structure();

    printf("\n");
    printf("========================================\n");
    printf("Test Summary\n");
    printf("========================================\n");
    printf("Total tests:  %lu\n", (unsigned long)g_test_stats.total_tests);
    printf("Passed:       %lu\n", (unsigned long)g_test_stats.passed_tests);
    printf("========================================\n");
    printf("\n");
}

test_stats_t get_test_stats(void)
{
    return g_test_stats;
}
