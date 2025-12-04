#include "tests.h"
#include "float.h"
#include "ekf.h"
#include "ekf_utils.h"
#include "../CControl/ccontrol.h"

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

bool test_what(void) {

	arm_matrix_instance_f32 xPrev, wMeas, wHatTrue, wHatTest;

	arm_matrix_instance_f32 q, gBias, aBias, g_sf, a_sf;
	float32_t quatBuff[4], gBiasBuff[3], aBiasBuff[3], gSFBias[3], aSFBias[3];

	float32_t xPrevData[22*1] = {0.7071068f, 0, 0.7071068f, 0, 35.34788f, -117.8068f,
								 625.0f, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
								 0, 0, 0, 0, 0};

	float32_t wMeasData[3*1] = {0.00032869797142236f,
								0.000155072323595796f,
								-0.00202279818123875f};

	float32_t wHatDataTrue[3*1] = {0.0002865102f,
		    				   0.0001550723f,
							  -0.002082277f};

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

	return areMatricesEqual(&wHatTest, &wHatTrue);
}

bool test_ahat(void) {

	arm_matrix_instance_f32 xPrev, aMeas, aMeasTrue, aHatTest;

	arm_matrix_instance_f32 q, gBias, aBias, g_sf, a_sf;
	float32_t quatBuff[4], gBiasBuff[3], aBiasBuff[3], gSFBias[3], aSFBias[3];

	float32_t xPrevData[22*1] = {0.7071068f, 0, 0.7071068f, 0, 35.34788f, -117.8068f,
								 625.0f, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
								 0, 0, 0, 0, 0};

	float32_t aMeasData[3*1] = {19.7918302847682, 0.0131653560858343, -0.000292017931905962};

	float32_t wHatDataTrue[3*1] = {0.0002865102f,
		    				   0.0001550723f,
							  -0.002082277f};

	arm_mat_init_f32(&xPrev, 22, 1, xPrevData);
	arm_mat_init_f32(&aMeas, 3, 1, aMeasData);
	// arm_mat_init_f32(&wHatTrue, 3, 1, wHatDataTrue);

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

	printMatrix(&aHatTest);
	return true;
}

bool test_qdot() {

	arm_matrix_instance_f32 xPrev, wHat, qDot;

	arm_matrix_instance_f32 q, gBias, aBias, g_sf, a_sf;
	float32_t quatBuff[4], gBiasBuff[3], aBiasBuff[3], gSFBias[3], aSFBias[3], qDotData[4];

	float32_t xPrevData[22*1] = {0.7071068f, 0, 0.7071068f, 0, 35.34788f, -117.8068f,
								 625.0f, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
								 0, 0, 0, 0, 0};

	float32_t wHatDataTrue[3*1] = {0.0002865102f,
		    				   0.0001550723f,
							  -0.002082277f};

	float32_t qDotTrue[4*1] = {
			   -5.482635e-05f,
			   -0.0006348993f,
			    5.482635e-05f,
			   -0.0008374926f,
	};

	arm_mat_init_f32(&xPrev, 22, 1, xPrevData);
	arm_mat_init_f32(&wHat, 3, 1, wHatDataTrue);
	// arm_mat_init_f32(&wHatTrue, 3, 1, wHatDataTrue);

	printMatrix(&xPrev);
	printMatrix(&wHat);

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

	compute_qdot(&q, &wHat, &qDot, qDotData);
	return true;
}

bool test_lla_dot(void) {

	float32_t xPrevData[22*1] = {0.7071068f, 0, 0.7071068f, 0, 35.34788f, -117.8068f,
								 625.0f, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
								 0, 0, 0, 0, 0};

	arm_matrix_instance_f32 llaDot, xPrev;
	float32_t llaDotData[3];\

	float32_t llaDotTrue[3] = {
		0,
		0,
		0
	};

	arm_mat_init_f32(&xPrev, 22, 1, xPrevData);

	float32_t phi = xPrev.pData[4];
	float32_t h = xPrev.pData[6];
	float32_t vn = xPrev.pData[7];
	float32_t ve = xPrev.pData[8];
	float32_t vd = xPrev.pData[9];

	compute_lla_dot(phi, h, vn, ve, vd, &llaDot, llaDotData);
	return true;

}

bool test_compute_vdot(void) {

	float32_t xPrevData[22*1] = {0.7071068f, 0, 0.7071068f, 0, 35.34788f, -117.8068f,
								 625.0f, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
								 0, 0, 0, 0, 0};

	float32_t ahatNData[3*1] = {-0.0002920179f, 0.01316535f, -19.79183};

	arm_matrix_instance_f32 xPrev, vDot;
	float32_t vDotData[3];

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

	compute_vdot(phi, h, vn, ve, vd, ahatNData, we, &vDot, vDotData);
	return true;
}

// i = 201
bool test_compute_dwdp(void) {

	arm_matrix_instance_f32 xPrev;

	float32_t xPrevData[22*1] = {0.707598f,
							   -0.0004724356f,
								   0.7066147f,
							   -0.0007329581f,
									35.34789f,
								   -117.8068f,
									671.9332f,
								 -0.02112561f,
								  0.01154266f,
								   -53.25378f,
								1.468478e-05f,
								0.0001556851f,
							   -8.201851e-06f,
							   -3.596146e-05f,
							   -3.867248e-09f,
							   -2.264934e-10f,
							   -7.372506e-12f,
								-5.61519e-13f,
								 1.10269e-12f,
							   -0.0009079037f,
							   -6.181286e-12f,
								5.466609e-12f};

	float32_t aMeasData[3] = {40.7497486442061f,
	        				  0.0398587448019052f,
							  -0.031245514204297f};

	float32_t wMeasData[3] = {-0.00141990364120018f,
							  -0.00497535811999743f,
							   0.00100243692685004f};

	arm_mat_init_f32(&xPrev, 22, 1, xPrevData);

	float32_t phi = xPrev.pData[4];
	float32_t h = xPrev.pData[6];
	float32_t vn = xPrev.pData[7];
	float32_t ve = xPrev.pData[8];

	arm_matrix_instance_f32 dwdp;
	float32_t dwdpBuff[9];

	/*  -4.218775e-05     0   -2.830432e-16
   	   	-3.155908e-11     0   -5.226865e-16
   	   	-5.948112e-05     0    2.007612e-16
	 */

	compute_dwdp(phi, h, ve, vn, we, &dwdp, dwdpBuff);
	return true;
}

bool test_compute_dwdv(void) {

	arm_matrix_instance_f32 xPrev;

	float32_t xPrevData[22*1] = {0.707598f,
							   -0.0004724356f,
								   0.7066147f,
							   -0.0007329581f,
									35.34789f,
								   -117.8068f,
									671.9332f,
								 -0.02112561f,
								  0.01154266f,
								   -53.25378f,
								1.468478e-05f,
								0.0001556851f,
							   -8.201851e-06f,
							   -3.596146e-05f,
							   -3.867248e-09f,
							   -2.264934e-10f,
							   -7.372506e-12f,
								-5.61519e-13f,
								 1.10269e-12f,
							   -0.0009079037f,
							   -6.181286e-12f,
								5.466609e-12f};

	arm_mat_init_f32(&xPrev, 22, 1, xPrevData);

	float32_t phi = xPrev.pData[4];
	float32_t h = xPrev.pData[6];

	arm_matrix_instance_f32 dwdv;
	float32_t dwdvBuff[9];

// Expected Result (i == 201)
//
//		0				 1.565934e-07               0
//		-1.572954e-07               0               0
//		0   			-1.110709e-07               0

	compute_dwdv(phi, h, &dwdv, dwdvBuff);
	return true;
}

bool test_compute_dpdot_dp(void) {

	arm_matrix_instance_f32 xPrev;

	float32_t xPrevData[22*1] = {0.707598f,
							   -0.0004724356f,
								   0.7066147f,
							   -0.0007329581f,
									35.34789f,
								   -117.8068f,
									671.9332f,
								 -0.02112561f,
								  0.01154266f,
								   -53.25378f,
								1.468478e-05f,
								0.0001556851f,
							   -8.201851e-06f,
							   -3.596146e-05f,
							   -3.867248e-09f,
							   -2.264934e-10f,
							   -7.372506e-12f,
								-5.61519e-13f,
								 1.10269e-12f,
							   -0.0009079037f,
							   -6.181286e-12f,
								5.466609e-12f};

	arm_mat_init_f32(&xPrev, 22, 1, xPrevData);

	float32_t phi = xPrev.pData[4];
	float32_t h = xPrev.pData[6];
	float32_t vn = xPrev.pData[7];
	float32_t ve = xPrev.pData[8];

	arm_matrix_instance_f32 dpdot_dp;
	float32_t dpDotBuff[9];

// Expected Result (i == 201)
//
//    3.155908e-11               0    2.994773e-14
//    1.564795e-09               0   -1.988242e-14
//               0               0               0

	compute_dpdot_dp(phi, h, vn, ve, &dpdot_dp, dpDotBuff);
	return true;
}

bool test_compute_dpdot_dv(void) {

	arm_matrix_instance_f32 xPrev;

	float32_t xPrevData[22*1] = {0.707598f,
							   -0.0004724356f,
								   0.7066147f,
							   -0.0007329581f,
									35.34789f,
								   -117.8068f,
									671.9332f,
								 -0.02112561f,
								  0.01154266f,
								   -53.25378f,
								1.468478e-05f,
								0.0001556851f,
							   -8.201851e-06f,
							   -3.596146e-05f,
							   -3.867248e-09f,
							   -2.264934e-10f,
							   -7.372506e-12f,
								-5.61519e-13f,
								 1.10269e-12f,
							   -0.0009079037f,
							   -6.181286e-12f,
								5.466609e-12f};

	arm_mat_init_f32(&xPrev, 22, 1, xPrevData);

	float32_t phi = xPrev.pData[4];
	float32_t h = xPrev.pData[6];

	arm_matrix_instance_f32 dpdot_dv;
	float32_t dpDotBuff[9];

// Expected Result (i == 201)
//
//		9.012364e-06               0               0
//	               0    1.099993e-05               0
//	               0               0              -1

	compute_dpdot_dv(phi, h, &dpdot_dv, dpDotBuff);
	return true;
}

bool test_compute_dvdot_dp(void) {

	arm_matrix_instance_f32 xPrev;

	float32_t xPrevData[22*1] = {0.707598f,
							   -0.0004724356f,
								   0.7066147f,
							   -0.0007329581f,
									35.34789f,
								   -117.8068f,
									671.9332f,
								 -0.02112561f,
								  0.01154266f,
								   -53.25378f,
								1.468478e-05f,
								0.0001556851f,
							   -8.201851e-06f,
							   -3.596146e-05f,
							   -3.867248e-09f,
							   -2.264934e-10f,
							   -7.372506e-12f,
								-5.61519e-13f,
								 1.10269e-12f,
							   -0.0009079037f,
							   -6.181286e-12f,
								5.466609e-12f};

	arm_mat_init_f32(&xPrev, 22, 1, xPrevData);

	float32_t phi = xPrev.pData[4];
	float32_t h = xPrev.pData[6];
	float32_t vn = xPrev.pData[7];
	float32_t ve = xPrev.pData[8];
	float32_t vd = xPrev.pData[9];

	arm_matrix_instance_f32 dpdot_dv;
	float32_t dpDotBuff[9];

//  Expected Value (i == 201)
//
//    -1.37479e-06               0   -2.783272e-14
//    0.004490802               0    1.507736e-14
//      0.04887648               0    -3.08613e-06

	compute_dvdot_dp(phi, h, vn, ve, vd, we, &dpdot_dv, dpDotBuff);
	return true;
}

bool test_compute_dvdot_dv(void) {

	arm_matrix_instance_f32 xPrev;

	float32_t xPrevData[22*1] = {0.707598f,
							   -0.0004724356f,
								   0.7066147f,
							   -0.0007329581f,
									35.34789f,
								   -117.8068f,
									671.9332f,
								 -0.02112561f,
								  0.01154266f,
								   -53.25378f,
								1.468478e-05f,
								0.0001556851f,
							   -8.201851e-06f,
							   -3.596146e-05f,
							   -3.867248e-09f,
							   -2.264934e-10f,
							   -7.372506e-12f,
								-5.61519e-13f,
								 1.10269e-12f,
							   -0.0009079037f,
							   -6.181286e-12f,
								5.466609e-12f};

	arm_mat_init_f32(&xPrev, 22, 1, xPrevData);

	float32_t phi = xPrev.pData[4];
	float32_t h = xPrev.pData[6];
	float32_t vn = xPrev.pData[7];
	float32_t ve = xPrev.pData[8];
	float32_t vd = xPrev.pData[9];

	arm_matrix_instance_f32 dvdot_dv;
	float32_t dvDotBuff[9];

//  Expected Value (i == 201)
//
//	   -8.376576e-06    8.437293e-05   -3.322961e-09
//	    8.437677e-05   -8.341535e-06    0.0001189586
//	    6.645922e-09   -0.0001189604               0

	compute_dvdot_dv(phi, h, vn, ve, vd, we, &dvdot_dv, dvDotBuff);
	return true;
}

bool test_compute_F(void) {

	arm_matrix_instance_f32 xPrev, aMeas, wMeas;

	float32_t xPrevData[22*1] = {0.707598f,
							   -0.0004724356f,
								   0.7066147f,
							   -0.0007329581f,
									35.34789f,
								   -117.8068f,
									671.9332f,
								 -0.02112561f,
								  0.01154266f,
								   -53.25378f,
								1.468478e-05f,
								0.0001556851f,
							   -8.201851e-06f,
							   -3.596146e-05f,
							   -3.867248e-09f,
							   -2.264934e-10f,
							   -7.372506e-12f,
								-5.61519e-13f,
								 1.10269e-12f,
							   -0.0009079037f,
							   -6.181286e-12f,
								5.466609e-12f};

	float32_t aMeasData[3] = {40.7497486442061f,
	        				  0.0398587448019052f,
							  -0.031245514204297f};

	float32_t wMeasData[3] = {-0.00141990364120018f,
							  -0.00497535811999743f,
							   0.00100243692685004f};

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

	compute_F(&q, &a_sf, &g_sf, &gBias, &aBias, phi, h, vn, ve, vd, &aMeas, &wMeas, we, &F, FBuff);
	return true;
}

bool test_compute_G(void) {

	arm_matrix_instance_f32 xPrev, aMeas, wMeas;

	float32_t xPrevData[22*1] = {0.707598f,
							   -0.0004724356f,
								   0.7066147f,
							   -0.0007329581f,
									35.34789f,
								   -117.8068f,
									671.9332f,
								 -0.02112561f,
								  0.01154266f,
								   -53.25378f,
								1.468478e-05f,
								0.0001556851f,
							   -8.201851e-06f,
							   -3.596146e-05f,
							   -3.867248e-09f,
							   -2.264934e-10f,
							   -7.372506e-12f,
								-5.61519e-13f,
								 1.10269e-12f,
							   -0.0009079037f,
							   -6.181286e-12f,
								5.466609e-12f};

	float32_t aMeasData[3] = {40.7497486442061f,
	        				  0.0398587448019052f,
							  -0.031245514204297f};

	float32_t wMeasData[3] = {-0.00141990364120018f,
							  -0.00497535811999743f,
							   0.00100243692685004f};

	arm_mat_init_f32(&xPrev, 22, 1, xPrevData);
	arm_mat_init_f32(&aMeas, 3, 1, aMeasData);
	arm_mat_init_f32(&wMeas, 3, 1, wMeasData);

	arm_matrix_instance_f32 q, gBias, aBias, g_sf, a_sf;
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

	arm_matrix_instance_f32 G;
	float32_t GBuff[21*12];

	compute_G(&g_sf, &a_sf, &q, &gBias, GBuff);
	return true;
}

bool test_compute_Pdot() {

	arm_matrix_instance_f32 xPrev, aMeas, wMeas;

	float32_t xPrevData[22*1] = {0.707598f,
							   -0.0004724356f,
								   0.7066147f,
							   -0.0007329581f,
									35.34789f,
								   -117.8068f,
									671.9332f,
								 -0.02112561f,
								  0.01154266f,
								   -53.25378f,
								1.468478e-05f,
								0.0001556851f,
							   -8.201851e-06f,
							   -3.596146e-05f,
							   -3.867248e-09f,
							   -2.264934e-10f,
							   -7.372506e-12f,
								-5.61519e-13f,
								 1.10269e-12f,
							   -0.0009079037f,
							   -6.181286e-12f,
								5.466609e-12f};

	float32_t aMeasData[3] = {40.7497486442061f,
	        				  0.0398587448019052f,
							  -0.031245514204297f};

	float32_t wMeasData[3] = {-0.00141990364120018f,
							  -0.00497535811999743f,
							   0.00100243692685004f};

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

	float32_t PBuff[21*21] = {0.00013657048,2.7062408e-05,0.00022695317,-7.3913635e-09,7.6066677e-08,-1.1648136e-06,-0.0012329968,0.010391491,2.7873214e-06,-5.1838688e-05,-9.8861938e-06,-8.4224383e-05,2.4745106e-12,2.4307056e-11,-1.508739e-12,-1.0468804e-09,3.4088381e-09,4.069511e-08,6.2464159e-11,2.0514117e-13,3.5960301e-15,
			2.7062404e-05,1.4328033e-05,4.8363563e-05,-1.724858e-09,1.6183003e-08,-2.6297573e-07,-0.00035478864,0.0022118932,6.4568911e-07,-9.962394e-06,-6.6793618e-06,-1.7967173e-05,5.241409e-13,4.1716357e-12,-6.9590687e-12,-3.4973344e-09,6.077812e-08,8.876718e-09,1.3231642e-11,2.3869519e-14,1.0910486e-13,
			0.00022695315,4.8363563e-05,0.00040954855,-1.3057948e-08,1.3469209e-07,-2.0743023e-06,-0.0021904828,0.018462012,4.9631813e-06,-8.3920386e-05,-1.7777766e-05,-0.000153299,4.370602e-12,4.1290783e-11,-2.9587914e-12,-2.9419754e-08,7.4480582e-09,1.1905557e-07,1.1032763e-10,3.2925098e-13,9.4099575e-15,
			-7.3913635e-09,-1.724858e-09,-1.3057947e-08,6.3231851e-06,-5.1340447e-12,-6.0734277e-07,1.2040528e-07,-6.5337537e-07,6.0954602e-07,1.8260216e-09,2.9636205e-10,4.0474863e-09,-4.5887153e-12,-6.3144027e-13,-2.0167163e-09,6.6022773e-13,1.691216e-12,-1.1084821e-12,-1.1599585e-10,-7.642392e-15,1.2760116e-11,
			7.6066691e-08,1.6183003e-08,1.346921e-07,-5.1340451e-12,7.6212996e-06,-1.8371353e-09,-8.312042e-07,7.0147062e-06,2.9605294e-09,-2.2205098e-08,-4.6713526e-09,-3.9263401e-08,6.9694093e-13,-2.4655438e-09,4.9427471e-13,-8.1430609e-12,-9.151695e-14,8.9915601e-12,1.854502e-11,-2.2136628e-11,-5.9055882e-15,
			-1.1648136e-06,-2.6297576e-07,-2.0743023e-06,-6.0734277e-07,-1.8371353e-09,0.42860335,0.00013019367,-0.00034038708,-0.47429562,4.5905912e-07,1.0552591e-07,8.2529203e-07,-0.00024788885,-6.6121295e-08,2.0428004e-08,1.4291764e-10,-1.0257653e-11,-2.4324193e-10,-0.0065694852,-1.0206708e-09,-4.4347692e-10,
			-0.0012329968,-0.00035478867,-0.0021904828,1.2040528e-07,-8.3120403e-07,0.00013019367,0.018693132,-0.10873403,-0.00015922434,0.00039653218,0.00011225755,0.00070680142,-7.4095581e-08,-1.1411775e-07,-0.00027800651,1.4182491e-07,-4.2644722e-08,-2.3394122e-07,-1.8088341e-06,-2.2966122e-09,3.9432525e-06,
			0.010391491,0.0022118934,0.018462013,-6.5337542e-07,7.0147057e-06,-0.00034038708,-0.10873404,0.917256,0.00050063012,-0.0033419356,-0.00070509216,-0.0059603779,1.5108678e-07,-0.00027800546,7.1549387e-08,-1.1982667e-06,9.8444957e-08,1.9682561e-06,3.8124749e-06,-4.7696617e-06,-1.6192911e-09,
			2.7873209e-06,6.4568917e-07,4.9631822e-06,6.0954602e-07,2.9605289e-09,-0.47429562,-0.00015922434,0.00050063012,0.52260005,-1.0425408e-06,-2.4794477e-07,-1.8711841e-06,0.00027521097,1.5197693e-07,-7.4862228e-08,-3.4471917e-10,1.3231011e-10,6.1225791e-10,0.0072037163,3.9225343e-09,2.2300752e-09,
			-5.1838677e-05,-9.9623931e-06,-8.3920393e-05,1.8260217e-09,-2.2205096e-08,4.5905901e-07,0.00039653224,-0.0033419356,-1.0425406e-06,3.5039277e-05,4.3915566e-06,3.7734641e-05,-6.269534e-13,1.845249e-12,8.0429093e-13,-2.4868221e-08,-2.0904383e-09,-1.937352e-08,-1.5829095e-11,1.141666e-14,-2.3362986e-15,
			-9.8861947e-06,-6.6793614e-06,-1.7777767e-05,2.9636238e-10,-4.6713509e-09,1.0552584e-07,0.00011225758,-0.00070509198,-2.4794463e-07,4.3915556e-06,1.4854118e-05,8.02878e-06,-1.2276809e-13,-2.0375594e-12,-1.6128045e-11,1.578915e-09,4.6248751e-08,-4.198812e-09,-3.1002969e-12,-1.2199178e-14,5.7526642e-14,
			-8.4224383e-05,-1.7967173e-05,-0.000153299,4.0474868e-09,-3.9263394e-08,8.2529209e-07,0.00070680148,-0.0059603783,-1.8711836e-06,3.7734633e-05,8.0287782e-06,8.067875e-05,-1.0885725e-12,-9.1784609e-13,1.4449108e-12,1.3867405e-08,-4.0679953e-09,1.1219076e-08,-2.748223e-11,-5.0750322e-15,-5.0097162e-15,
			2.474511e-12,5.2414106e-13,4.3706028e-12,-4.5887153e-12,6.9694093e-13,-0.00024788885,-7.4095581e-08,1.5108678e-07,0.00027521097,-6.2695335e-13,-1.2276806e-13,-1.0885723e-12,0.00017839829,-1.5615333e-13,-3.6538528e-15,-2.6805901e-16,-1.7598924e-16,3.2155726e-17,-4.0649663e-08,-2.5879586e-16,2.5161664e-16,
			2.4307056e-11,4.1716357e-12,4.129079e-11,-6.3144027e-13,-2.4655438e-09,-6.6121295e-08,-1.1411776e-07,-0.00027800546,1.5197693e-07,1.8452492e-12,-2.03756e-12,-9.1784523e-13,-1.5615333e-13,0.0001783999,-1.7128092e-18,1.5893337e-16,5.7724279e-15,1.8820145e-15,-3.9423352e-12,-1.3591324e-17,3.5206991e-20,
			-1.5087392e-12,-6.959067e-12,-2.9587918e-12,-2.0167163e-09,4.9427481e-13,2.0428004e-08,-0.00027800651,7.1549401e-08,-7.4862228e-08,8.0428925e-13,-1.6128047e-11,1.4449125e-12,-3.6538528e-15,-1.7128086e-18,0.0001783999,1.4075954e-16,4.7271252e-14,-3.5312071e-15,-9.2247668e-14,-1.2320088e-20,-1.0943587e-17,
			-1.0468999e-09,-3.497338e-09,-2.9419786e-08,6.6022784e-13,-8.14306e-12,1.4291796e-10,1.4182503e-07,-1.1982672e-06,-3.4471975e-10,-2.4868218e-08,1.5789176e-09,1.3867409e-08,-2.6805946e-16,1.5893668e-16,1.4076089e-16,9.9999925e-05,4.1744088e-13,-2.6092292e-12,-6.7674972e-15,3.3850754e-18,-9.4080928e-19,
			3.4088383e-09,6.0778113e-08,7.4480639e-09,1.6912146e-12,-9.1520765e-14,-1.0257505e-11,-4.2644885e-08,9.8444815e-08,1.3230966e-10,-2.0904261e-09,4.6248744e-08,-4.0680024e-09,-1.7598921e-16,5.7724258e-15,4.7271259e-14,4.1744538e-13,9.9999677e-05,2.4252819e-11,-4.4403949e-15,6.8497278e-17,-4.2274382e-16,
			4.069512e-08,8.8767225e-09,1.1905556e-07,-1.1084817e-12,8.991574e-12,-2.4324223e-10,-2.3394131e-07,1.9682568e-06,6.1225858e-10,-1.9373537e-08,-4.1988066e-09,1.1219086e-08,3.2156282e-17,1.8820217e-15,-3.5312037e-15,-2.6092271e-12,2.4252794e-11,9.9999939e-05,8.1038691e-16,2.5326785e-17,4.0579863e-17,
			6.2464166e-11,1.3231644e-11,1.1032764e-10,-1.1599585e-10,1.854502e-11,-0.0065694852,-1.8088341e-06,3.8124749e-06,0.0072037163,-1.5829092e-11,-3.1002958e-12,-2.7482232e-11,-4.0649663e-08,-3.9423352e-12,-9.2247655e-14,-6.7674756e-15,-4.4403923e-15,8.1039925e-16,9.8973731e-05,-6.5337064e-15,6.3524545e-15,
			2.0514115e-13,2.3869516e-14,3.2925092e-13,-7.642392e-15,-2.2136628e-11,-1.0206708e-09,-2.2966122e-09,-4.7696617e-06,3.9225343e-09,1.1416667e-14,-1.2199182e-14,-5.0750331e-15,-2.5879586e-16,-1.3591324e-17,-1.2320093e-20,3.3850737e-18,6.8497305e-17,2.5326795e-17,-6.5337064e-15,9.9999997e-05,1.602164e-22,
			3.596036e-15,1.0910487e-13,9.4099753e-15,1.2760116e-11,-5.9055882e-15,-4.4347692e-10,3.9432525e-06,-1.6192911e-09,2.2300752e-09,-2.3362854e-15,5.7526642e-14,-5.0097344e-15,2.5161664e-16,3.5206988e-20,-1.0943589e-17,-9.4079945e-19,-4.2274385e-16,4.0579909e-17,6.3524545e-15,1.6021638e-22,9.9999997e-05};

	float32_t QBuff[12*12] = {2.0943951e-05,0,0,0,0,0,0,0,0,0,0,0,0,2.0943951e-05,0,0,0,0,0,0,0,
			0,0,0,0,0,2.0943951e-05,0,0,0,0,0,0,0,0,0,0,0,0,1.454441e-06,0,0,
			0,0,0,0,0,0,0,0,0,0,1.454441e-06,0,0,0,0,0,0,0,0,0,0,
			0,0,1.454441e-06,0,0,0,0,0,0,0,0,0,0,0,0,0.0001962,0,0,0,0,0,
			0,0,0,0,0,0,0,0.0001962,0,0,0,0,0,0,0,0,0,0,0,0,0.0001962,
			0,0,0,0,0,0,0,0,0,0,0,0,3.92e-05,0,0,0,0,0,0,0,0,
			0,0,0,0,3.92e-05,0,0,0,0,0,0,0,0,0,0,0,0,3.92e-05};

	float32_t actualPdotBuff[21*21] = {0.00012700271,1.9647186e-05,0.00016962936,-1.3382646e-08,1.3721743e-07,-3.257215e-06,-0.0015021664,0.012699979,-7.586269e-06,-3.547961e-05,-4.4894437e-06,-3.8538696e-05,6.3338673e-13,2.9309676e-11,-6.400729e-13,1.6817259e-07,2.1900568e-09,1.9993166e-08,1.5991642e-11,5.2110584e-13,8.5930725e-16,
			1.9647185e-05,3.4109722e-05,3.4809069e-05,-3.4681584e-09,2.8731783e-08,-7.4704155e-07,-0.00069026626,0.0026416967,-1.8242936e-06,-4.218723e-06,-1.4818368e-05,-7.7236455e-06,1.0230435e-13,2.0045595e-12,-2.7594871e-11,-1.5356295e-09,4.6683977e-07,3.9869699e-09,2.5934127e-12,1.2337648e-14,5.6270803e-13,
			0.00016962938,3.4809069e-05,0.0003253457,-2.3486077e-08,2.4197601e-07,-5.782847e-06,-0.0026567867,0.022621751,-1.3713941e-05,-3.7482405e-05,-7.9875208e-06,-8.0271449e-05,1.0528018e-12,4.4289902e-11,-1.5275321e-12,-1.3866864e-08,4.1376631e-09,-1.1247928e-07,2.6579489e-11,7.5021619e-13,5.7126701e-15,
			-1.3382646e-08,-3.4681586e-09,-2.3486077e-08,2.1706714e-12,-1.4668301e-11,-6.0837266e-07,2.4028884e-07,-1.4843238e-06,3.0340226e-07,3.5736929e-09,1.0117062e-09,6.3699521e-09,-6.6778375e-13,-1.0284708e-12,-2.5054958e-09,1.2781787e-12,-3.8433124e-13,-2.1083643e-12,-1.6302067e-11,-2.0697906e-14,3.5538027e-11,
			1.3721743e-07,2.8731778e-08,2.4197598e-07,-1.4668301e-11,1.5432251e-10,-6.704771e-09,-1.8504657e-06,1.5588372e-05,1.0171388e-09,-3.6761048e-08,-7.7559603e-09,-6.5563725e-08,1.6619485e-12,-3.0580398e-09,7.8703504e-13,-1.3180852e-11,1.0828858e-12,2.1650681e-11,4.1937079e-11,-5.2465931e-11,-1.7812064e-14,
			-3.2572154e-06,-7.4704144e-07,-5.7828461e-06,-6.0837266e-07,-6.7047718e-09,0.94859123,0.00054238539,-0.001098463,-0.79055393,1.0425406e-06,2.4794463e-07,1.8711836e-06,-0.00027521097,-1.5197693e-07,7.4862228e-08,3.4471975e-10,-1.3230966e-10,-6.1225858e-10,-0.0072037163,-3.9225343e-09,-2.2300752e-09,
			-0.0015021667,-0.00069026626,-0.0026567869,2.4028884e-07,-1.8504659e-06,0.00054238539,0.029511284,-0.17883074,-0.00042101432,0.00040272184,0.00027170838,0.00072665449,-2.4597543e-07,-8.9563621e-08,-0.00017839711,1.4205941e-07,-2.4786898e-06,-3.5847455e-07,-5.607857e-06,-1.8766286e-09,3.1245108e-06,
			0.012699979,0.0026416965,0.022621753,-1.4843235e-06,1.5588372e-05,-0.001098463,-0.17883074,1.5074493,0.00020743764,-0.0034245548,-0.00072549272,-0.006255372,3.3452017e-07,-0.00017839562,4.1920487e-08,-1.2000032e-06,3.0479862e-07,4.8572679e-06,7.7374871e-06,-3.9858155e-06,-8.1404716e-10,
			-7.586269e-06,-1.8242938e-06,-1.3713942e-05,3.0340226e-07,1.0171384e-09,-0.79055393,-0.00042101432,0.00020743752,0.58784848,3.1398692e-06,7.8128187e-07,5.7127008e-06,0.00017690426,3.369793e-07,-2.4827122e-07,1.1040245e-09,-1.7819453e-09,-3.9914756e-09,0.0040331278,7.3607587e-09,4.3474291e-09,
			-3.547961e-05,-4.2187221e-06,-3.7482398e-05,3.5736925e-09,-3.6761048e-08,1.0425408e-06,0.00040272187,-0.0034245546,3.139869e-06,1.4544411e-06,0,0,0,0,0,0,0,0,0,0,0,
			-4.4894446e-06,-1.4818368e-05,-7.987519e-06,1.0117059e-09,-7.7559621e-09,2.4794477e-07,0.00027170841,-0.0007254926,7.8128193e-07,0,1.4544411e-06,0,0,0,0,0,0,0,0,0,0,
			-3.8538703e-05,-7.7236473e-06,-8.0271449e-05,6.3699517e-09,-6.5563725e-08,1.8711841e-06,0.00072665449,-0.006255372,5.7127004e-06,0,0,1.4544411e-06,0,0,0,0,0,0,0,0,0,
			6.3338679e-13,1.0230438e-13,1.0528021e-12,-6.6778375e-13,1.6619485e-12,-0.00027521097,-2.4597543e-07,3.3452017e-07,0.00017690426,0,0,0,3.92e-05,0,0,0,0,0,0,0,0,
			2.9309676e-11,2.0045588e-12,4.4289902e-11,-1.0284707e-12,-3.0580398e-09,-1.5197693e-07,-8.9563621e-08,-0.00017839562,3.369793e-07,0,0,0,0,3.92e-05,0,0,0,0,0,0,0,
			-6.4007458e-13,-2.7594873e-11,-1.5275304e-12,-2.5054958e-09,7.8703493e-13,7.4862228e-08,-0.00017839711,4.1920487e-08,-2.4827122e-07,0,0,0,0,0,3.92e-05,0,0,0,0,0,0,
			1.681726e-07,-1.5356271e-09,-1.3866861e-08,1.2781778e-12,-1.3180846e-11,3.4471917e-10,1.4205926e-07,-1.200002e-06,1.1040234e-09,0,0,0,0,0,0,0,0,0,0,0,0,
			2.190069e-09,4.6683974e-07,4.137656e-09,-3.8432975e-13,1.0828874e-12,-1.3231011e-10,-2.47869e-06,3.0479839e-07,-1.7819453e-09,0,0,0,0,0,0,0,0,0,0,0,0,
			1.9993148e-08,3.9869752e-09,-1.1247928e-07,-2.1083634e-12,2.1650674e-11,-6.1225791e-10,-3.5847435e-07,4.8572683e-06,-3.9914756e-09,0,0,0,0,0,0,0,0,0,0,0,0,
			1.5991646e-11,2.593414e-12,2.6579487e-11,-1.6302067e-11,4.1937079e-11,-0.0072037163,-5.607857e-06,7.7374871e-06,0.0040331278,0,0,0,0,0,0,0,0,0,0,0,0,
			5.2110584e-13,1.2337644e-14,7.5021619e-13,-2.0697906e-14,-5.2465931e-11,-3.9225343e-09,-1.8766286e-09,-3.9858155e-06,7.3607587e-09,0,0,0,0,0,0,0,0,0,0,0,0,
			8.5932027e-16,5.6270803e-13,5.7126519e-15,3.5538027e-11,-1.7812064e-14,-2.2300752e-09,3.1245108e-06,-8.1404716e-10,4.3474291e-09,0,0,0,0,0,0,0,0,0,0,0,0};

	arm_matrix_instance_f32 P, Q, Pdot, actualPdot;
	float32_t PdotBuff[21*21];
	arm_mat_init_f32(&P, 21, 21, PBuff);
	arm_mat_init_f32(&Q, 12, 12, QBuff);
	arm_mat_init_f32(&actualPdot, 21, 21, actualPdotBuff);

	compute_Pdot(&q, &a_sf, &g_sf, &gBias, &aBias, &aMeas, &wMeas, &P, &Q,
				 phi, h, vn, ve, vd, we, &Pdot, PdotBuff);

	bool test = areMatricesEqual(&Pdot, &actualPdot);
	return test;
}

bool test_integrate(void) {

	arm_matrix_instance_f32 x, P, Pq, qdot, pdot, vdot, Pdot, Pqdot, xMinus, Pminus, Pqminus;

	float32_t xData[22*1] = {0.70759803,
			-0.00047243558,
			0.70661467,
			-0.00073295814,
			35.347893,
			-117.80683,
			671.93317,
			-0.021125605,
			0.011542665,
			-53.253784,
			1.4684781e-05,
			0.00015568511,
			-8.2018505e-06,
			-3.5961464e-05,
			-3.8672479e-09,
			-2.2649335e-10,
			-7.3725063e-12,
			-5.6151904e-13,
			1.1026901e-12,
			-0.00090790371,
			-6.181286e-12,
			5.4666089e-12,
			};

	float32_t pData[21*21] = {0.00013657048,2.7062404e-05,0.00022695315,-7.3913635e-09,7.6066691e-08,-1.1648136e-06,-0.0012329968,0.010391491,2.7873209e-06,-5.1838677e-05,-9.8861947e-06,-8.4224383e-05,2.474511e-12,2.4307056e-11,-1.5087392e-12,-1.0468999e-09,3.4088383e-09,4.069512e-08,6.2464166e-11,2.0514115e-13,3.596036e-15,
			2.7062408e-05,1.4328033e-05,4.8363563e-05,-1.724858e-09,1.6183003e-08,-2.6297576e-07,-0.00035478867,0.0022118934,6.4568917e-07,-9.9623931e-06,-6.6793614e-06,-1.7967173e-05,5.2414106e-13,4.1716357e-12,-6.959067e-12,-3.497338e-09,6.0778113e-08,8.8767225e-09,1.3231644e-11,2.3869516e-14,1.0910487e-13,
			0.00022695317,4.8363563e-05,0.00040954855,-1.3057947e-08,1.346921e-07,-2.0743023e-06,-0.0021904828,0.018462013,4.9631822e-06,-8.3920393e-05,-1.7777767e-05,-0.000153299,4.3706028e-12,4.129079e-11,-2.9587918e-12,-2.9419786e-08,7.4480639e-09,1.1905556e-07,1.1032764e-10,3.2925092e-13,9.4099753e-15,
			-7.3913635e-09,-1.724858e-09,-1.3057948e-08,6.3231851e-06,-5.1340451e-12,-6.0734277e-07,1.2040528e-07,-6.5337542e-07,6.0954602e-07,1.8260217e-09,2.9636238e-10,4.0474868e-09,-4.5887153e-12,-6.3144027e-13,-2.0167163e-09,6.6022784e-13,1.6912146e-12,-1.1084817e-12,-1.1599585e-10,-7.642392e-15,1.2760116e-11,
			7.6066677e-08,1.6183003e-08,1.3469209e-07,-5.1340447e-12,7.6212996e-06,-1.8371353e-09,-8.3120403e-07,7.0147057e-06,2.9605289e-09,-2.2205096e-08,-4.6713509e-09,-3.9263394e-08,6.9694093e-13,-2.4655438e-09,4.9427481e-13,-8.14306e-12,-9.1520765e-14,8.991574e-12,1.854502e-11,-2.2136628e-11,-5.9055882e-15,
			-1.1648136e-06,-2.6297573e-07,-2.0743023e-06,-6.0734277e-07,-1.8371353e-09,0.42860335,0.00013019367,-0.00034038708,-0.47429562,4.5905901e-07,1.0552584e-07,8.2529209e-07,-0.00024788885,-6.6121295e-08,2.0428004e-08,1.4291796e-10,-1.0257505e-11,-2.4324223e-10,-0.0065694852,-1.0206708e-09,-4.4347692e-10,
			-0.0012329968,-0.00035478864,-0.0021904828,1.2040528e-07,-8.312042e-07,0.00013019367,0.018693132,-0.10873404,-0.00015922434,0.00039653224,0.00011225758,0.00070680148,-7.4095581e-08,-1.1411776e-07,-0.00027800651,1.4182503e-07,-4.2644885e-08,-2.3394131e-07,-1.8088341e-06,-2.2966122e-09,3.9432525e-06,
			0.010391491,0.0022118932,0.018462012,-6.5337537e-07,7.0147062e-06,-0.00034038708,-0.10873403,0.917256,0.00050063012,-0.0033419356,-0.00070509198,-0.0059603783,1.5108678e-07,-0.00027800546,7.1549401e-08,-1.1982672e-06,9.8444815e-08,1.9682568e-06,3.8124749e-06,-4.7696617e-06,-1.6192911e-09,
			2.7873214e-06,6.4568911e-07,4.9631813e-06,6.0954602e-07,2.9605294e-09,-0.47429562,-0.00015922434,0.00050063012,0.52260005,-1.0425406e-06,-2.4794463e-07,-1.8711836e-06,0.00027521097,1.5197693e-07,-7.4862228e-08,-3.4471975e-10,1.3230966e-10,6.1225858e-10,0.0072037163,3.9225343e-09,2.2300752e-09,
			-5.1838688e-05,-9.962394e-06,-8.3920386e-05,1.8260216e-09,-2.2205098e-08,4.5905912e-07,0.00039653218,-0.0033419356,-1.0425408e-06,3.5039277e-05,4.3915556e-06,3.7734633e-05,-6.2695335e-13,1.8452492e-12,8.0428925e-13,-2.4868218e-08,-2.0904261e-09,-1.9373537e-08,-1.5829092e-11,1.1416667e-14,-2.3362854e-15,
			-9.8861938e-06,-6.6793618e-06,-1.7777766e-05,2.9636205e-10,-4.6713526e-09,1.0552591e-07,0.00011225755,-0.00070509216,-2.4794477e-07,4.3915566e-06,1.4854118e-05,8.0287782e-06,-1.2276806e-13,-2.03756e-12,-1.6128047e-11,1.5789176e-09,4.6248744e-08,-4.1988066e-09,-3.1002958e-12,-1.2199182e-14,5.7526642e-14,
			-8.4224383e-05,-1.7967173e-05,-0.000153299,4.0474863e-09,-3.9263401e-08,8.2529203e-07,0.00070680142,-0.0059603779,-1.8711841e-06,3.7734641e-05,8.02878e-06,8.067875e-05,-1.0885723e-12,-9.1784523e-13,1.4449125e-12,1.3867409e-08,-4.0680024e-09,1.1219086e-08,-2.7482232e-11,-5.0750331e-15,-5.0097344e-15,
			2.4745106e-12,5.241409e-13,4.370602e-12,-4.5887153e-12,6.9694093e-13,-0.00024788885,-7.4095581e-08,1.5108678e-07,0.00027521097,-6.269534e-13,-1.2276809e-13,-1.0885725e-12,0.00017839829,-1.5615333e-13,-3.6538528e-15,-2.6805946e-16,-1.7598921e-16,3.2156282e-17,-4.0649663e-08,-2.5879586e-16,2.5161664e-16,
			2.4307056e-11,4.1716357e-12,4.1290783e-11,-6.3144027e-13,-2.4655438e-09,-6.6121295e-08,-1.1411775e-07,-0.00027800546,1.5197693e-07,1.845249e-12,-2.0375594e-12,-9.1784609e-13,-1.5615333e-13,0.0001783999,-1.7128086e-18,1.5893668e-16,5.7724258e-15,1.8820217e-15,-3.9423352e-12,-1.3591324e-17,3.5206988e-20,
			-1.508739e-12,-6.9590687e-12,-2.9587914e-12,-2.0167163e-09,4.9427471e-13,2.0428004e-08,-0.00027800651,7.1549387e-08,-7.4862228e-08,8.0429093e-13,-1.6128045e-11,1.4449108e-12,-3.6538528e-15,-1.7128092e-18,0.0001783999,1.4076089e-16,4.7271259e-14,-3.5312037e-15,-9.2247655e-14,-1.2320093e-20,-1.0943589e-17,
			-1.0468804e-09,-3.4973344e-09,-2.9419754e-08,6.6022773e-13,-8.1430609e-12,1.4291764e-10,1.4182491e-07,-1.1982667e-06,-3.4471917e-10,-2.4868221e-08,1.578915e-09,1.3867405e-08,-2.6805901e-16,1.5893337e-16,1.4075954e-16,9.9999925e-05,4.1744538e-13,-2.6092271e-12,-6.7674756e-15,3.3850737e-18,-9.4079945e-19,
			3.4088381e-09,6.077812e-08,7.4480582e-09,1.691216e-12,-9.151695e-14,-1.0257653e-11,-4.2644722e-08,9.8444957e-08,1.3231011e-10,-2.0904383e-09,4.6248751e-08,-4.0679953e-09,-1.7598924e-16,5.7724279e-15,4.7271252e-14,4.1744088e-13,9.9999677e-05,2.4252794e-11,-4.4403923e-15,6.8497305e-17,-4.2274385e-16,
			4.069511e-08,8.876718e-09,1.1905557e-07,-1.1084821e-12,8.9915601e-12,-2.4324193e-10,-2.3394122e-07,1.9682561e-06,6.1225791e-10,-1.937352e-08,-4.198812e-09,1.1219076e-08,3.2155726e-17,1.8820145e-15,-3.5312071e-15,-2.6092292e-12,2.4252819e-11,9.9999939e-05,8.1039925e-16,2.5326795e-17,4.0579909e-17,
			6.2464159e-11,1.3231642e-11,1.1032763e-10,-1.1599585e-10,1.854502e-11,-0.0065694852,-1.8088341e-06,3.8124749e-06,0.0072037163,-1.5829095e-11,-3.1002969e-12,-2.748223e-11,-4.0649663e-08,-3.9423352e-12,-9.2247668e-14,-6.7674972e-15,-4.4403949e-15,8.1038691e-16,9.8973731e-05,-6.5337064e-15,6.3524545e-15,
			2.0514117e-13,2.3869519e-14,3.2925098e-13,-7.642392e-15,-2.2136628e-11,-1.0206708e-09,-2.2966122e-09,-4.7696617e-06,3.9225343e-09,1.141666e-14,-1.2199178e-14,-5.0750322e-15,-2.5879586e-16,-1.3591324e-17,-1.2320088e-20,3.3850754e-18,6.8497278e-17,2.5326785e-17,-6.5337064e-15,9.9999997e-05,1.6021638e-22,
			3.5960301e-15,1.0910486e-13,9.4099575e-15,1.2760116e-11,-5.9055882e-15,-4.4347692e-10,3.9432525e-06,-1.6192911e-09,2.2300752e-09,-2.3362986e-15,5.7526642e-14,-5.0097162e-15,2.5161664e-16,3.5206991e-20,-1.0943587e-17,-9.4080928e-19,-4.2274382e-16,4.0579863e-17,6.3524545e-15,1.602164e-22,9.9999997e-05,
			};

	float32_t pqData[6*6] = {0.00013183476,2.754629e-05,0.00022928696,-4.9315957e-05,-1.0349562e-05,-8.5484651e-05,
			2.7546292e-05,8.1336611e-06,4.8778056e-05,-1.0242314e-05,-3.3395204e-06,-1.8188031e-05,
			0.00022928695,4.8778056e-05,0.00040836397,-8.5244814e-05,-1.8327106e-05,-0.0001525531,
			-4.9315928e-05,-1.0242307e-05,-8.524477e-05,3.297357e-05,4.6999962e-06,3.8813523e-05,
			-1.0349557e-05,-3.339519e-06,-1.8327099e-05,4.6999953e-06,1.2094387e-05,8.3462592e-06,
			-8.5484651e-05,-1.818803e-05,-0.00015255308,3.8813538e-05,8.3462564e-06,8.0041427e-05,
			};

	float32_t dt = 0.0100;

	float32_t qDotData[4*1] = {0.0018128692,
			-0.00018832,
			-0.0018146265,
			0.00085953728,
			};

	float32_t pDotData[3*1] = {-1.9039163e-07,
			1.2696849e-07,
			53.253784,
			};

	float32_t vDotData[3*1] = {0.025478311,
			-0.036005661,
			-30.99127,
			};

	float32_t PDotData[21*21] = {0.00012700271,1.9647186e-05,0.00016962936,-1.3382646e-08,1.3721743e-07,-3.257215e-06,-0.0015021664,0.012699979,-7.586269e-06,-3.547961e-05,-4.4894437e-06,-3.8538696e-05,6.3338673e-13,2.9309676e-11,-6.400729e-13,1.6817259e-07,2.1900568e-09,1.9993166e-08,1.5991642e-11,5.2110584e-13,8.5930725e-16,
			1.9647185e-05,3.4109722e-05,3.4809069e-05,-3.4681584e-09,2.8731783e-08,-7.4704155e-07,-0.00069026626,0.0026416967,-1.8242936e-06,-4.218723e-06,-1.4818368e-05,-7.7236455e-06,1.0230435e-13,2.0045595e-12,-2.7594871e-11,-1.5356295e-09,4.6683977e-07,3.9869699e-09,2.5934127e-12,1.2337648e-14,5.6270803e-13,
			0.00016962938,3.4809069e-05,0.0003253457,-2.3486077e-08,2.4197601e-07,-5.782847e-06,-0.0026567867,0.022621751,-1.3713941e-05,-3.7482405e-05,-7.9875208e-06,-8.0271449e-05,1.0528018e-12,4.4289902e-11,-1.5275321e-12,-1.3866864e-08,4.1376631e-09,-1.1247928e-07,2.6579489e-11,7.5021619e-13,5.7126701e-15,
			-1.3382646e-08,-3.4681586e-09,-2.3486077e-08,2.1706714e-12,-1.4668301e-11,-6.0837266e-07,2.4028884e-07,-1.4843238e-06,3.0340226e-07,3.5736929e-09,1.0117062e-09,6.3699521e-09,-6.6778375e-13,-1.0284708e-12,-2.5054958e-09,1.2781787e-12,-3.8433124e-13,-2.1083643e-12,-1.6302067e-11,-2.0697906e-14,3.5538027e-11,
			1.3721743e-07,2.8731778e-08,2.4197598e-07,-1.4668301e-11,1.5432251e-10,-6.704771e-09,-1.8504657e-06,1.5588372e-05,1.0171388e-09,-3.6761048e-08,-7.7559603e-09,-6.5563725e-08,1.6619485e-12,-3.0580398e-09,7.8703504e-13,-1.3180852e-11,1.0828858e-12,2.1650681e-11,4.1937079e-11,-5.2465931e-11,-1.7812064e-14,
			-3.2572154e-06,-7.4704144e-07,-5.7828461e-06,-6.0837266e-07,-6.7047718e-09,0.94859123,0.00054238539,-0.001098463,-0.79055393,1.0425406e-06,2.4794463e-07,1.8711836e-06,-0.00027521097,-1.5197693e-07,7.4862228e-08,3.4471975e-10,-1.3230966e-10,-6.1225858e-10,-0.0072037163,-3.9225343e-09,-2.2300752e-09,
			-0.0015021667,-0.00069026626,-0.0026567869,2.4028884e-07,-1.8504659e-06,0.00054238539,0.029511284,-0.17883074,-0.00042101432,0.00040272184,0.00027170838,0.00072665449,-2.4597543e-07,-8.9563621e-08,-0.00017839711,1.4205941e-07,-2.4786898e-06,-3.5847455e-07,-5.607857e-06,-1.8766286e-09,3.1245108e-06,
			0.012699979,0.0026416965,0.022621753,-1.4843235e-06,1.5588372e-05,-0.001098463,-0.17883074,1.5074493,0.00020743764,-0.0034245548,-0.00072549272,-0.006255372,3.3452017e-07,-0.00017839562,4.1920487e-08,-1.2000032e-06,3.0479862e-07,4.8572679e-06,7.7374871e-06,-3.9858155e-06,-8.1404716e-10,
			-7.586269e-06,-1.8242938e-06,-1.3713942e-05,3.0340226e-07,1.0171384e-09,-0.79055393,-0.00042101432,0.00020743752,0.58784848,3.1398692e-06,7.8128187e-07,5.7127008e-06,0.00017690426,3.369793e-07,-2.4827122e-07,1.1040245e-09,-1.7819453e-09,-3.9914756e-09,0.0040331278,7.3607587e-09,4.3474291e-09,
			-3.547961e-05,-4.2187221e-06,-3.7482398e-05,3.5736925e-09,-3.6761048e-08,1.0425408e-06,0.00040272187,-0.0034245546,3.139869e-06,1.4544411e-06,0,0,0,0,0,0,0,0,0,0,0,
			-4.4894446e-06,-1.4818368e-05,-7.987519e-06,1.0117059e-09,-7.7559621e-09,2.4794477e-07,0.00027170841,-0.0007254926,7.8128193e-07,0,1.4544411e-06,0,0,0,0,0,0,0,0,0,0,
			-3.8538703e-05,-7.7236473e-06,-8.0271449e-05,6.3699517e-09,-6.5563725e-08,1.8711841e-06,0.00072665449,-0.006255372,5.7127004e-06,0,0,1.4544411e-06,0,0,0,0,0,0,0,0,0,
			6.3338679e-13,1.0230438e-13,1.0528021e-12,-6.6778375e-13,1.6619485e-12,-0.00027521097,-2.4597543e-07,3.3452017e-07,0.00017690426,0,0,0,3.92e-05,0,0,0,0,0,0,0,0,
			2.9309676e-11,2.0045588e-12,4.4289902e-11,-1.0284707e-12,-3.0580398e-09,-1.5197693e-07,-8.9563621e-08,-0.00017839562,3.369793e-07,0,0,0,0,3.92e-05,0,0,0,0,0,0,0,
			-6.4007458e-13,-2.7594873e-11,-1.5275304e-12,-2.5054958e-09,7.8703493e-13,7.4862228e-08,-0.00017839711,4.1920487e-08,-2.4827122e-07,0,0,0,0,0,3.92e-05,0,0,0,0,0,0,
			1.681726e-07,-1.5356271e-09,-1.3866861e-08,1.2781778e-12,-1.3180846e-11,3.4471917e-10,1.4205926e-07,-1.200002e-06,1.1040234e-09,0,0,0,0,0,0,0,0,0,0,0,0,
			2.190069e-09,4.6683974e-07,4.137656e-09,-3.8432975e-13,1.0828874e-12,-1.3231011e-10,-2.47869e-06,3.0479839e-07,-1.7819453e-09,0,0,0,0,0,0,0,0,0,0,0,0,
			1.9993148e-08,3.9869752e-09,-1.1247928e-07,-2.1083634e-12,2.1650674e-11,-6.1225791e-10,-3.5847435e-07,4.8572683e-06,-3.9914756e-09,0,0,0,0,0,0,0,0,0,0,0,0,
			1.5991646e-11,2.593414e-12,2.6579487e-11,-1.6302067e-11,4.1937079e-11,-0.0072037163,-5.607857e-06,7.7374871e-06,0.0040331278,0,0,0,0,0,0,0,0,0,0,0,0,
			5.2110584e-13,1.2337644e-14,7.5021619e-13,-2.0697906e-14,-5.2465931e-11,-3.9225343e-09,-1.8766286e-09,-3.9858155e-06,7.3607587e-09,0,0,0,0,0,0,0,0,0,0,0,0,
			8.5932027e-16,5.6270803e-13,5.7126519e-15,3.5538027e-11,-1.7812064e-14,-2.2300752e-09,3.1245108e-06,-8.1404716e-10,4.3474291e-09,0,0,0,0,0,0,0,0,0,0,0,0,
			};

	float32_t pqDotData[6*6] = {0.00011957584,2.059187e-05,0.00017072941,-3.2520169e-05,-4.6851833e-06,-3.9076884e-05,
			2.059187e-05,2.7622997e-05,3.6515128e-05,-4.6018849e-06,-1.2090873e-05,-8.402154e-06,
			0.00017072947,3.6515135e-05,0.00032605013,-3.8004622e-05,-8.3158302e-06,-8.0497099e-05,
			-3.2520169e-05,-4.6018858e-06,-3.8004608e-05,1.0427146e-06,-5.1718093e-08,-2.4873563e-07,
			-4.6851824e-06,-1.2090873e-05,-8.3158329e-06,-5.1718121e-08,1.4512168e-06,1.1775946e-08,
			-3.9076898e-05,-8.4021513e-06,-8.0497099e-05,-2.4873563e-07,1.177597e-08,1.8693923e-06,
			};


	float32_t xMinusData[22];
	float32_t PMinusData[21*21];
	float32_t PqMinusBuff[6*6];

	arm_mat_init_f32(&x, 22, 1, xData);
	arm_mat_init_f32(&P, 21, 21, pData);
	arm_mat_init_f32(&Pq, 6, 6, pqData);
	arm_mat_init_f32(&qdot, 4, 1, qDotData);
	arm_mat_init_f32(&pdot, 3, 1, pDotData);
	arm_mat_init_f32(&vdot, 3, 1, vDotData);
	arm_mat_init_f32(&Pdot, 21, 21, PDotData);
	arm_mat_init_f32(&Pqdot, 6, 6, pqDotData);

	integrate(&x, &P, &qdot, &pdot,
			  &vdot, &Pdot, dt, &xMinus,
			  &Pminus, xMinusData,
			  PMinusData);

	arm_matrix_instance_f32 xMinusTrue, PMinusTrue, PqMinusTrue;

	float32_t xMinusTrueData[22*1] = {0.70761615,
			-0.00047431877,
			0.70659655,
			-0.00072436279,
			35.347893,
			-117.80683,
			672.4657,
			-0.020870822,
			0.011182608,
			-53.563698,
			1.4684781e-05,
			0.00015568511,
			-8.2018505e-06,
			-3.5961464e-05,
			-3.8672479e-09,
			-2.2649335e-10,
			-7.3725063e-12,
			-5.6151904e-13,
			1.1026901e-12,
			-0.00090790371,
			-6.181286e-12,
			5.4666089e-12,
			};

	float32_t PMinusDataTrue[21*21] = {0.00013784051,2.7258877e-05,0.00022864944,-7.5251902e-09,7.7438862e-08,-1.1973858e-06,-0.0012480185,0.010518491,2.7114581e-06,-5.2193474e-05,-9.9310892e-06,-8.4609768e-05,2.480845e-12,2.4600154e-11,-1.5151399e-12,6.3482597e-10,3.4307388e-09,4.0895053e-08,6.262408e-11,2.103522e-13,3.6046292e-15,
			2.725888e-05,1.466913e-05,4.8711652e-05,-1.7595396e-09,1.6470322e-08,-2.7044618e-07,-0.00036169135,0.0022383104,6.2744624e-07,-1.000458e-05,-6.8275449e-06,-1.8044409e-05,5.2516411e-13,4.1916813e-12,-7.2350155e-12,-3.5126944e-09,6.5446514e-08,8.9165919e-09,1.3257578e-11,2.3992893e-14,1.1473195e-13,
			0.00022864946,4.8711652e-05,0.00041280201,-1.3292808e-08,1.3711185e-07,-2.1321307e-06,-0.0022170506,0.018688232,4.8260426e-06,-8.4295214e-05,-1.7857643e-05,-0.00015410171,4.3811309e-12,4.1733689e-11,-2.9740672e-12,-2.9558455e-08,7.4894402e-09,1.1793077e-07,1.1059344e-10,3.3675309e-13,9.4671018e-15,
			-7.5251902e-09,-1.7595396e-09,-1.3292809e-08,6.3231851e-06,-5.2807281e-12,-6.1342649e-07,1.2280816e-07,-6.6821866e-07,6.1258004e-07,1.8617586e-09,3.0647945e-10,4.1111865e-09,-4.5953931e-12,-6.4172495e-13,-2.0417712e-09,6.7300961e-13,1.6873713e-12,-1.1295653e-12,-1.1615887e-10,-7.8493713e-15,1.3115497e-11,
			7.7438848e-08,1.6470322e-08,1.3711184e-07,-5.2807277e-12,7.6213009e-06,-1.904183e-09,-8.4970867e-07,7.1705895e-06,2.9707004e-09,-2.2572706e-08,-4.7489106e-09,-3.991903e-08,7.135604e-13,-2.4961242e-09,5.0214515e-13,-8.2748686e-12,-8.0691909e-14,9.2080805e-12,1.8964391e-11,-2.2661286e-11,-6.0837087e-15,
			-1.1973858e-06,-2.7044615e-07,-2.1321307e-06,-6.1342649e-07,-1.904183e-09,0.43808925,0.00013561752,-0.00035137171,-0.48220116,4.6948441e-07,1.0800529e-07,8.4400392e-07,-0.00025064097,-6.7641068e-08,2.1176627e-08,1.4636516e-10,-1.1580601e-11,-2.4936481e-10,-0.0066415225,-1.0598961e-09,-4.6577769e-10,
			-0.0012480185,-0.00036169132,-0.0022170506,1.2280816e-07,-8.4970884e-07,0.00013561752,0.018988244,-0.11052235,-0.00016343448,0.00040055945,0.00011497466,0.00071406801,-7.6555338e-08,-1.150134e-07,-0.00027979049,1.4324561e-07,-6.7431785e-08,-2.3752605e-07,-1.8649126e-06,-2.3153786e-09,3.9744978e-06,
			0.010518491,0.0022383102,0.01868823,-6.682186e-07,7.1705899e-06,-0.00035137171,-0.11052234,0.93233049,0.00050270453,-0.0033761812,-0.00071234693,-0.006022932,1.5443199e-07,-0.00027978941,7.1968607e-08,-1.2102672e-06,1.014928e-07,2.0168295e-06,3.8898497e-06,-4.8095199e-06,-1.6274316e-09,
			2.7114586e-06,6.2744618e-07,4.8260417e-06,6.1258004e-07,2.9707008e-09,-0.48220116,-0.00016343448,0.00050270447,0.52847856,-1.0111419e-06,-2.4013181e-07,-1.8140566e-06,0.00027698002,1.5534673e-07,-7.7344943e-08,-3.336795e-10,1.1449021e-10,5.7234384e-10,0.0072440477,3.9961421e-09,2.2735496e-09,
			-5.2193485e-05,-1.0004581e-05,-8.4295207e-05,1.8617585e-09,-2.2572708e-08,4.6948452e-07,0.00040055939,-0.0033761812,-1.0111421e-06,3.5053821e-05,4.3915556e-06,3.7734633e-05,-6.2695335e-13,1.8452492e-12,8.0428925e-13,-2.4868218e-08,-2.0904261e-09,-1.9373537e-08,-1.5829092e-11,1.1416667e-14,-2.3362854e-15,
			-9.9310882e-06,-6.8275453e-06,-1.7857641e-05,3.0647909e-10,-4.7489124e-09,1.0800535e-07,0.00011497463,-0.0007123471,-2.4013195e-07,4.3915566e-06,1.4868663e-05,8.0287782e-06,-1.2276806e-13,-2.03756e-12,-1.6128047e-11,1.5789176e-09,4.6248744e-08,-4.1988066e-09,-3.1002958e-12,-1.2199182e-14,5.7526642e-14,
			-8.4609768e-05,-1.8044409e-05,-0.00015410171,4.1111861e-09,-3.9919037e-08,8.4400386e-07,0.00071406795,-0.0060229315,-1.8140571e-06,3.7734641e-05,8.02878e-06,8.0693295e-05,-1.0885723e-12,-9.1784523e-13,1.4449125e-12,1.3867409e-08,-4.0680024e-09,1.1219086e-08,-2.7482232e-11,-5.0750331e-15,-5.0097344e-15,
			2.4808445e-12,5.2516395e-13,4.38113e-12,-4.5953931e-12,7.135604e-13,-0.00025064097,-7.6555338e-08,1.5443199e-07,0.00027698002,-6.269534e-13,-1.2276809e-13,-1.0885725e-12,0.00017879029,-1.5615333e-13,-3.6538528e-15,-2.6805946e-16,-1.7598921e-16,3.2156282e-17,-4.0649663e-08,-2.5879586e-16,2.5161664e-16,
			2.4600154e-11,4.1916813e-12,4.1733682e-11,-6.4172495e-13,-2.4961242e-09,-6.7641068e-08,-1.1501339e-07,-0.00027978941,1.5534673e-07,1.845249e-12,-2.0375594e-12,-9.1784609e-13,-1.5615333e-13,0.0001787919,-1.7128086e-18,1.5893668e-16,5.7724258e-15,1.8820217e-15,-3.9423352e-12,-1.3591324e-17,3.5206988e-20,
			-1.5151397e-12,-7.2350173e-12,-2.9740667e-12,-2.0417712e-09,5.0214504e-13,2.1176627e-08,-0.00027979049,7.1968593e-08,-7.7344943e-08,8.0429093e-13,-1.6128045e-11,1.4449108e-12,-3.6538528e-15,-1.7128092e-18,0.0001787919,1.4076089e-16,4.7271259e-14,-3.5312037e-15,-9.2247655e-14,-1.2320093e-20,-1.0943589e-17,
			6.3484573e-10,-3.5126906e-09,-2.9558421e-08,6.730095e-13,-8.2748695e-12,1.4636482e-10,1.432455e-07,-1.2102666e-06,-3.3367895e-10,-2.4868221e-08,1.578915e-09,1.3867405e-08,-2.6805901e-16,1.5893337e-16,1.4075954e-16,9.9999925e-05,4.1744538e-13,-2.6092271e-12,-6.7674756e-15,3.3850737e-18,-9.4079945e-19,
			3.4307388e-09,6.5446514e-08,7.4894349e-09,1.6873727e-12,-8.0688074e-14,-1.1580755e-11,-6.7431621e-08,1.0149294e-07,1.1449065e-10,-2.0904383e-09,4.6248751e-08,-4.0679953e-09,-1.7598924e-16,5.7724279e-15,4.7271252e-14,4.1744088e-13,9.9999677e-05,2.4252794e-11,-4.4403923e-15,6.8497305e-17,-4.2274385e-16,
			4.0895042e-08,8.9165875e-09,1.1793077e-07,-1.1295657e-12,9.2080666e-12,-2.493645e-10,-2.3752597e-07,2.0168288e-06,5.7234317e-10,-1.937352e-08,-4.198812e-09,1.1219076e-08,3.2155726e-17,1.8820145e-15,-3.5312071e-15,-2.6092292e-12,2.4252819e-11,9.9999939e-05,8.1039925e-16,2.5326795e-17,4.0579909e-17,
			6.2624073e-11,1.3257576e-11,1.1059342e-10,-1.1615887e-10,1.8964391e-11,-0.0066415225,-1.8649126e-06,3.8898497e-06,0.0072440477,-1.5829095e-11,-3.1002969e-12,-2.748223e-11,-4.0649663e-08,-3.9423352e-12,-9.2247668e-14,-6.7674972e-15,-4.4403949e-15,8.1038691e-16,9.8973731e-05,-6.5337064e-15,6.3524545e-15,
			2.1035223e-13,2.3992896e-14,3.3675314e-13,-7.8493713e-15,-2.2661286e-11,-1.0598961e-09,-2.3153786e-09,-4.8095199e-06,3.9961421e-09,1.141666e-14,-1.2199178e-14,-5.0750322e-15,-2.5879586e-16,-1.3591324e-17,-1.2320088e-20,3.3850754e-18,6.8497278e-17,2.5326785e-17,-6.5337064e-15,9.9999997e-05,1.6021638e-22,
			3.6046232e-15,1.1473194e-13,9.467084e-15,1.3115497e-11,-6.0837087e-15,-4.6577769e-10,3.9744978e-06,-1.6274316e-09,2.2735496e-09,-2.3362986e-15,5.7526642e-14,-5.0097162e-15,2.5161664e-16,3.5206991e-20,-1.0943587e-17,-9.4080928e-19,-4.2274382e-16,4.0579863e-17,6.3524545e-15,1.602164e-22,9.9999997e-05,
			};

	float32_t PqMinusDataTrue[6*6] = {0.00013303052,2.7752209e-05,0.00023099425,-4.9641159e-05,-1.0396414e-05,-8.5875421e-05,
			2.775221e-05,8.4098911e-06,4.9143207e-05,-1.0288332e-05,-3.4604291e-06,-1.8272052e-05,
			0.00023099424,4.9143207e-05,0.00041162447,-8.5624859e-05,-1.8410265e-05,-0.00015335807,
			-4.964113e-05,-1.0288326e-05,-8.5624815e-05,3.2983997e-05,4.6994792e-06,3.8811035e-05,
			-1.0396408e-05,-3.4604277e-06,-1.8410257e-05,4.6994783e-06,1.2108899e-05,8.3463765e-06,
			-8.5875421e-05,-1.8272051e-05,-0.00015335805,3.8811049e-05,8.3463738e-06,8.0060119e-05,
			};

	arm_mat_init_f32(&xMinusTrue, 22, 1, xMinusTrueData);
	arm_mat_init_f32(&PMinusTrue, 21, 21, PMinusDataTrue);
	arm_mat_init_f32(&PqMinusTrue, 6, 6, PqMinusDataTrue);

	bool test1 = areMatricesEqual(&xMinus, &xMinusTrue);
	bool test2 = areMatricesEqual(&Pminus, &PMinusTrue);
	bool test3 = areMatricesEqual(&Pqminus, &PqMinusTrue);

	bool test = (test1 && test2) & test3;
	return test;
}

bool test_propogate(void) {

	arm_matrix_instance_f32 xPlus, P_plus, Pq_plus, what, aHatN, wMeas, aMeas, Q, Qq;

		float32_t xData[22*1] = {0.70759803,
				-0.00047243558,
				0.70661467,
				-0.00073295814,
				35.347893,
				-117.80683,
				671.93317,
				-0.021125605,
				0.011542665,
				-53.253784,
				1.4684781e-05,
				0.00015568511,
				-8.2018505e-06,
				-3.5961464e-05,
				-3.8672479e-09,
				-2.2649335e-10,
				-7.3725063e-12,
				-5.6151904e-13,
				1.1026901e-12,
				-0.00090790371,
				-6.181286e-12,
				5.4666089e-12
				};

		float32_t pData[21*21] = {0.00013657048,2.7062404e-05,0.00022695315,-7.3913635e-09,7.6066691e-08,-1.1648136e-06,-0.0012329968,0.010391491,2.7873209e-06,-5.1838677e-05,-9.8861947e-06,-8.4224383e-05,2.474511e-12,2.4307056e-11,-1.5087392e-12,-1.0468999e-09,3.4088383e-09,4.069512e-08,6.2464166e-11,2.0514115e-13,3.596036e-15,
				2.7062408e-05,1.4328033e-05,4.8363563e-05,-1.724858e-09,1.6183003e-08,-2.6297576e-07,-0.00035478867,0.0022118934,6.4568917e-07,-9.9623931e-06,-6.6793614e-06,-1.7967173e-05,5.2414106e-13,4.1716357e-12,-6.959067e-12,-3.497338e-09,6.0778113e-08,8.8767225e-09,1.3231644e-11,2.3869516e-14,1.0910487e-13,
				0.00022695317,4.8363563e-05,0.00040954855,-1.3057947e-08,1.346921e-07,-2.0743023e-06,-0.0021904828,0.018462013,4.9631822e-06,-8.3920393e-05,-1.7777767e-05,-0.000153299,4.3706028e-12,4.129079e-11,-2.9587918e-12,-2.9419786e-08,7.4480639e-09,1.1905556e-07,1.1032764e-10,3.2925092e-13,9.4099753e-15,
				-7.3913635e-09,-1.724858e-09,-1.3057948e-08,6.3231851e-06,-5.1340451e-12,-6.0734277e-07,1.2040528e-07,-6.5337542e-07,6.0954602e-07,1.8260217e-09,2.9636238e-10,4.0474868e-09,-4.5887153e-12,-6.3144027e-13,-2.0167163e-09,6.6022784e-13,1.6912146e-12,-1.1084817e-12,-1.1599585e-10,-7.642392e-15,1.2760116e-11,
				7.6066677e-08,1.6183003e-08,1.3469209e-07,-5.1340447e-12,7.6212996e-06,-1.8371353e-09,-8.3120403e-07,7.0147057e-06,2.9605289e-09,-2.2205096e-08,-4.6713509e-09,-3.9263394e-08,6.9694093e-13,-2.4655438e-09,4.9427481e-13,-8.14306e-12,-9.1520765e-14,8.991574e-12,1.854502e-11,-2.2136628e-11,-5.9055882e-15,
				-1.1648136e-06,-2.6297573e-07,-2.0743023e-06,-6.0734277e-07,-1.8371353e-09,0.42860335,0.00013019367,-0.00034038708,-0.47429562,4.5905901e-07,1.0552584e-07,8.2529209e-07,-0.00024788885,-6.6121295e-08,2.0428004e-08,1.4291796e-10,-1.0257505e-11,-2.4324223e-10,-0.0065694852,-1.0206708e-09,-4.4347692e-10,
				-0.0012329968,-0.00035478864,-0.0021904828,1.2040528e-07,-8.312042e-07,0.00013019367,0.018693132,-0.10873404,-0.00015922434,0.00039653224,0.00011225758,0.00070680148,-7.4095581e-08,-1.1411776e-07,-0.00027800651,1.4182503e-07,-4.2644885e-08,-2.3394131e-07,-1.8088341e-06,-2.2966122e-09,3.9432525e-06,
				0.010391491,0.0022118932,0.018462012,-6.5337537e-07,7.0147062e-06,-0.00034038708,-0.10873403,0.917256,0.00050063012,-0.0033419356,-0.00070509198,-0.0059603783,1.5108678e-07,-0.00027800546,7.1549401e-08,-1.1982672e-06,9.8444815e-08,1.9682568e-06,3.8124749e-06,-4.7696617e-06,-1.6192911e-09,
				2.7873214e-06,6.4568911e-07,4.9631813e-06,6.0954602e-07,2.9605294e-09,-0.47429562,-0.00015922434,0.00050063012,0.52260005,-1.0425406e-06,-2.4794463e-07,-1.8711836e-06,0.00027521097,1.5197693e-07,-7.4862228e-08,-3.4471975e-10,1.3230966e-10,6.1225858e-10,0.0072037163,3.9225343e-09,2.2300752e-09,
				-5.1838688e-05,-9.962394e-06,-8.3920386e-05,1.8260216e-09,-2.2205098e-08,4.5905912e-07,0.00039653218,-0.0033419356,-1.0425408e-06,3.5039277e-05,4.3915556e-06,3.7734633e-05,-6.2695335e-13,1.8452492e-12,8.0428925e-13,-2.4868218e-08,-2.0904261e-09,-1.9373537e-08,-1.5829092e-11,1.1416667e-14,-2.3362854e-15,
				-9.8861938e-06,-6.6793618e-06,-1.7777766e-05,2.9636205e-10,-4.6713526e-09,1.0552591e-07,0.00011225755,-0.00070509216,-2.4794477e-07,4.3915566e-06,1.4854118e-05,8.0287782e-06,-1.2276806e-13,-2.03756e-12,-1.6128047e-11,1.5789176e-09,4.6248744e-08,-4.1988066e-09,-3.1002958e-12,-1.2199182e-14,5.7526642e-14,
				-8.4224383e-05,-1.7967173e-05,-0.000153299,4.0474863e-09,-3.9263401e-08,8.2529203e-07,0.00070680142,-0.0059603779,-1.8711841e-06,3.7734641e-05,8.02878e-06,8.067875e-05,-1.0885723e-12,-9.1784523e-13,1.4449125e-12,1.3867409e-08,-4.0680024e-09,1.1219086e-08,-2.7482232e-11,-5.0750331e-15,-5.0097344e-15,
				2.4745106e-12,5.241409e-13,4.370602e-12,-4.5887153e-12,6.9694093e-13,-0.00024788885,-7.4095581e-08,1.5108678e-07,0.00027521097,-6.269534e-13,-1.2276809e-13,-1.0885725e-12,0.00017839829,-1.5615333e-13,-3.6538528e-15,-2.6805946e-16,-1.7598921e-16,3.2156282e-17,-4.0649663e-08,-2.5879586e-16,2.5161664e-16,
				2.4307056e-11,4.1716357e-12,4.1290783e-11,-6.3144027e-13,-2.4655438e-09,-6.6121295e-08,-1.1411775e-07,-0.00027800546,1.5197693e-07,1.845249e-12,-2.0375594e-12,-9.1784609e-13,-1.5615333e-13,0.0001783999,-1.7128086e-18,1.5893668e-16,5.7724258e-15,1.8820217e-15,-3.9423352e-12,-1.3591324e-17,3.5206988e-20,
				-1.508739e-12,-6.9590687e-12,-2.9587914e-12,-2.0167163e-09,4.9427471e-13,2.0428004e-08,-0.00027800651,7.1549387e-08,-7.4862228e-08,8.0429093e-13,-1.6128045e-11,1.4449108e-12,-3.6538528e-15,-1.7128092e-18,0.0001783999,1.4076089e-16,4.7271259e-14,-3.5312037e-15,-9.2247655e-14,-1.2320093e-20,-1.0943589e-17,
				-1.0468804e-09,-3.4973344e-09,-2.9419754e-08,6.6022773e-13,-8.1430609e-12,1.4291764e-10,1.4182491e-07,-1.1982667e-06,-3.4471917e-10,-2.4868221e-08,1.578915e-09,1.3867405e-08,-2.6805901e-16,1.5893337e-16,1.4075954e-16,9.9999925e-05,4.1744538e-13,-2.6092271e-12,-6.7674756e-15,3.3850737e-18,-9.4079945e-19,
				3.4088381e-09,6.077812e-08,7.4480582e-09,1.691216e-12,-9.151695e-14,-1.0257653e-11,-4.2644722e-08,9.8444957e-08,1.3231011e-10,-2.0904383e-09,4.6248751e-08,-4.0679953e-09,-1.7598924e-16,5.7724279e-15,4.7271252e-14,4.1744088e-13,9.9999677e-05,2.4252794e-11,-4.4403923e-15,6.8497305e-17,-4.2274385e-16,
				4.069511e-08,8.876718e-09,1.1905557e-07,-1.1084821e-12,8.9915601e-12,-2.4324193e-10,-2.3394122e-07,1.9682561e-06,6.1225791e-10,-1.937352e-08,-4.198812e-09,1.1219076e-08,3.2155726e-17,1.8820145e-15,-3.5312071e-15,-2.6092292e-12,2.4252819e-11,9.9999939e-05,8.1039925e-16,2.5326795e-17,4.0579909e-17,
				6.2464159e-11,1.3231642e-11,1.1032763e-10,-1.1599585e-10,1.854502e-11,-0.0065694852,-1.8088341e-06,3.8124749e-06,0.0072037163,-1.5829095e-11,-3.1002969e-12,-2.748223e-11,-4.0649663e-08,-3.9423352e-12,-9.2247668e-14,-6.7674972e-15,-4.4403949e-15,8.1038691e-16,9.8973731e-05,-6.5337064e-15,6.3524545e-15,
				2.0514117e-13,2.3869519e-14,3.2925098e-13,-7.642392e-15,-2.2136628e-11,-1.0206708e-09,-2.2966122e-09,-4.7696617e-06,3.9225343e-09,1.141666e-14,-1.2199178e-14,-5.0750322e-15,-2.5879586e-16,-1.3591324e-17,-1.2320088e-20,3.3850754e-18,6.8497278e-17,2.5326785e-17,-6.5337064e-15,9.9999997e-05,1.6021638e-22,
				3.5960301e-15,1.0910486e-13,9.4099575e-15,1.2760116e-11,-5.9055882e-15,-4.4347692e-10,3.9432525e-06,-1.6192911e-09,2.2300752e-09,-2.3362986e-15,5.7526642e-14,-5.0097162e-15,2.5161664e-16,3.5206991e-20,-1.0943587e-17,-9.4080928e-19,-4.2274382e-16,4.0579863e-17,6.3524545e-15,1.602164e-22,9.9999997e-05
				};

		float32_t pqData[6*6] = {0.00013183476,2.754629e-05,0.00022928696,-4.9315957e-05,-1.0349562e-05,-8.5484651e-05,
				2.7546292e-05,8.1336611e-06,4.8778056e-05,-1.0242314e-05,-3.3395204e-06,-1.8188031e-05,
				0.00022928695,4.8778056e-05,0.00040836397,-8.5244814e-05,-1.8327106e-05,-0.0001525531,
				-4.9315928e-05,-1.0242307e-05,-8.524477e-05,3.297357e-05,4.6999962e-06,3.8813523e-05,
				-1.0349557e-05,-3.339519e-06,-1.8327099e-05,4.6999953e-06,1.2094387e-05,8.3462592e-06,
				-8.5484651e-05,-1.818803e-05,-0.00015255308,3.8813538e-05,8.3462564e-06,8.0041427e-05
				};

		float32_t aMeasData[3] = {40.749749,
				0.039858745,
				-0.031245514
				};

		float32_t dt = 0.0100;

		float32_t wHatData[3] = {-0.00147686,
				-0.0051311404,
				0.00095121731
				};

		float32_t aHatNData[3] = {0.025479108,
				-0.029668881,
				-40.786831
				};

		float32_t wMeasData[3] = {
				-0.0014199036,
				-0.0049753581,
				0.0010024369};

		float32_t QBuff[12*12] = {2.0943951e-05,0,0,0,0,0,0,0,0,0,0,0,0,2.0943951e-05,0,0,0,0,0,0,0,
				0,0,0,0,0,2.0943951e-05,0,0,0,0,0,0,0,0,0,0,0,0,1.454441e-06,0,0,
				0,0,0,0,0,0,0,0,0,0,1.454441e-06,0,0,0,0,0,0,0,0,0,0,
				0,0,1.454441e-06,0,0,0,0,0,0,0,0,0,0,0,0,0.0001962,0,0,0,0,0,
				0,0,0,0,0,0,0,0.0001962,0,0,0,0,0,0,0,0,0,0,0,0,0.0001962,
				0,0,0,0,0,0,0,0,0,0,0,0,3.92e-05,0,0,0,0,0,0,0,0,
				0,0,0,0,3.92e-05,0,0,0,0,0,0,0,0,0,0,0,0,3.92e-05};

		float32_t QqBuff[6*6] = {2.0943951e-05,0,0,0,0,0,
				0,2.0943951e-05,0,0,0,0,
				0,0,2.0943951e-05,0,0,0,
				0,0,0,1.454441e-06,0,0,
				0,0,0,0,1.454441e-06,0,
				0,0,0,0,0,1.454441e-06
				};

		arm_matrix_instance_f32 xMinus, Pminus, Pqminus;

		float32_t xMinusData[22];
		float32_t PMinusData[21*21];
		float32_t PqMinusBuff[6*6];

		arm_mat_init_f32(&xPlus, 22, 1, xData);
		arm_mat_init_f32(&P_plus, 21, 21, pData);
		arm_mat_init_f32(&Pq_plus, 6, 6, pqData);
		arm_mat_init_f32(&what, 3, 1, wHatData);
		arm_mat_init_f32(&aHatN, 3, 1, aHatNData);
		arm_mat_init_f32(&wMeas, 3, 1, wMeasData);
		arm_mat_init_f32(&aMeas, 3, 1, aMeasData);
		arm_mat_init_f32(&Q, 12, 12, QBuff);
		arm_mat_init_f32(&Qq, 6, 6, QqBuff);

		propogate(&xPlus, &P_plus, &what,
				  &aHatN, &wMeas, &aMeas, &Q,
				  dt, we, &xMinus, &Pminus,
				  xMinusData, PMinusData);

		arm_matrix_instance_f32 xMinusTrue, PMinusTrue, PqMinusTrue;

		float32_t xMinusTrueData[22*1] = {0.70761615,
				-0.00047431877,
				0.70659655,
				-0.00072436279,
				35.347893,
				-117.80683,
				672.4657,
				-0.020870822,
				0.011182608,
				-53.563698,
				1.4684781e-05,
				0.00015568511,
				-8.2018505e-06,
				-3.5961464e-05,
				-3.8672479e-09,
				-2.2649335e-10,
				-7.3725063e-12,
				-5.6151904e-13,
				1.1026901e-12,
				-0.00090790371,
				-6.181286e-12,
				5.4666089e-12,
				};

		float32_t PMinusDataTrue[21*21] = {0.00013784051,2.7258877e-05,0.00022864944,-7.5251902e-09,7.7438862e-08,-1.1973858e-06,-0.0012480185,0.010518491,2.7114581e-06,-5.2193474e-05,-9.9310892e-06,-8.4609768e-05,2.480845e-12,2.4600154e-11,-1.5151399e-12,6.3482597e-10,3.4307388e-09,4.0895053e-08,6.262408e-11,2.103522e-13,3.6046292e-15,
				2.725888e-05,1.466913e-05,4.8711652e-05,-1.7595396e-09,1.6470322e-08,-2.7044618e-07,-0.00036169135,0.0022383104,6.2744624e-07,-1.000458e-05,-6.8275449e-06,-1.8044409e-05,5.2516411e-13,4.1916813e-12,-7.2350155e-12,-3.5126944e-09,6.5446514e-08,8.9165919e-09,1.3257578e-11,2.3992893e-14,1.1473195e-13,
				0.00022864946,4.8711652e-05,0.00041280201,-1.3292808e-08,1.3711185e-07,-2.1321307e-06,-0.0022170506,0.018688232,4.8260426e-06,-8.4295214e-05,-1.7857643e-05,-0.00015410171,4.3811309e-12,4.1733689e-11,-2.9740672e-12,-2.9558455e-08,7.4894402e-09,1.1793077e-07,1.1059344e-10,3.3675309e-13,9.4671018e-15,
				-7.5251902e-09,-1.7595396e-09,-1.3292809e-08,6.3231851e-06,-5.2807281e-12,-6.1342649e-07,1.2280816e-07,-6.6821866e-07,6.1258004e-07,1.8617586e-09,3.0647945e-10,4.1111865e-09,-4.5953931e-12,-6.4172495e-13,-2.0417712e-09,6.7300961e-13,1.6873713e-12,-1.1295653e-12,-1.1615887e-10,-7.8493713e-15,1.3115497e-11,
				7.7438848e-08,1.6470322e-08,1.3711184e-07,-5.2807277e-12,7.6213009e-06,-1.904183e-09,-8.4970867e-07,7.1705895e-06,2.9707004e-09,-2.2572706e-08,-4.7489106e-09,-3.991903e-08,7.135604e-13,-2.4961242e-09,5.0214515e-13,-8.2748686e-12,-8.0691909e-14,9.2080805e-12,1.8964391e-11,-2.2661286e-11,-6.0837087e-15,
				-1.1973858e-06,-2.7044615e-07,-2.1321307e-06,-6.1342649e-07,-1.904183e-09,0.43808925,0.00013561752,-0.00035137171,-0.48220116,4.6948441e-07,1.0800529e-07,8.4400392e-07,-0.00025064097,-6.7641068e-08,2.1176627e-08,1.4636516e-10,-1.1580601e-11,-2.4936481e-10,-0.0066415225,-1.0598961e-09,-4.6577769e-10,
				-0.0012480185,-0.00036169132,-0.0022170506,1.2280816e-07,-8.4970884e-07,0.00013561752,0.018988244,-0.11052235,-0.00016343448,0.00040055945,0.00011497466,0.00071406801,-7.6555338e-08,-1.150134e-07,-0.00027979049,1.4324561e-07,-6.7431785e-08,-2.3752605e-07,-1.8649126e-06,-2.3153786e-09,3.9744978e-06,
				0.010518491,0.0022383102,0.01868823,-6.682186e-07,7.1705899e-06,-0.00035137171,-0.11052234,0.93233049,0.00050270453,-0.0033761812,-0.00071234693,-0.006022932,1.5443199e-07,-0.00027978941,7.1968607e-08,-1.2102672e-06,1.014928e-07,2.0168295e-06,3.8898497e-06,-4.8095199e-06,-1.6274316e-09,
				2.7114586e-06,6.2744618e-07,4.8260417e-06,6.1258004e-07,2.9707008e-09,-0.48220116,-0.00016343448,0.00050270447,0.52847856,-1.0111419e-06,-2.4013181e-07,-1.8140566e-06,0.00027698002,1.5534673e-07,-7.7344943e-08,-3.336795e-10,1.1449021e-10,5.7234384e-10,0.0072440477,3.9961421e-09,2.2735496e-09,
				-5.2193485e-05,-1.0004581e-05,-8.4295207e-05,1.8617585e-09,-2.2572708e-08,4.6948452e-07,0.00040055939,-0.0033761812,-1.0111421e-06,3.5053821e-05,4.3915556e-06,3.7734633e-05,-6.2695335e-13,1.8452492e-12,8.0428925e-13,-2.4868218e-08,-2.0904261e-09,-1.9373537e-08,-1.5829092e-11,1.1416667e-14,-2.3362854e-15,
				-9.9310882e-06,-6.8275453e-06,-1.7857641e-05,3.0647909e-10,-4.7489124e-09,1.0800535e-07,0.00011497463,-0.0007123471,-2.4013195e-07,4.3915566e-06,1.4868663e-05,8.0287782e-06,-1.2276806e-13,-2.03756e-12,-1.6128047e-11,1.5789176e-09,4.6248744e-08,-4.1988066e-09,-3.1002958e-12,-1.2199182e-14,5.7526642e-14,
				-8.4609768e-05,-1.8044409e-05,-0.00015410171,4.1111861e-09,-3.9919037e-08,8.4400386e-07,0.00071406795,-0.0060229315,-1.8140571e-06,3.7734641e-05,8.02878e-06,8.0693295e-05,-1.0885723e-12,-9.1784523e-13,1.4449125e-12,1.3867409e-08,-4.0680024e-09,1.1219086e-08,-2.7482232e-11,-5.0750331e-15,-5.0097344e-15,
				2.4808445e-12,5.2516395e-13,4.38113e-12,-4.5953931e-12,7.135604e-13,-0.00025064097,-7.6555338e-08,1.5443199e-07,0.00027698002,-6.269534e-13,-1.2276809e-13,-1.0885725e-12,0.00017879029,-1.5615333e-13,-3.6538528e-15,-2.6805946e-16,-1.7598921e-16,3.2156282e-17,-4.0649663e-08,-2.5879586e-16,2.5161664e-16,
				2.4600154e-11,4.1916813e-12,4.1733682e-11,-6.4172495e-13,-2.4961242e-09,-6.7641068e-08,-1.1501339e-07,-0.00027978941,1.5534673e-07,1.845249e-12,-2.0375594e-12,-9.1784609e-13,-1.5615333e-13,0.0001787919,-1.7128086e-18,1.5893668e-16,5.7724258e-15,1.8820217e-15,-3.9423352e-12,-1.3591324e-17,3.5206988e-20,
				-1.5151397e-12,-7.2350173e-12,-2.9740667e-12,-2.0417712e-09,5.0214504e-13,2.1176627e-08,-0.00027979049,7.1968593e-08,-7.7344943e-08,8.0429093e-13,-1.6128045e-11,1.4449108e-12,-3.6538528e-15,-1.7128092e-18,0.0001787919,1.4076089e-16,4.7271259e-14,-3.5312037e-15,-9.2247655e-14,-1.2320093e-20,-1.0943589e-17,
				6.3484573e-10,-3.5126906e-09,-2.9558421e-08,6.730095e-13,-8.2748695e-12,1.4636482e-10,1.432455e-07,-1.2102666e-06,-3.3367895e-10,-2.4868221e-08,1.578915e-09,1.3867405e-08,-2.6805901e-16,1.5893337e-16,1.4075954e-16,9.9999925e-05,4.1744538e-13,-2.6092271e-12,-6.7674756e-15,3.3850737e-18,-9.4079945e-19,
				3.4307388e-09,6.5446514e-08,7.4894349e-09,1.6873727e-12,-8.0688074e-14,-1.1580755e-11,-6.7431621e-08,1.0149294e-07,1.1449065e-10,-2.0904383e-09,4.6248751e-08,-4.0679953e-09,-1.7598924e-16,5.7724279e-15,4.7271252e-14,4.1744088e-13,9.9999677e-05,2.4252794e-11,-4.4403923e-15,6.8497305e-17,-4.2274385e-16,
				4.0895042e-08,8.9165875e-09,1.1793077e-07,-1.1295657e-12,9.2080666e-12,-2.493645e-10,-2.3752597e-07,2.0168288e-06,5.7234317e-10,-1.937352e-08,-4.198812e-09,1.1219076e-08,3.2155726e-17,1.8820145e-15,-3.5312071e-15,-2.6092292e-12,2.4252819e-11,9.9999939e-05,8.1039925e-16,2.5326795e-17,4.0579909e-17,
				6.2624073e-11,1.3257576e-11,1.1059342e-10,-1.1615887e-10,1.8964391e-11,-0.0066415225,-1.8649126e-06,3.8898497e-06,0.0072440477,-1.5829095e-11,-3.1002969e-12,-2.748223e-11,-4.0649663e-08,-3.9423352e-12,-9.2247668e-14,-6.7674972e-15,-4.4403949e-15,8.1038691e-16,9.8973731e-05,-6.5337064e-15,6.3524545e-15,
				2.1035223e-13,2.3992896e-14,3.3675314e-13,-7.8493713e-15,-2.2661286e-11,-1.0598961e-09,-2.3153786e-09,-4.8095199e-06,3.9961421e-09,1.141666e-14,-1.2199178e-14,-5.0750322e-15,-2.5879586e-16,-1.3591324e-17,-1.2320088e-20,3.3850754e-18,6.8497278e-17,2.5326785e-17,-6.5337064e-15,9.9999997e-05,1.6021638e-22,
				3.6046232e-15,1.1473194e-13,9.467084e-15,1.3115497e-11,-6.0837087e-15,-4.6577769e-10,3.9744978e-06,-1.6274316e-09,2.2735496e-09,-2.3362986e-15,5.7526642e-14,-5.0097162e-15,2.5161664e-16,3.5206991e-20,-1.0943587e-17,-9.4080928e-19,-4.2274382e-16,4.0579863e-17,6.3524545e-15,1.602164e-22,9.9999997e-05,
				};

		float32_t PqMinusDataTrue[6*6] = {0.00013303052,2.7752209e-05,0.00023099425,-4.9641159e-05,-1.0396414e-05,-8.5875421e-05,
				2.775221e-05,8.4098911e-06,4.9143207e-05,-1.0288332e-05,-3.4604291e-06,-1.8272052e-05,
				0.00023099424,4.9143207e-05,0.00041162447,-8.5624859e-05,-1.8410265e-05,-0.00015335807,
				-4.964113e-05,-1.0288326e-05,-8.5624815e-05,3.2983997e-05,4.6994792e-06,3.8811035e-05,
				-1.0396408e-05,-3.4604277e-06,-1.8410257e-05,4.6994783e-06,1.2108899e-05,8.3463765e-06,
				-8.5875421e-05,-1.8272051e-05,-0.00015335805,3.8811049e-05,8.3463738e-06,8.0060119e-05,
				};

		arm_mat_init_f32(&xMinusTrue, 22, 1, xMinusTrueData);
		arm_mat_init_f32(&PMinusTrue, 21, 21, PMinusDataTrue);
		arm_mat_init_f32(&PqMinusTrue, 6, 6, PqMinusDataTrue);

		bool test1 = areMatricesEqual(&xMinus, &xMinusTrue);
		bool test2 = areMatricesEqual(&Pminus, &PMinusTrue);
		bool test3 = areMatricesEqual(&Pqminus, &PqMinusTrue);

		bool test = (test1 && test2) & test3;
		return test;
}

bool test_right_divide(void) {

	float32_t BData[21*3] = {-7.5251902e-09,7.7438862e-08,-1.1973858e-06,
			-1.7595396e-09,1.6470322e-08,-2.7044618e-07,
			-1.3292808e-08,1.3711185e-07,-2.1321307e-06,
			6.3231851e-06,-5.2807281e-12,-6.1342649e-07,
			-5.2807277e-12,7.6213009e-06,-1.904183e-09,
			-6.1342649e-07,-1.904183e-09,0.43808925,
			1.2280816e-07,-8.4970884e-07,0.00013561752,
			-6.682186e-07,7.1705899e-06,-0.00035137171,
			6.1258004e-07,2.9707008e-09,-0.48220116,
			1.8617585e-09,-2.2572708e-08,4.6948452e-07,
			3.0647909e-10,-4.7489124e-09,1.0800535e-07,
			4.1111861e-09,-3.9919037e-08,8.4400386e-07,
			-4.5953931e-12,7.135604e-13,-0.00025064097,
			-6.4172495e-13,-2.4961242e-09,-6.7641068e-08,
			-2.0417712e-09,5.0214504e-13,2.1176627e-08,
			6.730095e-13,-8.2748695e-12,1.4636482e-10,
			1.6873727e-12,-8.0688074e-14,-1.1580755e-11,
			-1.1295657e-12,9.2080666e-12,-2.493645e-10,
			-1.1615887e-10,1.8964391e-11,-0.0066415225,
			-7.8493713e-15,-2.2661286e-11,-1.0598961e-09,
			1.3115497e-11,-6.0837087e-15,-4.6577769e-10
			};

	float32_t AData[3*3] = {1.9823185e-05,-5.2807281e-12,-6.1342649e-07,
			-5.2807277e-12,2.41213e-05,-1.904183e-09,
			-6.1342649e-07,-1.904183e-09,2.4380894
			};

	float32_t xRealData[21*3] = {-0.00037962996,0.0032103935,-4.9120951e-07,
			-8.8764951e-05,0.00068281242,-1.1094725e-07,
			-0.00067059428,0.0056842649,-8.7467316e-07,
			0.31897926,-1.4910522e-07,-1.713457e-07,
			-1.8223969e-07,0.31595731,-5.3429305e-10,
			-0.025384553,-6.4762811e-05,0.17968546,
			0.00619689,-0.03522649,5.5626042e-05,
			-0.033713322,0.29727212,-0.00014412589,
			0.024781972,0.00010754916,-0.19777828,
			9.3923947e-05,-0.00093579985,1.9258536e-07,
			1.5461957e-05,-0.00019687631,4.4302912e-08,
			0.00020740308,-0.001654929,3.4622519e-07,
			-3.413023e-06,2.1466015e-08,-0.0001028022,
			-3.3258534e-08,-0.00010348216,-2.7743564e-08,
			-0.00010299888,2.0795632e-08,8.659832e-09,
			3.3952389e-08,-3.4305239e-07,6.0040868e-11,
			8.5121023e-08,-3.3450784e-09,-4.7285162e-12,
			-5.6985112e-08,3.8174008e-07,-1.0229269e-10,
			-9.015578e-05,5.7114624e-07,-0.0027240685,
			-4.0967202e-10,-9.394721e-07,-4.3472487e-10,
			6.6161817e-07,-2.5208335e-10,-1.9087562e-10,
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

	float32_t xMinusData[22*1] = {7.09482610e-01, -2.81610852e-03,  7.04711735e-01, -2.83376710e-03,
        3.53478851e+01, -1.17806824e+02,  7.40749878e+02,  6.58162981e-02,
       -1.07167095e-01, -8.43544159e+01,  9.17429861e-05,  7.56012596e-05,
        1.40027958e-04,  1.06075327e-04,  3.02589171e-08, -5.37356160e-09,
       -6.20963974e-08,  2.02654905e-06, -8.68885945e-06,  3.70289013e-03,
        7.06479042e-10,  4.21090163e-10};

	float32_t PMinusData[21*21] = {8.06914759e-04, -8.95311096e-05, -4.57487564e-04,  5.13139327e-08,
       -3.00852548e-07, -9.46677574e-06,  6.20250357e-03, -2.96700764e-02,
        5.04852578e-05, -2.30749356e-04,  2.66296665e-05,  1.28825821e-04,
        1.49416216e-10,  3.30352022e-12,  6.62644072e-12, -2.65578901e-08,
       -6.02014580e-08, -4.62000202e-07,  5.00805530e-09,  2.92279818e-13,
       -7.79093180e-14, -8.95311096e-05,  2.52460486e-05,  5.18726520e-05,
       -6.55390142e-09,  3.40413493e-08,  1.22500001e-06, -9.77969496e-04,
        3.35943396e-03, -6.53472307e-06,  2.54064562e-05, -8.93000106e-06,
       -1.46275270e-05, -1.73182961e-11,  6.11959105e-13, -1.11052053e-11,
        4.21465396e-09,  4.60322298e-07,  5.43422907e-08, -5.80591797e-10,
        1.52003755e-14,  4.60971322e-13, -4.57487593e-04,  5.18726483e-05,
        2.70019169e-04, -2.94393629e-08,  1.72491951e-07,  5.48961771e-06,
       -3.56822158e-03,  1.70800574e-02, -2.93420526e-05,  1.30370099e-04,
       -1.54866757e-05, -7.68420869e-05, -8.57256349e-11,  7.99283296e-12,
       -4.03343678e-12,  1.79348163e-08,  4.43751382e-08,  6.44404281e-07,
       -2.87343482e-09,  3.53983309e-13,  5.40202953e-14,  5.13139362e-08,
       -6.55390098e-09, -2.94393665e-08,  4.30622049e-06, -2.22813452e-11,
       -7.48883679e-07,  6.97267240e-07, -2.07157677e-06,  4.92294248e-07,
       -1.30809994e-08,  9.50025059e-10,  7.40268513e-09, -3.79989998e-11,
       -1.21104266e-12, -3.34604766e-09, -4.02315551e-12,  3.32959632e-11,
       -8.71989980e-12, -1.27083666e-09, -3.48507844e-14,  7.29024757e-11,
       -3.00852605e-07,  3.40413564e-08,  1.72491994e-07, -2.22813452e-11,
        5.21340189e-06, -1.36887106e-08, -2.59233639e-06,  1.24327080e-05,
       -6.86138257e-09,  7.52122418e-08, -8.89591689e-09, -4.23634887e-08,
        2.56706592e-12, -4.09240020e-09,  7.13298778e-13,  2.43731788e-11,
        2.48834070e-12,  3.21409600e-11,  1.06913950e-10, -1.07462816e-10,
       -2.48089852e-14, -9.46677574e-06,  1.22500035e-06,  5.48961771e-06,
       -7.48883792e-07, -1.36887115e-08,  2.06816483e+00,  2.11887341e-03,
       -2.58845091e-03, -1.48030496e+00,  2.67417772e-06, -3.81792773e-07,
       -1.53507278e-06, -3.59758269e-04, -3.04388010e-07,  1.87294759e-07,
        2.26507882e-10,  5.08926146e-09,  7.27062233e-09, -1.30191809e-02,
       -1.64710965e-08, -1.15155014e-08,  6.20250311e-03, -9.77969263e-04,
       -3.56822112e-03,  6.97267240e-07, -2.59233661e-06,  2.11887364e-03,
        7.73226544e-02, -2.46598333e-01, -1.14807289e-03, -1.63532316e-03,
        2.44020091e-04,  9.28269466e-04, -3.11424628e-07, -1.14003832e-07,
       -3.00995365e-04, -4.17332586e-07, -9.44790770e-08, -1.76474157e-06,
       -1.36705048e-05, -4.00631128e-09,  1.00915413e-05, -2.96700709e-02,
        3.35943419e-03,  1.70800574e-02, -2.07157655e-06,  1.24327071e-05,
       -2.58845091e-03, -2.46598318e-01,  1.18182969e+00,  2.01179995e-04,
        7.82229099e-03, -9.26909386e-04, -4.44514072e-03,  4.24274106e-07,
       -3.00994900e-04,  3.56557308e-08,  2.01263447e-06,  1.02051843e-06,
        8.69302676e-06,  1.82247823e-05, -1.18376020e-05, -1.75866344e-09,
        5.04852578e-05, -6.53472262e-06, -2.93420562e-05,  4.92294248e-07,
       -6.86137858e-09, -1.48030496e+00, -1.14807277e-03,  2.01179937e-04,
        1.05585134e+00, -1.42589415e-05,  2.00694262e-06,  8.22838683e-06,
        2.35094936e-04,  5.46228648e-07, -4.10077575e-07, -2.06056772e-09,
       -2.20889884e-08, -3.96275261e-08,  9.25208814e-03,  3.43241346e-08,
        2.42506744e-08, -2.30749283e-04,  2.54064435e-05,  1.30370056e-04,
       -1.30809994e-08,  7.52122205e-08,  2.67417749e-06, -1.63532258e-03,
        7.82228727e-03, -1.42589388e-05,  8.06320386e-05, -8.22609672e-06,
       -4.02269907e-05, -4.23941750e-11,  1.50482283e-11, -2.05318163e-12,
       -2.60655786e-09,  2.37533637e-08,  1.63628329e-07, -1.41980605e-09,
        2.98606544e-13,  2.59981212e-14,  2.66296593e-05, -8.93000015e-06,
       -1.54866702e-05,  9.50025503e-10, -8.89591778e-09, -3.81792603e-07,
        2.44020179e-04, -9.26909503e-04,  2.00694240e-06, -8.22609945e-06,
        1.20907662e-05,  4.79277378e-06,  4.85886938e-12, -9.60757285e-15,
       -2.08124629e-11, -9.83168325e-10,  3.21933669e-07, -1.97569250e-08,
        1.62738906e-10, -8.75218551e-16,  3.26231338e-13,  1.28825806e-04,
       -1.46275279e-05, -7.68420796e-05,  7.40268202e-09, -4.23634923e-08,
       -1.53507278e-06,  9.28269292e-04, -4.44514165e-03,  8.22838774e-06,
       -4.02269870e-05,  4.79277605e-06,  3.22932319e-05,  2.42005929e-11,
        2.18949459e-11,  1.24563912e-12, -4.63350114e-09, -1.61280536e-08,
        2.82436872e-07,  8.10300382e-10,  4.62868296e-13, -1.81875050e-14,
        1.49416229e-10, -1.73182909e-11, -8.57256280e-11, -3.79990067e-11,
        2.56706592e-12, -3.59758269e-04, -3.11424628e-07,  4.24274106e-07,
        2.35094936e-04, -4.23941923e-11,  4.85886678e-12,  2.42005894e-11,
        9.99825425e-05, -4.31669327e-12,  5.32696337e-13, -1.33677706e-14,
        6.30911068e-14,  9.99347089e-15, -5.77173296e-07, -8.76268798e-14,
       -5.09239054e-14,  3.30352130e-12,  6.11958401e-13,  7.99283296e-12,
       -1.21104266e-12, -4.09240020e-09, -3.04388010e-07, -1.14003832e-07,
       -3.00994900e-04,  5.46228648e-07,  1.50482231e-11, -9.60721032e-15,
        2.18949459e-11, -4.31669327e-12,  9.99999975e-05,  1.36189756e-16,
       -2.00587465e-14,  1.14769568e-14, -5.82413675e-13, -1.43639586e-10,
       -1.87169379e-15, -1.32753786e-17,  6.62643768e-12, -1.11052044e-11,
       -4.03343288e-12, -3.34604766e-09,  7.13298886e-13,  1.87294759e-07,
       -3.00995365e-04,  3.56557273e-08, -4.10077575e-07, -2.05318293e-12,
       -2.08124611e-11,  1.24563847e-12,  5.32696337e-13,  1.36189769e-16,
        9.99999975e-05,  1.32009428e-15,  3.43823331e-13, -3.15156430e-14,
        1.81246285e-11,  2.88735830e-18,  1.04997474e-15, -2.65579398e-08,
        4.21465662e-09,  1.79348447e-08, -4.02315377e-12,  2.43731649e-11,
        2.26508853e-10, -4.17332899e-07,  2.01263424e-06, -2.06057171e-09,
       -2.60655542e-09, -9.83168102e-10, -4.63351313e-09, -1.33677757e-14,
       -2.00587515e-14,  1.32009344e-15,  2.17991808e-04, -5.24335853e-11,
       -4.62574562e-10, -4.47939998e-13, -8.33201905e-16, -5.37796174e-17,
       -6.02012733e-08,  4.60322298e-07,  4.43749997e-08,  3.32959459e-11,
        2.48842028e-12,  5.08926012e-09, -9.44793044e-08,  1.02052832e-06,
       -2.20889778e-08,  2.37533886e-08,  3.21933669e-07, -1.61280127e-08,
        6.30910593e-14,  1.14768170e-14,  3.43823331e-13, -5.24336650e-11,
        2.17980967e-04,  1.08204246e-09,  2.11332275e-12,  5.75289531e-16,
       -1.14917367e-14, -4.62001367e-07,  5.43422907e-08,  6.44404281e-07,
       -8.71972806e-12,  3.21410225e-11,  7.27062854e-09, -1.76474907e-06,
        8.69307223e-06, -3.96275830e-08,  1.63628499e-07, -1.97569445e-08,
        2.82436758e-07,  9.99348275e-15, -5.82413566e-13, -3.15156870e-14,
       -4.62574756e-10,  1.08204667e-09,  2.17975670e-04,  3.36740103e-13,
       -2.50809522e-14,  1.17753991e-15,  5.00805486e-09, -5.80591741e-10,
       -2.87343438e-09, -1.27083710e-09,  1.06913950e-10, -1.30191809e-02,
       -1.36705048e-05,  1.82247823e-05,  9.25208814e-03, -1.41980627e-09,
        1.62738850e-10,  8.10300493e-10, -5.77173296e-07, -1.43639586e-10,
        1.81246216e-11, -4.47940350e-13,  2.11332449e-12,  3.36727797e-13,
        8.08812765e-05, -2.92797232e-12, -1.70223005e-12,  2.92279466e-13,
        1.52005382e-14,  3.53983553e-13, -3.48507844e-14, -1.07462816e-10,
       -1.64710965e-08, -4.00631128e-09, -1.18376020e-05,  3.43241346e-08,
        2.98606815e-13, -8.75221410e-16,  4.62868567e-13, -8.76268662e-14,
       -1.87169357e-15,  2.88735830e-18, -8.33201746e-16,  5.75294243e-16,
       -2.50809624e-14, -2.92797232e-12,  9.99999975e-05, -2.74046333e-19,
       -7.79091418e-14,  4.60971268e-13,  5.40201124e-14,  7.29024757e-11,
       -2.48089852e-14, -1.15155014e-08,  1.00915413e-05, -1.75866344e-09,
        2.42506744e-08,  2.59981296e-14,  3.26231338e-13, -1.81874796e-14,
       -5.09239122e-14, -1.32753786e-17,  1.04997474e-15, -5.37796571e-17,
       -1.14917358e-14,  1.17753663e-15, -1.70222983e-12, -2.74046333e-19,
        9.99999975e-05};

	float32_t HData[3*21] = {0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
			0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
			0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
			};

	float32_t RData[3*3] = {1.35e-05,0,0,
			0,1.65e-05,0,
			0,0,2
			};

	float32_t llaMeasData[3] = {35.347897, -117.80686,  741.9873};

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

	arm_matrix_instance_f32 xPlusTrue, PPlusTrue, PqPlusTrue;

	float32_t xPlusTrueData[22*1] = {7.0948261e-01, -2.8161085e-03,  7.0471174e-01, -2.8337671e-03,
									 3.5347889e+01, -1.1780683e+02,  7.4137897e+02,  6.6465817e-02,
									 -1.0797763e-01, -8.4804688e+01,  9.1742986e-05,  7.5601260e-05,
									 1.4002796e-04, -3.3535762e-06, -5.5139687e-08,  4.9401947e-08,
									 -6.2096397e-08,  2.0265491e-06, -8.6888595e-06, -2.5719823e-04,
									 -4.1148378e-09, -3.0337879e-09};

	float32_t PPlusDataTrue[21*21] = {8.06910452e-04, -8.95306148e-05, -4.57485119e-04,  3.89027193e-08,
       -2.28617637e-07, -4.65310950e-06,  6.20247098e-03, -2.96699051e-02,
        4.70397354e-05, -2.30748265e-04,  2.66295392e-05,  1.28825224e-04,
       -6.87436108e-10, -5.41038846e-11,  1.67143261e-11, -2.65575402e-08,
       -6.02015078e-08, -4.61999690e-07, -2.52766377e-08, -1.23490411e-12,
       -3.15126343e-13, -8.95306148e-05,  2.52459940e-05,  5.18723755e-05,
       -4.96872188e-09,  2.58679975e-08,  6.02112209e-07, -9.77965770e-04,
        3.35941440e-03, -6.08887649e-06,  2.54063325e-05, -8.92998742e-06,
       -1.46274569e-05,  9.09711265e-11,  7.11904190e-12, -1.23942341e-11,
        4.21461444e-09,  4.60322298e-07,  5.43422338e-08,  3.33827699e-09,
        1.88621891e-13,  4.91308979e-13, -4.57485148e-04,  5.18723718e-05,
        2.70017743e-04, -2.23189058e-08,  1.31076504e-07,  2.69826251e-06,
       -3.56820272e-03,  1.70799587e-02, -2.73440601e-05,  1.30369488e-04,
       -1.54866029e-05, -7.68417376e-05,  3.99552280e-10,  4.09116491e-11,
       -9.82364884e-12,  1.79346156e-08,  4.43751631e-08,  6.44404054e-07,
        1.46881982e-08,  1.22983524e-12,  1.90279487e-13,  3.89027193e-08,
       -4.96872099e-09, -2.23189094e-08,  3.26481290e-06, -1.28383320e-11,
       -2.79130830e-07,  5.28935175e-07, -1.57094246e-06,  1.66639424e-07,
       -9.91708404e-09,  7.20212667e-10,  5.61218716e-09, -7.90192078e-11,
       -9.63832855e-13, -2.53682009e-09, -3.05015323e-12,  2.52444454e-11,
       -6.61005669e-12, -2.78052736e-09, -2.88049512e-14,  5.52702814e-11,
       -2.28617694e-07,  2.58680046e-08,  1.31076547e-07, -1.28383346e-11,
        3.96166070e-06, -5.11422504e-09, -1.96990868e-06,  9.44759904e-06,
       -8.99879815e-09,  5.71537271e-08, -6.76000189e-09, -3.21919842e-08,
        1.03073312e-12, -3.10981330e-09,  5.39332197e-13,  1.85211603e-11,
        1.89093316e-12,  2.44239073e-11,  4.79511257e-11, -8.16609697e-11,
       -1.88124493e-14, -4.65310950e-06,  6.02112436e-07,  2.69826251e-06,
       -2.79130916e-07, -5.11422549e-09,  1.01675558e+00,  1.04169874e-03,
       -1.27257896e-03, -7.27750778e-01,  1.31443790e-06, -1.87680911e-07,
       -7.54535904e-07, -1.76865142e-04, -1.49645189e-07,  9.20090741e-08,
        1.11280672e-10,  2.50268317e-09,  3.57422891e-09, -6.40051812e-03,
       -8.09759015e-09, -5.65976865e-09,  6.20247005e-03, -9.77965537e-04,
       -3.56820226e-03,  5.28935232e-07, -1.96990914e-06,  1.04169885e-03,
        7.73212165e-02, -2.46595412e-01, -3.77076969e-04, -1.63531501e-03,
        2.44019189e-04,  9.28264926e-04, -1.24042955e-07, -1.14333830e-07,
       -3.00995336e-04, -4.17329630e-07, -9.44827363e-08, -1.76474123e-06,
       -6.88940918e-06, -4.01056077e-09,  1.00915440e-05, -2.96699014e-02,
        3.35941464e-03,  1.70799606e-02, -1.57094223e-06,  9.44759813e-06,
       -1.27257884e-03, -2.46595398e-01,  1.18182063e+00, -7.40661169e-04,
        7.82224815e-03, -9.26904439e-04, -4.44511650e-03,  1.95357828e-07,
       -3.00992746e-04,  3.53851313e-08,  2.01262014e-06,  1.02052411e-06,
        8.69301221e-06,  9.94060611e-06, -1.18375510e-05, -1.75749304e-09,
        4.70397354e-05, -6.08887558e-06, -2.73440637e-05,  1.66639396e-07,
       -8.99879549e-09, -7.27750719e-01, -3.77076969e-04, -7.40661169e-04,
        5.17204821e-01, -1.32856712e-05,  1.86800105e-06,  7.66969697e-06,
        1.04187769e-04,  4.35467143e-07, -3.41884316e-07, -1.97808414e-09,
       -2.02375414e-08, -3.69817990e-08,  4.51472914e-03,  2.83306498e-08,
        2.00595665e-08, -2.30748206e-04,  2.54063198e-05,  1.30369430e-04,
       -9.91708404e-09,  5.71537129e-08,  1.31443778e-06, -1.63531443e-03,
        7.82224443e-03, -1.32856694e-05,  8.06317694e-05, -8.22606489e-06,
       -4.02268379e-05,  1.94008934e-10,  2.94228947e-11, -4.63674888e-12,
       -2.60664534e-09,  2.37533762e-08,  1.63628201e-07,  7.13534121e-09,
        6.81642405e-13,  8.72063381e-14,  2.66295337e-05, -8.92998560e-06,
       -1.54865957e-05,  7.20213000e-10, -6.76000234e-09, -1.87680826e-07,
        2.44019277e-04, -9.26904555e-04,  1.86800094e-06, -8.22606762e-06,
        1.20907625e-05,  4.79275604e-06, -2.88979153e-11, -1.71475139e-12,
       -2.06160870e-11, -9.83158110e-10,  3.21933669e-07, -1.97569108e-08,
       -1.05887477e-09, -4.64462148e-14,  3.21251272e-13,  1.28825195e-04,
       -1.46274588e-05, -7.68417376e-05,  5.61218538e-09, -3.21919913e-08,
       -7.54535904e-07,  9.28264752e-04, -4.44511743e-03,  7.66969788e-06,
       -4.02268342e-05,  4.79275832e-06,  3.22931446e-05, -1.11503855e-10,
        1.37962194e-11,  2.70870132e-12, -4.63345184e-09, -1.61280607e-08,
        2.82436929e-07, -4.10068290e-09,  2.47005649e-13, -5.28871651e-14,
       -6.87436108e-10,  9.09711403e-11,  3.99552280e-10, -7.90192287e-11,
        1.03073322e-12, -1.76865142e-04, -1.24042955e-07,  1.95357828e-07,
        1.04187769e-04,  1.94008934e-10, -2.88979327e-11, -1.11503883e-10,
        9.99507247e-05, -3.12342582e-11,  1.70760697e-11,  6.63783214e-15,
        5.13342298e-13,  6.52900151e-13, -1.72849309e-06, -1.54420162e-12,
       -1.06884250e-12, -5.41038915e-11,  7.11904190e-12,  4.09116560e-11,
       -9.63832746e-13, -3.10981330e-09, -1.49645189e-07, -1.14333830e-07,
       -3.00992746e-04,  4.35467172e-07,  2.94228947e-11, -1.71475095e-12,
        1.37962211e-11, -3.12342617e-11,  9.99999975e-05,  1.40456138e-14,
       -1.54483868e-14,  1.23291117e-14, -5.75812564e-13, -1.11774834e-09,
       -2.33580042e-14, -8.74366012e-16,  1.67143226e-11, -1.23942332e-11,
       -9.82364797e-12, -2.53682009e-09,  5.39332251e-13,  9.20090741e-08,
       -3.00995336e-04,  3.53851277e-08, -3.41884316e-07, -4.63675019e-12,
       -2.06160852e-11,  2.70870154e-12,  1.70760697e-11,  1.40456155e-14,
        9.99999975e-05,  5.52875557e-16,  3.49845858e-13, -3.34897549e-14,
        6.16827200e-10,  7.57596327e-16,  1.52790361e-14, -2.65575917e-08,
        4.21461666e-09,  1.79346440e-08, -3.05015171e-12,  1.85211499e-11,
        1.11281144e-10, -4.17329943e-07,  2.01261992e-06, -1.97808769e-09,
       -2.60664290e-09, -9.83157888e-10, -4.63346383e-09,  6.63791345e-15,
       -1.54483936e-14,  5.52875028e-16,  2.17991808e-04, -5.24335818e-11,
       -4.62574590e-10,  2.76047550e-13, -7.11666971e-16, -3.66396608e-17,
       -6.02013159e-08,  4.60322298e-07,  4.43750281e-08,  2.52444333e-11,
        1.89099366e-12,  2.50268228e-09, -9.44829637e-08,  1.02053400e-06,
       -2.02375308e-08,  2.37534010e-08,  3.21933669e-07, -1.61280198e-08,
        5.13342082e-13,  1.23289872e-14,  3.49845858e-13, -5.24336616e-11,
        2.17980967e-04,  1.08204246e-09,  1.84071230e-11,  6.08281570e-16,
       -1.16136484e-14, -4.62000884e-07,  5.43422338e-08,  6.44404054e-07,
       -6.60992745e-12,  2.44239507e-11,  3.57423202e-09, -1.76474873e-06,
        8.69305768e-06, -3.69818558e-08,  1.63628371e-07, -1.97569303e-08,
        2.82436815e-07,  6.52900802e-13, -5.75812455e-13, -3.34897651e-14,
       -4.62574784e-10,  1.08204667e-09,  2.17975670e-04,  2.36027448e-11,
       -2.48924620e-14,  1.23385098e-15, -2.52766377e-08,  3.33827743e-09,
        1.46882000e-08, -2.78052825e-09,  4.79511292e-11, -6.40051812e-03,
       -6.88941054e-06,  9.94060611e-06,  4.51472867e-03,  7.13534121e-09,
       -1.05887543e-09, -4.10068202e-09, -1.72849309e-06, -1.11774834e-09,
        6.16827200e-10,  2.76044080e-13,  1.84071300e-11,  2.36027135e-11,
        3.92165239e-05, -5.56394410e-11, -3.85397998e-11, -1.23490465e-12,
        1.88622108e-13,  1.22983568e-12, -2.88049512e-14, -8.16609697e-11,
       -8.09759015e-09, -4.01056077e-09, -1.18375510e-05,  2.83306498e-08,
        6.81642731e-13, -4.64462216e-14,  2.47005920e-13, -1.54420162e-12,
       -2.33580042e-14,  7.57596221e-16, -7.11666759e-16,  6.08285806e-16,
       -2.48924738e-14, -5.56394410e-11,  9.99999975e-05, -4.68651815e-17,
       -3.15126126e-13,  4.91308979e-13,  1.90279324e-13,  5.52702814e-11,
       -1.88124493e-14, -5.65976865e-09,  1.00915440e-05, -1.75749304e-09,
        2.00595665e-08,  8.72063516e-14,  3.21251272e-13, -5.28871584e-14,
       -1.06884250e-12, -8.74366012e-16,  1.52790361e-14, -3.66396972e-17,
       -1.16136476e-14,  1.23384844e-15, -3.85397998e-11, -4.68651815e-17,
        9.99999975e-05};

	arm_mat_init_f32(&xPlusTrue, 22, 1, xPlusTrueData);
	arm_mat_init_f32(&PPlusTrue, 21, 21, PPlusDataTrue);

	bool test1 = areMatricesEqual(&xPlus, &xPlusTrue);
	bool test2 = areMatricesEqual(&P_plus, &PPlusTrue);

	bool test = (test1 && test2);
	return test;
}

void test_update_mag(void) {

	arm_matrix_instance_f32 xMinus, P_minus, Pq_minus, Hq, Rq, R, magI, magMeas;

	float32_t xMinusData[22*1] = {7.0772630e-01, -5.7503802e-04,  7.0648605e-01, -7.8321825e-04,
        3.5347885e+01, -1.1780682e+02,  6.7762335e+02, -2.2120645e-02,
        9.5796110e-03, -5.6314919e+01,  1.9137944e-05,  1.1633299e-04,
        8.6476030e-06,  1.0607533e-04,  3.0547060e-08, -5.3322804e-09,
        2.8534330e-07,  8.8562723e-07, -1.0654069e-06,  3.7028901e-03,
        7.1882750e-10,  4.1987527e-10};

	float32_t PMinusData[21*21] = {4.45392099e-04, -5.07897348e-05, -2.47623509e-04,  1.09806422e-08,
       -6.67718183e-08,  1.43066859e-06,  2.49544275e-03, -1.18200071e-02,
       -2.78712878e-06, -1.61678909e-04,  1.82542935e-05,  8.88742361e-05,
        1.10891463e-10,  8.07097038e-14,  3.57144436e-12, -2.81321846e-08,
       -1.15048797e-08, -8.28858973e-08,  3.71804032e-09,  6.47403680e-14,
       -1.85658390e-14, -5.07897348e-05,  2.28860226e-05,  2.95264144e-05,
       -1.87720284e-09,  7.93434562e-09, -2.17003304e-07, -5.83106477e-04,
        1.40624715e-03,  4.37761429e-07,  1.80739517e-05, -1.11758654e-05,
       -1.06262305e-05, -1.67472633e-11,  6.30127461e-13, -7.39300635e-12,
        7.19993887e-09,  1.56080688e-07,  1.02364464e-08, -5.61232838e-10,
        1.00569738e-14,  1.67405667e-13, -2.47623509e-04,  2.95264163e-05,
        1.48769366e-04, -6.26257179e-09,  3.79962621e-08, -8.23811149e-07,
       -1.43177563e-03,  6.78870827e-03,  1.60300647e-06,  8.91804084e-05,
       -1.07591195e-05, -5.46435440e-05, -6.38942579e-11,  6.80202379e-12,
       -2.20019922e-12,  3.44222286e-08,  9.24775367e-09,  1.31618435e-07,
       -2.14217466e-09,  1.83758743e-13,  1.32301659e-14,  1.09806431e-08,
       -1.87720328e-09, -6.26257091e-09,  4.30622049e-06, -1.96150409e-12,
       -3.76830599e-07,  1.59540875e-07, -3.22223713e-07,  3.50603216e-07,
       -3.38335249e-09,  7.10300430e-11,  1.92638017e-09, -3.67895714e-11,
       -4.02103942e-13, -1.27368738e-09, -1.33941718e-12,  4.38973987e-12,
       -9.03453581e-13, -1.21785715e-09, -7.64513473e-15,  1.27670374e-11,
       -6.67718183e-08,  7.93434474e-09,  3.79962586e-08, -1.96150388e-12,
        5.21328138e-06, -1.28492084e-09, -4.35302951e-07,  2.07382186e-06,
        1.55537327e-09,  1.93685228e-08, -2.31562125e-09, -1.08228191e-08,
        3.71916853e-13, -1.56297131e-09,  2.82529669e-13,  7.75592611e-12,
        1.03992478e-13,  3.02964533e-12,  1.42098729e-11, -2.11403690e-11,
       -5.77924439e-15,  1.43066848e-06, -2.17003318e-07, -8.23811149e-07,
       -3.76830684e-07, -1.28492117e-09,  4.44505960e-01,  1.60130890e-04,
       -3.20905907e-04, -4.64372933e-01, -5.33052742e-07,  8.45006625e-08,
        3.10508995e-07, -1.79229493e-04, -5.20497743e-08,  1.10336105e-08,
       -1.97667757e-10,  1.49624008e-10, -5.40824330e-11, -6.05619000e-03,
       -1.30522448e-09, -7.93839605e-10,  2.49544298e-03, -5.83106477e-04,
       -1.43177551e-03,  1.59540875e-07, -4.35303036e-07,  1.60130890e-04,
        3.26340012e-02, -7.39569068e-02, -1.76091475e-04, -7.90779304e-04,
        1.73326422e-04,  4.51297936e-04, -4.54204212e-08, -8.48218136e-08,
       -2.10998769e-04, -3.13778287e-07, -9.08932520e-08, -2.76370798e-07,
       -1.99432952e-06, -2.64092992e-09,  4.60097590e-06, -1.18200090e-02,
        1.40624726e-03,  6.78870827e-03, -3.22223713e-07,  2.07382232e-06,
       -3.20905878e-04, -7.39568993e-02,  3.51736486e-01,  3.70700494e-04,
        3.73848993e-03, -4.48698032e-04, -2.13890034e-03,  9.18976681e-08,
       -2.10998653e-04,  4.72456492e-08,  1.48803167e-06,  1.42770162e-07,
        1.33178162e-06,  3.72720410e-06, -5.51295579e-06, -1.82265947e-09,
       -2.78712923e-06,  4.37761543e-07,  1.60300647e-06,  3.50603244e-07,
        1.55537350e-09, -4.64372963e-01, -1.76091475e-04,  3.70700494e-04,
        4.82698321e-01,  1.00059151e-06, -1.70692104e-07, -5.79270022e-07,
        1.66794183e-04,  1.10766749e-07, -5.92067764e-08,  3.85980498e-10,
       -1.13462566e-10,  1.37622330e-10,  6.26066327e-03,  4.99385866e-09,
        3.37843198e-09, -1.61678880e-04,  1.80739444e-05,  8.91803866e-05,
       -3.38335227e-09,  1.93685246e-08, -5.33052685e-07, -7.90779188e-04,
        3.73848947e-03,  1.00059128e-06,  8.06264652e-05, -7.61972160e-06,
       -3.81116624e-05, -4.20198425e-11,  1.02933261e-11, -1.48145027e-12,
       -9.08381637e-09,  6.60019150e-09,  3.87145391e-08, -1.40726775e-09,
        9.49577195e-14,  7.82582210e-15,  1.82542917e-05, -1.11758654e-05,
       -1.07591177e-05,  7.10299805e-11, -2.31562081e-09,  8.45006625e-08,
        1.73326451e-04, -4.48698032e-04, -1.70692218e-07, -7.61972251e-06,
        1.76754038e-05,  4.62498065e-06,  6.54106543e-12, -1.22276755e-13,
       -1.31389214e-11, -3.09745629e-09,  7.85456962e-08, -4.79965090e-09,
        2.19063170e-10, -2.51564698e-15,  6.57795487e-14,  8.88742361e-05,
       -1.06262296e-05, -5.46435440e-05,  1.92638061e-09, -1.08228182e-08,
        3.10509023e-07,  4.51297936e-04, -2.13890034e-03, -5.79270136e-07,
       -3.81116552e-05,  4.62498292e-06,  3.46215784e-05,  2.45400870e-11,
        1.38439399e-11,  9.12530359e-13, -1.54478936e-08, -4.63876981e-09,
        6.93592952e-08,  8.21675394e-10,  1.18692016e-13, -5.54741062e-15,
        1.10891463e-10, -1.67472668e-11, -6.38942510e-11, -3.67895783e-11,
        3.71916853e-13, -1.79229493e-04, -4.54204212e-08,  9.18976681e-08,
        1.66794183e-04, -4.20198563e-11,  6.54106500e-12,  2.45400835e-11,
        9.99825425e-05, -4.31669327e-12,  5.32698018e-13, -1.53892165e-14,
        1.39283226e-14, -3.61639616e-15, -5.77173296e-07, -8.76269069e-14,
       -5.09239562e-14,  8.07182148e-14,  6.30127298e-13,  6.80202032e-12,
       -4.02103942e-13, -1.56297131e-09, -5.20497707e-08, -8.48218136e-08,
       -2.10998653e-04,  1.10766749e-07,  1.02933200e-11, -1.22275901e-13,
        1.38439451e-11, -4.31669327e-12,  9.99999975e-05,  1.36396630e-16,
        1.19203116e-15,  2.53219102e-15, -6.15146429e-14, -1.43639586e-10,
       -1.87087259e-15, -1.32857746e-17,  3.57144609e-12, -7.39300721e-12,
       -2.20019966e-12, -1.27368738e-09,  2.82529696e-13,  1.10336114e-08,
       -2.10998769e-04,  4.72456492e-08, -5.92067764e-08, -1.48145114e-12,
       -1.31389188e-11,  9.12530034e-13,  5.32698018e-13,  1.36396630e-16,
        9.99999975e-05, -5.12742660e-16,  4.45851276e-14, -3.44068341e-15,
        1.81246840e-11,  2.89999638e-18,  1.04965498e-15, -2.81322023e-08,
        7.19994242e-09,  3.44222499e-08, -1.33941838e-12,  7.75593045e-12,
       -1.97667785e-10, -3.13778372e-07,  1.48803269e-06,  3.85980303e-10,
       -9.08381814e-09, -3.09745807e-09, -1.54479061e-08, -1.53892215e-14,
        1.19202904e-15, -5.12742713e-16,  1.82711883e-04,  1.10751252e-12,
       -6.54870949e-12, -5.15559709e-13, -1.12620177e-17,  2.98258437e-18,
       -1.15049028e-08,  1.56080702e-07,  9.24776078e-09,  4.38974031e-12,
        1.03992695e-13,  1.49623966e-10, -9.08932947e-08,  1.42770489e-07,
       -1.13462129e-10,  6.60020039e-09,  7.85456749e-08, -4.63876892e-09,
        1.39283023e-14,  2.53219610e-15,  4.45851276e-14,  1.10751425e-12,
        1.82711214e-04,  5.90312244e-11,  4.66295622e-13,  6.22803672e-17,
       -6.17564416e-16, -8.28857267e-08,  1.02364393e-08,  1.31618364e-07,
       -9.03450871e-13,  3.02960803e-12, -5.40823948e-11, -2.76369889e-07,
        1.33177446e-06,  1.37622122e-10,  3.87144929e-08, -4.79964157e-09,
        6.93593449e-08, -3.61635635e-15, -6.15146700e-14, -3.44067875e-15,
       -6.54865225e-12,  5.90311619e-11,  1.82711286e-04, -1.20783239e-13,
       -1.33076298e-15,  6.08198111e-17,  3.71804032e-09, -5.61232838e-10,
       -2.14217444e-09, -1.21785759e-09,  1.42098729e-11, -6.05619000e-03,
       -1.99432952e-06,  3.72720410e-06,  6.26066327e-03, -1.40726808e-09,
        2.19063170e-10,  8.21675228e-10, -5.77173296e-07, -1.43639586e-10,
        1.81246771e-11, -5.15559600e-13,  4.66295893e-13, -1.20784432e-13,
        8.08812765e-05, -2.92797319e-12, -1.70223167e-12,  6.47407272e-14,
        1.00569552e-14,  1.83758634e-13, -7.64513473e-15, -2.11403690e-11,
       -1.30522448e-09, -2.64092992e-09, -5.51295579e-06,  4.99385866e-09,
        9.49576721e-14, -2.51563809e-15,  1.18692111e-13, -8.76268933e-14,
       -1.87087238e-15,  2.89999638e-18, -1.12619218e-17,  6.22802614e-17,
       -1.33076309e-15, -2.92797319e-12,  9.99999975e-05, -2.74632390e-19,
       -1.85658661e-14,  1.67405667e-13,  1.32301727e-14,  1.27670383e-11,
       -5.77924397e-15, -7.93839605e-10,  4.60097590e-06, -1.82265947e-09,
        3.37843198e-09,  7.82582549e-15,  6.57795419e-14, -5.54741190e-15,
       -5.09239630e-14, -1.32857746e-17,  1.04965498e-15,  2.98258313e-18,
       -6.17564310e-16,  6.08198376e-17, -1.70223145e-12, -2.74632390e-19,
        9.99999975e-05};

	float32_t RData[3*3] = {1.35e-05, 0.00e+00, 0.00e+00,
							0.00e+00, 1.65e-05, 0.00e+00,
							0.00e+00, 0.00e+00, 2.00e+00};


	float32_t magIData[3*1] = {0.4891, 0.104, 0.866};

	float32_t magMeasData[3*1] = {-0.86539793,  0.10228254,  0.49052635};

	arm_mat_init_f32(&xMinus, 22, 1, xMinusData);
	arm_mat_init_f32(&P_minus, 21, 21, PMinusData);
	arm_mat_init_f32(&R, 3, 3, RData);
	arm_mat_init_f32(&magI, 3, 1, magIData);
	arm_mat_init_f32(&magMeas, 3, 1, magMeasData);

	arm_matrix_instance_f32 xPlus, Pplus, PqPlus;
	float32_t xPlusData[22*1], PPlusData[21*21], PqPlusData[6*6];

	update_mag(&xMinus, &P_minus, &R,
			   &magI, &magMeas, &xPlus,
			   &Pplus, xPlusData, PPlusData);

	float32_t xPlusDataTrue[22*1] = {7.07718968e-01, -6.07609458e-04,  7.06493437e-01, -7.93119951e-04,
        3.53478851e+01, -1.17806824e+02,  6.77623352e+02, -2.24415530e-02,
        8.95910896e-03, -5.63149185e+01,  3.53375908e-05,  1.05384126e-04,
        3.91423819e-05,  1.06075327e-04,  3.04987537e-08, -5.34176703e-09,
        1.58331190e-07,  1.10461804e-06, -1.67394455e-06,  3.70289013e-03,
        7.17267856e-10,  4.20110585e-10};

	float32_t PPlusDataTrue[21*21] = {4.44657373e-04, -5.11465223e-05, -2.48837896e-04,  1.09946132e-08,
       -6.67926159e-08,  1.43321427e-06,  2.50218227e-03, -1.18327569e-02,
       -2.79203937e-06, -1.61304532e-04,  1.84476667e-05,  8.94909754e-05,
        1.11093544e-10, -9.18256465e-13,  3.70049000e-12, -3.08589563e-08,
       -1.42004684e-08, -9.51201784e-08,  3.72479536e-09,  3.26219188e-14,
       -2.14144938e-14, -5.11465187e-05,  1.90361970e-05,  2.96376311e-05,
       -1.74581782e-09,  7.93089772e-09, -2.06347863e-07, -5.17570006e-04,
        1.40641991e-03,  4.13530387e-07,  1.82733038e-05, -9.11929874e-06,
       -1.06829284e-05, -1.59387677e-11,  6.11991360e-13, -5.80012080e-12,
        6.66034783e-09,  1.20700548e-07,  1.17264518e-08, -5.34187139e-10,
        1.01303997e-14,  1.29623173e-13, -2.48837896e-04,  2.96376311e-05,
        1.46628532e-04, -6.26323482e-09,  3.79606107e-08, -8.21399226e-07,
       -1.43250718e-03,  6.76642219e-03,  1.59905539e-06,  8.97958525e-05,
       -1.08136737e-05, -5.35562613e-05, -6.36956737e-11,  5.06184981e-12,
       -2.27863626e-12,  2.97656886e-08,  1.12878560e-08,  1.09980213e-07,
       -2.13554041e-09,  1.27683481e-13,  1.54611211e-14,  1.09946132e-08,
       -1.74581816e-09, -6.26323438e-09,  4.30622049e-06, -1.96133409e-12,
       -3.76830968e-07,  1.57302566e-07, -3.22196968e-07,  3.50604040e-07,
       -3.39106609e-09,  8.37330205e-13,  1.92672456e-09, -3.67895991e-11,
       -4.02100770e-13, -1.27368738e-09, -1.31415722e-12,  5.59568581e-12,
       -9.22664614e-13, -1.21785804e-09, -7.64505511e-15,  1.27670383e-11,
       -6.67926230e-08,  7.93089683e-09,  3.79606107e-08, -1.96133387e-12,
        5.21328138e-06, -1.28486577e-09, -4.35224820e-07,  2.07344942e-06,
        1.55527380e-09,  1.93790903e-08, -2.31369834e-09, -1.08047145e-08,
        3.71921298e-13, -1.56297131e-09,  2.82530564e-13,  7.67730931e-12,
        8.93279184e-14,  2.66983419e-12,  1.42100212e-11, -2.11403707e-11,
       -5.77925921e-15,  1.43321404e-06, -2.06347892e-07, -8.21399169e-07,
       -3.76831053e-07, -1.28486610e-09,  4.44505960e-01,  1.59948017e-04,
       -3.20878025e-04, -4.64372933e-01, -5.34394871e-07,  7.88023797e-08,
        3.09284616e-07, -1.79229493e-04, -5.20497707e-08,  1.10336060e-08,
       -1.90229776e-10,  2.46252879e-10, -3.07302413e-11, -6.05619000e-03,
       -1.30522437e-09, -7.93839494e-10,  2.50218203e-03, -5.17570006e-04,
       -1.43250707e-03,  1.57302580e-07, -4.35224905e-07,  1.59948017e-04,
        3.15177292e-02, -7.39477351e-02, -1.75676454e-04, -7.94510532e-04,
        1.38314383e-04,  4.51673171e-04, -4.54343052e-08, -8.48205559e-08,
       -2.10998798e-04, -3.02053820e-07,  5.10837594e-07, -2.90000202e-07,
       -1.99479405e-06, -2.64090061e-09,  4.60097635e-06, -1.18327579e-02,
        1.40642002e-03,  6.76642265e-03, -3.22196968e-07,  2.07344988e-06,
       -3.20877996e-04, -7.39477277e-02,  3.51504236e-01,  3.70653113e-04,
        3.74495587e-03, -4.48739709e-04, -2.12758174e-03,  9.18999419e-08,
       -2.10998667e-04,  4.72452406e-08,  1.43935722e-06,  1.54968731e-07,
        1.10662529e-06,  3.72728027e-06, -5.51295625e-06, -1.82264592e-09,
       -2.79203960e-06,  4.13530501e-07,  1.59905539e-06,  3.50604068e-07,
        1.55527402e-09, -4.64372963e-01, -1.75676454e-04,  3.70653113e-04,
        4.82698321e-01,  1.00319789e-06, -1.57737375e-07, -5.77264814e-07,
        1.66794183e-04,  1.10766742e-07, -5.92067657e-08,  3.72418790e-10,
       -3.33932382e-10,  1.00014400e-10,  6.26066327e-03,  4.99385866e-09,
        3.37843176e-09, -1.61304488e-04,  1.82732983e-05,  8.97958307e-05,
       -3.39106587e-09,  1.93790921e-08, -5.34394815e-07, -7.94510415e-04,
        3.74495541e-03,  1.00319767e-06,  8.04356168e-05, -7.72762723e-06,
       -3.84242230e-05, -4.21262053e-11,  1.08001212e-11, -1.55451173e-12,
       -7.69812569e-09,  8.13640888e-09,  4.49130866e-08, -1.41082324e-09,
        1.11248975e-13,  9.45114873e-15,  1.84476685e-05, -9.11929783e-06,
       -1.08136728e-05,  8.37319797e-13, -2.31369768e-09,  7.88023797e-08,
        1.38314412e-04, -4.48739680e-04, -1.57737503e-07, -7.72762723e-06,
        1.65767797e-05,  4.65280300e-06,  6.10866566e-12, -1.08634639e-13,
       -1.39897616e-11, -2.79859647e-09,  9.74433547e-08, -5.54655788e-09,
        2.04598588e-10, -2.42750304e-15,  8.59602645e-14,  8.94909826e-05,
       -1.06829266e-05, -5.35562576e-05,  1.92672500e-09, -1.08047127e-08,
        3.09284616e-07,  4.51673113e-04, -2.12758174e-03, -5.77264871e-07,
       -3.84242157e-05,  4.65280527e-06,  3.40693623e-05,  2.44392770e-11,
        1.47277381e-11,  9.52455888e-13, -1.30829525e-08, -5.67687186e-09,
        8.03490181e-08,  8.18307533e-10,  1.47171554e-13, -6.68257792e-15,
        1.11093544e-10, -1.59387711e-11, -6.36956668e-11, -3.67896061e-11,
        3.71921298e-13, -1.79229493e-04, -4.54343052e-08,  9.18999419e-08,
        1.66794183e-04, -4.21262192e-11,  6.10866436e-12,  2.44392718e-11,
        9.99825425e-05, -4.31669327e-12,  5.32697692e-13, -1.47908182e-14,
        2.12527529e-14, -1.68720948e-15, -5.77173296e-07, -8.76269001e-14,
       -5.09239494e-14, -9.18246815e-13,  6.11990818e-13,  5.06184938e-12,
       -4.02100770e-13, -1.56297131e-09, -5.20497672e-08, -8.48205559e-08,
       -2.10998667e-04,  1.10766742e-07,  1.08001143e-11, -1.08633595e-13,
        1.47277451e-11, -4.31669327e-12,  9.99999975e-05,  1.36377731e-16,
       -2.61511660e-15,  3.19453584e-15, -7.90929889e-14, -1.43639586e-10,
       -1.87091833e-15, -1.32850244e-17,  3.70049130e-12, -5.80012167e-12,
       -2.27863713e-12, -1.27368738e-09,  2.82530591e-13,  1.10336069e-08,
       -2.10998798e-04,  4.72452406e-08, -5.92067657e-08, -1.55451216e-12,
       -1.39897599e-11,  9.52455562e-13,  5.32697692e-13,  1.36377731e-16,
        9.99999975e-05, -3.60349309e-16,  5.92392888e-14, -4.38472704e-15,
        1.81246736e-11,  2.89911543e-18,  1.04967065e-15, -3.08589847e-08,
        6.66035138e-09,  2.97657134e-08, -1.31415777e-12,  7.67731104e-12,
       -1.90229846e-10, -3.02053735e-07,  1.43935767e-06,  3.72418485e-10,
       -7.69812747e-09, -2.79859869e-09, -1.30829632e-08, -1.47908165e-14,
       -2.61512211e-15, -3.60349177e-16,  1.82711869e-04, -1.62620958e-12,
       -5.35400682e-11, -4.95561057e-13, -1.33809368e-16,  1.77592369e-19,
       -1.42004897e-08,  1.20700562e-07,  1.12878711e-08,  5.59568537e-12,
        8.93282030e-14,  2.46252796e-10,  5.10837367e-07,  1.54968859e-07,
       -3.33931827e-10,  8.13641154e-09,  9.74433334e-08, -5.67687009e-09,
        2.12527207e-14,  3.19454347e-15,  5.92392888e-14, -1.62621002e-12,
        1.82710894e-04,  8.30093355e-11,  7.11315609e-13,  8.96604359e-17,
       -9.65330522e-16, -9.51200221e-08,  1.17264403e-08,  1.09980171e-07,
       -9.22661253e-13,  2.66978475e-12, -3.07304043e-11, -2.89998667e-07,
        1.10661449e-06,  1.00013525e-10,  4.49130297e-08, -5.54654500e-09,
        8.03490678e-08, -1.68714002e-15, -7.90930160e-14, -4.38472026e-15,
       -5.35399919e-11,  8.30092453e-11,  1.82711068e-04, -5.63352262e-14,
       -1.89726762e-15,  8.69546090e-17,  3.72479536e-09, -5.34187139e-10,
       -2.13554019e-09, -1.21785848e-09,  1.42100212e-11, -6.05619000e-03,
       -1.99479405e-06,  3.72728027e-06,  6.26066327e-03, -1.41082357e-09,
        2.04598588e-10,  8.18307422e-10, -5.77173296e-07, -1.43639586e-10,
        1.81246667e-11, -4.95561166e-13,  7.11315935e-13, -5.63377436e-14,
        8.08812765e-05, -2.92797297e-12, -1.70223145e-12,  3.26222204e-14,
        1.01303565e-14,  1.27683495e-13, -7.64505511e-15, -2.11403707e-11,
       -1.30522437e-09, -2.64090061e-09, -5.51295625e-06,  4.99385866e-09,
        1.11248948e-13, -2.42749965e-15,  1.47171622e-13, -8.76268865e-14,
       -1.87091812e-15,  2.89911543e-18, -1.33809209e-16,  8.96602837e-17,
       -1.89726762e-15, -2.92797297e-12,  9.99999975e-05, -2.74601785e-19,
       -2.14145260e-14,  1.29623173e-13,  1.54611329e-14,  1.27670392e-11,
       -5.77925879e-15, -7.93839494e-10,  4.60097635e-06, -1.82264592e-09,
        3.37843176e-09,  9.45114619e-15,  8.59602577e-14, -6.68257792e-15,
       -5.09239562e-14, -1.32850244e-17,  1.04967065e-15,  1.77594023e-19,
       -9.65330310e-16,  8.69546686e-17, -1.70223124e-12, -2.74601785e-19,
        9.99999975e-05};

	arm_matrix_instance_f32 xPlusTrue, PplusTrue;

	arm_mat_init_f32(&xPlusTrue, 22, 1, xPlusDataTrue);
	arm_mat_init_f32(&PplusTrue, 21, 21, PPlusDataTrue);

	bool test1 = areMatricesEqual(&xPlusTrue, &xPlus);
	bool test3 = areMatricesEqual(&PplusTrue, &Pplus);

	bool test = test1 && test3;
}

void test_update_baro(void) {

	arm_matrix_instance_f32 xMinus, P_minus, Hq, Rq, R, magI, magMeas;

	float32_t xMinusData[22*1] = {-8.3495724e-01, -3.4935978e-01, -4.1650939e-01, -8.5523278e-02,
							      3.5394650e+01, -1.1788577e+02,  4.6469664e+04,  1.7937307e+01,
							      -4.4092670e-02,  3.7876859e+02,  1.5291531e-05, -1.7716145e-04,
							      -2.8864868e-04,  8.8491691e-03,  1.8049498e-03,  1.2415712e-04,
							      4.2428655e-04, -1.6183165e-03, -5.1211531e-04,  1.5715049e-03,
							      1.1085608e-03,  4.4746863e-04};

	float32_t PMinusData[21*21] = {1.02079846e-03, -3.73842847e-03, -3.50543205e-03,  1.82524873e-05,
        1.09334669e-05,  1.11442947e+00,  1.92342952e-01, -2.00908333e-01,
       -8.30902085e-02, -1.91852723e-05,  2.45125175e-05,  3.85416679e-05,
        3.74752972e-06, -3.81368727e-06,  9.00041141e-06,  5.06533506e-05,
        2.04045384e-04,  1.70059575e-04,  1.48856225e-06,  4.61393392e-06,
        5.03924821e-06, -3.73842637e-03,  1.38408905e-02,  1.29699251e-02,
       -6.76267809e-05, -4.06053259e-05, -4.12973642e+00, -7.12426305e-01,
        7.43942320e-01,  3.07776481e-01,  6.26847032e-05, -9.21969404e-05,
       -1.41152617e-04, -1.38985788e-05,  1.41355886e-05, -3.33588323e-05,
       -2.11282400e-04, -7.58641341e-04, -6.26108667e-04, -5.51817584e-06,
       -1.71032825e-05, -1.86786438e-05, -3.50543018e-03,  1.29699241e-02,
        1.21656079e-02, -6.33290401e-05, -3.79472222e-05, -3.86671138e+00,
       -6.67330921e-01,  6.97052360e-01,  2.88285524e-01,  5.91122589e-05,
       -8.55058752e-05, -1.35107388e-04, -1.30055096e-05,  1.32367177e-05,
       -3.12339580e-05, -1.96545734e-04, -7.08889216e-04, -5.92893222e-04,
       -5.16531145e-06, -1.60092604e-05, -1.74859724e-05,  1.82524727e-05,
       -6.76267373e-05, -6.33289819e-05,  3.21093717e-06, -3.22456845e-06,
        3.48970294e-02,  4.67031123e-03, -5.41660096e-03, -1.83760177e-03,
       -1.82443316e-07,  1.95269962e-07,  2.03363243e-07, -6.84378256e-06,
        3.57580348e-07, -2.21909113e-06,  1.13193710e-06,  2.65206836e-06,
        3.15218335e-06,  1.69839177e-06, -1.12908947e-06,  2.04905518e-06,
        1.09334133e-05, -4.06051295e-05, -3.79470257e-05, -3.22456981e-06,
        1.62463948e-05,  5.79049923e-02,  5.41986490e-04,  6.85186684e-03,
       -1.67897157e-03, -2.80612426e-07, -1.33468916e-07, -2.45721083e-07,
        1.25984707e-06, -1.16526553e-06, -1.95409689e-06,  3.61693992e-06,
       -6.51025448e-06, -7.66407265e-06, -1.60778905e-06, -8.77113109e-07,
       -2.08932283e-06,  1.11442769e+00, -4.12973118e+00, -3.86670613e+00,
        3.48970331e-02,  5.79050593e-02,  7.16365527e+03,  2.45251602e+02,
       -1.97992645e+02, -1.78915802e+02, -1.25957197e-02,  1.06129777e-02,
        1.02559095e-02, -6.55746222e-01,  2.79283430e-03, -1.12771960e-02,
        9.15225968e-02,  1.69278130e-01,  9.86434370e-02,  3.63154523e-02,
       -3.11527345e-02,  1.22665241e-02,  1.92343190e-01, -7.12427437e-01,
       -6.67331874e-01,  4.67031449e-03,  5.41995978e-04,  2.45251801e+02,
        3.92354050e+01, -4.06830254e+01, -1.68146019e+01, -2.48434395e-03,
        2.64695100e-03,  3.47843440e-03, -1.99577142e-03, -2.89424846e-04,
        7.26329105e-04,  1.40353329e-02,  3.40655856e-02,  2.48597264e-02,
        5.78280480e-04,  2.47672852e-03,  3.65204061e-03, -2.00908154e-01,
        7.43941665e-01,  6.97052002e-01, -5.41660562e-03,  6.85185473e-03,
       -1.97992767e+02, -4.06829720e+01,  4.67135010e+01,  1.66180725e+01,
        2.66749715e-03, -3.30003514e-03, -4.60766489e-03, -3.03517445e-03,
        1.61681819e-04, -4.25582658e-03, -1.25370985e-02, -4.28001024e-02,
       -3.28884870e-02, -6.12530275e-04, -2.31392216e-03, -1.95809710e-03,
       -8.30901265e-02,  3.07776183e-01,  2.88285196e-01, -1.83760049e-03,
       -1.67897670e-03, -1.78915726e+02, -1.68145714e+01,  1.66180744e+01,
        8.41786766e+00,  1.06395583e-03, -1.09324849e-03, -1.40945974e-03,
        9.19215474e-03,  1.21027508e-04, -1.01240119e-03, -6.24099560e-03,
       -1.44284088e-02, -1.00793736e-02, -6.73796923e-04,  1.52626703e-03,
        1.22177182e-03, -1.91849595e-05,  6.26835754e-05,  5.91112184e-05,
       -1.82438853e-07, -2.80622373e-07, -1.25958854e-02, -2.48430134e-03,
        2.66744662e-03,  1.06393802e-03,  3.03829856e-05, -1.42254135e-06,
       -2.35813127e-06, -2.76576543e-07,  9.17094440e-07,  7.87664476e-07,
       -2.43111834e-04, -2.75944694e-06, -5.28876126e-06, -2.43067451e-08,
        6.43537561e-08,  4.72787391e-08,  2.45127321e-05, -9.21975370e-05,
       -8.55064500e-05,  1.95268868e-07, -1.33473435e-07,  1.06128594e-02,
        2.64700479e-03, -3.30003072e-03, -1.09323964e-03, -1.42251361e-06,
        9.89190994e-06,  6.46200897e-06,  7.28788550e-08,  1.99499084e-08,
        9.14921685e-08, -3.72881959e-06,  3.54231415e-05,  1.63207296e-05,
       -1.26663680e-09, -3.10205728e-08, -1.41058720e-08,  3.85419044e-05,
       -1.41154160e-04, -1.35108814e-04,  2.03369083e-07, -2.45720543e-07,
        1.02563091e-02,  3.47837014e-03, -4.60775662e-03, -1.40948431e-03,
       -2.35830271e-06,  6.46193894e-06,  1.66496520e-05,  3.84287979e-08,
        2.63607678e-08,  9.96239624e-09, -6.40073085e-06,  1.75550340e-05,
       -7.50322124e-06, -3.90255606e-09, -2.57776289e-08, -2.20806928e-08,
        3.74752699e-06, -1.38985688e-05, -1.30055005e-05, -6.84378256e-06,
        1.25984957e-06, -6.55746341e-01, -1.99577422e-03, -3.03517119e-03,
        9.19215567e-03, -2.76575804e-07,  7.28788123e-08,  3.84304819e-08,
        1.01653575e-04, -4.43229555e-06,  1.28094064e-06,  2.67544965e-06,
        7.60159128e-06, -4.73179654e-07, -1.04296387e-05,  1.02098065e-05,
       -8.38075539e-06, -3.81367363e-06,  1.41355395e-05,  1.32366740e-05,
        3.57579353e-07, -1.16526201e-06,  2.79282918e-03, -2.89422518e-04,
        1.61678501e-04,  1.21026664e-04,  9.17093757e-07,  1.99497112e-08,
        2.63635318e-08, -4.43229055e-06,  6.52256349e-05,  7.68161863e-06,
       -9.21249830e-06,  6.55887834e-06, -3.50928190e-06,  6.80926519e-07,
       -9.65532945e-06, -4.13432372e-06,  9.00037139e-06, -3.33586941e-05,
       -3.12338307e-05, -2.21909113e-06, -1.95409939e-06, -1.12772100e-02,
        7.26321363e-04, -4.25581867e-03, -1.01239863e-03,  7.87664135e-07,
        9.14924314e-08,  9.96466376e-09,  1.28093268e-06,  7.68161499e-06,
        8.61038934e-05, -8.17037562e-06,  2.38544908e-06,  9.10545441e-06,
       -2.97837659e-07,  3.02558419e-06,  2.89219270e-06,  5.06524229e-05,
       -2.11279053e-04, -1.96542445e-04,  1.13191106e-06,  3.61701632e-06,
        9.15209502e-02,  1.40350936e-02, -1.25367446e-02, -6.24091551e-03,
       -2.43111805e-04, -3.72891691e-06, -6.40029157e-06,  2.67546238e-06,
       -9.21250285e-06, -8.17037289e-06,  2.61307904e-03, -2.29368598e-05,
       -2.17219604e-05,  2.42931577e-07, -6.00153555e-07, -4.75783281e-07,
        2.04045471e-04, -7.58641341e-04, -7.08889391e-04,  2.65206768e-06,
       -6.51028404e-06,  1.69277772e-01,  3.40656005e-02, -4.28001657e-02,
       -1.44284079e-02, -2.75931961e-06,  3.54231815e-05,  1.75550740e-05,
        7.60158309e-06,  6.55885151e-06,  2.38549319e-06, -2.29375728e-05,
        4.88658017e-03,  9.18676596e-05, -1.25517317e-06, -7.19550781e-06,
       -5.98707675e-06,  1.70059109e-04, -6.26107736e-04, -5.92892407e-04,
        3.15219427e-06, -7.66407538e-06,  9.86439288e-02,  2.48595048e-02,
       -3.28885466e-02, -1.00793783e-02, -5.28891542e-06,  1.63207606e-05,
       -7.50356867e-06, -4.73191193e-07, -3.50927598e-06,  9.10543986e-06,
       -2.17224842e-05,  9.18679798e-05,  4.72018402e-03,  3.62505972e-07,
        4.86408112e-07,  2.14061879e-06,  1.48856793e-06, -5.51819812e-06,
       -5.16533373e-06,  1.69839245e-06, -1.60779098e-06,  3.63154672e-02,
        5.78282285e-04, -6.12532254e-04, -6.73797738e-04, -2.43056082e-08,
       -1.26630617e-09, -3.90346466e-09, -1.04296423e-05,  6.80926917e-07,
       -2.97839136e-07,  2.42920436e-07, -1.25517090e-06,  3.62504011e-07,
        2.63936840e-06, -1.89062632e-06,  2.34089339e-06,  4.61394393e-06,
       -1.71033171e-05, -1.60092932e-05, -1.12908867e-06, -8.77114246e-07,
       -3.11527215e-02,  2.47673085e-03, -2.31392705e-03,  1.52626541e-03,
        6.43561719e-08, -3.10190948e-08, -2.57816453e-08,  1.02098047e-05,
       -9.65532217e-06,  3.02557714e-06, -6.00181238e-07, -7.19552418e-06,
        4.86386512e-07, -1.89062553e-06,  7.99596528e-05,  1.24565604e-05,
        5.03926549e-06, -1.86787001e-05, -1.74860306e-05,  2.04905700e-06,
       -2.08932238e-06,  1.22665698e-02,  3.65204387e-03, -1.95810595e-03,
        1.22176856e-03,  4.72807393e-08, -1.41064316e-08, -2.20845369e-08,
       -8.38075630e-06, -4.13432781e-06,  2.89218769e-06, -4.75784873e-07,
       -5.98708357e-06,  2.14059583e-06,  2.34089293e-06,  1.24565613e-05,
        6.17994592e-05};

	float32_t Rb = 0.0025f;
	float32_t pressMeas = 0.00127722f;

	arm_mat_init_f32(&xMinus, 22, 1, xMinusData);
	arm_mat_init_f32(&P_minus, 21, 21, PMinusData);

	arm_matrix_instance_f32 xPlus, Pplus;
	float32_t xPlusData[22*1], PPlusData[21*21];

	update_baro(&xMinus, &P_minus, pressMeas, Rb, &xPlus, &Pplus, xPlusData, PPlusData);

	float32_t xPlusTrueData[22*1] = {-8.34957242e-01, -3.49359781e-01, -4.16509390e-01, -8.55232775e-02,
									 3.61659241e+01, -1.17796265e+02,  8.69715234e+04,  6.49743359e+03,
									 -6.71859717e+03, -2.39805859e+03,  1.52915309e-05, -1.77161448e-04,
									 -2.88648676e-04, -3.20741147e-01, -4.59914692e-02,  1.20071843e-01,
									 4.24286554e-04, -1.61831651e-03, -5.12115308e-04,  9.70714167e-02,
									 4.10126030e-01,  6.03560925e-01};

	float32_t PPlusTrueData[21*21] = {2.54049606e-04, -8.98437342e-04, -8.45208066e-04, -3.65060714e-07,
        8.77287675e-06,  1.36767954e-01,  3.59365642e-02, -3.87312174e-02,
       -1.60611793e-02, -9.28178724e-06,  1.39608237e-05,  2.46753825e-05,
        1.17033906e-05, -2.65993617e-06,  6.10500319e-06, -5.29651243e-06,
        6.82477548e-05,  7.09597953e-05, -8.16670990e-07, -5.25919313e-06,
       -9.51909351e-06, -8.98436294e-04,  3.32174031e-03,  3.11662117e-03,
        1.33145386e-06, -3.26026347e-05, -5.08538842e-01, -1.33106783e-01,
        1.43248409e-01,  5.95051311e-02,  2.60028100e-05, -5.31141195e-05,
       -8.97927530e-05, -4.33665882e-05,  9.86216583e-06, -2.26344218e-05,
       -4.04757884e-06, -2.55655381e-04, -2.59049237e-04,  3.02026456e-06,
        1.94661661e-05,  3.52445459e-05, -8.45206727e-04,  3.11661814e-03,
        2.93600094e-03,  1.26425493e-06, -3.04510868e-05, -4.74729538e-01,
       -1.24681354e-01,  1.34381294e-01,  5.57293333e-02,  2.47522694e-05,
       -4.88969308e-05, -8.69985233e-05, -4.06082436e-05,  9.23379685e-06,
       -2.11883835e-05, -2.42852911e-06, -2.37741391e-04, -2.49068078e-04,
        2.83266013e-06,  1.82454041e-05,  3.30239782e-05, -3.65084361e-07,
        1.33156300e-06,  1.26436225e-06,  2.75888124e-06, -3.27703015e-06,
        1.11582642e-02,  8.72581673e-04, -1.47875142e-03, -2.10058701e-04,
        5.80248596e-08, -6.09375235e-08, -1.33326353e-07, -6.65060497e-06,
        3.85594774e-07, -2.28939507e-06, -2.26590998e-07, -6.45256762e-07,
        7.45925149e-07,  1.64241806e-06, -1.36882045e-06,  1.69556165e-06,
        8.77285856e-06, -3.26025656e-05, -3.04510104e-05, -3.27703060e-06,
        1.62403066e-05,  5.51501252e-02,  1.01262514e-04,  7.30885146e-03,
       -1.49009633e-03, -2.52706258e-07, -1.63201619e-07, -2.84793686e-07,
        1.28226520e-06, -1.16201443e-06, -1.96225551e-06,  3.45928356e-06,
       -6.89290664e-06, -7.94331754e-06, -1.61428477e-06, -9.04933756e-07,
       -2.13034559e-06,  1.36765644e-01, -5.08530736e-01, -4.74722028e-01,
        1.11582708e-02,  5.51501438e-02,  5.91706348e+03,  4.58217964e+01,
        8.79528046e+00, -9.34487915e+01,  3.19627579e-05, -2.84122070e-03,
       -7.42464047e-03, -6.45601928e-01,  4.26395331e-03, -1.49690574e-02,
        2.01823413e-02, -3.87398899e-03, -2.77161784e-02,  3.33761126e-02,
       -4.37417105e-02, -6.29644794e-03,  3.59366089e-02, -1.33107007e-01,
       -1.24681532e-01,  8.72582314e-04,  1.01264297e-04,  4.58218346e+01,
        7.33058167e+00, -7.60104895e+00, -3.14157104e+00, -4.64164565e-04,
        4.94545442e-04,  6.49896334e-04, -3.72881681e-04, -5.40749461e-05,
        1.35704337e-04,  2.62230379e-03,  6.36467384e-03,  4.64468868e-03,
        1.08043547e-04,  4.62741766e-04,  6.82332262e-04, -3.87310535e-02,
        1.43247545e-01,  1.34380937e-01, -1.47875852e-03,  7.30884681e-03,
        8.79504013e+00, -7.60103893e+00,  1.24109831e+01,  2.44058347e+00,
        5.72784513e-04, -1.06821791e-03, -1.67476980e-03, -4.71793953e-03,
       -8.23511800e-05, -3.64341098e-03, -7.02992082e-04, -1.40771819e-02,
       -1.19276270e-02, -1.24944228e-04, -2.25630560e-04,  1.12117687e-03,
       -1.60611346e-02,  5.95048964e-02,  5.57290912e-02, -2.10059341e-04,
       -1.49009842e-03, -9.34487991e+01, -3.14156532e+00,  2.44059277e+00,
        2.55820704e+00,  1.98195674e-04, -1.70822022e-04, -1.97272631e-04,
        8.49665515e-03,  2.01668809e-05, -7.59285351e-04, -1.34987244e-03,
       -2.55701481e-03, -1.41609635e-03, -4.72274027e-04,  2.38937326e-03,
        2.49445834e-03, -9.28163263e-06,  2.60022571e-05,  2.47517710e-05,
        5.80253676e-08, -2.52716205e-07,  3.15899961e-05, -4.64156619e-04,
        5.72767225e-04,  1.98191206e-04,  3.02550725e-05, -1.28625561e-06,
       -2.17903425e-06, -3.79334438e-07,  9.02192596e-07,  8.25061591e-07,
       -2.42389186e-04, -1.00548493e-06, -4.00878889e-06,  5.46764412e-09,
        1.91875046e-07,  2.35314275e-07,  1.39608092e-05, -5.31138612e-05,
       -4.88967089e-05, -6.09439894e-08, -1.63207261e-07, -2.84162350e-03,
        4.94555454e-04, -1.06816518e-03, -1.70792744e-04, -1.28622275e-06,
        9.74669820e-06,  6.27118243e-06,  1.82366620e-07,  3.58277177e-08,
        5.16458485e-08, -4.49879644e-06,  3.35543082e-05,  1.49569278e-05,
       -3.29910357e-08, -1.66893585e-07, -2.14456350e-07,  2.46758591e-05,
       -8.97951613e-05, -8.70007643e-05, -1.33314543e-07, -2.84793117e-07,
       -7.42392614e-03,  6.49884343e-04, -1.67491171e-03, -1.97317320e-04,
       -2.17920615e-06,  6.27111967e-06,  1.63988916e-05,  1.82304234e-07,
        4.72254449e-08, -4.23987707e-08, -7.41254007e-06,  1.50992410e-05,
       -9.29536236e-06, -4.55908769e-08, -2.04325318e-07, -2.85356805e-07,
        1.17034087e-05, -4.33666646e-05, -4.06083127e-05, -6.65060452e-06,
        1.28226816e-06, -6.45601988e-01, -3.72882205e-04, -4.71794093e-03,
        8.49665422e-03, -3.79335575e-07,  1.82364502e-07,  1.82308781e-07,
        1.01571022e-04, -4.44426723e-06,  1.31098375e-06,  3.25599240e-06,
        9.01064413e-06,  5.55091901e-07, -1.04057190e-05,  1.03122511e-05,
       -8.22969650e-06, -2.65993026e-06,  9.86214491e-06,  9.23377866e-06,
        3.85593580e-07, -1.16201090e-06,  4.26393747e-03, -5.40745095e-05,
       -8.23528535e-05,  2.01666644e-05,  9.02191800e-07,  3.58270711e-08,
        4.72284256e-08, -4.44426178e-06,  6.52238959e-05,  7.68597511e-06,
       -9.12830910e-06,  6.76321588e-06, -3.36016433e-06,  6.84395275e-07,
       -9.64047285e-06, -4.11241763e-06,  6.10499046e-06, -2.26343800e-05,
       -2.11883489e-05, -2.28939439e-06, -1.96225824e-06, -1.49690351e-02,
        1.35702881e-04, -3.64340888e-03, -7.59285060e-04,  8.25061477e-07,
        5.16473442e-08, -4.23969126e-08,  1.31097545e-06,  7.68597147e-06,
        8.60929576e-05, -8.38165215e-06,  1.87265300e-06,  8.73123554e-06,
       -3.06542631e-07,  2.98830150e-06,  2.83721783e-06, -5.29656154e-06,
       -4.04742968e-06, -2.42826354e-06, -2.26594807e-07,  3.45935996e-06,
        2.01818570e-02,  2.62225885e-03, -7.02824676e-04, -1.34986686e-03,
       -2.42389156e-04, -4.49886466e-06, -7.41210215e-06,  3.25599444e-06,
       -9.12831456e-06, -8.38164851e-06,  2.60899635e-03, -3.28458918e-05,
       -2.89531836e-05,  7.47207594e-08, -1.32058688e-06, -1.53809265e-06,
        6.82476166e-05, -2.55654333e-04, -2.37740722e-04, -6.45261082e-07,
       -6.89294347e-06, -3.87455150e-03,  6.36467664e-03, -1.40771950e-02,
       -2.55698664e-03, -1.00532668e-06,  3.35543846e-05,  1.50992346e-05,
        9.01063504e-06,  6.76319087e-06,  1.87269131e-06, -3.28467795e-05,
        4.86252923e-03,  7.43162309e-05, -1.66344978e-06, -8.94412369e-06,
       -8.56548559e-06,  7.09600863e-05, -2.59051041e-04, -2.49069824e-04,
        7.45955731e-07, -7.94332209e-06, -2.77146511e-02,  4.64464771e-03,
       -1.19278450e-02, -1.41616270e-03, -4.00893214e-06,  1.49569987e-05,
       -9.29572707e-06,  5.55069732e-07, -3.36015864e-06,  8.73122099e-06,
       -2.89537656e-05,  7.43167184e-05,  4.70737601e-03,  6.45644036e-08,
       -7.89651494e-07,  2.59015138e-07, -8.16675197e-07,  3.02028275e-06,
        2.83267423e-06,  1.64241851e-06, -1.61428682e-06,  3.33761163e-02,
        1.08043881e-04, -1.24944068e-04, -4.72273852e-04,  5.46938450e-09,
       -3.29901511e-08, -4.55926852e-08, -1.04057226e-05,  6.84395673e-07,
       -3.06544223e-07,  7.47062217e-08, -1.66344876e-06,  6.45588685e-08,
        2.63243760e-06, -1.92031007e-06,  2.29712350e-06, -5.25920541e-06,
        1.94662261e-05,  1.82454551e-05, -1.36882011e-06, -9.04935405e-07,
       -4.37417179e-02,  4.62742173e-04, -2.25630734e-04,  2.38937396e-03,
        1.91879778e-07, -1.66889492e-07, -2.04332821e-07,  1.03122493e-05,
       -9.64046558e-06,  2.98829400e-06, -1.32062746e-06, -8.94414188e-06,
       -7.89685714e-07, -1.92030916e-06,  7.98325200e-05,  1.22690981e-05,
       -9.51910806e-06,  3.52446295e-05,  3.30240364e-05,  1.69556290e-06,
       -2.13034582e-06, -6.29643491e-03,  6.82332844e-04,  1.12117501e-03,
        2.49445857e-03,  2.35319675e-07, -2.14453024e-07, -2.85365758e-07,
       -8.22969741e-06, -4.11242127e-06,  2.83721215e-06, -1.53811322e-06,
       -8.56549377e-06,  2.58973756e-07,  2.29712305e-06,  1.22690990e-05,
        6.15230383e-05};

	arm_matrix_instance_f32 xPlusTrue, PplusTrue;
	arm_mat_init_f32(&xPlusTrue, 22, 1, xPlusTrueData);
	arm_mat_init_f32(&PplusTrue, 21, 21, PPlusTrueData);

	printMatrix(&xPlus);
	printMatrix(&xPlusTrue);

	printMatrix(&Pplus);
	printMatrix(&PplusTrue);


	bool test1 = areMatricesEqual(&xPlusTrue, &xPlus);
	bool test2 = areMatricesEqual(&PplusTrue, &Pplus);
}

void test_nearest_PSD(void) {

	arm_matrix_instance_f32 P;

	float32_t PData[21*21] = {0.0025420981,0.00056884775,0.0044895881,-1.8643639e-05,2.2661981e-05,-0.15808287,-0.14770506,0.39684299,0.11297939,-3.9111965e-05,0.00037699094,-0.00016086186,-0.00027748532,-5.3298896e-05,0.00018545915,3.4745105e-06,-3.4480383e-06,-2.0611285e-06,1.0788649e-09,-4.2548668e-06,-9.975749e-06,
			0.00056884775,0.00013492082,0.0010082235,-4.1284729e-06,5.0446179e-06,-0.035336785,-0.032503881,0.088887535,0.025101464,-8.833108e-06,8.2360049e-05,-3.7022928e-05,-6.1853141e-05,-1.190062e-05,4.1404906e-05,7.5420968e-07,-8.2554868e-07,-5.1530702e-07,2.7679792e-10,-9.4951412e-07,-2.2268439e-06,
			0.0044895881,0.0010082235,0.007961777,-3.3070079e-05,4.0170853e-05,-0.28013921,-0.26175976,0.70398289,0.2008445,-6.563466e-05,0.00066822238,-0.0002862504,-0.00049191422,-9.449388e-05,0.00032873536,6.0953403e-06,-6.128942e-06,-3.6629708e-06,1.9540101e-09,-7.5423545e-06,-1.7682958e-05,
			-1.8643621e-05,-4.1284702e-06,-3.3070053e-05,8.9184796e-06,8.0073496e-06,0.0019331971,0.0086544165,0.0020813155,-0.0015480893,3.9669057e-08,-3.0600088e-06,6.6289198e-07,-1.4828654e-05,2.6485437e-05,-2.7961698e-06,-9.5825783e-07,-1.8256821e-07,-1.1043047e-07,-3.0887929e-07,1.301584e-07,2.3472347e-07,
			2.2662007e-05,5.0446233e-06,4.01709e-05,8.0073787e-06,4.6720037e-05,-0.0012912743,0.0054481467,0.032609969,0.00089598761,-1.7433871e-07,3.5680007e-06,-1.0809998e-06,-1.4265719e-05,2.067161e-05,-3.8139609e-05,6.3878957e-07,-3.5127488e-08,-4.7955666e-07,-1.0792245e-08,-4.0350318e-07,-3.7224797e-07,
			-0.15808284,-0.035336774,-0.28013918,0.0019331973,-0.0012912803,16.809494,11.325293,-25.03236,-9.3817186,0.00151247,-0.024230015,0.0087118745,-0.039505888,-0.01019081,0.051102132,-0.00031894381,0.00028548413,0.0001402979,-9.2789314e-06,-0.00099551352,-0.0026343924,
			-0.14770496,-0.032503862,-0.26175961,0.0086544119,0.0054481202,11.325294,19.289989,-21.646988,-9.4418144,-0.00013677691,-0.024140595,0.0054962444,-0.037803859,0.096410193,-0.015827635,-0.0010584256,0.00020124357,-0.00015014365,-0.00024126736,0.0019981505,0.005893698,
			0.39684305,0.088887557,0.70398277,0.0020813285,0.032609969,-25.032372,-21.646996,80.472305,12.852535,-0.0030767312,0.061427224,-0.020836452,-0.074185878,-0.025973825,-0.054076828,0.00093928137,-0.00070677133,-0.00030580341,-1.8776336e-05,-0.00068366068,-0.0032429283,
			0.1129794,0.025101464,0.2008445,-0.0015480893,0.00089599297,-9.3817291,-9.4418144,12.852531,-0.75086272,-0.00078894594,0.017619692,-0.0058947923,0.038141936,0.0050895116,-0.046217456,0.00024930621,-0.00021792098,-9.864512e-05,8.5960692e-06,0.00096603419,0.002811075,
			-3.9111492e-05,-8.8329907e-06,-6.5633787e-05,3.9669217e-08,-1.7433867e-07,0.0015124457,-0.00013679649,-0.0030766968,-0.00078894035,9.5224959e-06,-3.0306351e-06,6.9548546e-06,1.4618729e-06,3.9293343e-07,-1.4601402e-06,-1.7197119e-06,-1.4016385e-07,2.7526303e-07,-3.4551244e-09,2.616569e-08,7.2604202e-08,
			0.00037699094,8.2360057e-05,0.00066822243,-3.0600097e-06,3.5679964e-06,-0.024230013,-0.024140617,0.061427195,0.017619692,-3.0306753e-06,6.4323147e-05,-1.9865422e-05,-4.3446969e-05,-8.2824126e-06,2.8725748e-05,5.8855352e-07,-3.0968417e-07,-6.5853598e-08,3.2059008e-10,-6.611258e-07,-1.5466574e-06,
			-0.00016086186,-3.7022932e-05,-0.00028625038,6.6288857e-07,-1.0809964e-06,0.0087118885,0.0054962155,-0.02083643,-0.0058947951,6.9549133e-06,-1.9865402e-05,2.3606683e-05,1.3694199e-05,2.7170272e-06,-9.6730473e-06,-8.2241726e-08,-3.3537599e-08,-1.4994227e-07,5.6516036e-10,2.1909059e-07,5.1818569e-07,
			-0.00027748541,-6.1853149e-05,-0.00049191434,-1.4828584e-05,-1.4265615e-05,-0.039505899,-0.037803799,-0.074185818,0.038141962,1.4618972e-06,-4.3446977e-05,1.3694188e-05,0.0025410778,-0.00017361385,-8.8160377e-05,1.2130721e-06,1.8751574e-07,-2.2972993e-06,1.4892549e-07,-6.1761093e-06,-7.8481644e-06,
			-5.3298911e-05,-1.1900624e-05,-9.449388e-05,2.6485421e-05,2.0671581e-05,-0.010190814,0.096410178,-0.025973847,0.0050895168,3.9293943e-07,-8.2824117e-06,2.717024e-06,-0.00017361387,0.0068906862,2.4403036e-05,-2.5692285e-07,1.1218403e-07,-2.7651998e-07,1.1950684e-07,-1.1770642e-06,-1.4346861e-06,
			0.00018545905,4.1404892e-05,0.00032873522,-2.7961148e-06,-3.8139551e-05,0.051102139,-0.015827583,-0.054076802,-0.046217464,-1.460172e-06,2.8725741e-05,-9.6730255e-06,-8.8160377e-05,2.4403043e-05,0.0067961668,8.1056356e-07,-4.5519499e-07,-7.5856616e-07,-2.6168184e-08,1.3785115e-06,4.2758943e-06,
			3.4745256e-06,7.5421292e-07,6.0953657e-06,-9.5827352e-07,6.3879469e-07,-0.00031894585,-0.0010584386,0.00093928794,0.00024930784,-1.7197119e-06,5.8855574e-07,-8.2242266e-08,1.2130857e-06,-2.5692256e-07,8.1056271e-07,1.8031147e-05,3.1826488e-07,3.0148877e-07,3.7237477e-08,3.8710571e-08,1.0427469e-08,
			-3.4480272e-06,-8.2554624e-07,-6.1289234e-06,-1.8256881e-07,-3.5129762e-08,0.00028548375,0.0002012422,-0.00070677116,-0.00021792075,-1.4016329e-07,-3.0968246e-07,-3.3538758e-08,1.8751555e-07,1.1218368e-07,-4.5519414e-07,3.1826332e-07,8.0112753e-05,-2.1427536e-07,-4.1715631e-09,-8.9357375e-09,8.4422966e-09,
			-2.0611303e-06,-5.1530725e-07,-3.6629731e-06,-1.1043157e-07,-4.7954359e-07,0.00014029916,-0.00015014494,-0.00030579453,-9.8646044e-05,2.7526403e-07,-6.5853413e-08,-1.4994347e-07,-2.2973018e-06,-2.7652013e-07,-7.5856479e-07,3.0148979e-07,-2.1427215e-07,8.9829686e-05,-3.4151633e-08,-8.6309321e-08,-3.5826261e-08,
			1.0789254e-09,2.7681055e-10,1.9541242e-09,-3.0887892e-07,-1.0789257e-08,-9.2789351e-06,-0.00024126701,-1.8774483e-05,8.5960764e-06,-3.4551191e-09,3.2060191e-10,5.6515032e-10,1.4892575e-07,1.1950698e-07,-2.6167596e-08,3.7237495e-08,-4.1716244e-09,-3.4150982e-08,3.219472e-08,-3.1883655e-08,-1.4081384e-08,
			-4.2548636e-06,-9.4951361e-07,-7.54235e-06,1.3015982e-07,-4.0350122e-07,-0.0009955134,0.0019981514,-0.00068365905,0.00096603413,2.6166452e-08,-6.611258e-07,2.1909014e-07,-6.1761098e-06,-1.1770642e-06,1.3785117e-06,3.8710269e-08,-8.9357544e-09,-8.6309264e-08,-3.1883665e-08,9.9854748e-05,-1.429797e-07,
			-9.9757481e-06,-2.2268434e-06,-1.7682956e-05,2.3472421e-07,-3.7224729e-07,-0.0026343935,0.0058936989,-0.0032429274,0.0028110757,7.2605559e-08,-1.5466576e-06,5.1818466e-07,-7.8481644e-06,-1.4346858e-06,4.2758948e-06,1.0427371e-08,8.4423002e-09,-3.5826268e-08,-1.4081392e-08,-1.4297976e-07,9.9739671e-05
			};

	arm_mat_init_f32(&P, 21, 21, PData);

	arm_matrix_instance_f32 PCorrect;
	float32_t PCorrectData[21*21];
	nearestPSD(&P, &PCorrect, PCorrectData);

	float32_t PDataTrue[21*21] = {0.0026760872,0.00059866486,0.0047298516,-1.3714354e-05,3.0298812e-05,-0.1656131,-0.15402287,0.3969577,0.086885832,-4.0780986e-05,0.00039724488,-0.00016965184,-0.00019695092,6.1695231e-05,2.7971342e-05,3.0106639e-06,-3.8330031e-06,-2.4827871e-06,-3.0394759e-07,1.0710348e-06,5.7245034e-06,
			0.00059866486,0.00014155611,0.00106169,-3.0316037e-06,6.7440833e-06,-0.03701251,-0.033909805,0.088913053,0.019294776,-9.2045366e-06,8.6867207e-05,-3.8979044e-05,-4.3931548e-05,1.3689354e-05,6.358634e-06,6.5103131e-07,-9.098826e-07,-6.0928426e-07,-7.117319e-08,2.369373e-07,1.2670018e-06,
			0.0047298516,0.00106169,0.0083926069,-2.4231071e-05,5.3864889e-05,-0.29364207,-0.2730886,0.70418835,0.15405464,-6.8627247e-05,0.00070454099,-0.00030201196,-0.00034750323,0.00011170909,4.633615e-05,5.2640676e-06,-6.8156069e-06,-4.4168537e-06,-5.8939088e-07,2.0134689e-06,1.0470007e-05,
			-1.3714354e-05,-3.0316037e-06,-2.4231071e-05,9.1004749e-06,8.2879178e-06,0.0016561614,0.0084219901,0.0020855353,-0.0025080622,-2.1628551e-08,-2.3148177e-06,3.3970809e-07,-1.1865743e-05,3.0715575e-05,-8.5908323e-06,-9.7525412e-07,-1.9487932e-07,-1.2590816e-07,-2.6889583e-07,3.2026051e-07,8.1234288e-07,
			3.0298812e-05,6.7440833e-06,5.3864889e-05,8.2879178e-06,4.715559e-05,-0.0017204625,0.0050880574,0.032616504,-0.00059119239,-2.6947163e-07,4.7223803e-06,-1.5820979e-06,-9.6755775e-06,2.7226237e-05,-4.7114656e-05,6.1229298e-07,-5.5209142e-08,-5.0395823e-07,-4.6509136e-08,-1.0088046e-07,5.2266364e-07,
			-0.1656131,-0.03701251,-0.29364207,0.0016561614,-0.0017204625,17.232695,11.68036,-25.038807,-7.9152536,0.0016062721,-0.025368284,0.0092058834,-0.044031944,-0.016653594,0.059952956,-0.0002928648,0.00030758081,0.00016397765,4.3580403e-06,-0.0012950972,-0.0035166496,
			-0.15402287,-0.033909805,-0.2730886,0.0084219901,0.0050880574,11.68036,19.587891,-21.652412,-8.2114763,-5.807844e-05,-0.025095619,0.0059106913,-0.041601092,0.090988122,-0.0084019527,-0.0010365474,0.00021986951,-0.00013057598,-0.00022802546,0.0017430251,0.0051532723,
			0.3969577,0.088913053,0.70418835,0.0020855353,0.032616504,-25.038807,-21.652412,80.47242,12.83022,-0.0030781417,0.061444543,-0.02084396,-0.074116975,-0.025875509,-0.054211494,0.0009388926,-0.00070631265,-0.0003073742,-2.6869682e-05,-0.00066507794,-0.003229487,
			0.086885832,0.019294776,0.15405464,-0.0025080622,-0.00059119239,-7.9152536,-8.2114763,12.83022,4.3306918,-0.00046386052,0.013675379,-0.0041829972,0.022458432,-0.017305061,-0.01554788,0.00033966295,-0.00014518441,-1.7120787e-05,5.3824944e-05,-5.1114534e-05,-0.00024669353,
			-4.0780986e-05,-9.2045366e-06,-6.8627247e-05,-2.1628551e-08,-2.6947163e-07,0.0016062721,-5.807844e-05,-0.0030781417,-0.00046386052,9.5433479e-06,-3.2829125e-06,7.0645237e-06,4.5856279e-07,-1.0394062e-06,5.0234428e-07,-1.7139505e-06,-1.3569466e-07,2.8054856e-07,1.5552731e-08,-3.9709171e-08,-1.2303991e-07,
			0.00039724488,8.6867207e-05,0.00070454099,-2.3148177e-06,4.7223803e-06,-0.025368284,-0.025095619,0.061444543,0.013675379,-3.2829125e-06,6.7384804e-05,-2.119397e-05,-3.1273332e-05,9.1003967e-06,4.9202754e-06,5.183951e-07,-3.6694917e-07,-1.3046215e-07,-3.2834748e-08,1.4643371e-07,8.265572e-07,
			-0.00016965184,-3.8979044e-05,-0.00030201196,3.3970809e-07,-1.5820979e-06,0.0092058834,0.0059106913,-0.02084396,-0.0041829972,7.0645237e-06,-2.119397e-05,2.4183682e-05,8.411168e-06,-4.826712e-06,6.5873246e-07,-5.1863651e-08,-8.952469e-09,-1.2218018e-07,4.9415156e-08,-1.295716e-07,-5.11873e-07,
			-0.00019695092,-4.3931548e-05,-0.00034750323,-1.1865743e-05,-9.6755775e-06,-0.044031944,-0.041601092,-0.074116975,0.022458432,4.5856279e-07,-3.1273332e-05,8.411168e-06,0.0025894823,-0.00010449549,-0.00018281666,9.3380413e-07,-4.858623e-08,-2.5447998e-06,1.4646622e-08,-3.0294559e-06,1.5888156e-06,
			6.1695231e-05,1.3689354e-05,0.00011170909,3.0715575e-05,2.7226237e-05,-0.016653594,0.090988122,-0.025875509,-0.017305061,-1.0394062e-06,9.1003967e-06,-4.826712e-06,-0.00010449549,0.0069893789,-0.00011076112,-6.5513319e-07,-1.9555924e-07,-6.4234786e-07,-8.6804789e-08,3.2955268e-06,1.2041188e-05,
			2.7971342e-05,6.358634e-06,4.633615e-05,-8.5908323e-06,-4.7114656e-05,0.059952956,-0.0084019527,-0.054211494,-0.01554788,5.0234428e-07,4.9202754e-06,6.5873246e-07,-0.00018281666,-0.00011076112,0.0069812704,1.357519e-06,-2.2651022e-08,-2.6647965e-07,2.6865425e-07,-4.7852936e-06,-1.4179057e-05,
			3.0106639e-06,6.5103131e-07,5.2640676e-06,-9.7525412e-07,6.1229298e-07,-0.0002928648,-0.0010365474,0.0009388926,0.00033966295,-1.7139505e-06,5.183951e-07,-5.1863651e-08,9.3380413e-07,-6.5513319e-07,1.357519e-06,1.8032826e-05,3.1942236e-07,3.0295411e-07,3.9364554e-08,2.1249082e-08,-4.3957531e-08,
			-3.8330031e-06,-9.098826e-07,-6.8156069e-06,-1.9487932e-07,-5.5209142e-08,0.00030758081,0.00021986951,-0.00070631265,-0.00014518441,-1.3569466e-07,-3.6694917e-07,-8.952469e-09,-4.858623e-08,-1.9555924e-07,-2.2651022e-08,3.1942236e-07,8.0760685e-05,-2.4007184e-07,-1.8740735e-08,-1.9554544e-07,-3.5366462e-08,
			-2.4827871e-06,-6.0928426e-07,-4.4168537e-06,-1.2590816e-07,-5.0395823e-07,0.00016397765,-0.00013057598,-0.0003073742,-1.7120787e-05,2.8054856e-07,-1.3046215e-07,-1.2218018e-07,-2.5447998e-06,-6.4234786e-07,-2.6647965e-07,3.0295411e-07,-2.4007184e-07,8.9788125e-05,-1.7864958e-08,-1.7390973e-07,-8.4885677e-08,
			-3.0394759e-07,-7.117319e-08,-5.8939088e-07,-2.6889583e-07,-4.6509136e-08,4.3580403e-06,-0.00022802546,-2.6869682e-05,5.3824944e-05,1.5552731e-08,-3.2834748e-08,4.9415156e-08,1.4646622e-08,-8.6804789e-08,2.6865425e-07,3.9364554e-08,-1.8740735e-08,-1.7864958e-08,4.3020822e-08,1.4539138e-06,-5.0489952e-08,
			1.0710348e-06,2.369373e-07,2.0134689e-06,3.2026051e-07,-1.0088046e-07,-0.0012950972,0.0017430251,-0.00066507794,-5.1114534e-05,-3.9709171e-08,1.4643371e-07,-1.295716e-07,-3.0294559e-06,3.2955268e-06,-4.7852936e-06,2.1249082e-08,-1.9554544e-07,-1.7390973e-07,1.4539138e-06,9.9787721e-05,4.7175925e-07,
			5.7245034e-06,1.2670018e-06,1.0470007e-05,8.1234288e-07,5.2266364e-07,-0.0035166496,0.0051532723,-0.003229487,-0.00024669353,-1.2303991e-07,8.265572e-07,-5.11873e-07,1.5888156e-06,1.2041188e-05,-1.4179057e-05,-4.3957531e-08,-3.5366462e-08,-8.4885677e-08,-5.0489952e-08,4.7175925e-07,0.00010623879
			};

	arm_matrix_instance_f32 PTrue;
	arm_mat_init_f32(&PTrue, 21, 21, PDataTrue);

	bool test = areMatricesEqual(&P, &PTrue);
}

void test_update_EKF(void) {

	arm_matrix_instance_f32 xPrev, PPrev, PqPrev, Q, Qq, H, Hq, R, Rq, aMeas, wMeas, llaMeas, magMeas, pressMeas, magI, dt, xPlus, Pplus, PqPlus, xPlusTrue, PplusTrue, PqPlusTrue;

	float32_t xPrevData[22*1] = {0.70759803,
			-0.00047243558,
			0.70661467,
			-0.00073295814,
			35.347893,
			-117.80683,
			671.93317,
			-0.021125605,
			0.011542665,
			-53.253784,
			1.4684781e-05,
			0.00015568511,
			-8.2018505e-06,
			-3.5961464e-05,
			-3.8672479e-09,
			-2.2649335e-10,
			-7.3725063e-12,
			-5.6151904e-13,
			1.1026901e-12,
			-0.00090790371,
			-6.181286e-12,
			5.4666089e-12
			};

	float32_t PPrevData[21*21] = {0.00013657048,2.7062404e-05,0.00022695315,-7.3913635e-09,7.6066691e-08,-1.1648136e-06,-0.0012329968,0.010391491,2.7873209e-06,-5.1838677e-05,-9.8861947e-06,-8.4224383e-05,2.474511e-12,2.4307056e-11,-1.5087392e-12,-1.0468999e-09,3.4088383e-09,4.069512e-08,6.2464166e-11,2.0514115e-13,3.596036e-15,
			2.7062408e-05,1.4328033e-05,4.8363563e-05,-1.724858e-09,1.6183003e-08,-2.6297576e-07,-0.00035478867,0.0022118934,6.4568917e-07,-9.9623931e-06,-6.6793614e-06,-1.7967173e-05,5.2414106e-13,4.1716357e-12,-6.959067e-12,-3.497338e-09,6.0778113e-08,8.8767225e-09,1.3231644e-11,2.3869516e-14,1.0910487e-13,
			0.00022695317,4.8363563e-05,0.00040954855,-1.3057947e-08,1.346921e-07,-2.0743023e-06,-0.0021904828,0.018462013,4.9631822e-06,-8.3920393e-05,-1.7777767e-05,-0.000153299,4.3706028e-12,4.129079e-11,-2.9587918e-12,-2.9419786e-08,7.4480639e-09,1.1905556e-07,1.1032764e-10,3.2925092e-13,9.4099753e-15,
			-7.3913635e-09,-1.724858e-09,-1.3057948e-08,6.3231851e-06,-5.1340451e-12,-6.0734277e-07,1.2040528e-07,-6.5337542e-07,6.0954602e-07,1.8260217e-09,2.9636238e-10,4.0474868e-09,-4.5887153e-12,-6.3144027e-13,-2.0167163e-09,6.6022784e-13,1.6912146e-12,-1.1084817e-12,-1.1599585e-10,-7.642392e-15,1.2760116e-11,
			7.6066677e-08,1.6183003e-08,1.3469209e-07,-5.1340447e-12,7.6212996e-06,-1.8371353e-09,-8.3120403e-07,7.0147057e-06,2.9605289e-09,-2.2205096e-08,-4.6713509e-09,-3.9263394e-08,6.9694093e-13,-2.4655438e-09,4.9427481e-13,-8.14306e-12,-9.1520765e-14,8.991574e-12,1.854502e-11,-2.2136628e-11,-5.9055882e-15,
			-1.1648136e-06,-2.6297573e-07,-2.0743023e-06,-6.0734277e-07,-1.8371353e-09,0.42860335,0.00013019367,-0.00034038708,-0.47429562,4.5905901e-07,1.0552584e-07,8.2529209e-07,-0.00024788885,-6.6121295e-08,2.0428004e-08,1.4291796e-10,-1.0257505e-11,-2.4324223e-10,-0.0065694852,-1.0206708e-09,-4.4347692e-10,
			-0.0012329968,-0.00035478864,-0.0021904828,1.2040528e-07,-8.312042e-07,0.00013019367,0.018693132,-0.10873404,-0.00015922434,0.00039653224,0.00011225758,0.00070680148,-7.4095581e-08,-1.1411776e-07,-0.00027800651,1.4182503e-07,-4.2644885e-08,-2.3394131e-07,-1.8088341e-06,-2.2966122e-09,3.9432525e-06,
			0.010391491,0.0022118932,0.018462012,-6.5337537e-07,7.0147062e-06,-0.00034038708,-0.10873403,0.917256,0.00050063012,-0.0033419356,-0.00070509198,-0.0059603783,1.5108678e-07,-0.00027800546,7.1549401e-08,-1.1982672e-06,9.8444815e-08,1.9682568e-06,3.8124749e-06,-4.7696617e-06,-1.6192911e-09,
			2.7873214e-06,6.4568911e-07,4.9631813e-06,6.0954602e-07,2.9605294e-09,-0.47429562,-0.00015922434,0.00050063012,0.52260005,-1.0425406e-06,-2.4794463e-07,-1.8711836e-06,0.00027521097,1.5197693e-07,-7.4862228e-08,-3.4471975e-10,1.3230966e-10,6.1225858e-10,0.0072037163,3.9225343e-09,2.2300752e-09,
			-5.1838688e-05,-9.962394e-06,-8.3920386e-05,1.8260216e-09,-2.2205098e-08,4.5905912e-07,0.00039653218,-0.0033419356,-1.0425408e-06,3.5039277e-05,4.3915556e-06,3.7734633e-05,-6.2695335e-13,1.8452492e-12,8.0428925e-13,-2.4868218e-08,-2.0904261e-09,-1.9373537e-08,-1.5829092e-11,1.1416667e-14,-2.3362854e-15,
			-9.8861938e-06,-6.6793618e-06,-1.7777766e-05,2.9636205e-10,-4.6713526e-09,1.0552591e-07,0.00011225755,-0.00070509216,-2.4794477e-07,4.3915566e-06,1.4854118e-05,8.0287782e-06,-1.2276806e-13,-2.03756e-12,-1.6128047e-11,1.5789176e-09,4.6248744e-08,-4.1988066e-09,-3.1002958e-12,-1.2199182e-14,5.7526642e-14,
			-8.4224383e-05,-1.7967173e-05,-0.000153299,4.0474863e-09,-3.9263401e-08,8.2529203e-07,0.00070680142,-0.0059603779,-1.8711841e-06,3.7734641e-05,8.02878e-06,8.067875e-05,-1.0885723e-12,-9.1784523e-13,1.4449125e-12,1.3867409e-08,-4.0680024e-09,1.1219086e-08,-2.7482232e-11,-5.0750331e-15,-5.0097344e-15,
			2.4745106e-12,5.241409e-13,4.370602e-12,-4.5887153e-12,6.9694093e-13,-0.00024788885,-7.4095581e-08,1.5108678e-07,0.00027521097,-6.269534e-13,-1.2276809e-13,-1.0885725e-12,0.00017839829,-1.5615333e-13,-3.6538528e-15,-2.6805946e-16,-1.7598921e-16,3.2156282e-17,-4.0649663e-08,-2.5879586e-16,2.5161664e-16,
			2.4307056e-11,4.1716357e-12,4.1290783e-11,-6.3144027e-13,-2.4655438e-09,-6.6121295e-08,-1.1411775e-07,-0.00027800546,1.5197693e-07,1.845249e-12,-2.0375594e-12,-9.1784609e-13,-1.5615333e-13,0.0001783999,-1.7128086e-18,1.5893668e-16,5.7724258e-15,1.8820217e-15,-3.9423352e-12,-1.3591324e-17,3.5206988e-20,
			-1.508739e-12,-6.9590687e-12,-2.9587914e-12,-2.0167163e-09,4.9427471e-13,2.0428004e-08,-0.00027800651,7.1549387e-08,-7.4862228e-08,8.0429093e-13,-1.6128045e-11,1.4449108e-12,-3.6538528e-15,-1.7128092e-18,0.0001783999,1.4076089e-16,4.7271259e-14,-3.5312037e-15,-9.2247655e-14,-1.2320093e-20,-1.0943589e-17,
			-1.0468804e-09,-3.4973344e-09,-2.9419754e-08,6.6022773e-13,-8.1430609e-12,1.4291764e-10,1.4182491e-07,-1.1982667e-06,-3.4471917e-10,-2.4868221e-08,1.578915e-09,1.3867405e-08,-2.6805901e-16,1.5893337e-16,1.4075954e-16,9.9999925e-05,4.1744538e-13,-2.6092271e-12,-6.7674756e-15,3.3850737e-18,-9.4079945e-19,
			3.4088381e-09,6.077812e-08,7.4480582e-09,1.691216e-12,-9.151695e-14,-1.0257653e-11,-4.2644722e-08,9.8444957e-08,1.3231011e-10,-2.0904383e-09,4.6248751e-08,-4.0679953e-09,-1.7598924e-16,5.7724279e-15,4.7271252e-14,4.1744088e-13,9.9999677e-05,2.4252794e-11,-4.4403923e-15,6.8497305e-17,-4.2274385e-16,
			4.069511e-08,8.876718e-09,1.1905557e-07,-1.1084821e-12,8.9915601e-12,-2.4324193e-10,-2.3394122e-07,1.9682561e-06,6.1225791e-10,-1.937352e-08,-4.198812e-09,1.1219076e-08,3.2155726e-17,1.8820145e-15,-3.5312071e-15,-2.6092292e-12,2.4252819e-11,9.9999939e-05,8.1039925e-16,2.5326795e-17,4.0579909e-17,
			6.2464159e-11,1.3231642e-11,1.1032763e-10,-1.1599585e-10,1.854502e-11,-0.0065694852,-1.8088341e-06,3.8124749e-06,0.0072037163,-1.5829095e-11,-3.1002969e-12,-2.748223e-11,-4.0649663e-08,-3.9423352e-12,-9.2247668e-14,-6.7674972e-15,-4.4403949e-15,8.1038691e-16,9.8973731e-05,-6.5337064e-15,6.3524545e-15,
			2.0514117e-13,2.3869519e-14,3.2925098e-13,-7.642392e-15,-2.2136628e-11,-1.0206708e-09,-2.2966122e-09,-4.7696617e-06,3.9225343e-09,1.141666e-14,-1.2199178e-14,-5.0750322e-15,-2.5879586e-16,-1.3591324e-17,-1.2320088e-20,3.3850754e-18,6.8497278e-17,2.5326785e-17,-6.5337064e-15,9.9999997e-05,1.6021638e-22,
			3.5960301e-15,1.0910486e-13,9.4099575e-15,1.2760116e-11,-5.9055882e-15,-4.4347692e-10,3.9432525e-06,-1.6192911e-09,2.2300752e-09,-2.3362986e-15,5.7526642e-14,-5.0097162e-15,2.5161664e-16,3.5206991e-20,-1.0943587e-17,-9.4080928e-19,-4.2274382e-16,4.0579863e-17,6.3524545e-15,1.602164e-22,9.9999997e-05,
			};

	float32_t PqData[6*6] = {0.00013183476,2.754629e-05,0.00022928696,-4.9315957e-05,-1.0349562e-05,-8.5484651e-05,
			2.7546292e-05,8.1336611e-06,4.8778056e-05,-1.0242314e-05,-3.3395204e-06,-1.8188031e-05,
			0.00022928695,4.8778056e-05,0.00040836397,-8.5244814e-05,-1.8327106e-05,-0.0001525531,
			-4.9315928e-05,-1.0242307e-05,-8.524477e-05,3.297357e-05,4.6999962e-06,3.8813523e-05,
			-1.0349557e-05,-3.339519e-06,-1.8327099e-05,4.6999953e-06,1.2094387e-05,8.3462592e-06,
			-8.5484651e-05,-1.818803e-05,-0.00015255308,3.8813538e-05,8.3462564e-06,8.0041427e-05,
			};

	float32_t QData[12*12] = {2.0943951e-05,0,0,0,0,0,0,0,0,0,0,0,
			0,2.0943951e-05,0,0,0,0,0,0,0,0,0,0,
			0,0,2.0943951e-05,0,0,0,0,0,0,0,0,0,
			0,0,0,1.454441e-06,0,0,0,0,0,0,0,0,
			0,0,0,0,1.454441e-06,0,0,0,0,0,0,0,
			0,0,0,0,0,1.454441e-06,0,0,0,0,0,0,
			0,0,0,0,0,0,0.0001962,0,0,0,0,0,
			0,0,0,0,0,0,0,0.0001962,0,0,0,0,
			0,0,0,0,0,0,0,0,0.0001962,0,0,0,
			0,0,0,0,0,0,0,0,0,3.92e-05,0,0,
			0,0,0,0,0,0,0,0,0,0,3.92e-05,0,
			0,0,0,0,0,0,0,0,0,0,0,3.92e-05,
			};

	float32_t QqData[6*6] = {2.0943951e-05,0,0,0,0,0,
			0,2.0943951e-05,0,0,0,0,
			0,0,2.0943951e-05,0,0,0,
			0,0,0,1.454441e-06,0,0,
			0,0,0,0,1.454441e-06,0,
			0,0,0,0,0,1.454441e-06,
			};

	float32_t HData[3*21] = {0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
			0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
			0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
			};

	float32_t RData[3*3] = {1.35e-05,0,0,
			0,1.65e-05,0,
			0,0,2,
			};

	float32_t RqData[3*3] = {3.2e-07,0,0,
			0,4.1e-07,0,
			0,0,3.2e-07,
			};

	float32_t Rb = 0.0025;

	float32_t aMeasData[3*1] = {40.749749,
			0.039858745,
			-0.031245514,
			};

	float32_t wMeasData[3*1] = {-0.0014199036,
			-0.0049753581,
			0.0010024369,
			};

	float32_t llaMeasData[3*1] = {35.347872,
			-117.80681,
			670.77311,
			};

	float32_t magMeasData[3*1] = {-0.86552111,
			0.10263403,
			0.49023479,
			};

	float32_t pressMeasData[3*1] = {1, 2, 3};

	float32_t magIData[3*1] = {0.4891,
			0.104,
			0.866,
			};

	float32_t HqData[3*6] = {0,-0.866,0.104,0,0,0,
			0.866,0,-0.4891,0,0,0,
			-0.104,0.4891,0,0,0,0,
			};

	float32_t vdStart = 0;
	float32_t mainAltStart = 0;
	float32_t drougeAltStart = 0;

	reco_message message;

	arm_mat_init_f32(&xPrev, 22, 1, xPrevData);
	arm_mat_init_f32(&PPrev, 21, 21, PPrevData);
	arm_mat_init_f32(&PqPrev, 6, 6, PqData);
	arm_mat_init_f32(&Q, 12, 12, QData);
	arm_mat_init_f32(&Qq, 6, 6, QqData);
	arm_mat_init_f32(&H, 3, 21, HData);
	arm_mat_init_f32(&R, 3, 3, RData);
	arm_mat_init_f32(&Rq, 3, 3, RqData);
	arm_mat_init_f32(&aMeas, 3, 1, aMeasData);
	arm_mat_init_f32(&wMeas, 3, 1, wMeasData);
	arm_mat_init_f32(&llaMeas, 3, 1, llaMeasData);
	arm_mat_init_f32(&magMeas, 3, 1, magIData);
	arm_mat_init_f32(&magI, 3, 1, magIData);
	arm_mat_init_f32(&Hq, 3, 6, HqData);


	float32_t xPlusBuff[22*1];
	float32_t PPlusBuff[21*21];
	float32_t PqPlusBuff[6*6];

	update_EKF(&xPrev, &PPrev, &Q,
			   &H, &R, Rb, &aMeas,
			   &wMeas, &llaMeas, &magMeas, 0, &magI,
			   we, 0.001f, &xPlus, &Pplus,
			   xPlusBuff, PPlusBuff, &vdStart,
			   &mainAltStart, &drougeAltStart, &message);

	float32_t xPlusDataTest[22*1] = {0.70758718,
			-0.00050323078,
			0.70662552,
			-0.00073620229,
			35.347885,
			-117.80682,
			672.16156,
			-0.020965694,
			0.011432247,
			-53.228939,
			2.6728259e-05,
			0.00011491068,
			2.0559492e-05,
			0.00013804069,
			4.1351974e-08,
			-1.2761682e-08,
			-1.1546478e-10,
			5.6319892e-12,
			1.8183503e-10,
			0.0037028317,
			7.138421e-10,
			3.1490585e-10,
			};

	float32_t PPlusDataTest[21*21] = {0.0001355586,2.7415161e-05,0.000229817,-5.1229891e-09,5.2971384e-08,-9.8400062e-07,-0.0012500731,0.010527903,2.4786459e-06,-5.0991275e-05,-1.0019453e-05,-8.5239757e-05,-1.2084031e-10,3.2302057e-11,-2.3945904e-12,-3.903414e-09,4.3002144e-09,4.7575128e-08,-3.205155e-09,2.7713069e-13,9.9902056e-15,
			2.7415164e-05,1.177666e-05,4.8947404e-05,-1.1620388e-09,1.1265461e-08,-2.1796609e-07,-0.0003303744,0.0022398948,5.5910971e-07,-1.0091243e-05,-5.2914443e-06,-1.8170505e-05,-2.6790948e-11,6.1679958e-12,-5.1533959e-12,-3.3441598e-09,4.428783e-08,1.0367508e-08,-7.1056921e-10,4.4948772e-14,7.8574097e-14,
			0.00022981699,4.8947404e-05,0.00041216874,-9.0580476e-09,9.3790518e-08,-1.7489552e-06,-0.0022194097,0.018683115,4.4043127e-06,-8.4909982e-05,-1.7980086e-05,-0.00015376035,-2.1480509e-10,5.5976574e-11,-4.5166605e-12,-2.7219654e-08,9.3570902e-09,1.1430195e-07,-5.6974341e-09,4.6685014e-13,2.1135974e-14,
			-5.12299e-09,-1.1620387e-09,-9.0580468e-09,4.30622e-06,-2.4602239e-12,-3.4269146e-07,8.3265761e-08,-4.5516646e-07,3.3455677e-07,1.2669663e-09,1.8947452e-10,2.8026383e-09,-4.6075824e-11,-4.4899322e-13,-1.3904848e-09,4.6416533e-13,1.4150642e-12,-7.9932301e-13,-1.2171032e-09,-5.5306334e-15,8.9318466e-12,
			5.2971384e-08,1.1265461e-08,9.3790518e-08,-2.4602234e-12,5.2132955e-06,-1.0685851e-09,-5.812268e-07,4.9049909e-06,1.7745562e-09,-1.5440641e-08,-3.2479568e-09,-2.7306404e-08,3.5418939e-13,-1.7074556e-09,3.4312868e-13,-5.6606135e-12,-6.2119919e-14,6.2996479e-12,9.4239165e-12,-1.5501288e-11,-4.1593879e-15,
			-9.8400062e-07,-2.1796609e-07,-1.7489552e-06,-3.4269152e-07,-1.068585e-09,0.35937095,0.00011120933,-0.00028824824,-0.3955566,3.8600973e-07,8.6516295e-08,6.9223813e-07,-0.00020560442,-5.5487128e-08,1.7319662e-08,1.171248e-10,1.9114781e-11,-2.0248495e-10,-0.0054481369,-8.6944973e-10,-3.8175124e-10,
			-0.0012500733,-0.0003303744,-0.0022194097,8.3265768e-08,-5.8122686e-07,0.00011120933,0.018649088,-0.11053775,-0.00013645353,0.00040168923,9.8342491e-05,0.00071532989,-6.2618483e-08,-1.1510063e-07,-0.00027979052,1.4069379e-07,1.6160675e-07,-2.5214999e-07,-1.4956117e-06,-2.3161792e-09,3.9744982e-06,
			0.010527903,0.0022398946,0.018683113,-4.5516643e-07,4.9049909e-06,-0.00028824824,-0.11053774,0.93228751,0.00043322213,-0.0033811424,-0.00071316707,-0.0060201827,1.1830822e-07,-0.00027978868,7.1901397e-08,-1.1913875e-06,1.1426624e-07,1.9877227e-06,2.9326366e-06,-4.8095135e-06,-1.6270326e-09,
			2.4786464e-06,5.5910965e-07,4.4043122e-06,3.3455677e-07,1.7745564e-09,-0.3955566,-0.00013645353,0.00043322207,0.43310961,-9.2036964e-07,-2.1086309e-07,-1.6470312e-06,0.00022740869,1.4196908e-07,-7.3106051e-08,-2.9785963e-10,3.600488e-12,5.2153731e-10,0.0059304987,3.7865204e-09,2.1811035e-09,
			-5.0991275e-05,-1.0091243e-05,-8.4909967e-05,1.2669662e-09,-1.5440643e-08,3.8600984e-07,0.00040168918,-0.0033811424,-9.2036998e-07,3.4420435e-05,4.4404019e-06,3.8066333e-05,4.7749985e-11,-3.3058858e-13,1.0554204e-12,-2.2477378e-08,-2.5800642e-09,-2.2890195e-08,1.2660685e-09,-6.705926e-15,-4.3909972e-15,
			-1.0019451e-05,-5.2914443e-06,-1.7980085e-05,1.8947424e-10,-3.2479583e-09,8.6516337e-08,9.834244e-05,-0.00071316725,-2.1086319e-07,4.4404032e-06,1.4052875e-05,8.0942518e-06,1.0719496e-11,-2.6750299e-12,-1.7297965e-11,1.4787539e-09,5.748484e-08,-4.9534363e-09,2.8420002e-10,-1.9600085e-14,7.7137746e-14,
			-8.523975e-05,-1.8170504e-05,-0.00015376037,2.8026381e-09,-2.7306413e-08,6.9223796e-07,0.00071532984,-0.0060201823,-1.6470317e-06,3.806634e-05,8.0942536e-06,8.0509264e-05,8.5664711e-11,-5.0867223e-12,1.9630074e-12,1.2605704e-08,-5.0674314e-09,1.3176074e-08,2.2713189e-09,-4.3407139e-14,-9.3158386e-15,
			-1.2084031e-10,-2.6790948e-11,-2.1480508e-10,-4.6075824e-11,3.5418936e-13,-0.00020560442,-6.261849e-08,1.1830822e-07,0.00022740869,4.7749971e-11,1.0719493e-11,8.5664725e-11,0.00017876452,-7.1097529e-12,2.1663814e-12,1.440708e-14,2.2226145e-15,-2.5339063e-14,-7.2341277e-07,-1.09218e-13,-4.7586592e-14,
			3.2302053e-11,6.1679949e-12,5.5976564e-11,-4.4899325e-13,-1.7074555e-09,-5.5487128e-08,-1.1510063e-07,-0.00027978868,1.4196908e-07,-3.3058934e-13,-2.6750286e-12,-5.0867223e-12,-7.1097529e-12,0.0001787919,5.6964157e-16,-1.2336927e-15,7.7765942e-15,3.4698561e-15,-1.8819989e-10,-2.3880366e-15,-1.3076925e-17,
			-2.3945893e-12,-5.1533972e-12,-4.5166596e-12,-1.3904847e-09,3.431286e-13,1.731966e-08,-0.00027979052,7.1901383e-08,-7.3106051e-08,1.0554225e-12,-1.7297963e-11,1.9630055e-12,2.166381e-12,5.6964146e-16,0.0001787919,9.5341043e-17,6.3984652e-14,-4.8066587e-15,5.741024e-11,8.8246225e-18,1.3440007e-15,
			-3.9033932e-09,-3.3441541e-09,-2.7219613e-08,4.6416495e-13,-5.6606178e-12,1.1712444e-10,1.4069367e-07,-1.1913866e-06,-2.9785868e-10,-2.2477389e-08,1.4787501e-09,1.2605698e-08,1.4407041e-14,-1.2336972e-15,9.5339527e-17,9.9999917e-05,1.1040348e-12,1.0771957e-11,3.8210025e-13,-1.4905851e-17,1.6815961e-20,
			4.3002135e-09,4.4287837e-08,9.357084e-09,1.4150658e-12,-6.2113597e-14,1.9114668e-11,1.6160701e-07,1.1426688e-07,3.600821e-12,-2.580081e-09,5.748484e-08,-5.0674238e-09,2.2225905e-15,7.7765993e-15,6.3984632e-14,1.1040268e-12,9.9999525e-05,3.5681548e-11,5.9111631e-14,1.0876669e-16,-6.965784e-16,
			4.7575117e-08,1.0367501e-08,1.1430195e-07,-7.9932371e-13,6.299644e-12,-2.0248453e-10,-2.5214985e-07,1.9877205e-06,5.2153631e-10,-2.2890168e-08,-4.9534412e-09,1.3176054e-08,-2.5339023e-14,3.4698463e-15,-4.8066642e-15,1.0771961e-11,3.5681583e-11,9.9999917e-05,-6.7148517e-13,4.6361167e-17,6.1165712e-17,
			-3.2051553e-09,-7.1056921e-10,-5.697435e-09,-1.2171032e-09,9.4239165e-12,-0.0054481369,-1.4956117e-06,2.9326368e-06,0.0059304987,1.2660685e-09,2.8419989e-10,2.2713194e-09,-7.2341277e-07,-1.8819986e-10,5.7410247e-11,3.8210079e-13,5.9111712e-14,-6.7148582e-13,8.0881771e-05,-2.8937508e-12,-1.2612754e-12,
			2.7713075e-13,4.4948776e-14,4.6685019e-13,-5.5306334e-15,-1.5501288e-11,-8.6944979e-10,-2.3161792e-09,-4.8095135e-06,3.7865204e-09,-6.7059361e-15,-1.960008e-14,-4.3407132e-14,-1.09218e-13,-2.3880368e-15,8.8246242e-18,-1.4905854e-17,1.0876667e-16,4.6361197e-17,-2.8937508e-12,9.9999997e-05,-2.025961e-19,
			9.9901853e-15,7.8574076e-14,2.1135961e-14,8.9318457e-12,-4.1593875e-15,-3.8175124e-10,3.9744982e-06,-1.6270326e-09,2.1811035e-09,-4.3910222e-15,7.7137739e-14,-9.3158157e-15,-4.7586592e-14,-1.3076925e-17,1.3440006e-15,1.6808207e-20,-6.9657835e-16,6.1165626e-17,-1.2612754e-12,-2.025961e-19,9.9999997e-05,
			};

	float32_t PqPlusDataTest[6*6] = {0.00013135246,2.7859951e-05,0.00023195099,-4.8803966e-05,-1.0450017e-05,-8.6351567e-05,
			2.7859951e-05,6.2080526e-06,4.9329508e-05,-1.0342371e-05,-2.3635407e-06,-1.8365537e-05,
			0.00023195098,4.9329505e-05,0.00041105095,-8.6102147e-05,-1.8503115e-05,-0.00015307259,
			-4.8803933e-05,-1.0342366e-05,-8.6102104e-05,3.2566317e-05,4.7263657e-06,3.9048573e-05,
			-1.045001e-05,-2.36354e-06,-1.8503106e-05,4.7263648e-06,1.1562463e-05,8.3929672e-06,
			-8.6351567e-05,-1.8365537e-05,-0.00015307257,3.9048584e-05,8.3929663e-06,7.9917998e-05,
			};

	arm_matrix_instance_f32 PPlusTrue;

	arm_mat_init_f32(&xPlusTrue, 22, 1, xPlusDataTest);
	arm_mat_init_f32(&PPlusTrue, 21, 21, PPlusDataTest);
	arm_mat_init_f32(&PqPlusBuff, 6, 6, PqPlusDataTest);

	bool test1 = areMatricesEqual(&xPlusTrue, &xPlus);
	bool test2 = areMatricesEqual(&PplusTrue, &Pplus);
	bool test3 = areMatricesEqual(&PqPlusTrue, &PqPlus);
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
