#include "tests.h"
#include "float.h"
#include "Inc/ekf.h"
#include "Inc/ekf_utils.h"
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

bool test_compute_Pqdot() {

	arm_matrix_instance_f32 PqDot, xPlus, Qq, wMeas, Pq, PqDotActual;

	float32_t actualPqDotBuff[6*6] = {0.00011957584,2.059187e-05,0.00017072941,-3.2520169e-05,-4.6851833e-06,-3.9076884e-05,
			2.059187e-05,2.7622997e-05,3.6515128e-05,-4.6018849e-06,-1.2090873e-05,-8.402154e-06,
			0.00017072947,3.6515135e-05,0.00032605013,-3.8004622e-05,-8.3158302e-06,-8.0497099e-05,
			-3.2520169e-05,-4.6018858e-06,-3.8004608e-05,1.0427146e-06,-5.1718093e-08,-2.4873563e-07,
			-4.6851824e-06,-1.2090873e-05,-8.3158329e-06,-5.1718121e-08,1.4512168e-06,1.1775946e-08,
			-3.9076898e-05,-8.4021513e-06,-8.0497099e-05,-2.4873563e-07,1.177597e-08,1.8693923e-06,
			};

	float32_t xPlusBuff[22*1] = {0.70759803,
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

	float32_t QqBuff[6*6] = {2.0943951e-05,0,0,0,0,0,
			0,2.0943951e-05,0,0,0,0,
			0,0,2.0943951e-05,0,0,0,
			0,0,0,1.454441e-06,0,0,
			0,0,0,0,1.454441e-06,0,
			0,0,0,0,0,1.454441e-06,
			};

	float32_t wMeasData[3] = {-0.00141990364120018f,
							  -0.00497535811999743f,
							   0.00100243692685004f};

	float32_t PqPlusBuff[6*6] = {0.00013183476,2.754629e-05,0.00022928696,-4.9315957e-05,-1.0349562e-05,-8.5484651e-05,
			2.7546292e-05,8.1336611e-06,4.8778056e-05,-1.0242314e-05,-3.3395204e-06,-1.8188031e-05,
			0.00022928695,4.8778056e-05,0.00040836397,-8.5244814e-05,-1.8327106e-05,-0.0001525531,
			-4.9315928e-05,-1.0242307e-05,-8.524477e-05,3.297357e-05,4.6999962e-06,3.8813523e-05,
			-1.0349557e-05,-3.339519e-06,-1.8327099e-05,4.6999953e-06,1.2094387e-05,8.3462592e-06,
			-8.5484651e-05,-1.818803e-05,-0.00015255308,3.8813538e-05,8.3462564e-06,8.0041427e-05,
			};

	float32_t PqDotBuff[6*6];

	arm_mat_init_f32(&PqDotActual, 6, 6, actualPqDotBuff);
	arm_mat_init_f32(&xPlus, 22, 1, xPlusBuff);
	arm_mat_init_f32(&Qq, 6, 6, QqBuff);
	arm_mat_init_f32(&wMeas, 3, 1, wMeasData);
	arm_mat_init_f32(&Pq, 6, 6, PqPlusBuff);

	compute_Pqdot(xPlusBuff, PqPlusBuff, QqBuff, wMeasData, &PqDot, PqDotBuff);

	bool test = areMatricesEqual(&PqDot, &PqDotActual);
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

	integrate(&x, &P, &Pq, &qdot, &pdot,
			  &vdot, &Pdot, &Pqdot, dt, &xMinus,
			  &Pminus, &Pqminus, xMinusData,
			  PMinusData, PqMinusBuff);

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

		propogate(&xPlus, &P_plus, &Pq_plus, &what,
				  &aHatN, &wMeas, &aMeas, &Q, &Qq,
				  dt, we, &xMinus, &Pminus, &Pqminus,
				  xMinusData, PMinusData, PqMinusBuff);

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

	float32_t xMinusData[22*1] = { 0.707616150379181f,
		     -0.000474318774649873f,
		         0.706596553325653f,
		     -0.000724362791515887f,
		          35.3478927612305f,
		         -117.806831359863f,
		          672.465698242188f,
		       -0.0208708215504885f,
		        0.0111826080828905f,
		         -53.5636978149414f,
		      1.46847814903595e-05f,
		       0.00015568510571029f,
		     -8.20185050542932e-06f,
		     -3.59614641638473e-05f,
		     -3.86724785528259e-09f,
		     -2.26493351851431e-10f,
		     -7.37250625132413e-12f,
		      -5.6151903873175e-13f,
		      1.10269009635094e-12f,
		      -0.00090790371177718f,
		     -6.18128595183953e-12f,
		      5.46660893635531e-12f
			};

	float32_t PMinusData[21*21] = {0.00013784051,2.7258877e-05,0.00022864944,-7.5251902e-09,7.7438862e-08,-1.1973858e-06,-0.0012480185,0.010518491,2.7114581e-06,-5.2193474e-05,-9.9310892e-06,-8.4609768e-05,2.480845e-12,2.4600154e-11,-1.5151399e-12,6.3482597e-10,3.4307388e-09,4.0895053e-08,6.262408e-11,2.103522e-13,3.6046292e-15,
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

	float32_t HData[3*21] = {0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
			0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
			0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
			};

	float32_t RData[3*3] = {1.35e-05,0,0,
			0,1.65e-05,0,
			0,0,2
			};

	float32_t llaMeasData[3] = {35.3478721589165f,
	         -117.806814545689f,
	          670.773107264234f,
			};

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

	float32_t xPlusTrueData[22*1] = {0.707616150379181,
		     -0.000474318774649873,
		         0.706596553325653,
		     -0.000724362791515887,
		          35.3478851318359,
		         -117.806823730469,
		          672.161560058594,
		       -0.0209656935185194,
		         0.011432247236371,
		         -53.2289390563965,
		      1.46847814903595e-05,
		       0.00015568510571029,
		     -8.20185050542932e-06,
		      0.000138040690217167,
		      4.13519742892277e-08,
		     -1.27616823775156e-08,
		     -1.15464776628826e-10,
		      5.63198923261843e-12,
		      1.81835033141731e-10,
		       0.00370283168740571,
		      7.13842096544681e-10,
		      3.14905851395153e-10
			};

	float32_t PPlusDataTrue[21*21] = {0.00013784027,2.7258824e-05,0.00022864901,-5.1250044e-09,5.297149e-08,-9.824189e-07,-0.0012480157,0.010518468,2.4748194e-06,-5.2193402e-05,-9.9310737e-06,-8.4609637e-05,-1.2064039e-10,3.2580227e-11,-2.2814673e-12,6.3485289e-10,3.4307397e-09,4.0895024e-08,-3.1998597e-09,2.8258022e-13,8.3744005e-15,
			2.7258828e-05,1.466912e-05,4.8711558e-05,-1.198327e-09,1.1266404e-08,-2.2189451e-07,-0.00036169073,0.0022383055,5.7399973e-07,-1.0004565e-05,-6.8275417e-06,-1.8044382e-05,-2.7283661e-11,5.8885041e-12,-7.4142463e-12,-3.5126888e-09,6.5446514e-08,8.9165857e-09,-7.2362438e-10,3.9348011e-14,1.1584864e-13,
			0.00022864902,4.8711558e-05,0.00041280125,-9.0530232e-09,9.3790362e-08,-1.7493462e-06,-0.0022170455,0.018688191,4.4046678e-06,-8.4295083e-05,-1.7857616e-05,-0.00015410148,-2.1485494e-10,5.5862724e-11,-4.327599e-12,-2.9558409e-08,7.489442e-09,1.1793072e-07,-5.6987535e-09,4.6463354e-13,1.7889456e-14,
			-5.1250044e-09,-1.1983268e-09,-9.0530232e-09,4.30622e-06,-2.460236e-12,-3.4269141e-07,8.3658009e-08,-4.5512985e-07,3.345566e-07,1.2679734e-09,2.0873667e-10,2.7999418e-09,-4.6075817e-11,-4.4899018e-13,-1.3904848e-09,4.5835736e-13,1.1491329e-12,-7.692988e-13,-1.2171031e-09,-5.530572e-15,8.9318457e-12,
			5.2971483e-08,1.1266405e-08,9.3790362e-08,-2.4602356e-12,5.2132955e-06,-1.0685863e-09,-5.8123697e-07,4.9049895e-06,1.7745608e-09,-1.5440694e-08,-3.2484575e-09,-2.7306321e-08,3.5418923e-13,-1.7074556e-09,3.4312795e-13,-5.6603632e-12,-5.5196414e-14,6.2987202e-12,9.4239122e-12,-1.5501288e-11,-4.1593756e-15,
			-9.8241901e-07,-2.2189451e-07,-1.7493462e-06,-3.4269146e-07,-1.0685862e-09,0.35937095,0.00011125208,-0.00028825182,-0.3955566,3.8517067e-07,8.8605773e-08,6.9245044e-07,-0.00020560442,-5.5487128e-08,1.7319666e-08,1.20082e-10,-9.4569075e-12,-2.0458565e-10,-0.0054481369,-8.6944973e-10,-3.817513e-10,
			-0.0012480158,-0.00036169071,-0.0022170458,8.3658023e-08,-5.8123703e-07,0.00011125208,0.018988205,-0.11052208,-0.00013661523,0.00040055861,0.00011497449,0.00071406655,-6.2613118e-08,-1.1509756e-07,-0.00027979049,1.4324532e-07,-6.7431799e-08,-2.3752571e-07,-1.4954696e-06,-2.3161177e-09,3.9744978e-06,
			0.010518468,0.0022383055,0.018688189,-4.5512985e-07,4.9049895e-06,-0.00028825182,-0.11052206,0.93232828,0.00043322661,-0.0033761745,-0.00071234547,-0.0060229199,1.1830777e-07,-0.00027978868,7.1902676e-08,-1.2102647e-06,1.0149288e-07,2.0168268e-06,2.9326245e-06,-4.8095135e-06,-1.6270548e-09,
			2.4748199e-06,5.7399961e-07,4.4046674e-06,3.345566e-07,1.7745611e-09,-0.3955566,-0.00013661523,0.00043322655,0.43310961,-9.1833175e-07,-2.187778e-07,-1.6472286e-06,0.00022740869,1.4196908e-07,-7.3106065e-08,-3.0474745e-10,1.1215801e-10,5.2305188e-10,0.0059304987,3.7865204e-09,2.1811037e-09,
			-5.2193413e-05,-1.0004565e-05,-8.4295076e-05,1.2679733e-09,-1.5440696e-08,3.8517075e-07,0.00040055855,-0.0033761745,-9.1833198e-07,3.5053799e-05,4.3915511e-06,3.7734597e-05,4.764393e-11,-4.7753609e-13,9.9245211e-13,-2.4868225e-08,-2.0904263e-09,-1.9373529e-08,1.2632595e-09,-9.5849012e-15,-3.4841358e-15,
			-9.9310737e-06,-6.8275422e-06,-1.7857614e-05,2.0873642e-10,-3.2484588e-09,8.860583e-08,0.00011497446,-0.00071234565,-2.1877793e-07,4.391552e-06,1.4868662e-05,8.02877e-06,1.0981568e-11,-2.5259812e-12,-1.6097316e-11,1.5789159e-09,4.6248744e-08,-4.1988049e-09,2.9114403e-10,-1.6613574e-14,5.734329e-14,
			-8.4609637e-05,-1.8044382e-05,-0.00015410148,2.7999416e-09,-2.7306328e-08,6.9245039e-07,0.0007140665,-0.0060229194,-1.6472291e-06,3.7734604e-05,8.0287718e-06,8.0693229e-05,8.5691787e-11,-5.0252007e-12,1.8618813e-12,1.3867394e-08,-4.0680028e-09,1.1219102e-08,2.2720357e-09,-4.2209258e-14,-7.5787324e-15,
			-1.2064039e-10,-2.7283661e-11,-2.1485491e-10,-4.6075817e-11,3.5418923e-13,-0.00020560442,-6.2613125e-08,1.1830777e-07,0.00022740869,4.7643917e-11,1.0981565e-11,8.5691801e-11,0.00017876452,-7.1097529e-12,2.1663819e-12,1.4781077e-14,-1.3607399e-15,-2.5607149e-14,-7.2341277e-07,-1.09218e-13,-4.7586599e-14,
			3.2580223e-11,5.8885041e-12,5.5862717e-11,-4.4899021e-13,-1.7074555e-09,-5.5487128e-08,-1.1509756e-07,-0.00027978868,1.4196908e-07,-4.7753609e-13,-2.5259801e-12,-5.0252007e-12,-7.1097529e-12,0.0001787919,5.6985899e-16,-6.9328143e-16,5.7638106e-15,2.8279379e-15,-1.8819989e-10,-2.3880355e-15,-1.308048e-17,
			-2.2814671e-12,-7.414248e-12,-4.3275986e-12,-1.3904847e-09,3.4312787e-13,1.7319664e-08,-0.00027979049,7.1902662e-08,-7.3106065e-08,9.9245373e-13,-1.6097314e-11,1.8618796e-12,2.1663814e-12,5.6985889e-16,0.0001787919,2.0898471e-16,4.7445158e-14,-3.6455798e-15,5.741025e-11,8.8289801e-18,1.3439716e-15,
			6.3487254e-10,-3.512685e-09,-2.9558374e-08,4.5835731e-13,-5.6603645e-12,1.2008174e-10,1.432452e-07,-1.2102641e-06,-3.0474695e-10,-2.4868228e-08,1.5789133e-09,1.3867391e-08,1.4781043e-14,-6.9328498e-16,2.0898336e-16,9.9999925e-05,4.1744529e-13,-2.6092238e-12,3.9200574e-13,-4.3250296e-18,-1.3602233e-18,
			3.4307397e-09,6.5446514e-08,7.4894366e-09,1.1491337e-12,-5.5193785e-14,-9.4570324e-12,-6.7431635e-08,1.0149302e-07,1.1215842e-10,-2.0904385e-09,4.6248751e-08,-4.0679957e-09,-1.3607556e-15,5.7638131e-15,4.7445151e-14,4.1744079e-13,9.9999677e-05,2.4252794e-11,-3.5834996e-14,6.8417154e-17,-4.2386249e-16,
			4.0895014e-08,8.9165813e-09,1.1793072e-07,-7.6929912e-13,6.2987107e-12,-2.045854e-10,-2.3752563e-07,2.0168261e-06,5.2305127e-10,-1.9373511e-08,-4.1988102e-09,1.1219092e-08,-2.5607119e-14,2.8279292e-15,-3.6455832e-15,-2.609226e-12,2.4252819e-11,9.9999939e-05,-6.7858279e-13,3.3868649e-17,4.1281974e-17,
			-3.1998597e-09,-7.2362438e-10,-5.6987544e-09,-1.2171031e-09,9.4239122e-12,-0.0054481369,-1.4954696e-06,2.9326247e-06,0.0059304987,1.2632593e-09,2.9114386e-10,2.2720361e-09,-7.2341277e-07,-1.8819986e-10,5.7410257e-11,3.920066e-13,-3.5834576e-14,-6.7858365e-13,8.0881771e-05,-2.8937508e-12,-1.2612756e-12,
			2.8258025e-13,3.9348014e-14,4.6463359e-13,-5.530572e-15,-1.5501288e-11,-8.6944979e-10,-2.3161177e-09,-4.8095135e-06,3.7865204e-09,-9.5849079e-15,-1.6613569e-14,-4.2209248e-14,-1.09218e-13,-2.3880358e-15,8.8289818e-18,-4.3250284e-18,6.8417128e-17,3.3868652e-17,-2.8937508e-12,9.9999997e-05,-2.0266737e-19,
			8.3743945e-15,1.1584863e-13,1.7889441e-14,8.9318448e-12,-4.1593752e-15,-3.817513e-10,3.9744978e-06,-1.6270548e-09,2.1811037e-09,-3.4841489e-15,5.734329e-14,-7.5787155e-15,-4.7586599e-14,-1.308048e-17,1.3439715e-15,-1.3602331e-18,-4.2386246e-16,4.1281927e-17,-1.2612756e-12,-2.0266737e-19,9.9999997e-05,
			};

	float32_t PqPlusDataTrue[6*6] = {0.00013303052,2.7752209e-05,0.00023099425,-4.9641159e-05,-1.0396414e-05,-8.5875421e-05,
			2.775221e-05,8.4098911e-06,4.9143207e-05,-1.0288332e-05,-3.4604291e-06,-1.8272052e-05,
			0.00023099424,4.9143207e-05,0.00041162447,-8.5624859e-05,-1.8410265e-05,-0.00015335807,
			-4.964113e-05,-1.0288326e-05,-8.5624815e-05,3.2983997e-05,4.6994792e-06,3.8811035e-05,
			-1.0396408e-05,-3.4604277e-06,-1.8410257e-05,4.6994783e-06,1.2108899e-05,8.3463765e-06,
			-8.5875421e-05,-1.8272051e-05,-0.00015335805,3.8811049e-05,8.3463738e-06,8.0060119e-05,
			};

	arm_mat_init_f32(&xPlusTrue, 22, 1, xPlusTrueData);
	arm_mat_init_f32(&PPlusTrue, 21, 21, PPlusDataTrue);
	arm_mat_init_f32(&PqPlusTrue, 6, 6, PqPlusDataTrue);

	bool test1 = areMatricesEqual(&xPlus, &xPlusTrue);
	bool test2 = areMatricesEqual(&P_plus, &PPlusTrue);

	bool test = (test1 && test2);
	return test;
}

bool test_update_mag(void) {

	arm_matrix_instance_f32 xMinus, P_minus, Pq_minus, Hq, Rq, R, magI, magMeas;

	float32_t xMinusData[22*1] = {0.70761615,
			-0.00047431877,
			0.70659655,
			-0.00072436279,
			35.347885,
			-117.80682,
			672.16156,
			-0.020965694,
			0.011432247,
			-53.228939,
			1.4684781e-05,
			0.00015568511,
			-8.2018505e-06,
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

	float32_t PMinusData[21*21] = {0.00013784027,2.7258824e-05,0.00022864901,-5.1250044e-09,5.297149e-08,-9.824189e-07,-0.0012480157,0.010518468,2.4748194e-06,-5.2193402e-05,-9.9310737e-06,-8.4609637e-05,-1.2064039e-10,3.2580227e-11,-2.2814673e-12,6.3485289e-10,3.4307397e-09,4.0895024e-08,-3.1998597e-09,2.8258022e-13,8.3744005e-15,
			2.7258828e-05,1.466912e-05,4.8711558e-05,-1.198327e-09,1.1266404e-08,-2.2189451e-07,-0.00036169073,0.0022383055,5.7399973e-07,-1.0004565e-05,-6.8275417e-06,-1.8044382e-05,-2.7283661e-11,5.8885041e-12,-7.4142463e-12,-3.5126888e-09,6.5446514e-08,8.9165857e-09,-7.2362438e-10,3.9348011e-14,1.1584864e-13,
			0.00022864902,4.8711558e-05,0.00041280125,-9.0530232e-09,9.3790362e-08,-1.7493462e-06,-0.0022170455,0.018688191,4.4046678e-06,-8.4295083e-05,-1.7857616e-05,-0.00015410148,-2.1485494e-10,5.5862724e-11,-4.327599e-12,-2.9558409e-08,7.489442e-09,1.1793072e-07,-5.6987535e-09,4.6463354e-13,1.7889456e-14,
			-5.1250044e-09,-1.1983268e-09,-9.0530232e-09,4.30622e-06,-2.460236e-12,-3.4269141e-07,8.3658009e-08,-4.5512985e-07,3.345566e-07,1.2679734e-09,2.0873667e-10,2.7999418e-09,-4.6075817e-11,-4.4899018e-13,-1.3904848e-09,4.5835736e-13,1.1491329e-12,-7.692988e-13,-1.2171031e-09,-5.530572e-15,8.9318457e-12,
			5.2971483e-08,1.1266405e-08,9.3790362e-08,-2.4602356e-12,5.2132955e-06,-1.0685863e-09,-5.8123697e-07,4.9049895e-06,1.7745608e-09,-1.5440694e-08,-3.2484575e-09,-2.7306321e-08,3.5418923e-13,-1.7074556e-09,3.4312795e-13,-5.6603632e-12,-5.5196414e-14,6.2987202e-12,9.4239122e-12,-1.5501288e-11,-4.1593756e-15,
			-9.8241901e-07,-2.2189451e-07,-1.7493462e-06,-3.4269146e-07,-1.0685862e-09,0.35937095,0.00011125208,-0.00028825182,-0.3955566,3.8517067e-07,8.8605773e-08,6.9245044e-07,-0.00020560442,-5.5487128e-08,1.7319666e-08,1.20082e-10,-9.4569075e-12,-2.0458565e-10,-0.0054481369,-8.6944973e-10,-3.817513e-10,
			-0.0012480158,-0.00036169071,-0.0022170458,8.3658023e-08,-5.8123703e-07,0.00011125208,0.018988205,-0.11052208,-0.00013661523,0.00040055861,0.00011497449,0.00071406655,-6.2613118e-08,-1.1509756e-07,-0.00027979049,1.4324532e-07,-6.7431799e-08,-2.3752571e-07,-1.4954696e-06,-2.3161177e-09,3.9744978e-06,
			0.010518468,0.0022383055,0.018688189,-4.5512985e-07,4.9049895e-06,-0.00028825182,-0.11052206,0.93232828,0.00043322661,-0.0033761745,-0.00071234547,-0.0060229199,1.1830777e-07,-0.00027978868,7.1902676e-08,-1.2102647e-06,1.0149288e-07,2.0168268e-06,2.9326245e-06,-4.8095135e-06,-1.6270548e-09,
			2.4748199e-06,5.7399961e-07,4.4046674e-06,3.345566e-07,1.7745611e-09,-0.3955566,-0.00013661523,0.00043322655,0.43310961,-9.1833175e-07,-2.187778e-07,-1.6472286e-06,0.00022740869,1.4196908e-07,-7.3106065e-08,-3.0474745e-10,1.1215801e-10,5.2305188e-10,0.0059304987,3.7865204e-09,2.1811037e-09,
			-5.2193413e-05,-1.0004565e-05,-8.4295076e-05,1.2679733e-09,-1.5440696e-08,3.8517075e-07,0.00040055855,-0.0033761745,-9.1833198e-07,3.5053799e-05,4.3915511e-06,3.7734597e-05,4.764393e-11,-4.7753609e-13,9.9245211e-13,-2.4868225e-08,-2.0904263e-09,-1.9373529e-08,1.2632595e-09,-9.5849012e-15,-3.4841358e-15,
			-9.9310737e-06,-6.8275422e-06,-1.7857614e-05,2.0873642e-10,-3.2484588e-09,8.860583e-08,0.00011497446,-0.00071234565,-2.1877793e-07,4.391552e-06,1.4868662e-05,8.02877e-06,1.0981568e-11,-2.5259812e-12,-1.6097316e-11,1.5789159e-09,4.6248744e-08,-4.1988049e-09,2.9114403e-10,-1.6613574e-14,5.734329e-14,
			-8.4609637e-05,-1.8044382e-05,-0.00015410148,2.7999416e-09,-2.7306328e-08,6.9245039e-07,0.0007140665,-0.0060229194,-1.6472291e-06,3.7734604e-05,8.0287718e-06,8.0693229e-05,8.5691787e-11,-5.0252007e-12,1.8618813e-12,1.3867394e-08,-4.0680028e-09,1.1219102e-08,2.2720357e-09,-4.2209258e-14,-7.5787324e-15,
			-1.2064039e-10,-2.7283661e-11,-2.1485491e-10,-4.6075817e-11,3.5418923e-13,-0.00020560442,-6.2613125e-08,1.1830777e-07,0.00022740869,4.7643917e-11,1.0981565e-11,8.5691801e-11,0.00017876452,-7.1097529e-12,2.1663819e-12,1.4781077e-14,-1.3607399e-15,-2.5607149e-14,-7.2341277e-07,-1.09218e-13,-4.7586599e-14,
			3.2580223e-11,5.8885041e-12,5.5862717e-11,-4.4899021e-13,-1.7074555e-09,-5.5487128e-08,-1.1509756e-07,-0.00027978868,1.4196908e-07,-4.7753609e-13,-2.5259801e-12,-5.0252007e-12,-7.1097529e-12,0.0001787919,5.6985899e-16,-6.9328143e-16,5.7638106e-15,2.8279379e-15,-1.8819989e-10,-2.3880355e-15,-1.308048e-17,
			-2.2814671e-12,-7.414248e-12,-4.3275986e-12,-1.3904847e-09,3.4312787e-13,1.7319664e-08,-0.00027979049,7.1902662e-08,-7.3106065e-08,9.9245373e-13,-1.6097314e-11,1.8618796e-12,2.1663814e-12,5.6985889e-16,0.0001787919,2.0898471e-16,4.7445158e-14,-3.6455798e-15,5.741025e-11,8.8289801e-18,1.3439716e-15,
			6.3487254e-10,-3.512685e-09,-2.9558374e-08,4.5835731e-13,-5.6603645e-12,1.2008174e-10,1.432452e-07,-1.2102641e-06,-3.0474695e-10,-2.4868228e-08,1.5789133e-09,1.3867391e-08,1.4781043e-14,-6.9328498e-16,2.0898336e-16,9.9999925e-05,4.1744529e-13,-2.6092238e-12,3.9200574e-13,-4.3250296e-18,-1.3602233e-18,
			3.4307397e-09,6.5446514e-08,7.4894366e-09,1.1491337e-12,-5.5193785e-14,-9.4570324e-12,-6.7431635e-08,1.0149302e-07,1.1215842e-10,-2.0904385e-09,4.6248751e-08,-4.0679957e-09,-1.3607556e-15,5.7638131e-15,4.7445151e-14,4.1744079e-13,9.9999677e-05,2.4252794e-11,-3.5834996e-14,6.8417154e-17,-4.2386249e-16,
			4.0895014e-08,8.9165813e-09,1.1793072e-07,-7.6929912e-13,6.2987107e-12,-2.045854e-10,-2.3752563e-07,2.0168261e-06,5.2305127e-10,-1.9373511e-08,-4.1988102e-09,1.1219092e-08,-2.5607119e-14,2.8279292e-15,-3.6455832e-15,-2.609226e-12,2.4252819e-11,9.9999939e-05,-6.7858279e-13,3.3868649e-17,4.1281974e-17,
			-3.1998597e-09,-7.2362438e-10,-5.6987544e-09,-1.2171031e-09,9.4239122e-12,-0.0054481369,-1.4954696e-06,2.9326247e-06,0.0059304987,1.2632593e-09,2.9114386e-10,2.2720361e-09,-7.2341277e-07,-1.8819986e-10,5.7410257e-11,3.920066e-13,-3.5834576e-14,-6.7858365e-13,8.0881771e-05,-2.8937508e-12,-1.2612756e-12,
			2.8258025e-13,3.9348014e-14,4.6463359e-13,-5.530572e-15,-1.5501288e-11,-8.6944979e-10,-2.3161177e-09,-4.8095135e-06,3.7865204e-09,-9.5849079e-15,-1.6613569e-14,-4.2209248e-14,-1.09218e-13,-2.3880358e-15,8.8289818e-18,-4.3250284e-18,6.8417128e-17,3.3868652e-17,-2.8937508e-12,9.9999997e-05,-2.0266737e-19,
			8.3743945e-15,1.1584863e-13,1.7889441e-14,8.9318448e-12,-4.1593752e-15,-3.817513e-10,3.9744978e-06,-1.6270548e-09,2.1811037e-09,-3.4841489e-15,5.734329e-14,-7.5787155e-15,-4.7586599e-14,-1.308048e-17,1.3439715e-15,-1.3602331e-18,-4.2386246e-16,4.1281927e-17,-1.2612756e-12,-2.0266737e-19,9.9999997e-05
			};

	float32_t PqMinusData[6*6] = {0.00013303052,2.7752209e-05,0.00023099425,-4.9641159e-05,-1.0396414e-05,-8.5875421e-05,
			2.775221e-05,8.4098911e-06,4.9143207e-05,-1.0288332e-05,-3.4604291e-06,-1.8272052e-05,
			0.00023099424,4.9143207e-05,0.00041162447,-8.5624859e-05,-1.8410265e-05,-0.00015335807,
			-4.964113e-05,-1.0288326e-05,-8.5624815e-05,3.2983997e-05,4.6994792e-06,3.8811035e-05,
			-1.0396408e-05,-3.4604277e-06,-1.8410257e-05,4.6994783e-06,1.2108899e-05,8.3463765e-06,
			-8.5875421e-05,-1.8272051e-05,-0.00015335805,3.8811049e-05,8.3463738e-06,8.0060119e-05
			};

	float32_t HqData[3*6] = {0,-0.866,0.104,0,0,0,
			0.866,0,-0.4891,0,0,0,
			-0.104,0.4891,0,0,0,0
			};

	float32_t RqData[3*3] = {3.2e-07,0,0,
			0,4.1e-07,0,
			0,0,3.2e-07
			};

	float32_t RData[3*3] = {1.35e-05,0,0,
			0,1.65e-05,0,
			0,0,2
			};

	float32_t magIData[3*1] = {0.4891,
			0.104,
			0.866
			};

	float32_t magMeasData[3*1] = {-0.86552111,
			0.10263403,
			0.49023479
			};

	arm_mat_init_f32(&xMinus, 22, 1, xMinusData);
	arm_mat_init_f32(&P_minus, 21, 21, PMinusData);
	arm_mat_init_f32(&Pq_minus, 6, 6, PqMinusData);
	arm_mat_init_f32(&Hq, 3, 6, HqData);
	arm_mat_init_f32(&Rq, 3, 3, RqData);
	arm_mat_init_f32(&R, 3, 3, RData);
	arm_mat_init_f32(&magI, 3, 1, magIData);
	arm_mat_init_f32(&magMeas, 3, 1, magMeasData);

	arm_matrix_instance_f32 xPlus, Pplus, PqPlus;
	float32_t xPlusData[22*1], PPlusData[21*21], PqPlusData[6*6];

	update_mag(&xMinus, &P_minus, &Pq_minus,
			   &Hq, &Rq, &R, &magI, &magMeas,
			   &xPlus, &Pplus, &PqPlus,
			   xPlusData, PPlusData, PqPlusData);

	float32_t xPlusDataTrue[22*1] = {0.70758718,
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

	float32_t PqPlusDataTrue[6*6] = {0.00013135246,2.7859951e-05,0.00023195099,-4.8803966e-05,-1.0450017e-05,-8.6351567e-05,
			2.7859951e-05,6.2080526e-06,4.9329508e-05,-1.0342371e-05,-2.3635407e-06,-1.8365537e-05,
			0.00023195098,4.9329505e-05,0.00041105095,-8.6102147e-05,-1.8503115e-05,-0.00015307259,
			-4.8803933e-05,-1.0342366e-05,-8.6102104e-05,3.2566317e-05,4.7263657e-06,3.9048573e-05,
			-1.045001e-05,-2.36354e-06,-1.8503106e-05,4.7263648e-06,1.1562463e-05,8.3929672e-06,
			-8.6351567e-05,-1.8365537e-05,-0.00015307257,3.9048584e-05,8.3929663e-06,7.9917998e-05,
			};

	float32_t PPlusDataTrue[21*21] = {0.0001355586,2.7415161e-05,0.000229817,-5.1229891e-09,5.2971384e-08,-9.8400062e-07,-0.0012500731,0.010527903,2.4786459e-06,-5.0991275e-05,-1.0019453e-05,-8.5239757e-05,-1.2084031e-10,3.2302057e-11,-2.3945904e-12,-3.903414e-09,4.3002144e-09,4.7575128e-08,-3.205155e-09,2.7713069e-13,9.9902056e-15,
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

	arm_matrix_instance_f32 xPlusTrue, PplusTrue, PqPlusTrue;

	arm_mat_init_f32(&xPlusTrue, 22, 1, xPlusDataTrue);
	arm_mat_init_f32(&PqPlusTrue, 6, 6, PqPlusDataTrue);
	arm_mat_init_f32(&PplusTrue, 21, 21, PPlusDataTrue);

	bool test1 = areMatricesEqual(&xPlusTrue, &xPlus);
	bool test2 = areMatricesEqual(&PqPlusTrue, &PqPlus);
	bool test3 = areMatricesEqual(&PplusTrue, &Pplus);

	bool test = (test1 && test2) && test3;
	return;
}

bool test_update_baro(void) {

	arm_matrix_instance_f32 xMinus, P_minus, Hq, Rq, R, magI, magMeas;

	float32_t xMinusData[22*1] = {-0.83571309,
			-0.34926426,
			-0.41556987,
			-0.083064094,
			35.406746,
			-117.91042,
			46733.414,
			30.471273,
			-12.752536,
			374.76144,
			0.00017442767,
			-0.00016044689,
			-0.00027719577,
			-0.021986734,
			-0.011906922,
			0.024403248,
			-0.0016164053,
			0.00024158983,
			0.00089838408,
			0.0011755408,
			0.00042544794,
			0.00025845651
			};

	float32_t PMinusData[21*21] = {0.0030647744,0.00066360191,0.0054201391,-2.3731023e-05,2.1753893e-05,-0.68233764,-0.20764889,0.38383338,0.10987093,-4.21564e-05,0.00044757052,-0.00019700336,-1.585046e-06,-3.3155814e-07,1.5681717e-07,4.3650234e-06,-5.3629547e-06,-2.4337751e-06,1.9275472e-08,1.6002925e-08,1.0230469e-08,
			0.00066360197,0.0001515972,0.001176713,-5.0926637e-06,4.6895238e-06,-0.14696237,-0.044630866,0.082911052,0.023712525,-9.2766313e-06,9.4866147e-05,-4.3707551e-05,-3.4234461e-07,-7.2203939e-08,3.3701557e-08,9.3003013e-07,-1.2115876e-06,-5.8552547e-07,4.1382147e-09,3.4663001e-09,2.1997648e-09,
			0.0054201391,0.0011767132,0.0096152881,-4.2085147e-05,3.8572416e-05,-1.2098722,-0.36821842,0.68052942,0.19480495,-7.1438037e-05,0.00079350418,-0.00035071658,-2.8138982e-06,-5.91608e-07,2.7976179e-07,7.7454179e-06,-9.5163714e-06,-4.3643254e-06,3.4220744e-08,2.8442615e-08,1.8045926e-08,
			-2.3730998e-05,-5.0926587e-06,-4.2085107e-05,8.8023362e-06,7.7738532e-06,0.043349825,0.0092511754,0.0021072451,-0.0021531468,9.6654105e-08,-3.7360819e-06,1.0486834e-06,-2.4150004e-05,2.4192514e-05,-3.5807577e-07,-9.470337e-07,-1.728781e-07,-1.2265444e-07,-3.0521497e-07,2.2327422e-08,2.5767767e-08,
			2.1753891e-05,4.6895234e-06,3.8572412e-05,7.7738814e-06,4.6193953e-05,-0.2000972,0.0043019331,0.031417273,0.0039068335,-1.7385813e-07,3.3344747e-06,-1.1296589e-06,2.3191331e-05,2.3328361e-05,-3.7073878e-05,5.7910802e-07,-1.7183654e-08,-3.7246215e-07,-1.5401648e-08,-2.1724109e-07,-1.9434783e-07,
			-0.68233752,-0.14696234,-1.2098721,0.043350521,-0.2000961,93725.672,510.4072,-66.981194,-1594.8839,0.00056024868,-0.10504341,0.034848649,-16.639101,-0.96910888,-0.60441923,0.024049964,-0.0083339857,-0.05127871,0.0023138451,-0.072405219,-0.040680911,
			-0.20764868,-0.044630814,-0.368218,0.0092511745,0.0043019122,510.40659,26.826317,-23.155752,-16.990402,0.00041500328,-0.032452557,0.0096029509,-0.13769753,0.09011969,0.00045077837,-0.0010296248,0.00029009019,-0.00034967158,-0.00023165623,0.0012623942,0.0034773906,
			0.38383341,0.082911044,0.68052942,0.002107257,0.031417273,-66.981911,-23.155764,69.001656,13.765761,-0.0029357704,0.058050055,-0.021303594,-0.03184386,-0.0090569705,-0.087171502,0.00093832944,-0.00078971329,-0.00035418654,-1.61362e-05,-0.00022481137,-0.00051307247,
			0.10987088,0.023712518,0.19480489,-0.0021531582,0.0039068223,-1594.8839,-16.990412,13.765747,32.54657,-0.00068885955,0.016708754,-0.0059335283,0.33149007,0.033590145,-0.045513172,-0.00016896127,-8.4929081e-05,0.00075534609,-2.9594636e-05,0.002273072,0.003482247,
			-4.2156316e-05,-9.2766295e-06,-7.1437855e-05,9.6655079e-08,-1.7385814e-07,0.00056028663,0.00041499356,-0.0029357763,-0.00068886374,9.5926325e-06,-3.4075063e-06,7.2178655e-06,2.6379385e-07,4.2472138e-08,-5.8667875e-09,-1.7353382e-06,-1.0516169e-07,2.9208513e-07,-3.7168142e-09,-2.6230076e-09,-1.8905306e-09,
			0.00044757061,9.486614e-05,0.00079350424,-3.736084e-06,3.3344752e-06,-0.10504341,-0.03245258,0.058050044,0.016708761,-3.407557e-06,7.3652795e-05,-2.4855719e-05,-2.5773829e-07,-3.8661188e-08,2.0701584e-08,7.1919033e-07,-4.7566405e-07,-1.096489e-07,2.9864093e-09,2.5489582e-09,1.9447124e-09,
			-0.00019700314,-4.37075e-05,-0.00035071617,1.0486799e-06,-1.1296571e-06,0.034848619,0.0096029239,-0.021303561,-0.0059335297,7.2178859e-06,-2.4855704e-05,2.5959553e-05,2.9424095e-08,1.1740777e-08,-1.3180105e-09,-1.3988708e-07,1.2052361e-07,-4.9684282e-08,-5.8087563e-10,-3.8484249e-10,-2.1390262e-10,
			-1.585042e-06,-3.4234381e-07,-2.8138918e-06,-2.4150066e-05,2.3191242e-05,-16.639101,-0.13769759,-0.031843919,0.33149007,2.6379161e-07,-2.5773778e-07,2.9423909e-08,0.0055051036,1.743627e-05,-5.6583736e-05,-2.6953496e-06,1.3812743e-06,6.6867342e-06,-2.4886415e-07,8.1864873e-06,3.246952e-06,
			-3.3155808e-07,-7.2203918e-08,-5.9160794e-07,2.419249e-05,2.3328323e-05,-0.96910888,0.090119675,-0.0090570003,0.033590145,4.2472045e-08,-3.8661213e-08,1.1740847e-08,1.7436258e-05,0.0068509639,8.7012477e-06,-4.0033063e-07,1.0878141e-07,2.1337756e-07,9.9352228e-08,8.9436085e-09,1.0820769e-07,
			1.5681927e-07,3.3701991e-08,2.7976554e-07,-3.5802717e-07,-3.7073838e-05,-0.60441959,0.00045082261,-0.087171473,-0.045513168,-5.8673089e-09,2.0701941e-08,-1.3180822e-09,-5.6583664e-05,8.7012586e-06,0.0068328031,2.2644228e-07,-2.7424948e-08,-2.5481023e-07,-5.6275482e-08,9.1986813e-08,3.4536072e-08,
			4.365043e-06,9.3003422e-07,7.7454524e-06,-9.4704887e-07,5.7911302e-07,0.024050185,-0.0010296372,0.00093833555,-0.00016896328,-1.7353384e-06,7.1919362e-07,-1.3988793e-07,-2.6953726e-06,-4.0033194e-07,2.2643727e-07,1.8039656e-05,3.136345e-07,2.8693063e-07,3.7907292e-08,2.864804e-08,2.1935129e-08,
			-5.3629415e-06,-1.2115846e-06,-9.5163477e-06,-1.7287871e-07,-1.7186073e-08,-0.0083339773,0.00029008914,-0.00078971364,-8.4929125e-05,-1.0516214e-07,-4.7566209e-07,1.205228e-07,1.381273e-06,1.0878107e-07,-2.7424392e-08,3.13633e-07,8.0119513e-05,-2.052882e-07,-4.4521467e-09,-9.7856896e-09,-6.958782e-09,
			-2.4337744e-06,-5.8552519e-07,-4.3643249e-06,-1.2265585e-07,-3.7244962e-07,-0.051278781,-0.00034967304,-0.00035417828,0.00075534609,2.9208519e-07,-1.096494e-07,-4.9685099e-08,6.6867433e-06,2.1337779e-07,-2.5480651e-07,2.8693174e-07,-2.0528496e-07,8.9861242e-05,-3.5456143e-08,-5.0010158e-08,-2.2323043e-08,
			1.9275527e-08,4.1382253e-09,3.4220839e-08,-3.0521457e-07,-1.5398662e-08,0.0023138451,-0.00023165584,-1.6134338e-05,-2.9594634e-05,-3.7168115e-09,2.9864207e-09,-5.8088256e-10,-2.488639e-07,9.935237e-08,-5.6274889e-08,3.7907302e-08,-4.4522079e-09,-3.5455493e-08,3.2254484e-08,-3.3368899e-08,-1.4308755e-08,
			1.6002897e-08,3.4662946e-09,2.8442569e-08,2.2328319e-08,-2.1723982e-07,-0.072405219,0.0012623951,-0.00022481044,0.002273072,-2.6229903e-09,2.5489526e-09,-3.848423e-10,8.1864864e-06,8.9436778e-09,9.1987246e-08,2.8647857e-08,-9.7857011e-09,-5.0010108e-08,-3.3368909e-08,9.9945595e-05,-2.1836797e-08,
			1.0230456e-08,2.1997622e-09,1.8045899e-08,2.5768221e-08,-1.9434711e-07,-0.040680911,0.0034773911,-0.00051307207,0.003482247,-1.8905162e-09,1.9447102e-09,-2.1390098e-10,3.2469525e-06,1.0820766e-07,3.4536296e-08,2.1934987e-08,-6.9587891e-09,-2.2322975e-08,-1.4308762e-08,-2.1836792e-08,9.9987752e-05,
			};

	float32_t Rb = 0.0025f;
	float32_t pressMeas = 0.00127722;

	arm_mat_init_f32(&xMinus, 22, 1, xMinusData);
	arm_mat_init_f32(&P_minus, 21, 21, PMinusData);

	arm_matrix_instance_f32 xPlus, Pplus;
	float32_t xPlusData[22*1], PPlusData[21*21];

	update_baro(&xMinus, &P_minus, pressMeas, Rb, &xPlus, &Pplus, xPlusData, PPlusData);

	float32_t xPlusTrueData[22*1] = {-0.83571309,
			-0.34926426,
			-0.41556987,
			-0.083064094,
			35.410431,
			-117.92744,
			54701.93,
			73.865829,
			-18.447309,
			239.16508,
			0.00017442767,
			-0.00016044689,
			-0.00027719577,
			-1.4366361,
			-0.094300151,
			-0.026984252,
			0.00042833132,
			-0.00046696147,
			-0.0034613167,
			0.001372263,
			-0.0057304138,
			-0.0032002174,
			};

	float32_t PPlusTrueData[21*21] = {0.0030598075,0.00066253211,0.005411332,-2.3415458e-05,2.0297311e-05,-7.0133319e-05,-0.20393343,0.38334578,0.09826111,-4.2152322e-05,0.00044680588,-0.00019674968,-0.00012270786,-7.3860983e-06,-4.2429979e-06,4.540093e-06,-5.4236211e-06,-2.8070538e-06,3.6118898e-08,-5.1106429e-07,-2.859025e-07,
			0.00066253217,0.00015136678,0.0011748162,-5.0246967e-06,4.3758046e-06,-1.5105367e-05,-0.043830626,0.082806036,0.021211995,-9.2757527e-06,9.4701456e-05,-4.3652912e-05,-2.6429867e-05,-1.591616e-06,-9.1393377e-07,9.677367e-07,-1.2246539e-06,-6.6592253e-07,7.7659639e-09,-1.1005383e-07,-6.1581581e-08,
			0.005411332,0.0011748163,0.0095996717,-4.1525607e-05,3.5989709e-05,-0.00012435539,-0.36163044,0.67966485,0.17421927,-7.1430804e-05,0.00079214835,-0.00035026678,-0.00021758022,-1.3100213e-05,-7.5216753e-06,8.0558384e-06,-9.623941e-06,-5.0261965e-06,6.4086294e-08,-9.0611519e-07,-5.0703591e-07,
			-2.3415436e-05,-5.0246931e-06,-4.1525574e-05,8.7822882e-06,7.8663916e-06,4.4556637e-06,0.0090151271,0.0021382219,-0.0014155598,9.6395006e-08,-3.6875024e-06,1.032567e-06,-1.6454909e-05,2.4640698e-05,-7.8549654e-08,-9.5815608e-07,-1.6902388e-07,-9.8939545e-08,-3.0628505e-07,5.5812706e-08,4.458149e-08,
			2.0297302e-05,4.3758023e-06,3.5989691e-05,7.8664216e-06,4.5766807e-05,-2.0566771e-05,0.0053915023,0.031274289,0.0005022274,-1.7266215e-07,3.1102381e-06,-1.0552674e-06,-1.2328237e-05,2.1259599e-05,-3.8364135e-05,6.3044757e-07,-3.4974249e-08,-4.8192703e-07,-1.0462273e-08,-3.7180484e-07,-2.8118956e-07,
			-7.0133319e-05,-1.5105364e-05,-0.00012435539,4.455736e-06,-2.056666e-05,9.6334906,0.052461646,-0.0068845889,-0.1639284,5.7584547e-08,-1.0796772e-05,3.5818803e-06,-0.0017102319,-9.9608798e-05,-6.2124571e-05,2.4719491e-06,-8.5659957e-07,-5.2706259e-06,2.3782604e-07,-7.4420914e-06,-4.1813432e-06,
			-0.20393322,-0.043830577,-0.36163002,0.0090151224,0.0053914734,0.052461583,24.047052,-22.791027,-8.3059559,0.00041195261,-0.031880576,0.009413193,-0.047094464,0.095396675,0.0037419561,-0.0011605815,0.00033547034,-7.0449256e-05,-0.00024425556,0.0016566544,0.0036989059,
			0.38334581,0.082806028,0.67966485,0.0021382347,0.031274289,-0.0068846615,-22.791035,68.953789,12.626081,-0.0029353702,0.057974994,-0.021278692,-0.043733925,-0.0097494815,-0.087603413,0.00095551519,-0.00079566863,-0.00039082958,-1.448276e-05,-0.00027655112,-0.00054214249,
			0.098261073,0.021211989,0.17421921,-0.0014155593,0.00050223473,-0.1639284,-8.3059549,12.626078,5.4099994,-0.00067932706,0.014921466,-0.0053405869,0.048379723,0.017100988,-0.055797219,0.00024024314,-0.00022672986,-0.00011714899,9.7748871e-06,0.0010411144,0.0027900711,
			-4.2152238e-05,-9.2757509e-06,-7.1430622e-05,9.6395958e-08,-1.726621e-07,5.7588441e-08,0.00041194269,-0.0029353758,-0.00067933061,9.5926289e-06,-3.4068785e-06,7.2176572e-06,3.6325122e-07,4.826482e-08,-2.2539759e-09,-1.7354819e-06,-1.0511187e-07,2.9239163e-07,-3.7306447e-09,-2.1902178e-09,-1.6473675e-09,
			0.00044680596,9.4701449e-05,0.00079214841,-3.6875037e-06,3.1102397e-06,-1.0796771e-05,-0.031880599,0.057974983,0.014921473,-3.4069292e-06,7.3535077e-05,-2.4816667e-05,-1.8904158e-05,-1.1246821e-06,-6.5663397e-07,7.4614167e-07,-4.8500345e-07,-1.6711381e-07,5.5793938e-09,-7.8591128e-08,-4.364389e-08,
			-0.00019674946,-4.3652861e-05,-0.00035026637,1.0325632e-06,-1.055266e-06,3.5818769e-06,0.009413166,-0.021278659,-0.0053405887,7.2176776e-06,-2.4816653e-05,2.5946596e-05,6.2154563e-06,3.7203301e-07,2.2339107e-07,-1.4882829e-07,1.2362199e-07,-3.0620047e-08,-1.4411097e-09,2.6533742e-08,1.491032e-08,
			-0.00012270783,-2.642986e-05,-0.00021758018,-1.6454847e-05,-1.232813e-05,-0.0017102316,-0.047094416,-0.043733854,0.048379738,3.6324221e-07,-1.8904157e-05,6.2154622e-06,0.0025514709,-0.00015459178,-0.00016387513,1.5737968e-06,-9.8104522e-08,-2.4158292e-06,1.6187009e-07,-4.6662758e-06,-3.97438e-06,
			-7.386097e-06,-1.5916154e-06,-1.3100211e-05,2.4640682e-05,2.1259573e-05,-9.9608791e-05,0.09539666,-0.0097495038,0.01710099,4.8264333e-08,-1.1246821e-06,3.7203341e-07,-0.00015459179,0.0068409443,2.4522894e-06,-1.5168332e-07,2.2618149e-08,-3.1678181e-07,1.2327456e-07,-7.396381e-07,-3.1238338e-07,
			-4.2429974e-06,-9.139336e-07,-7.5216744e-06,-7.8496399e-08,-3.8364087e-05,-6.21246e-05,0.0037420061,-0.087603375,-0.055797223,-2.2547397e-09,-6.5663397e-07,2.2339132e-07,-0.00016387513,2.4522967e-06,0.0068289055,3.8152012e-07,-8.1163762e-08,-5.8546323e-07,-4.1355456e-08,-3.7489312e-07,-2.2778067e-07,
			4.5401139e-06,9.6774113e-07,8.0558757e-06,-9.5817154e-07,6.304528e-07,2.4719718e-06,-0.0011605951,0.0009555213,0.0002402449,-1.7354821e-06,7.4614519e-07,-1.4882924e-07,1.5738135e-06,-1.5168231e-07,3.8151643e-07,1.8033486e-05,3.1577281e-07,3.0008749e-07,3.7313615e-08,4.7225452e-08,3.2372853e-08,
			-5.4236079e-06,-1.224651e-06,-9.6239173e-06,-1.6902443e-07,-3.4976559e-08,-8.5659877e-07,0.00033546929,-0.00079566892,-0.00022672977,-1.0511233e-07,-4.8500146e-07,1.2362118e-07,-9.8104387e-08,2.2617895e-08,-8.116313e-08,3.1577127e-07,8.0118771e-05,-2.0984737e-07,-4.2464232e-09,-1.6223217e-08,-1.0575709e-08,
			-2.8070535e-06,-6.659223e-07,-5.026197e-06,-9.8940539e-08,-4.8191407e-07,-5.2706332e-06,-7.0449976e-05,-0.00039082096,-0.00011715021,2.9239169e-07,-1.6711439e-07,-3.0620818e-08,-2.4158335e-06,-3.1678238e-07,-5.854597e-07,3.0008849e-07,-2.0984415e-07,8.9833185e-05,-3.4190332e-08,-8.9620116e-08,-4.4577916e-08,
			3.6118951e-08,7.7659736e-09,6.4086393e-08,-3.0628468e-07,-1.0459313e-08,2.3782604e-07,-0.00024425521,-1.4480915e-05,9.774888e-06,-3.7306411e-09,5.5794049e-09,-1.4411174e-09,1.6187035e-07,1.232747e-07,-4.1354873e-08,3.7313633e-08,-4.2464845e-09,-3.4189682e-08,3.2197367e-08,-3.1581585e-08,-1.3304551e-08,
			-5.1106423e-07,-1.100538e-07,-9.0611519e-07,5.5814144e-08,-3.7180274e-07,-7.4420914e-06,0.0016566557,-0.00027654963,0.0010411144,-2.1902298e-09,-7.8591142e-08,2.6533767e-08,-4.6662776e-06,-7.396381e-07,-3.7489241e-07,4.7225097e-08,-1.6223234e-08,-8.9620009e-08,-3.1581596e-08,9.9889665e-05,-5.32605e-08,
			-2.8590247e-07,-6.1581559e-08,-5.0703591e-07,4.4582254e-08,-2.8118836e-07,-4.1813428e-06,0.0036989066,-0.00054214173,0.0027900711,-1.6473696e-09,-4.3643894e-08,1.4910334e-08,-3.97438e-06,-3.1238343e-07,-2.2778029e-07,3.2372618e-08,-1.057572e-08,-4.4577817e-08,-1.3304558e-08,-5.3260496e-08,9.9970093e-05,
			};

	arm_matrix_instance_f32 xPlusTrue, PplusTrue;
	arm_mat_init_f32(&xPlusTrue, 22, 1, xPlusTrueData);
	arm_mat_init_f32(&PplusTrue, 21, 21, PPlusTrueData);

	bool test1 = areMatricesEqual(&xPlusTrue, &xPlus);
	bool test2 = areMatricesEqual(&PplusTrue, &Pplus);
	return;
}

bool test_compute_eigen(void) {

	float A[4] = {
	        4, 1,
	        2, 3
	    };

	    float dr[2];      // Real parts of eigenvalues
	    float di[2];      // Imag parts (will be 0 for this real matrix)
	    float wr[4];      // Real eigenvectors
	    float wi[4];      // Imag eigenvectors (0)

	    size_t n = 2;

	    bool success = eig(A, dr, di, wr, wi, n);
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
    printf("Failed:       %lu\n", (unsigned long)g_test_stats.failed_tests);
    printf("Success rate: %.1f%%\n",
           g_test_stats.total_tests > 0 ? (100.0f * g_test_stats.passed_tests / g_test_stats.total_tests) : 0.0f);
    printf("========================================\n");
    printf("\n");
}

test_stats_t get_test_stats(void)
{
    return g_test_stats;
}
