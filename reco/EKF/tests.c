#include "tests.h"
#include "float.h"
#include "Inc/ekf.h"
#include "Inc/ekf_utils.h"

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

	arm_matrix_instance_f32 G;
	float32_t GBuff[21*12];

	compute_G(&g_sf, &a_sf, &q, &gBias, GBuff);
	return true;
}

bool test_compute_Pdot(arm_matrix_instance_f32* Q) {

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

	float32_t PBuff[21*21] = {
	    0.0001366, 0.0000271, 0.0002270, -0.0000000, 0.0000001, -0.0000012, -0.0012330, 0.0103915, 0.0000028, -0.0000518, -0.0000099, -0.0000842, 0.0000000, 0.0000000, -0.0000000, -0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000,
	    0.0000271, 0.0000143, 0.0000484, -0.0000000, 0.0000000, -0.0000003, -0.0003548, 0.0022119, 0.0000006, -0.0000100, -0.0000067, -0.0000180, 0.0000000, 0.0000000, -0.0000000, -0.0000000, 0.0000001, 0.0000000, 0.0000000, 0.0000000, 0.0000000,
	    0.0002270, 0.0000484, 0.0004095, -0.0000000, 0.0000001, -0.0000021, -0.0021905, 0.0184620, 0.0000050, -0.0000839, -0.0000178, -0.0001533, 0.0000000, 0.0000000, -0.0000000, -0.0000000, 0.0000000, 0.0000001, 0.0000000, 0.0000000, 0.0000000,
	    -0.0000000, -0.0000000, -0.0000000, 0.0000063, -0.0000000, -0.0000006, 0.0000001, -0.0000007, 0.0000006, 0.0000000, 0.0000000, 0.0000000, -0.0000000, -0.0000000, -0.0000000, 0.0000000, 0.0000000, -0.0000000, -0.0000000, -0.0000000, 0.0000000,
	    0.0000001, 0.0000000, 0.0000001, -0.0000000, 0.0000076, -0.0000000, -0.0000008, 0.0000070, 0.0000000, -0.0000000, -0.0000000, -0.0000000, 0.0000000, -0.0000000, 0.0000000, -0.0000000, -0.0000000, 0.0000000, 0.0000000, -0.0000000, -0.0000000,
	    -0.0000012, -0.0000003, -0.0000021, -0.0000006, -0.0000000, 0.4286034, 0.0001302, -0.0003404, -0.4742956, 0.0000005, 0.0000001, 0.0000008, -0.0002479, -0.0000001, 0.0000000, 0.0000000, -0.0000000, -0.0000000, -0.0065695, -0.0000000, -0.0000000,
	    -0.0012330, -0.0003548, -0.0021905, 0.0000001, -0.0000008, 0.0001302, 0.0186931, -0.1087340, -0.0001592, 0.0003965, 0.0001123, 0.0007068, -0.0000001, -0.0000001, -0.0002780, 0.0000001, -0.0000000, -0.0000002, -0.0000018, -0.0000000, 0.0000039,
	    0.0103915, 0.0022119, 0.0184620, -0.0000007, 0.0000070, -0.0003404, -0.1087340, 0.9172560, 0.0005006, -0.0033419, -0.0007051, -0.0059604, 0.0000002, -0.0002780, 0.0000001, -0.0000012, 0.0000001, 0.0000020, 0.0000038, -0.0000048, -0.0000000,
	    0.0000028, 0.0000006, 0.0000050, 0.0000006, 0.0000000, -0.4742956, -0.0001592, 0.0005006, 0.5226001, -0.0000010, -0.0000002, -0.0000019, 0.0002752, 0.0000002, -0.0000001, -0.0000000, 0.0000000, 0.0000000, 0.0072037, 0.0000000, 0.0000000,
	    -0.0000518, -0.0000100, -0.0000839, 0.0000000, -0.0000000, 0.0000005, 0.0003965, -0.0033419, -0.0000010, 0.0000350, 0.0000044, 0.0000377, -0.0000000, 0.0000000, 0.0000000, -0.0000000, -0.0000000, -0.0000000, -0.0000000, 0.0000000, -0.0000000,
	    -0.0000099, -0.0000067, -0.0000178, 0.0000000, -0.0000000, 0.0000001, 0.0001123, -0.0007051, -0.0000002, 0.0000044, 0.0000149, 0.0000080, -0.0000000, -0.0000000, -0.0000000, 0.0000000, 0.0000000, -0.0000000, -0.0000000, -0.0000000, 0.0000000,
	    -0.0000842, -0.0000180, -0.0001533, 0.0000000, -0.0000000, 0.0000008, 0.0007068, -0.0059604, -0.0000019, 0.0000377, 0.0000080, 0.0000807, -0.0000000, -0.0000000, 0.0000000, 0.0000000, -0.0000000, 0.0000000, -0.0000000, -0.0000000, -0.0000000,
	    0.0000000, 0.0000000, 0.0000000, -0.0000000, 0.0000000, -0.0002479, -0.0000001, 0.0000002, 0.0002752, -0.0000000, -0.0000000, -0.0000000, 0.0001784, -0.0000000, -0.0000000, -0.0000000, -0.0000000, 0.0000000, -0.0000000, -0.0000000, 0.0000000,
	    0.0000000, 0.0000000, 0.0000000, -0.0000000, -0.0000000, -0.0000001, -0.0000001, -0.0002780, 0.0000002, 0.0000000, -0.0000000, -0.0000000, -0.0000000, 0.0001784, -0.0000000, 0.0000000, 0.0000000, 0.0000000, -0.0000000, -0.0000000, 0.0000000,
	    -0.0000000, -0.0000000, -0.0000000, -0.0000000, 0.0000000, 0.0000000, -0.0002780, 0.0000001, -0.0000001, 0.0000000, -0.0000000, 0.0000000, -0.0000000, -0.0000000, 0.0001784, 0.0000000, 0.0000000, -0.0000000, -0.0000000, -0.0000000, -0.0000000,
	    -0.0000000, -0.0000000, -0.0000000, 0.0000000, -0.0000000, 0.0000000, 0.0000001, -0.0000012, -0.0000000, -0.0000000, 0.0000000, 0.0000000, -0.0000000, 0.0000000, 0.0000000, 0.0001000, 0.0000000, -0.0000000, -0.0000000, 0.0000000, -0.0000000,
	    0.0000000, 0.0000001, 0.0000000, 0.0000000, -0.0000000, -0.0000000, -0.0000000, 0.0000001, 0.0000000, -0.0000000, 0.0000000, -0.0000000, -0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0001000, 0.0000000, -0.0000000, 0.0000000, -0.0000000,
	    0.0000000, 0.0000000, 0.0000001, -0.0000000, 0.0000000, -0.0000000, -0.0000002, 0.0000020, 0.0000000, -0.0000000, -0.0000000, 0.0000000, 0.0000000, 0.0000000, -0.0000000, -0.0000000, 0.0000000, 0.0001000, 0.0000000, 0.0000000, 0.0000000,
	    0.0000000, 0.0000000, 0.0000000, -0.0000000, 0.0000000, -0.0065695, -0.0000018, 0.0000038, 0.0072037, -0.0000000, -0.0000000, -0.0000000, -0.0000000, -0.0000000, -0.0000000, -0.0000000, -0.0000000, 0.0000000, 0.0000990, -0.0000000, 0.0000000,
	    0.0000000, 0.0000000, 0.0000000, -0.0000000, -0.0000000, -0.0000000, -0.0000000, -0.0000048, 0.0000000, 0.0000000, -0.0000000, -0.0000000, -0.0000000, -0.0000000, -0.0000000, 0.0000000, 0.0000000, 0.0000000, -0.0000000, 0.0001000, 0.0000000,
	    0.0000000, 0.0000000, 0.0000000, 0.0000000, -0.0000000, -0.0000000, 0.0000039, -0.0000000, 0.0000000, -0.0000000, 0.0000000, -0.0000000, 0.0000000, 0.0000000, -0.0000000, -0.0000000, -0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0001000
	};

	arm_matrix_instance_f32 P, Pdot;
	float32_t PdotBuff[21*21];
	arm_mat_init_f32(&P, 21, 21, &PBuff);

	compute_Pdot(&q, &a_sf, &g_sf, &gBias, &aBias, &aMeas, &wMeas, &P, Q,
				 phi, h, vn, ve, vd, we, &Pdot, PdotBuff);


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
