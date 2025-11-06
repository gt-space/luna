#ifndef TESTS_H_
#define TESTS_H_

#include "common.h"
#include "compute_F.h"
#include "compute_hats.h"
#include "ekf_utils.h"
#include "stdio.h"
#include "stdbool.h"
#include "string.h"

// Test result structure
typedef struct
{
    const char *test_name;
    bool passed;
    const char *error_message;
} test_result_t;

// Test statistics
typedef struct
{
    uint32_t total_tests;
    uint32_t passed_tests;
    uint32_t failed_tests;
} test_stats_t;

// Test helper functions
bool test_matrix_dimensions(arm_matrix_instance_f32 *matrix, uint16_t expected_rows, uint16_t expected_cols);
bool test_matrix_finite(arm_matrix_instance_f32 *matrix);
bool test_vector_finite(float32_t *vector, uint32_t length);
bool test_matrix_not_all_zero(arm_matrix_instance_f32 *matrix);
void print_test_result(const char *test_name, bool passed, const char *message);

// Test functions for compute_hats.c
bool test_compute_wn_basic(void);
bool test_compute_wn_zero_velocity(void);
bool test_compute_wn_edge_cases(void);
bool test_compute_what_basic(void);
bool test_compute_what_zero_inputs(void);
bool test_compute_ahat_basic(void);
bool test_compute_ahat_zero_inputs(void);

// Test functions for compute_F.c
bool test_compute_F_dimensions(void);
bool test_compute_F_finite_values(void);
bool test_compute_F_zero_velocity(void);
bool test_compute_G_dimensions(void);
bool test_compute_G_finite_values(void);
bool test_compute_G_structure(void);

// Test runner
void run_all_tests(void);
test_stats_t get_test_stats(void);

#endif
