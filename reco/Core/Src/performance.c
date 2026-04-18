#include "performance.h"

#ifdef PERF_ANALYSIS

const char* perf_names[PERF_COUNT] = {
    "main_loop",
    "update_EKF",
    "compute_what",
    "compute_wn",
    "compute_g_dg2",
    "quaternion2DCM",
    "compute_ahat",
    "propagate",
    "compute_qdot",
    "compute_lladot",
    "compute_radius",
    "compute_vdot",
    "compute_pdot",
    "compute_F",
    "compute_G",
	"pdot_calculation",
    "compute_dwdp",
    "compute_dwdv",
    "compute_dpdot_dp",
    "compute_dpdot_dv",
    "compute_dvdot_dp",
    "compute_dvdot_dv",
    "integrate",
    "update_gps",
    "linsolve_gps",
    "update_mag",
    "linsolve_mag",
    "update_baro",
    "nearestPSD",
    "eig",
	"gather_baro",
	"gather_mag",
	"gather_imu",
    "p2alt",
	"filter_dp_dh",
	"baro_kalman_update",
	"add 21x21",
	"multiply 21x21",
	"memcpy 21x21"
};

/**
 * @brief Updates performance statistics for a given function index.
 *
 * This function updates the exponential moving average (EMA), minimum,
 * and maximum cycle counts for a specified performance index using a new
 * measurement (`@p input`).
 *
 * On the first invocation (when `perf->initialized` is false), the EMA,
 * minimum, and maximum values are initialized to the input value.
 * On subsequent calls, the EMA is updated using the configured smoothing
 * factor (`perf->alpha`), and the min/max values are updated if the new
 * input exceeds previous bounds.
 *
 * @param[in,out] perf  Pointer to the performance tracking structure.
 *                      Must be a valid, initialized pointer.
 * @param[in]     idx   Index of the function being profiled (see perf_index_t).
 * @param[in]     input Measured cycle count for the function execution.
 *
 * @note
 * - The EMA is computed as:
 *   ema = alpha * input + (1 - alpha) * previous_ema
 * - `perf->initialized` must be set externally after initial population
 *   to enable steady-state EMA updates.
 * - This function assumes `idx` is within bounds [0, PERF_COUNT).
 */
void perf_update(perf_t* perf, perf_index_t idx, uint32_t input) {

    if (!perf->initialized) {

        perf->ema[idx] = (float32_t) input;
        perf->max[idx] = input;
        perf->min[idx] = input;

    } else {

        if (input > perf->max[idx]) {
            perf->max[idx] = input;
        }

        if (input < perf->min[idx]) {
            perf->min[idx] = input;
        }

        perf->ema[idx] =  perf->alpha * input + (1.0f - perf->alpha) * perf->ema[idx];
    }
}

void perf_main_loop_time(perf_t* perf, float32_t time) {

    if (!perf->initialized) {

        perf->main_loop_time = time;

    } else {

        perf->main_loop_time =  perf->alpha * time + (1.0f - perf->alpha) * perf->main_loop_time;
    }
    
}

/**
 * @brief Initializes the performance tracking structure.
 *
 * This function initializes the smoothing factor (`alpha`) used for the
 * exponential moving average and measures the intrinsic overhead of the
 * profiling update function. The overhead is estimated by timing a single
 * call to the update routine using the DWT cycle counter.
 *
 * The measured overhead is stored in `perf->overhead` for diagnostic purposes.
 *
 * @param[out] perf Pointer to the performance tracking structure to initialize.
 *                  Must be a valid pointer.
 *
 * @note
 * - The DWT cycle counter must be enabled prior to calling this function.
 * - The overhead measurement includes the cost of calling the update function
 *   and any associated instructions.
 * - This function does not initialize all fields (e.g., EMA, min, max arrays);
 *   these are initialized on first use in perf_update().
 */
void perf_init(perf_t* perf) {
    perf->alpha = ALPHA;
    uint32_t start = DWT->CYCCNT;
    perf_update(perf, PERF_MAIN_LOOP, 0);  // or any dummy input
    perf->overhead = DWT->CYCCNT - start;
    return;
}



/**
 * @brief Prints a formatted performance summary table.
 *
 * This function outputs a table of performance metrics for all tracked
 * functions, including:
 * - Exponential moving average (EMA) cycle count
 * - Minimum observed cycle count
 * - Maximum observed cycle count
 * - Percentage of total execution time (relative to PERF_MAIN_LOOP)
 *
 * The output is formatted for readability and is intended for use via
 * standard output (e.g., semihosting, UART, or debugger console).
 *
 * @param[in] perf Pointer to the performance tracking structure.
 *                 Must be a valid pointer.
 *
 * @note
 * - If `perf` is NULL, an error message is printed and the function returns.
 * - The percentage calculation uses PERF_MAIN_LOOP as the total reference.
 * - If the total EMA is zero, percentages default to 0 to avoid division by zero.
 * - This function is safe to call from a debugger (e.g., via GDB `call` command).
 */
__attribute__((section(".text.perf_print")))
void perf_print(const perf_t* perf) {
    if (perf == NULL) {
        printf("perf_print: NULL pointer\n");
        return;
    }

    float32_t total_cycles = perf->ema[PERF_MAIN_LOOP];
    float32_t total_time_ms = perf->main_loop_time;

    printf("\n==================== EKF PERFORMANCE ====================\n");
    printf("Iteration: %lu | Overhead: %.2f cycles\n",
           (unsigned long)perf->indexNum,
           perf->overhead);

    printf("EMA Main Loop Time (ms): %.3f\n", total_time_ms);

    printf("--------------------------------------------------------------------------\n");

    printf("%-25s | %-12s | %-10s | %-10s | %-10s | %-8s\n",
           "Function", "EMA", "Min", "Max", "Time (ms)", "% Total");

    printf("--------------------------------------------------------------------------\n");

    for (uint32_t i = 0; i < PERF_COUNT; i++) {

        const char* name = (perf_names[i] != NULL) ? perf_names[i] : "UNKNOWN";

        float32_t ema_cycles = perf->ema[i];

        float32_t pct = 0.0f;
        if (total_cycles > 0.0f) {
            pct = (ema_cycles / total_cycles) * 100.0f;
        }

        float32_t time_ms = 0.0f;
        if (total_cycles > 0.0f) {
            time_ms = (ema_cycles / total_cycles) * total_time_ms;
        }

        printf("%-25s | %-12.2f | %-10lu | %-10lu | %-10.3f | %7.2f%%\n",
               name,
               ema_cycles,
               (unsigned long)perf->min[i],
               (unsigned long)perf->max[i],
               time_ms,
               pct);
    }

    printf("==========================================================================\n\n");
}

// Required such that the linker doesn't keep transhing
// my damn function because nothing "references" it. Like bruh,
// did you never think that someone would want to GDB it.
volatile void* perf_print_ref = (void*)&perf_print;
#endif
