#ifndef FAULT_LOGIC
#define FAULT_LOGIC

#include "stdbool.h"
#include "stdio.h"
#include "math.h"
#include "main.h"
#include <stdint.h>

void fault_logic_init(void);

void check_fault_pins(uint32_t time_ms);

void solve_faults(void);

#endif