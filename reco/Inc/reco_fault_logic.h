#ifndef RECO_FAULT_LOGIC
#define RECO_FAULT_LOGIC

#include "stm32h5xx_hal.h"
#include <stdint.h>

#define NUM_RECO_DRIVERS 5
#define FAULT_LATCH_DELAY_TICKS 3000

void checkForFault(void);
void solveFault(void);
void setLatch(void);

#endif 
