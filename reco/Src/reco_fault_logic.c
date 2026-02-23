#include "reco_fault_logic.h"
#include "main.h"
#include "comms.h"


extern reco_message doubleBuffReco[2];
extern volatile atomic_uchar writeIdx;
extern TIM_HandleTypeDef htim5;

/* Stores the TIM5 counter value at the moment checkForFault() runs.
 * Used by solveFault() to enforce the 3ms minimum delay non-blocking. */
static uint32_t faultDetectedTick = 0;

void checkForFault(void) {
    
     // LOW pin means driver faulted    
    doubleBuffReco[writeIdx].fault[0] = !HAL_GPIO_ReadPin(FLT_A_GPIO_Port, FLT_A_Pin);
    doubleBuffReco[writeIdx].fault[1] = !HAL_GPIO_ReadPin(FLT_B_GPIO_Port, FLT_B_Pin);
    doubleBuffReco[writeIdx].fault[2] = !HAL_GPIO_ReadPin(FLT_C_GPIO_Port, FLT_C_Pin);
    doubleBuffReco[writeIdx].fault[3] = !HAL_GPIO_ReadPin(FLT_D_GPIO_Port, FLT_D_Pin);
    doubleBuffReco[writeIdx].fault[4] = !HAL_GPIO_ReadPin(FLT_E_GPIO_Port, FLT_E_Pin);

    // Record time of fault detection
    faultDetectedTick = __HAL_TIM_GET_COUNTER(&htim5);
}

void solveFault(void) {
    if ((__HAL_TIM_GET_COUNTER(&htim5) - faultDetectedTick) < FAULT_LATCH_DELAY_TICKS) {
        return;
    }

    // LATCH LOW to reset faulted driver
    if (doubleBuffReco[writeIdx].fault[0]) {
        HAL_GPIO_WritePin(LATCH_A_GPIO_Port, LATCH_A_Pin, GPIO_PIN_RESET);
    }
    if (doubleBuffReco[writeIdx].fault[1]) {
        HAL_GPIO_WritePin(LATCH_B_GPIO_Port, LATCH_B_Pin, GPIO_PIN_RESET);
    }
    if (doubleBuffReco[writeIdx].fault[2]) {
        HAL_GPIO_WritePin(LATCH_C_GPIO_Port, LATCH_C_Pin, GPIO_PIN_RESET);
    }
    if (doubleBuffReco[writeIdx].fault[3]) {
        HAL_GPIO_WritePin(LATCH_D_GPIO_Port, LATCH_D_Pin, GPIO_PIN_RESET);
    }
    if (doubleBuffReco[writeIdx].fault[4]) {
        HAL_GPIO_WritePin(LATCH_E_GPIO_Port, LATCH_E_Pin, GPIO_PIN_RESET);
    }
}

void setLatch(void) {
    // Return LATCH HIGH to restore normal state 
    if (doubleBuffReco[writeIdx].fault[0]) {
        HAL_GPIO_WritePin(LATCH_A_GPIO_Port, LATCH_A_Pin, GPIO_PIN_SET);
    }
    if (doubleBuffReco[writeIdx].fault[1]) {
        HAL_GPIO_WritePin(LATCH_B_GPIO_Port, LATCH_B_Pin, GPIO_PIN_SET);
    }
    if (doubleBuffReco[writeIdx].fault[2]) {
        HAL_GPIO_WritePin(LATCH_C_GPIO_Port, LATCH_C_Pin, GPIO_PIN_SET);
    }
    if (doubleBuffReco[writeIdx].fault[3]) {
        HAL_GPIO_WritePin(LATCH_D_GPIO_Port, LATCH_D_Pin, GPIO_PIN_SET);
    }
    if (doubleBuffReco[writeIdx].fault[4]) {
        HAL_GPIO_WritePin(LATCH_E_GPIO_Port, LATCH_E_Pin, GPIO_PIN_SET);
    }
}