#include "fault_logic.h"

// Array that holds the GPIO ports of the LATCH pins
static GPIO_TypeDef* latch_ports[NUM_DRIVERS] = {
	LATCH_A_GPIO_Port,
	LATCH_B_GPIO_Port,
	LATCH_C_GPIO_Port,
	LATCH_D_GPIO_Port,
	LATCH_E_GPIO_Port
};

// Array that holds the GPIO pin numbers of the LATCH pins

static uint16_t latch_pins[NUM_DRIVERS] = {
	LATCH_A_Pin,
	LATCH_B_Pin,
	LATCH_C_Pin,
	LATCH_D_Pin,
	LATCH_E_Pin
};

// Array that holds the GPIO ports of the FAULT pins
static GPIO_TypeDef* fault_ports[NUM_CHANNELS] = {
    FLT_A1_GPIO_Port,
    FLT_A2_GPIO_Port,
    FLT_B1_GPIO_Port,
    FLT_B2_GPIO_Port,
    FLT_C1_GPIO_Port,
    FLT_C2_GPIO_Port,
    FLT_D1_GPIO_Port,
    FLT_D2_GPIO_Port,
    FLT_E1_GPIO_Port,
    FLT_E2_GPIO_Port
};

// Array that holds the GPIO pin numbers of the FAULT pins
static uint16_t fault_pins[NUM_CHANNELS] = {
	FLT_A1_Pin,
    FLT_A2_Pin,
    FLT_B1_Pin,
    FLT_B2_Pin,
    FLT_C1_Pin,
    FLT_C2_Pin,
    FLT_D1_Pin,
    FLT_D2_Pin,
    FLT_E1_Pin,
    FLT_E2_Pin
};

// Holds data about all five recovery drivers on the STM32
static drv_fault_status_t driver_statuses[NUM_DRIVERS];

/**
 * @brief Initializes the fault logic system for all recovery drivers.
 *
 * This function prepares the STM32 recovery driver system by:
 *   - Setting all LATCH pins to a high-impedance or "float" state
 *     (GPIO_PIN_SET on open-drain pins, per MX configuration)
 *   - Initializing the internal driver status array:
 *       - `state` set to DRV_IDLE
 *       - `fault_reported` set to false
 *       - `fault_detect_ms` and `wait_finished_ms` set to 0
 *
 * After calling this function, the fault logic system is ready to
 * monitor FAULT pins and perform latch recovery sequences automatically.
 *
 * @note This function should be called once at system startup,
 *       before any fault monitoring occurs.
 * @note Uses the GPIO ports and pins defined in latch_ports[] and fault_ports[].
 */
void fault_logic_init(void){
    for (uint8_t i = 0; i < NUM_DRIVERS; i++){
        //sets latch pins to FLOAT (due to open-drain configuarion in MX)
        HAL_GPIO_WritePin(latch_ports[i], latch_pins[i], GPIO_PIN_SET);	
        
        //initializes fault status for each driver
        driver_statuses[i].state = DRV_IDLE;
        driver_statuses[i].fault_reported = false;
        driver_statuses[i].fault_detect_ms = 0;
        driver_statuses[i].wait_finished_ms = 0;
    }
}

/**
 * @brief Monitors all fault pins and updates recovery driver states.
 *
 * This function iterates through all recovery drivers and their associated
 * FAULT channels, updating their internal state machine based on the current
 * input signals and elapsed time. It handles automatic fault latching
 * and recovery sequences according to the following states:
 *
 * 1. DRV_IDLE
 *    - Normal operation
 *    - Checks if both FAULT channels report a fault
 *    - If a fault is detected, moves to DRV_WAIT_5MS
 *      and records the fault detection timestamp
 *
 * 2. DRV_WAIT_5MS
 *    - Waits for a defined period (WAIT_PERIOD, 5 ms)
 *    - After wait expires, pulls the corresponding LATCH pin low
 *      to handle the fault
 *    - Moves to DRV_WAIT_3MS and records the latch timestamp
 *
 * 3. DRV_WAIT_3MS
 *    - Maintains the LATCH low for LATCH_PERIOD (3 ms)
 *    - After the period, restores the LATCH pin to float (GPIO_PIN_SET)
 *    - Returns the driver to DRV_IDLE
 *
 * @param time_ms The current system time in milliseconds
 *                (should be obtained from a consistent time base, get_system_time())
 */
void check_fault_pins(uint32_t time_ms){
	for (uint8_t i = 0; i < NUM_DRIVERS; i++) {
        
        // Get the current driver we are analyzing
        drv_fault_status_t* curr_driver = &driver_statuses[i];

		// Check if either channel on curr_driver has faulted. 1 indicates fault.
		bool ch1_fault = HAL_GPIO_ReadPin(fault_ports[i], fault_pins[i]);
        bool ch2_fault = HAL_GPIO_ReadPin(fault_ports[i + 1], fault_pins[i + 1]);
		curr_driver->fault_reported = !ch1_fault || !ch2_fault;

		//switch decides action based on curr_driver
		switch (curr_driver->state){
            /*
            The DRV_IDLE case is the state that a recovery driver is in during normal operation.
            While in this state, it continuously checks if a fault has occured on either one of its
            channels. If it has, it then moves to the DRV_WAIT_5MS state where it waits for 5 ms.
            */
			case DRV_IDLE:
				if (curr_driver->fault_reported) {
					curr_driver->state = DRV_WAIT_5MS;
					curr_driver->fault_detect_ms = time_ms;
				}
				break;
            /*
            The DRV_WAIT_5MS case is the waiting state that a recovery driver is in after a FAULT has
            been detected on one of its channels. While in this state, we do no actions and simply wait
            5 ms. At the end of the state, however, we pull LATCH low, and move to the next state 
            DRV_WAIT_3MS and also set the time we set LATCH to low.
            */
            case DRV_WAIT_5MS:
                if (time_ms - curr_driver->fault_detect_ms >= WAIT_PERIOD) {
                    curr_driver->state = DRV_WAIT_3MS;
                    curr_driver->wait_finished_ms = time_ms;
                    HAL_GPIO_WritePin(latch_ports[i], latch_pins[i], GPIO_PIN_RESET);
                }
                break;
            /*
            The DRV_WAIT_3MS case is the waiting state that a recovery driver is in after a FAULT has
            been handled by pulling the associated LATCH low. After pulling LATCH low, we must keep it
            in this state for at least 3 ms before setting it back to float. We also set the driver to 
            be back in normal operation by setting its state to be DRV_WAIT_3MS.
            */
			case DRV_WAIT_3MS:                
				if (time_ms - curr_driver->wait_finished_ms >= LATCH_PERIOD) {
                    curr_driver->state = DRV_IDLE;
					HAL_GPIO_WritePin(latch_ports[i], latch_pins[i], GPIO_PIN_SET);
				} 
				break;
		}
	}
}
