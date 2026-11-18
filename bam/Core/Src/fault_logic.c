#include "fault_logic.h"

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
 * @param message struct that holds data that gets sent to FC
 */
void check_fault_pins(uint32_t time_ms, reco_message_t* message){
	for (uint8_t i = 0; i < NUM_DRIVERS; i++) {

        // Get the current driver we are analyzing
        drv_fault_status_t* curr_driver = &driver_statuses[i];

		// Check if either channel on curr_driver has faulted. 1 indicates fault.
		bool ch1_fault = 0;
        bool ch2_fault = 0;

        message->reco_driver_faults[i * 2] 	   = ch1_fault;
        message->reco_driver_faults[i * 2 + 1] = ch2_fault;

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
				}
				break;
		}
	}
}
