#include "fault_logic.h"

#define NUM_DRIVERS 5
#define FAULT_WAIT_MS 3

//defines array of each GPIO port for easy iteration through each
static GPIO_TypeDef* latch_ports[NUM_DRIVERS] = {
	LATCH_A_GPIO_Port,
	LATCH_B_GPIO_Port,
	LATCH_C_GPIO_Port,
	LATCH_D_GPIO_Port,
	LATCH_E_GPIO_Port
};

//defines array of each latch pin for easy iteration through each
static uint8_t latch_pins[NUM_DRIVERS] = {
	LATCH_A_Pin,
	LATCH_B_Pin,
	LATCH_C_Pin,
	LATCH_D_Pin,
	LATCH_E_Pin
}

static GPIO_TypeDef* fault_ports[NUM_DRIVERS] = {
	FLT_A_GPIO_Port,
	FLT_B_GPIO_Port,
	FLT_C_GPIO_Port,
	FLT_D_GPIO_Port,
	FLT_E_GPIO_Port
};

static uint8_t fault_pins[NUM_DRIVERS] = {
	FLT_A_Pin,
	FLT_B_Pin,
	FLT_C_Pin,
	FLT_D_Pin,
	FLT_E_Pin
}

//Defines the possible fault states each pin can have
typedef enum {
 	DRV_IDLE,
  DRV_WAIT_3MS,
  DRV_LATCH_LOW,
  DRV_RESTORE_HIGH
} driver_state_t;

//Defines all data relevant to faulting
typedef struct {
  bool fault_reported;
  driver_state_t state;
  uint32_t fault_detect_ms;
} drv_fault_status_t;

static drv_fault_status_t driver_statuses [NUM_DRIVERS];

//sets each pin to HIGH. runs once at start of main
void fault_logic_init(void){
		for (uint8_t i = 0; i < NUM_DRIVERS; i++){
			//sets latch pins to HIGH
			HAL_GPIO_WritePin(latch_ports[i], latch_pins[i], GPIO_PIN_SET);	
			
			//initializes fault status for each driver
			driver_statuses[i].state = DRV_IDLE;
			driver_statuses[i].fault_reported = false;
			driver_statuses[i].fault_detect_ms = 0;
		}
}

void check_fault_pins(uint32_t time_ms){
	for (uint8_t i = 0; i < NUM_DRIVERS, i++){
		//read pin returns a 1 or 0. 1 means fault
		bool fault = HAL_GPIO_ReadPin(fault_ports[i], fault_pins[i]);
		driver_statuses[i].fault_reported = fault;

		//switch decides action based on current driver_state_t
		switch (driver_statuses[i].state){
			case DRV_IDLE:
			//if fault exists, sets state to waiting and record detection time
				if (fault){
					driver_statuses[i].state = DRV_WAIT_3MS;
					driver_statuses[i].fault_detect_ms = time_ms;
				}
				break;
			case DRV_WAIT_3MS:
				if(!fault){
					driver_statuses[i].state = DRV_IDLE;
					break;
				}
				if (time_ms - driver_statuses[i].fault_detect_ms >= FAULT_WAIT_MS){
					//3ms has elapsed since fault detected, latch is pulled low
					HAL_GPIO_WritePin(latch_ports[i], latch_pins[i], GPIO_PIN_RESET);
					driver_statuses[i].state = DRV_LATCH_LOW;
				} 
				break;
			case DRV_LATCH_LOW:
				HAL_GPIO_WritePin(latch_ports[i], latch_pins[i], GPIO_PIN_SET);
				driver_statuses[i].state = DRV_IDLE;
				break;
		}
	}
}

void solve_faults(){
	//Pull Latch pins of correct drivers to low. if they are already low, set them to high
}