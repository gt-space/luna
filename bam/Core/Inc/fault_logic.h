#ifndef FAULT_LOGIC
#define FAULT_LOGIC

#include "main.h"
#include "stdbool.h"
#include "stdint.h"
#include "comms.h"

#define NUM_DRIVERS  5
#define NUM_CHANNELS 10
#define LATCH_PERIOD 3
#define WAIT_PERIOD 5

typedef enum {
  DRV_IDLE,
  DRV_WAIT_3MS,
  DRV_WAIT_5MS,
} driver_state_t;

typedef struct {
  bool fault_reported;
  driver_state_t state;
  uint32_t fault_detect_ms;
  uint32_t wait_finished_ms;
} drv_fault_status_t;

void fault_logic_init(void);
void check_fault_pins(uint32_t time_ms, reco_message_t* message);

#endif
