#ifndef EC_APP_H
#define EC_APP_H

#include <stdbool.h>
#include <stdint.h>
#include "ec_side.h"

/* Communication-only integration: DOES NOT energize or command actuators.
 * Requires nonzero, freshly generated boot ID in a flight-ready build.
 */
bool EC_App_Init(uint32_t boot_id);
void EC_App_Process(uint64_t now_us);

/* Read-only access for debugger/diagnostics. */
const fcec_ec_t *EC_App_State(void);

#endif /* EC_APP_H */
