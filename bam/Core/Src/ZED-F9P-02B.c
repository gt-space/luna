#include "ZED-F9P-02B.h"
/****
 *
 * I2C Interface via Fast-Mode
 * 
 * Slave Mode maximum bit rate = 400 kbps
 *  > Hold SCL LOW to pause transaction [max 20 ms]
 *  > 300 ns transition time required from SCL falling edge 
 * 
 * UBX-MON-COMMS PortId - I2C = 0x0000 
 * 
 */

/// Register library
// TODO find driver or write a new one...