
#include <xc.h>
#include "tof.h"
#include "main.h"
#include "system.h"

uint8_t GetRange (void)
{
    // Check to see if device is ready for sensing
    while(! (I2C1Read8(TOFAddr, 0x4D) == 0)) {} // TODO: Set up timeout
    // Start measurement
    I2C1Write8(TOFAddr, 0x18, 0x01);
    //Wait for measurement to be finished
    while (!(I2C1Read8(TOFAddr, 0x4F) == 0x04)) {} // TODO: Set up timeout
    //Read value of measurement
    uint8_t val = I2C1Read8(TOFAddr, 0x62);
    
    //Clear interrupt flags
    I2C1Write8(TOFAddr, 0x15, 0x07);
    
    return val;
}