
#include <xc.h>
#include "servo.h"
#include "main.h"
#include "system.h"

/**
 * Takes degrees and sets up the global turn pulse width
 * @param signed int8 degrees Max + 45, Min -45
 */
void TurnThermoByDegrees (int8_t degrees)
{
    if (degrees >= -45 && degrees <= 45)
    {
        u16_glb_ThermoScanServoPW = 1500 + US_PER_DEGREE*degrees;
    }
    else if (degrees <=0 && degrees >= -45)
    {
        return;
    }
    
}

void TiltThermoByDegrees (int8_t degrees)
{
    if (degrees >= -45 && degrees <= 45)
    {
        u16_glb_ThermoTiltPW = 1500 + US_PER_DEGREE*degrees;
    }
    else 
    {
        return;
    }
}

void TurnDistanceSensorByDegrees (int8_t degrees)
{
    if (degrees >= -45 && degrees <= 45)
    {
        u16_glb_TOFServoPW = 1500 + US_PER_DEGREE*degrees;
    }
    else 
    {
        return;
    }
}

void CenterThermoTurn (void)
{
    u16_glb_ThermoScanServoPW = 1500;
}

void CenterThermoTilt (void)
{
    u16_glb_ThermoTiltPW = 1500;
}

void CenterThermo (void)
{
    CenterThermoTurn();
    CenterThermoTilt();
}

void CenterTOF (void)
{
    u16_glb_TOFServoPW = 1500;
}

void ScanWithThermo (void) //TODO
{
    
}
        