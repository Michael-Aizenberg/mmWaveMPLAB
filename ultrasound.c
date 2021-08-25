
#include "ultrasound.h"

float ManualScanFloat (uint8_t channel)
{
    uint32_t timerVal = 0;
    float distance = 0.0;
    switch (channel)
    {
        case 1 :
             TriggerUltrasound(channel);       
            while (_RF13 == 0)
            {
                
            }
            TMR3_Start();
            while (_RF13 == 1)
            {
                
            }
            TMR3_Stop();
            timerVal = TMR3;
            distance = ((float)(timerVal >> 1))/58.0;
            break;
        case 2 :
            break;   
        case 3 :
            TriggerUltrasound(channel);       
            while (_RA14 == 0)
            {
                
            }
            TMR3_Start();
            while (_RA14 == 1)
            {
                
            }
            TMR3_Stop();
            timerVal = TMR3;
            distance = ((float)(timerVal >> 1))/58.0;
            break;
        case 4 :
            //SCCP4_CAPTURE_Start();
            TriggerUltrasound(channel);       
            while (_RD1 == 0)
            {
                
            }
            TMR3_Start();
            while (_RD1 == 1)
            {
                
            }
            TMR3_Stop();
            //SCCP4_CAPTURE_Stop();
            timerVal = TMR3;
            distance = ((float)(timerVal >> 1))/58.0;
            break;
        case 5 :
            break;
        case 6:
            TriggerUltrasound(channel);
            while (_RF12 == 0)
            {
                
            }
            TMR3_Start();
            while (_RF12 == 1)
            {
                
            }
            TMR3_Stop();
            //SCCP4_CAPTURE_Stop();
            timerVal = TMR3;
            distance = ((float)(timerVal >> 1))/58.0;
            break;
    }
    
    return distance;
}

uint16_t ManualScanInt (uint8_t channel)
{
    uint32_t timerVal = 0;
    uint16_t distance = 0;
    switch (channel)
    {
        case 1 :
             TriggerUltrasound(channel);       
            while (_RF13 == 0)
            {
                
            }
            TMR3_Start();
            while (_RF13 == 1)
            {
                
            }
            TMR3_Stop();
            timerVal = TMR3;
            distance = (timerVal >> 1)/58;
            break;
        case 2 :
            break;   
        case 3 :
            TriggerUltrasound(channel);       
            while (_RD8 == 0)
            {
                
            }
            TMR3_Start();
            while (_RD8 == 1)
            {
                
            }
            TMR3_Stop();
            timerVal = TMR3;
            distance = (timerVal >> 1)/58;
            break;
        case 4 :
            //SCCP4_CAPTURE_Start();
            TriggerUltrasound(channel);       
            while (_RD7 == 0)
            {
                
            }
            TMR3_Start();  
            while (_RD7 == 1)
            {
                
            }
            TMR3_Stop();
            //SCCP4_CAPTURE_Stop();
            timerVal = TMR3;
            distance = (timerVal >> 1)/58;
            break;
        case 5 :
            break;
        case 6:
            TriggerUltrasound(channel);
            while (_RF12 == 0)
            {
                
            }
            TMR3_Start();
            while (_RF12 == 1)
            {
                
            }
            TMR3_Stop();
            //SCCP4_CAPTURE_Stop();
            timerVal = TMR3;
            distance = (timerVal >> 1)/58;
            break;
    }
    
    return distance;
}

void TriggerUltrasound (uint8_t channel)
{
    switch (channel)
    {
        case 1 :
            US1Trigger_SetLow();
            __delay_us(5);
            US1Trigger_SetHigh();
            __delay_us(10);
            US1Trigger_SetLow();
            break;
        case 2 :
            break;   
        case 3 :
            US3Trigger_SetLow();
            __delay_us(5);
            US3Trigger_SetHigh();
            __delay_us(10);
            US3Trigger_SetLow();
            break;
        case 4 :
            US4Trigger_SetLow();
            __delay_us(5);
            US4Trigger_SetHigh();
            __delay_us(10);
            US4Trigger_SetLow();
            break;  
        case 5:
            break;
        case 6:
            US6Trigger_SetLow();
            __delay_us(5);
            US6Trigger_SetHigh();
            __delay_us(10);
            US6Trigger_SetLow();
            break;
    }
}
