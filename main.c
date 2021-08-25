
/*
 * File:   main.c
 * Author: sidha
 *
 * Created on December 19, 2016, 5:39 PM
 */

#include <p24FJ1024GB610.h>
#include <xc.h>

#include "main.h"
#include "system.h"
#include "roboclaw.h"
#include "servo.h"
#include "ultrasound.h"
#include "motion.h"
#include "lcd.h"
#include "tof.h"

// Init Globals
uint16_t u16_glb_ThermoScanServoPW = 1500;
uint16_t u16_glb_ThermoTiltPW = 1500;
uint16_t u16_glb_TOFServoPW = 1500;

uint8_t numTLVs = 0;
uint8_t numOfPeople = 0;
uint32_t total_packet_len = 0;

uint8_t pointCloudTLVLengthBytes[4];
uint32_t pointCloudTLVLength;
                
uint8_t targetListTLVLengthBytes[4];
uint32_t targetListTLVLength;





//unsigned char cfg[31][80] = {
//"sensorStop\n", 
//"flushCfg\n", 
//"dfeDataOutputtMode 1\n",
//"channelCfg 15 7 0\n", 
//"adcCfg 2 1\n", 
//"adcbufCfg -1 0 1 1 1\n",
//"lowPower 0 0\n",
//"profileCfg 0 60.75 30.00 25.00 59.10 657930 0 54.71 1 96 2950.00 2 1 36\n",
//"chirpCfg 0 0 0 0 0 0 0 1\n",
//"chirpCfg 1 1 0 0 0 0 0 2\n",
//"chirpCfg 2 2 0 0 0 0 0 4\n",
//"frameCfg 0 2 96 0 55.00 1 0\n",
//"dynamicRACfarCfg -1 4 4 2 2 8 12 4 8 5.00 8.00 0.40 1 1\n",
//"staticRACfarCfg -1 6 2 2 2 8 8 6 4 8.00 15.00 .30 0 0\n",
//"dynamicRangeAngleCfg -1 .75 0.0010 1 0\n",
//"dynamic2DAngleCfg -1 1.5 0.0300 1 0 1 0.30 0.85 8.00\n",
//"staticRangeAngleCfg -1 0 8 8\n",
//"antGeometry0 0 0 -1 -1 -2 -2 -3 -3 -2 -2 -3 -3\n",
//"antGeometry1 0 -1 -1 0 0 -1 -1 0 -2 -3 -3 -2\n",
//"antPhaseRot 1 -1 -1 1 1 -1 -1 1 1 -1 -1 1\n",
//"fovCfg -1 70.0 20.0\n",
//"compRangeBiasAndRxChanPhase 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 \n",
//"staticBoundaryBox -3 3 2 6 0 3\n",
//"boundaryBox -4 4 0.5 8 0 3\n",
//"sensorPosition 2 0 15\n",
//"gatingParam 3 2 2 2 4\n",
//"stateParam 3 3 6 500 50 6000\n",
//"allocationParam 40 100 0.1 20 0.5 20\n",
//"maxAcceleration 0.1 0.1 0.1\n",
//"trackingCfg 1 2 800 30 46 96 55\n",
//"sensorStart\n"
//};

      
int main(void)
{
    SYSTEM_Initialize();
    Run_LED_Off();
    Det_LED_Off();
    Mic_LED_Off();
    
//    uint8_t i = 0;
//    for (i = 0; i < 31; i++) {
//        UART1_SendString(*(cfg + i));
//        __delay_ms(500);
//        
//    }
    //TODO : 
    while (1)
    {
        if (Rx_DataIndex >= 16 && Rx_DataIndex <=17) {
            total_packet_len = (uint32_t)(( ((uint32_t)uart1_RxPacket[Rx_DataIndex]) << 24) | ((uint32_t)(uart1_RxPacket[Rx_DataIndex - 1]) << 16) | ((uint32_t)(uart1_RxPacket[Rx_DataIndex -2 ]) << 8) | ((uint32_t)(uart1_RxPacket[Rx_DataIndex - 3])));                        
        }
        else if (total_packet_len != 0 && Rx_DataIndex >= total_packet_len )
        {
            //extract number of people
            numTLVs = uart1_RxPacket[44];
            
            if (numTLVs == 2 || numTLVs == 3) {
                pointCloudTLVLengthBytes = {uart1_RxPacket[52], uart1_RxPacket[53], uart1_RxPacket[54], uart1_RxPacket[55]};
                pointCloudTLVLength = (pointCloudTLVLengthBytes[3] << 24) | (pointCloudTLVLengthBytes[2] << 16) | (pointCloudTLVLengthBytes[1] << 8) | (pointCloudTLVLengthBytes[0]);
                
                targetListTLVLengthBytes = {uart1_RxPacket[52 + pointCloudTLVLength], uart1_RxPacket[53 + pointCloudTLVLength], uart1_RxPacket[54 + pointCloudTLVLength],uart1_RxPacket[55 + pointCloudTLVLength]};
                targetListTLVLength = (targetListTLVLengthBytes[3] << 24) | (targetListTLVLengthBytes[2] << 16) | (targetListTLVLengthBytes[1] << 8) | (targetListTLVLengthBytes[3]);
                
                numOfPeople = (targetListTLVLength-8)/112;
            __delay_ms(1000);
        }
       //tofTest = GetRange();
       //tofTest++;
       //printf("count = %d", tofTest); 
       //__delay_ms(1000);
       //printf( "\f" );
        __delay_ms(100);
        UART1_SendString("Main");
    }
    
   return 0;
}



























// Below is a test main function. The current (7/11/2017) test function is above
//int main(void) 
//{
//    SYSTEM_Initialize();
//    TMR1_Stop();
//    TMR2_Stop();
//    Run_LED_Off();
//    Det_LED_Off();
//    Mic_LED_Off();
//    
////    uint16_t micAdcValue = 1023; //speaker is at 3V3 to start with
////  
////    while (micAdcValue >= 550) // Waits for buzzer
////    {
////        micAdcValue = ADC1_ReadMic();
////        __delay_us(1);
////    }
////    Mic_LED_On();
////    __delay_ms(1000);
////    Run_LED_On();
////    Flush_Rx_Buffer(); //the roboclaw buffer
////    uint16_t U1 = 0;
////    uint16_t U6 = 0;
////    U1 = ManualScanInt(1);
////    U6 = ManualScanInt(6);
////    
//////    if (U1 > U6)
//////    {
//////        TurnLeftDegreesPivot(90, -3000, 5000);
//////    }
////
////    bool MotionDone = false;
////    bool turnLeft = true;
////    //now moving forward in velocity mode
////    Drive_Motors_Velocity(DEFAULT_ACCEL, DEFAULT_SPEED, DEFAULT_SPEED);
////    __delay_ms(1000);
//    
//    //*****************
//    //***** MAIN ******
//    //*****************
//    
////    while (1) 
////    {
////        //make sure nothing is in front before moving        
////        if (CheckForObstacleInFront(MinDistanceToObstacle))
////        {
////            Stop_Motors_Decel(3000);
////            
////            TurnRightDegreesPivot(90, -3000, 2500); 
////            
////            MotionDone = areMotorsStopped();
////            while (!MotionDone)
////            {  
////                MotionDone = areMotorsStopped();
////                __delay_ms(10); //so as to not flood the roboclaw buffer
////            }            
////            Stop_Motors_Decel(3000);
////   
//////            //flipping the turn direction just for the heck of it ->
//////            if (turnLeft)
//////            {
//////                TurnLeftDegreesPivot(90, -5000, 2500); 
//////                turnLeft = false;
//////            }
//////            else
//////            {
//////                TurnRightDegreesPivot(90, -5000, 2500); 
//////                turnLeft = true;
//////            }
//////            //reading encoder speeds to make sure move is done:
//////            MotionDone = areMotorsStopped();
//////            while (!MotionDone)
//////            {  
//////                MotionDone = areMotorsStopped();
//////                __delay_ms(10); //so as to not flood the roboclaw buffer
//////            }            
//////            Stop_Motors_Decel(3000); //a backup stop in case we are still moving
//////            Drive_Motors_Velocity(DEFAULT_ACCEL, DEFAULT_SPEED, DEFAULT_SPEED);
////        }
////        else if (ManualScanInt(1) >= 20)
////        {
////            Stop_Motors_Decel(3000);
////            Det_LED_On();
////            __delay_ms(2000);
////            Det_LED_Off();
////            Drive_Motors_Velocity(DEFAULT_ACCEL, DEFAULT_SPEED, DEFAULT_SPEED);
////            while (ManualScanInt(1) >= 20 || !CheckForObstacleInFront(MinDistanceToObstacle))
////            {
////                
////            }
////             Stop_Motors_Decel(3000);
////        }
//
//        
//        //servicing any bluetooth UART messages here:
////        if (BTH_RxInterrupt_DataReceived)
////        {
////            __delay_ms(1);
////            Flush_Rx_Buffer();
////            UART_RxInterrupt_DataReceived = false;
////        }
//        
//    }
//
//
//    
//    return 0;
//}
