/* 
 * File:   roboclaw.h
 * Author: sidha
 *
 * Created on March 14, 2017, 10:40 AM
 */

#ifndef ROBOCLAW_H
#define	ROBOCLAW_H

#include <xc.h> // board info
#include "main.h"

#define ROBOCLAW_ADDRESS 128
#define RESPONSE_TIMEOUT 625       //(10*1000) >> 4 //10 in ms cause Roboclaw clears packet buffer after this 

extern uint32_t glb_M1EncoderCount;
extern uint32_t glb_M2EncoderCount;
extern bool glb_RoboclawNotResponding;
extern bool glb_EncodersReset;

//Function declarations
unsigned int crc16 (unsigned char *packet, uint8_t nBytes);

//Advanced Drive Functions
bool Drive_Motors_Distance (uint32_t accel, uint32_t deccel, int32_t speedM1, uint32_t distanceM1, int32_t speedM2, uint32_t distanceM2, uint8_t buffer);
void Drive_Motors_Velocity (uint32_t accel, int32_t speedM1, int32_t speedM2);
void Stop_Motors_Decel (uint32_t decel);
// Basic Drive Functions
void Drive_Forward_M1 (uint8_t speed);
void Drive_Backward_M1 (uint8_t speed);
void Drive_Forward_M2 (int8_t speed);
void Drive_Backward_M2 (uint8_t speed);
void Drive_Forward_Motors (uint8_t speed);
void Drive_Backward_Motors (uint8_t speed);
void Drive_M2_Position (uint32_t speed, uint32_t accel,uint32_t position);
void Stop_M1 (void);
void Stop_M2 (void);
void Stop_Motors (void);

// Encoder functions
void Reset_Quadrature_Encoder_Counters (void);
void Read_Encoder_Counters (void);

//Status Read Functions
void Read_Firmware_Version_RC (void);
void Read_Main_Battery_Voltage_Level_RC (void);

uint32_t Read_M1Speed(void);
uint32_t Read_M2Speed(void);
bool areMotorsStopped(void);

void CheckRoboclawReception (void);

#ifdef	__cplusplus
extern "C" {
#endif

#ifdef	__cplusplus
}
#endif

#endif	/* ROBOCLAW_H */

