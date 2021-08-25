/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef XC_HEADER_TEMPLATE_H
#define	XC_HEADER_TEMPLATE_H

#define _XTAL_FREQ  32000000UL
#define FCY 16000000UL

#include <xc.h> // include processor files - each processor file is guarded.  
#include <libpic30.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/*
 Globals
 */
// Servo control
extern uint16_t u16_glb_ThermoScanServoPW; // Servo 1
extern uint16_t u16_glb_ThermoTiltPW; // Servo 2
extern uint16_t u16_glb_TOFServoPW; // Servo 3
extern bool b_glb_ThermoScanServoPulseHigh;
extern bool b_glb_ThermoTiltPulseHigh;
extern bool b_glb_DistTOFServoTurnPulseHigh;



// I/O and Communications
#define MAX_RECIEVED_PACKET_LENGTH_ROBOCLAW 500
#define DEFAULT_ACCEL 1500
#define DEFAULT_SPEED 1500
#define DEFAULT_TURN_SPEED 3900
#define MIN_ALLOWED_DIST_TO_OBSTACLE 8 //in cm


#define mf glb_motionFailed //TEMPORARY

#define ButtonS3NotPushed PORTDbits.RD6 == 1
#define ButtonS4Pushed PORTDbits.RD13 == 0

extern uint8_t uart1_RxPacket[MAX_RECIEVED_PACKET_LENGTH_ROBOCLAW];
extern uint16_t Rx_DataIndex;
extern bool Roboclaw_DataRecieved;

// live documentation
#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */

