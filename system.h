
#ifndef SYSTEM_H // if not defined
#define	SYSTEM_H // define

#include <xc.h> // board info
#include "main.h"

#define MICROPHONE_ADC_CH 1

#define LED_D3_On()  _LATA0 = 1; // basic "method" for turning D3 on
#define LED_D3_Off() _LATA0 = 0; // basic "method" for turning D3 off

//Methods for turning the handle LEDs on and off
#define Run_LED_On() _LATB3 = 1;
#define Run_LED_Off() _LATB3 = 0;
#define Det_LED_On() _LATB10 = 1;
#define Det_LED_Off() _LATB10 = 0;
#define Mic_LED_On() _LATB11 = 1;
#define Mic_LED_Off() _LATB11 = 0;

#define ThermoServoScanIO_SetHigh() _LATC1 = 1; // Sets Pin 6 (C1) high
#define ThermoServoScanIO_SetLow() _LATC1 = 0; // Sets Pin 6 (C1) low

#define ThermoServoTiltIO_SetHigh() _LATC2 = 1; // Sets Pin 7 (C2) high
#define ThermoServoTiltIO_SetLow() _LATC2 = 0; // Sets Pin 7 (C2) low

#define DistTOFServoTurnIO_SetHigh() _LATC3 = 1; // Sets Pin 8 (C3) high
#define DistTOFServoTurnIO_SetLow() _LATC3 = 0; // Sets Pin 8 (C3) low

#define US4Trigger_SetHigh() _LATG13 = 1;
#define US4Trigger_SetLow() _LATG13 = 0;
#define US1Trigger_SetHigh() _LATF0 = 1;
#define US1Trigger_SetLow() _LATF0 = 0;
#define US6Trigger_SetHigh() _LATF1 = 1;
#define US6Trigger_SetLow() _LATF1 = 0;
#define US3Trigger_SetHigh() _LATA10 = 1;
#define US3Trigger_SetLow() _LATA10 = 0;

typedef enum 
{
    /* ----- Traps ----- */
    TRAPS_OSC_FAIL = 0, /** Oscillator Fail Trap vector */
    TRAPS_STACK_ERR = 1, /** Stack Error Trap Vector */
    TRAPS_ADDRESS_ERR = 2, /** Address Error Trap Vector */
    TRAPS_MATH_ERR = 3, /** Math Error Trap Vector */
} TRAPS_ERROR_CODE;

typedef enum
{
    //LEDS
    LED_D3 = 0
}LED;

extern uint8_t glb_numBytesToWaitForRoboclaw;


// Function declarations
void SYSTEM_Reset(void);
void SYSTEM_Initialize(void);
void OSCILLATOR_Initialize(void);

void PIN_Init(void);

void UART1_Initialize(void);

void ADC1_Initialize(void);
uint16_t ADC1_ReadMic(void);

void SCCP4_CAPTURE_Initialize(void);
void SCCP4_CAPTURE_Start( void );
void SCCP4_CAPTURE_Stop( void );
uint32_t SCCP4_CAPTURE_Data32Read( void );

void UART1_SendByte(uint8_t msg);
void UART1_SendBytes(uint8_t numBytes, unsigned char *msg);
void UART1_SendString(unsigned char *msg);
void Flush_Rx_Buffer (void);

void TMR1_Initialize (void);
void TMR1_Start (void);
void TMR1_Stop (void);

void TMR2_Initialize (void);
void TMR2_Start( void );
void TMR2_Stop( void );

void TMR3_Initialize (void);
void TMR3_Start( void );
void TMR3_Stop( void );

void TMR4_Initialize(void);
void TMR4_Start(void);
void TMR4_Stop(void);

void TMR5_Initialize(void);
void TMR5_Start(void);
void TMR5_Stop(void);

void I2C1_Initialize(void);
void I2C1_ResetBus(void);
void I2C1_Start(void);
void I2C1_Stop(void);
void I2C1_Restart(void);
uint8_t I2C1_SendByte(uint8_t data);
uint8_t I2C1_ReadNoAck(void);
uint8_t I2C1_ReadAck(void);
uint8_t I2C1Read8(uint8_t DevAddr, uint8_t RegAddr);
void I2C1Write8(uint8_t DevAddr, uint8_t RegAddr, uint8_t data);


#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif


#endif	/* SYSTEM_H */

