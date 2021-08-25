/* 
 * File:   servo.h
 * Author: sidha
 *
 * Created on March 28, 2017, 2:37 PM
 */

#ifndef SERVO_H
#define	SERVO_H

#include <xc.h> // board info
#include "main.h"

#define US_PER_DEGREE 9

void TurnThermoByDegrees (int8_t degrees);
void TiltThermoByDegrees (int8_t degrees);
void CenterThermoTurn (void);
void CenterThermoTilt (void);
void CenterThermo (void);
void ScanWithThermo (void);

void TurnDistanceSensorByDegrees (int8_t degrees);
void CenterTOF (void);
#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

#endif	/* SERVO_H */

