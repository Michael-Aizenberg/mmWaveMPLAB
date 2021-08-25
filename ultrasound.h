/* 
 * File:   ultrasound.h
 * Author: sidha
 *
 * Created on March 31, 2017, 9:49 PM
 */

#ifndef ULTRASOUND_H
#define	ULTRASOUND_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <xc.h>
#include "main.h"
#include "system.h"
    
uint16_t ManualScanInt (uint8_t channel);
float ManualScanFloat (uint8_t channel);
void TriggerUltrasound (uint8_t channel);



#ifdef	__cplusplus
}
#endif

#endif	/* ULTRASOUND_H */

