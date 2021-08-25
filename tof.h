/* 
 * File:   tof.h
 * Author: sid
 *
 * Created on April 11, 2018, 5:49 PM
 */
#ifndef TOF_H
#define	TOF_H

#include <xc.h> // board info
#include "main.h"

#define TOFAddr 0x29

uint8_t GetRange (void);

#ifdef	__cplusplus
extern "C" {
#endif





#ifdef	__cplusplus
}
#endif

#endif	/* TOF_H */

