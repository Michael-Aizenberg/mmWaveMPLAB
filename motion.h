/* 
 * File:   motion.h
 * Author: sidha
 *
 * Created on April 1, 2017, 9:25 AM
 */

#ifndef MOTION_H
#define	MOTION_H

#ifdef	__cplusplus
extern "C" {
#endif
    
#include <xc.h>
#include "main.h"
#include "system.h"
#include "roboclaw.h"
    
#define QPP_PER_REV 3200
#define WHEEL_BASE_RADIUS 105
#define NUM_RETRIES 1
#define WHEEL_REVS_FOR_360 2.58
    
extern bool glb_motionFailed;
    
void TurnLeftDegreesPivot (uint16_t degrees, int32_t speed, uint32_t accel);  

void TurnRightDegreesPivot (uint16_t degrees, int32_t speed, uint32_t accel);

void MoveDistanceForwards (uint32_t distance, int32_t speed, uint32_t accel, uint32_t deccel); // In MM

void MoveDistanceBackwards(uint32_t distance, int32_t speed, uint32_t accel);

bool CheckForObstacleInFront(uint32_t MinDistance);

#ifdef	__cplusplus
}
#endif

#endif	/* MOTION_H */

