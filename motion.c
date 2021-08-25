
#include "motion.h"
#include "main.h"
#include "roboclaw.h"
#include "ultrasound.h"
#include "system.h"

/*
 NOTE: The Buffer argument can be set to a 1 or 0. If a value of 0 is used the command will be buffered
and executed in the order sent. If a value of 1 is used the current running command is stopped,
any other commands in the buffer are deleted and the new command is executed.
 */
uint8_t bufferParam = 0;
bool glb_motionFailed = false; 

void TurnLeftDegreesPivot (uint16_t degrees, int32_t speed, uint32_t accel)
{
    bool ret = false;
    //float DistToMoveMotor = 0;
    Read_Encoder_Counters();
    uint32_t preMoveM1Count = glb_M1EncoderCount;
    degrees = (float) degrees;
  //  uint32_t distanceOffset = CalculateDistanceOffset(speed, accel);
  //  DistToMoveMotor = (((degrees*WHEEL_REVS_FOR_360)/360)*QPP_PER_REV) - distanceOffset;
    
//    ret = Drive_Motors_Distance(accel, speed, (uint32_t) DistToMoveMotor, -speed, (uint32_t) DistToMoveMotor, bufferParam);
    
    uint8_t retries = 0;
    while (!ret)
    {
        if (glb_RoboclawNotResponding == true && retries < NUM_RETRIES) // Uh oh
        {
            Read_Encoder_Counters();
            if (preMoveM1Count >= (glb_M1EncoderCount - 100) || preMoveM1Count <= (glb_M1EncoderCount + 100))
            {
    //            ret = Drive_Motors_Distance(accel, speed, (uint32_t) DistToMoveMotor, -speed, (uint32_t) DistToMoveMotor, bufferParam);
                retries++;
            }
            else
            {
                //TODO: CHECK GYRO!!!
                ret = true;
                glb_motionFailed = false;
                glb_RoboclawNotResponding = false;
            }
        }
        else if (glb_RoboclawNotResponding == true && retries > NUM_RETRIES) // if we are above the retry limit, then get out, main will handle
        {
            ret = true;
            glb_motionFailed = true;
        }
        else // Just in case the loop messes up here or in roboclaw
        {
            ret = true;
        }
    }
}

void TurnRightDegreesPivot (uint16_t degrees, int32_t speed, uint32_t accel)
{
    bool ret = false;
    uint32_t DistToMoveMotor = 0;
    Read_Encoder_Counters();
    uint32_t preMoveM1Count = glb_M1EncoderCount;
    DistToMoveMotor = (QPP_PER_REV*(WHEEL_BASE_RADIUS*degrees)) >> 8;
//    ret = Drive_Motors_Distance(accel, -speed, DistToMoveMotor, speed, DistToMoveMotor, bufferParam);
   
     uint8_t retries = 0;
    while (!ret)
    {
        if (glb_RoboclawNotResponding == true && retries < NUM_RETRIES) // Uh oh
        {
            Read_Encoder_Counters();
            if (preMoveM1Count >= (glb_M1EncoderCount - 100) || preMoveM1Count <= (glb_M1EncoderCount + 100))
            {
  //              ret = Drive_Motors_Distance(accel, -speed, (uint32_t) DistToMoveMotor, speed, (uint32_t) DistToMoveMotor, bufferParam);
                retries++;
            }
            else
            {
                //TODO: CHECK GYRO!!!
                ret = true;
                glb_motionFailed = false;
                glb_RoboclawNotResponding = false;
            }
        }
        else if (glb_RoboclawNotResponding == true && retries > NUM_RETRIES) // if we are above the retry limit, then get out, main will handle
        {
            ret = true;
            glb_motionFailed = true;
        }
        else // Just in case the loop messes up here or in roboclaw
        {
            ret = true;
        }
    }
   
}

void MoveDistanceForwards(uint32_t distance, int32_t speed, uint32_t accel, uint32_t deccel) // In MM
{
    //Var. Decs.
    bool ret = false;
    uint8_t retries = 0;
    uint32_t DistToMoveMotor = (QPP_PER_REV*distance) >> 8;
    
    //Check for Encoders reset
    Reset_Quadrature_Encoder_Counters();
    while(!glb_EncodersReset && retries < NUM_RETRIES)
    {
        Reset_Quadrature_Encoder_Counters();
        retries++;
    }
    retries = 0;

    ret = Drive_Motors_Distance(accel, deccel, speed, DistToMoveMotor, speed, DistToMoveMotor, bufferParam); // Command
    
    // Check to see if command has executed
    while (!ret)
    {
        if (glb_RoboclawNotResponding == true && retries < NUM_RETRIES) // Uh oh
        {
            Read_Encoder_Counters();
            if (!(glb_M1EncoderCount <= DistToMoveMotor + 10 && glb_M1EncoderCount >= DistToMoveMotor - 10) || !(glb_M2EncoderCount <= DistToMoveMotor + 10 && glb_M2EncoderCount >= DistToMoveMotor - 10)) // Didn't move, retry
            {
                ret = Drive_Motors_Distance(accel, deccel, speed, DistToMoveMotor, speed, DistToMoveMotor, bufferParam);
                retries++;
            }
            else // Umm, looks like we moved but someone messed up, so forget the error
            {
                //TODO: CHECK IMU!!!
                ret = true;
                glb_motionFailed = false;
                glb_RoboclawNotResponding = false;
            }
        }
        else if (glb_RoboclawNotResponding == true && retries > NUM_RETRIES) // if we are above the retry limit, then get out, main will handle
        {
            ret = true;
            glb_motionFailed = true;
        }
        else // Just in case the loop messes up here or in roboclaw
        {
            ret = true;
        }
    }
}

void MoveDistanceBackwards(uint32_t distance, int32_t speed, uint32_t accel) // In MM
{
    bool ret = false;
    uint32_t DistToMoveMotor = 0;
    Read_Encoder_Counters();
    uint32_t preMoveM1Count = glb_M1EncoderCount;
    DistToMoveMotor = (QPP_PER_REV * distance) >> 8;
    
  //  ret = Drive_Motors_Distance(accel, -speed, DistToMoveMotor, -speed, DistToMoveMotor, bufferParam);
    
    uint8_t retries = 0;
    while (!ret)
    {
        if (glb_RoboclawNotResponding == true && retries < NUM_RETRIES) // Uh oh
        {
            Read_Encoder_Counters();
            if (preMoveM1Count >= (glb_M1EncoderCount - 100) || preMoveM1Count <= (glb_M1EncoderCount + 100))
            {
  //              ret = Drive_Motors_Distance(accel, -speed, (uint32_t) DistToMoveMotor, speed, (uint32_t) DistToMoveMotor, bufferParam);
                retries++;
            }
            else
            {
                //TODO: CHECK GYRO!!!
                ret = true;
                glb_motionFailed = false;
                glb_RoboclawNotResponding = false;
            }
        }
        else if (glb_RoboclawNotResponding == true && retries > NUM_RETRIES) // if we are above the retry limit, then get out, main will handle
        {
            ret = true;
            glb_motionFailed = true;
        }
        else // Just in case the loop messes up here or in roboclaw
        {
            ret = true;
        }
    }
    
}

bool CheckForObstacleInFront(uint32_t MinDistance)
{
    uint16_t U3 = 0;
    uint16_t U4 = 0;
    U3 = ManualScanInt(3);
    U4 = ManualScanInt(4);
    if (U3 <= MinDistance || U4 <= MinDistance)
    {
        return true;
    }
    else
    {
        return false;
    }
}