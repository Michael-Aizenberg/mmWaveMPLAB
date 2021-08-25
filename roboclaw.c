
#include <xc.h>
#include "roboclaw.h"
#include "main.h"
#include "system.h"

uint32_t glb_M1EncoderCount = 0;
uint32_t glb_M2EncoderCount = 0;
bool glb_RoboclawNotResponding = false;
bool glb_EncodersReset = false;
/*
 CRC
 */
//Calculates CRC16 of nBytes of data in byte array message
unsigned int crc16(unsigned char *packet, uint8_t nBytes) 
{
    uint16_t crc = 0;
    uint8_t byte = 0;
    uint8_t b = 0;
    for (byte = 0; byte < nBytes; byte++) 
    {
        crc = crc ^ ((unsigned int)packet[byte] << 8);
        for (b = 0; b < 8; b++) 
        {
            if (crc & 0x8000) 
            {
                crc = (crc << 1) ^ 0x1021;
            } else 
            {
                crc = crc << 1;
            }
        }
    }
    return crc;
}


/****
 Command functions
 ****/

/*
 Drive Functions
 */
/*
 NOTE: The Buffer argument can be set to a 1 or 0. If a value of 0 is used the command will be buffered
and executed in the order sent. If a value of 1 is used the current running command is stopped,
any other commands in the buffer are deleted and the new command is executed.
 */
// Advanced Drive Functions

bool Drive_Motors_Distance (uint32_t accel, uint32_t deccel, int32_t speedM1, uint32_t distanceM1, int32_t speedM2, uint32_t distanceM2, uint8_t buffer)
{
    Flush_Rx_Buffer();
    uint8_t commandID = 67;
    uint8_t numBytes = 37; // include 2 bytes for crc
    
    // All 5 following blocks are breaking up the 4 byte inputs into one byte chunks for the packet
    uint8_t accel1 = (uint8_t) (accel >> 24);
    uint8_t accel2 = (uint8_t) (accel >> 16);
    uint8_t accel3 = (uint8_t) (accel >> 8);
    uint8_t accel4 = (uint8_t) accel;
    
    uint8_t deccel1 = (uint8_t) (deccel >> 24);
    uint8_t deccel2 = (uint8_t) (deccel >> 16);
    uint8_t deccel3 = (uint8_t) (deccel >> 8);
    uint8_t deccel4 = (uint8_t) deccel;
    
    int8_t speedM1One = (int8_t) (speedM1 >> 24);
    int8_t speedM1Two = (int8_t) (speedM1 >> 16);
    int8_t speedM1Three = (int8_t) (speedM1 >> 8);
    int8_t speedM1Four = (int8_t) speedM1;
    
    uint8_t distanceM1One = (uint8_t) (distanceM1 >> 24);
    uint8_t distanceM1Two = (uint8_t) (distanceM1 >> 16);
    uint8_t distanceM1Three = (uint8_t) (distanceM1 >> 8);
    uint8_t distanceM1Four = (uint8_t) distanceM1;
    
    int8_t speedM2One = (int8_t) (speedM2 >> 24);
    int8_t speedM2Two = (int8_t) (speedM2 >> 16);
    int8_t speedM2Three = (int8_t) (speedM2 >> 8);
    int8_t speedM2Four = (int8_t) speedM2;
    
    uint8_t distanceM2One = (uint8_t) (distanceM2 >> 24);
    uint8_t distanceM2Two = (uint8_t) (distanceM2 >> 16);
    uint8_t distanceM2Three = (uint8_t) (distanceM2 >> 8);
    uint8_t distanceM2Four = (uint8_t) distanceM2;
    
    // This packet declaration is long to accommodate the above variables
    unsigned char packet[35] = {ROBOCLAW_ADDRESS, commandID, accel1, accel2, accel3, accel4, speedM1One, speedM1Two, speedM1Three, speedM1Four, deccel1, deccel2, deccel3, deccel4, distanceM1One, distanceM1Two, distanceM1Three, distanceM1Four, accel1, accel2, accel3, accel4, speedM2One, speedM2Two, speedM2Three, speedM2Four, deccel1, deccel2, deccel3, deccel4, distanceM2One, distanceM2Two, distanceM2Three, distanceM2Four, buffer};
    uint16_t crc = crc16(packet, numBytes - 2);
    uint8_t crc1 = (uint8_t)(crc);
    uint8_t crc2 = (uint8_t)(crc >> 8);
   
    unsigned char msg[37] = {ROBOCLAW_ADDRESS, commandID, accel1, accel2, accel3, accel4, speedM1One, speedM1Two, speedM1Three, speedM1Four, deccel1, deccel2, deccel3, deccel4, distanceM1One, distanceM1Two, distanceM1Three, distanceM1Four, accel1, accel2, accel3, accel4, speedM2One, speedM2Two, speedM2Three, speedM2Four, deccel1, deccel2, deccel3, deccel4, distanceM2One, distanceM2Two, distanceM2Three, distanceM2Four, buffer, crc2, crc1};
    glb_numBytesToWaitForRoboclaw = 1;
    UART1_SendBytes(numBytes , msg);
    
//    CheckRoboclawReception();
//    
//    if (uart1_RxPacket[0] == 0xFF && glb_RoboclawNotResponding == false)
//    {
//        return true;
//    }
//    else
//    {
//        return false;
//    }
//    
    return true;
}

void Drive_Motors_Velocity (uint32_t accel, int32_t speedM1, int32_t speedM2)
{
    uint8_t commandID = 40;
    uint8_t numBytes = 16;
    
    uint8_t accel1 = (uint8_t) (accel >> 24);
    uint8_t accel2 = (uint8_t) (accel >> 16);
    uint8_t accel3 = (uint8_t) (accel >> 8);
    uint8_t accel4 = (uint8_t) accel;
    
    int8_t speedM1One = (int8_t) (speedM1 >> 24);
    int8_t speedM1Two = (int8_t) (speedM1 >> 16);
    int8_t speedM1Three = (int8_t) (speedM1 >> 8);
    int8_t speedM1Four = (int8_t) speedM1;
    
    int8_t speedM2One = (int8_t) (speedM2 >> 24);
    int8_t speedM2Two = (int8_t) (speedM2 >> 16);
    int8_t speedM2Three = (int8_t) (speedM2 >> 8);
    int8_t speedM2Four = (int8_t) speedM2;
    
    // This packet declaration is long to accommodate the above variables
    unsigned char packet[14] = {ROBOCLAW_ADDRESS, commandID, accel1, accel2, accel3, accel4, speedM1One, speedM1Two, speedM1Three, speedM1Four, speedM2One, speedM2Two, speedM2Three, speedM2Four};
    uint16_t crc = crc16(packet, numBytes - 2);
    uint8_t crc1 = (uint8_t)(crc);
    uint8_t crc2 = (uint8_t)(crc >> 8);
    unsigned char msg[16] = {ROBOCLAW_ADDRESS, commandID, accel1, accel2, accel3, accel4, speedM1One, speedM1Two, speedM1Three, speedM1Four, speedM2One, speedM2Two, speedM2Three, speedM2Four, crc2, crc1}; 
    UART1_SendBytes(numBytes , msg);
}

void Stop_Motors_Decel (uint32_t decel)
{
    uint8_t commandID = 40;
    uint8_t numBytes = 16;
    uint32_t accel = decel;
    uint8_t accel1 = (uint8_t) (accel >> 24);
    uint8_t accel2 = (uint8_t) (accel >> 16);
    uint8_t accel3 = (uint8_t) (accel >> 8);
    uint8_t accel4 = (uint8_t) accel;
    
    unsigned char packet[14] = {ROBOCLAW_ADDRESS, commandID, accel1, accel2, accel3, accel4, 0, 0, 0, 0, 0, 0, 0, 0};
    uint16_t crc = crc16(packet, numBytes - 2);
    uint8_t crc1 = (uint8_t)(crc);
    uint8_t crc2 = (uint8_t)(crc >> 8);
    unsigned char msg[16] = {ROBOCLAW_ADDRESS, commandID, accel1, accel2, accel3, accel4, 0, 0, 0, 0, 0, 0, 0, 0, crc2, crc1}; 
    UART1_SendBytes(numBytes , msg);
}

// Basic Drive Functions
void Drive_Forward_M1 (uint8_t speed) // Send: [Address,  (commandID), Speed (0-127), CRC(2 bytes)]
{
    uint8_t commandID = 1; 
    unsigned char packet[4] = {ROBOCLAW_ADDRESS, commandID, speed};
    uint16_t crc = crc16(packet, 3);
    uint8_t crc1 = (uint8_t)(crc);
    uint8_t crc2 = (uint8_t)(crc >> 8);
    unsigned char msg[6] = {ROBOCLAW_ADDRESS, commandID, speed, crc2, crc1}; 
    uint8_t numBytes = 5;
    UART1_SendBytes(numBytes , msg);
}// Receive: 0xFF

void Drive_M2_Position(uint32_t speed, uint32_t accel, uint32_t position)
{
    Flush_Rx_Buffer();
    uint8_t commandID = 66;
    uint8_t numBytes = 21; // include 2 bytes for crc
    
    uint8_t accel1 = (uint8_t) (accel >> 24);
    uint8_t accel2 = (uint8_t) (accel >> 16);
    uint8_t accel3 = (uint8_t) (accel >> 8);
    uint8_t accel4 = (uint8_t) accel;
    
    uint8_t speed1 = (uint8_t) (speed >> 24);
    uint8_t speed2 = (uint8_t) (speed >> 16);
    uint8_t speed3 = (uint8_t) (speed >> 8);
    uint8_t speed4 = (uint8_t) speed;
    
     uint8_t pos1 = (uint8_t) (position >> 24);
    uint8_t pos2 = (uint8_t) (position >> 16);
    uint8_t pos3 = (uint8_t) (position >> 8);
    uint8_t pos4 = (uint8_t) position;
    
    unsigned char packet[19] = {ROBOCLAW_ADDRESS, commandID, accel1, accel2, accel3, accel4, speed1,speed2,speed3,speed4,accel1, accel2, accel3, accel4,pos1,pos2,pos3,pos4,1};
    uint16_t crc = crc16(packet, 19);
    uint8_t crc1 = (uint8_t)(crc);
    uint8_t crc2 = (uint8_t)(crc >> 8);
    unsigned char msg[21] = {ROBOCLAW_ADDRESS, commandID, accel1, accel2, accel3, accel4, speed1,speed2,speed3,speed4,accel1, accel2, accel3, accel4,pos1,pos2,pos3,pos4,1,crc1, crc2};
    UART1_SendBytes(numBytes, msg);
    //waiting for confirmation
//    glb_numBytesToWaitForRoboclaw = 1;
//    while (uart1_RxPacket[0] != 0xff)
//    {
//        glb_RoboclawNotResponding = true;
//    }
//    
//    glb_RoboclawNotResponding = false;
}

void Drive_Forward_M2 (int8_t speed) // Send: [Address, (commandID), Speed (0-127), CRC(2 bytes)]
{
    uint8_t commandID = 5; 
    unsigned char packet[4] = {ROBOCLAW_ADDRESS, commandID, speed};
    uint16_t crc = crc16(packet, 3);
    uint8_t crc1 = (uint8_t)(crc);
    uint8_t crc2 = (uint8_t)(crc >> 8);
    unsigned char msg[6] = {ROBOCLAW_ADDRESS, commandID, speed, crc2, crc1}; 
    uint8_t numBytes = 5;
    UART1_SendBytes(numBytes , msg);
}// Receive: 0xFF

void Drive_Backward_M1 (uint8_t speed) // Send: [Address,  (commandID), Speed (0-127), CRC(2 bytes)]
{
    uint8_t commandID = 0; 
    unsigned char packet[4] = {ROBOCLAW_ADDRESS, commandID, speed};
    uint16_t crc = crc16(packet, 3);
    uint8_t crc1 = (uint8_t)(crc);
    uint8_t crc2 = (uint8_t)(crc >> 8);
    unsigned char msg[6] = {ROBOCLAW_ADDRESS, commandID, speed, crc2, crc1}; 
    uint8_t numBytes = 5;
    UART1_SendBytes(numBytes , msg);
} // Receive: 0xFF

void Drive_Backward_M2 (uint8_t speed) // Send: [Address,  (commandID), Speed (0-127), CRC(2 bytes)]
{
    uint8_t commandID = 4; 
    unsigned char packet[4] = {ROBOCLAW_ADDRESS, commandID, speed};
    uint16_t crc = crc16(packet, 3);
    uint8_t crc1 = (uint8_t)(crc);
    uint8_t crc2 = (uint8_t)(crc >> 8);
    unsigned char msg[6] = {ROBOCLAW_ADDRESS, commandID, speed, crc2, crc1}; 
    uint8_t numBytes = 5;
    UART1_SendBytes(numBytes , msg);
} // Receive: 0xFF

void Drive_Forward_Motors (uint8_t speed)
{
    Drive_Forward_M1(speed);
    Drive_Forward_M2(speed);
} 

void Drive_Backward_Motors (uint8_t speed) 
{
    Drive_Backward_M1(speed);
    Drive_Backward_M2(speed);
} 

void Stop_M1 (void) // Send: [Address, 6 (commandID), 64 (stop speed), CRC(2 bytes)]
{
    uint8_t speed = 64; // Stops motor
    uint8_t commandID = 6;
    unsigned char packet[4] = {ROBOCLAW_ADDRESS, commandID, speed};
    uint16_t crc = crc16(packet, 3);
    uint8_t crc1 = (uint8_t)(crc);
    uint8_t crc2 = (uint8_t)(crc >> 8);
    unsigned char msg[6] = {ROBOCLAW_ADDRESS, commandID, speed, crc2, crc1}; 
    uint8_t numBytes = 5;
    UART1_SendBytes(numBytes , msg);
} // Receive: 0xFF

void Stop_M2 (void) // Send: [Address, 7 (commandID), 64 (stop speed), CRC(2 bytes)]
{
    uint8_t speed = 64; // Stops motor
    uint8_t commandID = 7;
    unsigned char packet[4] = {ROBOCLAW_ADDRESS, commandID, speed};
    uint16_t crc = crc16(packet, 3);
    uint8_t crc1 = (uint8_t)(crc);
    uint8_t crc2 = (uint8_t)(crc >> 8);
    unsigned char msg[6] = {ROBOCLAW_ADDRESS, commandID, speed, crc2, crc1}; 
    uint8_t numBytes = 5;
    UART1_SendBytes(numBytes , msg);
} // Receive: 0xFF

void Stop_Motors (void) // Send: [Address, 12 (commandID), 64 (stop speed), CRC(2 bytes)]
{
    Stop_M1();
    Stop_M2();
} // Receive: 0xFF

/*
 Encoder Functions
 */

void Reset_Quadrature_Encoder_Counters (void) // Send: [Address, 20 (command ID), CRC (2 bytes)]
{
    uint8_t commandID = 20;
    uint8_t numBytes = 4;
    unsigned char packet[2] = {ROBOCLAW_ADDRESS, commandID};
    uint16_t crc = crc16(packet, 2);
    uint8_t crc1 = (uint8_t)(crc);
    uint8_t crc2 = (uint8_t)(crc >> 8);
    unsigned char msg[4] = {ROBOCLAW_ADDRESS, commandID, crc1, crc2};
    UART1_SendBytes(numBytes, msg);
    glb_M1EncoderCount = 0;
    glb_M2EncoderCount = 0;
//    glb_numBytesToWaitForRoboclaw = 1;
    
//    CheckRoboclawReception();
//    
//    Read_Encoder_Counters();
//    if (glb_M1EncoderCount <= 3 && glb_M2EncoderCount <= 3)
//    {
//        glb_EncodersReset = true;
//    }
//    else
//    {
//        glb_EncodersReset = false;
//    }
} 

void Read_Encoder_Counters (void) // TO Comment
{
    uint8_t commandID = 78;
    uint8_t numBytes = 2;
    
    unsigned char msg[2] = {ROBOCLAW_ADDRESS, commandID};
    glb_numBytesToWaitForRoboclaw = 10;
    Flush_Rx_Buffer();
    UART1_SendBytes(numBytes, msg);
   
  //  CheckRoboclawReception();
    
    glb_M1EncoderCount = ((uint32_t)uart1_RxPacket[0] << 24) | ((uint32_t)uart1_RxPacket[1] << 16) | ((uint32_t)uart1_RxPacket[2] << 8) | ((uint32_t)uart1_RxPacket[3]);
    glb_M2EncoderCount = ((uint32_t)uart1_RxPacket[4] << 24) | ((uint32_t)uart1_RxPacket[5] << 16) | ((uint32_t)uart1_RxPacket[6] << 8) | ((uint32_t)uart1_RxPacket[7]);
    Flush_Rx_Buffer();
    
} // TO Comment

/*
 Status read functions
 */
void Read_Firmware_Version_RC (void) // Send: [Address, 21]
{
    glb_numBytesToWaitForRoboclaw = 26;
    UART1_SendByte(ROBOCLAW_ADDRESS); // Sends address 0x80 (decimal: 128)
    UART1_SendByte(21); // Sends command ID 21;
} // Receive: ["Roboclaw 10.2A v4.1.11", 10, 0, CRC (2 byte)]

void Read_Main_Battery_Voltage_Level_RC (void) // Send: [Address, 24]
{
    glb_numBytesToWaitForRoboclaw = 4;
    UART1_SendByte(ROBOCLAW_ADDRESS);
    UART1_SendByte(24);
} // Receive: [Value(2 bytes), CRC (2 bytes)]

void Read_Temperature (void)
{
    glb_numBytesToWaitForRoboclaw = 4;
    UART1_SendByte(ROBOCLAW_ADDRESS);
    UART1_SendByte(82);
}

uint32_t Read_M1Speed(void)
{
    uint8_t commandID = 18;
    uint8_t numBytes = 4;
    unsigned char packet[2] = {ROBOCLAW_ADDRESS, commandID};
    uint16_t crc = crc16(packet, 2);
    uint8_t crc1 = (uint8_t)(crc);
    uint8_t crc2 = (uint8_t)(crc >> 8);
    unsigned char msg[4] = {ROBOCLAW_ADDRESS, commandID, crc1, crc2};
    glb_numBytesToWaitForRoboclaw = 7;
    Flush_Rx_Buffer();
    UART1_SendBytes(numBytes, msg);
    
   
    CheckRoboclawReception();

    
    return ((uint32_t)uart1_RxPacket[0] << 24) | ((uint32_t)uart1_RxPacket[1] << 16) | ((uint32_t)uart1_RxPacket[2] << 8) | ((uint32_t)uart1_RxPacket[3]);
    
}

uint32_t Read_M2Speed(void)
{
    uint8_t commandID = 19;
    uint8_t numBytes = 4;
    unsigned char packet[2] = {ROBOCLAW_ADDRESS, commandID};
    uint16_t crc = crc16(packet, 2);
    uint8_t crc1 = (uint8_t)(crc);
    uint8_t crc2 = (uint8_t)(crc >> 8);
    unsigned char msg[4] = {ROBOCLAW_ADDRESS, commandID, crc1, crc2};
    glb_numBytesToWaitForRoboclaw = 7;
    Flush_Rx_Buffer();
    UART1_SendBytes(numBytes, msg);
    
   
    CheckRoboclawReception();
 
    
    return ((uint32_t)uart1_RxPacket[0] << 24) | ((uint32_t)uart1_RxPacket[1] << 16) | ((uint32_t)uart1_RxPacket[2] << 8) | ((uint32_t)uart1_RxPacket[3]);
    
}

bool areMotorsStopped(void)
{
    uint32_t M1Speed = 0xffffffff;
    uint32_t M2Speed = 0xffffffff;
    M1Speed = Read_M1Speed();
    M2Speed = Read_M2Speed();
    if (M1Speed > 0 || M2Speed >0 )
    {
        return false;
    }
    else
    {
        return true;
    }
    
}

void CheckRoboclawReception (void)
 {
    glb_RoboclawNotResponding = false;
    TMR5_Stop();
    __delay_us(3);
    TMR5_Start();   
    while (!Roboclaw_DataRecieved && TMR5 < RESPONSE_TIMEOUT) {}
    TMR5_Stop();
    if (!Roboclaw_DataRecieved && TMR5 >= RESPONSE_TIMEOUT)
    {
        glb_RoboclawNotResponding = true;
    }
}
/*
 Write Functions
 */