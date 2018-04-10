
#ifndef _MPU6050_6AXIS_MOTIONAPPS20_H_
#define _MPU6050_6AXIS_MOTIONAPPS20_H_

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup TM_STM32Fxxx_HAL_Libraries
 * @{
 */

/**
 * @defgroup TM_MPU6050
 * @brief    MPU6050 library for STM32Fxxx - http://stm32f4-discovery.net/2015/10/hal-library-30-mpu6050-for-stm32fxxx
 * @{
 *
 * \par Features
 *
 * Library supports basic operation with MPU6050 device:
 *
\verbatim
- Read accelerometer, gyroscope and temperature data,
- Set custom output data rate for measurements
- Enable/disable interrupts
- Up to 2 MPU devices at a time
\endverbatim
 *
 * \par MPU6050 interrupts
 *
 * When you enable interrupts using @ref TM_MPU6050_EnableInterrupts function, 
 * "DataReady" and "MotionDetected" interrupts are enabled. 
 *
 * MPU pin for interrupt detection on STM device is rising edge and triggers on any interrupt.
 *
 * You can read interrupts status register to detect which interrupt happened using @ref TM_MPU6050_ReadInterrupts function.
 *
 * \par MPU6050 data rate
 *
 * Device can output data at specific rate. It has 8-bit register with custom value to set data rate you need.
 *
 * Equation for data rate is below:
\f[
	DataRate = 
		\frac{8 MHz}{REGVAL + 1}
\f]
 * where:
 *  - 8 Mhz is Gyro internal output used for data rate
 *  - REGVAL is a value to be used in @ref TM_MPU6050_SetDataRate function
 *
 * \note  There are already some predefined constants in library for some "standard" data rates
 *
 * \par Default pinout
 * 
@verbatim
MPU6050     STM32Fxxx     Descrption
 
SCL         PB6           Clock line for I2C
SDA         PB7           Data line for I2C
IRQ         -             User selectable pin if needed. Interrupts for STM must be manually enabled by user.
VCC         3.3V
GND         GND
AD0         -             If pin is low, I2C address is 0xD0, if pin is high, the address is 0xD2
@endverbatim
 *
 * To change default pinout for I2C, you need to open defines.h file and copy/edit some defines:
 *
\code
//Set I2C used
MPU6050_I2C               I2C1  
//Set I2C pins used
MPU6050_I2C_PINSPACK      TM_I2C_PinsPack_1
\endcode
 *
 * \par Changelog
 *
@verbatim
 Version 1.0
  - First release
@endverbatim
 *
 * \par Dependencies
 *
@verbatim
 - STM32Fxxx HAL
 - defines.h
 - TM I2C
@endverbatim
 */
 
#include "MPU6050_dmp.h"
#include "helper_3dmath.h"

#ifdef DEBUG
    #define DEBUG_PRINT(x) Serial.print(x)
    #define DEBUG_PRINTF(x, y) Serial.print(x, y)
    #define DEBUG_PRINTLN(x) Serial.println(x)
    #define DEBUG_PRINTLNF(x, y) Serial.println(x, y)
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTF(x, y)
    #define DEBUG_PRINTLN(x)
    #define DEBUG_PRINTLNF(x, y)
#endif

#define MPU6050_DMP_CODE_SIZE       1929    // dmpMemory[]
#define MPU6050_DMP_CONFIG_SIZE     192     // dmpConfig[]
#define MPU6050_DMP_UPDATES_SIZE    47      // dmpUpdates[]


/* ================================================================================================ *
 | Default MotionApps v2.0 42-byte FIFO packet structure:                                           |
 |                                                                                                  |
 | [QUAT W][      ][QUAT X][      ][QUAT Y][      ][QUAT Z][      ][GYRO X][      ][GYRO Y][      ] |
 |   0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  |
 |                                                                                                  |
 | [GYRO Z][      ][ACC X ][      ][ACC Y ][      ][ACC Z ][      ][      ]                         |
 |  24  25  26  27  28  29  30  31  32  33  34  35  36  37  38  39  40  41                          |
 * ================================================================================================ */


extern uint8_t *dmpPacketBuffer;
extern uint16_t dmpPacketSize;

uint8_t MPU6050_dmpInitialize(void);
bool MPU6050_dmpPacketAvailable(void);
uint8_t MPU6050_dmpGetAccel_data32(int32_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetAccel_data16(int16_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetAccelVectorInt16(VectorInt16_t *v, const uint8_t* packet);
uint8_t MPU6050_dmpGetQuaternion_int32(int32_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetQuaternion_int16(int16_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetQuaternion(Quaternion_t *q, const uint8_t* packet);
uint8_t MPU6050_dmpGetGyro_int32(int32_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetGyro_int16(int16_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetGyro_VectorInt16(VectorInt16_t *v, const uint8_t* packet);
uint8_t MPU6050_dmpGetLinearAccel_VectorInt16(VectorInt16_t *v, VectorInt16_t *vRaw, VectorFloat_t *gravity);
uint8_t MPU6050_dmpGetLinearAccelInWorld(VectorInt16_t *v, VectorInt16_t *vReal, Quaternion_t *q);
uint8_t MPU6050_dmpGetGravity(VectorFloat_t *v, Quaternion_t *q);
uint8_t MPU6050_dmpGetEuler(float *data, Quaternion_t *q);
uint8_t MPU6050_dmpGetYawPitchRoll(float *data, Quaternion_t *q, VectorFloat_t *gravity);
uint8_t MPU6050_dmpProcessFIFOPacket(const unsigned char *dmpData);
uint8_t MPU6050_dmpReadAndProcessFIFOPacket(uint8_t numPackets, uint8_t *processed);
uint16_t MPU6050_dmpGetFIFOPacketSize(void);

/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif
