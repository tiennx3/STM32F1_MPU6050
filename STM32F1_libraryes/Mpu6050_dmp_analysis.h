#ifndef _TM_STM32_MPU6050_DMP_H
#define _TM_STM32_MPU6050_DMP_H

/*  C++ detection  */
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
#include <tm_stm32_mpu6050_dmp.h>
	
typedef struct MPU6050_data_t{
	int16_t Accelerometer_X; /*!< Accelerometer value X axis */
	int16_t Accelerometer_Y; /*!< Accelerometer value Y axis */
	int16_t Accelerometer_Z; /*!< Accelerometer value Z axis */
	int16_t Gyroscope_X;     /*!< Gyroscope value X axis */
	int16_t Gyroscope_Y;     /*!< Gyroscope value Y axis */
	int16_t Gyroscope_Z;     /*!< Gyroscope value Z axis */
	float Yaw;
	float Pitch;
	float Roll;
	uint16_t time_ms;
} MPU6050_data_t;
	
	
/*   C++ detection  */
#ifdef __cplusplus
}
#endif

#endif
