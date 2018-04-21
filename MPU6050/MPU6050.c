/* Includes */
#include "MPU6050.h"
#include "stdint.h"
/** @defgroup MPU6050_Library
* @{
*/
InitMPU6050_t Init_MPU6050;
I2C_HandleTypeDef hi2c1;

//I2C_HandleTypeDef *hi2c_MPU6050 = &hi2c1 ;

// example main 
/*
  * void main()
  *{
  *   HAL_Init();
  *   SystemClock_Config();
  *   MX_GPIO_Init();
  *   MX_DMA_Init();
  *   //MX_I2C1_Init();
  *   MX_SDIO_SD_Init();
  *   MX_USART1_UART_Init();
  *   MX_FATFS_Init();
	*   MPU6050_Initialize();
  *   while (1)
  *   {
	*   	MPU6050_GetRawAccelTempGyro(MPU6050data);
	*   	HAL_Delay(6);
  *   }
  *}
*/


/**
* @brief  Initializes the I2C peripheral used to drive the MPU6050
* @param  None
* @return None
*/
void MPU6050_I2C_Init(void)
{
	#ifdef FAST_I2C_MODE
	#define I2C_SPEED_MODE 400000
	#define I2C_DUTYCYCLE I2C_DUTYCYCLE_16_9
	#else
	#define I2C_SPEED_MODE 100000
	#define I2C_DUTYCYCLE I2C_DUTYCYCLE_2
	#endif
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = I2C_SPEED_MODE;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
		_Error_Handler(__FILE__, __LINE__);
     // thoong bao loi len led 
  }

}

/** Power on and prepare for general usage.
 * This will activate the device and take it out of sleep mode (which must be done
 * after start-up). This function also sets both the accelerometer and the gyroscope
 * to their most sensitive settings, namely +/- 2g and +/- 250 degrees/sec, and sets
 * the clock source to use the X Gyro for reference, which is slightly better than
 * the default internal clock source.
 */
NT_MPU6050_Result_t MPU6050_Initialize(NT_MPU6050_Device_t DeviceNumber,NT_MPU6050_Accelerometer_t AccelerometerSensitivity , NT_MPU6050_Gyroscope_t GyroscopeSensitivity) 
{
  
  Init_MPU6050.Device_t=DeviceNumber;
  Init_MPU6050.Accelerometer_t=AccelerometerSensitivity;
  Init_MPU6050.Gyroscope_t= GyroscopeSensitivity;
  if(AccelerometerSensitivity == NT_MPU6050_Accelerometer_2G)
    {
      Init_MPU6050.ACCEL_LSB_t= NT_MPU6050_ACCEL_LSB_2;
    }
  if(AccelerometerSensitivity == NT_MPU6050_Accelerometer_4G)
    {
      Init_MPU6050.ACCEL_LSB_t= NT_MPU6050_ACCEL_LSB_4;
    }
  if(AccelerometerSensitivity == NT_MPU6050_Accelerometer_8G)
    {
      Init_MPU6050.ACCEL_LSB_t= NT_MPU6050_ACCEL_LSB_8;
    }
  if(AccelerometerSensitivity == NT_MPU6050_Accelerometer_16G)
    {
      Init_MPU6050.ACCEL_LSB_t= NT_MPU6050_ACCEL_LSB_16;
    }
	MPU6050_I2C_Init();
	//NEW
		MPU6050_I2C_ByteWrite(Init_MPU6050.Device_t,MPU6050_RA_PWR_MGMT_1, 0x00);
		MPU6050_I2C_ByteWrite(Init_MPU6050.Device_t,MPU6050_RA_SMPLRT_DIV, 0x4);
		MPU6050_SetFullScaleGyroRange(MPU6050_GYRO_FS_1000);
		MPU6050_SetFullScaleAccelRange(MPU6050_ACCEL_FS_16);
		
//	MPU6050_SetClockSource(MPU6050_CLOCK_PLL_XGYRO);
//	MPU6050_SetFullScaleGyroRange(MPU6050_GYRO_FS_1000);
//	MPU6050_SetFullScaleAccelRange(MPU6050_ACCEL_FS_16);
//	MPU6050_SetSleepModeStatus(DISABLE);
//	
//	MPU6050_setSleepDisabled();
//	HAL_Delay(10);

	//MPU6050_WriteBits(Init_MPU6050.Device_t,MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, MPU6050_CLOCK_PLL_XGYRO);
//	//set DLPF bandwidth to 42Hz
	//MPU6050_WriteBits(Init_MPU6050.Device_t, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH,MPU6050_DLPF_BW_256 ); //MPU6050_DLPF_BW_42);
//    //set sampe rate
	//MPU6050_I2C_ByteWrite(Init_MPU6050.Device_t,MPU6050_RA_SMPLRT_DIV, 0x04); //1khz / (1 + 4) = 200Hz
//	//set gyro range
//	MPU6050_WriteBits(Init_MPU6050.Device_t,MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, Init_MPU6050.Gyroscope_t);
//	//set accel range
//	MPU6050_WriteBits(Init_MPU6050.Device_t,MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH,Init_MPU6050.Accelerometer_t);
  return NT_MPU6050_Result_Ok;
}


/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, FALSE otherwise
 */
bool MPU6050_TestConnection(void) 
{
    if(MPU6050_GetDeviceID() == 0x34) //0b110100; 8-bit representation in hex = 0x34
      return TRUE;
    else
      return FALSE;
}
// WHO_AM_I register

/** Get Device ID.
 * This register is used to verify the identity of the device (0b110100).
 * @return Device ID (should be 0x68, 104 dec, 150 oct)
 * @see MPU6050_RA_WHO_AM_I
 * @see MPU6050_WHO_AM_I_BIT
 * @see MPU6050_WHO_AM_I_LENGTH
 */
unsigned char MPU6050_GetDeviceID(void)
{
    uint8_t tmp;
    MPU6050_ReadBits(Init_MPU6050.Device_t, MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, &tmp);
    return tmp; 
}
/** Set clock source setting.
 * An internal 8MHz oscillator, gyroscope based clock, or external sources can
 * be selected as the MPU-60X0 clock source. When the internal 8 MHz oscillator
 * or an external source is chosen as the clock source, the MPU-60X0 can operate
 * in low power modes with the gyroscopes disabled.
 *
 * Upon power up, the MPU-60X0 clock source defaults to the internal oscillator.
 * However, it is highly recommended that the device be configured to use one of
 * the gyroscopes (or an external clock source) as the clock reference for
 * improved stability. The clock source can be selected according to the following table:
 *
 * <pre>
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
 * </pre>
 *
 * @param source New clock source setting
 * @see MPU6050_GetClockSource()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_CLKSEL_BIT
 * @see MPU6050_PWR1_CLKSEL_LENGTH
 */
 
 
void MPU6050_SetClockSource(uint8_t source) 
{
    MPU6050_WriteBits(Init_MPU6050.Device_t, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see MPU6050_GetFullScaleGyroRange()
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
void MPU6050_SetFullScaleGyroRange(uint8_t range) 
{
    MPU6050_WriteBits(Init_MPU6050.Device_t, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

// GYRO_CONFIG register

/** Get full-scale gyroscope range.
 * The FS_SEL parameter allows setting the full-scale range of the gyro sensors,
 * as described in the table below.
 *
 * <pre>
 * 0 = +/- 250 degrees/sec
 * 1 = +/- 500 degrees/sec
 * 2 = +/- 1000 degrees/sec
 * 3 = +/- 2000 degrees/sec
 * </pre>
 *
 * @return Current full-scale gyroscope range setting
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
unsigned char MPU6050_GetFullScaleGyroRange(void) 
{
    uint8_t tmp;
    MPU6050_ReadBits(Init_MPU6050.Device_t, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, &tmp);
    return tmp;
}
/** Get full-scale accelerometer range.
 * The FS_SEL parameter allows setting the full-scale range of the accelerometer
 * sensors, as described in the table below.
 *
 * <pre>
 * 0 = +/- 2g
 * 1 = +/- 4g
 * 2 = +/- 8g
 * 3 = +/- 16g
 * </pre>
 *
 * @return Current full-scale accelerometer range setting
 * @see MPU6050_ACCEL_FS_2
 * @see MPU6050_RA_ACCEL_CONFIG
 * @see MPU6050_ACONFIG_AFS_SEL_BIT
 * @see MPU6050_ACONFIG_AFS_SEL_LENGTH
 */
unsigned char MPU6050_GetFullScaleAccelRange(void) 
{
    uint8_t tmp;
    MPU6050_ReadBits(Init_MPU6050.Device_t, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, &tmp);
    return tmp;
}
/** Set full-scale accelerometer range.
 * @param range New full-scale accelerometer range setting
 * @see MPU6050_GetFullScaleAccelRange()
 */
void MPU6050_SetFullScaleAccelRange(uint8_t range) 
{
    MPU6050_WriteBits(Init_MPU6050.Device_t, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

/** Get sleep mode status.
 * Setting the SLEEP bit in the register puts the device into very low power
 * sleep mode. In this mode, only the serial interface and internal registers
 * remain active, allowing for a very low standby current. Clearing this bit
 * puts the device back into normal mode. To save power, the individual standby
 * selections for each of the gyros should be used if any gyro axis is not used
 * by the application.
 * @return Current sleep mode enabled status
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_SLEEP_BIT
 */
bool MPU6050_GetSleepModeStatus(void) 
{
    uint8_t tmp;
    MPU6050_ReadBit(Init_MPU6050.Device_t, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, &tmp);
    if(tmp == 0x00)
      return FALSE;
    else
      return TRUE;    
}
/** Set sleep mode status.
 * @param enabled New sleep mode enabled status
 * @see MPU6050_GetSleepModeStatus()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_SLEEP_BIT
 */
void MPU6050_SetSleepModeStatus(FunctionalState NewState) 
{
    MPU6050_WriteBit(Init_MPU6050.Device_t, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, NewState);
}

/** Get raw 6-axis motion sensor readings (accel/temp/gyro).
 * Retrieves all currently available motion sensor values.
 * @param AccelGyro 16-bit signed integer array of length 7
 * @see MPU6050_RA_ACCEL_XOUT_H
 * data 0 -> 2 : Accel
 * data 3      : Temp
 * data 4 -> 6 : Gyro
 */
void MPU6050_GetRawAccelTempGyro(DataMpu6050 * dataMPU6050)
{
    uint8_t dataR[14];
    int16_t ax = 0;
    int16_t ay = 0;
    int16_t az = 0;
    int16_t gx = 0;
    int16_t gy = 0;
    int16_t gz = 0;
    MPU6050_I2C_BufferRead(Init_MPU6050.Device_t, dataR, MPU6050_RA_ACCEL_XOUT_H, 14);
    ax  = (dataR[0]<<8)|dataR[1];
    ay  = (dataR[2]<<8)|dataR[3];
    az  = (dataR[4]<<8)|dataR[5];
  
    gx = (dataR[8]<<8)|dataR[9];
    gy = (dataR[10]<<8)|dataR[11];
    gz = (dataR[12]<<8)|dataR[13];
  
    dataMPU6050->NT_MPU6050_Temp = (dataR[6]<<8)|dataR[7];

    dataMPU6050->NT_int_MPU6050_Ax = ax;
    dataMPU6050->NT_int_MPU6050_Ay = ay;
    dataMPU6050->NT_int_MPU6050_Az = az;
    dataMPU6050->NT_int_MPU6050_Gx = gx;
    dataMPU6050->NT_int_MPU6050_Gy = gy;
    dataMPU6050->NT_int_MPU6050_Gz = gz;
}
void MPU6050_convert(DataMpu6050 * dataMPU6050)
{
    int16_t ax = dataMPU6050->NT_int_MPU6050_Ax ;
    int16_t ay = dataMPU6050->NT_int_MPU6050_Ay ;
    int16_t az = dataMPU6050->NT_int_MPU6050_Az ;
    int16_t gx = dataMPU6050->NT_int_MPU6050_Gx ;
    int16_t gy = dataMPU6050->NT_int_MPU6050_Gy ;
    int16_t gz = dataMPU6050->NT_int_MPU6050_Gz ;
    dataMPU6050->NT_MPU6050_Ax = (double)ax/Init_MPU6050.ACCEL_LSB_t;
    dataMPU6050->NT_MPU6050_Ay = (double)ay/Init_MPU6050.ACCEL_LSB_t;
    dataMPU6050->NT_MPU6050_Az = ((double)az-4500)/Init_MPU6050.ACCEL_LSB_t;
    dataMPU6050->NT_MPU6050_Gx =((double)gx-MPU6050_GXOFFSET)/MPU6050_GYRO_LSB_2000;
    dataMPU6050->NT_MPU6050_Gy =((double)gy-MPU6050_GYOFFSET)/MPU6050_GYRO_LSB_2000;
    dataMPU6050->NT_MPU6050_Gz =((double)gz-MPU6050_GZOFFSET)/MPU6050_GYRO_LSB_2000;
}


/** Write multiple bits in an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 */
void MPU6050_WriteBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) 
{
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t tmp, mask;
    MPU6050_I2C_BufferRead(slaveAddr, &tmp, regAddr, 1);  
    mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1); // shift data into correct position
    data &= mask; // zero all non-important bits in data
    tmp &= ~(mask); // zero all important bits in existing byte
    tmp |= data; // combine data with existing byte
    MPU6050_I2C_ByteWrite(slaveAddr,regAddr,tmp);   
}
/** write a single bit in an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param value New bit value to write
 */
void MPU6050_WriteBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data) 
{
    uint8_t tmp;
    MPU6050_I2C_BufferRead(slaveAddr, &tmp, regAddr, 1);  
    tmp = (data != 0) ? (tmp | (1 << bitNum)) : (tmp & ~(1 << bitNum));
    MPU6050_I2C_ByteWrite(slaveAddr,regAddr,tmp); 
}
/** Read multiple bits from an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-7)
 * @param length Number of bits to read (not more than 8)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in readTimeout)
 */
void MPU6050_ReadBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data) 
{
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    uint8_t tmp, mask;
    MPU6050_I2C_BufferRead(slaveAddr, &tmp, regAddr, 1); 
    mask = ((1 << length) - 1) << (bitStart - length + 1);
    tmp &= mask;
    tmp >>= (bitStart - length + 1);
    *data = tmp;
}

/** Read a single bit from an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-7)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in readTimeout)
 */
void MPU6050_ReadBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data) 
{
    uint8_t tmp;
    MPU6050_I2C_BufferRead(slaveAddr, &tmp, regAddr, 1);  
    *data = tmp & (1 << bitNum);
}



/**
* @brief  Writes one byte to the  MPU6050.
* @param  slaveAddr : slave address MPU6050_DEFAULT_ADDRESS
* @param  pBuffer : pointer to the buffer  containing the data to be written to the MPU6050.
* @param  writeAddr : address of the register in which the data will be written
* @return None
*/
void MPU6050_I2C_ByteWrite(uint8_t slaveAddr , uint8_t writeAddr, uint8_t pBuffer)
{
	//HAL_I2C_Master_Transmit(&I2C_MPU6050,0xD0,pBuffer,1,2000);
	HAL_I2C_Mem_Write(&I2C_MPU6050,slaveAddr,writeAddr,I2C_MEMADD_SIZE_8BIT,& pBuffer,1,2000);
}

/**
* @brief  Reads a block of data from the MPU6050.
* @param  slaveAddr  : slave address MPU6050_DEFAULT_ADDRESS
* @param  pBuffer : pointer to the buffer that receives the data read from the MPU6050.
* @param  readAddr : MPU6050's internal address to read from.
* @param  NumByteToRead : number of bytes to read from the MPU6050 ( NumByteToRead >1  only for the Mgnetometer readinf).
* @return None
*/

void MPU6050_I2C_BufferRead(uint8_t slaveAddr,uint8_t* pBuffer, uint8_t readAddr, uint16_t NumByteToRead)
{
 HAL_I2C_Mem_Read(&I2C_MPU6050,slaveAddr,readAddr,I2C_MEMADD_SIZE_8BIT,pBuffer,NumByteToRead,2000);

}


/*
 * set sleep disabled
 */
void MPU6050_setSleepDisabled() {
	MPU6050_WriteBit(Init_MPU6050.Device_t,MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, 0);
}

/*
 * set sleep enabled
 */
void MPU6050_setSleepEnabled() {
	MPU6050_WriteBit(Init_MPU6050.Device_t, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, 1);
}






