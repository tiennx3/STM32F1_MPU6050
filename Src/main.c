/*
* Note :
*		- Connecter:
*					STM32F1      MPU6050
*						PB6					SCL
*						PB7					SDA
*						PA2					INT
*
*		- user UART 
*					TX     PA9
*					RX		 PA10
*		
*/


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "MPU6050.h"
#include "MPU6050_dmp_6axis_MotionApps20.h"
bool Debug_app = true ;
// TIM2
TIM_HandleTypeDef htim2;
static void MX_TIM2_Init(void);
void timer_enabel(TIM_HandleTypeDef *htim);
void time_disabel(TIM_HandleTypeDef *htim);
void enabelcheck(void);
void disabel_check(void);
bool intimecheck = false;
uint8_t time_count=0;
static void MX_GPIO_Init(void);
// END TIM2
UART_HandleTypeDef huart1;
void SystemClock_Config(void);
static void MX_USART1_UART_Init(void);
uint8_t check=0;

// MPU6050 data value
uint32_t read = 0;
uint8_t devStatus;
uint8_t mpuIntStatus; 
DataMpu6050 MPU6050data;

typedef enum {
	KHONG_NGA = 0x00,
	NGA = 0x01,   
	NGA_NGUA = 0x02,  
	NGA_SAP = 0x03,
	NGA_TRAI= 0x04,
	NGA_PHAI= 0x05,
} NT_nga_t;

NT_nga_t Loai_nga = KHONG_NGA;


int8_t a_status = 0;
int8_t ax_status = 0;
int8_t ay_status = 0;
int8_t az_status = 0;
int8_t gx_status = 0;
int8_t gy_status = 0;
int8_t gz_status = 0;
int8_t roll_x_status = 0;
int8_t pitch_y_status = 0;
int8_t yaw_z_status = 0;
double Pitch_y;
double Roll_x;
double ax;
double ay;
double az;
double a ;
bool dmpReady = false;
uint16_t packetSize;
double A= 0.0;
data_MPU6050_t data_offset;
Quaternion_t q; 
VectorFloat_t gravity;
float ypr[3]; 
bool Getdata_offset = true;
float Yaw,Pitch,Roll;
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
void reset_status(void);
void check_status (void);

uint16_t fifoCount;    
uint8_t fifoBuffer[64];
unsigned char buff_char[50];
uint8_t k=0;
uint8_t dataz=0;
// END MPU6050 data value
int main(void)
{
  HAL_Init();
  SystemClock_Config();
	__HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
	MX_GPIO_Init();
  MX_USART1_UART_Init();
	MX_TIM2_Init();
	//HAL_TIM_Base_Start_IT(&htim2);
  MPU6050_Initialize(NT_MPU6050_Device_AD0_LOW,NT_MPU6050_Accelerometer_2G,NT_MPU6050_Gyroscope_2000s);
	MPU6050address(0xD0);
	//HAL_Delay(1000);

//	while(1)
//		{
//			MPU6050_GetRawAccelTempGyro(&MPU6050data);
//			MPU6050_convert(&MPU6050data);
//			HAL_Delay(1000);
//		}
	MPU6050_initialize();
	GPIO_Init_IRQ(GPIOA,GPIO_PIN_2,EXTI2_IRQn);
	devStatus = MPU6050_dmpInitialize();
	
//	MPU6050_setXGyroOffset(220);    // value set up only MPU6050 module
//	MPU6050_setYGyroOffset(76);			// value set up only MPU6050 module
//	MPU6050_setZGyroOffset(-85);		// value set up only MPU6050 module
//	MPU6050_setZAccelOffset(1788);	// value set up only MPU6050 module
	MPU6050_setXGyroOffset(95);    // value set up only MPU6050 module
	MPU6050_setYGyroOffset(30);			// value set up only MPU6050 module
	MPU6050_setZGyroOffset(-52);		// value set up only MPU6050 module
	MPU6050_setZAccelOffset(1788);	// value set up only MPU6050 module
	MPU6050_setIntEnabled(0x12);
	if (devStatus == 0) {
		MPU6050_setDMPEnabled(true);
		mpuIntStatus = MPU6050_getIntStatus();
		dmpReady = true;
		packetSize = MPU6050_dmpGetFIFOPacketSize();
	}
	HAL_Delay(10000);
	while(1)
	{
		while (!mpuInterrupt && fifoCount <= packetSize);
				mpuInterrupt = false;
				mpuIntStatus = MPU6050_getIntStatus();
				fifoCount = MPU6050_getFIFOCount();
				if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
						// reset so we can continue cleanly
						MPU6050_resetFIFO();
						}else if (mpuIntStatus & 0x02) {
								// wait for correct available data length, should be a VERY short wait
								while (fifoCount < packetSize) fifoCount = MPU6050_getFIFOCount();
								// read a packet from FIFO
								MPU6050_getFIFOBytes(fifoBuffer, packetSize);
								// track FIFO count here in case there is > 1 packet available
								// (this lets us immediately read more without waiting for an interrupt)
								fifoCount -= packetSize;
								MPU6050_dmpGetQuaternion(&q, fifoBuffer);
								MPU6050_dmpGetGravity(&gravity, &q);
								MPU6050_dmpGetYawPitchRoll(ypr, &q, &gravity);
								Yaw=ypr[0]*180/3.14;
								Pitch=ypr[1]*180/3.14;
								Roll=ypr[2]*180/3.14;
								MPU6050_GetRawAccelTempGyro(&MPU6050data);
								MPU6050_convert(&MPU6050data);
								A= sqrt((MPU6050data.NT_MPU6050_Ax*MPU6050data.NT_MPU6050_Ax)+(MPU6050data.NT_MPU6050_Ay*MPU6050data.NT_MPU6050_Ay)+(MPU6050data.NT_MPU6050_Az*MPU6050data.NT_MPU6050_Az));
								if(Getdata_offset == false)
								{
									if(intimecheck == true)
									{
										Pitch_y = Pitch-data_offset.pitch_y;
										Roll_x = Roll-data_offset.roll_x;
										ax = MPU6050data.NT_MPU6050_Ax - data_offset.ax;
										ay = MPU6050data.NT_MPU6050_Ay - data_offset.ay;
										az = MPU6050data.NT_MPU6050_Az - data_offset.az;	
										a = A - data_offset.a;	
										// pitch-y
										if(pitch_y_status ==0){
											if(Pitch_y >45)
											{
												pitch_y_status=1;
											}
											if(Pitch_y <-45)
											{
												pitch_y_status=-1;
											}
										}
										// roll x
										if(roll_x_status ==0){
											if(Roll_x >60)
											{
												roll_x_status=1;
											}
											if(Roll_x <-60)
											{
												roll_x_status=-1;
											}
										}
										
										// ax
										if(ax_status ==0){
											if(ax>0.7)
											{
												ax_status = 1;
											}
											if(ax<-0.7)
											{
												ax_status = -1;
											}
										}
										// ay
										if(ay_status ==0){
											if(ay>0.7)
											{
												ay_status = 1;
											}
											if(ay<-0.7)
											{
												ay_status = -1;
											}
										}
										// az
										if(az_status ==0){
											if(az>0.7)
											{
												az_status = 1;
											}
											if(az<-0.7)
											{
												az_status = -1;
											}
										}
										if(a_status ==0){
											if(a>0.5)
											{
												a_status = 1;
											}
										}
									}
									else  // check tung gia tri de bat timer 
									{
										Pitch_y = Pitch-data_offset.pitch_y;
										Roll_x = Roll-data_offset.roll_x;
										ax = MPU6050data.NT_MPU6050_Ax - data_offset.ax;
										ay = MPU6050data.NT_MPU6050_Ay - data_offset.ay;
										az = MPU6050data.NT_MPU6050_Az - data_offset.az;	
										a = A - data_offset.a;	
											if(Pitch_y >45)
											{
												pitch_y_status=1;
												enabelcheck();
											} else {
														if(Pitch_y <-45)
														{
															pitch_y_status=-1;
															enabelcheck();
														} else {
																if(Roll_x >60)   // roll
																	{
																		roll_x_status=1;
																		enabelcheck();
																	} else {
																		if(Roll_x <-60)
																		{
																			roll_x_status=-1;
																			enabelcheck();
																		} else {
																			if(a>0.4)
																			{
																				a_status = 1;
																				enabelcheck();
																			}
//																		} else {
//																				// ax
//																				if(ax>0.7)
//																				{
//																					ax_status = 1;
//																					enabelcheck();
//																				}else 
//																				{
//																						if(ax<-0.7)
//																						{
//																							ax_status = -1;
//																							enabelcheck();
//																						} else {
//																								// ay
//																								if(ay>0.7)
//																								{
//																									ay_status = 1;
//																									enabelcheck();
//																								} else {
//																										if(ay<-0.7)
//																										{
//																											ay_status = -1;
//																											enabelcheck();
//																										} else {
//																												// az
//																												if(az>0.7)
//																												{
//																													az_status = 1;
//																													enabelcheck();
//																												} else {
//																														if(az<-0.7)
//																														{
//																															az_status = -1;
//																															enabelcheck();
//																														} else {
//																																if(a>0.4)
//																																	{
//																																		a_status = 1;
//																																		enabelcheck();
//																																	}
//																														}
//																												}	
//																										}			
//																								}
//																							}
//																					}
																		}
																}
														}
											}	
											
									}
								}
								if(Getdata_offset == true)
								{
									data_offset.a = A;
									data_offset.ax = MPU6050data.NT_MPU6050_Ax;
									data_offset.ay = MPU6050data.NT_MPU6050_Ay;
									data_offset.az = MPU6050data.NT_MPU6050_Az;
									data_offset.gx = MPU6050data.NT_MPU6050_Gx;
									data_offset.gy = MPU6050data.NT_MPU6050_Gy;
									data_offset.gz = MPU6050data.NT_MPU6050_Gz;
									data_offset.roll_x = Roll;
									data_offset.pitch_y = Pitch;
									data_offset.yaw_z = Yaw;
									Getdata_offset = false;
								}
								if(Debug_app == true)
								{
										k++;
										if(k==10)
											{k=0;
											//sprintf((char*)buff_char,"%0.2f %0.2f %0.2f %0.1f %0.1f %0.1f %0.3f\r\n",MPU6050data.NT_MPU6050_Ax*30,MPU6050data.NT_MPU6050_Ay*30,MPU6050data.NT_MPU6050_Az*30,Roll,Pitch,Yaw,A*30);
											dataz = strlen((char*)buff_char) ;
											//HAL_UART_Transmit(&huart1,buff_char,dataz,100);
											}
								}
					}
		}	
}

void check_status (void)
	{
		if(a_status ==1)
			{
				//if(ax_status ==1 )
				if( pitch_y_status ==1)
					{
//						if(ay_status==1)
//						{
//							Loai_nga=NGA_NGUA;
//							disabel_check();
//						} else if(ay_status == -1)
//						{
//							Loai_nga=NGA_SAP;
//							disabel_check();
//						} else { // ay ==0
//								if(az_status == -1)  
//								{
//									Loai_nga=NGA_PHAI;
//									disabel_check();
//								}else if(az_status == 1)   // ay ==0
//								{
//									Loai_nga=NGA_TRAI;
//									disabel_check();
//								} else {
//									Loai_nga=NGA;
//									disabel_check();
//								}
//						}
						if(roll_x_status ==1)
						{
							Loai_nga = NGA_NGUA;
						}
						if(roll_x_status ==-1)
						{
							Loai_nga = NGA_SAP;
						}
						if(roll_x_status ==0)
						{
							if(az_status == -1)  
								{
									Loai_nga=NGA_PHAI;
									disabel_check();
								}else if(az_status == 1)   // ay ==0
								{
									Loai_nga=NGA_TRAI;
									disabel_check();
								} else {
									Loai_nga=NGA;
									disabel_check();
								}
						}
					}
			}
	}



void enabelcheck(void)
	{
		intimecheck = true;
		timer_enabel(&htim2);
	}
void disabel_check(void)
	{
		intimecheck = false;
		time_disabel(&htim2);
	}

void timer_enabel(TIM_HandleTypeDef *htim)
	{
		assert_param(IS_TIM_INSTANCE(htim->Instance));
		__HAL_TIM_ENABLE(htim);
		__HAL_TIM_ENABLE_IT(htim, TIM_IT_UPDATE);
	}
void time_disabel(TIM_HandleTypeDef *htim)
	{
		assert_param(IS_TIM_INSTANCE(htim->Instance));
		__HAL_TIM_DISABLE_IT(htim, TIM_IT_UPDATE);
		__HAL_TIM_DISABLE(htim);
	}

void reset_status(void)
{
	a_status = 0;
	ax_status = 0;
	ay_status = 0;
	az_status = 0;
	gx_status = 0;
	gy_status = 0;
	gz_status = 0;
	roll_x_status = 0;
	pitch_y_status = 0;
	yaw_z_status = 0;
	Loai_nga= KHONG_NGA;
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_SET);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}
/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 36000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}



/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
	{
		if(GPIO_Pin==GPIO_PIN_2)
			{
				read = 1;
				dmpDataReady();
			}
	}
	
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
	{
		if(htim->Instance == htim2.Instance)
			{
//				uint8_t buff_char[30];
//				sprintf((char*)buff_char,"Yaw: \t Pitch: \t Roll: \r\n");
//				HAL_UART_Transmit(&huart1,buff_char,strlen((char*)buff_char),100);
				time_count++;
				if(time_count >=4)
					{
						time_count =0;
						check_status();
						if(Loai_nga == NGA_NGUA)
						{
							sprintf((char*)buff_char,"@0000000001 NGA_NGUA^\r\n");
						}
						if(Loai_nga == NGA_SAP)
						{
							sprintf((char*)buff_char,"@0000000001 NGA_SAP^\r\n");
						}
						if(Loai_nga == NGA_PHAI)
						{
							sprintf((char*)buff_char,"@0000000001 NGA_PHAI^\r\n");
						}
						if(Loai_nga == NGA_TRAI)
						{
							sprintf((char*)buff_char,"@0000000001 NGA_TRAI^\r\n");
						}
						//sprintf((char*)buff_char,"a: %d\t ax: %d\t ay: %d\t az: %d\r\n",a_status,ax_status,ay_status,az_status);
						HAL_UART_Transmit(&huart1,buff_char,strlen((char*)buff_char),100);
						disabel_check();
						reset_status();
					}
				
			}
	}
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
