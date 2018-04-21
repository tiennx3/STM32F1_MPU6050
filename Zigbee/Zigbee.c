#include "Zigbee.h"
#include "jsmn.h"
#include "json_handl.h"
uint8_t data_receive[512];
uint8_t data_handl[512];
uint8_t data_en = 0;
uint8_t cc2530_handl=0;
uint16_t lenDataReceive=0;
uint8_t received=0;
uint8_t Addr[10];
char thongbao[50];
CC2530_t CC2530;
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

uint8_t Resul;
// ham khoi tao
void CC2530_Init(CC2530_t * CC2530N)
{
	
	CC2530_t * CC2530_N = CC2530N;
	Resul=0;
	MX_USART1_UART_Init();
	DMA_CC2530_Init();
	HAL_UART_Receive_DMA(&huart1,&received,1);
	
//	HAL_Delay(30000) ; // delay 3s (init) qua khoang thoi gian zigbee khoi dong 
//	do{
//			Resul = CC2530_getAddr(CC2530_N);
//		} while(!Resul);
//	HAL_UART_Receive_DMA(&huart1,&received,1);
//	do{
//			Resul = CC2530_getFAddr(CC2530_N);
//		} while(!Resul);
	sprintf(CC2530N->ADDR,"805C");
	sprintf(CC2530N->FADDR,"0000");
	CC2530_N->CC2530_CMD = CC2530_CONNECT; // nhan du lieu 
}	
 
// ham xu ly khi co chuoi du lieu dieu khien tu tren xuong (can goi nhanh nhat)


void CC2530_update_hander(uint8_t * DEN)  // ham xu ly , nen duoc cap nhat nhanh nnhat 
	{
		if(cc2530_handl==1)
			{
				Json_handl((char*)data_receive,DEN,thongbao);
				//CC2530_send((char*)data_handl);
				cc2530_handl=0;
				
			}
			return;
	}
// ham lay dia chi thiet bi
uint8_t CC2530_getAddr(CC2530_t * CC2530)
{
	CC2530_t* CC2530N = CC2530;
	CC2530_send((char*)FROM_AT_GETADDR);
	HAL_Delay(1000);
	strcpy((char*)data_handl,(char*)data_receive);
	memset(data_receive,0,512);
	lenDataReceive=0;
	char * locationstr = strstr((const char*)data_handl,"ADDR=");
	if(strstr((const char*)data_handl,"ADDR=")!=NULL)
	{
		CC2530N->ADDR[0]=*(locationstr+7);
		CC2530N->ADDR[1]=*(locationstr+8);
		CC2530N->ADDR[2]=*(locationstr+9);
		CC2530N->ADDR[3]=*(locationstr+10);
		CC2530_send(CC2530N->ADDR);
		return 1;
	}
	return 0;
}

// ham lay F diachi 
uint8_t CC2530_getFAddr(CC2530_t * CC2530)
{
	CC2530_t* CC2530N = CC2530;
	CC2530_send((char*)FROM_AT_GETFADDR);
	HAL_Delay(1000);
	strcpy((char*)data_handl,(char*)data_receive);
	memset(data_receive,0,512);
	lenDataReceive=0;
	char * locationstr = strstr((const char*)data_handl,"ADDR=");
	if(strstr((const char*)data_handl,"ADDR=")!=NULL)
	{
		CC2530N->FADDR[0]=*(locationstr+7);
		CC2530N->FADDR[1]=*(locationstr+8);
		CC2530N->FADDR[2]=*(locationstr+9);
		CC2530N->FADDR[3]=*(locationstr+10);
		CC2530_send(CC2530N->FADDR);
		return 1;
	}
	return 0;
}

// ham gui du lieu CC2530
void CC2530_send(char * data)
	{
		uint16_t lenData = (uint16_t)strlen(data);
		//char dataa[10];
		//sprintf(dataa,"_%d_" , lenData);
		//HAL_UART_Transmit(&huart1,(uint8_t *)dataa,10,10);
		for(uint8_t i =0 ; i<lenData/100;i++)
		{
			HAL_UART_Transmit(&huart1,(uint8_t *)(data+i*100),100,10);
		}
		HAL_UART_Transmit(&huart1,(uint8_t *)(data+(lenData/100)*100),lenData%100,10);
		//HAL_UART_Transmit(&huart1,(uint8_t *)data,lenData,10);
	}

// ham nhan du lieu CC2530
void CC2530_receive(uint8_t data)
{
	if(CC2530.CC2530_CMD == CC2530_init)
		{
			data_receive[lenDataReceive]=data;
			lenDataReceive++;
			
		} else if(CC2530.CC2530_CMD == CC2530_CONNECT)
		{
			if(data_en==0)
			{
				if(data=='@')
				{
					memset(data_receive,0,512);
					lenDataReceive=0;
					data_en=1;
				}
			} else if(data_en==1)
			{
				if(data=='^')
				{
					for(uint16_t i = 0 ; i<strlen((char*)data_receive); i++)
					{
						data_handl[i]=data_receive[i];
					}
					//strcpy((char*)data_handl,(char*)data_receive);
					//HAL_Delay(1);
					//memset(data_receive,0,512);
					data_en=0;
					cc2530_handl =1;
				} else {
					data_receive[lenDataReceive]=data;
					lenDataReceive++;
				}
			}
		}
}


// ham khoi tao UART 
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

// ham khoi tao DMA
static void DMA_CC2530_Init(void)  // chanel 4, 5 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
	/* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  

}

// ham ngat nhan du lieu uart 
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == huart1.Instance)
	{
		CC2530_receive(received);
		HAL_UART_Receive_DMA(&huart1,&received,1);
	}
}

