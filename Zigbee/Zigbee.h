#ifndef __ZIGBEE_H
#define __ZIGBEE_H
#include "stm32f1xx_hal.h" // sua lai thu vien voi tung chip
#include "string.h"
 
#define FROMMEM(x)                          	((const char *)(x))
#define FROM_AT_GETCFG												FROMMEM("AT+GETCFG")   				// get all info    	"115200 0PANID=0xD34C ADDR=0x0000 ADDR=0x0000 CHANNEL=11/2405MHz "
#define FROM_AT_GETUART												FROMMEM("AT+GETUART")  				// get serial info 	"115200 0"
#define FROM_AT_GETCHN												FROMMEM("AT+GETCHN")  				// get signal CH    "CHANNEL=11/2405MHz "
#define FROM_AT_GETPANID											FROMMEM("AT+GETPANID")  			// get self PANID   "PANID=0xD34C"
#define FROM_AT_GETADDR												FROMMEM("AT+GETADDR")  				// get self short addr   	"ADDR=0x0000"
#define FROM_AT_GETFADDR											FROMMEM("AT+GETFADDR")  			// get parent short addr	"ADDR=0x0000"
#define FROM_AT_GETIEEE												FROMMEM("AT+GETIEEE")  				// get self IEEE addr			"IEEE=7E 39 87 15 00 4B 12 00"
#define FROM_AT_GETFIEEE											FROMMEM("AT+GETFIEEE")  			// get parent IEEE addr		"IEEE=00 00 00 00 00 00 00 00"
#define FROM_AT_RESTART												FROMMEM("AT+RESTART")  				// 	Reset	
#define FROM_AT_RESET													FROMMEM("AT+RESET")  					// default setting 
typedef enum _CC2530_CMD_t {
	CC2530_init = 0x00 ,
	CC2530_CONNECT
} CC2530_CMD_t;


typedef struct _CC2530_t {
	CC2530_CMD_t CC2530_CMD;
	char ADDR[10];
	char FADDR[10];
} CC2530_t;


extern uint8_t data_receive[512];
extern uint8_t data_handl[512];
extern uint8_t data_en;
extern uint8_t cc2530_handl;
extern uint16_t lenDataReceive;
extern uint8_t received;
extern uint8_t Addr[10];

extern CC2530_t CC2530;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;




static void MX_USART1_UART_Init(void);
static void DMA_CC2530_Init(void) ;
void CC2530_send(char * data);
void CC2530_receive(uint8_t data);
void CC2530_Init(CC2530_t * CC2530N);
uint8_t CC2530_getAddr(CC2530_t * CC2530);
uint8_t CC2530_getFAddr(CC2530_t * CC2530);
void CC2530_update_hander(uint8_t * DEN);

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __ZIGBEE_H */


