#ifndef _COMMUNICATION_H
#define _COMMUNICATION_H
#include "Sys.h"
#include "stm32f4xx.h"
#include "Communication.h"
#include "Periph.h"

#define UART_WriteChar( x )  USART_WriteChar( USART1 , x )

void UART_WriteCurve(  u16 *s , u32 Num );
void UART_WriteCurve_s16(  s16 *s , u32 Num1 );

/************************esp8266.h************************/
#define UART3_RXBUFFER_MAX_SIZE 128  	//定义最大接收字节数 200
extern uint8_t esp_rxbuf[UART3_RXBUFFER_MAX_SIZE];		//ESP数据接收缓冲区
extern uint8_t WIFI_CONNECT_FLAG;    //WIFI连接标志位
// AT命令号
typedef enum{	
	AT_IDIE  = 0,
	AT,
	AT_CWMODE,
	AT_CWSAP,
	AT_CIPMUX,
	AT_CIPSERVER,
	AT_CIFSR,
	AT_CIPSEND
}tATCmdNum;
// 命令返回结果的状态
typedef enum{	
	NO_RCV  = 0,
	RCV_SUCCESS,
	RCV_TIMEOUT,
	NO_CONNECT,
}tCmdStatus;
// 命令结构体
typedef struct{			  
	char *CmdSend;         //发送的命令
	char *CmdRcv; 	       //正确返回包含的字符串
	uint16_t TimeOut;      //超时的时间
	tCmdStatus CmdStatus;  //命令返回的状态
}tATCmd; 

void EP32_RcvData_Extract(uint8_t *rev_buf);
void ESP_Cmd_Rcv(tATCmdNum ATCmdNum);
#endif

