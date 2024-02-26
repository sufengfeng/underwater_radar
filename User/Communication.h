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
#define UART3_RXBUFFER_MAX_SIZE 128  	//�����������ֽ��� 200
extern uint8_t esp_rxbuf[UART3_RXBUFFER_MAX_SIZE];		//ESP���ݽ��ջ�����
extern uint8_t WIFI_CONNECT_FLAG;    //WIFI���ӱ�־λ
// AT�����
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
// ����ؽ����״̬
typedef enum{	
	NO_RCV  = 0,
	RCV_SUCCESS,
	RCV_TIMEOUT,
	NO_CONNECT,
}tCmdStatus;
// ����ṹ��
typedef struct{			  
	char *CmdSend;         //���͵�����
	char *CmdRcv; 	       //��ȷ���ذ������ַ���
	uint16_t TimeOut;      //��ʱ��ʱ��
	tCmdStatus CmdStatus;  //����ص�״̬
}tATCmd; 

void EP32_RcvData_Extract(uint8_t *rev_buf);
void ESP_Cmd_Rcv(tATCmdNum ATCmdNum);
#endif

