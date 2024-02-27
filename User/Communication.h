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
#define BUF_MAX_SIZE 128  	//�����������ֽ��� 200

extern uint8_t usart3_rxbuf[UART3_RXBUFFER_MAX_SIZE];		//ESP���ݽ��ջ�����
extern uint8_t WIFI_CONNECT_FLAG;    //WIFI���ӱ�־λ
extern uint8_t flag_uart3_recv;
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

typedef struct{
	int len;
	char buf[BUF_MAX_SIZE];
}FIFO_RES,*P_FIFO;

#define FIFO_MAX 8
typedef struct {
	uint16_t front;	 //��ͷָ��
	uint16_t rear;	//��βָ�룬����β��Ϊ�գ���ָ���βԪ�ص���һ��λ��
	FIFO_RES fifo[FIFO_MAX];	//�·��������
}Queue;

extern Queue l_sT_Response;	//�����������
extern Queue l_sT_Response;	//�����������
typedef enum{
	FIFO_EMPTY=-99,
	FIFO_FULL,
	FIFO_FUOK=0,	
}FIFO_STATUS;
int enQueue(Queue *queue,char *buf,int len);
int deQueue(Queue *queue,char *buf,int* p_len);
int length(Queue *Q);
#endif

