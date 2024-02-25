#include "UART.h"


#if 1
#pragma import(__use_no_semihosting)             
                
struct __FILE 				//标准库需要的支持函数 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
/* FILE is typedef’ d in stdio.h. */ 
FILE __stdout;       
					  
void _sys_exit(int x) 							//定义_sys_exit()以避免使用半主机模式  
{ 
	x = x; 
} 

int fputc( int ch , FILE *f )				//重定义fputc函数 
{      
	USART_SendData( USART1 , ch );
	while (USART_GetFlagStatus( USART1 , USART_FLAG_TC) == RESET );	
	return ch;
}
#endif


/*
	USART 设置程序
*/
void UART_Init( USART_TypeDef *USARTx , u32 Baud )				//初始化UARTx
{
	USART_InitTypeDef  USART_InitStructure;
	USART_DeInit( USARTx );

	RCC_APB2PeriphClockCmd( RCC_APB2Periph_USART1 , ENABLE ); 
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_USART2 , ENABLE ); 
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_USART3 , ENABLE ); 
	
	USART_InitStructure.USART_BaudRate = Baud;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;			//8位数据位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;				//停止位1
	USART_InitStructure.USART_Parity = USART_Parity_No;					//无奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;   //无流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;		//允许发送接收
	USART_Init( USARTx , &USART_InitStructure);
	
	USART_Cmd( USARTx , ENABLE );
}

void  USART_WriteChar( USART_TypeDef *USARTx , u8 WC )
{
	USART_SendData( USARTx , WC);
	while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);
}

void  USART_WriteNum( USART_TypeDef *USARTx , u8 WC )
{
	USART_SendData( USART1 , WC+0x30 );
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
}
