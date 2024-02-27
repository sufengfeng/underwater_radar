#include "Communication.h"
//https://blog.csdn.net/Chuangke_Andy/article/details/117923344
#define HAL_Delay Delay_Ms
uint8_t esp_rxbuf[UART3_RXBUFFER_MAX_SIZE];		//ESP数据接收缓冲区
void UART_WriteCurve(  u16 *s , u32 Num )
{
	u32 i;
	u8 j;
	s32 temps32;
	u8 temp8[3];
	
	UART_WriteChar(170);
	UART_WriteChar(90);
	UART_WriteChar(80);
	
	UART_WriteChar(50);		//通道
	
	UART_WriteChar(100);
	UART_WriteChar(180);
	UART_WriteChar( (Num*3+1)/256 );
	UART_WriteChar( (Num*3+1)%256 );
	
	UART_WriteChar( 1 );           //增益
	
	for( i=0; i<Num ; i++)
	{
		temps32 = s[i];
		temps32 -= 2047;
		temps32 *= 65536*2;
		temp8[0] = (temps32 & 0xff000000)>>24;
		temp8[1] = (temps32 & 0x00ff0000)>>16;
		temp8[2] = (temps32 & 0x0000ff00)>>8;
		for( j=0 ; j<3 ; j++ )
		{
			UART_WriteChar( temp8[j] );
		}
	}
	
	for( i=0; i<30 ; i++)
	{
		UART_WriteChar(0);
	}
}

void UART_WriteCurve_s16(  s16 *s , u32 Num1 )
{
	u32 i;
	u32 Num = Num1*4;
	UART_WriteChar( 'A' );
	UART_WriteChar( 'D' );
	UART_WriteChar( 'C' );
	UART_WriteChar( 'P' );
	UART_WriteChar( '_' );
	UART_WriteChar( 'R' );
	UART_WriteChar( 'X' );
	
	
	UART_WriteChar( (Num&0xff000000)>>24);
	UART_WriteChar( (Num&0x00ff0000)>>16);
	UART_WriteChar( (Num&0x0000ff00)>>8);
	UART_WriteChar( (Num&0x000000ff) );
	
	for( i=0 ; i<Num1 ; i++ )
	{
		if( s[i]&0x8000 )
		{
			UART_WriteChar(0xff);
			UART_WriteChar(0xff);
		}
		else
		{
			UART_WriteChar(0);
			UART_WriteChar(0);
		}
		UART_WriteChar( (s[i]&0xff00)>>8 );
		UART_WriteChar( s[i]&0x00ff );
	}
	
}



/************************esp8266.c**********************************************/
#define AT_CWSAP_STRING   "AT+CWSAP=\"ESP8266_DYW-300\",\"12345678\",5,3\r\n"
uint8_t WIFI_CONNECT_FLAG = 0;    //WIFI连接标志位
uint8_t esp_rxbuf[128];			  //ESP数据接收缓冲区
uint8_t t_buf[40];
uint8_t h_buf[40];
//定义需要用到的命令集合
volatile tATCmd  ATCmds[9]= {
  /*CmdSend,             		*CmdRcv,      TimeOut,   CmdStatus*/  
  {NULL,				  		NULL,		  0,		 NO_RCV},
  {"AT\r\n",             		"OK",         5000,  NO_RCV},   //检测AT 复位指令       
  {"AT+CWMODE=2\r\n",    		"OK" ,        2000,      NO_RCV},   //设置WIFI模式AP
  {AT_CWSAP_STRING,      		"OK" ,     	  2000,      NO_RCV},   //建立MAC相关的AP名称  
  {"AT+CIPMUX=1\r\n",    		"OK" ,        2000,      NO_RCV},   //设置多连接
  {"AT+CIPSERVER=1,8000\r\n", 	"OK" ,   	  2000,      NO_RCV},   //初始化TCP服务器,默认IP（192.168.4.1）设置端口号（8000） 
  {"AT+CIFSR\r\n",   			"CIFSR",   	  2000,      NO_RCV},   //获取IP及MAC地址    
  {"AT+CIPSEND\r\n",   			">" ,         2000,      NO_RCV}    //TCP发送数据
};
/*******************************************************************************
* Function Name  	: Usart2_Data_Send
* Description    	: USART2数据发送函数
* Input          	: str 要发送的字符串; len 字符串长度
* Return         	: none
*******************************************************************************/
void Usart2_Data_Send(uint8_t *str, uint16_t len){
	//HAL_UART_Transmit(&huart2, (uint8_t *)str, len,100);
	for(int j=0;j<len;j++)							//??????
	{
		// while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET); //????,??????   
		//USART_WriteChar(USART3,); 
		while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
		USART_SendData( USART3 , str[j]);
	} 
}
/*******************************************************************************
* Function Name  	: ESP_Send_Cmd
* Description    	: MCU向ESP发送命令
* Input          	: ATCmdNum 命令编号
* Return         	: tCmdStatus 命令响应状态 
*******************************************************************************/
tCmdStatus ESP_Send_Cmd(tATCmdNum ATCmdNum){
	uint8_t len;
	//清空接收缓存以及接收状态
	ATCmds[ATCmdNum].CmdStatus = NO_RCV;
	//发送命令
	len = strlen(ATCmds[ATCmdNum].CmdSend);	
	Usart2_Data_Send((uint8_t *)ATCmds[ATCmdNum].CmdSend, len);
	//等待一定时间		
	HAL_Delay(ATCmds[ATCmdNum].TimeOut);
	while(ATCmds[ATCmdNum].CmdStatus != RCV_SUCCESS){
		ESP_Cmd_Rcv(ATCmdNum);
		if(ATCmds[ATCmdNum].CmdStatus == RCV_TIMEOUT)
			return RCV_TIMEOUT;
	}
	return RCV_SUCCESS;	 
}
/*******************************************************************************
* Function Name  	: ESP_Cmd_Rcv
* Description    	: 命令返回值处理函数
* Input          	: ATCmdNum 命令编号
* Return         	: none
*******************************************************************************/
void ESP_Cmd_Rcv(tATCmdNum ATCmdNum){	
	//接收处理命令
	if(strstr((const char*)esp_rxbuf,ATCmds[ATCmdNum].CmdRcv) != NULL){
		ATCmds[ATCmdNum].CmdStatus = RCV_SUCCESS;						
	}			
	else{
		HAL_Delay(ATCmds[ATCmdNum].TimeOut);
		ATCmds[ATCmdNum].CmdStatus = RCV_TIMEOUT;
	}
	memset(&esp_rxbuf,0,sizeof(esp_rxbuf));
}	
/*******************************************************************************
* Function Name  	: ESP_Send_Data
* Description    	: ESP发送数据给WiFi设备
* Input          	: SendBuf 要发送的字符串; len 字符串长度
* Return         	: tCmdStatus 响应状态
*******************************************************************************/
tCmdStatus ESP_Send_Data(uint8_t *SendBuf,uint8_t len){
	uint8_t buf[30] =  {0};
	tATCmdNum ATCmdNum;	
	if(!WIFI_CONNECT_FLAG){  //未连接状态不能发送数据
		printf("No device connected!\r\n");
		return NO_CONNECT;
	}				
	else{ //WIFI模式
		sprintf((char *)buf,"AT+CIPSEND=%d,%d\r\n",0,len);
		ATCmdNum = AT_CIPSEND;	
		Usart2_Data_Send(buf,strlen((char *)buf));   //发送命令
		// printf("Send AT_CIPSEND cmd ok!\r\n");	
	}
	//等待一定时间	
	HAL_Delay(ATCmds[ATCmdNum].TimeOut);
	ATCmds[ATCmdNum].CmdStatus = NO_RCV;        //清接收状态		 
	while(ATCmds[ATCmdNum].CmdStatus != RCV_SUCCESS){		 
		ESP_Cmd_Rcv(ATCmdNum);
	  	if(ATCmds[ATCmdNum].CmdStatus == RCV_TIMEOUT)
			return RCV_TIMEOUT;
	}
		
	Usart2_Data_Send(SendBuf,len);              //发送数据
	// printf("Send data ok!\r\n");	 
	return RCV_SUCCESS;
}
/*******************************************************************************
* Function Name  	: ESP_Data_Rcv
* Description    	: 接收WiFi设备返回给ESP的数据
* Input          	: none
* Return         	: none
*******************************************************************************/
void ESP_Data_Rcv(void){
    if((!WIFI_CONNECT_FLAG) && (strstr((char *)(esp_rxbuf),"CONNECT")!=NULL)){ 
		printf("WIFI已连接\n");	
		WIFI_CONNECT_FLAG = 1;   //WIFI设备已连接，置位连接标志位
	}
	if(strstr((char *)(esp_rxbuf),"+IPD") != NULL  ){ //收到客户端数据
		printf("WIFI收到上位机数据\n");	
		//提取并处理数据;
		EP32_RcvData_Extract(esp_rxbuf);	
		return ;							
	}	
	if(strstr((char *)(esp_rxbuf),"CLOSED") != NULL){ //客户端断开连接
		printf("WIFI断开连接\n");		 
		WIFI_CONNECT_FLAG = 0;    //清除连接标志位
	}		
	memset(&esp_rxbuf,0,sizeof(esp_rxbuf));
}
#define ServoSet "ServoSet"
/*******************************************************************************
* Function Name  	: EP32_RcvData_Extract
* Description    	: 提取WiFi设备发送给ESP的数据并做处理
* Input          	: rev_buf 接收到的数据
* Return         	: none
*******************************************************************************/
void EP32_RcvData_Extract(uint8_t *rev_buf){
	char *buf = "VALID CMD,please enter TEMPERATURE or HUMIDITY!!!" ;
	if(strstr((char *)(rev_buf),"TEMPERATURE")!=NULL)
		ESP_Send_Data(t_buf,strlen((char *)t_buf));
	else if(strstr((char *)(rev_buf),"HUMIDITY")!=NULL)
		ESP_Send_Data(h_buf,strlen((char *)h_buf));
	else if (!strncmp((char*) rev_buf, ServoSet, strlen(ServoSet))) {
			int val;
			int cnt = sscanf((char*) buf, "ServoSet(%d)", &val);
			if (cnt == 1) {
				printf("ServoSet [%d]\r\n", val);
				TIM_SetCompare1(TIM1,val);		//取值范围为50-250对应正向最大转速和反向最大转速
			} else {
				printf("Error CMD\r\n");
			}

		}
	else
		ESP_Send_Data((uint8_t *)buf,strlen(buf));
}
/*******************************************************************************
* Function Name  	: ESP_Init
* Description    	: ESP模块初始化
* Input          	: none
* Return         	: none
*******************************************************************************/
void ESP_Init(void){
	//ESP_RST_Pin_SetH;
	//ESP_CH_PD_Pin_SetH;
	HAL_Delay(1000);	
	tATCmdNum i = AT_IDIE;
	//WIFI模式初始化
	for(i = AT; i<=AT_CIPSERVER ; i++){						
		if( ESP_Send_Cmd(i) != RCV_SUCCESS){
			printf("PES32 Init failed\n");
			return ;
		}
	}
	printf("PES32 Init Success\n");
}




//////////////////////////
/************中断处理函数里的处理********************************************/

uint8_t rx_len;			//接收一帧数据的长度
uint8_t recv_end_flag;	//一帧数据接收完成标志
uint8_t usart3_rxbuf[UART3_RXBUFFER_MAX_SIZE];	//USART2_RMA接收缓冲区
uint8_t flag_uart3_recv=0;
void USART3_IRQHandler(void)
{
	if(USART_GetITStatus(USART3,USART_IT_IDLE) != RESET)
	{
		//null read hardware, no delete
		u8 ret = USART3->SR;  
		ret = USART3->DR; 
		DMA_Cmd(DMA1_Stream1,DISABLE); 
		rx_len =  UART3_RXBUFFER_MAX_SIZE - DMA_GetCurrDataCounter(DMA1_Stream1);		
		memcpy(esp_rxbuf,usart3_rxbuf,rx_len);
		memset(&usart3_rxbuf,0,sizeof(usart3_rxbuf));
		flag_uart3_recv=1;
		// Process_Uart3_data(esp_rxbuf, rx_len);
		//drv_UART3_RecvTest(UART3_RxBuffer,UART3_RxCounter);
		DMA_SetCurrDataCounter(DMA1_Stream1,UART3_RXBUFFER_MAX_SIZE);
		DMA_Cmd(DMA1_Stream1,ENABLE);
	}
}

//舵机控制
//https://blog.csdn.net/qq_42866708/article/details/113355329


//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
void TIM1_PWM_Init(u32 arr,u32 psc)
{		 					 
	//此部分需手动修改IO口设置
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);  	//TIM1时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	//使能PORTA时钟	
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource8,GPIO_AF_TIM1); //GPIOA8复用为定时器1
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;           //GPIOA8
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure);              //初始化PA8
	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);//初始化定时器1
		 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;	//设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 4OC1

	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  //使能TIM1在CCR1上的预装载寄存器
 
  TIM_ARRPreloadConfig(TIM1,ENABLE);//ARPE使能 
	
	TIM_Cmd(TIM1, ENABLE);  //使能TIM1
	TIM_CtrlPWMOutputs(TIM1, ENABLE);									  
}  
