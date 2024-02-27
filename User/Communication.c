#include "Communication.h"
//https://blog.csdn.net/Chuangke_Andy/article/details/117923344
#define HAL_Delay Delay_Ms
uint8_t esp_rxbuf[UART3_RXBUFFER_MAX_SIZE];		//ESP���ݽ��ջ�����
// Queue l_sT_Response;	//�����������
// Queue l_sT_Response;	//�����������
// void UART_WriteCurve(  u16 *s , u32 Num )
// {
// 	u32 i;
// 	u8 j;
// 	s32 temps32;
// 	u8 temp8[3];
	
// 	UART_WriteChar(170);
// 	UART_WriteChar(90);
// 	UART_WriteChar(80);
	
// 	UART_WriteChar(50);		//ͨ��
	
// 	UART_WriteChar(100);
// 	UART_WriteChar(180);
// 	UART_WriteChar( (Num*3+1)/256 );
// 	UART_WriteChar( (Num*3+1)%256 );
	
// 	UART_WriteChar( 1 );           //����
	
// 	for( i=0; i<Num ; i++)
// 	{
// 		temps32 = s[i];
// 		temps32 -= 2047;
// 		temps32 *= 65536*2;
// 		temp8[0] = (temps32 & 0xff000000)>>24;
// 		temp8[1] = (temps32 & 0x00ff0000)>>16;
// 		temp8[2] = (temps32 & 0x0000ff00)>>8;
// 		for( j=0 ; j<3 ; j++ )
// 		{
// 			UART_WriteChar( temp8[j] );
// 		}
// 	}
	
// 	for( i=0; i<30 ; i++)
// 	{
// 		UART_WriteChar(0);
// 	}
// }

// void UART_WriteCurve_s16(  s16 *s , u32 Num1 )
// {
// 	u32 i;
// 	u32 Num = Num1*4;
// 	UART_WriteChar( 'A' );
// 	UART_WriteChar( 'D' );
// 	UART_WriteChar( 'C' );
// 	UART_WriteChar( 'P' );
// 	UART_WriteChar( '_' );
// 	UART_WriteChar( 'R' );
// 	UART_WriteChar( 'X' );
	
	
// 	UART_WriteChar( (Num&0xff000000)>>24);
// 	UART_WriteChar( (Num&0x00ff0000)>>16);
// 	UART_WriteChar( (Num&0x0000ff00)>>8);
// 	UART_WriteChar( (Num&0x000000ff) );
	
// 	for( i=0 ; i<Num1 ; i++ )
// 	{
// 		if( s[i]&0x8000 )
// 		{
// 			UART_WriteChar(0xff);
// 			UART_WriteChar(0xff);
// 		}
// 		else
// 		{
// 			UART_WriteChar(0);
// 			UART_WriteChar(0);
// 		}
// 		UART_WriteChar( (s[i]&0xff00)>>8 );
// 		UART_WriteChar( s[i]&0x00ff );
// 	}
	
// }



/************************esp8266.c**********************************************/
#define AT_CWSAP_STRING   "AT+CWSAP=\"ESP8266_DYW-300\",\"12345678\",5,3\r\n"
uint8_t WIFI_CONNECT_FLAG = 0;    //WIFI���ӱ�־λ
uint8_t esp_rxbuf[128];			  //ESP���ݽ��ջ�����
uint8_t t_buf[40];
uint8_t h_buf[40];
//������Ҫ�õ��������
volatile tATCmd  ATCmds[9]= {
  /*CmdSend,             		*CmdRcv,      TimeOut,   CmdStatus*/  
  {NULL,				  		NULL,		  0,		 NO_RCV},
  {"AT\r\n",             		"OK",         5000,  NO_RCV},   //���AT ��λָ��       
  {"AT+CWMODE=2\r\n",    		"OK" ,        2000,      NO_RCV},   //����WIFIģʽAP
  {AT_CWSAP_STRING,      		"OK" ,     	  2000,      NO_RCV},   //����MAC��ص�AP����  
  {"AT+CIPMUX=1\r\n",    		"OK" ,        2000,      NO_RCV},   //���ö�����
  {"AT+CIPSERVER=1,8000\r\n", 	"OK" ,   	  2000,      NO_RCV},   //��ʼ��TCP������,Ĭ��IP��192.168.4.1�����ö˿ںţ�8000�� 
  {"AT+CIFSR\r\n",   			"CIFSR",   	  2000,      NO_RCV},   //��ȡIP��MAC��ַ    
  {"AT+CIPSEND\r\n",   			">" ,         2000,      NO_RCV}    //TCP��������
};
/*******************************************************************************
* Function Name  	: Usart2_Data_Send
* Description    	: USART2���ݷ��ͺ���
* Input          	: str Ҫ���͵��ַ���; len �ַ�������
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
* Description    	: MCU��ESP��������
* Input          	: ATCmdNum ������
* Return         	: tCmdStatus ������Ӧ״̬ 
*******************************************************************************/
tCmdStatus ESP_Send_Cmd(tATCmdNum ATCmdNum){
	uint8_t len;
	//��ս��ջ����Լ�����״̬
	ATCmds[ATCmdNum].CmdStatus = NO_RCV;
	//��������
	len = strlen(ATCmds[ATCmdNum].CmdSend);	
	Usart2_Data_Send((uint8_t *)ATCmds[ATCmdNum].CmdSend, len);
	//�ȴ�һ��ʱ��		
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
* Description    	: �����ֵ������
* Input          	: ATCmdNum ������
* Return         	: none
*******************************************************************************/
void ESP_Cmd_Rcv(tATCmdNum ATCmdNum){	
	//���մ�������
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
* Description    	: ESP�������ݸ�WiFi�豸
* Input          	: SendBuf Ҫ���͵��ַ���; len �ַ�������
* Return         	: tCmdStatus ��Ӧ״̬
*******************************************************************************/
tCmdStatus ESP_Send_Data(uint8_t *SendBuf,uint8_t len){
	uint8_t buf[30] =  {0};
	tATCmdNum ATCmdNum;	
	if(!WIFI_CONNECT_FLAG){  //δ����״̬���ܷ�������
		printf("No device connected!\r\n");
		return NO_CONNECT;
	}				
	else{ //WIFIģʽ
		sprintf((char *)buf,"AT+CIPSEND=%d,%d\r\n",0,len);
		ATCmdNum = AT_CIPSEND;	
		Usart2_Data_Send(buf,strlen((char *)buf));   //��������
		// printf("Send AT_CIPSEND cmd ok!\r\n");	
	}
	//�ȴ�һ��ʱ��	
	HAL_Delay(ATCmds[ATCmdNum].TimeOut);
	ATCmds[ATCmdNum].CmdStatus = NO_RCV;        //�����״̬		 
	while(ATCmds[ATCmdNum].CmdStatus != RCV_SUCCESS){		 
		ESP_Cmd_Rcv(ATCmdNum);
	  	if(ATCmds[ATCmdNum].CmdStatus == RCV_TIMEOUT)
			return RCV_TIMEOUT;
	}
		
	Usart2_Data_Send(SendBuf,len);              //��������
	// printf("Send data ok!\r\n");	 
	return RCV_SUCCESS;
}
/*******************************************************************************
* Function Name  	: ESP_Data_Rcv
* Description    	: ����WiFi�豸���ظ�ESP������
* Input          	: none
* Return         	: none
*******************************************************************************/
void ESP_Data_Rcv(void){
	char esp_rxbuf_[BUF_MAX_SIZE];
	memcpy(esp_rxbuf_,esp_rxbuf,BUF_MAX_SIZE);
    if((!WIFI_CONNECT_FLAG) && (strstr((char *)(esp_rxbuf_),"CONNECT")!=NULL)){ 
		printf("WIFI������\n");	
		WIFI_CONNECT_FLAG = 1;   //WIFI�豸�����ӣ���λ���ӱ�־λ
	}
	if(strstr((char *)(esp_rxbuf_),"+IPD") != NULL  ){ //�յ��ͻ�������
		printf("WIFI�յ���λ������\n");	
		//��ȡ����������;
		EP32_RcvData_Extract(esp_rxbuf_);	
		return ;							
	}	
	if(strstr((char *)(esp_rxbuf_),"CLOSED") != NULL){ //�ͻ��˶Ͽ�����
		printf("WIFI�Ͽ�����\n");		 
		WIFI_CONNECT_FLAG = 0;    //������ӱ�־λ
	}		
	memset(&esp_rxbuf_,0,sizeof(esp_rxbuf_));
}
#define ServoSet "ServoSet"
/*******************************************************************************
* Function Name  	: EP32_RcvData_Extract
* Description    	: ��ȡWiFi�豸���͸�ESP�����ݲ�������
* Input          	: rev_buf ���յ�������
* Return         	: none
*******************************************************************************/
void EP32_RcvData_Extract(uint8_t *rev_buf){
	char *buf = "VALID CMD,please enter TEMPERATURE or HUMIDITY!!!" ;
	if(strstr((char *)(rev_buf),"TEMPERATURE")!=NULL)
		ESP_Send_Data(t_buf,strlen((char *)t_buf));
	else if(strstr((char *)(rev_buf),"HUMIDITY")!=NULL)
		ESP_Send_Data(h_buf,strlen((char *)h_buf));
	else  if(strstr((char *)(rev_buf),ServoSet)!=NULL) {
			int val,status,len;
			//\r\n+IPD,0,13:ServoSet(%d)
			int cnt = sscanf((char*) rev_buf,"\r\n+IPD,%d,%d:ServoSet(%d)",&status,&len, &val);
			if (cnt == 3) {
				if(val<0)
					val=0;
				if(val>360)
					val=360;
				val=val*5/9+50;
				printf("ServoSet [%d]\r\n", val);
				TIM_SetCompare1(TIM1,val);		//ȡֵ��ΧΪ50-250��Ӧ�������ת�ٺͷ������ת��
			} else {
				printf("Error CMD\r\n");
			}

		}
	else
		ESP_Send_Data((uint8_t *)buf,strlen(buf));
}
/*******************************************************************************
* Function Name  	: ESP_Init
* Description    	: ESPģ���ʼ��
* Input          	: none
* Return         	: none
*******************************************************************************/
void ESP_Init(void){
	//ESP_RST_Pin_SetH;
	//ESP_CH_PD_Pin_SetH;
	HAL_Delay(1000);	
	tATCmdNum i = AT_IDIE;
	//WIFIģʽ��ʼ��
	for(i = AT; i<=AT_CIPSERVER ; i++){						
		if( ESP_Send_Cmd(i) != RCV_SUCCESS){
			printf("PES32 Init failed\n");
			return ;
		}
	}
	printf("PES32 Init Success\n");
}



//////////////////////////
/************�жϴ�������Ĵ���********************************************/

uint8_t rx_len;			//����һ֡���ݵĳ���
uint8_t recv_end_flag;	//һ֡���ݽ�����ɱ�־
uint8_t usart3_rxbuf[UART3_RXBUFFER_MAX_SIZE];	//USART2_RMA���ջ�����
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


//PWM�����ʼ��
//arr���Զ���װֵ
//psc��ʱ��Ԥ��Ƶ��
void TIM1_PWM_Init(u32 arr,u32 psc)
{		 					 
	//�˲������ֶ��޸�IO������
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);  	//TIM1ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	//ʹ��PORTAʱ��	
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource8,GPIO_AF_TIM1); //GPIOA8����Ϊ��ʱ��1
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;           //GPIOA8
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
	GPIO_Init(GPIOA,&GPIO_InitStructure);              //��ʼ��PA8
	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=arr;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);//��ʼ����ʱ��1
		 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;	//���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ���
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM1 4OC1

	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  //ʹ��TIM1��CCR1�ϵ�Ԥװ�ؼĴ���
 
  TIM_ARRPreloadConfig(TIM1,ENABLE);//ARPEʹ�� 
	
	TIM_Cmd(TIM1, ENABLE);  //ʹ��TIM1
	TIM_CtrlPWMOutputs(TIM1, ENABLE);									  
}  



int enQueue(Queue *queue,char *buf,int len){
	if((queue->rear+1)%FIFO_MAX==queue->front){
		return FIFO_FULL;
	}
	if (len > BUF_MAX_SIZE)
		len = BUF_MAX_SIZE;
	memcpy(queue->fifo[queue->rear].buf,buf,len);
	queue->fifo[queue->rear].len=len;
	queue->rear=(queue->rear+1)%FIFO_MAX;
	return len;
}
int deQueue(Queue *queue,char *buf,int* p_len){
	if((queue->front )==queue->rear){
		return FIFO_EMPTY;
	}
	memcpy(buf,queue->fifo[queue->front].buf,queue->fifo[queue->front].len);
	* p_len=queue->fifo[queue->front].len;
	queue->front = (queue->front + 1) % FIFO_MAX;  //��ͷָ��+1
	return *p_len;
}
//���ض��г���
int length(Queue *Q) {
    return (Q->rear - Q->front + FIFO_MAX) % FIFO_MAX; 
}