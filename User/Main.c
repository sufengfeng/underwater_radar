/******************** STM32F407 ***********************************************
 
* Project            : Ultrasonic Range Finder 
* HardWare           : DYSTM32F4_V1.2    MCU=STM32F407VGT6
* File Name          : main.c
* Author             : MC_benten
* Version            : V1.0
* Date               : 2016/09/16
* Description        : 
					   main file
					   Ultrasonic Range Finder 超声水下 测距
			MODULE：	
					Internal Periph：
						
					
					External IC：
					
					
* Modify			 :
        
********************************************************************************/


#include "stm32f4xx.h"
#include "stdio.h"
#include "PortConfig.h"
#include "Sys.h"
#include "Periph.h"
#include "arm_math.h"
#include "Communication.h"
#include "FIR.h"
#include "Correlation.h"
#include "stm32f4xx_dma.h"
typedef void (*Func)(void);

typedef struct _SoftTimer{
	uint32_t m_nCounter;		//计数器
	uint32_t m_nMaxCounter;		//定时器
	Func funcCallBack;			//回调函数
}SoftTimer;

void UpdataSoftTimer(void );
void TaskSchedule(void);
void Task_Update_Range(void );
void Delay_ms (uint32_t ms);
void Delay_us(uint32_t us);

u8 DMA_Flag = 0;
u16 K_Value  = 400;
s16 pK;
float Range;

//采样频率200K  周期 5us
//us
#define  _WIN_SUM_TIM     		8
//加和窗点数
#define  _WIN_SUM_POINT   		((_WIN_SUM_TIM*SAM_FRE)/1000000)
#define  _CQ_SPAN_POINT   		1
//#define  _CQ_SIZE         		((ADC1_DMA_Size/_CQ_SPAN_POINT))
#define  _CQ_SIZE         		((ADC1_DMA_Size/_CQ_SPAN_POINT))


//窗口每点 距离  
//声速设置
//#define  _US_SPEED              340.0//空气中
#define  _US_SPEED              1450.0//水下


#define  _CQ_RANGE_PERPOINT   	(3.625)          //抽取以后的每点距离mm  

  
//换能器频率
#define  _ULTRO_TRANS_FRE     	(200000)
//换能器周期us
#define  _ULTRO_TRANS_PER	  	((1000000)/_ULTRO_TRANS_FRE)

u16  adc_sum_table[ _CQ_SIZE ];

s16 Max_N;  //最大噪声
u16 T_End;
u32 Times = 0;
float range;
u32 max_point;
u32 max_point_loca;

u8  Test_RangeThshort( uint32_t dead_point );
u8 Test_RangeTh( uint16_t waves , uint32_t dead_point );
float real_speed = 340;
u32 test_deadpoint = 100;   
#define USART3_MAX_SEND_LEN (128)
char USART3_TX_BUF[USART3_MAX_SEND_LEN];
// #define  delay_s Delay_Ms
#include <stdarg.h>
 /* 串口3,printf 函数
  * 确保一次发送数据不超过USART3_MAX_SEND_LEN字节
  * */
void u2_printf(char* fmt,...)  
{  
	u16 i,j; 
	va_list ap; 
	va_start(ap,fmt);
	vsprintf((char*)USART3_TX_BUF,fmt,ap);
	va_end(ap);
	i=strlen((const char*)USART3_TX_BUF);		//此次发送数据的长度
	for(j=0;j<i;j++)							//循环发送数据
	{
		// while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET); //循环发送,直到发送完毕   
		USART_WriteChar(USART3,USART3_TX_BUF[j]); 
	} 
}
void esp8266_Config(void){
 /*
	* 1.AT+CWMODE=1 :设置模组为STA模式。（串口助手）
	* 2.AT+CWLAP :查询附近WIFI（串口助手）
	* 3.AT+CWJAP="123123","12345678" :连接?WIFI（串口助手）
	* 4.AT+CIFSR :查看路由器分配给模组的IP地址,例如192.168.43.104（串口助手）
	* 5.AT+CIPMUX=1 :打开多连接（串口助手）
	* 6.AT+CIPSERVER=1,8800 :设置模块服务器端口（串口助手）
	*/
	Delay_Ms(200);
	u2_printf("AT+RST\r\n");
	Delay_Ms(200);
	u2_printf("AT+CWMODE=2\r\n");	//设置为AP模式
	Delay_Ms(200);
	// u2_printf("AT+CWJAP=\"Xiaomi_604\",\"wei123456\"\r\n");
	// Delay_Ms(12000);
	//设置账户密码
	u2_printf("AT+CWSAP=\"MY_ESP\",\"12345678\",1,3,4,0\r\n");	//checkout IP address
	Delay_Ms(2000);

	u2_printf("AT+CIPMUX=1\r\n");
	Delay_Ms(2000);
	// u2_printf("AT+CIPMODE=1\r\n");
	// Delay_Ms(2000);
	u2_printf("AT+CIPSERVER=1,8800\r\n");
	Delay_Ms(2000);
	u2_printf("AT+CIPAP?\r\n");	//checkout IP address
	Delay_Ms(2000);

	// u2_printf("AT+CIPSEND\r\n");
	// Delay_Ms(2000);
}

//在sys.c里定义一个全局变量，定时器初始化时将其置为0，启动定时器
__IO uint64_t  us_tick=0;  //#define   __IO    volatile 
void SysTick_Init(void)
{
  us_tick = 0;
	/* SystemFrequency / 1000    1ms????
	 * SystemFrequency / 100000	 10us????
	 * SystemFrequency / 1000000 1us????
	 */
	if (SysTick_Config(SystemCoreClock / 1000000))	// ST3.5.0???
	{ 
		while (1);
	}
}

void SysTick_Handler(void)
{
	static int32_t m_nCounte=1000;
  	us_tick++;
	m_nCounte--;
	if(m_nCounte<=0){
		m_nCounte=1000;
		UpdataSoftTimer();
	}
}
uint64_t getCurrentMillis(void){
	return us_tick;
}
//参数为延时的ms数，×1000得到us数，也就是需要中断发生的次
//数，设置目前中断次数+需要的，us_tick会不断+1更新，
//当到了所设置的值后，退出while循环，实现ms延时
void Delay_ms (uint32_t ms)
{ 
	uint32_t target;
	
	target = us_tick + ms*1000;
	while(getCurrentMillis() <= target);
} 

void Delay_us(uint32_t us)
{ 
	uint32_t target;
	
	target = us_tick + us;
	while(getCurrentMillis() <= target);
}

//20ms回调事件
void Func_Task_1000ms01(void){	//忽略
	static uint32_t j=0;
	j++;
	Task_Update_Range();
	// esp8266_Config();
	// static uint32_t counter=0;
	// counter++;
	// printf( "Current counter = %d \r\n" , counter );
	u2_printf("AT+CIPSEND=0,4\r\n");
	Delay_us(4);
	u2_printf("%f\r\n",range);
	
}
//40ms回调事件
void Func_Task_100ms01(void){

}
//50ms回调事件
void Func_Task_10ms01(void){	//忽略
;
}

//50ms回调事件
void Func_Task_1ms01(void){	//忽略
;
}

//systick定时器1msz中断调度任务
void Func_Task_Interrupt(void){

}

/*****************************callback end***************************************************/
//多任务软定时器		异步任务
volatile SoftTimer g_sTimerArray[]={
	{0,1,Func_Task_1ms01},
	{5,9,Func_Task_10ms01},
	{37,99,Func_Task_100ms01},
	{915,999,Func_Task_1000ms01},
};

//Systik定时器中断更新定时器 1us调度一次
void UpdataSoftTimer(void ){
	Func_Task_Interrupt();		//中断回调服务函数
	for(uint32_t i=0;i<sizeof(g_sTimerArray)/sizeof(SoftTimer);i++){			//更新软定时器
		g_sTimerArray[i].m_nCounter++;
	}
}

//定时器任务调度
void TaskSchedule(void){
	for(uint32_t i=0;i<sizeof(g_sTimerArray)/sizeof(SoftTimer);i++){			//更新软定时器
			if(g_sTimerArray[i].m_nCounter>g_sTimerArray[i].m_nMaxCounter){
				g_sTimerArray[i].m_nCounter=0;
				(*(g_sTimerArray[i].funcCallBack))();
		}
	}
}
//1s扫描一次
void Task_Update_Range(void ){
		u32 i;
		u8 shortflag=0;
		
//在此处设置盲区 盲区位置
/*例如：
	Test_Range( 8 , 60 );
	Test_Range( 8 , 70 );
	Test_Range( 8 , 80 );
	
	参数1      发射脉冲个数（可以适当调整）
	参数2：    盲区距离  =  参数2 * 0.85 厘米  盲区时间 = 参数2 * _CQ_RANGE_PERPOINT（定义在上面）
    设置该参数 ：test_deadpoint    设置盲区  
     
    声速修正：使用已经定义好的变量 real_speed  单位m/s
    例如设置： real_speed = 320;    那么 声速 = 320m/s
*/
		//短脉冲测量
			shortflag=0;
		
		   test_deadpoint =236;       //设置盲区点数 1个点5us
       shortflag =Test_RangeThshort(test_deadpoint );         //阈值测试短距离
		   range=range;//距离修正cm单位
		
		//短脉冲测不到是用长脉冲
	  // shortflag=0;//测试长脉冲用
		if(shortflag==0)
		{
        test_deadpoint =256;       //设置盲区点数 
        Test_RangeTh( 8 , test_deadpoint );         //阈值测试方法长距离 
				range = range;//距离修正cm单位
		}
       
				//lcd显示距离
		ClrScreen();
		i = range;
		printFront(4,0,0);//距	
		printFront(4,16,1);//离
		printFront(4,32,2);//:
		printNumbers(4,48,i/10000%10);//0
		printNumbers(4,56,i/1000%10);//0
		printNumbers(4,64,i/100%10);//.
		printNumbers(4,72,i/10%10);//0
		printNumbers(4,80,i%10);//0
		printFront(4,96,3);//厘
		printFront(4,112,4);//米
		
		printf( "Current Range = %f \r\n" , range );//串口输出距离；232通讯；115200；字符显示（不勾选16进制）

        // Delay_Ms(1000);
	}


void Process_Uart3_data(u8* rx_buf, int len)
{
	if(len>1){
		for(int i=0;i<len;i++)							//??????
		{
			while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET); //????,??????   
			USART_WriteChar(USART1,rx_buf[i]); 
		} 
	}
}
#define UART3_RXBUFFER_MAX_SIZE (1024+20)  	//定义最大接收字节数 200
u8 UART3_RxBuffer[UART3_RXBUFFER_MAX_SIZE]={0};
/*
void DMA1_Stream3_IRQHandler(void){
     if(DMA_GetITStatus(USART_Rx_DMA_FLAG, DMA_IT_TCIF3) == SET){        
        DMA_ClearITPendingBit(USART_Rx_DMA_FLAG, DMA_IT_TCIF3);
				DMA_Cmd(USART_Rx_DMA_FLAG, DISABLE);
    }
}
*/
// #define UART1_TXBUFFER_MAX_SIZE       (1024+20)  	//定义最大接收字节数 200
// #define UART1_RXBUFFER_MAX_SIZE  			(1024+20)  	//定义最大接收字节数 200

static volatile u8 UART3_RxCounter = 0;
void USART3_IRQHandler(void)
{
	if(USART_GetITStatus(USART3,USART_IT_IDLE) != RESET)
	{
		//null read hardware, no delete
		u8 ret = USART3->SR;  
		ret = USART3->DR; 
		DMA_Cmd(DMA1_Stream1,DISABLE); 
		UART3_RxCounter =  UART3_RXBUFFER_MAX_SIZE - DMA_GetCurrDataCounter(DMA1_Stream1);		
		Process_Uart3_data(UART3_RxBuffer, UART3_RxCounter);
		
		//drv_UART3_RecvTest(UART3_RxBuffer,UART3_RxCounter);
		DMA_SetCurrDataCounter(DMA1_Stream1,UART3_RXBUFFER_MAX_SIZE);
		DMA_Cmd(DMA1_Stream1,ENABLE);
	}
}
void drv_UART3_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOD,ENABLE); //Enable GPIOB Clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE); //Enable UART3 Clock

#if (V50_POWER_DIR_CTRL)
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_USART3); //PD8  alternate to USART3-TX
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART3); //PD9  alternate to USART3-RX

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOD,&GPIO_InitStructure);
#else
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); //PB10  alternate to USART3-TX
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); //PB11  alternate to USART3-RX

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOB,&GPIO_InitStructure);
#endif
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART3, &USART_InitStructure); 
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);

	USART_Cmd(USART3, ENABLE);  

	//USART_ClearFlag(USART3, USART_FLAG_TC);
	USART_ClearITPendingBit(USART3, USART_IT_TC);
	//Usart3_RX  NVIC
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
	NVIC_Init(&NVIC_InitStructure);			
}


/*!<USART3_RX DMA at DMA1 channel 4  stream 5 and use AHB1 clock
* @Author: Chenxi
* @Date: 2015-02-14
*/
void drv_UART3_RxDMAInit()
{
	DMA_InitTypeDef DMA_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);
	DMA_DeInit(DMA1_Stream1);

	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&(USART3->DR));
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)(UART3_RxBuffer); 
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = UART3_RXBUFFER_MAX_SIZE;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream1,&DMA_InitStructure);

	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
	DMA_SetCurrDataCounter(DMA1_Stream1,UART3_RXBUFFER_MAX_SIZE);
	DMA_Cmd(DMA1_Stream1,ENABLE); 
}

int main( void )
{
	u32 i;
	u32 j;
	u32 tempu32;
	u32 up;
	u32 down;
	float tempf;
	float tempf1;
// STEP1 .配置系统时钟RCC  启动时已经配置过了 可以注释掉	
	SystemInit();
	SysTick_Init();

// STEP2 .外设配置
	
	/*
		配置时钟和GPIO口
		GPIO的时钟和 映射情况均通过该函数配置
	*/
	Port_Clk_Config();             	
	 /* LCD复位	*/
	
	TIM2_Config( _ULTRO_TRANS_FRE );            //发射驱动PWM产生 PA1口	参数为触发频率
	TIM3_Config( SAM_FRE );                     //触发ADC采样频率
	ADC_Config();                               //ADC
	UART1_Config( 9600 );
	printf( " build time = %s-%s \r\n" ,__DATE__, __TIME__ );

	ResetLcd();
	ClrScreen();
	
	drv_UART3_Init();
	drv_UART3_RxDMAInit();
	esp8266_Config();
	/*
	printFront(4,0,0);//距	
	printFront(4,16,1);//离
	printFront(4,32,2);//:
	printNumbers(4,48,0);//0
	printNumbers(4,56,0);//0
	printNumbers(4,64,10);//.
	printNumbers(4,72,0);//0
	printNumbers(4,80,0);//0
	printFront(4,96,3);//厘
	printFront(4,112,4);//米
	*/
	//while( DMA_Flag==0 );
	TRG = 0;
	for( i=0 ; i<_CQ_SIZE ; i++ )
	{
		adc_sum_table[i] = 0;
	}
	while(1){
		TaskSchedule();
		// Task_Update_Range();
		// Delay_Ms(1000);
	}
	
	
}
float range_min;
u32 avr;

u8 Test_RangeThshort(uint32_t dead_point )//参数盲区点数
{
  	u32 i;
	u32 j;
	u32 tempu32;
	u32 up;
	u32 down;
	//u16 lun_tim = waves * _ULTRO_TRANS_PER;            //40KHz//发射脉冲的时间=脉冲周期*脉冲数量
	
	range_min = (float)dead_point*_CQ_RANGE_PERPOINT;      //抽取以后的每点距离*死区点数=实际盲区（cm）
	range_min += 5;
	
	//TRG = 1;//PA1引脚配置为输出模式////////////////////////////////////////////////////////////////////
	TIM2_Start();//开定时器使PA1引脚输出PWM信号
	ADC_DMA_Trig( ADC1_DMA_Size );//开始AD采集，参数为采样点数
	Delay_Us(_ULTRO_TRANS_PER);//延时，时间为发射脉冲的时间
	TIM2_Stop();//关闭定时器1，结束PA1引脚输出PWM
	Delay_Ms(120);
	//TRG = 0;/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	for( i=0; i<(_CQ_SIZE-_WIN_SUM_POINT/_CQ_SPAN_POINT) ; i++ )//平滑滤波，滤波后adc_sum_table[]
	{
		tempu32 = 0;
		down = (i*_CQ_SPAN_POINT);
		up = down + _WIN_SUM_POINT;
		for( j= down ; j<up ; j++)
		{
			tempu32 += ADC1_ConvertedValue[j];
		}
		adc_sum_table[i] = tempu32/(_WIN_SUM_POINT);
	}
	
    avr=0;
    for( i=dead_point ; i<_CQ_SIZE-100 ; i++ )
	{
        avr += adc_sum_table[i];
    }
    avr /= _CQ_SIZE - 100 - dead_point;           //直流  平均电压   avr
    
	max_point = 0;
	//for( i=dead_point ; i<_CQ_SIZE-100 ; i++ )//查找最大值
		for( i=0 ; i<_CQ_SIZE-100 ; i++ )//查找最大值
	{
		tempu32 = adc_sum_table[i];
		tempu32 += adc_sum_table[i-2];
		tempu32 += adc_sum_table[i-1];
		tempu32 += adc_sum_table[i+2];
		tempu32 += adc_sum_table[i+1];
		if( max_point<tempu32)
		{
			max_point = tempu32;
			//max_point_loca = i;
		}
	}
    
    //avr = (max_point/5 - avr)/2+avr;                    //半阈值判断
	 avr = (max_point/5- avr)/2+avr;                    //半阈值判断   
	
    max_point_loca=0;
    for( i=dead_point ; i<_CQ_SIZE-100 ; i++ )
	{
        if( adc_sum_table[i] > avr )
            if( adc_sum_table[i+1] > avr )
                if( adc_sum_table[i+2] > avr )
                    if( adc_sum_table[i+3] > avr )
                    {
                        max_point_loca = i;
                        break;
                    }
    }
      
	range = (float)max_point_loca*_CQ_RANGE_PERPOINT;//当上述条件达不到时max_point_loca=0；range=0放回0用长脉冲测量
	if( range==0 )
	{
		return 0;
	}
	return 1;;
}



u8 Test_RangeTh( uint16_t waves , uint32_t dead_point )//参数1脉冲数量。参数2死区点数
{
	u32 i;
	u32 j;
	u32 tempu32;
	u32 up;
	u32 down;
	u16 lun_tim = waves * _ULTRO_TRANS_PER;            //发射脉冲的时间=脉冲周期*脉冲数量
	
	range_min = (float)dead_point*_CQ_RANGE_PERPOINT;      //抽取以后的每点距离*死区点数=实际盲区（cm）
	range_min += 5;
	
	TRG = 1;//PA1引脚配置为输出模式
	TIM2_Start();//开定时器使PA1引脚输出PWM信号
	ADC_DMA_Trig( ADC1_DMA_Size );//开始AD采集，参数为采样点数
	Delay_Us(lun_tim);//延时，时间为发射脉冲的时间
	TIM2_Stop();//关闭定时器1，结束PA1引脚输出PWM
	Delay_Ms(120);
	TRG = 0;
	for( i=0; i<(_CQ_SIZE-_WIN_SUM_POINT/_CQ_SPAN_POINT) ; i++ )//平滑滤波，滤波后adc_sum_table[]
	{
		tempu32 = 0;
		down = (i*_CQ_SPAN_POINT);
		up = down + _WIN_SUM_POINT;
		for( j= down ; j<up ; j++)
		{
			tempu32 += ADC1_ConvertedValue[j];
		}
		adc_sum_table[i] = tempu32/(_WIN_SUM_POINT);
	}
	
    avr=0;
    for( i=dead_point ; i<_CQ_SIZE-100 ; i++ )
	{
        avr += adc_sum_table[i];
    }
    avr /= _CQ_SIZE - 100 - dead_point;           //直流  平均电压   avr
    
	max_point = 0;
	//for( i=dead_point ; i<_CQ_SIZE-100 ; i++ )//查找最大值
	for( i=0 ; i<_CQ_SIZE-100 ; i++ )//查找最大值
	{
		tempu32 = adc_sum_table[i];
		tempu32 += adc_sum_table[i-2];
		tempu32 += adc_sum_table[i-1];
		tempu32 += adc_sum_table[i+2];
		tempu32 += adc_sum_table[i+1];
		if( max_point<tempu32)
		{
			max_point = tempu32;
			//max_point_loca = i;
		}
	}
    
    //avr = (max_point/5 - avr)/2+avr;                    //半阈值判断
	 avr = (max_point/5- avr)/4+avr;                    //半阈值判断   
	
    max_point_loca = 0;
    for( i=dead_point ; i<_CQ_SIZE-100 ; i++ )
	{
        if( adc_sum_table[i] > avr )
            if( adc_sum_table[i+1] > avr )
                if( adc_sum_table[i+2] > avr )
                    if( adc_sum_table[i+3] > avr )
                    {
                        max_point_loca = i;
                        break;
                    }
    }

	range = (float)max_point_loca*_CQ_RANGE_PERPOINT;
	if( range>range_min )
	{
		return 0;
	}
	return 1;
}

 
void DMA2_Stream0_IRQHandler( void )
{
	DMA_ClearITPendingBit( DMA2_Stream0 ,DMA_IT_TCIF0|DMA_IT_DMEIF0|DMA_IT_TEIF0|DMA_IT_HTIF0|DMA_IT_TCIF0 );
	DMA2->HIFCR = 0xffff;
	DMA2->LIFCR = 0xffff;
	DMA_Flag=1;
}
 











