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
#define  delay_s Delay_Ms
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
	delay_s(2);
	u2_printf("AT+RST\r\n");
	delay_s(10);
	u2_printf("AT+CWMODE=1\r\n");
	delay_s(5);
	u2_printf("AT+CWJAP=\"123123\",\"12345678\"\r\n");
	delay_s(15);
	u2_printf("AT+CIPMUX=0\r\n");
	delay_s(5);
	u2_printf("AT+CIPMODE=1\r\n");
	delay_s(5);
	u2_printf("AT+CIPSERVER=1,8800\r\n");
	delay_s(5);
	u2_printf("AT+CIPSEND\r\n");
	delay_s(5);
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
	Task_Update_Range();
	// static uint32_t counter=0;
	// counter++;
	// printf( "Current counter = %d \r\n" , counter );
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
	UART3_Config(115200);
	esp8266_Config();
	ResetLcd();
	ClrScreen();
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
 











