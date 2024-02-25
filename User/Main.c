/******************** STM32F407 ***********************************************
 
* Project            : Ultrasonic Range Finder 
* HardWare           : DYSTM32F4_V1.2    MCU=STM32F407VGT6
* File Name          : main.c
* Author             : MC_benten
* Version            : V1.0
* Date               : 2016/09/16
* Description        : 
					   main file
					   Ultrasonic Range Finder ����ˮ�� ���
			MODULE��	
					Internal Periph��
						
					
					External IC��
					
					
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
	uint32_t m_nCounter;		//������
	uint32_t m_nMaxCounter;		//��ʱ��
	Func funcCallBack;			//�ص�����
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

//����Ƶ��200K  ���� 5us
//us
#define  _WIN_SUM_TIM     		8
//�Ӻʹ�����
#define  _WIN_SUM_POINT   		((_WIN_SUM_TIM*SAM_FRE)/1000000)
#define  _CQ_SPAN_POINT   		1
//#define  _CQ_SIZE         		((ADC1_DMA_Size/_CQ_SPAN_POINT))
#define  _CQ_SIZE         		((ADC1_DMA_Size/_CQ_SPAN_POINT))


//����ÿ�� ����  
//��������
//#define  _US_SPEED              340.0//������
#define  _US_SPEED              1450.0//ˮ��


#define  _CQ_RANGE_PERPOINT   	(3.625)          //��ȡ�Ժ��ÿ�����mm  

  
//������Ƶ��
#define  _ULTRO_TRANS_FRE     	(200000)
//����������us
#define  _ULTRO_TRANS_PER	  	((1000000)/_ULTRO_TRANS_FRE)

u16  adc_sum_table[ _CQ_SIZE ];

s16 Max_N;  //�������
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
 /* ����3,printf ����
  * ȷ��һ�η������ݲ�����USART3_MAX_SEND_LEN�ֽ�
  * */
void u2_printf(char* fmt,...)  
{  
	u16 i,j; 
	va_list ap; 
	va_start(ap,fmt);
	vsprintf((char*)USART3_TX_BUF,fmt,ap);
	va_end(ap);
	i=strlen((const char*)USART3_TX_BUF);		//�˴η������ݵĳ���
	for(j=0;j<i;j++)							//ѭ����������
	{
		// while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET); //ѭ������,ֱ���������   
		USART_WriteChar(USART3,USART3_TX_BUF[j]); 
	} 
}
void esp8266_Config(void){
 /*
	* 1.AT+CWMODE=1 :����ģ��ΪSTAģʽ�����������֣�
	* 2.AT+CWLAP :��ѯ����WIFI���������֣�
	* 3.AT+CWJAP="123123","12345678" :����?WIFI���������֣�
	* 4.AT+CIFSR :�鿴·���������ģ���IP��ַ,����192.168.43.104���������֣�
	* 5.AT+CIPMUX=1 :�򿪶����ӣ��������֣�
	* 6.AT+CIPSERVER=1,8800 :����ģ��������˿ڣ��������֣�
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

//��sys.c�ﶨ��һ��ȫ�ֱ�������ʱ����ʼ��ʱ������Ϊ0��������ʱ��
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
//����Ϊ��ʱ��ms������1000�õ�us����Ҳ������Ҫ�жϷ����Ĵ�
//��������Ŀǰ�жϴ���+��Ҫ�ģ�us_tick�᲻��+1���£�
//�����������õ�ֵ���˳�whileѭ����ʵ��ms��ʱ
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

//20ms�ص��¼�
void Func_Task_1000ms01(void){	//����
	Task_Update_Range();
	// static uint32_t counter=0;
	// counter++;
	// printf( "Current counter = %d \r\n" , counter );
}
//40ms�ص��¼�
void Func_Task_100ms01(void){

}
//50ms�ص��¼�
void Func_Task_10ms01(void){	//����
;
}

//50ms�ص��¼�
void Func_Task_1ms01(void){	//����
;
}

//systick��ʱ��1msz�жϵ�������
void Func_Task_Interrupt(void){

}

/*****************************callback end***************************************************/
//��������ʱ��		�첽����
volatile SoftTimer g_sTimerArray[]={
	{0,1,Func_Task_1ms01},
	{5,9,Func_Task_10ms01},
	{37,99,Func_Task_100ms01},
	{915,999,Func_Task_1000ms01},
};

//Systik��ʱ���жϸ��¶�ʱ�� 1us����һ��
void UpdataSoftTimer(void ){
	Func_Task_Interrupt();		//�жϻص�������
	for(uint32_t i=0;i<sizeof(g_sTimerArray)/sizeof(SoftTimer);i++){			//������ʱ��
		g_sTimerArray[i].m_nCounter++;
	}
}

//��ʱ���������
void TaskSchedule(void){
	for(uint32_t i=0;i<sizeof(g_sTimerArray)/sizeof(SoftTimer);i++){			//������ʱ��
			if(g_sTimerArray[i].m_nCounter>g_sTimerArray[i].m_nMaxCounter){
				g_sTimerArray[i].m_nCounter=0;
				(*(g_sTimerArray[i].funcCallBack))();
		}
	}
}
//1sɨ��һ��
void Task_Update_Range(void ){
		u32 i;
		u8 shortflag=0;
		
//�ڴ˴�����ä�� ä��λ��
/*���磺
	Test_Range( 8 , 60 );
	Test_Range( 8 , 70 );
	Test_Range( 8 , 80 );
	
	����1      ������������������ʵ�������
	����2��    ä������  =  ����2 * 0.85 ����  ä��ʱ�� = ����2 * _CQ_RANGE_PERPOINT�����������棩
    ���øò��� ��test_deadpoint    ����ä��  
     
    ����������ʹ���Ѿ�����õı��� real_speed  ��λm/s
    �������ã� real_speed = 320;    ��ô ���� = 320m/s
*/
		//���������
			shortflag=0;
		
		   test_deadpoint =236;       //����ä������ 1����5us
       shortflag =Test_RangeThshort(test_deadpoint );         //��ֵ���Զ̾���
		   range=range;//��������cm��λ
		
		//������ⲻ�����ó�����
	  // shortflag=0;//���Գ�������
		if(shortflag==0)
		{
        test_deadpoint =256;       //����ä������ 
        Test_RangeTh( 8 , test_deadpoint );         //��ֵ���Է��������� 
				range = range;//��������cm��λ
		}
       
				//lcd��ʾ����
		ClrScreen();
		i = range;
		printFront(4,0,0);//��	
		printFront(4,16,1);//��
		printFront(4,32,2);//:
		printNumbers(4,48,i/10000%10);//0
		printNumbers(4,56,i/1000%10);//0
		printNumbers(4,64,i/100%10);//.
		printNumbers(4,72,i/10%10);//0
		printNumbers(4,80,i%10);//0
		printFront(4,96,3);//��
		printFront(4,112,4);//��
		
		printf( "Current Range = %f \r\n" , range );//����������룻232ͨѶ��115200���ַ���ʾ������ѡ16���ƣ�

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
// STEP1 .����ϵͳʱ��RCC  ����ʱ�Ѿ����ù��� ����ע�͵�	
	SystemInit();
	SysTick_Init();

// STEP2 .��������
	
	/*
		����ʱ�Ӻ�GPIO��
		GPIO��ʱ�Ӻ� ӳ�������ͨ���ú�������
	*/
	Port_Clk_Config();             	
	 /* LCD��λ	*/
	
	TIM2_Config( _ULTRO_TRANS_FRE );            //��������PWM���� PA1��	����Ϊ����Ƶ��
	TIM3_Config( SAM_FRE );                     //����ADC����Ƶ��
	ADC_Config();                               //ADC
	UART1_Config( 9600 );
	UART3_Config(115200);
	esp8266_Config();
	ResetLcd();
	ClrScreen();
	/*
	printFront(4,0,0);//��	
	printFront(4,16,1);//��
	printFront(4,32,2);//:
	printNumbers(4,48,0);//0
	printNumbers(4,56,0);//0
	printNumbers(4,64,10);//.
	printNumbers(4,72,0);//0
	printNumbers(4,80,0);//0
	printFront(4,96,3);//��
	printFront(4,112,4);//��
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

u8 Test_RangeThshort(uint32_t dead_point )//����ä������
{
  	u32 i;
	u32 j;
	u32 tempu32;
	u32 up;
	u32 down;
	//u16 lun_tim = waves * _ULTRO_TRANS_PER;            //40KHz//���������ʱ��=��������*��������
	
	range_min = (float)dead_point*_CQ_RANGE_PERPOINT;      //��ȡ�Ժ��ÿ�����*��������=ʵ��ä����cm��
	range_min += 5;
	
	//TRG = 1;//PA1��������Ϊ���ģʽ////////////////////////////////////////////////////////////////////
	TIM2_Start();//����ʱ��ʹPA1�������PWM�ź�
	ADC_DMA_Trig( ADC1_DMA_Size );//��ʼAD�ɼ�������Ϊ��������
	Delay_Us(_ULTRO_TRANS_PER);//��ʱ��ʱ��Ϊ���������ʱ��
	TIM2_Stop();//�رն�ʱ��1������PA1�������PWM
	Delay_Ms(120);
	//TRG = 0;/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	for( i=0; i<(_CQ_SIZE-_WIN_SUM_POINT/_CQ_SPAN_POINT) ; i++ )//ƽ���˲����˲���adc_sum_table[]
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
    avr /= _CQ_SIZE - 100 - dead_point;           //ֱ��  ƽ����ѹ   avr
    
	max_point = 0;
	//for( i=dead_point ; i<_CQ_SIZE-100 ; i++ )//�������ֵ
		for( i=0 ; i<_CQ_SIZE-100 ; i++ )//�������ֵ
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
    
    //avr = (max_point/5 - avr)/2+avr;                    //����ֵ�ж�
	 avr = (max_point/5- avr)/2+avr;                    //����ֵ�ж�   
	
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
      
	range = (float)max_point_loca*_CQ_RANGE_PERPOINT;//�����������ﲻ��ʱmax_point_loca=0��range=0�Ż�0�ó��������
	if( range==0 )
	{
		return 0;
	}
	return 1;;
}



u8 Test_RangeTh( uint16_t waves , uint32_t dead_point )//����1��������������2��������
{
	u32 i;
	u32 j;
	u32 tempu32;
	u32 up;
	u32 down;
	u16 lun_tim = waves * _ULTRO_TRANS_PER;            //���������ʱ��=��������*��������
	
	range_min = (float)dead_point*_CQ_RANGE_PERPOINT;      //��ȡ�Ժ��ÿ�����*��������=ʵ��ä����cm��
	range_min += 5;
	
	TRG = 1;//PA1��������Ϊ���ģʽ
	TIM2_Start();//����ʱ��ʹPA1�������PWM�ź�
	ADC_DMA_Trig( ADC1_DMA_Size );//��ʼAD�ɼ�������Ϊ��������
	Delay_Us(lun_tim);//��ʱ��ʱ��Ϊ���������ʱ��
	TIM2_Stop();//�رն�ʱ��1������PA1�������PWM
	Delay_Ms(120);
	TRG = 0;
	for( i=0; i<(_CQ_SIZE-_WIN_SUM_POINT/_CQ_SPAN_POINT) ; i++ )//ƽ���˲����˲���adc_sum_table[]
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
    avr /= _CQ_SIZE - 100 - dead_point;           //ֱ��  ƽ����ѹ   avr
    
	max_point = 0;
	//for( i=dead_point ; i<_CQ_SIZE-100 ; i++ )//�������ֵ
	for( i=0 ; i<_CQ_SIZE-100 ; i++ )//�������ֵ
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
    
    //avr = (max_point/5 - avr)/2+avr;                    //����ֵ�ж�
	 avr = (max_point/5- avr)/4+avr;                    //����ֵ�ж�   
	
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
 











