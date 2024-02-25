
#include "PortConfig.h"

void Clock_Config( void )
{
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA , ENABLE );  
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB , ENABLE );
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOC , ENABLE );
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOD , ENABLE );
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOE , ENABLE );
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOF , ENABLE );
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOG , ENABLE );
	
	RCC_AHB1PeriphClockCmd( RCC_APB2Periph_SYSCFG , ENABLE );
}

void Port_Clk_Config( void )
{
	GPIO_InitTypeDef      GPIO_InitStructure;
//CLK	
	Clock_Config();                  						   //配置外设时钟
//PORTA
	//超声驱动PWM产生口 TIM2 CH2 口 PA1 驱动
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;                      //T2 CH2  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init( GPIOA , &GPIO_InitStructure ); 
	GPIO_PinAFConfig( GPIOA , GPIO_PinSource1 , GPIO_AF_TIM2 );    //映射IO
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;                  //  ADCIN2     SIG_AD      
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init( GPIOA , &GPIO_InitStructure );
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9| GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init( GPIOA , &GPIO_InitStructure );
	GPIO_PinAFConfig( GPIOA , GPIO_PinSource9 , GPIO_AF_USART1 );			//开启复用
	GPIO_PinAFConfig( GPIOA , GPIO_PinSource10 , GPIO_AF_USART1 );
	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;       //           
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
//	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;       			//RS485_TEN          
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
//PORTB
	
//PORTC
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;                  //  ADCIN12          
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
//PORTD

/* 
	LCD12864 PORT CONFIG
	GPIO_Pin_8 控制LCD RES ,
	GPIO_Pin_9控制LCD A0,  
	GPIO_Pin_10 控制LCD Data6 ,
	GPIO_Pin_11控制LCD Data7
*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 |GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init( GPIOD , &GPIO_InitStructure );
	GPIO_ResetBits( GPIOD, GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 );   //输出低电平

//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;                  //           
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_Init(GPIOD, &GPIO_InitStructure);	
//	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5| GPIO_Pin_6;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_Init( GPIOD , &GPIO_InitStructure );
//	GPIO_PinAFConfig( GPIOD , GPIO_PinSource5 , GPIO_AF_USART2 );			//开启复用
//	GPIO_PinAFConfig( GPIOD , GPIO_PinSource6 , GPIO_AF_USART2 );
//	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8| GPIO_Pin_9;                   //USART3
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init( GPIOD , &GPIO_InitStructure );
	GPIO_PinAFConfig( GPIOD , GPIO_PinSource8 , GPIO_AF_USART3 );			//开启复用
	GPIO_PinAFConfig( GPIOD , GPIO_PinSource9 , GPIO_AF_USART3 );
//	
//PORTD
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;      //       
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
//	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;                      //T1 CH4   
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
//	GPIO_Init( GPIOE , &GPIO_InitStructure ); 
//	
//	GPIO_PinAFConfig( GPIOE, GPIO_PinSource14 , GPIO_AF_TIM1 );    //映射IO
//	
}
