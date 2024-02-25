#include "TIM.h"

#define TIM_PERIOD       (168000000/109000-1)

void TIM1_Config( void )
{
	TIM_TimeBaseInitTypeDef  	TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  			TIM_OCInitStructure;
	
	TIM_DeInit(TIM1);
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_TIM1 , ENABLE );
	
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = TIM_PERIOD ;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit( TIM1 , &TIM_TimeBaseStructure );
	
	/* PWM1 Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = TIM_PERIOD/2;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC4Init( TIM1 , &TIM_OCInitStructure );
	
	TIM_OC4PreloadConfig( TIM1 , TIM_OCPreload_Enable );
	TIM_ARRPreloadConfig( TIM1 , ENABLE );
	
	TIM_Cmd( TIM1 , ENABLE );							/* TIM3 enable counter */
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

void TIM2_Config( u32 Fre )
{
	TIM_TimeBaseInitTypeDef  	TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  			TIM_OCInitStructure;
	
	u32 MaxData;
	u16 div=1;
	
	while( (SystemCoreClock/2/Fre/div)>65535 )
	{
		div++;
	}
	MaxData =  SystemCoreClock/2/Fre/div - 1;
	
/*   时钟等配置 */
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM2 , ENABLE );			//开启时钟
	
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = MaxData ;
	TIM_TimeBaseStructure.TIM_Prescaler = div-1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit( TIM2 , &TIM_TimeBaseStructure );
	
		/* PWM1 Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = MaxData/2;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC2Init( TIM2 , &TIM_OCInitStructure );
	
	TIM_OC2PreloadConfig( TIM2 , TIM_OCPreload_Enable );

	TIM_ARRPreloadConfig( TIM2 , DISABLE );
	TIM_Cmd( TIM2 , DISABLE );							/* TIM2 enable counter */
	TIM_CtrlPWMOutputs( TIM2, ENABLE );
}

void TIM2_Start( void )
{
	TIM2->CNT = 0;
	TIM_Cmd( TIM2 , ENABLE );							/* TIM2 enable counter */
}

void TIM2_Stop( void )
{
	TIM_Cmd( TIM2 , DISABLE );							/* TIM2 enable counter */
	TIM2->CNT = 0;
}

void TIM3_Config( u32 Fre )
{
	TIM_TimeBaseInitTypeDef  	TIM_TimeBaseStructure;
	u32 MaxData;
	u16 div=1;
	
	while( (SystemCoreClock/2/Fre/div)>65535 )
	{
		div++;
	}
	MaxData =  SystemCoreClock/2/Fre/div - 1;
	
/*   时钟等配置 */
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM3 , ENABLE );			//开启时钟
	
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = MaxData ;
	TIM_TimeBaseStructure.TIM_Prescaler = div-1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit( TIM3 , &TIM_TimeBaseStructure );
	
	TIM_SelectOutputTrigger( TIM3 , TIM_TRGOSource_Update );
	TIM_ARRPreloadConfig( TIM3 , ENABLE );
	TIM_Cmd( TIM3 , ENABLE );							/* TIM3 enable counter */
}


