//2014Äê8ÔÂ22ÈÕ
/******************** STM32F103 .C File  keil 4.72 v********************************************************************
* Company			 : BDK
* File Name          : 
* Author             : benten22  BDK
* firmware Version   : V3.5.0
* Date               : 2014/08/10
* Describe			 :

*************************************************************************************************************************/
#include "DAC.h"
#include "math.h"

void TIM2_Trigger_Init( void )
{
	TIM_TimeBaseInitTypeDef    TIM_TimeBaseStructure;
	
	/* TIM2 Periph clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	/* TIM2 Configuration */
	/* Time base configuration */
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 
	TIM_TimeBaseStructure.TIM_Period = 100;          
	TIM_TimeBaseStructure.TIM_Prescaler = 0x00;       
	TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;    
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	TIM_ARRPreloadConfig( TIM2, ENABLE ); 
	TIM_SetAutoreload( TIM2, 130 );
	
	/* TIM2 TRGO selection */
	TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);
	/* TIM2 enable counter */
	TIM_Cmd(TIM2, ENABLE);
}

void DAC_Config( void )
{
	DAC_InitTypeDef            DAC_InitStructure;

/* DAC--------------------------------------------------------------------------------------------------------*/
	/* DAC Periph clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
	
	/* DAC channel1 Configuration */
	DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;//DAC_Trigger_T2_TRGO;
	DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
	DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable;
	DAC_Init(DAC_Channel_1, &DAC_InitStructure);

	DAC_Cmd( DAC_Channel_1 , ENABLE );
	DAC_Cmd( DAC_Channel_2 , DISABLE );
	
    //DAC_DMACmd(DAC_Channel_2, ENABLE);
	DAC_SoftwareTriggerCmd( DAC_Channel_1 , ENABLE );
    DAC_SetChannel1Data( DAC_Align_12b_R , 1000 );
}


