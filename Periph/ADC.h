#ifndef _ADC_H
#define _ADC_H
#include "ADC.h"
#include "Sys.h"


void ADC_Config( void );
void ADC_DMA_Trig( u16 Size );

#define SAM_FRE        200000//采样频率
#define ADC1_DMA_Size  25000 //采样点数
extern u16 ADC1_ConvertedValue[ ADC1_DMA_Size ];

#endif




