#ifndef _DAC_H
#define _DAC_H
#include "DAC.h"
#include "Sys.h"


#define DAC_DHR12RD_Address      0x40007420					//DAC2 DR
#define DAC_Buffer_Size			 SINWAVE_SIZE				//DAC_»º³å´óÐ¡ÉèÖÃ

#define DAC1 DAC->DHR12R1

void TIM2_Trigger_Init( void );
void DAC_Config( void );

#endif

