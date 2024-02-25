#ifndef _TIM_H
#define _TIM_H
#include "Sys.h"
#include "TIM.h"


void TIM1_Config( void );
void TIM2_Config( u32 Fre );
void TIM2_Start( void );
void TIM2_Stop( void );

void TIM3_Config( u32 Fre );

#define PWM_ON()   TIM_CtrlPWMOutputs(TIM2, ENABLE)
#define PWM_OFF()  TIM_CtrlPWMOutputs(TIM2, DISABLE)


#endif
