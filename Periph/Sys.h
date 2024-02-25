#ifndef _SYS_H
#define _SYS_H
#include "stm32f4xx.h"
#include "BitBand.h"
#include "Delay.h"
#include "Sys.h"
#include "Stack.h"
#include <stdio.h>

void WFI_SET(void);		//ִ��WFIָ��
void INTX_DISABLE(void);//�ر������ж�
void INTX_ENABLE(void);	//���������ж�
void MSR_MSP(u32 addr);	//���ö�ջ��ַ 

#endif
