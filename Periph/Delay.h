#ifndef _DELAY_H
#define _DELAY_H
#include "Sys.h"
#include "stm32f4xx.h"

#define nop() __nop()

// void Delay_Us( u32 Us );
// void Delay_Ms( u32 Ms );
#define Delay_ms Delay_Ms
#define Delay_us Delay_Us
#define Delay_RUs(x)   Delay_Us((x))

#endif

