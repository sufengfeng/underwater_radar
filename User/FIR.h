#ifndef _FIR_H
#define _FIR_H
#include "Sys.h"
#include "FIR.h"

#define NUM_TAPS   142
void FIR( u16 *ps , u32 Num , u16 *pc , u8 TP );
extern u16 Coefficient_Ram[NUM_TAPS];

void FIR_f( s16 *ps , u32 Num , float AMP );
	
#endif

