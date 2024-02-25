#include "Delay.h"


// void Delay_Us( u32 Us )
// {
// 	u32 nTime;
// 	if( Us!=0)	
// 	{
// 		Us = 21*Us;
// 		SysTick->CTRL = 0;
// 		SysTick->LOAD  = Us;			//装载重装值
// 		SysTick->VAL = 0;					//清寄存器值开始计数
	
// 		SysTick->CTRL = 0x00000001;		//开启计数 SysTick->CTRL &= SysTick_CLKSource_HCLK_Div8; SysTick_CTRL_TICKINT_Msk
// 		do
// 		{
// 			nTime = SysTick->CTRL;
// 			nTime &= 0x00010000;
// 		}while( nTime == 0  );
// 		SysTick->CTRL = 0;		//关闭
// 	}
// }

// void Delay_Ms( u32 Ms )	  			//ms级别延时函数
// {
// 		for( ; Ms>0 ; Ms-- )
// 		{
// 			Delay_Us(1000);
// 		}
// }

// void Delay_Mis(u32 i)
// {
// 	u32 j;
// 	for( ; i>0 ; i-- )
// 	{
// 		for( j=0xffff ; j>0 ; j-- );
// 	}
// }

