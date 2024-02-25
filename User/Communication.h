#ifndef _COMMUNICATION_H
#define _COMMUNICATION_H
#include "Sys.h"
#include "Communication.h"
#include "Periph.h"

#define UART_WriteChar( x )  USART_WriteChar( USART1 , x )

void UART_WriteCurve(  u16 *s , u32 Num );
void UART_WriteCurve_s16(  s16 *s , u32 Num1 );

#endif

