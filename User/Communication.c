#include "Communication.h"

void UART_WriteCurve(  u16 *s , u32 Num )
{
	u32 i;
	u8 j;
	s32 temps32;
	u8 temp8[3];
	
	UART_WriteChar(170);
	UART_WriteChar(90);
	UART_WriteChar(80);
	
	UART_WriteChar(50);		//通道
	
	UART_WriteChar(100);
	UART_WriteChar(180);
	UART_WriteChar( (Num*3+1)/256 );
	UART_WriteChar( (Num*3+1)%256 );
	
	UART_WriteChar( 1 );           //增益
	
	for( i=0; i<Num ; i++)
	{
		temps32 = s[i];
		temps32 -= 2047;
		temps32 *= 65536*2;
		temp8[0] = (temps32 & 0xff000000)>>24;
		temp8[1] = (temps32 & 0x00ff0000)>>16;
		temp8[2] = (temps32 & 0x0000ff00)>>8;
		for( j=0 ; j<3 ; j++ )
		{
			UART_WriteChar( temp8[j] );
		}
	}
	
	for( i=0; i<30 ; i++)
	{
		UART_WriteChar(0);
	}
}

void UART_WriteCurve_s16(  s16 *s , u32 Num1 )
{
	u32 i;
	u32 Num = Num1*4;
	UART_WriteChar( 'A' );
	UART_WriteChar( 'D' );
	UART_WriteChar( 'C' );
	UART_WriteChar( 'P' );
	UART_WriteChar( '_' );
	UART_WriteChar( 'R' );
	UART_WriteChar( 'X' );
	
	
	UART_WriteChar( (Num&0xff000000)>>24);
	UART_WriteChar( (Num&0x00ff0000)>>16);
	UART_WriteChar( (Num&0x0000ff00)>>8);
	UART_WriteChar( (Num&0x000000ff) );
	
	for( i=0 ; i<Num1 ; i++ )
	{
		if( s[i]&0x8000 )
		{
			UART_WriteChar(0xff);
			UART_WriteChar(0xff);
		}
		else
		{
			UART_WriteChar(0);
			UART_WriteChar(0);
		}
		UART_WriteChar( (s[i]&0xff00)>>8 );
		UART_WriteChar( s[i]&0x00ff );
	}
	
}




