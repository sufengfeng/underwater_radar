
#include "Correlation.h"

/*
FUNCTION  £∫ ª•œ‡πÿº∆À„FUNCTION
PARAMETER :

*/
void Correlation_Cal( s16 *pI , u16 sLen , u16 rLen )
{
	s32 table[1000];
	u32 i;
	s32 temps32;
	u32 j;
	s32 max;
	max = 0;
	for( i=0 ; i<rLen ; i++ )
	{
		table[i] = pI[i];
		if( table[i]>max )
		{
			max = table[i];
		}
	}
	max *= rLen;
	for( i=rLen ; i<sLen-rLen ; i++ )
	{
		temps32 = 0;
		for( j=i ; j<i+rLen-200 ; j++ )
		{
			temps32 += table[j-100]*pI[j];
		}
		pI[i] = temps32/max;
	}
}





