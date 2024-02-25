#ifndef __LCD12864_H
#define	__LCD12864_H

#include "Periph.h"
#include "LCD12864.h"

 // �������дLCD����ģ��Ĵ����ĺ�ָ�� 
#define Disp_On			0xAF		// ��LCD��ʾ
#define Disp_Off		0xAE		// �ر�LCD��ʾ
#define SetStartLine	0x40		// ������ʾ��ʼ��(��Ҫ�;���������)
#define SetPage			0xB0		// ������ʾҳ(0~9,��Ҫ�;���ҳ����)
#define SetColMsb		0x10		// ������ʾ�и�4λ()
#define SetColLsb 		0x00		// ������ʾ�е�4λ(�����ߵ�λ�ϲ�������,��Ҫ�;����л�)
#define StatusRead		0x00		// ��״̬��,�ڸ�4λ����
#define ADC_Normal		0xA0		// ������������
#define ADC_Rever		0xA1		// ������������
#define DispNormal		0xA6		// ����������ʾ
#define DispRever		0xA7		// ����������ʾ
#define AllOn			0xA5		// ALL point ON
#define AllNormal		0xA4		// normal display
#define SetBias9		0xA2		// ����BIASΪ1/9
#define SetBias7		0xA3		// ����BIASΪ1/7
#define SetAddWrit		0xE0		//
#define SetAddRead		0xE1		//
#define ClrAdd			0xEE		//
#define Reset			0xE2		// ��λ
#define OUTNormal		0xc0		// com1->com64
#define OUTRever		0xc8		// com64->com1
#define POWERSet		0x2F		// power control
#define V5Ratio			0x24		// Select internal resistor ratio(Rb/Ra)mode
#define V5OutVol   		0x81		// Set the v5 output voltage
#define BoostRadio		0xF8		// Booster radio
#define SetBright		0x22		// ����Һ����������


#define SetRES 		GPIO_SetBits(GPIOD, GPIO_Pin_5)   //LCD_RES       14    ��λ�ź��ߣ�����ʱ����Ҫһֱ�ߵ�ƽ
#define ClrRES 		GPIO_ResetBits(GPIOD, GPIO_Pin_5) //                    �ź��ߴӸߵ�ƽ1���䵽�͵�ƽ0��Һ����λ

#define SetA0 		GPIO_SetBits(GPIOD, GPIO_Pin_6)    //LCD/A0        13    1����Ҫ��ʾ������
#define ClrA0 		GPIO_ResetBits(GPIOD, GPIO_Pin_6)  //                    0����Ҫ���Ƶ����� 

#define Set_CLK 	GPIO_SetBits(GPIOD, GPIO_Pin_4)  //LCD/Data6 SCL    9    ����ģʽʱ����
#define Clr_CLK 	GPIO_ResetBits(GPIOD, GPIO_Pin_4)//

#define Set_SI 		GPIO_SetBits(GPIOD, GPIO_Pin_3)   //LCD/Data7   SI   10    ����ģʽ������
#define Clr_SI 		GPIO_ResetBits(GPIOD, GPIO_Pin_3) //

#define SetCS1 		GPIO_SetBits(GPIOD, GPIO_Pin_7)    //LCD_CS        15    1������Һ��?
#define ClrCS1 		GPIO_ResetBits(GPIOD, GPIO_Pin_7)  //                    0д����Һ��




void LCD_12864_Config(void);   
void Write_COM(uint8_t cmdcode);
void WriteData(uint8_t Dispdata);
void ClrScreen(void);
void ResetLcd(void);
void printFront(uint8_t left, uint8_t top, int16_t uChar);
void printNumbers(uint8_t left,uint8_t top,int16_t code);



#endif /*__LCD12864_H */

