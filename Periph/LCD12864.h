#ifndef __LCD12864_H
#define	__LCD12864_H

#include "Periph.h"
#include "LCD12864.h"

 // 定义各个写LCD驱动模块寄存器的宏指令 
#define Disp_On			0xAF		// 打开LCD显示
#define Disp_Off		0xAE		// 关闭LCD显示
#define SetStartLine	0x40		// 设置显示起始行(需要和具体行数或)
#define SetPage			0xB0		// 设置显示页(0~9,需要和具体页数或)
#define SetColMsb		0x10		// 设置显示列高4位()
#define SetColLsb 		0x00		// 设置显示列低4位(两个高低位合并后作用,需要和具体列或)
#define StatusRead		0x00		// 读状态字,在高4位保存
#define ADC_Normal		0xA0		// 从左至右输入
#define ADC_Rever		0xA1		// 从右至左输入
#define DispNormal		0xA6		// 从左至右显示
#define DispRever		0xA7		// 从右至左显示
#define AllOn			0xA5		// ALL point ON
#define AllNormal		0xA4		// normal display
#define SetBias9		0xA2		// 设置BIAS为1/9
#define SetBias7		0xA3		// 设置BIAS为1/7
#define SetAddWrit		0xE0		//
#define SetAddRead		0xE1		//
#define ClrAdd			0xEE		//
#define Reset			0xE2		// 复位
#define OUTNormal		0xc0		// com1->com64
#define OUTRever		0xc8		// com64->com1
#define POWERSet		0x2F		// power control
#define V5Ratio			0x24		// Select internal resistor ratio(Rb/Ra)mode
#define V5OutVol   		0x81		// Set the v5 output voltage
#define BoostRadio		0xF8		// Booster radio
#define SetBright		0x22		// 调节液晶屏的亮度


#define SetRES 		GPIO_SetBits(GPIOD, GPIO_Pin_5)   //LCD_RES       14    复位信号线，工作时，需要一直高电平
#define ClrRES 		GPIO_ResetBits(GPIOD, GPIO_Pin_5) //                    信号线从高电平1跳变到低电平0，液晶复位

#define SetA0 		GPIO_SetBits(GPIOD, GPIO_Pin_6)    //LCD/A0        13    1送入要显示的数据
#define ClrA0 		GPIO_ResetBits(GPIOD, GPIO_Pin_6)  //                    0送入要控制的命令 

#define Set_CLK 	GPIO_SetBits(GPIOD, GPIO_Pin_4)  //LCD/Data6 SCL    9    串行模式时钟线
#define Clr_CLK 	GPIO_ResetBits(GPIOD, GPIO_Pin_4)//

#define Set_SI 		GPIO_SetBits(GPIOD, GPIO_Pin_3)   //LCD/Data7   SI   10    串行模式数据线
#define Clr_SI 		GPIO_ResetBits(GPIOD, GPIO_Pin_3) //

#define SetCS1 		GPIO_SetBits(GPIOD, GPIO_Pin_7)    //LCD_CS        15    1读操作液晶?
#define ClrCS1 		GPIO_ResetBits(GPIOD, GPIO_Pin_7)  //                    0写操作液晶




void LCD_12864_Config(void);   
void Write_COM(uint8_t cmdcode);
void WriteData(uint8_t Dispdata);
void ClrScreen(void);
void ResetLcd(void);
void printFront(uint8_t left, uint8_t top, int16_t uChar);
void printNumbers(uint8_t left,uint8_t top,int16_t code);



#endif /*__LCD12864_H */

