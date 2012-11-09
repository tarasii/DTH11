#ifndef __EXT_LCD_H
#define __EXT_LCD_H

#include "utils.h"


#define LCD_DELAY  		1
#define LCD_DELAY_LONG  	50

#define X_SIZE 			24

/* HD44780 COMMANDS ---------------------------*/
#define CMD_INIT		0x33
#define CMD_4BINIT	0x32
#define CMD_HOME		0x02
#define CMD_4BIT		0x28
#define CMD_ENTRY		0x06
#define CMD_CLRLCD	0x01
#define CMD_CURS		0x08
#define CMD_LINE1		0x80
#define CMD_LINE2		0x40

#define CURS_EN_BIT	0x04
#define CURS_CR_BIT	0x02
#define CURS_BL_BIT	0x01

/* PIN CONFIG ---------------------------------*/
#define DATA_PORT 		GPIOC
#define DATA_PIN0 		GPIO_Pin_0
#define DATA_PIN1 		GPIO_Pin_1
#define DATA_PIN2 		GPIO_Pin_2
#define DATA_PIN3 		GPIO_Pin_3
//#define DATA_PIN4 	GPIO_Pin_4
//#define DATA_PIN5 	GPIO_Pin_5
//#define DATA_PIN6 	GPIO_Pin_6
//#define DATA_PIN7 	GPIO_Pin_7

#define CTRL_PORT 	GPIOB
#define CTRL_E 			GPIO_Pin_5
//#define CTRL_RW 	GPIO_Pin_1
#define CTRL_RS			GPIO_Pin_4	 

#define WriteCommonBit(v, bit) 						\
 	if ((v) & (1 << bit)) 									\
 		GPIO_HIGH(DATA_PORT, DATA_PIN##bit);	\
 	else																		\
 		GPIO_LOW(DATA_PORT, DATA_PIN##bit);	  

void Write_LCD( unsigned char* message );
void Init_Ext_LCD(void);
void Clr_LCD(void);
void GotoHome(void);
void GotoXY(uint8_t pos, uint8_t line);

void LCD_CURS(uint8_t enable, uint8_t curs, uint8_t blink);
void LCD_ON(uint8_t curs, uint8_t blink);
void LCD_OFF(void);

#endif /* __EXT_LCD_H */

