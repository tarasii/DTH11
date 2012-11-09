#include "ext_lcd.h"

void Pulse_E(void){
  GPIO_HIGH(CTRL_PORT, CTRL_E);	//E=1
	Delay(LCD_DELAY);
  GPIO_LOW(CTRL_PORT, CTRL_E);	//E=0
	Delay(LCD_DELAY);
}

void ClearBits(void){
   GPIO_LOW(DATA_PORT, DATA_PIN0);	//D4
   GPIO_LOW(DATA_PORT, DATA_PIN1);	//D5
   GPIO_LOW(DATA_PORT, DATA_PIN2);	//D6
   GPIO_LOW(DATA_PORT, DATA_PIN3);	//D7
   GPIO_LOW(CTRL_PORT, CTRL_RS);		//RS
   GPIO_LOW(CTRL_PORT, CTRL_E);  		//E
}

void WriteCommon(uint8_t  value){
	value = value & 0x0F;
	
  WriteCommonBit(value, 0);
  WriteCommonBit(value, 1);
  WriteCommonBit(value, 2);
  WriteCommonBit(value, 3);

	Pulse_E();	
}

void WriteCtrl(uint8_t  value){
  GPIO_LOW(CTRL_PORT, CTRL_RS);	//RS=0
	WriteCommon( value >> 4 );
	WriteCommon( value );
}

void WriteData(uint8_t value){
  GPIO_HIGH(CTRL_PORT, CTRL_RS);	//RS=1
	WriteCommon( value >> 4 );
	WriteCommon( value );
}

void Clr_LCD(void){
	WriteCtrl(CMD_CLRLCD);
	Delay(LCD_DELAY_LONG);
}

void GotoHome(void){
	WriteCtrl(CMD_HOME);
	Delay(LCD_DELAY_LONG);
}

void GotoXY(uint8_t pos, uint8_t line){
	uint8_t res;
	res = CMD_LINE1 + ((CMD_LINE2 * line) & 0xEF);	
	WriteCtrl(res);	
}

void LCD_CURS(uint8_t enable, uint8_t curs, uint8_t blink){	
	uint8_t res;
	res = CMD_CURS | (CURS_EN_BIT * enable) | (CURS_CR_BIT * curs) | (CURS_BL_BIT * blink);	
	WriteCtrl(res);
	Delay(LCD_DELAY_LONG);	
}

void LCD_ON(uint8_t curs, uint8_t blink){
	LCD_CURS(1, curs, blink);
}
void LCD_OFF(void){
	LCD_CURS(0, 0, 0);
}

void Init_Ext_LCD(void){
	ClearBits();
	
 	WriteCtrl(CMD_INIT);
 	Delay(LCD_DELAY_LONG);
 	WriteCtrl(CMD_4BINIT);

	WriteCtrl(CMD_4BIT);
	
	LCD_OFF();
	WriteCtrl(CMD_ENTRY);
	Clr_LCD();	
	LCD_ON(0,0);	
}

void Write_LCD( unsigned char* message ){
	uint8_t i;
	for( i=0; i < X_SIZE; i++ ){
		if( !message[i] ) break;
		WriteData(message[i]);
	}
}
