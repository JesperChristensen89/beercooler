//-------------------------------------------------------------
// include
//-------------------------------------------------------------
#include "lcd.h"

//-------------------------------------------------------------
// functions
//-------------------------------------------------------------
void LCD_write(uint8_t data_or_command, uint8_t input) {
  //There are two memory banks in the LCD, data/RAM and commands. This 
  //function sets the DC pin high or low depending, and then sends
  //the data byte
  
  // tell the LCD if data or a command is being sent
  if (data_or_command) {
    LCD_SEND_DATA;
  } else {
    LCD_SEND_CMD;
  }
  
  LCD_CE_LOW; // pull LCD_CE (SS) pin low to signal transmission start
  SPI_write_byte(input);
  LCD_CE_HIGH; // pull LCD_CE (SS) pin high to signal transmission end

}

void LCD_goto_XY(uint8_t x, uint8_t y) {
  LCD_write(LCD_COMMAND, 0x80 | x);  // Column.
  LCD_write(LCD_COMMAND, 0x40 | y);  // Row.  ?
}

void LCD_clear(void) {
  
  for (int i = 0; i < (LCD_X * LCD_Y / 8); i++) {
    LCD_write(LCD_DATA, 0x00);
  }
    
  LCD_goto_XY(0, 0); // after display is cleared, return to the home position
}


void LCD_init(void) {
  
  // setup control pins
  LCD_DDR  |=  (1 << LCD_CE) | (1 << LCD_DC) | (1 << LCD_RESET);
  
  // reset LCD to a known state
  LCD_PORT &= ~(1 << LCD_RESET);
  LCD_PORT |=  (1 << LCD_RESET);

  LCD_write(LCD_COMMAND, 0x21); // tell LCD that extended commands follow
  LCD_write(LCD_COMMAND, 0xB4); // set LCD Vop (Contrast): Try 0xB1(good @ 3.3V) or 0xBF if your display is too dark
  LCD_write(LCD_COMMAND, 0x04); // set Temp coefficent
  LCD_write(LCD_COMMAND, 0x14); // LCD bias mode 1:48: Try 0x13 or 0x14

  LCD_write(LCD_COMMAND, 0x20); // must send 0x20 before modifying the display control mode
  LCD_write(LCD_COMMAND, 0x0C); // set display control, normal mode. 0x0D for inverse
  
  LCD_clear();
  
}

//this function takes in a character, looks it up in the font table/array
//And writes it to the screen
//Each character is 8 bits tall and 5 bits wide. We pad one blank column of
//pixels on each side of the character for readability.
void LCD_write_char(uint8_t input) {
  
  LCD_write(LCD_DATA, 0x00); // Blank vertical line padding

  for (int i = 0; i < 5; i++) {
    LCD_write(LCD_DATA, LCD_ASCII[input - 0x20][i]);
    // 0x20 is the ASCII character for Space (' '). The font table starts with this character
  }
  
  LCD_write(LCD_DATA, 0x00); // blank vertical line padding
}

void LCD_write_decimal(uint16_t input) {
  
  uint8_t output = 0;
  
  // input / 10000 returns the integer number of 10000's. input % 10000 returns the remainder of 1000's, '100's, '10's and 1's
	output = 0x30 + (input / 10000);
	LCD_write_char(output); // add 0x30 to get the ascii code of the numbers 0x30 = 0, 0x39 = 9
	input = input % 10000;
	
  // input / 1000 returns the integer number of 1000's. input % 1000 returns the remainder of 100's, '10's and 1's
	output = 0x30 + (input / 1000);
	LCD_write_char(output); // add 0x30 to get the ascii code of the numbers 0x30 = 0, 0x39 = 9
	input = input % 1000;
  
  // input / 100 returns the integer number of 100's. input % 100 returns the remainder of 10's and 1's
	output = 0x30 + (input / 100);
	LCD_write_char(output); // add 0x30 to get the ascii code of the numbers 0x30 = 0, 0x39 = 9
	input = input % 100;
	
	// input / 10 returns the integer number of 10's. input % 10 returns the remainder of 1's
	output = 0x30 + (input / 10);
	LCD_write_char(output);
	input = input % 10;

	// only 1's left, print those
	output = 0x30 + input;
	LCD_write_char(output);
	
}

//Given a string of characters, one by one is passed to the LCD
//void LCDString(char *characters) {
//  while (*characters)
//    LCDCharacter(*characters++);
//}
