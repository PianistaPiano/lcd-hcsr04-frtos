/*
 * LCD_H44780U.h
 *
 *  Created on: Sep 4, 2023
 *      Author: Mati
 */

#ifndef INC_LCD_H44780U_H_
#define INC_LCD_H44780U_H_

#include "main.h"
#include "string.h"

// === Basics instructions for LCD === //
/*
 * In format: RS RW DB7 DB6 DB5 DB4 DB3 DB2 DB1 DB0
 * RW = 1: Read, RW = 0: Write
 * RS = 0: Command Register, RS = 1 Data Register
 */
#define ONE_BIT_MASK    		0b00000001
#define FIRST_LINE				0x80
#define	SECOND_LINE				0xC0
#define	THIRD_LINE				0x94
#define	FOURTH_LINE				0xD4
#define PLACES_IN_LINE			20
#define MAX_CHARS_ONE_LINE		40
#define MAX_CHARS_TWO_LINE		80

/*
 * I/D = 1: Increment
 * I/D = 0: Decrement
 * S = 1:	Accompanies display shift
 * S/C = 1:	Display Shift
 * S/C = 0: Cursor move
 * R/L = 1: Shift to the right
 * R/L = 0: Shift to the left
 * DL = 1:  8 bits
 * DL = 0:  4 bits
 * N = 1:	2 lines
 * N = 0:	1 line
 * F = 1:	5 x 10 dots
 * F = 0:	5 x 8 dots
 * BF = 1:	Internally operating
 * BF = 0:	Instructions acceptable
 *
 * D = 1: Entire display on, D = 0: off
 * C = 1: Cursor on, C = 0: Cursor off
 * B = 1: blinking of cursor position character
 */

// Instructions:
/*
 * Clear display: 				0  0  0  0  0  0  0  0  0  1
 * Return home: 				0  0  0  0  0  0  0  0  1  —
 * Entry mode set:				0  0  0  0  0  0  0  1 I/D S
 * Display on/off control: 		0  0  0  0  0  0  1  D  C  B
 * Cursor or display shift:		0  0  0  0  0  1 S/C R/L — —
 * Function set:				0  0  0  0  1  DL  N  F  — —
 * Set CGRAM address:			0  0  0  1 ACG ACG ACG ACG ACG ACG
 * Set DDRAM address:			0  0  1 ADD ADD ADD ADD ADD ADD ADD
 * Read busy flag & address: 	0  1  BF  AC  AC  AC  AC  AC  AC  AC
 * Write data to CG or DDRAM:	1  0  Write data
 * Read data from CG or DDRAM:  1  1  Read data
 */
/*
L1:         .eq 80h       ; pocz. 1. linii LCD
L2:         .eq 0C0h      ; pocz. 2. linii
L3:         .eq 94h       ; pocz. 3. linii
L4:         .eq 0D4h      ; pocz. 4. linii
*/
#define CLEAR_DISPLAY 			0b00000001 // nothing to configure here, clean command
#define RETURN_HOME 			0b00000010 // nothing to configure here, clean command
// wrtie commands with configure
#define ENTRY_MODE_SET 			0b00000100 //DB1 and DB0 are configured here [I/D S]
#define DISPLAY_CURSOR_ON_OFF	0b00001000 //DB2, DB1 and DB0 are configured here [D C B]
#define DISPLAY_OR_CURSOR_SHIFT 0b00010000 //DB3 and DB2 are configured[S/C R/L], DB1 and DB0 dont care
#define FUNCTION_SET			0b00100000 //DB4, DB3 and DB2 are configured [DL, N, F], DB1 and DB0 dont care
#define SET_CGRAM_ADDR			0b01000000 //DB5, DB4, DB3, DB2, DB1 and DB0 are configured [6xACG...]
#define SET_DDRAM_ADDR			0b10000000 //DB6, DB5, DB4, DB3, DB2, DB1 and DB0 are configured [7x ADD]
// write data with configure
#define WRITE_CG_OR_DDRAM		0b00000000
// reads commands with configure
#define READ_BUSY_FALG_AND_ADDR	0b00000000 //DB7, DB6, DB5, DB4, DB3, DB2, DB1 and DB0 are configured [BF, 7xAC], here is read

// Possible configure for ENTRY_MODE_SET
#define CURSOR_INC_ON_AND_ACC_DISP_ON 		0b11
#define CURSOR_INC_OFF_AND_ACC_DISP_OFF 	0b00
#define CURSOR_INC_ON_AND_ACC_DISP_OFF  	0b10

// Possible configure for DISPLAY_CURSOR_ON_OFF
#define DISP_ON_CURSOR_ON_BLINK_ON			0b111
#define DISP_ON_CURSOR_ON_BLINK_OFF			0b110
#define DISP_ON_CURSOR_OFF_BLINK_OFF		0b100
#define DISP_ON_CURSOR_OFF_BLINK_ON			0b101
#define DISP_OFF_CURSOR_OFF_BLINK_OFF		0b000

// Possible configure for DISPLAY_OR_CURSOR_SHIFT
#define DISP_SHIFT_RIGHT 	0b1100
#define DISP_SHIFT_LEFT  	0b1000
#define CURSOR_SHIFT_RIGHT	0b0100
#define CURSOR_SHIFT_LEFT	0b0000

// Possible configure for FUNCTION_SET
#define MODE_8BIT_TWO_LINE 	0b11000
#define MODE_8BIT_ONE_LINE 	0b10000
#define MODE_4BIT_TWO_LINE 	0b01000
#define MODE_4BIT_ONE_LINE 	0b00000
#define DOTS_5x10			0b100
#define	DOTS_5x8			0b000

// Possible configure for SET_CGRAM_ADDR


// Possible configure for SET_DDRAM_ADDR


// Possible configure for CG_OR_DDRAM
// Normal ASCII

// === Structures and enums === //

typedef enum
{
	LCD_4BIT_MODE = 0,
	LCD_8BIT_MODE
} LCD_MODE_t;

typedef enum
{
	LCD_ONE_LINE = 0,
	LCD_TWO_LINE
} LCD_LINES_t;

typedef enum
{
	LCD_DATA = 0,
	LCD_COMMAND
} LCD_REGISTER_t;

typedef enum
{
	LCD_5x8 = 0,
	LCD_5x10
} LCD_DOTS_t;

typedef enum
{
	LCD_CURSOR_ON = 0,
	LCD_CURSOR_OFF
} LCD_CURSOR_t;

typedef enum
{
	LCD_BLINKING_ON = 0,
	LCD_BLINKING_OFF
} LCD_BLINKING_t;

typedef struct LCD_H44780
{
	// DB7 to DB0 pins
	GPIO_TypeDef** Data_Ports; // pointer to array with ports in order {DB7, DB6, DB5, DB4, DB3, DB2, DB1, DB0}
	uint16_t* Data_Pins; // pointer to array with pins in order {DB7, DB6, DB5, DB4, DB3, DB2, DB1, DB0}
	// Register Select pin
	GPIO_TypeDef* RS_Port; // one pointer
	uint16_t RS_Pin; // one value
	// Read/Write pin
	GPIO_TypeDef* RW_Port; // one pointer
	uint16_t RW_Pin; // one value
	// Enable pin
	GPIO_TypeDef* EN_Port; // one pointer
	uint16_t EN_Pin; // one value

	// 8 bits or 4 bits mode
	LCD_MODE_t mode;
	// number of lines 1 or 2
	LCD_LINES_t num_of_lines;
	// dots
	LCD_DOTS_t dots;

	// visible cursor
	LCD_CURSOR_t cursor;
	// blinking position of cursor
	LCD_BLINKING_t blink;

} LCD_H44780_t;

// === Functions prototypes === //
uint8_t LCD_New_Line(LCD_H44780_t* lcd, uint8_t num_of_line);
uint8_t LCD_GoTo_X(LCD_H44780_t* lcd, uint8_t num_of_line, uint8_t x_postion_cursor_in_line);
uint8_t LCD_Write_Data(LCD_H44780_t* lcd, uint8_t* data, uint8_t length);
void LCD_Clear_Display(LCD_H44780_t* lcd);
uint8_t LCD_Clear_Line(LCD_H44780_t* lcd, uint8_t num_of_line);
uint8_t LCD_Clear_XtoY_In_Line(LCD_H44780_t* lcd, uint8_t num_of_line, uint8_t x_postion_cursor_in_line, uint8_t num_to_clear);




void LCD_Init(LCD_H44780_t* lcd, LCD_MODE_t mode, LCD_LINES_t num_of_lines, LCD_DOTS_t dots,
				LCD_CURSOR_t cursor, LCD_BLINKING_t blink);


void LCD_create(LCD_H44780_t* lcd, GPIO_TypeDef** Data_Ports, uint16_t* Data_Pins,
		  GPIO_TypeDef* RS_Port, uint16_t RS_Pin, GPIO_TypeDef* RW_Port, uint16_t RW_Pin,
		  GPIO_TypeDef* EN_Port, uint16_t EN_Pin);


#endif /* INC_LCD_H44780U_H_ */
