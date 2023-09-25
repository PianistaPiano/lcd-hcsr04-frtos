/*
 * LCD_H44780U.c
 *
 *  Created on: Sep 4, 2023
 *      Author: Mati
 */

#include "LCD_H44780U.h"
#include "cmsis_os.h"
#include "semphr.h"
#include "timers.h"

extern osTimerId_t LcdTransferTimerHandle;
extern osMutexId_t LcdWriteMutexHandle;
extern osSemaphoreId_t LcdTransferSemHandle;

// ==== statics functions ==== //
/**
  * @brief  This function Write 1 and after 1ms 0 to transfer data to LCD
  * @param  lcd - is a pointer to a lcd structure which contains all needed parameters
  * @retval None
  */
static void LCD_Transfer(LCD_H44780_t* lcd)
{
	// Raise E
	HAL_GPIO_WritePin(lcd->EN_Port, lcd->EN_Pin, 1);
	//HAL_Delay(1);
	xTimerStart(LcdTransferTimerHandle, 1);
	xSemaphoreTake(LcdTransferSemHandle, portMAX_DELAY);
	// Fall E
	HAL_GPIO_WritePin(lcd->EN_Port, lcd->EN_Pin, 0);

}


/**
  * @brief  This function writes Command or Data
  * @param  lcd - is a pointer to a lcd structure which contains all needed parameters
  * 		reg - specifies where to wrtie a data,
  * 		data - contains data to write
  * @retval None
  */
static void LCD_write(LCD_H44780_t* lcd, LCD_REGISTER_t reg, uint8_t data)
{
	if(reg == LCD_COMMAND)
	{
		HAL_GPIO_WritePin(lcd->RS_Port,lcd->RS_Pin, 0);
		HAL_GPIO_WritePin(lcd->RW_Port,lcd->RW_Pin, 0);
	}
	else if (reg == LCD_DATA)
	{
		HAL_GPIO_WritePin(lcd->RS_Port,lcd->RS_Pin, 1);
		HAL_GPIO_WritePin(lcd->RW_Port,lcd->RW_Pin, 0);
	}
	if(lcd->mode == LCD_8BIT_MODE)
	{
		for(uint8_t i = 1; i <= 8; i++)
		{
			HAL_GPIO_WritePin(lcd->Data_Ports[i-1], lcd->Data_Pins[i-1], (data >> (8-i)) & ONE_BIT_MASK);
		}
		LCD_Transfer(lcd);
	}
	else if(lcd->mode == LCD_4BIT_MODE)
	{
		for(uint8_t i = 1; i <= 4; i++)
		{
			HAL_GPIO_WritePin(lcd->Data_Ports[i-1], lcd->Data_Pins[i-1], (data >> (8-i)) & ONE_BIT_MASK);
		}
		LCD_Transfer(lcd);
		xTimerStart(LcdTransferTimerHandle, 1);
		xSemaphoreTake(LcdTransferSemHandle, portMAX_DELAY);
		for(uint8_t i = 1; i <= 4; i++)
		{
			HAL_GPIO_WritePin(lcd->Data_Ports[i-1], lcd->Data_Pins[i-1], (data >> (4-i)) & ONE_BIT_MASK);
		}
		LCD_Transfer(lcd);
	}
}

/**
  * @brief  This function writes Command to register command
  * @param  lcd - is a pointer to a lcd structure which contains all needed parameters
  * 		command - contains command to write
  * @retval None
  */
static void LCD_write_command(LCD_H44780_t* lcd, uint8_t command)
{
	LCD_write(lcd, LCD_COMMAND, command);
}
/**
  * @brief  This function writes data to register data
  * @param  lcd - is a pointer to a lcd structure which contains all needed parameters
  * 		data - contains data to write
  * @retval None
  */
static void LCD_write_data(LCD_H44780_t* lcd, uint8_t data)
{
	LCD_write(lcd, LCD_DATA, data);
}

// ==== functions for user ==== //
/**
  * @brief  This function goes to beginning  of specific line
  * @param  lcd - is a pointer to a lcd structure which contains all needed parameters
  * 		num_of_line - contains line number
  * @retval 1 - if params are out of range, 0 - if ok
  */
uint8_t LCD_New_Line(LCD_H44780_t* lcd, uint8_t num_of_line)
{
	if(num_of_line <= 4)
	{
		if(lcd->num_of_lines == LCD_TWO_LINE)
		{
			switch (num_of_line)
			{
				case 1:
					LCD_write_command(lcd, SET_DDRAM_ADDR | FIRST_LINE);
					break;
				case 2:
					LCD_write_command(lcd, SET_DDRAM_ADDR | SECOND_LINE);
					break;
				case 3:
					LCD_write_command(lcd, SET_DDRAM_ADDR | THIRD_LINE);
					break;
				case 4:
					LCD_write_command(lcd, SET_DDRAM_ADDR | FOURTH_LINE);
					break;
				default:
					break;
			}
		}
		else
		{
			// if num lines eq. 1 then any other num_of_line than 1 is line 3
			switch (num_of_line)
			{
				case 1:
					LCD_write_command(lcd, SET_DDRAM_ADDR | FIRST_LINE);
					break;
				default:
					LCD_write_command(lcd, SET_DDRAM_ADDR | THIRD_LINE);
					break;
			}
		}

	}
	else
	{
		return 1;
	}
	return 0;

}
/**
  * @brief  This function goes to specific place of specific line
  * @param  lcd - is a pointer to a lcd structure which contains all needed parameters
  * 		num_of_line - contains line number. Range: 1 to 4
  * 		x_postion_cursor_in_line - contains position in specific line. Range: 1 to PLACES_IN_LINE
  * @retval 1 - if params are out of range, 0 - if ok
  */
uint8_t LCD_GoTo_X(LCD_H44780_t* lcd, uint8_t num_of_line, uint8_t x_postion_cursor_in_line)
{
	if(lcd->num_of_lines == LCD_ONE_LINE)
	{
		if((num_of_line <= 4) && ((x_postion_cursor_in_line-1) <= (PLACES_IN_LINE -1)))
		{
			// if num lines eq. 1 then any other num_of_line than 1 is line 3
			switch (num_of_line)
			{
				case 1:
					LCD_write_command(lcd, (SET_DDRAM_ADDR | FIRST_LINE) + (x_postion_cursor_in_line - 1));
					break;
				default:
					LCD_write_command(lcd, (SET_DDRAM_ADDR | THIRD_LINE) + (x_postion_cursor_in_line - 1));
					break;
			}
		}
	}
	else
	{
		if((num_of_line <= 4) && ((x_postion_cursor_in_line - 1) <= (PLACES_IN_LINE -1)))
		{
			switch (num_of_line)
			{
				case 1:
					LCD_write_command(lcd, (SET_DDRAM_ADDR | FIRST_LINE) + (x_postion_cursor_in_line - 1));
					break;
				case 2:
					LCD_write_command(lcd, (SET_DDRAM_ADDR | SECOND_LINE) + (x_postion_cursor_in_line - 1));
					break;
				case 3:
					LCD_write_command(lcd, (SET_DDRAM_ADDR | THIRD_LINE) + (x_postion_cursor_in_line - 1));
					break;
				case 4:
					LCD_write_command(lcd, (SET_DDRAM_ADDR | FOURTH_LINE) + (x_postion_cursor_in_line - 1));
					break;
				default:
					// can't be here
					break;
			}
		}
		else
		{
			return 1;
		}

	}
	return 0;
}
/**
  * @brief  This function writes string to LCD. If string is long and LCD_TWO_LINE
  * is active after 20 chars go to new line.
  * @important Go to new line works ok when cursor is on the beginnig of the LCD!
  * @param  lcd - is a pointer to a lcd structure which contains all needed parameters
  * 		data - pointer to array with data
  * 		length - contains length of data
  * @retval 1 - if params are out of range, 0 - if ok
  */
uint8_t LCD_Write_Data(LCD_H44780_t* lcd, uint8_t* data, uint8_t length)
{
	if(lcd->num_of_lines == LCD_ONE_LINE && length > MAX_CHARS_ONE_LINE)
	{
		return 1;
	}
	else if(lcd->num_of_lines == LCD_TWO_LINE && length > MAX_CHARS_TWO_LINE)
	{
		return 1;
	}
	else
	{
		if(lcd->num_of_lines == LCD_TWO_LINE)
		{
			for(uint8_t i = 0; i < length; i++)
				{
					if(i == PLACES_IN_LINE)
					{
						LCD_New_Line(lcd, 2);
					}
					else if(i == PLACES_IN_LINE*2)
					{
						LCD_New_Line(lcd, 3);
					}
					else if(i == PLACES_IN_LINE*3)
					{
						LCD_New_Line(lcd, 4);
					}
					LCD_write_data(lcd, data[i]);
				}
		}
		else
		{
			for(uint8_t i = 0; i < length; i++)
			{
				LCD_write_data(lcd, data[i]);
			}
		}

	}
	return 0;

}

/**
  * @brief  This function clears entire display
  * @param  lcd - is a pointer to a lcd structure which contains all needed parameters
  * @retval None
  */
void LCD_Clear_Display(LCD_H44780_t* lcd)
{
	LCD_write_command(lcd, CLEAR_DISPLAY);
}
/**
  * @brief  This function clears specific line
  * @param  lcd - is a pointer to a lcd structure which contains all needed parameters
  * 		num_of_line - contains line number. Range: 1 to 4
  * @retval 1 - if params are out of range, 0 - if ok
  */
uint8_t LCD_Clear_Line(LCD_H44780_t* lcd, uint8_t num_of_line)
{
	if((num_of_line <= 4))
	{
		uint8_t clear_data[PLACES_IN_LINE];
		memset(clear_data, ' ', PLACES_IN_LINE*sizeof(uint8_t));
		if(lcd->num_of_lines == LCD_TWO_LINE)
		{
			switch (num_of_line)
			{
				case 1:
					LCD_write_command(lcd, (SET_DDRAM_ADDR | FIRST_LINE));
					LCD_Write_Data(lcd, clear_data, PLACES_IN_LINE);
					LCD_write_command(lcd, (SET_DDRAM_ADDR | FIRST_LINE));
					break;
				case 2:
					LCD_write_command(lcd, (SET_DDRAM_ADDR | SECOND_LINE));
					LCD_Write_Data(lcd, clear_data, PLACES_IN_LINE);
					LCD_write_command(lcd, (SET_DDRAM_ADDR | SECOND_LINE));
					break;
				case 3:
					LCD_write_command(lcd, (SET_DDRAM_ADDR | THIRD_LINE));
					LCD_Write_Data(lcd, clear_data, PLACES_IN_LINE);
					LCD_write_command(lcd, (SET_DDRAM_ADDR | THIRD_LINE));
					break;
				case 4:
					LCD_write_command(lcd, (SET_DDRAM_ADDR | FOURTH_LINE));
					LCD_Write_Data(lcd, clear_data, PLACES_IN_LINE);
					LCD_write_command(lcd, (SET_DDRAM_ADDR | FOURTH_LINE));
					break;
				default:
					// can't be here
					break;
			}
		}
		else
		{
			// if num lines eq. 1 then any other num_of_line than 1 is line 3
			switch (num_of_line)
			{
				case 1:
					LCD_write_command(lcd, (SET_DDRAM_ADDR | FIRST_LINE));
					LCD_Write_Data(lcd, clear_data, PLACES_IN_LINE);
					LCD_write_command(lcd, (SET_DDRAM_ADDR | FIRST_LINE));
					break;
				default:
					LCD_write_command(lcd, (SET_DDRAM_ADDR | THIRD_LINE));
					LCD_Write_Data(lcd, clear_data, PLACES_IN_LINE);
					LCD_write_command(lcd, (SET_DDRAM_ADDR | THIRD_LINE));
					break;
			}
		}

	}
	else
	{
		return 1;
	}
	return 0;

}
/**
  * @brief  This function clears from specific place X, a Y places in specific line
  * @param  lcd - is a pointer to a lcd structure which contains all needed parameters
  * 		num_of_line - contains line number. Range: 1 to 4
  * 		x_postion_cursor_in_line - contains position in specific line. Range: 1 to PLACES_IN_LINE
  * @retval 1 - if params are out of range, 0 - if ok
  */
uint8_t LCD_Clear_XtoY_In_Line(LCD_H44780_t* lcd, uint8_t num_of_line, uint8_t x_postion_cursor_in_line, uint8_t num_to_clear)
{

	if((num_of_line <= 4) && ((x_postion_cursor_in_line - 1) <= (PLACES_IN_LINE - 1)))
	{
		uint8_t clear_data[num_to_clear];
		memset(clear_data, ' ', num_to_clear*sizeof(uint8_t));
		if(lcd->num_of_lines == LCD_TWO_LINE)
		{
			switch (num_of_line)
			{
				case 1:
					LCD_write_command(lcd, (SET_DDRAM_ADDR | FIRST_LINE) + (x_postion_cursor_in_line - 1));
					LCD_Write_Data(lcd, clear_data, num_to_clear);
					LCD_write_command(lcd, (SET_DDRAM_ADDR | FIRST_LINE) + (x_postion_cursor_in_line - 1));
					break;
				case 2:
					LCD_write_command(lcd, (SET_DDRAM_ADDR | SECOND_LINE) + (x_postion_cursor_in_line - 1));
					LCD_Write_Data(lcd, clear_data, num_to_clear);
					LCD_write_command(lcd, (SET_DDRAM_ADDR | SECOND_LINE) + (x_postion_cursor_in_line - 1));
					break;
				case 3:
					LCD_write_command(lcd, (SET_DDRAM_ADDR | THIRD_LINE) + (x_postion_cursor_in_line - 1));
					LCD_Write_Data(lcd, clear_data, num_to_clear);
					LCD_write_command(lcd, (SET_DDRAM_ADDR | THIRD_LINE) + (x_postion_cursor_in_line - 1));
					break;
				case 4:
					LCD_write_command(lcd, (SET_DDRAM_ADDR | FOURTH_LINE) + (x_postion_cursor_in_line - 1));
					LCD_Write_Data(lcd, clear_data, num_to_clear);
					LCD_write_command(lcd, (SET_DDRAM_ADDR | FOURTH_LINE) + (x_postion_cursor_in_line - 1));
					break;
				default:
					// can't be here
					break;
			}
		}
		else
		{
			switch (num_of_line)
			{
				// if num lines eq. 1 then any other num_of_line than 1 is line 3
				case 1:
					LCD_write_command(lcd, (SET_DDRAM_ADDR | FIRST_LINE) + (x_postion_cursor_in_line - 1));
					LCD_Write_Data(lcd, clear_data, num_to_clear);
					LCD_write_command(lcd, (SET_DDRAM_ADDR | FIRST_LINE) + (x_postion_cursor_in_line - 1));
					break;
				default:
					LCD_write_command(lcd, (SET_DDRAM_ADDR | THIRD_LINE) + (x_postion_cursor_in_line - 1));
					LCD_Write_Data(lcd, clear_data, num_to_clear);
					LCD_write_command(lcd, (SET_DDRAM_ADDR | THIRD_LINE) + (x_postion_cursor_in_line - 1));
					break;
			}
		}

	}
	else
	{
		return 1;
	}
	return 0;
}

/**
  * @brief  This function Init lcd
  * @param  lcd - is a pointer to a lcd structure which contains all needed parameters
  * 		mode - 8 Bit or 4Bit communication
  * 		num_of_lines - 1 line or 2 lines
  * 		dots - 5x8 or 5x10 format
  * 		cursor - visible cursor or not visible
  * 		blink - blinking on cursor position or not
  * @retval None
  */
void LCD_Init(LCD_H44780_t* lcd, LCD_MODE_t mode, LCD_LINES_t num_of_lines, LCD_DOTS_t dots,
				LCD_CURSOR_t cursor, LCD_BLINKING_t blink)
{
	lcd->mode = mode;
	lcd->num_of_lines = num_of_lines;

	lcd->dots = dots;

	lcd->cursor = cursor;
	lcd->blink = blink;


	uint8_t command = 0;
	// for test // uint8_t data = 0;

	// Init

	if((lcd->mode == LCD_8BIT_MODE) && (lcd->num_of_lines == LCD_ONE_LINE) && (lcd->dots == LCD_5x8))
	{
		command = FUNCTION_SET | MODE_8BIT_ONE_LINE | DOTS_5x8;
	}
	else if((lcd->mode == LCD_8BIT_MODE) && (lcd->num_of_lines == LCD_TWO_LINE) && (lcd->dots == LCD_5x8))
	{
		command = FUNCTION_SET | MODE_8BIT_TWO_LINE | DOTS_5x8;
	}
	else if((lcd->mode == LCD_8BIT_MODE) && (lcd->num_of_lines == LCD_TWO_LINE) && (lcd->dots == LCD_5x10))
	{
		command = FUNCTION_SET | MODE_8BIT_TWO_LINE | DOTS_5x10;
	}
	else if((lcd->mode == LCD_8BIT_MODE) && (lcd->num_of_lines == LCD_ONE_LINE) && (lcd->dots == LCD_5x10))
	{
		command = FUNCTION_SET | MODE_8BIT_ONE_LINE | DOTS_5x10;
	}
	else if((lcd->mode == LCD_4BIT_MODE) && (lcd->num_of_lines == LCD_ONE_LINE) && (lcd->dots == LCD_5x8))
	{
		command = FUNCTION_SET | MODE_4BIT_ONE_LINE | DOTS_5x8;
	}
	else if((lcd->mode == LCD_4BIT_MODE) && (lcd->num_of_lines == LCD_TWO_LINE) && (lcd->dots == LCD_5x8))
	{
		command = FUNCTION_SET | MODE_4BIT_TWO_LINE | DOTS_5x8;
	}
	else if((lcd->mode == LCD_4BIT_MODE) && (lcd->num_of_lines == LCD_TWO_LINE) && (lcd->dots == LCD_5x10))
	{
		command = FUNCTION_SET | MODE_4BIT_TWO_LINE | DOTS_5x10;
	}
	else if((lcd->mode == LCD_4BIT_MODE) && (lcd->num_of_lines == LCD_ONE_LINE) && (lcd->dots == LCD_5x10))
	{
		command = FUNCTION_SET | MODE_4BIT_ONE_LINE | DOTS_5x10;
	}

	LCD_write_command(lcd, command);

	// On display and cursor and blink
	if((lcd->cursor == LCD_CURSOR_ON) && (lcd->blink == LCD_BLINKING_ON))
	{
		command = DISPLAY_CURSOR_ON_OFF | DISP_ON_CURSOR_ON_BLINK_ON;
	}
	else if((lcd->cursor == LCD_CURSOR_OFF) && (lcd->blink == LCD_BLINKING_ON))
	{
		command = DISPLAY_CURSOR_ON_OFF | DISP_ON_CURSOR_OFF_BLINK_ON;
	}
	else if((lcd->cursor == LCD_CURSOR_OFF) && (lcd->blink == LCD_BLINKING_OFF))
	{
		command = DISPLAY_CURSOR_ON_OFF | DISP_ON_CURSOR_OFF_BLINK_OFF;
	}
	else if((lcd->cursor == LCD_CURSOR_ON) && (lcd->blink == LCD_BLINKING_OFF))
	{
		command = DISPLAY_CURSOR_ON_OFF | DISP_ON_CURSOR_ON_BLINK_OFF;
	}
	LCD_write_command(lcd, command);

	// Entry mode set inc not accompanish
	command = ENTRY_MODE_SET | CURSOR_INC_ON_AND_ACC_DISP_OFF;
	LCD_write_command(lcd, command);

	// clear display
	command = CLEAR_DISPLAY;
	LCD_write_command(lcd, command);
	HAL_Delay(1);


//	// for test
//	data = (uint8_t)'H';
//	LCD_write_data(lcd, data);


}
/**
  * @brief  This function create a lcd
  * @param  lcd - is a pointer to a lcd structure which contains all needed parameters
  * 		Data_Ports - pointer to a array of pointers of GPIO_TypeDef (ports)
  * 		Data_Pins - pointer to a array of pins
  * 		RS_Port - port of RS pin
  * 		RS_Pin - RS pin number
  * 		RW_Port - port of RW pin
  * 		RW_Pin - RW pin number
  * 		EN_Port - port of EN pin
  * 		EN_Pin - EN pin number
  * 		cursor - visible cursor or not visible
  * 		blink - blinking on cursor position or not
  * @retval None
  */
void LCD_create(LCD_H44780_t* lcd, GPIO_TypeDef** Data_Ports, uint16_t* Data_Pins,
						GPIO_TypeDef* RS_Port, uint16_t RS_Pin,
						GPIO_TypeDef* RW_Port, uint16_t RW_Pin,
						GPIO_TypeDef* EN_Port, uint16_t EN_Pin)
{
	// Write parameters to struct
	lcd->Data_Ports = Data_Ports;
	lcd->Data_Pins = Data_Pins;

	lcd->RS_Port = RS_Port;
	lcd->RS_Pin = RS_Pin;

	lcd->RW_Port = RW_Port;
	lcd->RW_Pin = RW_Pin;

	lcd->EN_Port = EN_Port;
	lcd->EN_Pin = EN_Pin;
}


















#if 0
  HAL_Delay(100);

	// Function Set
	//RS and RW
	HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET); 	//0
	HAL_GPIO_WritePin(LCD_RW_GPIO_Port, LCD_RW_Pin, GPIO_PIN_RESET); 	//0
	// DB7...
	HAL_GPIO_WritePin(LCD_DB7_GPIO_Port, LCD_DB7_Pin, GPIO_PIN_RESET); 	//0
	HAL_GPIO_WritePin(LCD_DB6_GPIO_Port, LCD_DB6_Pin, GPIO_PIN_RESET); 	//0

	HAL_GPIO_WritePin(LCD_DB5_GPIO_Port, LCD_DB5_Pin, GPIO_PIN_SET); 	//1
	HAL_GPIO_WritePin(LCD_DB4_GPIO_Port, LCD_DB4_Pin, GPIO_PIN_SET); 	//1

	HAL_GPIO_WritePin(LCD_DB3_GPIO_Port, LCD_DB3_Pin, GPIO_PIN_RESET); 	//0
	HAL_GPIO_WritePin(LCD_DB2_GPIO_Port, LCD_DB2_Pin, GPIO_PIN_RESET); 	//0

	HAL_GPIO_WritePin(LCD_DB1_GPIO_Port, LCD_DB1_Pin, GPIO_PIN_RESET); 	//0
	HAL_GPIO_WritePin(LCD_DB0_GPIO_Port, LCD_DB0_Pin, GPIO_PIN_RESET); 	//0
	// Rise E
	HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_SET);
	// Falling E
	HAL_Delay(3);
	HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_RESET);

	HAL_Delay(100);

	// Clear Display
	HAL_GPIO_WritePin(LCD_DB7_GPIO_Port, LCD_DB7_Pin, GPIO_PIN_RESET); 	//0
	HAL_GPIO_WritePin(LCD_DB6_GPIO_Port, LCD_DB6_Pin, GPIO_PIN_RESET); 	//0

	HAL_GPIO_WritePin(LCD_DB5_GPIO_Port, LCD_DB5_Pin, GPIO_PIN_RESET); 	//0
	HAL_GPIO_WritePin(LCD_DB4_GPIO_Port, LCD_DB4_Pin, GPIO_PIN_RESET); 	//0

	HAL_GPIO_WritePin(LCD_DB3_GPIO_Port, LCD_DB3_Pin, GPIO_PIN_RESET); 	//0
	HAL_GPIO_WritePin(LCD_DB2_GPIO_Port, LCD_DB2_Pin, GPIO_PIN_RESET); 	//0

	HAL_GPIO_WritePin(LCD_DB1_GPIO_Port, LCD_DB1_Pin, GPIO_PIN_RESET); 	//0
	HAL_GPIO_WritePin(LCD_DB0_GPIO_Port, LCD_DB0_Pin, GPIO_PIN_SET); 	//1
	// Rise E
	HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_SET);
	// Falling E
	HAL_Delay(3);
	HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_RESET);

	HAL_Delay(100);

	// Display on off control
	HAL_GPIO_WritePin(LCD_DB7_GPIO_Port, LCD_DB7_Pin, GPIO_PIN_RESET); 	//0
	HAL_GPIO_WritePin(LCD_DB6_GPIO_Port, LCD_DB6_Pin, GPIO_PIN_RESET); 	//0

	HAL_GPIO_WritePin(LCD_DB5_GPIO_Port, LCD_DB5_Pin, GPIO_PIN_RESET); 	//0
	HAL_GPIO_WritePin(LCD_DB4_GPIO_Port, LCD_DB4_Pin, GPIO_PIN_RESET); 	//0

	HAL_GPIO_WritePin(LCD_DB3_GPIO_Port, LCD_DB3_Pin, GPIO_PIN_SET); 	//1
	HAL_GPIO_WritePin(LCD_DB2_GPIO_Port, LCD_DB2_Pin, GPIO_PIN_SET); 	//1

	HAL_GPIO_WritePin(LCD_DB1_GPIO_Port, LCD_DB1_Pin, GPIO_PIN_SET);   	//1
	HAL_GPIO_WritePin(LCD_DB0_GPIO_Port, LCD_DB0_Pin, GPIO_PIN_RESET); 	//0
	// Rise E
	HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_SET);
	// Falling E
	HAL_Delay(3);
	HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);

	//Entry mode set
	//DB7...
	HAL_GPIO_WritePin(LCD_DB7_GPIO_Port, LCD_DB7_Pin, GPIO_PIN_RESET);  //0
	HAL_GPIO_WritePin(LCD_DB6_GPIO_Port, LCD_DB6_Pin, GPIO_PIN_RESET);  //0

	HAL_GPIO_WritePin(LCD_DB5_GPIO_Port, LCD_DB5_Pin, GPIO_PIN_RESET);  //0
	HAL_GPIO_WritePin(LCD_DB4_GPIO_Port, LCD_DB4_Pin, GPIO_PIN_RESET);  //0

	HAL_GPIO_WritePin(LCD_DB3_GPIO_Port, LCD_DB3_Pin, GPIO_PIN_RESET);  //0
	HAL_GPIO_WritePin(LCD_DB2_GPIO_Port, LCD_DB2_Pin, GPIO_PIN_SET);  	//1

	HAL_GPIO_WritePin(LCD_DB1_GPIO_Port, LCD_DB1_Pin, GPIO_PIN_SET);  	//1
	HAL_GPIO_WritePin(LCD_DB0_GPIO_Port, LCD_DB0_Pin, GPIO_PIN_RESET);  //0
	// Rise E
	HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_SET);
	// Falling E
	HAL_Delay(3);
	HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);

	//Write data to CGRAM//DDRAM

	//RS and RW
	HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);	 	//1
	HAL_GPIO_WritePin(LCD_RW_GPIO_Port, LCD_RW_Pin, GPIO_PIN_RESET);	 //0
	//DB7...
	HAL_GPIO_WritePin(LCD_DB7_GPIO_Port, LCD_DB7_Pin, GPIO_PIN_RESET);	 //0
	HAL_GPIO_WritePin(LCD_DB6_GPIO_Port, LCD_DB6_Pin, GPIO_PIN_SET);	 //1

	HAL_GPIO_WritePin(LCD_DB5_GPIO_Port, LCD_DB5_Pin, GPIO_PIN_RESET);	 //0
	HAL_GPIO_WritePin(LCD_DB4_GPIO_Port, LCD_DB4_Pin, GPIO_PIN_RESET);	 //0

	HAL_GPIO_WritePin(LCD_DB3_GPIO_Port, LCD_DB3_Pin, GPIO_PIN_SET);	 //1
	HAL_GPIO_WritePin(LCD_DB2_GPIO_Port, LCD_DB2_Pin, GPIO_PIN_RESET);	 //0

	HAL_GPIO_WritePin(LCD_DB1_GPIO_Port, LCD_DB1_Pin, GPIO_PIN_RESET);	 //0
	HAL_GPIO_WritePin(LCD_DB0_GPIO_Port, LCD_DB0_Pin, GPIO_PIN_RESET);   //0
	// Rise E
	HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_SET);
	// Falling E
	HAL_Delay(3);
	HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);
#endif




