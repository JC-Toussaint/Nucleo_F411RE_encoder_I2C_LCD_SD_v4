#include <i2c_HD44780.h>
#include <stdarg.h>
#include <stdio.h>
extern I2C_HandleTypeDef hi2c1;  // change your handler here accordingly

//#define SLAVE_ADDRESS_LCD (0x3F<<1) // change this to (0x27<<1) if necessary
#define SLAVE_ADDRESS_LCD (0x20<<1)

/**
  * @brief  send command to HD44780 LCD module via I2C
  * @param  cmd
  * @param  None
  * @retval None
  */
void lcd_send_cmd (char cmd)
{
  char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	data_t[0] = data_u|0x0C;  //en=1, rs=0
	data_t[1] = data_u|0x08;  //en=0, rs=0
	data_t[2] = data_l|0x0C;  //en=1, rs=0
	data_t[3] = data_l|0x08;  //en=0, rs=0

	while(HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100)!= HAL_OK)
	  {
	    /* Error_Handler() function is called when Timout error occurs */
	    if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
	    {
	      Error_Handler();
	    }
	  }
}

/**
  * @brief  send data to HD44780 LCD module via I2C
  * @param  data
  * @param  None
  * @retval None
  */
void lcd_send_data (char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=0
	data_t[1] = data_u|0x09;  //en=0, rs=0
	data_t[2] = data_l|0x0D;  //en=1, rs=0
	data_t[3] = data_l|0x09;  //en=0, rs=0

	while(HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100)!= HAL_OK)
	  {
	    /* Error_Handler() function is called when Timout error occurs */
	    if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
	    {
	      Error_Handler();
	    }
	  }
}

/**
  * @brief  Initializes HD44780 LCD module in 4-bit mode
  * @param  None
  * @param  None
  * @retval None
  */
void lcd_init (void)
{
	  //Initialization of HD44780-based LCD (4-bit HW)
	lcd_send_cmd (0x33);
	lcd_send_cmd (0x32);
	lcd_send_cmd (0x28);   //Function Set 4-bit mode
	lcd_send_cmd (0x0c);   //Display On/Off Control
	lcd_send_cmd (0x06);   //Entry mode set
	lcd_send_cmd (0x02);   //Clear Display
	//Minimum delay to wait before driving LCD module
	HAL_Delay(200);
}

/**
  * @brief  Set Display On
  * @param  None
  * @param  None
  * @retval None
  */
void lcd_disp_on(void)
{
  lcd_send_cmd(0x0C);
}

/**
  * @brief  Set Display Off
  * @param  None
  * @param  None
  * @retval None
  */
void lcd_disp_off(void)
{
  lcd_send_cmd(0x08);
}

/**
  * @brief  Set Cursor to Home position
  * @param  None
  * @param  None
  * @retval None
  */
void lcd_home(void)
{
  lcd_send_cmd(0x02);
  HAL_Delay(2);
}

/**
  * @brief  Clear LCD module display
  * @param  None
  * @param  None
  * @retval None
  */
void lcd_clear_display(void)
{
  lcd_send_cmd(0x01);
  HAL_Delay(2);
}

/**
  * @brief  Set Cursor to a specified location given by row and column information
  * @param  Row Number (1 to 4)
  * @param  Column Number (1 to 20) Assuming a 4 X 20 characters display
  * @retval None
  */
void lcd_locate(uint8_t row, uint8_t column)
{
  column--;
  switch (row)
  {
    case 1:
      /* Set cursor to 1st row address and add index*/
    	lcd_send_cmd(column |= 0x80);
        break;
    case 2:
      /* Set cursor to 2nd row address and add index*/
    	lcd_send_cmd(column |= 0xc0);
        break;
    case 3:
        /* Set cursor to 3rd row address and add index*/
      	lcd_send_cmd(column |= 0x94);
      	break;
    case 4:
        /* Set cursor to 4th row address and add index*/
      	lcd_send_cmd(column |= 0xd4);
      	break;
    default:
        break;
  }
}

/**
  * @brief  Print Character on LCD module
  * @param  Ascii value of character
  * @param  None
  * @retval None
  */
void lcd_printchar(unsigned char ascode)
{
	lcd_send_data(ascode);
}

/**
  * @brief  Display of a characters string
  * @param  Text to be displayed
  * @param  None
  * @retval None
  */
void lcd_print_string (char *str)
{
	while (*str) lcd_send_data (*str++);
}

/**
  * @brief  lcd printf function
  * @param  string with standard defined formats
  * @param
  * @retval None
  */
void lcd_printf(const char *fmt, ...)
{
  uint32_t i;
  uint32_t text_size, letter;
  static char text_buffer[32];
  va_list args;

  va_start(args, fmt);
  text_size = vsprintf(text_buffer, fmt, args);

  // Process the string
  for (i = 0; i < text_size; i++)
  {
    letter = text_buffer[i];

    if (letter == 10)
      break;
    else
    {
      if ((letter > 0x1F) && (letter < 0x80))
        lcd_printchar(letter);
    }
  }
}

