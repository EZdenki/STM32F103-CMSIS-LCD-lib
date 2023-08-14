# STM32F103-CMSIS-LCD-lib
Simple library to provide LCD functionality for an STM32F103 microcontroller. (Blue Pill). The LCD is hooked up using the 4-bit connection (RS, EN, D4, D5, D6, D7 pins.)

## Supported Functionality
+ void LCD_cmd( uint8_t data )<br>
Send a command to the LCD module. Includes #define statements for useful commands.
+ void LCD_putc( char data )<br>
Display a single character on the LCD
+ void LCD_puts( char *data )<br>
Display a string on the LCD. The string is a null-terminated string and is passed by reference.
+ void LCD_init( void )<br>
Initialize the LCD and associated GPIO ports. Will use GPIO pins A8, A9, A10, A11, A14, and A15.

### See STM32F103-CMSIS-lib.c for further function and connection details.