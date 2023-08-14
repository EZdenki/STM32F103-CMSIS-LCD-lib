//  STM32F103-CMSIS-LCD-lib.c
//
//  Simple functionality to control a 16x2 LCD via the Blue Pill.
//      Mike Shegedin, EZdenki.com
//
//  Target Microcontroller: STM32F103 (Blue Pill)
//
//      Version 1.0   14 Aug 2023   General Cleanup. Changed wiring to be compatible with
//                                  3.3 V IO pins.
//      Version 0.9      Jul 2023   Started
//
//
//  HARDWARE SETUP
//  ==============
//  The 16x2 LCD module generally requires 5 V power and signals to operate. However,
//  3.3 V signalling is sufficient to drive the LCD IO pins. That being said, 5V is required
//  to properly drive the liquid crystal driver circuit.
//
//  In order to make protect the microcontroller from the 5 V signals on the LCD IO pins, the
//  pins that will be connected to the microcontroller should have approx. 10 k ohm dropping
//  reistors to ground. With that, the microcontroller will drive the pins high with 3.3 V and
//  when left floating, the pins will be tied to ground.
//
//  Note that this procedure is not strictly required on the STM32F103 as the IO pins used are
//  in fact 5 V tolerant. But this technique is useful when using a microcontroller that only
//  has 3.3 V IO pins available.
//
//     Blue Pill  16x2 LCD  INLINE  5V/GND
//     =========  ========  ======  ======
//        GND ----- VSS ------------- GND
//         5V ----- VDD ------------- 5V
//                                /-- 5V
//                   VO -[10k Pot]**     (LCD contrast control)
//                                \-- GND
//         B14 ----- RS -- [100K]* -- GND
//                   RW ------------- GND
//         B15 ----- EN -- [100K] --- GND
//                   D0 ------------- GND
//                   D1 ------------- GND
//                   D2 ------------- GND
//                   D3 ------------- GND
//         A8 ------ D4 -- [100K] --- GND
//         A9 ------ D5 -- [100K] --- GND
//        A10 ------ D6 -- [100K] --- GND
//        A11 ------ D7 -- [100K] --- GND
//                    A -- [1.0K] --- GND
//                    K ------------- GND
//
// ** Pot may be replaced by a 2k resistor between V0 and GND.
//         VO -- [ 2k ] --- GND (LCD contrast control)


#include "stm32f103xb.h"      // Primary CMSIS header file
#include "STM32F103-Delay-lib.c"  // for microsecond delay routine

#define LCD_RS_BIT (1<<14)
#define LCD_EN_BIT (1<<15)

#define LCD_RS_ON()  GPIOB->ODR |=  LCD_RS_BIT
#define LCD_EN_ON()  GPIOB->ODR |=  LCD_EN_BIT
#define LCD_RS_OFF() GPIOB->ODR &= ~LCD_RS_BIT
#define LCD_EN_OFF() GPIOB->ODR &= ~LCD_EN_BIT

#define LCD_CLEAR                0x01 // Clear display
#define LCD_HOME                 0x02 // Move cursor home
#define LCD_OFF                  0x08 // Disable LCD
#define LCD_ON_NO_CURSOR         0x0C // Turn on LCD, no cursor
#define LCD_ON_BLINK_CURSOR      0x0D // Turn on LCD, blinking block cursor
#define LCD_ON_LINE_CURSOR       0x0E // Turn on LCD, line cursor
#define LCD_ON_LINE_BLINK_CURSOR 0x0F // Turn on LCD, blinking line cursor
#define LCD_4B_58F_2L            0x2B // Set to 4-bit, 5x8 dot char, 2 lines
#define LCD_8B_58F_2L            0x23 // Set to 8-bit, 5x8 dot char, 2 lines
#define LCD_1ST_LINE             0x80 // Go to beginning of 1st line
#define LCD_2ND_LINE             0xC0 // Go to beginning of 2nd line


//  writeLowerNibble
//  Puts the lower 4 bits of 'data' onto GPIO pins A[11:7]
void
LCD_writeLowerNibble( uint8_t data )
{
  GPIOA->ODR &= 0xF0FF;    // Clear GPIO A[11:8]
  // Mask off the lower nibble of data, shift 8 bits to the left, and write those bits
  // to the newly cleared nibble from A[11:8]
  GPIOA->ODR |= (data & 0x0F) << 8;
}


//  writeUpperNibble
//  Writes the upper 4 bits of 'data' onto GPIO pins A[11:8]
void
LCD_writeUpperNibble( uint8_t data )
{
  GPIOA->ODR &= 0xF0FF;    // Clear GPIO A[11:8]
  // Mask off the upper nibble of data, shift 4 bits to the left, and write those bits
  // to the newly cleared nibble from A[11:8]
  GPIOA->ODR |= (data & 0xF0) << 4;
}


//  Pause
//  Actually halts the program here by entering an endless loop
//  For debugging.
void
pause( void )
{
  while(1);
}



//  LCD_pulse_EN
//  Make EN high, wait for 1 ms, and then bring it down low. This longish timing requirement
//  seems to be necessary due to 4-bit processing.
void
LCD_pulse_EN(void)
{
  LCD_EN_ON();
  delay_us( 1e3 );
  LCD_EN_OFF();
}


//  LCD_cmd
//  Send a 4-bit command to the LCD
void
LCD_cmd( uint8_t data )
{
  // Make sure EN and RS are low
  LCD_EN_OFF();
  LCD_RS_OFF();
  // Place upper nibble of command on GPIOA[11:8]
  LCD_writeUpperNibble( data );
  // Pulse EN pin to set this nibble
  LCD_pulse_EN();
  // Place lower nibble of command on GPIOA[11:8]
  LCD_writeLowerNibble( data );
  // Pulse EN pin to set this nibble
  LCD_pulse_EN();
  delay_us( 43 );
  if(( data == LCD_CLEAR ) || ( data == LCD_HOME ))
    delay_us( 1487 );
}


//  LCD_putc
//  Send a character to the LCD
void
LCD_putc( char data )
{
  // Set EN low, and RS high
  LCD_EN_OFF();
  LCD_RS_ON();
  // Place upper nibble of character on GPIOA[11:8]
  LCD_writeUpperNibble( data );
  // Pulse EN pin to set this nibble
  LCD_pulse_EN();
  // Place lower nibble of command on GPIOA[11:8]
  LCD_writeLowerNibble( data );
  // Pulse EN pin to set this nibble
  LCD_pulse_EN();
  delay_us( 43 );
}


//  LCD_puts
//  Takes a pointer to a null-terminated string and displays that string
//  from the current LCD cursor position. Does not check for LCD line/string
//  overflow.
void
LCD_puts( char *data )
{
  uint8_t j=0;

  while( data[j] != 0 )
  {
    LCD_putc( data[j] );
    j++;
  }
}


//  LCD_init
//  Initializes LCD by initializing required GPIO ports and pins used to talk
//  to the LCD. Also initializes the LCD screen itself to be in 4-bit mode.
//  This setup uses GPIO A8, A9, A10, A11 for LCD data pins 4, 5, 6, 7 respectively.
//  GPIO B14 is LCD RS and GPIO B15 is the LCD EN pin.
//  Note that these GPIO pins are set as floating in order to allow external resistor-
//  pullups to 5V. This is okay as these are 5V-tolerant pins.
void
LCD_init( void )
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN |
                  RCC_APB2ENR_IOPBEN;           // Enable GPIO Ports A and B

  GPIOA->CRH &= ~( 0b11 << GPIO_CRH_CNF8_Pos  |
                   0b11 << GPIO_CRH_CNF9_Pos  |
                   0b11 << GPIO_CRH_CNF10_Pos |
                   0b11 << GPIO_CRH_CNF11_Pos ); // Clear CNF Bits

  GPIOB->CRH &= ~( 0b11 << GPIO_CRH_CNF14_Pos |
                   0b11 << GPIO_CRH_CNF15_Pos ); // Clear CNF Bits

  GPIOA->CRH |= ( 0b01 << GPIO_CRH_MODE8_Pos  |
                /*  0b01 << GPIO_CRH_CNF8_Pos */  
                  0b01 << GPIO_CRH_MODE9_Pos  |
                /*  0b01 << GPIO_CRH_CNF9_Pos */  
                  0b01 << GPIO_CRH_MODE10_Pos |
                /*  0b01 << GPIO_CRH_CNF10_Pos */ 
                  0b01 << GPIO_CRH_MODE11_Pos  
                /*  0b01 << GPIO_CRH_CNF11_Pos */ ); // Set Mode bits to 01

  GPIOB->CRH |= ( 0b01 << GPIO_CRH_MODE14_Pos |
                /*  0b01 << GPIO_CRH_CNF14_Pos */ 
                  0b01 << GPIO_CRH_MODE15_Pos 
                /*  0b01 << GPIO_CRH_CNF15_Pos */ ); // Set Mode bits to 01

  // Start with delay to make sure the LCD module is fully powered up.
  delay_us(300e3);
  // Set EN and RS low
  LCD_EN_OFF();
  LCD_RS_OFF();
  // Write 0x02 to data pins to start 4-bit mode
  LCD_writeLowerNibble( 0x2 );
  // Pulse EN pin to set this nibble
  LCD_pulse_EN();

  LCD_cmd( LCD_4B_58F_2L );     // 4-bit, 5x8 font, 2 lines
  LCD_cmd( LCD_CLEAR );         // Clear display
  LCD_cmd( LCD_ON_NO_CURSOR );  // Display on, no cursor
}


