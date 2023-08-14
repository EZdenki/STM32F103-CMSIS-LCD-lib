//  main.c for STM32F103-CMSIS-LCD.lib
//  
//  Display text on a 16x2 LCD display using the STM32F103-CMSIS-LCD-lib.c library.
//      Mike Shegedin, EZdenki.com
//
//  Target Microcontroller: STM32F103 (Blue Pill)
//
//      Version 1.0   14 Aug 2023   General Cleanup. Changed wiring to be compatible with
//                                  3.3 V IO pins.
//      Version 0.9      Jul 2023   Started
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


#include "stm32f103xb.h"                // Primary CMSIS header file
#include "STM32F103-CMSIS-LCD-lib.c"    // Contains LCD_ routines
#include <stdlib.h>                     // Needed for atoi function


int
main()
{
  char numStr[16];                // These variables used for numerical output
  uint16_t count = 0;

  LCD_init();                     // Initialize the LCD
  LCD_cmd( LCD_1ST_LINE + 2 );    // Go to the first line of the LCD
  LCD_puts( "2x8 LCD" );          // Display text
  LCD_cmd( LCD_2ND_LINE + 1 );    // Go to the 2nd line
  LCD_puts( "Hello, World!!" );   // Display more text

  while( 1 )                                // Continuously count and display number
  {                                         // from 0 to 9999.
    LCD_cmd( LCD_1ST_LINE + 11 );           // Go to middle of 1st line
    LCD_puts( itoa( count, numStr, 10 ));   // Display the count

    if( ++count > 999 )               // If the count goes over 9999:
    {
      count = 0;                      // Reset the count
      LCD_cmd( LCD_1ST_LINE + 11 );   // Erase the old count
      LCD_puts( "    " );
    }
  }  
  return 1;
}  
