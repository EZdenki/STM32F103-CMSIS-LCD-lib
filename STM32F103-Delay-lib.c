//  STM32F103-Delay-lib.c
//    Version 1.0   6 Aug 2023    Forked from STM32F103-Delay-lib. Renamed pause() to halt().
//                                Updated Comments
//
//  Target Microcontroller: STM32F103 (Blue Pill) operating at a clock speed of 8 MHz
//  Mike Shegedin, 6 Aug 2023
//
//  Code to implement the following routines:
//  -----------------------------------------
//  halt(void)
//    Halts program by entering endless loop.
//  -----------------------------------------
//  delay_us( uint32_t d )
//    Delay d microseconds. Range: 5(?) to 858 million us (approx. 14 min, 18 s)
//  -----------------------------------------

#ifndef __STM32F103_DELAY_LIB_C
#define __STM32F103_DELAY_LIB_C
//  Halt
//  Halts the program here by entering an endless loop
//  For debugging.
void
halt( void )
{
  while(1);
 }

//  delay_us
//  Input: uint16_t d
//  Causes a delay of approx d uS. The shortest time is approx. 8 us.
//  ** Only works at clock speed of 8 MHz!
void
delay_us( uint32_t d)
{
  d=d*5/7;
  for( uint32_t x=0; x< d; x++) ;
}


#endif /*__STM32F103_HALT_LIB_C */
