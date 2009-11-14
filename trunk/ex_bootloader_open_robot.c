///////////////////////////////////////////////////////////////////////////
////                      ex_bootloader_open_robot.c                   ////
////                                                                   ////
////  This program is a bootloader for Open-Robot's PIC18F4520.        ////
////                                                                   ////
///////////////////////////////////////////////////////////////////////////

#include <18F4520.h>

// #define _20MHZ_   // Uncomment this line if using older 20MHz resonator board.
#define _40MHZ_   // Uncomment this line if using the new 40MHz crystal board.

#if defined(_40MHZ_)
#fuses H4,NOLVP,WDT2048,PUT,NOBROWNOUT,NOPROTECT
#use delay(clock=40000000)
#use rs232(baud=115200, xmit=PIN_C6, rcv=PIN_C7,ERRORS,RESTART_WDT,DISABLE_INTS)
#elif defined(_20MHZ_)
#fuses HS,NOLVP,WDT2048,PUT,NOBROWNOUT,NOPROTECT
#use delay(clock=20000000)
#use rs232(baud=115200, xmit=PIN_C6, rcv=PIN_C7,ERRORS,RESTART_WDT,DISABLE_INTS)
#endif

#define _bootloader

#include <bootloader_open_robot.h>
#include <loader.c>

#org LOADER_END+2,LOADER_END+20

void application(void)
{
  while(TRUE);
}

#org 0x40,0x7F

void main(void)
{
   if (!input(PIN_B3))
   {
      load_program();
   }
   application();
}

#ORG default

#int_global
void isr(void)
{
   jump_to_isr(LOADER_END+5*(getenv("BITS_PER_INSTRUCTION")/8));
}


