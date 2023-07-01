/*----------------------------------------------------------------------------
 * Name:    BTN.c
 * Purpose: low Level Push Button functions
 * Note(s):
 *----------------------------------------------------------------------------
 * This file is part of the uVision/ARM development tools.
 * This software may only be used under the terms of a valid, current,
 * end user licence from KEIL for a compatible version of KEIL software
 * development tools. Nothing else gives you the right to use this software.
 *
 * This software is supplied "AS IS" without warranties of any kind.
 *
 * Copyright (c) 2014 Keil - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include <TM4C129.h>

/*----------------------------------------------------------------------------
  initialize Push Button Pins (PJ0, PJ1)
 *----------------------------------------------------------------------------*/
void BTN_Initialize (void) {

  SYSCTL->RCGCGPIO |= (1ul << 8);                /* enable clock for GPIOs    */
  GPIOJ_AHB->DR2R |=  ((1ul << 0) | (1ul << 1)); /* PJ0, PJ1 2-mA Drive       */
  GPIOJ_AHB->PUR  |=  ((1ul << 0) | (1ul << 1)); /* PJ0, PJ1 pull-up          */
  GPIOJ_AHB->DIR  &= ~((1ul << 0) | (1ul << 1)); /* PJ0, PJ1 is intput        */
  GPIOJ_AHB->DEN  |=  ((1ul << 0) | (1ul << 1)); /* PJ0, PJ1 is digital func. */
}


/*----------------------------------------------------------------------------
  Get Push Button status
 *----------------------------------------------------------------------------*/
uint32_t BTN_Get (void) {

  return (~(GPIOJ_AHB->DATA) & ((1ul << 0) | (1ul << 1)));
}
