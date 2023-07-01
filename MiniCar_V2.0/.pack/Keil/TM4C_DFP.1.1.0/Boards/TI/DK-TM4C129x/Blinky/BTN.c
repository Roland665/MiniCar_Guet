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
  initialize Push Button Pins (PN3, PE5)
 *----------------------------------------------------------------------------*/
void BTN_Initialize (void) {

  SYSCTL->RCGCGPIO |= (1ul << 12) | (1ul << 4);  /* enable clock for GPIOs    */

  GPIOE_AHB->DR2R |=  (1ul << 5);                /* PE5 2-mA Drive            */
  GPIOE_AHB->PUR  |=  (1ul << 5);                /* PE5 pull-up               */
  GPIOE_AHB->DIR  &= ~(1ul << 5);                /* PE5 is intput             */
  GPIOE_AHB->DEN  |=  (1ul << 5);                /* PE5 is digital func.      */

  GPION->DR2R     |=  (1ul << 3);                /* PN3 2-mA Drive            */
  GPION->PUR      |=  (1ul << 3);                /* PN3 pull-up               */
  GPION->DIR      &= ~(1ul << 3);                /* PN3 is intput             */
  GPION->DEN      |=  (1ul << 3);                /* PN3 is digital func.      */
}


/*----------------------------------------------------------------------------
  Get Push Button status
 *----------------------------------------------------------------------------*/
uint32_t BTN_Get (void) {
	
	return ((~(GPIOE_AHB->DATA) & (1ul << 5)) |
	        (~(GPION->DATA)     & (1ul << 3))  );
}
