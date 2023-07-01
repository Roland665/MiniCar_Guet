/*----------------------------------------------------------------------------
 * Name:    LED.c
 * Purpose: low level LED functions
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
#include "LED.h"

const uint32_t led_mask[] = { 1UL << 1, 1UL << 0, 1UL << 4, 1UL << 0  };

/*----------------------------------------------------------------------------
  initialize LED Pins (GPION1, GPION0, GPIOF0, GPIOF4)
 *----------------------------------------------------------------------------*/
void LED_Initialize (void) {

  /* Enable clock and init GPIO outputs */
  SYSCTL->RCGCGPIO |= (1UL << 12) | (1UL << 5);  /* enable clock for GPIOs    */
  GPION->DIR     |= led_mask[0] | led_mask[1];   /* PN1, PN0 is output        */
  GPION->DEN     |= led_mask[0] | led_mask[1];   /* PN1, PN0 is digital func. */
  GPIOF_AHB->DIR |= led_mask[2] | led_mask[3];   /* PF4, PF0 is output        */
  GPIOF_AHB->DEN |= led_mask[2] | led_mask[3];   /* PF4, PF0 is digital func. */

}

/*----------------------------------------------------------------------------
  Function that turns on requested LED
 *----------------------------------------------------------------------------*/
void LED_On (uint32_t num) {

  if (num < LED_NUM) {
    switch (num) {
      case 0: GPION->DATA     |=  led_mask[num]; break;
      case 1: GPION->DATA     |=  led_mask[num]; break;
      case 2: GPIOF_AHB->DATA |=  led_mask[num]; break;
      case 3: GPIOF_AHB->DATA |=  led_mask[num]; break;
    }
  }
}

/*----------------------------------------------------------------------------
  Function that turns off requested LED
 *----------------------------------------------------------------------------*/
void LED_Off (uint32_t num) {

  if (num < LED_NUM) {
    switch (num) {
      case 0: GPION->DATA     &= ~led_mask[num]; break;
      case 1: GPION->DATA     &= ~led_mask[num]; break;
      case 2: GPIOF_AHB->DATA &= ~led_mask[num]; break;
      case 3: GPIOF_AHB->DATA &= ~led_mask[num]; break;
    }
  }
}

/*----------------------------------------------------------------------------
  Function that outputs value to LEDs
 *----------------------------------------------------------------------------*/
void LED_Out(uint32_t value) {
  int i;

  for (i = 0; i < LED_NUM; i++) {
    if (value & (1<<i)) {
      LED_On (i);
    } else {
      LED_Off(i);
    }
  }
}
