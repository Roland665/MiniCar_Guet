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

const uint32_t led_mask[] = { 1UL << 5, 1UL << 4, 1UL << 7  };

/*----------------------------------------------------------------------------
  initialize LED Pins (PN5, PQ4, PQ7)
 *----------------------------------------------------------------------------*/
void LED_Initialize (void) {

  /* Enable clock and init GPIO outputs */
  SYSCTL->RCGCGPIO |= (1UL << 14) | (1UL << 12); /* enable clock for GPIOs    */
  GPION->DIR     |= led_mask[0];                 /* PN5      is output        */
  GPION->DEN     |= led_mask[0];                 /* PN5      is digital func. */
  GPIOQ->DIR     |= led_mask[1] | led_mask[2];   /* PQ4, PQ7 is output        */
  GPIOQ->DEN     |= led_mask[1] | led_mask[2];   /* PQ4, PQ7 is digital func. */

}

/*----------------------------------------------------------------------------
  Function that turns on requested LED
 *----------------------------------------------------------------------------*/
void LED_On (uint32_t num) {

  if (num < LED_NUM) {
    switch (num) {
      case 0: GPION->DATA     |=  led_mask[num]; break;
      case 1: GPIOQ->DATA     |=  led_mask[num]; break;
      case 2: GPIOQ->DATA     |=  led_mask[num]; break;
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
      case 1: GPIOQ->DATA     &= ~led_mask[num]; break;
      case 2: GPIOQ->DATA     &= ~led_mask[num]; break;
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
