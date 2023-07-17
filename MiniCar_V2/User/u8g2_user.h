#ifndef __U8G2_USER_H
#define __U8G2_USER_H			  	 
#include "sys.h"
#include "u8g2.h"

#define OLED_ADDR 0x3C 

uint8_t u8x8_byte_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
uint8_t u8x8_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);

#endif  
