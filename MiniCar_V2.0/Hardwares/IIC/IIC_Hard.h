#ifndef __IIC_HARD_H
#define __IIC_HARD_H			  	 
#include "sys.h"

void IIC3_Init(void);
uint8_t IIC3_Read_Byte(uint16_t target_address, uint16_t data_address);
void IIC3_Write_Byte(uint16_t target_address, uint16_t data_address, uint8_t data);
#endif  
