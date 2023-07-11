#ifndef __IIC_HARD_H
#define __IIC_HARD_H			  	 
#include "sys.h"
void IIC1_Init(void);
void IIC3_Init(void);
uint8_t IIC_Read_One_Byte(u32 ui32Base, uint16_t target_address, uint16_t data_address);
void IIC_Write_One_Byte(u32 ui32Base, uint16_t target_address, uint16_t data_address, uint8_t data);

#endif  
