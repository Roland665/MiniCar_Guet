#ifndef __HC_SR04_H
#define __HC_SR04_H			  	 
#include "sys.h"

#define HC_SR04_GPIO GPIOA
#define HC_SR04_Pin GPIO_Pin_11
#define HC_SR04_Clock RCC_APB2Periph_GPIOA

#define HC_SR04_TRIG PAout(11)// ������ʾ�ն�����Ѱ���п�

void HC_SR04_Init(void);
#endif 
