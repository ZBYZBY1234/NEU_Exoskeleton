#ifndef __stm32_dac_H
#define __stm32_dac_H	
#include "stm32f10x.h"
#include <stdio.h>

void DAC_Configuration(void);
void TIM6_Configuration(void);
void Dac1_Set_Vol(u16 vol);
#endif 
