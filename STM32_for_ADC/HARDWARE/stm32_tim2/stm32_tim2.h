#ifndef STM32_TIM2_H
#define STM32_TIM2_H

#include "stm32f10x.h"

 
#define START_TIME  time=0;RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);TIM_Cmd(TIM2, ENABLE)
#define STOP_TIME  TIM_Cmd(TIM2, DISABLE);RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , DISABLE)

void TIM2_NVIC_Configuration(void);
void TIM2_Config(u16 arr,u16 psc);

#endif	

