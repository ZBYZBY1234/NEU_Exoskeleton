#ifndef __USART1_H
#define	__USART1_H

#include "stm32f10x.h"
#include <stdio.h>

void USART1_Config(void);
void UART1Test(void);
void UART1SendByte(unsigned char SendData);
int fputc(int ch, FILE* stream);
void UART1_Send_byte(USART_TypeDef* USARTx,u8 Data);

#endif /* __USART1_H */
