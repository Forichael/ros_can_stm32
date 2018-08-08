#ifndef __USART2_H
#define __USART2_H

#include "includes.h"

#define USART2_Baud						RMDS_Baud
#define RMDS_Baud							115200
#define RMDS_USART						USART2

typedef struct
{
  bool  Flag;
  u8    Data[32];
  u8    Pointer;
  u8 		Descrip;
	
}USART_Data_TypeDef;

extern USART_Data_TypeDef USART_Data;

void Usart2_Init(void);
void Usart2_GpioConfig(void);
void Usart2_ModeConfig(void);
void USART2_NVIC_Configuration(void);
void USART2_IRQHandler(void);
void USART_Send(u16 data);
void USART_Send_Assign(u8 *data,u8 Data_Len);
void USART_Send_Str(USART_TypeDef* USARTx,u8 *str);
void RMDS_USART_Data_Receiver(u16 usart_data);
#endif
