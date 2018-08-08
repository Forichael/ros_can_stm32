#ifndef __USART1_H
#define __USART1_H

#define USART1_Baud				115200

#include "includes.h"

void Usart1_Init(void);
void Usart1_GpioConfig(void);
void Usart1_ModeConfig(void);
void USART1_NVIC_Configuration(void);
void USART1_IRQHandler(void);
void USART1_Send(u16 data);
void USART1_Send_Assign(u8 *data,u8 Data_Len);
void USART1_Send_Str(USART_TypeDef* USARTx,u8 *str);
void USART1_Data_RX_Handler(u8 usart_data);
void USART1_Data_Pack(void);

typedef struct{
	s16 Motor_Ang_Vel_Average[3];
	s32 Pos[3];
}usart1_send_data;

typedef struct{
	s16 Motor_Ang_Vel[3];
}usart1_rcv_data;

typedef struct
{
  u8    Data[32];
  u8    Pointer;
	
}USART1_Data_TypeDef;

extern usart1_send_data usart_send_data;
extern usart1_rcv_data usart_rcv_data;



#endif
