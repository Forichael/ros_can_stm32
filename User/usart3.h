
#ifndef __USART3_H
#define __USART3_H

#define USART3_Baud				115200

#include "includes.h"

void Usart3_Init(void);
void Usart3_GpioConfig(void);
void Usart3_ModeConfig(void);
void USART3_NVIC_Configuration(void);
void USART3_IRQHandler(void);
void USART3_Send(u16 data);
void USART3_Send_Assign(u8 *data,u8 Data_Len);
void USART3_Send_Str(USART_TypeDef* USARTx,u8 *str);
void USART3_Data_RX_Handler(u8 usart_data);
void USART3_Data_Pack(void);

typedef struct{
	s16 Motor_Ang_Vel_Average[3];
	s32 Pos[3];
}usart3_send_data;

typedef struct{
	s16 Motor_Ang_Vel[3];
}usart3_rcv_data;

typedef struct
{
  u8    Data[32];
  u8    Pointer;
	
}USART3_Data_TypeDef;

extern usart3_send_data usart_send_data;
extern usart3_rcv_data usart_rcv_data;



#endif
