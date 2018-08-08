#include "includes.h"

USART1_Data_TypeDef USART1_Data;
usart1_send_data usart_send_data;
usart1_rcv_data usart_rcv_data;

void Usart1_Init(void)
{
		Usart1_GpioConfig();
		Usart1_ModeConfig();
		USART1_NVIC_Configuration();
}
void Usart1_GpioConfig(void)
{
	  GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
    

   //串口使用的GPIO口配置    
   // Configure USART3 Rx (PB.11) as input floating      
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;    
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOA, &GPIO_InitStructure);    
    
   // Configure USART3 Tx (PB.10) as alternate function push-pull    
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;    
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;    
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    
   GPIO_Init(GPIOA, &GPIO_InitStructure);          
}
void Usart1_ModeConfig(void)
{
	 USART_InitTypeDef USART_InitStructure;

   USART_InitStructure.USART_BaudRate = USART1_Baud;    
   USART_InitStructure.USART_WordLength = USART_WordLength_8b;    
   USART_InitStructure.USART_StopBits = USART_StopBits_1;    
   USART_InitStructure.USART_Parity = USART_Parity_No;    
   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;    
   USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;    
    
    
   // Configure USART3    
   USART_Init(USART1, &USART_InitStructure);//配置串口2  3
    
  // Enable USART3 Receive interrupts 使能串口接收中断    
   USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);    
   //串口发送中断在发送数据时开启    
   //USART_ITConfig(USART3, USART_IT_TXE, ENABLE);    
    
   // Enable the USART3
   USART_Cmd(USART1, ENABLE);//使能串口3
}
void USART1_NVIC_Configuration(void)
{
		NVIC_InitTypeDef NVIC_InitStructure; 
		 //串口中断配置    
		 //Configure the NVIC Preemption Priority Bits       
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);    
    
   // Enable the USART3 Interrupt     
		NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;	 
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure); 
}
void USART1_IRQHandler(void)
{
	  if(USART_GetFlagStatus(USART1, USART_IT_RXNE) != RESET)
		{
			USART1_Data_RX_Handler(USART1->DR);
			USART_ClearITPendingBit(USART1, USART_IT_RXNE);
		}
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
}
/*******************USART3串口发送数据函数*************************/
void USART1_Send(u16 data)
{
  USART_SendData(USART1,data);
  while(USART_GetFlagStatus(USART1,USART_FLAG_TC) !=SET);//等待发送完成中断标志位	
  USART_ClearFlag(USART1,USART_FLAG_TC);
}

//发送指定长度的数据
void USART1_Send_Assign(u8 *data,u8 Data_Len)
{
  while(Data_Len--)
  {
	USART_SendData(USART1,*data++);
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) !=SET); //XE
	USART_ClearFlag(USART1,USART_FLAG_TC);
  }
}

//发送字符串
void USART1_Send_Str(USART_TypeDef* USARTx,u8 *str)
{
  u8 i=0;
  while(str[i]!='\0')
  {
	USART_Send(str[i]);
	i++;
  }
}

void USART1_Data_Pack(void)
{
	/*
	usart_send_data.Motor_Ang_Vel_Average[A_Wheel] = Motion_Data.Status.Motor_Ang_Vel_Average[A_Wheel];
	usart_send_data.Motor_Ang_Vel_Average[B_Wheel] = Motion_Data.Status.Motor_Ang_Vel_Average[B_Wheel];
	usart_send_data.Motor_Ang_Vel_Average[C_Wheel] = Motion_Data.Status.Motor_Ang_Vel_Average[C_Wheel];
	
	usart_send_data.Pos[A_Wheel] = Motion_Data.Status.Pos[A_Wheel];
	usart_send_data.Pos[B_Wheel] = Motion_Data.Status.Pos[B_Wheel];
	usart_send_data.Pos[C_Wheel] = Motion_Data.Status.Pos[C_Wheel];
	*/
	
	const u8 usart1_send_buf_len = 22;
	u8 usart1_send_buf[usart1_send_buf_len];
	
	usart1_send_buf[0] = 0x5A;
	usart1_send_buf[1] = 0x5A;
	/*
	usart1_send_buf[2] = Motion_Data.Status.Motor_Ang_Vel_Average[A_Wheel]>>8;
	usart1_send_buf[3] = Motion_Data.Status.Motor_Ang_Vel_Average[A_Wheel];
	usart1_send_buf[4] = Motion_Data.Status.Motor_Ang_Vel_Average[B_Wheel]>>8;
	usart1_send_buf[5] = Motion_Data.Status.Motor_Ang_Vel_Average[B_Wheel];
	usart1_send_buf[6] = Motion_Data.Status.Motor_Ang_Vel_Average[C_Wheel]>>8;
	usart1_send_buf[7] = Motion_Data.Status.Motor_Ang_Vel_Average[C_Wheel];
	
	usart1_send_buf[8]  = Motion_Data.Status.Pos[A_Wheel]>>24;
	usart1_send_buf[9]  = Motion_Data.Status.Pos[A_Wheel]>>16;
	usart1_send_buf[10] = Motion_Data.Status.Pos[A_Wheel]>>8;
	usart1_send_buf[11] = Motion_Data.Status.Pos[A_Wheel];
	
	usart1_send_buf[12] = Motion_Data.Status.Pos[B_Wheel]>>24;
	usart1_send_buf[13] = Motion_Data.Status.Pos[B_Wheel]>>16;
	usart1_send_buf[14] = Motion_Data.Status.Pos[B_Wheel]>>8;
	usart1_send_buf[15] = Motion_Data.Status.Pos[B_Wheel];
	
	usart1_send_buf[16] = Motion_Data.Status.Pos[C_Wheel]>>24;
	usart1_send_buf[17] = Motion_Data.Status.Pos[C_Wheel]>>16;
	usart1_send_buf[18] = Motion_Data.Status.Pos[C_Wheel]>>8;
	usart1_send_buf[19] = Motion_Data.Status.Pos[C_Wheel];
	
	*/
	
	usart1_send_buf[2] = 255>>8;
	usart1_send_buf[3] = 255;
	usart1_send_buf[4] = 255>>8;
	usart1_send_buf[5] = 255;
	usart1_send_buf[6] = 255>>8;
	usart1_send_buf[7] = 255;
	
	usart1_send_buf[8]  = 255>>24;
	usart1_send_buf[9]  = 255>>16;
	usart1_send_buf[10] = 255>>8;
	usart1_send_buf[11] = 255;
	
	usart1_send_buf[12] = 255>>24;
	usart1_send_buf[13] = 255>>16;
	usart1_send_buf[14] = 255>>8;
	usart1_send_buf[15] = 255;
	
	usart1_send_buf[16] = 255>>24;
	usart1_send_buf[17] = 255>>16;
	usart1_send_buf[18] = 255>>8;
	usart1_send_buf[19] = 255;
	
	
	usart1_send_buf[20] = usart1_send_buf_len;
	usart1_send_buf[21] = 0xAA;
								 
	USART1_Send_Assign(usart1_send_buf,usart1_send_buf_len);
	
}

void USART1_Data_RX_Handler(u8 usart_data)
{
  USART1_Data.Data[USART1_Data.Pointer]=usart_data;	
	USART1_Data.Pointer++;
	
	if(USART1_Data.Data[0] != 0xFF)
		USART1_Data.Pointer = 0;
	
	if(USART1_Data.Pointer>31)//防止数据溢出
		USART1_Data.Pointer = 0;
	
  if(USART1_Data.Pointer > 9)
  {//接受完一帧数据
		if((USART1_Data.Data[0] == 0xFF) && USART1_Data.Data[1] == 0xFF)//说明是返回的参数数据
		{
				if((USART1_Data.Data[USART1_Data.Pointer-1] == 0xAA) && 
					USART1_Data.Data[USART1_Data.Pointer-2] == USART1_Data.Pointer)
				{
					Motion_Control_Obj_Handler(USART1_Data.Data);
				}
		}
		USART1_Data.Pointer=0;
  }
 
}
