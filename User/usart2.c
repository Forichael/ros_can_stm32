#include "includes.h"


/*******************USART2串口初始化函数*************************/
void Usart2_Init(void){
		Usart2_GpioConfig();
		Usart2_ModeConfig();
		USART2_NVIC_Configuration();
}

/*******************GPIO初始化函数*************************/
void Usart2_GpioConfig(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE); //使能UART2所在GPIOA的时钟    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);   
    

   //串口使用的GPIO口配置    
   // Configure USART2 Rx (PA.3) as input floating      
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;    
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;    
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOA, &GPIO_InitStructure);    
    
   // Configure USART2 Tx (PA.2) as alternate function push-pull    
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;    
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    
   GPIO_Init(GPIOA, &GPIO_InitStructure);          
}

/*******************Mode初始化函数*************************/
void Usart2_ModeConfig(void)
{
   USART_InitTypeDef USART_InitStructure;

   USART_InitStructure.USART_BaudRate = USART2_Baud;    
   USART_InitStructure.USART_WordLength = USART_WordLength_8b;    
   USART_InitStructure.USART_StopBits = USART_StopBits_1;    
   USART_InitStructure.USART_Parity = USART_Parity_No;    
   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;    
   USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;    
    
    
   // Configure USART2    
   USART_Init(USART2, &USART_InitStructure);//配置串口2   
    
  // Enable USART2 Receive interrupts 使能串口接收中断    
   USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);    
   //串口发送中断在发送数据时开启    
   //USART_ITConfig(USART2, USART_IT_TXE, ENABLE);    
    
   // Enable the USART2
   USART_Cmd(USART2, ENABLE);//使能串口2  
	 

}

/*******************中断初始化函数*************************/
void USART2_NVIC_Configuration(void){
	 NVIC_InitTypeDef NVIC_InitStructure; 
   //串口中断配置    
   //Configure the NVIC Preemption Priority Bits       
   NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);    
    
   // Enable the USART2 Interrupt     
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;	 
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);   
}

/*******************USART2中断处理函数*************************/
void USART2_IRQHandler(void)
{	
  if(USART_GetFlagStatus(USART2, USART_IT_RXNE) != RESET)
  {
		RMDS_USART_Data_RX_Handler(RMDS_USART->DR);
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
  }
  USART_ClearITPendingBit(USART2, USART_IT_RXNE);
	
}

/*******************USART2串口发送数据函数*************************/
void USART_Send(u16 data)
{
  USART_SendData(USART2,data);
  while(USART_GetFlagStatus(USART2,USART_FLAG_TC) !=SET);//等待发送完成中断标志位	
  USART_ClearFlag(USART2,USART_FLAG_TC);
}

//发送指定长度的数据
void USART_Send_Assign(u8 *data,u8 Data_Len)
{
  while(Data_Len--)
  {
	USART_SendData(USART2,*data++);
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) !=SET); //XE
	USART_ClearFlag(USART2,USART_FLAG_TC);
  }
}

//发送字符串
void USART_Send_Str(USART_TypeDef* USARTx,u8 *str)
{
  u8 i=0;
  while(str[i]!='\0')
  {
	USART_Send(str[i]);
	i++;
  }
}
