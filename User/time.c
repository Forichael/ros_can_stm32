#include "time.h"

void Timer3_Init(u16 arr,u16 psc)
{	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);////////////////////////////
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //时钟使能
	//定时器定时时间T计算公式：T=(TIM_Period+1)*(TIM_Prescaler+1)/TIMxCLK=(3600*10/72M)s=0.0005s，即2K频率
	TIM_TimeBaseStructure.TIM_Period = arr;//自动重装载值，取值必须在0x0000~0xFFFF之间
	TIM_TimeBaseStructure.TIM_Prescaler =psc;//预分频值，+1为分频系数，取值必须在0x0000~0xFFFF之间							 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 				//时钟分割
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	//向上计数模式	 
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);//根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位 
        TIM_ITConfig(  //使能或者失能指定的TIM中断
                     TIM3, 
                     TIM_IT_Update,// |   //TIM 中断源
                       //TIM_IT_Trigger ,  //TIM 触发中断源 
                       ENABLE  //使能
                         );
        
	TIM_ARRPreloadConfig(TIM3, ENABLE);               	//使能TIMx在ARR上的预装载寄存器   
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM2中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器							 
	TIM_Cmd(TIM3, ENABLE);								//使能定时器	
}

void TIM3_IRQHandler(void)   //TIM5中断
{ 	 
  if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源 
  {
	Motion_Data.Status.Feedback_Flag = true;
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);  //清除TIMx的中断待处理位:TIM 中断源 //延时20US
  }
  TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
}
