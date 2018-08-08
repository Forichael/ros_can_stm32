#include "time.h"

void Timer3_Init(u16 arr,u16 psc)
{	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);////////////////////////////
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //ʱ��ʹ��
	//��ʱ����ʱʱ��T���㹫ʽ��T=(TIM_Period+1)*(TIM_Prescaler+1)/TIMxCLK=(3600*10/72M)s=0.0005s����2KƵ��
	TIM_TimeBaseStructure.TIM_Period = arr;//�Զ���װ��ֵ��ȡֵ������0x0000~0xFFFF֮��
	TIM_TimeBaseStructure.TIM_Prescaler =psc;//Ԥ��Ƶֵ��+1Ϊ��Ƶϵ����ȡֵ������0x0000~0xFFFF֮��							 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 				//ʱ�ӷָ�
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	//���ϼ���ģʽ	 
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);//����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ 
        TIM_ITConfig(  //ʹ�ܻ���ʧ��ָ����TIM�ж�
                     TIM3, 
                     TIM_IT_Update,// |   //TIM �ж�Դ
                       //TIM_IT_Trigger ,  //TIM �����ж�Դ 
                       ENABLE  //ʹ��
                         );
        
	TIM_ARRPreloadConfig(TIM3, ENABLE);               	//ʹ��TIMx��ARR�ϵ�Ԥװ�ؼĴ���   
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM2�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;  //�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���							 
	TIM_Cmd(TIM3, ENABLE);								//ʹ�ܶ�ʱ��	
}

void TIM3_IRQHandler(void)   //TIM5�ж�
{ 	 
  if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //���ָ����TIM�жϷ������:TIM �ж�Դ 
  {
	Motion_Data.Status.Feedback_Flag = true;
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);  //���TIMx���жϴ�����λ:TIM �ж�Դ //��ʱ20US
  }
  TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
}
