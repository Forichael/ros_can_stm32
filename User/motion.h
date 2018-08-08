#ifndef __MOTION_H
#define __MOTION_H

#define AVERAGE_NUM	25
#include "includes.h"

typedef struct
{
	bool Update_Flag;			//�˶�״̬���±�־λ
	bool Enable_Flag;			//�˶�ʹ�ܱ�־λ
	s16  Motor_Ang_Vel[3];		//ת�ٿ��Ƶ�
  u16 PWM_Limit;
}Control_Data_TypDef;			//�˶���������

typedef struct
{
	s16 Cur[3];
	s16 Motor_Ang_Vel[3][AVERAGE_NUM];
	s32 Pos[3];
	s16 Motor_Ang_Vel_Average[3];			//�����ı�־λ
	bool Feedback_Flag;
}Status_Data_TypDef;			//�˶�״̬����

typedef struct
{
	Control_Data_TypDef		Control;
	Status_Data_TypDef		Status;
}Motion_Data_Typdef;

extern Motion_Data_Typdef Motion_Data;


void Motion_Control_Data_Time_Out_Check_Handler(void);	//���ʱ��⺯��
void Motion_Control_Obj_Handler(u8 *param);
void Motion_Data_Init(void);
void Motion_Control_Data_Send(void);
void Motor_Ang_Vel_Average_Calculate(s8 wheel);

void Motion_debug_test(void);

#endif
