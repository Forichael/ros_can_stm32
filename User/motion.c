#include "includes.h"

Motion_Data_Typdef Motion_Data;

/*******************************************************************************
	�˶����ݳ�ʼ��
********************************************************************************/
void Motion_Data_Init(void)
{//�˶����ݳ�ʼ��
  Motion_Data.Control.Update_Flag = false;
	Motion_Data.Control.Enable_Flag = false;
	Motion_Data.Control.PWM_Limit = 5000;
	
	Motion_Data.Control.Motor_Ang_Vel[A_Wheel]  = 0;
	Motion_Data.Control.Motor_Ang_Vel[B_Wheel]  = 0;
  Motion_Data.Control.Motor_Ang_Vel[C_Wheel]  = 0;
}
/*******************************************************************************
	��λ��ָ�ʱ��⣨��RTC�ж��б����ã�
********************************************************************************/
void Motion_Control_Data_Time_Out_Check_Handler(void)
{
  if (!Motion_Data.Control.Update_Flag)//1sʱ����û�յ�������ֹͣ�˶�
  {
    Motion_Data.Control.Motor_Ang_Vel[A_Wheel]  = 0;
		Motion_Data.Control.Motor_Ang_Vel[B_Wheel]  = 0;
    Motion_Data.Control.Motor_Ang_Vel[C_Wheel]  = 0;
  }
  Motion_Data.Control.Update_Flag = false;
}
/*******************************************************************************
	�˶����ƶ���Ĵ��������ros�������ݺ���ܱ����ã�
********************************************************************************/
void Motion_Control_Obj_Handler(u8 *param)
{
	//�����ǽ�����λ�������������ݣ����������ӵ��ٶ�ֵ����Motion_Data��
	Motion_Data.Control.Motor_Ang_Vel[A_Wheel] =  (param[2]<<8 | param[3]);
	Motion_Data.Control.Motor_Ang_Vel[B_Wheel] =  (param[4]<<8 | param[5]);
	Motion_Data.Control.Motor_Ang_Vel[C_Wheel] =  (param[6]<<8 | param[7]);

	Motion_Data.Control.Update_Flag = true;
}

void Motion_debug_test(void)
{
	Motion_Data.Control.Motor_Ang_Vel[A_Wheel] =  1000;
	Motion_Data.Control.Motor_Ang_Vel[B_Wheel] =  1000;
	Motion_Data.Control.Motor_Ang_Vel[C_Wheel] =  1000;
	Motion_Data.Control.Update_Flag = true;
}
/*******************************************************************************
	�˶��������ݵķ��ͣ����͸���������
********************************************************************************/
void Motion_Control_Data_Send(void)
{
		RMDS_CAN_Vel_Mode(0,1,Motion_Data.Control.PWM_Limit,Motion_Data.Control.Motor_Ang_Vel[A_Wheel]);
		RMDS_CAN_Vel_Mode(0,2,Motion_Data.Control.PWM_Limit,Motion_Data.Control.Motor_Ang_Vel[B_Wheel]);
		RMDS_CAN_Vel_Mode(0,3,Motion_Data.Control.PWM_Limit,Motion_Data.Control.Motor_Ang_Vel[C_Wheel]);
		
		Delay_ms(1);
	
	  //��ֹ���
	  if((abs_s32(Motion_Data.Status.Pos[A_Wheel]) >= MAX_POSITIVE_POS)  || 
			 (abs_s32(Motion_Data.Status.Pos[B_Wheel]) >= MAX_POSITIVE_POS)  ||
			 (abs_s32(Motion_Data.Status.Pos[C_Wheel]) >= MAX_POSITIVE_POS))
	  {
		Motion_Data.Status.Motor_Ang_Vel[A_Wheel][0] = 0;
		Motion_Data.Status.Motor_Ang_Vel[B_Wheel][0] = 0;
		Motion_Data.Status.Motor_Ang_Vel[C_Wheel][0] = 0;
		
		//RMDS_Init(RMDS_VEL_MODE);
		//�����Ϊ��CAN����
		RMDS_CAN_Mode_Config(0,0,RMDS_CAN_VEL_MODE);
		Delay_ms(500);
	  }
}

/*******************************************************************************
	�˶����ٶȵ�ƽ��ֵ����
********************************************************************************/
void Motor_Ang_Vel_Average_Calculate(s8 wheel)
{
  u8 i;
	s32 buf;
	if(wheel == A_Wheel)
	{
		buf = 0;
		for(i=0;i<AVERAGE_NUM;i++)
		{
			buf +=Motion_Data.Status.Motor_Ang_Vel[A_Wheel][i];
		}
		Motion_Data.Status.Motor_Ang_Vel_Average[A_Wheel] =buf/(AVERAGE_NUM);
	}
	else if(wheel == B_Wheel)
	{
		buf = 0;
		for(i=0;i<AVERAGE_NUM;i++)
		{
			buf +=Motion_Data.Status.Motor_Ang_Vel[B_Wheel][i];
		}
		Motion_Data.Status.Motor_Ang_Vel_Average[B_Wheel] =buf/(AVERAGE_NUM);
	}
	else if(wheel == C_Wheel)
	{
		buf = 0;
		for(i=0;i<AVERAGE_NUM;i++)
		{
			buf +=Motion_Data.Status.Motor_Ang_Vel[C_Wheel][i];
		}
		Motion_Data.Status.Motor_Ang_Vel_Average[C_Wheel] =buf/(AVERAGE_NUM);
	}
}
