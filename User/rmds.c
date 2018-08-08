
#include "includes.h"
RMDS_Frame_TypeDef RMDS_Frame_TX;
RMDS_Data rmds_data;
USART_Data_TypeDef USART_Data;

/*******************************************************************************
								�������������ݺ���
********************************************************************************/
void RMDS_Send_Frame(RMDS_Frame_TypeDef rmds_frame)
{
  u8 send_buf[10];
  u8 i;
  send_buf[0] = rmds_frame.Frame_Head;
  send_buf[1] = rmds_frame.RMDS_ID;
  for(i=0;i<8;i++)
		send_buf[i+2] = RMDS_Frame_TX.Data[i];   
  //USART3����
  USART_Send_Assign(send_buf,10);
  Delay_ms(2);
}
/*******************************************************************************
								��������λ����
********************************************************************************/
void RMDS_Reset(u16 rmds_id,u8 cmd)
{//��������,���IDΪ0��㲥��ָ��
  u8 i;
  for(i=0;i<8;i++)
		RMDS_Frame_TX.Data[i] = 0X55;
  RMDS_Frame_TX.Frame_Head = RMDS_FRAME_HEAD;
	//�㲥485_ID = 0x00
  RMDS_Frame_TX.RMDS_ID = (rmds_id<<4) | cmd;
  
  RMDS_Send_Frame(RMDS_Frame_TX);
}
/*******************************************************************************
								������ģʽ�ı亯��
********************************************************************************/
void RMDS_Mode_Change_Send(u16 rmds_id,u8 mode)
{//�ı�ģʽ
  u8 i;
  for(i=0;i<8;i++)
	RMDS_Frame_TX.Data[i] = 0X55;
  RMDS_Frame_TX.Frame_Head = RMDS_FRAME_HEAD;
  RMDS_Frame_TX.RMDS_ID = (rmds_id<<4) | 1;
  RMDS_Frame_TX.Data[0] = mode;
  
  RMDS_Send_Frame(RMDS_Frame_TX);
}
/*******************************************************************************
								�������ٶ�λ��ģʽ����
********************************************************************************/
void RMDS_Vel_Pos_Send_Arg(u16 rmds_id,u16 pwm,s16 vel,s32 pos)//vel�ĵ�λ��RPM
{//�ٶ�λ��ģʽ�·��Ͳ���
  RMDS_Frame_TX.Frame_Head = RMDS_FRAME_HEAD;
  RMDS_Frame_TX.RMDS_ID = (rmds_id<<4) | 6;//�������
  RMDS_Frame_TX.Data[0] = (u8)((pwm>>8) & 0xff);
  RMDS_Frame_TX.Data[1] = (u8)(pwm & 0xff);
  RMDS_Frame_TX.Data[2] = (u8)((vel>>8) & 0xff);
  RMDS_Frame_TX.Data[3] = (u8)(vel & 0xff);
  RMDS_Frame_TX.Data[4] = (u8)((pos>>24) & 0xff);
  RMDS_Frame_TX.Data[5] = (u8)((pos>>16) & 0xff);
  RMDS_Frame_TX.Data[6] = (u8)((pos>>8) & 0xff);
  RMDS_Frame_TX.Data[7] = (u8)(pos & 0xff);
  
  RMDS_Send_Frame(RMDS_Frame_TX);
}
/*******************************************************************************
								�������ٶ�ģʽ����
********************************************************************************/
void RMDS_Vel_Send_Arg(u16 rmds_id,u16 pwm,s16 vel)//vel�ĵ�λ��RPM
{//�ٶ�ģʽ�·��Ͳ���
  RMDS_Frame_TX.Frame_Head = RMDS_FRAME_HEAD;
  RMDS_Frame_TX.RMDS_ID = (rmds_id<<4) | 4;//�������
  RMDS_Frame_TX.Data[0] = (u8)((pwm>>8) & 0xff);
  RMDS_Frame_TX.Data[1] = (u8)(pwm & 0xff);
  RMDS_Frame_TX.Data[2] = (u8)((vel>>8) & 0xff);
  RMDS_Frame_TX.Data[3] = (u8)(vel & 0xff);
  RMDS_Frame_TX.Data[4] = 0x55;
  RMDS_Frame_TX.Data[5] = 0x55;
  RMDS_Frame_TX.Data[6] = 0x55;
  RMDS_Frame_TX.Data[7] = 0x55;
  
  RMDS_Send_Frame(RMDS_Frame_TX);
}
/*******************************************************************************
							��ȡ���������в�������
********************************************************************************/
void RMDS_Get_Status(u8 rmds_id)
{//��ȡ��������в���
  u8 i;
  for(i=0;i<8;i++)
		RMDS_Frame_TX.Data[i] = 0X55;
  RMDS_Frame_TX.Frame_Head = RMDS_FRAME_HEAD;
  RMDS_Frame_TX.RMDS_ID = (rmds_id<<4) | 0X0A;
  RMDS_Frame_TX.Data[0] = 0X01;

  RMDS_Send_Frame(RMDS_Frame_TX);
}
/*******************************************************************************
							ping����������
********************************************************************************/
void RMDS_Ping_Device(u8 rmds_id)
{
  u8 i;
  for(i=0;i<8;i++)
	RMDS_Frame_TX.Data[i] = 0X55;
  RMDS_Frame_TX.Frame_Head = RMDS_FRAME_HEAD;
  RMDS_Frame_TX.RMDS_ID = (rmds_id<<4) | 0X0F;
  
  RMDS_Send_Frame(RMDS_Frame_TX);
}
/*******************************************************************************
							��������ʼ������
********************************************************************************/
void RMDS_Init(u8 mode)
{//��ʼ��
  RMDS_Reset(0,RMDS_RESET_CMD);
  Delay_us(30);
  RMDS_Reset(0,RMDS_RESET_CMD);
  Delay_us(30);
  RMDS_Reset(0,RMDS_RESET_CMD);
  Delay_ms(500);
  RMDS_Mode_Change_Send(0,mode);	
}

void RMDS_USART_Data_RX_Handler(u16 usart_data)
{
  int Device_ID;
  int i=0;/////////////////////////////////////////////////

  USART_Data.Data[USART_Data.Pointer]=usart_data;	
  USART_Data.Pointer++;
  if(USART_Data.Pointer>9)
  {//������һ֡����
	if((USART_Data.Data[0] == 0x48) && (USART_Data.Data[1] & 0x0B) == 0x0B)//˵���Ƿ��صĲ�������
	{
	  Device_ID = USART_Data.Data[1]>>4;
	  Motion_Data.Status.Cur[Device_ID-1]  = (USART_Data.Data[2]<<8)|USART_Data.Data[3];
	  Motion_Data.Status.Motor_Ang_Vel[Device_ID-1][0] = (USART_Data.Data[4]<<8)|USART_Data.Data[5];
	  Motion_Data.Status.Pos[Device_ID-1] = (USART_Data.Data[6]<<24)|(USART_Data.Data[7]<<16)|(USART_Data.Data[8]<<8)|USART_Data.Data[9];

	  for(i=AVERAGE_NUM-1;i>0;i--)//�����˲�
	  	Motion_Data.Status.Motor_Ang_Vel[Device_ID-1][i] = Motion_Data.Status.Motor_Ang_Vel[Device_ID-1][i-1];
	}
	USART_Data.Pointer=0;
  }
  if(USART_Data.Data[0] != 0x48)
	USART_Data.Pointer = 0;
}

