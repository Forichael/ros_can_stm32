#ifndef _RMDS_H_
#define _RMDS_H_
#include "includes.h"

//RS485��0X48
//RS232��0X23
//#define AVERAGE_NUM							25

#define RMDS_FRAME_HEAD 				0X48 //RMDS������֡ͷ����
//����Ķ��ǹ㲥��
#define RMDS_RESET_CMD					0X00
#define RMDS_MODE_CMD						0x01

#define RMDS_OPEN_MODE					0x01				//����ģʽ
#define RMDS_CUR_MODE						0x02				//����ģʽ
#define RMDS_VEL_MODE						0x03				//�ٶ�ģʽ
#define RMDS_POS_MODE						0x04				//λ��ģʽ
#define RMDS_VEL_POS_MODE				0x05				//�ٶ�λ��ģʽ
#define RMDS_CUR_VEL_MODE				0x06				//�����ٶ�ģʽ
#define RMDS_CUR_POS_MODE				0x07				//����λ��ģʽ
#define RMDS_CUR_VEL_POS_MODE		0x08				//�����ٶ�λ��ģʽ

#define MAX_POSITIVE_POS                 1073741820       //�������ֵ
#define MAX_NAGETIVE_POS                -1073741820      //�������ֵ


typedef struct
{//��ʮ���ֽڵĳ���
	u8 Frame_Head;
	u8 RMDS_ID;
	u8 Data[8];
}RMDS_Frame_TypeDef;

typedef struct
{
	u8    Data[32];
  u8    Pointer;
  u8 		Descrip;
}RMDS_Data;

extern RMDS_Data rmds_data;



void RMDS_Init(u8 mode);
void RMDS_Get_Status(u8 rmds_id);
void RMDS_Ping_Device(u8 rmds_id);
void RMDS_Reset(u16 rmds_id,u8 cmd);
void RMDS_Mode_Change_Send(u16 rmds_id,u8 mode);
void RMDS_Send_Frame(RMDS_Frame_TypeDef rmds_frame);
void RMDS_Vel_Pos_Send_Arg(u16 rmds_id,u16 pwm,s16 vel,s32 pos);
void RMDS_Vel_Send_Arg(u16 rmds_id,u16 pwm,s16 vel);
void RMDS_USART_Data_RX_Handler(u16 usart_data);

#endif
