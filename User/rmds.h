#ifndef _RMDS_H_
#define _RMDS_H_
#include "includes.h"

//RS485是0X48
//RS232是0X23
//#define AVERAGE_NUM							25

#define RMDS_FRAME_HEAD 				0X48 //RMDS驱动器帧头定义
//下面的都是广播的
#define RMDS_RESET_CMD					0X00
#define RMDS_MODE_CMD						0x01

#define RMDS_OPEN_MODE					0x01				//开环模式
#define RMDS_CUR_MODE						0x02				//电流模式
#define RMDS_VEL_MODE						0x03				//速度模式
#define RMDS_POS_MODE						0x04				//位置模式
#define RMDS_VEL_POS_MODE				0x05				//速度位置模式
#define RMDS_CUR_VEL_MODE				0x06				//电流速度模式
#define RMDS_CUR_POS_MODE				0x07				//电流位置模式
#define RMDS_CUR_VEL_POS_MODE		0x08				//电流速度位置模式

#define MAX_POSITIVE_POS                 1073741820       //正向最大值
#define MAX_NAGETIVE_POS                -1073741820      //反向最大值


typedef struct
{//共十个字节的长度
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
