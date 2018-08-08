#ifndef __RMDSCAN_H__
#define __RMDSCAN_H__

#define RMDS_CAN_OPEN_MODE					0x01				//����ģʽ
#define RMDS_CAN_CUR_MODE						0x02				//����ģʽ
#define RMDS_CAN_VEL_MODE						0x03				//�ٶ�ģʽ
#define RMDS_CAN_POS_MODE						0x04				//λ��ģʽ
#define RMDS_CAN_VEL_POS_MODE				0x05				//�ٶ�λ��ģʽ
#define RMDS_CAN_CUR_VEL_MODE				0x06				//�����ٶ�ģʽ
#define RMDS_CAN_CUR_POS_MODE				0x07				//����λ��ģʽ
#define RMDS_CAN_CUR_VEL_POS_MODE		0x08				//�����ٶ�λ��ģʽ

#define MAX_POSITIVE_POS                 1073741820       //�������ֵ
#define MAX_NAGETIVE_POS                -1073741820      //�������ֵ

short short_abs(short x);

void RMDS_CAN1_Config_Init(void);
void RMDS_CAN_Init(void);

void RMDS_CAN_Reset(unsigned char Group,unsigned char Number);
void RMDS_CAN_Mode_Config(unsigned char Group,unsigned char Number,unsigned char Mode);
void RMDS_CAN_Open_Mode(unsigned char Group,unsigned char Number,short Temp_PWM);
void RMDS_CAN_Cur_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Current);
void RMDS_CAN_Vel_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Velocity);
void RMDS_CAN_Pos_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,long Temp_Position);
void RMDS_CAN_Vel_Pos_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Velocity,long Temp_Position);
void RMDS_CAN_Cur_Vel_Mode(unsigned char Group,unsigned char Number,short Temp_Current,short Temp_Velocity);
void RMDS_CAN_Cur_Pos_Mode(unsigned char Group,unsigned char Number,short Temp_Current,long Temp_Position);
void RMDS_CAN_Cur_Vel_Pos_Mode(unsigned char Group,unsigned char Number,short Temp_Current,short Temp_Velocity,long Temp_Position);

void RMDS_CAN_Config(unsigned char Group,unsigned char Number,unsigned char Temp_Time1,unsigned char Temp_Time2);
void RMDS_CAN_Data_Feedback(void);

void USB_HP_CAN1_TX_IRQHandler(void);
void USB_LP_CAN1_RX0_IRQHandler(void);

void CAN_NVIC_Configuration(void);
void CAN_GPIO_Config(void);
void CAN_INIT(void);

#endif
