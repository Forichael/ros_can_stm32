#include "includes.h"

unsigned int CAN_Time_Out = 0;

short short_abs(short x)
{
	return (x < 0 ? -x : x);
}


static void CAN_Delay_Us(unsigned int t)
{
	int i;
	for(i=0;i<t;i++)
	{
		int a=9;
		while(a--);
	}
}

void CAN_NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	
  NVIC_InitStructure.NVIC_IRQChannel = USB_HP_CAN1_TX_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
	
  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;  
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void CAN_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,  ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | 
                         RCC_APB2Periph_AFIO, ENABLE);
	
	GPIO_PinRemapConfig(GPIO_Remap1_CAN1,ENABLE);
         
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);	             //CAN_RX
        
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;	   
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);               //CAN_TX
	//重映射
	
}

void CAN_INIT(void)
{
	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
		//先De初始化
	CAN_DeInit(CAN1);
	CAN_StructInit(&CAN_InitStructure);
	
	//关闭时间触发模式
	CAN_InitStructure.CAN_TTCM = DISABLE;
	//关闭自动离线管理
	CAN_InitStructure.CAN_ABOM = DISABLE;
	//关闭自动唤醒模式
	CAN_InitStructure.CAN_AWUM = DISABLE;
	//禁止报文自动重传
	CAN_InitStructure.CAN_NART = DISABLE;
	//FIFO溢出时报文覆盖源文件
	CAN_InitStructure.CAN_RFLM = DISABLE;
	//报文发送优先级取决于ID号，本次只用了一个发送邮箱，关闭TXFP
	CAN_InitStructure.CAN_TXFP = ENABLE;		
	//工作模式（正常）
 	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
	
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1 = CAN_BS1_5tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_3tq;
	//波特率设置1024 KBPS
	CAN_InitStructure.CAN_Prescaler = 4;     //CAN BaudRate 36/(1+5+3)/4=1Mbps
	//初始化CAN
	CAN_Init(CAN1, &CAN_InitStructure);
	
		//屏蔽滤波（can接收才涉及）
	CAN_FilterInitStructure.CAN_FilterNumber = 0;//0号滤波器
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;//标识符屏蔽位模式
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;//32位寄存器
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;//标识符寄存器高x位(这里好像是0位)
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;//标识符寄存器低x位(这里好像是0位)
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;//屏蔽寄存器高x位
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;//屏蔽寄存器低x位
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;//过滤器将ID报文关联到FIFO0缓存区中，数据只能从这里导出
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;//过滤器使能
	CAN_FilterInit(&CAN_FilterInitStructure);//初始化过滤器

  CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//接收中断使能
  CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE);//发送中断使能
}

void RMDS_CAN1_Config_Init(void)
{
	CAN_GPIO_Config();//CAN管脚初始化
	CAN_NVIC_Configuration(); //CAN中断初始化   
	CAN_INIT();//CA初始化N模块	
}

unsigned char can_tx_success_flag=0;
void USB_HP_CAN1_TX_IRQHandler(void) //CAN TX
{
    if (CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET)
	{
	   CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
       can_tx_success_flag=1;
    }
}

/****************************************************************************************
                                       复位指令
Group   取值范围 0-7
Number  取值范围 0-15，其中Number==0时，为广播发送
*****************************************************************************************/
void RMDS_CAN_Reset(unsigned char Group,unsigned char Number)
{
	  unsigned short can_id = 0x000;
    CanTxMsg tx_message;
    
		//传送数据的帧设置
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
		//通过位运算符计算得出当前的组别跟编号
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
		//帧头为CAN_ID 数据位8位
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID
    
    tx_message.Data[0] = 0x55;
    tx_message.Data[1] = 0x55;
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    
		//flag置为0，传送成功的时候，flag变成1
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
    
		//最多等待100us
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}

/****************************************************************************************
                                     模式选择指令
Group   取值范围 0-7
Number  取值范围 0-15，其中Number==0时，为广播发送

Mode    取值范围

OpenLoop_Mode                       0x01
Current_Mode                        0x02
Velocity_Mode                       0x03
Position_Mode                       0x04
Velocity_Position_Mode              0x05
Current_Velocity_Mode               0x06
Current_Position_Mode               0x07
Current_Velocity_Position_Mode      0x08

					PS.发送方式跟上面一样
*****************************************************************************************/
void RMDS_CAN_Mode_Config(unsigned char Group,unsigned char Number,unsigned char Mode)
{
	  unsigned short can_id = 0x001;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID
    
    tx_message.Data[0] = Mode;
    tx_message.Data[1] = 0x55;
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
    
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}

/****************************************************************************************
                                   开环模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_pwm的取值范围如下：
-5000 ~ +5000，满值5000，其中temp_pwm = ±5000时，最大输出电压为电源电压

*****************************************************************************************/
void RMDS_CAN_Open_Mode(unsigned char Group,unsigned char Number,short Temp_PWM)
{
		unsigned short can_id = 0x002;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    tx_message.Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_PWM&0xff);
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
    
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}

/****************************************************************************************
                                   电流模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_pwm的取值范围如下：
0 ~ +5000，满值5000，其中temp_pwm = 5000时，最大输出电压为电源电压

temp_current的取值范围如下：
-32768 ~ +32767，单位mA

*****************************************************************************************/
void RMDS_CAN_Cur_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Current)
{
		unsigned short can_id = 0x003;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    if(Temp_PWM < 0)
    {
        Temp_PWM = short_abs(Temp_PWM);
    }
    
    tx_message.Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_PWM&0xff);
    tx_message.Data[2] = (unsigned char)((Temp_Current>>8)&0xff);
    tx_message.Data[3] = (unsigned char)(Temp_Current&0xff);
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
    
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}

/****************************************************************************************
                                   速度模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_pwm的取值范围如下：
0 ~ +5000，满值5000，其中temp_pwm = 5000时，最大输出电压为电源电压

temp_velocity的取值范围如下：
-32768 ~ +32767，单位RPM

*****************************************************************************************/
void RMDS_CAN_Vel_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Velocity)
{
	  unsigned short can_id = 0x004;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    if(Temp_PWM < 0)
    {
        Temp_PWM = short_abs(Temp_PWM);
    }
    
    tx_message.Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_PWM&0xff);
    tx_message.Data[2] = (unsigned char)((Temp_Velocity>>8)&0xff);
    tx_message.Data[3] = (unsigned char)(Temp_Velocity&0xff);
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
    
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}

/****************************************************************************************
                                   位置模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_pwm的取值范围如下：
0 ~ +5000，满值5000，其中temp_pwm = 5000时，最大输出电压为电源电压

temp_position的取值范围如下：
-2147483648~+2147483647，单位qc

*****************************************************************************************/
void RMDS_CAN_Pos_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,long Temp_Position)
{
	    unsigned short can_id = 0x005;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    if(Temp_PWM < 0)
    {
        Temp_PWM = short_abs(Temp_PWM);
    }
    
    tx_message.Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_PWM&0xff);
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = (unsigned char)((Temp_Position>>24)&0xff);
    tx_message.Data[5] = (unsigned char)((Temp_Position>>16)&0xff);
    tx_message.Data[6] = (unsigned char)((Temp_Position>>8)&0xff);
    tx_message.Data[7] = (unsigned char)(Temp_Position&0xff);
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
    
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}

/****************************************************************************************
                                  速度位置模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_pwm的取值范围如下：
0 ~ +5000，满值5000，其中temp_pwm = 5000时，最大输出电压为电源电压

temp_velocity的取值范围如下：
0 ~ +32767，单位RPM

temp_position的取值范围如下：
-2147483648~+2147483647，单位qc
*****************************************************************************************/
void RMDS_CAN_Vel_Pos_Mode(unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Velocity,long Temp_Position)
{
	    unsigned short can_id = 0x006;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    if(Temp_PWM < 0)
    {
        Temp_PWM = short_abs(Temp_PWM);
    }
    
    if(Temp_Velocity < 0)
    {
        Temp_Velocity = short_abs(Temp_Velocity);
    }
    
    tx_message.Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_PWM&0xff);
    tx_message.Data[2] = (unsigned char)((Temp_Velocity>>8)&0xff);
    tx_message.Data[3] = (unsigned char)(Temp_Velocity&0xff);
    tx_message.Data[4] = (unsigned char)((Temp_Position>>24)&0xff);
    tx_message.Data[5] = (unsigned char)((Temp_Position>>16)&0xff);
    tx_message.Data[6] = (unsigned char)((Temp_Position>>8)&0xff);
    tx_message.Data[7] = (unsigned char)(Temp_Position&0xff);
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
    
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}

/****************************************************************************************
                                  电流速度模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_current的取值范围如下：
0 ~ +32767，单位mA

temp_velocity的取值范围如下：
-32768 ~ +32767，单位RPM

*****************************************************************************************/
void RMDS_CAN_Cur_Vel_Mode(unsigned char Group,unsigned char Number,short Temp_Current,short Temp_Velocity)
{
	    unsigned short can_id = 0x007;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID
    
    if(Temp_Current < 0)
    {
        Temp_Current = short_abs(Temp_Current);
    }
    
    tx_message.Data[0] = (unsigned char)((Temp_Current>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_Current&0xff);
    tx_message.Data[2] = (unsigned char)((Temp_Velocity>>8)&0xff);
    tx_message.Data[3] = (unsigned char)(Temp_Velocity&0xff);
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
    
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}

/****************************************************************************************
                                  电流位置模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_current的取值范围如下：
0 ~ +32767，单位mA

temp_position的取值范围如下：
-2147483648~+2147483647，单位qc

*****************************************************************************************/
void RMDS_CAN_Cur_Pos_Mode(unsigned char Group,unsigned char Number,short Temp_Current,long Temp_Position)
{
	unsigned short can_id = 0x008;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID

    
    if(Temp_Current < 0)
    {
        Temp_Current = short_abs(Temp_Current);
    }
    
    tx_message.Data[0] = (unsigned char)((Temp_Current>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_Current&0xff);
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = (unsigned char)((Temp_Position>>24)&0xff);
    tx_message.Data[5] = (unsigned char)((Temp_Position>>16)&0xff);
    tx_message.Data[6] = (unsigned char)((Temp_Position>>8)&0xff);
    tx_message.Data[7] = (unsigned char)(Temp_Position&0xff);
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
    
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}

/****************************************************************************************
                                  电流速度位置模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_current的取值范围如下：
0 ~ +32767，单位mA

temp_velocity的取值范围如下：
0 ~ +32767，单位RPM

temp_position的取值范围如下：
-2147483648~+2147483647，单位qc

*****************************************************************************************/
void RMDS_CAN_Cur_Vel_Pos_Mode(unsigned char Group,unsigned char Number,short Temp_Current,short Temp_Velocity,long Temp_Position)
{
	    unsigned short can_id = 0x009;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;      //帧ID为传入参数的CAN_ID
    
    if(Temp_Current < 0)
    {
        Temp_Current = short_abs(Temp_Current);
    }
    
    if(Temp_Velocity < 0)
    {
        Temp_Velocity = short_abs(Temp_Velocity);
    }
    
    tx_message.Data[0] = (unsigned char)((Temp_Current>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(Temp_Current&0xff);
    tx_message.Data[2] = (unsigned char)((Temp_Velocity>>8)&0xff);
    tx_message.Data[3] = (unsigned char)(Temp_Velocity&0xff);
    tx_message.Data[4] = (unsigned char)((Temp_Position>>24)&0xff);
    tx_message.Data[5] = (unsigned char)((Temp_Position>>16)&0xff);
    tx_message.Data[6] = (unsigned char)((Temp_Position>>8)&0xff);
    tx_message.Data[7] = (unsigned char)(Temp_Position&0xff);
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
    
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}

/****************************************************************************************
                                      配置指令
Temp_Time1的取值范围: 0 ~ 255，为0时候，为关闭电流速度位置反馈功能
Temp_Time2的取值范围: 0 ~ 255，为0时候，为关闭限位信号反馈功能
*****************************************************************************************/
void RMDS_CAN_Config(unsigned char Group,unsigned char Number,unsigned char Temp_Time1,unsigned char Temp_Time2)
{
	unsigned short can_id = 0x00A;
    CanTxMsg tx_message;
    
    tx_message.IDE = CAN_ID_STD;    //标准帧
    tx_message.RTR = CAN_RTR_DATA;  //数据帧
    tx_message.DLC = 0x08;          //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.StdId = can_id;
    
    tx_message.Data[0] = Temp_Time1;
    tx_message.Data[1] = Temp_Time2;
    tx_message.Data[2] = 0x55;
    tx_message.Data[3] = 0x55;
    tx_message.Data[4] = 0x55;
    tx_message.Data[5] = 0x55;
    tx_message.Data[6] = 0x55;
    tx_message.Data[7] = 0x55;
    
    can_tx_success_flag = 0;
    CAN_Transmit(CAN1,&tx_message);
    
    CAN_Time_Out = 0;
    while(can_tx_success_flag == 0)
    {
        CAN_Delay_Us(1);
        CAN_Time_Out++;
        if(CAN_Time_Out>100)
        {
            break;
        }
    }
}


void RMDS_CAN_Init(void)
{
	//RMDS_CAN1_Config_Init();//CAN1初始化
	delay_ms(500);//刚开始要有足够的延时，确保驱动器已经初始化完成 
	RMDS_CAN_Reset(0,0);//对0组所有驱动器进行复位 
	delay_ms(500);//发送复位指令后的延时必须要有，等待驱动器再次初始化完成
	
	RMDS_CAN_Config(0,1,10,0);//1号驱动器配置为100ms传回一次数据
	delay_us(100);//此处延时为了不让传回数据时候4个不一起传
	RMDS_CAN_Config(0,2,10,0);
	delay_us(100);
	RMDS_CAN_Config(0,3,10,0);
	delay_us(100);
	
	RMDS_CAN_Mode_Config(0,0,RMDS_CAN_VEL_MODE);
	//RMDS_CAN_Mode_Config(0,0,RMDS_CAN_OPEN_MODE);
	delay_ms(500);
}

/*************************************************************************
                          USB_LP_CAN1_RX0_IRQHandler
描述：CAN1的接收中断函数
*************************************************************************/
void USB_LP_CAN1_RX0_IRQHandler(void) //CAN RX
{
	CanRxMsg rx_message;
  s8 i;
    if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
	{
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
        CAN_Receive(CAN1, CAN_FIFO0, &rx_message);
        
		//实际上这里是0x02B，这里默认是第0组别了
		//滑动滤波从第25个数据开始才准确，考虑到小车一开始动都那么快了，忽略不计前25的误差
		//实际上我在ROS上并没有用到这个速度返回数据，用的是pos数据
        if((rx_message.IDE == CAN_Id_Standard)&&(rx_message.IDE == CAN_RTR_Data)&&(rx_message.DLC == 8)) //标准帧、数据帧、数据长度为8
        {
            if(rx_message.StdId == 0x1B)
            {
                Motion_Data.Status.Cur[A_Wheel] = (rx_message.Data[0]<<8)|(rx_message.Data[1]);
                Motion_Data.Status.Motor_Ang_Vel[A_Wheel][0] = (rx_message.Data[2]<<8)|(rx_message.Data[3]);
                Motion_Data.Status.Pos[A_Wheel] = ((rx_message.Data[4]<<24)|(rx_message.Data[5]<<16)|(rx_message.Data[6]<<8)|(rx_message.Data[7]));
								for(i=AVERAGE_NUM-1;i>0;i--)//滑动滤波
									Motion_Data.Status.Motor_Ang_Vel[A_Wheel][i] = Motion_Data.Status.Motor_Ang_Vel[A_Wheel][i-1];
								Motor_Ang_Vel_Average_Calculate(A_Wheel);
            }
            else if(rx_message.StdId == 0x2B)
            {
                Motion_Data.Status.Cur[B_Wheel] = (rx_message.Data[0]<<8)|(rx_message.Data[1]);
                Motion_Data.Status.Motor_Ang_Vel[B_Wheel][0] = (rx_message.Data[2]<<8)|(rx_message.Data[3]);
                Motion_Data.Status.Pos[B_Wheel] = ((rx_message.Data[4]<<24)|(rx_message.Data[5]<<16)|(rx_message.Data[6]<<8)|(rx_message.Data[7]));
								for(i=AVERAGE_NUM-1;i>0;i--)//滑动滤波
									Motion_Data.Status.Motor_Ang_Vel[B_Wheel][i] = Motion_Data.Status.Motor_Ang_Vel[B_Wheel][i-1];
								Motor_Ang_Vel_Average_Calculate(B_Wheel);
            }
            else if(rx_message.StdId == 0x3B)
            {
                Motion_Data.Status.Cur[C_Wheel] = (rx_message.Data[0]<<8)|(rx_message.Data[1]);
                Motion_Data.Status.Motor_Ang_Vel[C_Wheel][0] = (rx_message.Data[2]<<8)|(rx_message.Data[3]);
                Motion_Data.Status.Pos[C_Wheel] = ((rx_message.Data[4]<<24)|(rx_message.Data[5]<<16)|(rx_message.Data[6]<<8)|(rx_message.Data[7]));
								for(i=AVERAGE_NUM-1;i>0;i--)//滑动滤波
									Motion_Data.Status.Motor_Ang_Vel[C_Wheel][i] = Motion_Data.Status.Motor_Ang_Vel[C_Wheel][i-1];
								Motor_Ang_Vel_Average_Calculate(C_Wheel);
            }

        }
                
    }
}
