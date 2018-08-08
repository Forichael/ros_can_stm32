
/*********************************************
STM32F103 串口通信


					PA9   USART1_TX    
					PA10  USART1_RX

					PB10	USART3_TX
					PB11	USART3_RX
					
					PA2   USART2_Tx
					PA3   USART2_Rx
					
*********************************************/
/* Includes ------------------------------------------------------------------*/
#include "includes.h"

s32 abs_s32(s32 i)
{    
  return (i < 0 ? -i : i);
}
short temp_pwm = 0;
/**********************************************************************************************************************/
int main(void)
{
	SystemInit();
	Delay_ms(2000);
	delay_init(); 
	Usart3_Init();
	Motion_Data_Init();
	RMDS_CAN1_Config_Init();
	RMDS_CAN_Init();
	
	RTC_Init();
	Timer3_Init(2500,719);//周期提交传感器数据到ROS（2500，719）-25ms，40hz

	
	
	while (1)
	{
		//Motion_debug_test();
		Motion_Control_Data_Send();
		//Motion_Status_Data_Read();
		//Motor_Ang_Vel_Average_Calculate();

		if(Motion_Data.Status.Feedback_Flag == true)
		{
			//data_pack();	        Delay_us(10);	
			USART3_Data_Pack();			//Delay_us(10);
			//反馈完成置为false
			Motion_Data.Status.Feedback_Flag = false;
		}
	}
	
		
}





#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
		/* User can add his own implementation to report the file name and line number,
			 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

		/* Infinite loop */
		while (1)
		{}
}

#endif

/**
  * @}
  */ 



