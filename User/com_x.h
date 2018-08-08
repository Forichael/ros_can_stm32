#ifndef _COM_X_H
#define	_COM_X_H

#include "includes.h"

#define	DINT()										__disable_irq()
#define	EINT()										__enable_irq()

#define USARTzTxBufferSize   			1024
#define USARTz										USART3
#define USARTz_GPIO								GPIOB
#define USARTz_CLK								RCC_APB1Periph_USART3
#define USARTz_GPIO_CLK						RCC_APB2Periph_GPIOB
#define USARTz_RxPin							GPIO_Pin_11
#define USARTz_TxPin							GPIO_Pin_10
#define USARTz_IRQn								USART3_IRQn
#define USARTz_DR_Base						(uint32_t)(&USART3->DR)
//#define USARTz_DR_Base						USART3_BASE+0x04
#define USARTz_Tx_DMA_Channe			DMA1_Channel2
#define USARTz_Tx_DMA_FLAG				DMA1_FLAG_GL2 //DMA1_FLAG_TC2|DMA1_FLAG_TE2
#define UASRTz_TX_DMA_IRQ					DMA1_Channel2_IRQn

#define USARTz_Rx_DMA_Channe			DMA1_Channel3
#define USARTz_Rx_DMA_FLAG				DMA1_FLAG_GL3 //DMA1_FLAG_TC3 |DMA1_FLAG_TE3
#define UASRTz_RX_DMA_IRQ					DMA1_Channel3_IRQn 

//浮点数与HEX快速获取
typedef union{
		s16 uv;
		uint8_t cv[2];
}s16_union;

typedef union{
		s32 uv;
		uint8_t cv[4];
}s32_union;

typedef struct{
		s16_union Motor_Ang_Vel_Average[3];
		s32_union Pos[3];
}send_data;

typedef struct{
		s16_union Motor_Ang_Vel[3];
}rcv_data;

extern send_data	com_x_send_data;//数据发送
extern rcv_data	com_x_rcv_data;//数据接收
extern u16 flag;
extern s16 debug_A_Wheel;
extern s16 debug_B_Wheel;
extern s16 debug_C_Wheel;

void com_x_usart_config(void);
void com_x_usart_gpio_config(void);
void com_x_uasrt_nvic_config(void);
void com_x_usart_dma_config(void);
void com_x_usart_dma_tx_over(void);
void com_x_usart_dma_start_tx(uint8_t size);
void com_x_usart_dma_read(void);//DMA 读串口数据 在串口的空闲中断中调用，或者重写该函数，放到其他地方调用
void com_x_usart_rcc_config(void);
//****************外部调用函数************************
void com_x_usart_Init(void);//初始化
int8_t get_data_analyze(uint8_t	*pdata);//接收数据分析
void data_pack(void);//数据打包并发送

#endif
