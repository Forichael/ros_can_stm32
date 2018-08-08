#ifndef __INCLUDES_H
#define __INCLUDES_H

#include "stm32f10x_conf.h"
#include "stm32f10x.h"

#include "stdio.h"
#include "string.h"
#include "stdbool.h"

#include "motion.h"
#include "delay.h"
#include "systick.h"
#include "rtc.h"
#include "usart3.h"
#include "rmdsCan.h"
#include "time.h"


#define A_Wheel 		0
#define B_Wheel 		1
#define C_Wheel			2


s32 abs_s32(s32 i);

void Timer3_Init(u16 arr,u16 psc);

#endif
