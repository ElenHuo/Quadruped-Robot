#ifndef _pbdata_H
#define _pbdata_H

#include "stm32f10x.h"
#include "misc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_usart.h"
#include "stdio.h"
#include "string.h"
#include "I2C.h"
#include "AT24Cxx.h"
#include "as5048b.h" 
#include "sys.h"
#include "pd.h"

//定义变量

extern u8 dt;
extern u8 RS485_RX_BUF[128]; 		//接收缓冲,最大64个字节
extern u8 RS485_RX_CNT;   			//接收到的数据长度
extern float resAngles[ENG_NUM];

extern int flag;

//定义函数

void RCC_HSE_Configuration(void);
void delay_init(u8 SYSCLK);
void delay_ms(u16 nms);
void delay_us(u32 nus);
int fputc(int ch,FILE *f);

#endif
