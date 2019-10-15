/**
  ******************************************************************************
  * @file GPIO/IOToggle/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version  V3.0.0
  * @date  04/06/2009
  * @brief  Main Interrupt Service Routines.
  *         This file provides template for all exceptions handler and 
  *         peripherals interrupt service routine.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"	 
#include "stm32f10x_exti.h"
#include "stm32f10x_rcc.h"
#include "misc.h"
#include "pbdata.h"

void NMI_Handler(void)
{
}

u8 UART1_RX_BUF[8*ENG_NUM+2]={0};
u8 UART1_RX_CNT=0;
float resAngles[ENG_NUM];
void USART1_IRQHandler(void)
{
	u8 res;
   if(USART_GetITStatus(USART1,USART_IT_RXNE)!=RESET)
   {
		//USART_SendData(USART1,USART_ReceiveData(USART1));
		//while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);
		 res =USART_ReceiveData(USART1);
		if(UART1_RX_CNT<8*ENG_NUM+2)
		{
			UART1_RX_BUF[UART1_RX_CNT]=res;		//记录接收到的值
			UART1_RX_CNT++;					//接收数据增加1 
		} 
		if(res=='\n')
		{
			u8 i=0, j=0, dec=0, neg=0;
			float k=1.0;
			memset(resAngles, 0, sizeof(resAngles));
			while(i<UART1_RX_CNT)
			{
				if(UART1_RX_BUF[i]==' '){	j++;  dec=0;  neg=0;	k=1.0; }
				else if(UART1_RX_BUF[i]=='-')	neg=1;
				else if(UART1_RX_BUF[i]>='0'&&UART1_RX_BUF[i]<='9')	
				{
					if(neg)	resAngles[j]=-resAngles[j];
					if(dec){ resAngles[j]*=k;  k*=10.0; }
					resAngles[j]=UART1_RX_BUF[i]-'0' + resAngles[j]*10.0;
					resAngles[j]/=k;
					if(neg)	resAngles[j]=-resAngles[j];
				}
				else if(UART1_RX_BUF[i]=='.')	dec=1;
				i++;
			}
			memset(UART1_RX_BUF, 0, sizeof(UART1_RX_BUF));
			UART1_RX_CNT=0;
		}
   }
	 USART_ClearITPendingBit(USART1,USART_IT_RXNE);
}
/*
u8 UART1_RX_BUF[5*ENG_NUM]={0};
u8 UART1_RX_CNT=0;
float resAngles[ENG_NUM];
int flag;
void USART1_IRQHandler(void)
{
	u8 res;
   if(USART_GetITStatus(USART1,USART_IT_RXNE)!=RESET)
   {
		//USART_SendData(USART1,USART_ReceiveData(USART1));
		//while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);
		 res =USART_ReceiveData(USART1);
		if(UART1_RX_CNT<6*ENG_NUM)
		{
			UART1_RX_BUF[UART1_RX_CNT]=res;		//记录接收到的值
			UART1_RX_CNT++;					//接收数据增加1 
		} 
		if(res=='\n')
		{
			u8 i=0, j=0;
			memset(resAngles, 0, sizeof(resAngles));
			while(i<UART1_RX_CNT)
			{
				if(UART1_RX_BUF[i]!=' ')
					resAngles[j]=UART1_RX_BUF[i]-'0' + resAngles[j]*10;
				else	j++;
				i++;
			}
			memset(UART1_RX_BUF, 0, sizeof(UART1_RX_BUF));
			UART1_RX_CNT=0;
		}
   }
	 USART_ClearITPendingBit(USART1,USART_IT_RXNE);
}*/

u8 RS485_RX_BUF[128];  	//接收缓冲,最大128个字节
u8 RS485_RX_CNT=0;      //接收到的数据长度
void USART2_IRQHandler(void)
{
	u8 res;	   
	
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)//接收到数据
	{	 	 
	  res =USART_ReceiveData(USART2);//;读取接收到的数据USART2->DR
		if(RS485_RX_CNT<128)
		{
			RS485_RX_BUF[RS485_RX_CNT]=res;		//记录接收到的值
			RS485_RX_CNT++;					//接收数据增加1 
		} 
	} 
	USART_ClearITPendingBit(USART2,USART_IT_RXNE); 											 
} 
/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval : None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval : None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval : None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval : None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval : None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval : None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval : None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval : None
  */
void SysTick_Handler(void)
{
}

void TIM3_IRQHandler(void)
{
	

}
/****************************************************************************
* 名    称：void EXTI9_5_IRQHandler(void)
* 功    能：EXTI9-5中断处理程序
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：无 
****************************************************************************/
void EXTI9_5_IRQHandler(void)
{
}

/****************************************************************************
* 名    称：void EXTI1_IRQHandler(void)
* 功    能：EXTI2中断处理程序
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：无 
****************************************************************************/
void EXTI1_IRQHandler(void)
{
    
}

/****************************************************************************
* 名    称：void EXTI2_IRQHandler(void)
* 功    能：EXTI2中断处理程序
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：无 
****************************************************************************/
void EXTI2_IRQHandler(void)
{
    
}

/****************************************************************************
* 名    称：void EXTI3_IRQHandler(void)
* 功    能：EXTI3中断处理程序
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：无 
****************************************************************************/
void EXTI3_IRQHandler(void)
{
   
}
