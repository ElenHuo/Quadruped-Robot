#ifndef __AS5048_H
#define __AS5048_H
#include "pbdata.h"



#define AS5048B		255

u8 AS5048B_ReadOneByte(u8 add,u8 ReadAddr);							//指定地址读取一个字节
u32 AS5048B_ReadLenByte(u8 ReadAddr,u8 Len);					//指定地址开始读取指定长度数据
void AS5048B_Read(u8 ReadAddr,u8 *pBuffer,u8 NumToRead);   	//从指定地址开始读出指定长度的数据

u8 AS5048B_Check(void);  //检查器件
void AS5048B_Init(void); //初始化IIC
#endif
