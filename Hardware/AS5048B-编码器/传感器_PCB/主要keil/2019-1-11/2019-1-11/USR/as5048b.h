#ifndef __AS5048_H
#define __AS5048_H
#include "pbdata.h"



#define AS5048B		255

u8 AS5048B_ReadOneByte(u8 add,u8 ReadAddr);							//ָ����ַ��ȡһ���ֽ�
u32 AS5048B_ReadLenByte(u8 ReadAddr,u8 Len);					//ָ����ַ��ʼ��ȡָ����������
void AS5048B_Read(u8 ReadAddr,u8 *pBuffer,u8 NumToRead);   	//��ָ����ַ��ʼ����ָ�����ȵ�����

u8 AS5048B_Check(void);  //�������
void AS5048B_Init(void); //��ʼ��IIC
#endif
