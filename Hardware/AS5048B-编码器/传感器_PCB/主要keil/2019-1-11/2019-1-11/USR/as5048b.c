#include "pbdata.h"



//��ʼ��IIC�ӿ�
void AS5048B_Init(void)
{
	IIC_Init();
}

//��AS5048Bָ����ַ����һ������
//ReadAddr:��ʼ�����ĵ�ַ  
//����ֵ  :����������
u8 AS5048B_ReadOneByte(u8 add,u8 ReadAddr)
{				  
	u8 rdata=0;		  	    																 
    IIC_Start();  
    IIC_Send_Byte(add);   //����������ַ	   
	IIC_Wait_Ack();

    IIC_Send_Byte(ReadAddr);   //���Ͷ�ȡ���ݵĵ�ַ
	IIC_Wait_Ack();

	  IIC_Start();  	 	   
	  IIC_Send_Byte((add|0x01));           //�������ģʽ			   
	IIC_Wait_Ack();	

    rdata=IIC_Read_Byte(0);		   
    IIC_Stop();//����һ��ֹͣ����	    
	return rdata;
}

