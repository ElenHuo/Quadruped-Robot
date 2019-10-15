#include "pbdata.h"



//初始化IIC接口
void AS5048B_Init(void)
{
	IIC_Init();
}

//在AS5048B指定地址读出一个数据
//ReadAddr:开始读数的地址  
//返回值  :读到的数据
u8 AS5048B_ReadOneByte(u8 add,u8 ReadAddr)
{				  
	u8 rdata=0;		  	    																 
    IIC_Start();  
    IIC_Send_Byte(add);   //发送器件地址	   
	IIC_Wait_Ack();

    IIC_Send_Byte(ReadAddr);   //发送读取数据的地址
	IIC_Wait_Ack();

	  IIC_Start();  	 	   
	  IIC_Send_Byte((add|0x01));           //进入接收模式			   
	IIC_Wait_Ack();	

    rdata=IIC_Read_Byte(0);		   
    IIC_Stop();//产生一个停止条件	    
	return rdata;
}

