#include "pbdata.h"
u8 out6[6]= {0};
u32 a[101]= {0},b[101]= {0};
int v1=1,v2=1;
int i;
u8 AS5048B_ADDR1,AS5048B_ADDR2,AS5048B_ADDR3,AS5048B_ADDR4,AS5048B_ADDR5,AS5048B_ADDR6,AS5048B_ADDR7,AS5048B_ADDR8;//����IIC������ַ
u8 buff1[6],buff2[6],buff3[6],buff4[6],buff5[6],buff6[6];



float MID_FILTER()
{
    //////////////////////////////////////////////////////////��ֵ�˲�ȡһ�ٸ�ֵ
    for(v1=1; v1<=100; v1++)
    {
        for(i = 4; i < 6; i++)  // AS5048Bͨ����������I2C�ӿڶ�ȡ����
        {
            //buffer5[i] = AS5048B_ReadOneByte(AS5048B_ADDR5,0xFA+i);//��0xFA�Զ�������ƼĴ�����ʼ������
            buff6[i] = AS5048B_ReadOneByte(AS5048B_ADDR6,0xFA+i);//��0xFA�Զ�������ƼĴ�����ʼ������
        }
        a[v1]=buff6[4];
        b[v2]=buff6[5];
        a[v1]+=a[v1-1];
        b[v2]+=b[v2-1];
        v2++;
        //printf("%d %d\r\n",buffer5[4],buffer5[5]);
    }
    out6[0]=a[100]/100;   //��100
    out6[1]=b[100]/100;
    out6[2]=a[100]/100;   //��100
    out6[3]=b[100]/100;
    buffer6[4]=out6[0];
    buffer6[5]=out6[1];
}