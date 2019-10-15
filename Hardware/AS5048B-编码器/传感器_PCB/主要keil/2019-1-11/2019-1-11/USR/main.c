#include "pbdata.h"
#include "math.h"
#define pi 3.14159265

void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void uart_init(u32 pclk2,u32 bound);
void LED_Init(void);
void rank1(void);	//传感器1中值滤波排序
void rank2(void);	//传感器2中值滤波排序
void rank3(void);	//传感器3中值滤波排序
void rank4(void);	//传感器4中值滤波排序
void rank5(void);	//传感器5中值滤波排序
void rank6(void);	//传感器6中值滤波排序
void MID_FILTER(void); //中值滤波前一百个值

//////////////////////////////////////////////////////
//                                                  //
//                 传感器读取                       //
//                                                  //
//////////////////////////////////////////////////////
u8 AS5048B_ADDR1,AS5048B_ADDR2,AS5048B_ADDR3,AS5048B_ADDR4,AS5048B_ADDR5,AS5048B_ADDR6,AS5048B_ADDR7,AS5048B_ADDR8;//定义IIC器件地址
u8 buffer1[6],buffer2[6],buffer3[6],buffer4[6],buffer5[6],buffer6[6],buffer7[6],buffer8[6];//*//用于存放AS5048B传输过来的数据
unsigned int i;

float ANGLE(u8 buffer1,u8 buffer2);
float NowAngle[6],InitAngle[6],speeds[6]= {0,0,0,0,0,0};
float resAngles0[6];
float setAngles[6];

//////////////////////////////////////////////////////
//                                                  //
//                   力控                           //
//                                                  //
//////////////////////////////////////////////////////
float LSEA=19.05;//mm
float TSEA[6]= {4000,4000,4000,4000,4000,2000}; //N.mm
float Tr1=0,Tr2=0,Tr3=0,Tr4=0,Tr5=0,Tr6=0;//N
float KSEA=180;//N/mm
float EMotorAngle[6];//°
float m[6];

float Ek1[4],uk1,ek1,Ek12[4]= {0,0,0,0},uk12;
float Ek2[4],uk2,ek2,Ek22[4]= {0,0,0,0},uk22;
float Ek3[4],uk3,ek3,Ek32[4]= {0,0,0,0},uk32;
float Ek4[4],uk4,ek4,Ek42[4]= {0,0,0,0},uk42;
float Ek5[4],uk5,ek5,Ek52[4]= {0,0,0,0},uk52;
float Ek6[4],uk6,ek6,Ek62[4]= {0,0,0,0},uk62;
float Kp=0.1,Ki=0.01,Kd=0.1,Kp2=0.1,Ki2=2,Kd2=1;
void DigitalPIDController1(void);
void DigitalPIDController2(void);
void DigitalPIDController3(void);
void DigitalPIDController4(void);
void DigitalPIDController5(void);
void DigitalPIDController6(void);
void DigitalPIDController12(void);
void DigitalPIDController22(void);
void DigitalPIDController32(void);
void DigitalPIDController42(void);
void DigitalPIDController52(void);
void DigitalPIDController62(void);

//////////////////////////////////////////////////////////
//                                                      //
//                   中值滤波                           //
//                                                      //
//////////////////////////////////////////////////////////
u8 out1[6]= {0},out2[6]= {0},out3[6]= {0},out4[6]= {0},out5[6]= {0},out6[6]= {0};
u32 a1[101]= {0},b1[101]= {0},a2[101]= {0},b2[101]= {0},a3[101]= {0},b3[101]= {0},a4[101]= {0},b4[101]= {0},a5[101]= {0},b5[101]= {0},a6[101]= {0},b6[101]= {0};
int v1=1,v2=1;

int main(void)
{
    u8 i=0;

    AS5048B_ADDR1=0x80;  //AS5048B器件地址为0x80，A2,A1接地
    AS5048B_ADDR2=0x90;  //AS5048B器件地址为0x90，A2,A1接地
    AS5048B_ADDR3=0x98;  //AS5048B器件地址为0xA0，A2,A1接地
    AS5048B_ADDR4=0xA0;  //AS5048B器件地址为0x88，A2,A1接地
    AS5048B_ADDR5=0xA8;  //AS5048B器件地址为0xB0，A2,A1接地
    AS5048B_ADDR6=0xB0;    //AS5048B器件地址为0xA8，A2,A1接地
    //AS5048B_ADDR7=0xB8;  //AS5048B器件地址为0xB8，A2,A1接地
    //AS5048B_ADDR8=0xC0;  //AS5048B器件地址为0xC0，A2,A1接地

    RCC_Configuration();	 //系统时钟初始化
    GPIO_Configuration();  //端口初始化
    delay_init(72);	       //延时初始化
    uart_init(72,115200);	 //串口初始化为115200
    NVIC_Configuration();
    LED_Init();		  	     //初始化与LED连接的硬件接口
    AS5048B_Init();        //初始化AS5048B

    //PBout(5)=1;

    Ek1[0]=Tr1-TSEA[0];
    Ek1[1]=Tr1-TSEA[0];
    Ek1[2]=Tr1-TSEA[0];
    Ek1[3]=Tr1-TSEA[0];         //传感器角度偏差PID控制器初始化
    Ek2[0]=Tr2-TSEA[1];
    Ek2[1]=Tr2-TSEA[1];
    Ek2[2]=Tr2-TSEA[1];
    Ek2[3]=Tr2-TSEA[1];         //传感器角度偏差PID控制器初始
    Ek3[0]=Tr3-TSEA[2];
    Ek3[1]=Tr3-TSEA[2];
    Ek3[2]=Tr3-TSEA[2];
    Ek3[3]=Tr3-TSEA[2];         //传感器角度偏差PID控制器初始化
    Ek4[0]=Tr4-TSEA[3];
    Ek4[1]=Tr4-TSEA[3];
    Ek4[2]=Tr4-TSEA[3];
    Ek4[3]=Tr4-TSEA[3];         //传感器角度偏差PID控制器初始
    Ek5[0]=Tr5-TSEA[4];
    Ek5[1]=Tr5-TSEA[4];
    Ek5[2]=Tr5-TSEA[4];
    Ek5[3]=Tr5-TSEA[4];         //传感器角度偏差PID控制器初始
    Ek6[0]=Tr6-TSEA[5];
    Ek6[1]=Tr6-TSEA[5];
    Ek6[2]=Tr6-TSEA[5];
    Ek6[3]=Tr6-TSEA[5];         //传感器角度偏差PID控制器初始

    vel_modes();            //前6设置车轮模式，速度为0
    enables(1);			        //前6开扭矩

    enable(7,0);  					//7开启/关闭扭矩status=0为关闭
    delay_us(100);
    vel_mode(7);            //7设置为速度模式
    delay_us(100);
    enable(7,1);  					//7开启/关闭扭矩status=0为关
    delay_us(100);
    set_speed(7,100);
    delay_us(100);

    read_angles(InitAngle);//读初始角度
    //printf("......%.2f....\r\n",InitAngle[1]);
    //InitAngle[0]=read_angle(1);//--------------------------------------
    //InitAngle[1]=read_angle(2);//--------------------------------------
    //printf("......%.2f..\r\n",InitAngle[1]);
    MID_FILTER();	            //中值滤波前一百个值
/////////////////////////////////////////////////////////////////
//																			  	                   //
//						        循环读取角度控制电机			  	           //
//																			  	                   //
/////////////////////////////////////////////////////////////////
    while (1)
    {
        //while(!(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3)==Bit_RESET));
        //enables(0);
        if ((GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3)==Bit_RESET))
        {
            pos_modes();
            enables(1);			        //前6开扭矩

            resAngles0[0]=180;//零位
            resAngles0[1]=149;
            resAngles0[2]=222;
            resAngles0[3]=88;
            resAngles0[4]=158;
            resAngles0[5]=165;
            while (1)
            {
                setAngles[0]=resAngles0[0]-resAngles[0];
                setAngles[1]=resAngles0[1]+resAngles[1];
                setAngles[2]=resAngles0[2]+resAngles[2];
                setAngles[3]=resAngles0[3]+resAngles[3];
                setAngles[4]=resAngles0[4]+resAngles[4];
                setAngles[5]=resAngles0[5]+resAngles[5];
        //printf("%f %f %f %f %f %f\r\n",setAngles[0],setAngles[1],setAngles[2],setAngles[3],setAngles[4],setAngles[5]);

                //set_angles(setAngles);
            }
        }
        else if ((GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_4)==Bit_RESET))
        {
            while(1)
            {
                //PBout(5)=!PBout(5);

/////////////////////////////////////////////////////////////////
//																			  	                   //
//						          读传感器角度	                         //
//														                                 //
/////////////////////////////////////////////////////////////////
                for(i = 4; i < 6; i++)  // AS5048B通过控制器的I2C接口读取数据
                {
                    buffer1[i] = AS5048B_ReadOneByte(AS5048B_ADDR1,0xFA+i);//从0xFA自动增益控制寄存器开始读数据
                    buffer2[i] = AS5048B_ReadOneByte(AS5048B_ADDR2,0xFA+i);//从0xFA自动增益控制寄存器开始读数据
                    buffer3[i] = AS5048B_ReadOneByte(AS5048B_ADDR3,0xFA+i);//从0xFA自动增益控制寄存器开始读数据
                    buffer4[i] = AS5048B_ReadOneByte(AS5048B_ADDR4,0xFA+i);//从0xFA自动增益控制寄存器开始读数据
                    buffer5[i] = AS5048B_ReadOneByte(AS5048B_ADDR5,0xFA+i);//从0xFA自动增益控制寄存器开始读数据
                    buffer6[i] = AS5048B_ReadOneByte(AS5048B_ADDR6,0xFA+i);//从0xFA自动增益控制寄存器开始读数据
                }
                //printf("%.2d- %.2d- \r\n",buffer1[4],buffer1[5]);
                rank1();
                rank2();
                rank3();
                rank4();
                rank5();
                rank6();

                m[0]=(ANGLE(buffer1[4],buffer1[5])-331.15);					//读取一个传感器1.48KHZ,0.6755ms
                /*if(m[0]>0)
                {
                    m[0]=m[0]-0.34;
                    if(m[0]<0)
                        m[0]=0;
                }
                else if(m[0]<0)
                {
                    m[0]=m[0]+0.34;
                    if(m[0]>0)
                        m[0]=0;
                }
                if(m[0]>7||m[0]<-7) m[0]=0;*/

                m[1]=(ANGLE(buffer2[4],buffer2[5])-118.55);					//读取一个传感器1.48KHZ,0.6755ms
                /*if(m[1]>0)
                {
                    m[1]=m[1]-0.4;
                    if(m[1]<0)
                        m[1]=0;
                }
                else if(m[1]<0)
                {
                    m[1]=m[1]+0.4;
                    if(m[1]>0)
                        m[1]=0;
                }
                if(m[1]>7||m[1]<-7) m[1]=0;*/

                m[2]=(ANGLE(buffer3[4],buffer3[5])-234.72);					//读取一个传感器1.48KHZ,0.6755ms
                /*if(m[2]>0)
                {
                    m[2]=m[2]-0.2;
                    if(m[2]<0)
                        m[2]=0;
                }
                else if(m[2]<0)
                {
                    m[2]=m[2]+0.2;
                    if(m[2]>0)
                        m[2]=0;
                }
                if(m[2]>7||m[2]<-7) m[2]=0;*/

                m[3]=(ANGLE(buffer4[4],buffer4[5])-317.15);					//读取一个传感器1.48KHZ,0.6755ms
                /*if(m[3]>0)
                {
                    m[3]=m[3]-0.25;
                    if(m[3]<0)
                        m[3]=0;
                }
                else if(m[3]<0)
                {
                    m[3]=m[3]+0.25;
                    if(m[3]>0)
                        m[3]=0;
                }
                if(m[3]>7||m[3]<-7) m[3]=0;*/

                m[4]=(ANGLE(buffer5[4],buffer5[5])-332);					//读取一个传感器1.48KHZ,0.6755ms
                /*if(m[4]>0)
                {
                    m[4]=m[4]-0.15;
                    if(m[4]<0)
                        m[4]=0;
                }
                else if(m[4]<0)
                {
                    m[4]=m[4]+0.15;
                    if(m[4]>0)
                        m[4]=0;
                }
                if(m[4]>7||m[4]<-7) m[4]=0;*/

                m[5]=(ANGLE(buffer6[4],buffer6[5])-109.12);					//读取一个传感器1.48KHZ,0.6755ms
                /*if(m[5]>0)
                {
                    m[5]=m[5]-0.35;
                    if(m[5]<0)
                        m[5]=0;
                }
                else if(m[5]<0)
                {
                    m[5]=m[5]+0.35;
                    if(m[5]>0)
                        m[5]=0;
                }
                if(m[5]>7||m[5]<-7) m[5]=0;*/
        printf("%f %f %f %f %f %f\r\n",m[0],m[1],m[2],m[3],m[4],m[5]);
								
///////////////////////////////////////////////////////////////////////////////
//																			  	                                 //
//						             计算SEA扭力					  	                         //
//																			                                     //
///////////////////////////////////////////////////////////////////////////////
                Tr1=(KSEA*m[0]*pow(LSEA,2)*pi)/180;
                Tr2=(KSEA*m[1]*pow(LSEA,2)*pi)/180;
                Tr3=(KSEA*m[2]*pow(LSEA,2)*pi)/180;
                Tr4=(KSEA*m[3]*pow(LSEA,2)*pi)/180;
                Tr5=(KSEA*m[4]*pow(LSEA,2)*pi)/180;
                Tr6=(KSEA*m[5]*pow(LSEA,2)*pi)/180;
///////////////////////////////////////////////////////////////////////////////
//																			  	                                 //
//						                     力控			  	  	                         //
//																			  	                                 //
///////////////////////////////////////////////////////////////////////////////
                if (Tr1>TSEA[0])
                {
                    ek1=Tr1-TSEA[0];
                    DigitalPIDController1();
                    speeds[0]=uk1;
                }
                else if (Tr1<-TSEA[0])
                {
                    ek1=-Tr1-TSEA[0];
                    DigitalPIDController1();
                    speeds[0]=-uk1;
                }
                else
                {
                    NowAngle[0]=read_angle(1);   //读电机角度
                    EMotorAngle[0]=InitAngle[0]-NowAngle[0];
                    if (EMotorAngle[0]>-2.5&&EMotorAngle[0]<2.5)
                    {
                        EMotorAngle[0]=0;
                    }
                    DigitalPIDController12();

                    if (EMotorAngle[0]==0)
                        speeds[0]=0;
                    else speeds[0]=uk12;
                }
//////////////////////////////ID2///////////////////////////////
                if (Tr2>TSEA[1])
                {
                    ek2=Tr2-TSEA[1];
                    DigitalPIDController2();
                    speeds[1]=uk2;
                }
                else if (Tr2<-TSEA[1])
                {
                    ek2=-Tr2-TSEA[1];
                    DigitalPIDController2();
                    speeds[1]=-uk2;
                }
                else
                {
                    NowAngle[1]=read_angle(2);   //读电机角度
                    EMotorAngle[1]=InitAngle[1]-NowAngle[1];
                    if (EMotorAngle[1]>-2&&EMotorAngle[1]<2)
                    {
                        EMotorAngle[1]=0;
                    }
                    DigitalPIDController22();
                    if (EMotorAngle[1]==0)
                        speeds[1]=0;
                    else speeds[1]=uk22;
                }
//////////////////////////////ID3///////////////////////////////
                if (Tr3>TSEA[2])
                {
                    ek3=Tr3-TSEA[2];
                    DigitalPIDController3();
                    speeds[2]=uk3;
                }
                else if (Tr3<-TSEA[2])
                {
                    ek3=-Tr3-TSEA[2];
                    DigitalPIDController3();
                    speeds[2]=-uk3;
                }
                else
                {
                    NowAngle[2]=read_angle(3);   //读电机角度
                    EMotorAngle[2]=InitAngle[2]-NowAngle[2];
                    if (EMotorAngle[2]>-1&&EMotorAngle[2]<1)
                    {
                        EMotorAngle[2]=0;
                    }
                    DigitalPIDController32();
                    if (EMotorAngle[2]==0)
                        speeds[2]=0;
                    else speeds[2]=uk32;
                }
//////////////////////////////ID4///////////////////////////////
                if (Tr4>TSEA[3])
                {
                    ek4=Tr4-TSEA[3];
                    DigitalPIDController4();
                    speeds[3]=uk4;
                }
                else if (Tr4<-TSEA[3])
                {
                    ek4=-Tr4-TSEA[3];
                    DigitalPIDController4();
                    speeds[3]=-uk4;
                }
                else
                {
                    NowAngle[3]=read_angle(4);   //读电机角度
                    EMotorAngle[3]=InitAngle[3]-NowAngle[3];
                    if (EMotorAngle[3]>-2&&EMotorAngle[3]<2)
                    {
                        EMotorAngle[3]=0;
                    }
                    DigitalPIDController42();
                    if (EMotorAngle[3]==0)
                        speeds[3]=0;
                    else speeds[3]=uk42;
                }
//////////////////////////////ID5///////////////////////////////
                if (Tr5>TSEA[4])
                {
                    ek5=Tr5-TSEA[4];
                    DigitalPIDController5();
                    speeds[4]=uk5;
                }
                else if (Tr5<-TSEA[4])
                {
                    ek5=-Tr5-TSEA[4];
                    DigitalPIDController5();
                    speeds[4]=-uk5;
                }
                else
                {
                    NowAngle[4]=read_angle(5);   //读电机角度
                    EMotorAngle[4]=InitAngle[4]-NowAngle[4];
                    if (EMotorAngle[4]>-1&&EMotorAngle[4]<1)
                    {
                        EMotorAngle[4]=0;
                    }
                    DigitalPIDController52();
                    if (EMotorAngle[4]==0)
                        speeds[4]=0;
                    else speeds[4]=uk52;
                }
//////////////////////////////ID6///////////////////////////////
                if (Tr6>TSEA[5])
                {
                    ek6=Tr6-TSEA[5];
                    DigitalPIDController6();
                    speeds[5]=uk6;
                }
                else if (Tr6<-TSEA[5])
                {
                    ek6=-Tr6-TSEA[5];
                    DigitalPIDController6();
                    speeds[5]=-uk6;
                }
                else
                {
                    NowAngle[5]=read_angle(6);   //读电机角度
                    EMotorAngle[5]=InitAngle[5]-NowAngle[5];
                    if (EMotorAngle[5]>-1.5&&EMotorAngle[5]<1.5)
                    {
                        EMotorAngle[5]=0;
                    }
                    DigitalPIDController62();
                    if (EMotorAngle[5]==0)
                        speeds[5]=0;
                    else speeds[5]=uk62;
                }
                //set_speeds(speeds);
            }
        }
    }
}

void LED_Init(void)
{
    RCC->APB2ENR|=1<<3;     //使能PORTA时钟
    //RCC->APB2ENR|=1<<5;     //使能PORTD时钟
    GPIOB->CRL&=0XFFFFFFF0; //对CRH的低四位进行清零
    GPIOB->CRL|=0X00000003; //设置PB.0为推挽输出模式
    GPIOB->ODR|=1<<0;       //设置PB.0输出1

    //GPIOB->CRL&=0XFFFFFF0F;	//对CRL的8-11位进行清零
    //GPIOB->CRL|=0X00000030; //设置PB.1为推挽输出模式
    //GPIOB->ODR|=1<<1;       //设置PB.1输出1
}



//////////////////////////////////////////////////////
//																									//
//							角度转换  													//
//																									//
//////////////////////////////////////////////////////
float ANGLE(u8 buffer1,u8 buffer2)
{
    float Angle=0;
    //buffer1=(((buffer1&0XF0)>>4)*16+(buffer1&0X0F));
    //buffer2=(((buffer2&0XF0)>>4)*16+(buffer2&0X0F));
    Angle=((buffer1*(64))+(buffer2))*0.0219;
//	Angle=((Angle&0x01)*1+(Angle&0x02)*2+(Angle&0x04)*4+(Angle&0x08)*8+(Angle&0x10)*16+(Angle&0x20)*32+(Angle&0x40)*64+(Angle&0x80)*128)*0.0219;
    return Angle;
}

void RCC_Configuration(void)
{
    Stm32_Clock_Init(9); //系统时钟设置
//    SystemInit();//72m
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE);
}



void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;


    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_7;//CS_485
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
    GPIO_Init(GPIOD,&GPIO_InitStructure);


    /*GPIO_InitStructure.GPIO_Pin=GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB,&GPIO_InitStructure);
    */

    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3;  //按键1
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
    GPIO_Init(GPIOE,&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;//232TX
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA,&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;//232RX
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA,&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;//485TX
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA,&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3;//485RX
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA,&GPIO_InitStructure);

    GPIO_ResetBits(GPIOD,GPIO_Pin_7);
}

void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void uart_init(u32 pclk2,u32 bound)
{
    USART_InitTypeDef  USART_InitStructure;

    float temp;
    u16 mantissa;
    u16 fraction;
    temp=(float)(pclk2*1000000)/(bound*16);//得到USARTDIV
    mantissa=temp;				 //得到整数部分
    fraction=(temp-mantissa)*16; //得到小数部分
    mantissa<<=4;
    mantissa+=fraction;
    RCC->APB2ENR|=1<<2;   //使能PORTA口时钟
    RCC->APB2ENR|=1<<14;  //使能串口时钟
    GPIOA->CRH&=0XFFFFF00F;//IO状态设置
    GPIOA->CRH|=0X000008B0;//IO状态设置

    RCC->APB2RSTR|=1<<14;   //复位串口1
    RCC->APB2RSTR&=~(1<<14);//停止复位
    //波特率设置
    USART1->BRR=mantissa; // 波特率设置
    USART1->CR1|=0X200C;  //1位停止,无校验位.
#if EN_USART1_RX		  //如果使能了接收
    //使能接收中断
    USART1->CR1|=1<<8;    //PE中断使能
    USART1->CR1|=1<<5;    //接收缓冲区非空中断使能
    MY_NVIC_Init(3,3,USART1_IRQn,2);//组2，最低优先级
#endif


    USART_InitStructure.USART_BaudRate=115200;  //RS232
    USART_InitStructure.USART_WordLength=USART_WordLength_8b;
    USART_InitStructure.USART_StopBits=USART_StopBits_1;
    USART_InitStructure.USART_Parity=USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;

    USART_Init(USART1,&USART_InitStructure);
    USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
    USART_Cmd(USART1,ENABLE);
    USART_ClearFlag(USART1,USART_FLAG_TC);

    USART_InitStructure.USART_BaudRate=2000000;  //RS485

    USART_Init(USART2,&USART_InitStructure);
    USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);
    USART_Cmd(USART2,ENABLE);
    USART_ClearFlag(USART2,USART_FLAG_TC);

}

void DigitalPIDController1(void)
{
    int counter=0;
    float sum_Ek=0;
    for(counter=0; counter<3; counter++)
    {
        Ek1[counter]=Ek1[counter+1];
    }
    Ek1[counter]=ek1;
    for(counter=0; counter<4; counter++)
    {
        sum_Ek=Ek1[counter]+sum_Ek;
    }
    uk1=Kp*Ek1[3]+Ki*(sum_Ek/4)+Kd*(Ek1[3]-Ek1[2]);
}

void DigitalPIDController2(void)
{
    int counter=0;
    float sum_Ek=0;
    for(counter=0; counter<3; counter++)
    {
        Ek2[counter]=Ek2[counter+1];
    }
    Ek2[counter]=ek2;
    for(counter=0; counter<4; counter++)
    {
        sum_Ek=Ek2[counter]+sum_Ek;
    }
    uk2=Kp*Ek2[3]+Ki*(sum_Ek/4)+Kd*(Ek2[3]-Ek2[2]);
}

void DigitalPIDController3(void)
{
    int counter=0;
    float sum_Ek=0;
    for(counter=0; counter<3; counter++)
    {
        Ek3[counter]=Ek3[counter+1];
    }
    Ek3[counter]=ek3;
    for(counter=0; counter<4; counter++)
    {
        sum_Ek=Ek3[counter]+sum_Ek;
    }
    uk3=Kp*Ek3[3]+Ki*(sum_Ek/4)+Kd*(Ek3[3]-Ek3[2]);
}

void DigitalPIDController4(void)
{
    int counter=0;
    float sum_Ek=0;
    for(counter=0; counter<3; counter++)
    {
        Ek4[counter]=Ek4[counter+1];
    }
    Ek4[counter]=ek4;
    for(counter=0; counter<4; counter++)
    {
        sum_Ek=Ek4[counter]+sum_Ek;
    }
    uk4=Kp*Ek4[3]+Ki*(sum_Ek/4)+Kd*(Ek4[3]-Ek4[2]);
}

void DigitalPIDController5(void)
{
    int counter=0;
    float sum_Ek=0;
    for(counter=0; counter<3; counter++)
    {
        Ek5[counter]=Ek5[counter+1];
    }
    Ek5[counter]=ek5;
    for(counter=0; counter<4; counter++)
    {
        sum_Ek=Ek5[counter]+sum_Ek;
    }
    uk5=Kp*Ek5[3]+Ki*(sum_Ek/4)+Kd*(Ek5[3]-Ek5[2]);
}
void DigitalPIDController6(void)
{
    int counter=0;
    float sum_Ek=0;
    for(counter=0; counter<3; counter++)
    {
        Ek6[counter]=Ek6[counter+1];
    }
    Ek6[counter]=ek6;
    for(counter=0; counter<4; counter++)
    {
        sum_Ek=Ek6[counter]+sum_Ek;
    }
    uk6=Kp*Ek6[3]+Ki*(sum_Ek/4)+Kd*(Ek6[3]-Ek6[2]);
}
void DigitalPIDController12(void)
{
    int counter=0;
    float sum_Ek=0;
    for(counter=0; counter<3; counter++)
    {
        Ek12[counter]=Ek12[counter+1];
    }
    Ek12[counter]=EMotorAngle[0];
    for(counter=0; counter<4; counter++)
    {
        sum_Ek=Ek12[counter]+sum_Ek;
    }
    uk12=Kp2*Ek12[3]+Ki2*(sum_Ek/4)+Kd2*(Ek12[3]-Ek12[2]);
}

void DigitalPIDController22(void)
{
    int counter=0;
    float sum_Ek=0;
    for(counter=0; counter<3; counter++)
    {
        Ek22[counter]=Ek22[counter+1];
    }
    Ek22[counter]=EMotorAngle[1];
    for(counter=0; counter<4; counter++)
    {
        sum_Ek=Ek22[counter]+sum_Ek;
    }
    uk22=Kp2*Ek22[3]+Ki2*(sum_Ek/4)+Kd2*(Ek22[3]-Ek22[2]);
}

void DigitalPIDController32(void)
{
    int counter=0;
    float sum_Ek=0;
    for(counter=0; counter<3; counter++)
    {
        Ek32[counter]=Ek32[counter+1];
    }
    Ek32[counter]=EMotorAngle[2];
    for(counter=0; counter<4; counter++)
    {
        sum_Ek=Ek32[counter]+sum_Ek;
    }
    uk32=Kp2*Ek32[3]+Ki2*(sum_Ek/4)+Kd2*(Ek32[3]-Ek32[2]);
}

void DigitalPIDController42(void)
{
    int counter=0;
    float sum_Ek=0;
    for(counter=0; counter<3; counter++)
    {
        Ek42[counter]=Ek42[counter+1];
    }
    Ek42[counter]=EMotorAngle[3];
    for(counter=0; counter<4; counter++)
    {
        sum_Ek=Ek42[counter]+sum_Ek;
    }
    uk42=Kp2*Ek42[3]+Ki2*(sum_Ek/4)+Kd2*(Ek42[3]-Ek42[2]);
}

void DigitalPIDController52(void)
{
    int counter=0;
    float sum_Ek=0;
    for(counter=0; counter<3; counter++)
    {
        Ek52[counter]=Ek52[counter+1];
    }
    Ek52[counter]=EMotorAngle[4];
    for(counter=0; counter<4; counter++)
    {
        sum_Ek=Ek52[counter]+sum_Ek;
    }
    uk52=Kp2*Ek52[3]+Ki2*(sum_Ek/4)+Kd2*(Ek52[3]-Ek52[2]);
}
void DigitalPIDController62(void)
{
    int counter=0;
    float sum_Ek=0;
    for(counter=0; counter<3; counter++)
    {
        Ek62[counter]=Ek62[counter+1];
    }
    Ek62[counter]=EMotorAngle[5];
    for(counter=0; counter<4; counter++)
    {
        sum_Ek=Ek62[counter]+sum_Ek;
    }
    uk62=Kp2*Ek62[3]+Ki2*(sum_Ek/4)+Kd2*(Ek62[3]-Ek62[2]);
}
//////////////////////////////////////////////////////
//																									//
//					中值滤波前100值 												//
//																									//
//////////////////////////////////////////////////////
void MID_FILTER()
{
    //////////////////////////////////////////////////////////中值滤波取一百个值
    for(v1=1; v1<=100; v1++)
    {
        for(i = 4; i < 6; i++)  // AS5048B通过控制器的I2C接口读取数据
        {
            buffer1[i] = AS5048B_ReadOneByte(AS5048B_ADDR5,0xFA+i);//从0xFA自动增益控制寄存器开始读数据
            buffer2[i] = AS5048B_ReadOneByte(AS5048B_ADDR6,0xFA+i);//从0xFA自动增益控制寄存器开始读数据
            buffer3[i] = AS5048B_ReadOneByte(AS5048B_ADDR5,0xFA+i);//从0xFA自动增益控制寄存器开始读数据
            buffer4[i] = AS5048B_ReadOneByte(AS5048B_ADDR6,0xFA+i);//从0xFA自动增益控制寄存器开始读数据
            buffer5[i] = AS5048B_ReadOneByte(AS5048B_ADDR5,0xFA+i);//从0xFA自动增益控制寄存器开始读数据
            buffer6[i] = AS5048B_ReadOneByte(AS5048B_ADDR6,0xFA+i);//从0xFA自动增益控制寄存器开始读数据
        }
        a1[v1]=buffer1[4];
        b1[v2]=buffer1[5];
        a1[v1]+=a1[v1-1];
        b1[v2]+=b1[v2-1];///////求前100个传感器1的数据和

        a2[v1]=buffer2[4];
        b2[v2]=buffer2[5];
        a2[v1]+=a2[v1-1];
        b2[v2]+=b2[v2-1];///////求前100个传感器2的数据和

        a3[v1]=buffer3[4];
        b3[v2]=buffer3[5];
        a3[v1]+=a3[v1-1];
        b3[v2]+=b3[v2-1];///////求前100个传感器3的数据和

        a4[v1]=buffer4[4];
        b4[v2]=buffer4[5];
        a4[v1]+=a4[v1-1];
        b4[v2]+=b4[v2-1];///////求前100个传感器4的数据和

        /*a5[v1]=buffer5[4];
        b5[v2]=buffer5[5];
        a5[v1]+=a5[v1-1];
        b5[v2]+=b5[v2-1];///////求前100个传感器5的数据和

        a6[v1]=buffer6[4];
        b6[v2]=buffer6[5];
        a6[v1]+=a6[v1-1];
        b6[v2]+=b6[v2-1];///////求前100个传感器6的数据和
        */
        v2++;
        //printf("%d %d\r\n",buffer5[4],buffer5[5]);
    }
    out1[0]=a1[100]/100;   //传感器1数据和除100
    out1[1]=b1[100]/100;
    out1[2]=a1[100]/100;   //传感器1数据和除100
    out1[3]=b1[100]/100;

    out2[0]=a2[100]/100;   //传感器2数据和除100
    out2[1]=b2[100]/100;
    out2[2]=a2[100]/100;   //传感器2数据和除100
    out2[3]=b2[100]/100;

    out3[0]=a3[100]/100;   //传感器3数据和除100
    out3[1]=b3[100]/100;
    out3[2]=a3[100]/100;   //传感器3数据和除100
    out3[3]=b3[100]/100;

    out4[0]=a4[100]/100;   //传感器4数据和除100
    out4[1]=b4[100]/100;
    out4[2]=a4[100]/100;   //传感器4数据和除100
    out4[3]=b4[100]/100;

    out5[0]=a5[100]/100;   //传感器5数据和除100
    out5[1]=b5[100]/100;
    out5[2]=a5[100]/100;   //传感器5数据和除100
    out5[3]=b5[100]/100;

    out6[0]=a6[100]/100;   //传感器6数据和除100
    out6[1]=b6[100]/100;
    out6[2]=a6[100]/100;   //传感器6数据和除100
    out6[3]=b6[100]/100;

    buffer1[4]=out1[0];    //得到传感器1滤波的第一个初始值
    buffer1[5]=out1[1];
    buffer2[4]=out2[0];			//得到传感器2滤波的第一个初始值
    buffer2[5]=out2[1];
    buffer3[4]=out3[0];
    buffer3[5]=out3[1];
    buffer4[4]=out4[0];
    buffer4[5]=out4[1];
    buffer5[4]=out5[0];
    buffer5[5]=out5[1];
    buffer6[4]=out6[0];
    buffer6[5]=out6[1];

}
//////////////////////////////////////////////////////
//																									//
//					       以下为中值滤波										//
//																									//
//////////////////////////////////////////////////////
void rank1()   ////假设前两个数是A和B，A小于B，C是新读取的数
{
    out1[4]=buffer1[4];
    out1[5]=buffer1[5];
    if(out1[5]>out1[3])    /////////C大于B和A
    {
        buffer1[5]=out1[3];
        buffer1[4]=out1[2];
        out1[0]=out1[2];
        out1[1]=out1[3];
        out1[2]=out1[4];
        out1[3]=out1[5];
    }
    else if(out1[5]==out1[3])
    {
        if(out1[4]>out1[2]||out1[4]==out1[2])   /////C大于等于B,C大于A
        {
            buffer1[5]=out1[3];
            buffer1[4]=out1[2];
            out1[0]=out1[2];
            out1[1]=out1[3];
            out1[2]=out1[4];
            out1[3]=out1[5];
        }
        else if(out1[4]<out1[2])
        {
            if(out1[5]>out1[1])  //C小于B，C大于A
            {
                buffer1[5]=out1[5];
                buffer1[4]=out1[4];
                out1[1]=out1[5];
                out1[0]=out1[4];
            }
            else if(out1[5]<out1[1])  //C小于B，C小于A
            {
                buffer1[5]=out1[1];
                buffer1[4]=out1[0];
                out1[3]=out1[1];
                out1[2]=out1[0];
                out1[1]=out1[5];
                out1[0]=out1[4];
            }
            else if(out1[5]==out1[1]) //C小于B，C和A高字节相等
            {
                if(out1[4]==out1[0]||out1[4]>out1[0])//C小于B，C大于等于A
                {
                    buffer1[5]=out1[5];
                    buffer1[4]=out1[4];
                    out1[1]=out1[5];
                    out1[0]=out1[4];
                }
                else if(out1[4]<out1[0])  //C小于B，C小于A
                {
                    buffer1[5]=out1[1];
                    buffer1[4]=out1[0];
                    out1[3]=out1[1];
                    out1[2]=out1[0];
                    out1[1]=out1[5];
                    out1[0]=out1[4];
                }
            }
        }
    }
    else if(out1[5]<out1[3])
    {
        if(out1[5]>out1[1])  //C小于B，C大于A
        {
            buffer1[5]=out1[5];
            buffer1[4]=out1[4];
            out1[1]=out1[5];
            out1[0]=out1[4];
        }
        else if(out1[5]<out1[1])  //C小于B，C小于A
        {
            buffer1[5]=out1[1];
            buffer1[4]=out1[0];
            out1[3]=out1[1];
            out1[2]=out1[0];
            out1[1]=out1[5];
            out1[0]=out1[4];
        }
        else if(out1[5]==out1[1]) //C小于B，C和A高字节相等
        {
            if(out1[4]==out1[0]||out1[4]>out1[0])//C小于B，C大于等于A
            {
                buffer1[5]=out1[5];
                buffer1[4]=out1[4];
                out1[1]=out1[5];
                out1[0]=out1[4];
            }
            else if(out1[4]<out1[0])  //C小于B，C小于A
            {
                buffer1[5]=out1[1];
                buffer1[4]=out1[0];
                out1[3]=out1[1];
                out1[2]=out1[0];
                out1[1]=out1[5];
                out1[0]=out1[4];
            }
        }
    }
}
///////////////////////////////////////////////////////////////////
void rank2()   ////假设前两个数是A和B，A小于B，C是新读取的数
{
    out2[4]=buffer2[4];
    out2[5]=buffer2[5];
    if(out2[5]>out2[3])    /////////C大于B和A
    {
        buffer2[5]=out2[3];
        buffer2[4]=out2[2];
        out2[0]=out2[2];
        out2[1]=out2[3];
        out2[2]=out2[4];
        out2[3]=out2[5];
    }
    else if(out2[5]==out2[3])
    {
        if(out2[4]>out2[2]||out2[4]==out2[2])   /////C大于等于B,C大于A
        {
            buffer2[5]=out2[3];
            buffer2[4]=out2[2];
            out2[0]=out2[2];
            out2[1]=out2[3];
            out2[2]=out2[4];
            out2[3]=out2[5];
        }
        else if(out2[4]<out2[2])
        {
            if(out2[5]>out2[1])  //C小于B，C大于A
            {
                buffer2[5]=out2[5];
                buffer2[4]=out2[4];
                out2[1]=out2[5];
                out2[0]=out2[4];
            }
            else if(out2[5]<out2[1])  //C小于B，C小于A
            {
                buffer2[5]=out2[1];
                buffer2[4]=out2[0];
                out2[3]=out2[1];
                out2[2]=out2[0];
                out2[1]=out2[5];
                out2[0]=out2[4];
            }
            else if(out2[5]==out2[1]) //C小于B，C和A高字节相等
            {
                if(out2[4]==out2[0]||out2[4]>out2[0])//C小于B，C大于等于A
                {
                    buffer2[5]=out2[5];
                    buffer2[4]=out2[4];
                    out2[1]=out2[5];
                    out2[0]=out2[4];
                }
                else if(out2[4]<out2[0])  //C小于B，C小于A
                {
                    buffer2[5]=out2[1];
                    buffer2[4]=out2[0];
                    out2[3]=out2[1];
                    out2[2]=out2[0];
                    out2[1]=out2[5];
                    out2[0]=out2[4];
                }
            }
        }
    }
    else if(out2[5]<out2[3])
    {
        if(out2[5]>out2[1])  //C小于B，C大于A
        {
            buffer2[5]=out2[5];
            buffer2[4]=out2[4];
            out2[1]=out2[5];
            out2[0]=out2[4];
        }
        else if(out2[5]<out2[1])  //C小于B，C小于A
        {
            buffer2[5]=out2[1];
            buffer2[4]=out2[0];
            out2[3]=out2[1];
            out2[2]=out2[0];
            out2[1]=out2[5];
            out2[0]=out2[4];
        }
        else if(out2[5]==out2[1]) //C小于B，C和A高字节相等
        {
            if(out2[4]==out2[0]||out2[4]>out2[0])//C小于B，C大于等于A
            {
                buffer2[5]=out2[5];
                buffer2[4]=out2[4];
                out2[1]=out2[5];
                out2[0]=out2[4];
            }
            else if(out2[4]<out2[0])  //C小于B，C小于A
            {
                buffer2[5]=out2[1];
                buffer2[4]=out2[0];
                out2[3]=out2[1];
                out2[2]=out2[0];
                out2[1]=out2[5];
                out2[0]=out2[4];
            }
        }
    }
}
///////////////////////////////////////////////////////////////////////////////
void rank3()   ////假设前两个数是A和B，A小于B，C是新读取的数
{
    out3[4]=buffer3[4];
    out3[5]=buffer3[5];
    if(out3[5]>out3[3])    /////////C大于B和A
    {
        buffer3[5]=out3[3];
        buffer3[4]=out3[2];
        out3[0]=out3[2];
        out3[1]=out3[3];
        out3[2]=out3[4];
        out3[3]=out3[5];
    }
    else if(out3[5]==out3[3])
    {
        if(out3[4]>out3[2]||out3[4]==out3[2])   /////C大于等于B,C大于A
        {
            buffer3[5]=out3[3];
            buffer3[4]=out3[2];
            out3[0]=out3[2];
            out3[1]=out3[3];
            out3[2]=out3[4];
            out3[3]=out3[5];
        }
        else if(out3[4]<out3[2])
        {
            if(out3[5]>out3[1])  //C小于B，C大于A
            {
                buffer3[5]=out3[5];
                buffer3[4]=out3[4];
                out3[1]=out3[5];
                out3[0]=out3[4];
            }
            else if(out3[5]<out3[1])  //C小于B，C小于A
            {
                buffer3[5]=out3[1];
                buffer3[4]=out3[0];
                out3[3]=out3[1];
                out3[2]=out3[0];
                out3[1]=out3[5];
                out3[0]=out3[4];
            }
            else if(out3[5]==out3[1]) //C小于B，C和A高字节相等
            {
                if(out3[4]==out3[0]||out3[4]>out3[0])//C小于B，C大于等于A
                {
                    buffer3[5]=out3[5];
                    buffer3[4]=out3[4];
                    out3[1]=out3[5];
                    out3[0]=out3[4];
                }
                else if(out3[4]<out3[0])  //C小于B，C小于A
                {
                    buffer3[5]=out3[1];
                    buffer3[4]=out3[0];
                    out3[3]=out3[1];
                    out3[2]=out3[0];
                    out3[1]=out3[5];
                    out3[0]=out3[4];
                }
            }
        }
    }
    else if(out3[5]<out3[3])
    {
        if(out3[5]>out3[1])  //C小于B，C大于A
        {
            buffer3[5]=out3[5];
            buffer3[4]=out3[4];
            out3[1]=out3[5];
            out3[0]=out3[4];
        }
        else if(out3[5]<out3[1])  //C小于B，C小于A
        {
            buffer3[5]=out3[1];
            buffer3[4]=out3[0];
            out3[3]=out3[1];
            out3[2]=out3[0];
            out3[1]=out3[5];
            out3[0]=out3[4];
        }
        else if(out3[5]==out3[1]) //C小于B，C和A高字节相等
        {
            if(out3[4]==out3[0]||out3[4]>out3[0])//C小于B，C大于等于A
            {
                buffer3[5]=out3[5];
                buffer3[4]=out3[4];
                out3[1]=out3[5];
                out3[0]=out3[4];
            }
            else if(out3[4]<out3[0])  //C小于B，C小于A
            {
                buffer3[5]=out3[1];
                buffer3[4]=out3[0];
                out3[3]=out3[1];
                out3[2]=out3[0];
                out3[1]=out3[5];
                out3[0]=out3[4];
            }
        }
    }
}
///////////////////////////////////////////////////////////////////
void rank4()   ////假设前两个数是A和B，A小于B，C是新读取的数
{
    out4[4]=buffer4[4];
    out4[5]=buffer4[5];
    if(out4[5]>out4[3])    /////////C大于B和A
    {
        buffer4[5]=out4[3];
        buffer4[4]=out4[2];
        out4[0]=out4[2];
        out4[1]=out4[3];
        out4[2]=out4[4];
        out4[3]=out4[5];
    }
    else if(out4[5]==out4[3])
    {
        if(out4[4]>out4[2]||out4[4]==out4[2])   /////C大于等于B,C大于A
        {
            buffer4[5]=out4[3];
            buffer4[4]=out4[2];
            out4[0]=out4[2];
            out4[1]=out4[3];
            out4[2]=out4[4];
            out4[3]=out4[5];
        }
        else if(out4[4]<out4[2])
        {
            if(out4[5]>out4[1])  //C小于B，C大于A
            {
                buffer4[5]=out4[5];
                buffer4[4]=out4[4];
                out4[1]=out4[5];
                out4[0]=out4[4];
            }
            else if(out4[5]<out4[1])  //C小于B，C小于A
            {
                buffer4[5]=out4[1];
                buffer4[4]=out4[0];
                out4[3]=out4[1];
                out4[2]=out4[0];
                out4[1]=out4[5];
                out4[0]=out4[4];
            }
            else if(out4[5]==out4[1]) //C小于B，C和A高字节相等
            {
                if(out4[4]==out4[0]||out4[4]>out4[0])//C小于B，C大于等于A
                {
                    buffer4[5]=out4[5];
                    buffer4[4]=out4[4];
                    out4[1]=out4[5];
                    out4[0]=out4[4];
                }
                else if(out4[4]<out4[0])  //C小于B，C小于A
                {
                    buffer4[5]=out4[1];
                    buffer4[4]=out4[0];
                    out4[3]=out4[1];
                    out4[2]=out4[0];
                    out4[1]=out4[5];
                    out4[0]=out4[4];
                }
            }
        }
    }
    else if(out4[5]<out4[3])
    {
        if(out4[5]>out4[1])  //C小于B，C大于A
        {
            buffer4[5]=out4[5];
            buffer4[4]=out4[4];
            out4[1]=out4[5];
            out4[0]=out4[4];
        }
        else if(out4[5]<out4[1])  //C小于B，C小于A
        {
            buffer4[5]=out4[1];
            buffer4[4]=out4[0];
            out4[3]=out4[1];
            out4[2]=out4[0];
            out4[1]=out4[5];
            out4[0]=out4[4];
        }
        else if(out4[5]==out4[1]) //C小于B，C和A高字节相等
        {
            if(out4[4]==out4[0]||out4[4]>out4[0])//C小于B，C大于等于A
            {
                buffer4[5]=out4[5];
                buffer4[4]=out4[4];
                out4[1]=out4[5];
                out4[0]=out4[4];
            }
            else if(out4[4]<out4[0])  //C小于B，C小于A
            {
                buffer4[5]=out4[1];
                buffer4[4]=out4[0];
                out4[3]=out4[1];
                out4[2]=out4[0];
                out4[1]=out4[5];
                out4[0]=out4[4];
            }
        }
    }
}
///////////////////////////////////////////////////////////////
void rank5()   ////假设前两个数是A和B，A小于B，C是新读取的数
{
    out5[4]=buffer5[4];
    out5[5]=buffer5[5];
    if(out5[5]>out5[3])    /////////C大于B和A
    {
        buffer5[5]=out5[3];
        buffer5[4]=out5[2];
        out5[0]=out5[2];
        out5[1]=out5[3];
        out5[2]=out5[4];
        out5[3]=out5[5];
    }
    else if(out5[5]==out5[3])
    {
        if(out5[4]>out5[2]||out5[4]==out5[2])   /////C大于等于B,C大于A
        {
            buffer5[5]=out5[3];
            buffer5[4]=out5[2];
            out5[0]=out5[2];
            out5[1]=out5[3];
            out5[2]=out5[4];
            out5[3]=out5[5];
        }
        else if(out5[4]<out5[2])
        {
            if(out5[5]>out5[1])  //C小于B，C大于A
            {
                buffer5[5]=out5[5];
                buffer5[4]=out5[4];
                out5[1]=out5[5];
                out5[0]=out5[4];
            }
            else if(out5[5]<out5[1])  //C小于B，C小于A
            {
                buffer5[5]=out5[1];
                buffer5[4]=out5[0];
                out5[3]=out5[1];
                out5[2]=out5[0];
                out5[1]=out5[5];
                out5[0]=out5[4];
            }
            else if(out5[5]==out5[1]) //C小于B，C和A高字节相等
            {
                if(out5[4]==out5[0]||out5[4]>out5[0])//C小于B，C大于等于A
                {
                    buffer5[5]=out5[5];
                    buffer5[4]=out5[4];
                    out5[1]=out5[5];
                    out5[0]=out5[4];
                }
                else if(out5[4]<out5[0])  //C小于B，C小于A
                {
                    buffer5[5]=out5[1];
                    buffer5[4]=out5[0];
                    out5[3]=out5[1];
                    out5[2]=out5[0];
                    out5[1]=out5[5];
                    out5[0]=out5[4];
                }
            }
        }
    }
    else if(out5[5]<out5[3])
    {
        if(out5[5]>out5[1])  //C小于B，C大于A
        {
            buffer5[5]=out5[5];
            buffer5[4]=out5[4];
            out5[1]=out5[5];
            out5[0]=out5[4];
        }
        else if(out5[5]<out5[1])  //C小于B，C小于A
        {
            buffer5[5]=out5[1];
            buffer5[4]=out5[0];
            out5[3]=out5[1];
            out5[2]=out5[0];
            out5[1]=out5[5];
            out5[0]=out5[4];
        }
        else if(out5[5]==out5[1]) //C小于B，C和A高字节相等
        {
            if(out5[4]==out5[0]||out5[4]>out5[0])//C小于B，C大于等于A
            {
                buffer5[5]=out5[5];
                buffer5[4]=out5[4];
                out5[1]=out5[5];
                out5[0]=out5[4];
            }
            else if(out5[4]<out5[0])  //C小于B，C小于A
            {
                buffer5[5]=out5[1];
                buffer5[4]=out5[0];
                out5[3]=out5[1];
                out5[2]=out5[0];
                out5[1]=out5[5];
                out5[0]=out5[4];
            }
        }
    }
}
////////////////////////////////////////////////////////////////////
void rank6()   ////假设前两个数是A和B，A小于B，C是新读取的数
{
    out6[4]=buffer6[4];
    out6[5]=buffer6[5];
    if(out6[5]>out6[3])    /////////C大于B和A
    {
        buffer6[5]=out6[3];
        buffer6[4]=out6[2];
        out6[0]=out6[2];
        out6[1]=out6[3];
        out6[2]=out6[4];
        out6[3]=out6[5];
    }
    else if(out6[5]==out6[3])
    {
        if(out6[4]>out6[2]||out6[4]==out6[2])   /////C大于等于B,C大于A
        {
            buffer6[5]=out6[3];
            buffer6[4]=out6[2];
            out6[0]=out6[2];
            out6[1]=out6[3];
            out6[2]=out6[4];
            out6[3]=out6[5];
        }
        else if(out6[4]<out6[2])
        {
            if(out6[5]>out6[1])  //C小于B，C大于A
            {
                buffer6[5]=out6[5];
                buffer6[4]=out6[4];
                out6[1]=out6[5];
                out6[0]=out6[4];
            }
            else if(out6[5]<out6[1])  //C小于B，C小于A
            {
                buffer6[5]=out6[1];
                buffer6[4]=out6[0];
                out6[3]=out6[1];
                out6[2]=out6[0];
                out6[1]=out6[5];
                out6[0]=out6[4];
            }
            else if(out6[5]==out6[1]) //C小于B，C和A高字节相等
            {
                if(out6[4]==out6[0]||out6[4]>out6[0])//C小于B，C大于等于A
                {
                    buffer6[5]=out6[5];
                    buffer6[4]=out6[4];
                    out6[1]=out6[5];
                    out6[0]=out6[4];
                }
                else if(out6[4]<out6[0])  //C小于B，C小于A
                {
                    buffer6[5]=out6[1];
                    buffer6[4]=out6[0];
                    out6[3]=out6[1];
                    out6[2]=out6[0];
                    out6[1]=out6[5];
                    out6[0]=out6[4];
                }
            }
        }
    }
    else if(out6[5]<out6[3])
    {
        if(out6[5]>out6[1])  //C小于B，C大于A
        {
            buffer6[5]=out6[5];
            buffer6[4]=out6[4];
            out6[1]=out6[5];
            out6[0]=out6[4];
        }
        else if(out6[5]<out6[1])  //C小于B，C小于A
        {
            buffer6[5]=out6[1];
            buffer6[4]=out6[0];
            out6[3]=out6[1];
            out6[2]=out6[0];
            out6[1]=out6[5];
            out6[0]=out6[4];
        }
        else if(out6[5]==out6[1]) //C小于B，C和A高字节相等
        {
            if(out6[4]==out6[0]||out6[4]>out6[0])//C小于B，C大于等于A
            {
                buffer6[5]=out6[5];
                buffer6[4]=out6[4];
                out6[1]=out6[5];
                out6[0]=out6[4];
            }
            else if(out6[4]<out6[0])  //C小于B，C小于A
            {
                buffer6[5]=out6[1];
                buffer6[4]=out6[0];
                out6[3]=out6[1];
                out6[2]=out6[0];
                out6[1]=out6[5];
                out6[0]=out6[4];
            }
        }
    }
}


////////////////////////////////////////////////////////
//              以上为中值滤波                        //
////////////////////////////////////////////////////////


