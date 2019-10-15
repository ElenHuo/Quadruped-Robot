#include "pbdata.h"


void sendbytes(u8 *buf,u8 len)
{
    u8 t;
    RS485_RX_CNT=0;
    GPIO_SetBits(GPIOD,GPIO_Pin_7);			//设置为发送模式
    for(t=0; t<len; t++)		//循环发送数据
    {
        while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET); //等待发送结束
        USART_SendData(USART2,buf[t]); //发送数据
    }
    while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET); //等待发送结束

    GPIO_ResetBits(GPIOD,GPIO_Pin_7);	//设置为接收模式

}

void readbytes(u8 *buf)
{
    u8 i=0;
    delay_us(150);
    //delay_us(80);	//等待接收结束
    for(i=0; i<RS485_RX_CNT; i++)
    {
        buf[i]=RS485_RX_BUF[i];
        //printf(",%X,",buf[i]);
    }
    RS485_RX_CNT=0;		//清零
}

const u16 crc_table[256] = {
    0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
    0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
    0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
    0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
    0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
    0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
    0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
    0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
    0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
    0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
    0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
    0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
    0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
    0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
    0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
    0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
    0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
    0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
    0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
    0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
    0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
    0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
    0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
    0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
    0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
    0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
    0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
    0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
    0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
    0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
    0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
};


u16 get_crc(u8 *data_blk_ptr, u16 data_blk_size)
{
    u16 i, j;
    u16 crc_accum=0;

    for(j = 0; j < data_blk_size; j++)
    {
        i = ((u16)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }

    return crc_accum;
}



u8 read_temp(u8 ID)
{
    u8 databuf[14];
    u8 rec[20];
    u16 crc=0;

    databuf[0] = 0xff;
    databuf[1] = 0xff;
    databuf[2] = 0xfd;
    databuf[3] = 0x00;
    databuf[4] = ID;
    databuf[5] = 0x07;
    databuf[6] = 0x00;
    databuf[7] = 0x02;
    databuf[8] = 146;
    databuf[9] = 0x00;
    databuf[10]= 0x01;
    databuf[11]= 0x00;

    crc=get_crc(databuf,12);
    databuf[12]= (u8)(crc & 0xff);
    databuf[13]= (u8)((crc>>8) & 0xff);

    sendbytes(databuf, 14);
    readbytes(rec);
    printf("error = %d 温度 = %d\r\n", rec[8], rec[9]);
    return rec[9];
}

void set_angle(u8 ID, float angle)
{
    u8 databuf[16];
    u16 crc=0;
    u32 sangle;

    //if(angle> 180) angle=180;
    //else if(angle<-180) angle=-180;
    //if(angle<0)	sangle = 2048-(u32)((-angle/360.0)*4095);
    //else	sangle = 2048+(u32)((angle/360.0)*4095);

    if(angle> 360) angle=360;
    else if(angle<0) angle=0;
    sangle = (u32)((angle/360.0)*4095);

    //sangle=1900;

    databuf[0] = 0xff;
    databuf[1] = 0xff;
    databuf[2] = 0xfd;
    databuf[3] = 0x00;
    databuf[4] = ID;
    databuf[5] = 0x09;
    databuf[6] = 0x00;
    databuf[7] = 0x03;
    databuf[8] = 116;
    databuf[9] = 0x00;
    databuf[10]= (u8)( sangle      & 0xff);
    databuf[11]= (u8)((sangle>>8 ) & 0xff);
    databuf[12]= (u8)((sangle>>16) & 0xff);
    databuf[13]= (u8)((sangle>>24) & 0xff);

    crc=get_crc(databuf,14);
    databuf[14]= (u8)(crc & 0xff);
    databuf[15]= (u8)((crc>>8) & 0xff);

    sendbytes(databuf, 16);
}


float read_angle(u8 ID)
{
    u8 databuf[14];
    u8 rec[25];
    u16 crc=0;
    u32 sangle;
    float angle;

    databuf[0] = 0xff;
    databuf[1] = 0xff;
    databuf[2] = 0xfd;
    databuf[3] = 0x00;
    databuf[4] = ID;
    databuf[5] = 0x07;
    databuf[6] = 0x00;
    databuf[7] = 0x02;
    databuf[8] = 132;
    databuf[9] = 0x00;
    databuf[10]= 0x04;
    databuf[11]= 0x00;

    crc=get_crc(databuf,12);
    databuf[12]= (u8)(crc & 0xff);
    databuf[13]= (u8)((crc>>8) & 0xff);

    sendbytes(databuf, 14);
    readbytes(rec);

    sangle =(u32)rec[9] | (u32)(rec[10]<<8) | (u32)(rec[11]<<16) | (u32)(rec[12]<<24);
    if(sangle>=2048) angle= (float)((sangle-2048)%4095)*360.0/4095.0;
    else angle=-(float)((2048-sangle)%4095)*360.0/4095.0;
    //printf("error = %d 位置 = %.2f°\r\n", rec[8], angle);
    return angle;
}

void enable(u8 ID,u8 status)
{
    u8 databuf[13];
    u16 crc=0;

    databuf[0] = 0xff;
    databuf[1] = 0xff;
    databuf[2] = 0xfd;
    databuf[3] = 0x00;
    databuf[4] = ID;
    databuf[5] = 0x06;
    databuf[6] = 0x00;
    databuf[7] = 0x03;
    databuf[8] = 64;
    databuf[9] = 0x00;
    databuf[10]= status;

    crc=get_crc(databuf,11);
    databuf[11]= (u8)(crc & 0xff);
    databuf[12]= (u8)((crc>>8) & 0xff);

    sendbytes(databuf, 13);
}

void enables(u8 status)
{
    u8 databuf[ENG_NUM*2+14];
    u8 i = 0;
    u16 crc=0;

    databuf[0] = 0xff;
    databuf[1] = 0xff;
    databuf[2] = 0xfd;
    databuf[3] = 0x00;
    databuf[4] = 0xfe;
    databuf[5] = (1+1)*ENG_NUM+4+3;
    databuf[6] = 0x00;
    databuf[7] = 0x83;
    databuf[8] = 64;
    databuf[9] = 0x00;
    databuf[10]= 0x01;
    databuf[11]= 0x00;

    for(i=0; i<ENG_NUM; i++)
    {
        databuf[12+2*i]=i+1;
        databuf[12+2*i+1]=status;
    }

    crc=get_crc(databuf,ENG_NUM*2+12);
    databuf[ENG_NUM*2+12]= (u8)(crc & 0xff);
    databuf[ENG_NUM*2+13]= (u8)((crc>>8) & 0xff);

    sendbytes(databuf, ENG_NUM*2+14);
}

void set_ID(u8 ID, u8 newID)
{
    u8 databuf[13];
    u16 crc=0;

    databuf[0] = 0xff;
    databuf[1] = 0xff;
    databuf[2] = 0xfd;
    databuf[3] = 0x00;
    databuf[4] = ID;
    databuf[5] = 0x06;
    databuf[6] = 0x00;
    databuf[7] = 0x03;
    databuf[8] = 7;
    databuf[9] = 0x00;
    databuf[10]= newID;

    crc=get_crc(databuf,11);
    databuf[11]= (u8)(crc & 0xff);
    databuf[12]= (u8)((crc>>8) & 0xff);

    sendbytes(databuf, 13);
}

void set_baud(u8 ID,u8 baud)
{
    u8 databuf[13];
    u16 crc=0;

    databuf[0] = 0xff;
    databuf[1] = 0xff;
    databuf[2] = 0xfd;
    databuf[3] = 0x00;
    databuf[4] = ID;
    databuf[5] = 0x06;
    databuf[6] = 0x00;
    databuf[7] = 0x03;
    databuf[8] = 8;
    databuf[9] = 0x00;
    databuf[10]= baud;

    crc=get_crc(databuf,11);
    databuf[11]= (u8)(crc & 0xff);
    databuf[12]= (u8)((crc>>8) & 0xff);

    sendbytes(databuf, 13);
}

void set_bauds(u8 baud)
{
    u8 databuf[ENG_NUM*2+14];
    u8 i = 0;
    u16 crc=0;

    databuf[0] = 0xff;
    databuf[1] = 0xff;
    databuf[2] = 0xfd;
    databuf[3] = 0x00;
    databuf[4] = 0xfe;
    databuf[5] = (1+1)*ENG_NUM+4+3;
    databuf[6] = 0x00;
    databuf[7] = 0x83;
    databuf[8] = 8;
    databuf[9] = 0x00;
    databuf[10]= 0x01;
    databuf[11]= 0x00;

    for(i=0; i<ENG_NUM; i++)
    {
        databuf[12+2*i]=i+1;
        databuf[12+2*i+1]=baud;
    }

    crc=get_crc(databuf,ENG_NUM*2+12);
    databuf[ENG_NUM*2+12]= (u8)(crc & 0xff);
    databuf[ENG_NUM*2+13]= (u8)((crc>>8) & 0xff);

    sendbytes(databuf, ENG_NUM*2+14);
}

void set_LED(u8 ID,u8 status)
{
    u8 databuf[13];
    u16 crc=0;

    databuf[0] = 0xff;
    databuf[1] = 0xff;
    databuf[2] = 0xfd;
    databuf[3] = 0x00;
    databuf[4] = ID;
    databuf[5] = 0x06;
    databuf[6] = 0x00;
    databuf[7] = 0x03;
    databuf[8] = 65;
    databuf[9] = 0x00;
    databuf[10]= status;

    crc=get_crc(databuf,11);
    databuf[11]= (u8)(crc & 0xff);
    databuf[12]= (u8)((crc>>8) & 0xff);

    sendbytes(databuf, 13);
}


void set_mode(u8 ID, u8 mode)
{
    u8 databuf[13];
    u16 crc=0;

    enable(ID,0);

    databuf[0] = 0xff;
    databuf[1] = 0xff;
    databuf[2] = 0xfd;
    databuf[3] = 0x00;
    databuf[4] = ID;
    databuf[5] = 0x06;
    databuf[6] = 0x00;
    databuf[7] = 0x03;
    databuf[8] = 11;
    databuf[9] = 0x00;
    databuf[10]= mode;

    crc=get_crc(databuf,11);
    databuf[11]= (u8)(crc & 0xff);
    databuf[12]= (u8)((crc>>8) & 0xff);

    sendbytes(databuf, 13);
}

void vel_mode(u8 ID)
{
    set_mode(ID, 1);
}

void vel_modes(void)
{
    set_modes(1);
}

void pos_mode(u8 ID)
{
    set_mode(ID,3);
}

void pos_modes(void)
{
    set_modes(3);
}

void set_modes(u8 mode)
{
    u8 databuf[ENG_NUM*2+14];
    u8 i = 0;
    u16 crc=0;

    enables(0);

    databuf[0] = 0xff;
    databuf[1] = 0xff;
    databuf[2] = 0xfd;
    databuf[3] = 0x00;
    databuf[4] = 0xfe;
    databuf[5] = (1+1)*ENG_NUM+4+3;
    databuf[6] = 0x00;
    databuf[7] = 0x83;
    databuf[8] = 11;
    databuf[9] = 0x00;
    databuf[10]= 0x01;
    databuf[11]= 0x00;

    for(i=0; i<ENG_NUM; i++)
    {
        databuf[12+2*i]=i+1;
        databuf[12+2*i+1]=mode;
    }

    crc=get_crc(databuf,ENG_NUM*2+12);
    databuf[ENG_NUM*2+12]= (u8)(crc & 0xff);
    databuf[ENG_NUM*2+13]= (u8)((crc>>8) & 0xff);

    sendbytes(databuf, ENG_NUM*2+14);
}


float read_speed(u8 ID)
{
    u8 databuf[14];
    u8 rec[25];
    u16 crc=0;
    float speed;
    s32 sspeed;

    databuf[0] = 0xff;
    databuf[1] = 0xff;
    databuf[2] = 0xfd;
    databuf[3] = 0x00;
    databuf[4] = ID;
    databuf[5] = 0x07;
    databuf[6] = 0x00;
    databuf[7] = 0x02;
    databuf[8] = 128;
    databuf[9] = 0x00;
    databuf[10]= 0x04;
    databuf[11]= 0x00;

    crc=get_crc(databuf,12);
    databuf[12]= (u8)(crc & 0xff);
    databuf[13]= (u8)((crc>>8) & 0xff);

    sendbytes(databuf, 14);
    readbytes(rec);
    sspeed=(s32)rec[9] | (s32)(rec[10]<<8) | (s32)(rec[11]<<16) | (s32)(rec[12]<<24);
    speed = (float)sspeed;

    printf("error = %d 速度 = %.2f\r\n", rec[8], speed);
    return speed;
}

void set_speed(u8 ID, float speed)
{
    u8 databuf[16];
    u16 crc=0;
    s32 sspeed;
    if(speed>1023) speed=1023;
    else if(speed<-1023) speed=-1023;
    sspeed = (s32)(speed);

    databuf[0] = 0xff;
    databuf[1] = 0xff;
    databuf[2] = 0xfd;
    databuf[3] = 0x00;
    databuf[4] = ID;
    databuf[5] = 0x09;
    databuf[6] = 0x00;
    databuf[7] = 0x03;
    databuf[8] = 104;
    databuf[9] = 0x00;
    databuf[10]= (u8)( sspeed      & 0xff);
    databuf[11]= (u8)((sspeed>>8 ) & 0xff);
    databuf[12]= (u8)((sspeed>>16) & 0xff);
    databuf[13]= (u8)((sspeed>>24) & 0xff);

    crc=get_crc(databuf,14);
    databuf[14]= (u8)(crc & 0xff);
    databuf[15]= (u8)((crc>>8) & 0xff);

    sendbytes(databuf, 16);
}

void set_same_speeds(float speed)
{
    u8 databuf[ENG_NUM*(4+1)+14];
    u8 i = 0;
    u16 crc=0;
    s32 sspeed;
    if(speed>1023) speed=1023;
    else if(speed<-1023) speed=-1023;
    sspeed = (s32)(speed);
    sspeed = (s32)(speed);

    databuf[0] = 0xff;
    databuf[1] = 0xff;
    databuf[2] = 0xfd;
    databuf[3] = 0x00;
    databuf[4] = 0xfe;
    databuf[5] = (4+1)*ENG_NUM+4+3;
    databuf[6] = 0x00;
    databuf[7] = 0x83;
    databuf[8] = 104;
    databuf[9] = 0x00;
    databuf[10]= 0x04;
    databuf[11]= 0x00;

    for(i=0; i<ENG_NUM; i++)
    {
        databuf[12+5*i]=i+1;
        databuf[12+5*i+1]= (u8)( sspeed      & 0xff);
        databuf[12+5*i+2]= (u8)((sspeed>>8 ) & 0xff);
        databuf[12+5*i+3]= (u8)((sspeed>>16) & 0xff);
        databuf[12+5*i+4]= (u8)((sspeed>>24) & 0xff);
    }

    crc=get_crc(databuf,ENG_NUM*(4+1)+12);
    databuf[ENG_NUM*(4+1)+12]= (u8)(crc & 0xff);
    databuf[ENG_NUM*(4+1)+13]= (u8)((crc>>8) & 0xff);

    sendbytes(databuf, ENG_NUM*(4+1)+14);
}

void set_speeds(float speeds[ENG_NUM])
{
    u8 databuf[ENG_NUM*(4+1)+14];
    u8 i = 0;
    u16 crc=0;
    s32 sspeeds[ENG_NUM];
    for(i=0; i<ENG_NUM; i++)
    {
        if(speeds[i]>1023) speeds[i]=1023;
        else if(speeds[i]<-1023) speeds[i]=-1023;
        sspeeds[i] = (s32)(speeds[i]);
    }

    databuf[0] = 0xff;
    databuf[1] = 0xff;
    databuf[2] = 0xfd;
    databuf[3] = 0x00;
    databuf[4] = 0xfe;
    databuf[5] = (4+1)*ENG_NUM+4+3;
    databuf[6] = 0x00;
    databuf[7] = 0x83;
    databuf[8] = 104;
    databuf[9] = 0x00;
    databuf[10]= 0x04;
    databuf[11]= 0x00;

    for(i=0; i<ENG_NUM; i++)
    {
        databuf[12+5*i]=i+1;
        databuf[12+5*i+1]= (u8)( sspeeds[i]      & 0xff);
        databuf[12+5*i+2]= (u8)((sspeeds[i]>>8 ) & 0xff);
        databuf[12+5*i+3]= (u8)((sspeeds[i]>>16) & 0xff);
        databuf[12+5*i+4]= (u8)((sspeeds[i]>>24) & 0xff);
    }

    crc=get_crc(databuf,ENG_NUM*(4+1)+12);
    databuf[ENG_NUM*(4+1)+12]= (u8)(crc & 0xff);
    databuf[ENG_NUM*(4+1)+13]= (u8)((crc>>8) & 0xff);

    sendbytes(databuf, ENG_NUM*(4+1)+14);
}

void set_same_angles(float angle)
{
    u8 databuf[ENG_NUM*(4+1)+14];
    u8 i = 0;
    u16 crc=0;
    u32 sangle;

    if(angle> 180) angle=180;
    else if(angle<-180) angle=-180;
    if(angle<0)	sangle = 2048-(u32)((-angle/360.0)*4095);
    else	sangle = 2048+(u32)((angle/360.0)*4095);

    databuf[0] = 0xff;
    databuf[1] = 0xff;
    databuf[2] = 0xfd;
    databuf[3] = 0x00;
    databuf[4] = 0xfe;
    databuf[5] = (4+1)*ENG_NUM+4+3;
    databuf[6] = 0x00;
    databuf[7] = 0x83;
    databuf[8] = 116;
    databuf[9] = 0x00;
    databuf[10]= 0x04;
    databuf[11]= 0x00;

    for(i=0; i<ENG_NUM; i++)
    {
        databuf[12+5*i]=i+1;
        databuf[12+5*i+1]= (u8)( sangle      & 0xff);
        databuf[12+5*i+2]= (u8)((sangle>>8 ) & 0xff);
        databuf[12+5*i+3]= (u8)((sangle>>16) & 0xff);
        databuf[12+5*i+4]= (u8)((sangle>>24) & 0xff);
    }

    crc=get_crc(databuf,ENG_NUM*(4+1)+12);
    databuf[ENG_NUM*(4+1)+12]= (u8)(crc & 0xff);
    databuf[ENG_NUM*(4+1)+13]= (u8)((crc>>8) & 0xff);

    sendbytes(databuf, ENG_NUM*(4+1)+14);
}

void set_angles(float angles[ENG_NUM])
{
    u8 databuf[ENG_NUM*(4+1)+14];
    u8 i = 0;
    u16 crc=0;

    u32 sangles[ENG_NUM];
    for(i=0; i<ENG_NUM; i++)
    {
        if(angles[i]> 360) angles[i]=360;
        else if(angles[i]<0) angles[i]=0;
        sangles[i] = (u32)((angles[i]/360.0)*4095);
        //if(angles[i]> 180) angles[i]=180;
        //else if(angles[i]<-180) angles[i]=-180;
        //if(angles[i]<0)	sangles[i] = 2048-(u32)((-angles[i]/360.0)*4095);
        //else	sangles[i] = 2048+(u32)((angles[i]/360.0)*4095);
    }
    //printf("%d %d %d %d %d %d\r\n",sangles[0],sangles[1],sangles[2],sangles[3],sangles[4],sangles[5]);

    databuf[0] = 0xff;
    databuf[1] = 0xff;
    databuf[2] = 0xfd;
    databuf[3] = 0x00;
    databuf[4] = 0xfe;
    databuf[5] = (4+1)*ENG_NUM+4+3;
    databuf[6] = 0x00;
    databuf[7] = 0x83;
    databuf[8] = 116;
    databuf[9] = 0x00;
    databuf[10]= 0x04;
    databuf[11]= 0x00;

    for(i=0; i<ENG_NUM; i++)
    {
        databuf[12+5*i]=i+1;
        databuf[12+5*i+1]= (u8)( sangles[i]      & 0xff);
        databuf[12+5*i+2]= (u8)((sangles[i]>>8 ) & 0xff);
        databuf[12+5*i+3]= (u8)((sangles[i]>>16) & 0xff);
        databuf[12+5*i+4]= (u8)((sangles[i]>>24) & 0xff);
    }

    crc=get_crc(databuf,ENG_NUM*(4+1)+12);
    databuf[ENG_NUM*(4+1)+12]= (u8)(crc & 0xff);
    databuf[ENG_NUM*(4+1)+13]= (u8)((crc>>8) & 0xff);

    sendbytes(databuf, ENG_NUM*(4+1)+14);
}


void set_speed_range(u8 ID, u32 maxSpeed)
{
    u8 databuf[16];
    u16 crc=0;

    databuf[0] = 0xff;
    databuf[1] = 0xff;
    databuf[2] = 0xfd;
    databuf[3] = 0x00;
    databuf[4] = ID;
    databuf[5] = 0x09;
    databuf[6] = 0x00;
    databuf[7] = 0x03;
    databuf[8] = 44;
    databuf[9] = 0x00;
    databuf[10]= (u8)( maxSpeed      & 0xff);
    databuf[11]= (u8)((maxSpeed>>8 ) & 0xff);
    databuf[12]= (u8)((maxSpeed>>16) & 0xff);
    databuf[13]= (u8)((maxSpeed>>24) & 0xff);

    crc=get_crc(databuf,14);
    databuf[14]= (u8)(crc & 0xff);
    databuf[15]= (u8)((crc>>8) & 0xff);

    sendbytes(databuf, 16);
}

void set_angle_range(u8 ID, u32 minAngle, u32 maxAngle)
{
    u8 databuf[20];
    u16 crc=0;

    databuf[0] = 0xff;
    databuf[1] = 0xff;
    databuf[2] = 0xfd;
    databuf[3] = 0x00;
    databuf[4] = ID;
    databuf[5] = 13;
    databuf[6] = 0x00;
    databuf[7] = 0x03;
    databuf[8] = 48;
    databuf[9] = 0x00;
    databuf[10]= (u8)( maxAngle      & 0xff);
    databuf[11]= (u8)((maxAngle>>8 ) & 0xff);
    databuf[12]= (u8)((maxAngle>>16) & 0xff);
    databuf[13]= (u8)((maxAngle>>24) & 0xff);
    databuf[14]= (u8)( minAngle      & 0xff);
    databuf[15]= (u8)((minAngle>>8 ) & 0xff);
    databuf[16]= (u8)((minAngle>>16) & 0xff);
    databuf[17]= (u8)((minAngle>>24) & 0xff);

    crc=get_crc(databuf,18);
    databuf[18]= (u8)(crc & 0xff);
    databuf[19]= (u8)((crc>>8) & 0xff);

    sendbytes(databuf, 20);
}

u8 read_error(u8 ID)
{
    u8 databuf[14];
    u8 rec[25];
    u16 crc=0;

    databuf[0] = 0xff;
    databuf[1] = 0xff;
    databuf[2] = 0xfd;
    databuf[3] = 0x00;
    databuf[4] = ID;
    databuf[5] = 0x07;
    databuf[6] = 0x00;
    databuf[7] = 0x02;
    databuf[8] = 70;
    databuf[9] = 0x00;
    databuf[10]= 0x01;
    databuf[11]= 0x00;

    crc=get_crc(databuf,12);
    databuf[12]= (u8)(crc & 0xff);
    databuf[13]= (u8)((crc>>8) & 0xff);

    sendbytes(databuf, 14);
    readbytes(rec);

    printf("error = %d ERROR = %d\r\n", rec[8], rec[9]);
    return rec[9];
}

void reboot(u8 ID)
{
    u8 databuf[10];
    u16 crc=0;

    databuf[0] = 0xff;
    databuf[1] = 0xff;
    databuf[2] = 0xfd;
    databuf[3] = 0x00;
    databuf[4] = ID;
    databuf[5] = 0x03;
    databuf[6] = 0x00;
    databuf[7] = 0x08;

    crc=get_crc(databuf,8);
    databuf[8]= (u8)(crc & 0xff);
    databuf[9]= (u8)((crc>>8) & 0xff);

    sendbytes(databuf, 10);
}

u8 read_mode(u8 ID)
{
    u8 databuf[14];
    u8 rec[25];
    u16 crc=0;

    databuf[0] = 0xff;
    databuf[1] = 0xff;
    databuf[2] = 0xfd;
    databuf[3] = 0x00;
    databuf[4] = ID;
    databuf[5] = 0x07;
    databuf[6] = 0x00;
    databuf[7] = 0x02;
    databuf[8] = 11;
    databuf[9] = 0x00;
    databuf[10]= 0x01;
    databuf[11]= 0x00;

    crc=get_crc(databuf,12);
    databuf[12]= (u8)(crc & 0xff);
    databuf[13]= (u8)((crc>>8) & 0xff);

    sendbytes(databuf, 14);
    readbytes(rec);

    printf("error = %d 模式 = %d\r\n", rec[8], rec[9]);
    return rec[9];
}

void set_temp_range(u8 ID, u8 maxTemp)
{
    u8 databuf[13];
    u16 crc=0;

    databuf[0] = 0xff;
    databuf[1] = 0xff;
    databuf[2] = 0xfd;
    databuf[3] = 0x00;
    databuf[4] = ID;
    databuf[5] = 0x06;
    databuf[6] = 0x00;
    databuf[7] = 0x03;
    databuf[8] = 31;
    databuf[9] = 0x00;
    databuf[10]= maxTemp;

    crc=get_crc(databuf,11);
    databuf[11]= (u8)(crc & 0xff);
    databuf[12]= (u8)((crc>>8) & 0xff);

    sendbytes(databuf, 13);
}

void cur_mode(u8 ID)
{
    set_mode(ID,0);
}

void cur_modes(void)
{
    set_modes(0);
}

void set_current(u8 ID, u16 current)
{
    u8 databuf[14];
    u16 crc=0;

    databuf[0] = 0xff;
    databuf[1] = 0xff;
    databuf[2] = 0xfd;
    databuf[3] = 0x00;
    databuf[4] = ID;
    databuf[5] = 0x07;
    databuf[6] = 0x00;
    databuf[7] = 0x03;
    databuf[8] = 102;
    databuf[9] = 0x00;
    databuf[10]= (u8)( current      & 0xff);
    databuf[11]= (u8)((current>>8 ) & 0xff);

    crc=get_crc(databuf,12);
    databuf[12]= (u8)(crc & 0xff);
    databuf[13]= (u8)((crc>>8) & 0xff);

    sendbytes(databuf, 14);
}

void set_currents(u16 current)
{
    u8 databuf[ENG_NUM*(2+1)+14];
    u8 i = 0;
    u16 crc=0;

    databuf[0] = 0xff;
    databuf[1] = 0xff;
    databuf[2] = 0xfd;
    databuf[3] = 0x00;
    databuf[4] = 0xfe;
    databuf[5] = (2+1)*ENG_NUM+4+3;
    databuf[6] = 0x00;
    databuf[7] = 0x83;
    databuf[8] = 102;
    databuf[9] = 0x00;
    databuf[10]= 0x02;
    databuf[11]= 0x00;

    for(i=0; i<ENG_NUM; i++)
    {
        databuf[12+3*i]=i+1;
        databuf[12+3*i+1]= (u8)( current      & 0xff);
        databuf[12+3*i+2]= (u8)((current>>8 ) & 0xff);
    }

    crc=get_crc(databuf,ENG_NUM*(2+1)+12);
    databuf[ENG_NUM*(2+1)+12]= (u8)(crc & 0xff);
    databuf[ENG_NUM*(2+1)+13]= (u8)((crc>>8) & 0xff);

    sendbytes(databuf, ENG_NUM*(2+1)+14);
}

void set_current_range(u8 ID, u16 maxCurrent)
{
    u8 databuf[14];
    u16 crc=0;

    databuf[0] = 0xff;
    databuf[1] = 0xff;
    databuf[2] = 0xfd;
    databuf[3] = 0x00;
    databuf[4] = ID;
    databuf[5] = 0x07;
    databuf[6] = 0x00;
    databuf[7] = 0x03;
    databuf[8] = 38;
    databuf[9] = 0x00;
    databuf[10]= (u8)( maxCurrent      & 0xff);
    databuf[11]= (u8)((maxCurrent>>8 ) & 0xff);

    crc=get_crc(databuf,12);
    databuf[12]= (u8)(crc & 0xff);
    databuf[13]= (u8)((crc>>8) & 0xff);

    sendbytes(databuf, 14);
}

u16 read_current(u8 ID)
{
    u8 databuf[14];
    u8 rec[25];
    u16 crc=0;
    u16 current=0;

    databuf[0] = 0xff;
    databuf[1] = 0xff;
    databuf[2] = 0xfd;
    databuf[3] = 0x00;
    databuf[4] = ID;
    databuf[5] = 0x07;
    databuf[6] = 0x00;
    databuf[7] = 0x02;
    databuf[8] = 126;
    databuf[9] = 0x00;
    databuf[10]= 0x02;
    databuf[11]= 0x00;

    crc=get_crc(databuf,12);
    databuf[12]= (u8)(crc & 0xff);
    databuf[13]= (u8)((crc>>8) & 0xff);

    sendbytes(databuf, 14);
    readbytes(rec);
    current=(u16)rec[9] | (u16)(rec[10]<<8);

    printf("error = %d 速度 = %d\r\n", rec[8], current);
    return current;
}

void set_return_delay(u8 ID, u8 time)
{
    u8 databuf[13];
    u16 crc=0;

    databuf[0] = 0xff;
    databuf[1] = 0xff;
    databuf[2] = 0xfd;
    databuf[3] = 0x00;
    databuf[4] = ID;
    databuf[5] = 0x06;
    databuf[6] = 0x00;
    databuf[7] = 0x03;
    databuf[8] = 9;
    databuf[9] = 0x00;
    databuf[10]= time;

    crc=get_crc(databuf,11);
    databuf[11]= (u8)(crc & 0xff);
    databuf[12]= (u8)((crc>>8) & 0xff);

    sendbytes(databuf, 13);
}

void read_angles(float angles[ENG_NUM])
{
    u8 databuf[14+ENG_NUM];
    u8 rec[16*ENG_NUM];
    u16 crc=0;
    u8 i;
    u32 sangles[ENG_NUM];

    databuf[0] = 0xff;
    databuf[1] = 0xff;
    databuf[2] = 0xfd;
    databuf[3] = 0x00;
    databuf[4] = 0xfe;
    databuf[5] = ENG_NUM+4+3;
    databuf[6] = 0x00;
    databuf[7] = 0x82;
    databuf[8] = 132;
    databuf[9] = 0x00;
    databuf[10]= 0x04;
    databuf[11]= 0x00;

    for(i=0; i<ENG_NUM; i++)
        databuf[12+i] = 1+i;

    crc=get_crc(databuf,12+ENG_NUM);
    databuf[12+ENG_NUM]= (u8)(crc & 0xff);
    databuf[13+ENG_NUM]= (u8)((crc>>8) & 0xff);

    sendbytes(databuf, 14+ENG_NUM);
    delay_us(100*(ENG_NUM-1));
    readbytes(rec);

    for(i=0; i<ENG_NUM; i++)
    {
        sangles[i] =(u32)rec[9+i*15] | (u32)(rec[10+i*15]<<8) | (u32)(rec[11+i*15]<<16) | (u32)(rec[12+i*15]<<24);
        if(sangles[i]>=2048) angles[i]= (float)((sangles[i]-2048)%4095)*360.0/4095.0;
        else angles[i]=-(float)((2048-sangles[i])%4095)*360.0/4095.0;
    }
    //printf("位置为 %.2f°  %.2f°  %.2f°\r\n", angles[0],angles[1],angles[2]);
}

void read_speeds(float speeds[ENG_NUM])
{
    u8 databuf[14+ENG_NUM];
    u8 rec[16*ENG_NUM];
    u16 crc=0;
    u8 i;
    s32 sspeeds[ENG_NUM];

    databuf[0] = 0xff;
    databuf[1] = 0xff;
    databuf[2] = 0xfd;
    databuf[3] = 0x00;
    databuf[4] = 0xfe;
    databuf[5] = ENG_NUM+4+3;
    databuf[6] = 0x00;
    databuf[7] = 0x82;
    databuf[8] = 128;
    databuf[9] = 0x00;
    databuf[10]= 0x04;
    databuf[11]= 0x00;

    for(i=0; i<ENG_NUM; i++)
        databuf[12+i] = 1+i;

    crc=get_crc(databuf,12+ENG_NUM);
    databuf[12+ENG_NUM]= (u8)(crc & 0xff);
    databuf[13+ENG_NUM]= (u8)((crc>>8) & 0xff);

    sendbytes(databuf, 14+ENG_NUM);
    delay_us(100*(ENG_NUM-1));
    readbytes(rec);

    for(i=0; i<ENG_NUM; i++)
    {
        sspeeds[i]= (s32)rec[9+i*15] | (s32)(rec[10+i*15]<<8) | (s32)(rec[11+i*15]<<16) | (s32)(rec[12+i*15]<<24);
        speeds[i]= (float)sspeeds[i];
    }
    //printf("速度为 %.2f  %.2f  %.2f\r\n", speeds[0],speeds[1],speeds[2]);
}

void set_all(float ans[ENG_NUM+1])
{
    u8 databuf[10+(ENG_NUM+1)*9];
    u16 crc=0;
    u8 i;

    s32 sans[ENG_NUM+1];
    for(i=0; i<ENG_NUM; i++)
    {
        if(ans[i]> 180) ans[i]=180;
        else if(ans[i]<-180) ans[i]=-180;
        if(ans[i]<0)	sans[i] = 2048-(u32)((-ans[i]/360.0)*4095);
        else	sans[i] = 2048+(u32)((ans[i]/360.0)*4095);
    }
    if(ans[i]>1023) ans[i]=1023;
    else if(ans[i]<-1023) ans[i]=-1023;
    sans[i] = (s32)(ans[i]);

    databuf[0] = 0xff;
    databuf[1] = 0xff;
    databuf[2] = 0xfd;
    databuf[3] = 0x00;
    databuf[4] = 0xfe;
    databuf[5] = (ENG_NUM+1)*9+3;
    databuf[6] = 0x00;
    databuf[7] = 0x93;

    for(i=0; i<ENG_NUM; i++)
    {
        databuf[8+i*9] = i+1;
        databuf[9+i*9] = 116;
        databuf[10+i*9]= 0x00;
        databuf[11+i*9]= 0x04;
        databuf[12+i*9]= 0x00;
        databuf[13+i*9]= (u8)( sans[i]      & 0xff);
        databuf[14+i*9]= (u8)((sans[i]>>8 ) & 0xff);
        databuf[15+i*9]= (u8)((sans[i]>>16) & 0xff);
        databuf[16+i*9]= (u8)((sans[i]>>24) & 0xff);
    }

    databuf[8+i*9] = i+1;
    databuf[9+i*9] = 104;
    databuf[10+i*9]= 0x00;
    databuf[11+i*9]= 0x04;
    databuf[12+i*9]= 0x00;
    databuf[13+i*9]= (u8)( sans[i]      & 0xff);
    databuf[14+i*9]= (u8)((sans[i]>>8 ) & 0xff);
    databuf[15+i*9]= (u8)((sans[i]>>16) & 0xff);
    databuf[16+i*9]= (u8)((sans[i]>>24) & 0xff);

    crc=get_crc(databuf,8+(ENG_NUM+1)*9);
    databuf[8+(ENG_NUM+1)*9]= (u8)(crc & 0xff);
    databuf[9+(ENG_NUM+1)*9]= (u8)((crc>>8) & 0xff);

    sendbytes(databuf, 10+(ENG_NUM+1)*9);
}

void read_all(float sna[ENG_NUM+1])
{
    u8 databuf[10+(ENG_NUM+1)*5];
    u16 crc=0;
    u8 i;
    u8 rec[16*(ENG_NUM+1)];
    s32 ssna[ENG_NUM+1];

    databuf[0] = 0xff;
    databuf[1] = 0xff;
    databuf[2] = 0xfd;
    databuf[3] = 0x00;
    databuf[4] = 0xfe;
    databuf[5] = (ENG_NUM+1)*5+3;
    databuf[6] = 0x00;
    databuf[7] = 0x92;

    for(i=0; i<ENG_NUM; i++)
    {
        databuf[8+i*5] = i+1;
        databuf[9+i*5] = 132;
        databuf[10+i*5]= 0x00;
        databuf[11+i*5]= 0x04;
        databuf[12+i*5]= 0x00;
    }

    databuf[8+i*5] = i+1;
    databuf[9+i*5] = 128;
    databuf[10+i*5]= 0x00;
    databuf[11+i*5]= 0x04;
    databuf[12+i*5]= 0x00;

    crc=get_crc(databuf,8+(ENG_NUM+1)*5);
    databuf[8+(ENG_NUM+1)*5]= (u8)(crc & 0xff);
    databuf[9+(ENG_NUM+1)*5]= (u8)((crc>>8) & 0xff);

    sendbytes(databuf, 10+(ENG_NUM+1)*5);
    delay_us(100*(ENG_NUM));
    readbytes(rec);

    for(i=0; i<ENG_NUM; i++)
    {
        ssna[i] =(u32)rec[9+i*15] | (u32)(rec[10+i*15]<<8) | (u32)(rec[11+i*15]<<16) | (u32)(rec[12+i*15]<<24);
        if(ssna[i]>=2048) sna[i]= (float)((ssna[i]-2048)%4095)*360.0/4095.0;
        else sna[i]=-(float)((2048-ssna[i])%4095)*360.0/4095.0;
    }
    ssna[i]= (s32)rec[9+i*15] | (s32)(rec[10+i*15]<<8) | (s32)(rec[11+i*15]<<16) | (s32)(rec[12+i*15]<<24);
    sna[i]= (float)ssna[i];

    //printf("%.2f°  %.2f°  %.2f°  %.2f\r\n", sna[0],sna[1],sna[2],sna[3]);
}

void set_acc(u8 ID, float acc)
{
    u8 databuf[16];
    u16 crc=0;
    u32 sacc;
    sacc = (u32)(acc/214.577);

    databuf[0] = 0xff;
    databuf[1] = 0xff;
    databuf[2] = 0xfd;
    databuf[3] = 0x00;
    databuf[4] = ID;
    databuf[5] = 0x09;
    databuf[6] = 0x00;
    databuf[7] = 0x03;
    databuf[8] = 108;
    databuf[9] = 0x00;
    databuf[10]= (u8)( sacc      & 0xff);
    databuf[11]= (u8)((sacc>>8 ) & 0xff);
    databuf[12]= (u8)((sacc>>16) & 0xff);
    databuf[13]= (u8)((sacc>>24) & 0xff);

    crc=get_crc(databuf,14);
    databuf[14]= (u8)(crc & 0xff);
    databuf[15]= (u8)((crc>>8) & 0xff);

    sendbytes(databuf, 16);
}

void set_vel(u8 ID, float vel)
{
    u8 databuf[16];
    u16 crc=0;
    u32 svel;
    svel = (u32)(vel/0.229);

    databuf[0] = 0xff;
    databuf[1] = 0xff;
    databuf[2] = 0xfd;
    databuf[3] = 0x00;
    databuf[4] = ID;
    databuf[5] = 0x09;
    databuf[6] = 0x00;
    databuf[7] = 0x03;
    databuf[8] = 112;
    databuf[9] = 0x00;
    databuf[10]= (u8)( svel      & 0xff);
    databuf[11]= (u8)((svel>>8 ) & 0xff);
    databuf[12]= (u8)((svel>>16) & 0xff);
    databuf[13]= (u8)((svel>>24) & 0xff);

    crc=get_crc(databuf,14);
    databuf[14]= (u8)(crc & 0xff);
    databuf[15]= (u8)((crc>>8) & 0xff);

    sendbytes(databuf, 16);
}

void set_accs(float accs[ENG_NUM])
{
    u8 databuf[ENG_NUM*(4+1)+14];
    u8 i = 0;
    u16 crc=0;
    u32 saccs[ENG_NUM];
    for(i=0; i<ENG_NUM; i++)
        saccs[i] = (u32)(accs[i]/214.577);

    databuf[0] = 0xff;
    databuf[1] = 0xff;
    databuf[2] = 0xfd;
    databuf[3] = 0x00;
    databuf[4] = 0xfe;
    databuf[5] = (4+1)*ENG_NUM+4+3;
    databuf[6] = 0x00;
    databuf[7] = 0x83;
    databuf[8] = 108;
    databuf[9] = 0x00;
    databuf[10]= 0x04;
    databuf[11]= 0x00;

    for(i=0; i<ENG_NUM; i++)
    {
        databuf[12+5*i]=i+1;
        databuf[12+5*i+1]= (u8)( saccs[i]      & 0xff);
        databuf[12+5*i+2]= (u8)((saccs[i]>>8 ) & 0xff);
        databuf[12+5*i+3]= (u8)((saccs[i]>>16) & 0xff);
        databuf[12+5*i+4]= (u8)((saccs[i]>>24) & 0xff);
    }

    crc=get_crc(databuf,ENG_NUM*(4+1)+12);
    databuf[ENG_NUM*(4+1)+12]= (u8)(crc & 0xff);
    databuf[ENG_NUM*(4+1)+13]= (u8)((crc>>8) & 0xff);

    sendbytes(databuf, ENG_NUM*(4+1)+14);
}

void set_vels(float vels[ENG_NUM])
{
    u8 databuf[ENG_NUM*(4+1)+14];
    u8 i = 0;
    u16 crc=0;
    u32 svels[ENG_NUM];
    for(i=0; i<ENG_NUM; i++)
        svels[i] = (u32)(vels[i]/0.229);

    databuf[0] = 0xff;
    databuf[1] = 0xff;
    databuf[2] = 0xfd;
    databuf[3] = 0x00;
    databuf[4] = 0xfe;
    databuf[5] = (4+1)*ENG_NUM+4+3;
    databuf[6] = 0x00;
    databuf[7] = 0x83;
    databuf[8] = 112;
    databuf[9] = 0x00;
    databuf[10]= 0x04;
    databuf[11]= 0x00;

    for(i=0; i<ENG_NUM; i++)
    {
        databuf[12+5*i]=i+1;
        databuf[12+5*i+1]= (u8)( svels[i]      & 0xff);
        databuf[12+5*i+2]= (u8)((svels[i]>>8 ) & 0xff);
        databuf[12+5*i+3]= (u8)((svels[i]>>16) & 0xff);
        databuf[12+5*i+4]= (u8)((svels[i]>>24) & 0xff);
    }

    crc=get_crc(databuf,ENG_NUM*(4+1)+12);
    databuf[ENG_NUM*(4+1)+12]= (u8)(crc & 0xff);
    databuf[ENG_NUM*(4+1)+13]= (u8)((crc>>8) & 0xff);

    sendbytes(databuf, ENG_NUM*(4+1)+14);
}

void set_angle_VA(u8 ID, float angle, float acc, float vel)
{
    u8 databuf[25];
    u16 crc=0;
    u32 svel= (u32)(vel/0.229);
    u32 sacc= (u32)(acc/214.577);
    u32 sangle;
    float dec = angle - (int)angle;
    if(angle<0)
    {
        dec = -dec;
        angle = (int)(-angle)%360 +dec;
        angle = -angle;
    }
    else  angle = (int)angle%360 +dec;
    sangle = 2048+(u32)((angle/360.0)*4095.0);

    databuf[0] = 0xff;
    databuf[1] = 0xff;
    databuf[2] = 0xfd;
    databuf[3] = 0x00;
    databuf[4] = ID;
    databuf[5] = 17;
    databuf[6] = 0x00;
    databuf[7] = 0x03;
    databuf[8] = 108;
    databuf[9] = 0x00;
    databuf[10]= (u8)( sacc      & 0xff);
    databuf[11]= (u8)((sacc>>8 ) & 0xff);
    databuf[12]= (u8)((sacc>>16) & 0xff);
    databuf[13]= (u8)((sacc>>24) & 0xff);
    databuf[14]= (u8)( svel      & 0xff);
    databuf[15]= (u8)((svel>>8 ) & 0xff);
    databuf[16]= (u8)((svel>>16) & 0xff);
    databuf[17]= (u8)((svel>>24) & 0xff);
    databuf[18]= (u8)( sangle      & 0xff);
    databuf[19]= (u8)((sangle>>8 ) & 0xff);
    databuf[20]= (u8)((sangle>>16) & 0xff);
    databuf[21]= (u8)((sangle>>24) & 0xff);

    crc=get_crc(databuf,22);
    databuf[22]= (u8)(crc & 0xff);
    databuf[23]= (u8)((crc>>8) & 0xff);

    sendbytes(databuf, 24);
}

void set_speed_A(u8 ID, float speed, float acc)
{
    u8 databuf[20];
    u16 crc=0;
    u32 sspeed;
    u32 sacc= (u32)(acc/214.577);
    if(speed>1023) speed=1023;
    else if(speed<-1023) speed=-1023;
    sspeed = (u32)(speed);

    databuf[0] = 0xff;
    databuf[1] = 0xff;
    databuf[2] = 0xfd;
    databuf[3] = 0x00;
    databuf[4] = ID;
    databuf[5] = 13;
    databuf[6] = 0x00;
    databuf[7] = 0x03;
    databuf[8] = 104;
    databuf[9] = 0x00;
    databuf[10]= (u8)( sspeed      & 0xff);
    databuf[11]= (u8)((sspeed>>8 ) & 0xff);
    databuf[12]= (u8)((sspeed>>16) & 0xff);
    databuf[13]= (u8)((sspeed>>24) & 0xff);
    databuf[14]= (u8)( sacc      & 0xff);
    databuf[15]= (u8)((sacc>>8 ) & 0xff);
    databuf[16]= (u8)((sacc>>16) & 0xff);
    databuf[17]= (u8)((sacc>>24) & 0xff);

    crc=get_crc(databuf,18);
    databuf[18]= (u8)(crc & 0xff);
    databuf[19]= (u8)((crc>>8) & 0xff);

    sendbytes(databuf, 20);
}

void set_angles_VA(float angles[ENG_NUM], float accs[ENG_NUM], float vels[ENG_NUM])
{
    u8 databuf[ENG_NUM*13+14];
    u8 i = 0;
    u16 crc=0;
    u32 sangles[ENG_NUM];
    u32 saccs[ENG_NUM];
    u32 svels[ENG_NUM];

    for(i=0; i<ENG_NUM; i++)
    {
        if(angles[i]> 180) angles[i]=180;
        else if(angles[i]<-180) angles[i]=-180;
        if(angles[i]<0)	sangles[i] = 2048-(u32)((-angles[i]/360.0)*4095);
        else	sangles[i] = 2048+(u32)((angles[i]/360.0)*4095);
        svels[i] = (u32)(vels[i]/0.229);
        saccs[i] = (u32)(accs[i]/214.577);
    }

    databuf[0] = 0xff;
    databuf[1] = 0xff;
    databuf[2] = 0xfd;
    databuf[3] = 0x00;
    databuf[4] = 0xfe;
    databuf[5] = 13*ENG_NUM+4+3;
    databuf[6] = 0x00;
    databuf[7] = 0x83;
    databuf[8] = 108;
    databuf[9] = 0x00;
    databuf[10]= 12;
    databuf[11]= 0x00;

    for(i=0; i<ENG_NUM; i++)
    {
        databuf[12+13*i]=i+1;
        databuf[12+13*i+1]= (u8)( saccs[i]      & 0xff);
        databuf[12+13*i+2]= (u8)((saccs[i]>>8 ) & 0xff);
        databuf[12+13*i+3]= (u8)((saccs[i]>>16) & 0xff);
        databuf[12+13*i+4]= (u8)((saccs[i]>>24) & 0xff);
        databuf[16+13*i+1]= (u8)( svels[i]      & 0xff);
        databuf[16+13*i+2]= (u8)((svels[i]>>8 ) & 0xff);
        databuf[16+13*i+3]= (u8)((svels[i]>>16) & 0xff);
        databuf[16+13*i+4]= (u8)((svels[i]>>24) & 0xff);
        databuf[20+13*i+1]= (u8)( sangles[i]      & 0xff);
        databuf[20+13*i+2]= (u8)((sangles[i]>>8 ) & 0xff);
        databuf[20+13*i+3]= (u8)((sangles[i]>>16) & 0xff);
        databuf[20+13*i+4]= (u8)((sangles[i]>>24) & 0xff);
    }

    crc=get_crc(databuf,ENG_NUM*13+12);
    databuf[ENG_NUM*13+12]= (u8)(crc & 0xff);
    databuf[ENG_NUM*13+13]= (u8)((crc>>8) & 0xff);

    sendbytes(databuf, ENG_NUM*13+14);
}

void set_speeds_A(float speeds[ENG_NUM], float accs[ENG_NUM])
{
    u8 databuf[ENG_NUM*9+14];
    u8 i = 0;
    u16 crc=0;
    u32 sspeeds[ENG_NUM];
    u32 saccs[ENG_NUM];

    for(i=0; i<ENG_NUM; i++)
    {
        if(speeds[i]>1023) speeds[i]=1023;
        else if(speeds[i]<-1023) speeds[i]=-1023;
        sspeeds[i] = (u32)(speeds[i]);
        saccs[i] = (u32)(accs[i]/214.577);
    }

    databuf[0] = 0xff;
    databuf[1] = 0xff;
    databuf[2] = 0xfd;
    databuf[3] = 0x00;
    databuf[4] = 0xfe;
    databuf[5] = 9*ENG_NUM+4+3;
    databuf[6] = 0x00;
    databuf[7] = 0x83;
    databuf[8] = 104;
    databuf[9] = 0x00;
    databuf[10]= 12;
    databuf[11]= 0x00;

    for(i=0; i<ENG_NUM; i++)
    {
        databuf[12+13*i]=i+1;
        databuf[12+13*i+1]= (u8)( sspeeds[i]      & 0xff);
        databuf[12+13*i+2]= (u8)((sspeeds[i]>>8 ) & 0xff);
        databuf[12+13*i+3]= (u8)((sspeeds[i]>>16) & 0xff);
        databuf[12+13*i+4]= (u8)((sspeeds[i]>>24) & 0xff);
        databuf[16+13*i+1]= (u8)( saccs[i]      & 0xff);
        databuf[16+13*i+2]= (u8)((saccs[i]>>8 ) & 0xff);
        databuf[16+13*i+3]= (u8)((saccs[i]>>16) & 0xff);
        databuf[16+13*i+4]= (u8)((saccs[i]>>24) & 0xff);
    }

    crc=get_crc(databuf,ENG_NUM*9+12);
    databuf[ENG_NUM*9+12]= (u8)(crc & 0xff);
    databuf[ENG_NUM*9+13]= (u8)((crc>>8) & 0xff);

    sendbytes(databuf, ENG_NUM*9+14);
}

void set_all_VAV(float ans[ENG_NUM+1], float accs[ENG_NUM+1], float vels[ENG_NUM])
{
    u8 databuf[10+ENG_NUM*17+13];
    u16 crc=0;
    u8 i;
    u32 sans[ENG_NUM+1];
    u32 saccs[ENG_NUM+1];
    u32 svels[ENG_NUM];

    for(i=0; i<ENG_NUM; i++)
    {
        if(ans[i]> 180) ans[i]=180;
        else if(ans[i]<-180) ans[i]=-180;
        if(ans[i]<0)	sans[i] = 2048-(u32)((-ans[i]/360.0)*4095);
        else	sans[i] = 2048+(u32)((ans[i]/360.0)*4095);
        svels[i] = (u32)(vels[i]/0.229);
        saccs[i] = (u32)(accs[i]/214.577);
    }
    if(ans[i]>1023) ans[i]=1023;
    else if(ans[i]<-1023) ans[i]=-1023;
    sans[i] = (u32)(ans[i]);
    saccs[i]= (u32)((ans[i]/360.0)*4095.0);

    databuf[0] = 0xff;
    databuf[1] = 0xff;
    databuf[2] = 0xfd;
    databuf[3] = 0x00;
    databuf[4] = 0xfe;
    databuf[5] = ENG_NUM*17+13+3;
    databuf[6] = 0x00;
    databuf[7] = 0x93;

    for(i=0; i<ENG_NUM; i++)
    {
        databuf[8+i*17] = i+1;
        databuf[9+i*17] = 108;
        databuf[10+i*17]= 0x00;
        databuf[11+i*17]= 12;
        databuf[12+i*17]= 0x00;
        databuf[13+i*17]= (u8)( saccs[i]      & 0xff);
        databuf[14+i*17]= (u8)((saccs[i]>>8 ) & 0xff);
        databuf[15+i*17]= (u8)((saccs[i]>>16) & 0xff);
        databuf[16+i*17]= (u8)((saccs[i]>>24) & 0xff);
        databuf[17+i*17]= (u8)( svels[i]      & 0xff);
        databuf[18+i*17]= (u8)((svels[i]>>8 ) & 0xff);
        databuf[19+i*17]= (u8)((svels[i]>>16) & 0xff);
        databuf[20+i*17]= (u8)((svels[i]>>24) & 0xff);
        databuf[21+i*17]= (u8)( sans[i]      & 0xff);
        databuf[22+i*17]= (u8)((sans[i]>>8 ) & 0xff);
        databuf[23+i*17]= (u8)((sans[i]>>16) & 0xff);
        databuf[24+i*17]= (u8)((sans[i]>>24) & 0xff);
    }

    databuf[8+i*17] = i+1;
    databuf[9+i*17] = 104;
    databuf[10+i*17]= 0x00;
    databuf[11+i*17]= 0x08;
    databuf[12+i*17]= 0x00;
    databuf[13+i*17]= (u8)( sans[i]      & 0xff);
    databuf[14+i*17]= (u8)((sans[i]>>8 ) & 0xff);
    databuf[15+i*17]= (u8)((sans[i]>>16) & 0xff);
    databuf[16+i*17]= (u8)((sans[i]>>24) & 0xff);
    databuf[17+i*17]= (u8)( saccs[i]      & 0xff);
    databuf[18+i*17]= (u8)((saccs[i]>>8 ) & 0xff);
    databuf[19+i*17]= (u8)((saccs[i]>>16) & 0xff);
    databuf[20+i*17]= (u8)((saccs[i]>>24) & 0xff);

    crc=get_crc(databuf,8+ENG_NUM*17+13);
    databuf[8+ENG_NUM*17+13]= (u8)(crc & 0xff);
    databuf[9+ENG_NUM*17+13]= (u8)((crc>>8) & 0xff);

    sendbytes(databuf, 10+ENG_NUM*17+13);
}
