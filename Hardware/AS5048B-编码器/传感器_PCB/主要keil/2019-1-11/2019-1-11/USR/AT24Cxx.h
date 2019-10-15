#ifndef _AT24Cxx_H
#define _AT24Cxx_H
#include "pbdata.h"


#define ENG_NUM    6   //总舵机数-1，也就是位置模式的电机数
void set_ID(u8 ID, u8 newID);
void reboot(u8 ID);											//重启
void set_baud(u8 ID,u8 baud);  					//设定波特率  baud=0:9600  1:57600  2:115200  3:1M  4:2M  5:3M  6:4M  7:4.5M
void set_bauds(u8 baud);      					//全部设定波特率

void enable(u8 ID,u8 status);  					//开启/关闭扭矩status=0为关闭
void enables(u8 status);     						//全部开启/关闭扭矩

void set_LED(u8 ID,u8 status);					//LED灯
u8 read_error(u8 ID);										//读取错误状态
u8 read_temp(u8 ID);										//读取温度
void set_temp_range(u8 ID, u8 maxTemp);	//设定温度范围
void set_return_delay(u8 ID,u8 time);		//设定状态传回延时


void set_mode(u8 ID, u8 mode);  //模式设置 mode=0:扭矩 1:速度 3:位置 4:外部位置 5:基于扭矩的位置 16:PWM
void set_modes(u8 mode);				//全部模式设置
u8 read_mode(u8 ID);						//读取模式设置
void vel_mode(u8 ID);      					      //速度模式
void vel_modes(void);											//全部设为速度模式
void pos_mode(u8 ID);											//位置模式
void pos_modes(void);											//全部设为位置模式
void cur_mode(u8 ID);											//扭矩（电流）模式
void cur_modes(void);											//全部设为扭矩模式

void set_angle(u8 ID,float angle);													//设定位置，单位：1°
void set_same_angles(float angle);													//全部设定相同的位置
void set_angles(float angles[ENG_NUM]);											//全部设定不同的位置
void set_angle_range(u8 ID, u32 minAngle, u32 maxAngle);  	//限定位置范围，输入范围：0~4095
float read_angle(u8 ID);																		//读取位置
void read_angles(float angles[ENG_NUM]);										//全部读取位置

void set_speed(u8 ID, float speed);  											//设定速度，单位：0.229rpm，最大234.267
void set_same_speeds(float speed);													//全部设定相同的速度
void set_speeds(float speeds[ENG_NUM]);											//全部设定不同的速度
void set_speed_range(u8 ID, u32 maxSpeed); 								//限定速度范围，输入范围：0~1023
float read_speed(u8 ID);																	//读取速度
void read_speeds(float speeds[ENG_NUM]);									//全部读取速度

void set_all(float ans[ENG_NUM+1]);							//同时设定位置模式电机的位置和速度模式的速度
void read_all(float sna[ENG_NUM+1]);							//同时读取位置模式电机的位置和速度模式的速度

void set_acc(u8 ID, float acc);					//设定加速度，单位：1rev/min^2，最大7031044
void set_vel(u8 ID, float vel);					//位置模式下设定速度，单位：1rpm，最大7503.643
void set_accs(float accs[ENG_NUM]);			//全部设定不同的加速度
void set_vels(float vels[ENG_NUM]);			//位置模式下全部设定不同的速度

void set_angle_VA(u8 ID, float angle, float acc, float vel);   //设定位置且同时设定加速度和速度
void set_angles_VA(float angles[ENG_NUM], float accs[ENG_NUM], float vels[ENG_NUM]);   //全部设定位置且同时设定加速度和速度
void set_speed_A(u8 ID, float speed, float acc);						//设定速度且同时设定加速度
void set_speeds_A(float speeds[ENG_NUM], float accs[ENG_NUM]);		//全部设定速度且同时设定加速度
void set_all_VAV(float ans[ENG_NUM+1], float accs[ENG_NUM+1], float vels[ENG_NUM]); //同时设定位置模式电机的位置和速度模式的速度且附带加速度和速度

void set_current(u8 ID, u16 current);  										//设定电流，单位：2.69mA
void set_currents(u16 current);				 										//全部设定电流
void set_current_range(u8 ID, u16 maxCurrent); 						//限定电流范围，输入范围：0~2047
u16 read_current(u8 ID);																	//读取电流

#endif

