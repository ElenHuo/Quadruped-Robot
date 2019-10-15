#ifndef _PD_H
#define _PD_H
#include "pbdata.h"

#define K 180        
#define L 18.05
#define PI 3.1415927
#define Lf 274


typedef struct
{
	float kp, kd;  //比例、微分系数
	float ek1, ek2;		//前一次、再前一次的误差ek
}PDstruct;        //声明一个结构体，用于保存PD相关参数

extern PDstruct* PD;
extern float Dspeed;  //设置的速度


float incPD(PDstruct *PD, float setValue, float realValue);   //增量式PD。输出的是增量Δuk
void PD_set(PDstruct *PD, float kp, float kd);   //调整PD参数
void PD_init(PDstruct *PD, float kp, float kd);  //初始化PD
void ctrl_speed(u8 ID, PDstruct *PD, float desiredF);  //用PD控制速度.desiredF是Fsea期望值


#endif
