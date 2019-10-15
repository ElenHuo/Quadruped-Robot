#ifndef _PD_H
#define _PD_H
#include "pbdata.h"

#define K 180        
#define L 18.05
#define PI 3.1415927
#define Lf 274


typedef struct
{
	float kp, kd;  //������΢��ϵ��
	float ek1, ek2;		//ǰһ�Ρ���ǰһ�ε����ek
}PDstruct;        //����һ���ṹ�壬���ڱ���PD��ز���

extern PDstruct* PD;
extern float Dspeed;  //���õ��ٶ�


float incPD(PDstruct *PD, float setValue, float realValue);   //����ʽPD���������������uk
void PD_set(PDstruct *PD, float kp, float kd);   //����PD����
void PD_init(PDstruct *PD, float kp, float kd);  //��ʼ��PD
void ctrl_speed(u8 ID, PDstruct *PD, float desiredF);  //��PD�����ٶ�.desiredF��Fsea����ֵ


#endif
