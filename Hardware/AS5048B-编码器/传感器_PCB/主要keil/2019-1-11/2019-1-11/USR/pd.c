#include "pbdata.h"

float incPD(PDstruct *PD, float setValue, float realValue)   //增量式PD。输出的是增量Δuk
{   
	float duk;
	float ek = setValue - realValue;
	duk = PD->kp * (ek - PD->ek1) + PD->kd * (ek - 2*PD->ek1 + PD->ek2);
	
	PD->ek2 = PD->ek1;
	PD->ek1 = ek;
	return duk;
} 

void PD_set(PDstruct *PD, float kp, float kd)      
{   
	PD->kp = kp;     
  PD->kd = kd;
}

void PD_init(PDstruct *PD, float kp, float kd)
{
	PD->ek1=0;
	PD->ek2=0;
	PD->kp = kp;     
  PD->kd = kd;
}

float Dspeed = 0;

/*void ctrl_speed(u8 ID, PDstruct *PD, float desiredF)
{
	float realF;
	float setF;
	
	realF = (float) AngleNow * K * L * PI /180.0;
	setF = (float) desiredF * Lf / L;
	
	if(AngleNow>=0) Dspeed += incPD(PD, setF, realF);
	else Dspeed += incPD(PD, -setF, realF);
	
	set_speed(ID, Dspeed);
}*/

