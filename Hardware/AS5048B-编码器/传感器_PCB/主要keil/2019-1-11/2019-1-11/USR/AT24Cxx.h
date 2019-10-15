#ifndef _AT24Cxx_H
#define _AT24Cxx_H
#include "pbdata.h"


#define ENG_NUM    6   //�ܶ����-1��Ҳ����λ��ģʽ�ĵ����
void set_ID(u8 ID, u8 newID);
void reboot(u8 ID);											//����
void set_baud(u8 ID,u8 baud);  					//�趨������  baud=0:9600  1:57600  2:115200  3:1M  4:2M  5:3M  6:4M  7:4.5M
void set_bauds(u8 baud);      					//ȫ���趨������

void enable(u8 ID,u8 status);  					//����/�ر�Ť�أsstatus=0Ϊ�ر�
void enables(u8 status);     						//ȫ������/�ر�Ť��

void set_LED(u8 ID,u8 status);					//LED��
u8 read_error(u8 ID);										//��ȡ����״̬
u8 read_temp(u8 ID);										//��ȡ�¶�
void set_temp_range(u8 ID, u8 maxTemp);	//�趨�¶ȷ�Χ
void set_return_delay(u8 ID,u8 time);		//�趨״̬������ʱ


void set_mode(u8 ID, u8 mode);  //ģʽ���� mode=0:Ť�� 1:�ٶ� 3:λ�� 4:�ⲿλ�� 5:����Ť�ص�λ�� 16:PWM
void set_modes(u8 mode);				//ȫ��ģʽ����
u8 read_mode(u8 ID);						//��ȡģʽ����
void vel_mode(u8 ID);      					      //�ٶ�ģʽ
void vel_modes(void);											//ȫ����Ϊ�ٶ�ģʽ
void pos_mode(u8 ID);											//λ��ģʽ
void pos_modes(void);											//ȫ����Ϊλ��ģʽ
void cur_mode(u8 ID);											//Ť�أ�������ģʽ
void cur_modes(void);											//ȫ����ΪŤ��ģʽ

void set_angle(u8 ID,float angle);													//�趨λ�ã���λ��1��
void set_same_angles(float angle);													//ȫ���趨��ͬ��λ��
void set_angles(float angles[ENG_NUM]);											//ȫ���趨��ͬ��λ��
void set_angle_range(u8 ID, u32 minAngle, u32 maxAngle);  	//�޶�λ�÷�Χ�����뷶Χ��0~4095
float read_angle(u8 ID);																		//��ȡλ��
void read_angles(float angles[ENG_NUM]);										//ȫ����ȡλ��

void set_speed(u8 ID, float speed);  											//�趨�ٶȣ���λ��0.229rpm�����234.267
void set_same_speeds(float speed);													//ȫ���趨��ͬ���ٶ�
void set_speeds(float speeds[ENG_NUM]);											//ȫ���趨��ͬ���ٶ�
void set_speed_range(u8 ID, u32 maxSpeed); 								//�޶��ٶȷ�Χ�����뷶Χ��0~1023
float read_speed(u8 ID);																	//��ȡ�ٶ�
void read_speeds(float speeds[ENG_NUM]);									//ȫ����ȡ�ٶ�

void set_all(float ans[ENG_NUM+1]);							//ͬʱ�趨λ��ģʽ�����λ�ú��ٶ�ģʽ���ٶ�
void read_all(float sna[ENG_NUM+1]);							//ͬʱ��ȡλ��ģʽ�����λ�ú��ٶ�ģʽ���ٶ�

void set_acc(u8 ID, float acc);					//�趨���ٶȣ���λ��1rev/min^2�����7031044
void set_vel(u8 ID, float vel);					//λ��ģʽ���趨�ٶȣ���λ��1rpm�����7503.643
void set_accs(float accs[ENG_NUM]);			//ȫ���趨��ͬ�ļ��ٶ�
void set_vels(float vels[ENG_NUM]);			//λ��ģʽ��ȫ���趨��ͬ���ٶ�

void set_angle_VA(u8 ID, float angle, float acc, float vel);   //�趨λ����ͬʱ�趨���ٶȺ��ٶ�
void set_angles_VA(float angles[ENG_NUM], float accs[ENG_NUM], float vels[ENG_NUM]);   //ȫ���趨λ����ͬʱ�趨���ٶȺ��ٶ�
void set_speed_A(u8 ID, float speed, float acc);						//�趨�ٶ���ͬʱ�趨���ٶ�
void set_speeds_A(float speeds[ENG_NUM], float accs[ENG_NUM]);		//ȫ���趨�ٶ���ͬʱ�趨���ٶ�
void set_all_VAV(float ans[ENG_NUM+1], float accs[ENG_NUM+1], float vels[ENG_NUM]); //ͬʱ�趨λ��ģʽ�����λ�ú��ٶ�ģʽ���ٶ��Ҹ������ٶȺ��ٶ�

void set_current(u8 ID, u16 current);  										//�趨��������λ��2.69mA
void set_currents(u16 current);				 										//ȫ���趨����
void set_current_range(u8 ID, u16 maxCurrent); 						//�޶�������Χ�����뷶Χ��0~2047
u16 read_current(u8 ID);																	//��ȡ����

#endif

