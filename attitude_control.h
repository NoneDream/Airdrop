#ifndef _ATTITUDE_CONTROL_
#define _ATTITUDE_CONTROL_

#include "r_cg_userdefine.h"

//˫��pid
int duallooppid(ppid par,float *delta0,float *data0,float data,float aim,float *integral);

//����Ƕ�pid�͸߶�pid������õ��������
void cal_mot_all(void);

//�޷������ڽǶ�pid�����
void limitang(motor *mot_in,int motrlim);

//���ص��pwmֵ
motor *getmot(void);

#endif