#ifndef _GY95_
#define _GY95_

#include "r_cg_userdefine.h"

//��������������Ƿ��͸�λָ��
void sensorinit(void);

//��������ش�����������
void getangledata(void);

//��������������������ݲ��˲����������Դ��������˲����˺���ֻ�����˳�ƫ���ǵ��쳣ֵ��
void angle_filter(void);

//�����������ŷ����
angle *return_angle(void);

#endif