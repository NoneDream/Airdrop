#ifndef _SYSTEM_
#define _SYSTEM_

#include "r_cg_userdefine.h"

#define OPTIMIZE

#define DEBUG

	#ifdef DEBUG

	//#define DEBUG_MOT_H 0
	//#define DEBUG_MOT_X 0
	//#define DEBUG_MOT_Y 0
	#define DEBUG_MOT_Z 0

	#endif

#define SIGN_STOP_EM P5.2

#define MOT_MAX 3800
#define MOT_MIN 2020
#define PROTECT 60

//ϵͳ��ʼ��
void sysinit(void);

//����
void start(void);

//ֹͣ
void stop(void);

//���������pwm���������޷�
void mot_preoutput(void);

//��������������pwm�仯
void mot_output(void);

//�෭����
void protect(void);

#endif