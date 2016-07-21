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

//系统初始化
void sysinit(void);

//启动
void start(void);

//停止
void stop(void);

//这个函数对pwm进行最终限幅
void mot_preoutput(void);

//这个函数真正输出pwm变化
void mot_output(void);

//侧翻保护
void protect(void);

#endif