#ifndef _ATTITUDE_CONTROL_
#define _ATTITUDE_CONTROL_

#include "r_cg_userdefine.h"

//双环pid
int duallooppid(ppid par,float *delta0,float *data0,float data,float aim,float *integral);

//计算角度pid和高度pid的输出得到最终输出
void cal_mot_all(void);

//限幅，用于角度pid的输出
void limitang(motor *mot_in,int motrlim);

//返回电机pwm值
motor *getmot(void);

#endif