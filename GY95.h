#ifndef _GY95_
#define _GY95_

#include "r_cg_userdefine.h"

//这个函数向陀螺仪发送复位指令
void sensorinit(void);

//这个函数回传陀螺仪数据
void getangledata(void);

//这个函数解析陀螺仪数据并滤波（陀螺仪自带卡尔曼滤波，此函数只负责滤除偏航角的异常值）
void angle_filter(void);

//这个函数返回欧拉角
angle *return_angle(void);

#endif