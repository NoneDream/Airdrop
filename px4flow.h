#ifndef _HIGHT_
#define _HIGHT_

#include "r_cg_userdefine.h"

//extern float hight0,oph,iph,iih,idh,inaim0h,integralh;
//extern int limh;
//extern int motout_hit;

//发送请求
void send_com(void);

//获取传感器数据
void get_data(void);

//高度数据解析和滤波（单位：厘米）
void hight_filter(void);

//这个函数返回高度
float return_hight(void);

#endif