#ifndef _RASPBERRY_PI_
#define _RASPBERRY_PI_

#include "r_cg_userdefine.h"

//准备接收上位机数据
void receive_start(void);

//检查上位机数据，成功返回0，失败返回-1，并无论如何复位串口
int receive_check(void);

//上位机数据解析
void translate(void);

//这个函数向上位机发送滤波后的欧拉角数据
void report_angle(void)；

//这个函数向上位机发送飞行器状态
void report(void);

//返回指令数据
remotedata returncommand(void);

#endif