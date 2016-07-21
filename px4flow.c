#include "r_cg_macrodriver.h"
#include "r_cg_timer.h"
#include "px4flow.h"
#include "attitude_control.h"
#include "raspberry_pi.h"
#include "r_cg_serial.h"
#include "delay.h"

const char adr=0x84;
const char flow_tx=0x00;
float hight_before=0,init_hight,hight;
px4flow_frame px4flow_data;
extern bool iicsend,iicreceive;

//发送请求
void send_com(void)
{
	iicsend=1;
	R_IICA0_Master_Send(adr, （uint8_t *）&flow_tx, 1, 40);
	while(iicsend){NOP();}
}

//获取传感器数据
void get_data(void)
{
	iicsend=1;
	R_IICA0_Master_Send(adr, （uint8_t *）&flow_tx, 1, 40);
	while(iicsend){NOP();}
	
#ifndef OPTIMIZE
	delay_ms(2);
	iicreceive=1;
	R_IICA0_Master_Receive(adr, (uint8_t *)&px4flow_data, 22, 40);
	while(iicreceive){NOP();}
#endif
}

//高度数据解析和滤波（单位：厘米）
void hight_filter(void)
{
	float hightbuf;
	
#ifdef OPTIMIZE
	iicreceive=1;
	R_IICA0_Master_Receive(adr, (uint8_t *)&px4flow_data, 22, 40);
	while(iicreceive){NOP();}
#endif

	hightbuf=（(float)px4flow_data.ground_distance）/10;
	hight=(hightbuf-init_hight)*0.8;//(hight_before*0.2)+((hightbuf-init_hight)*0.8);/*减去初始高度*/
	
	hight_before=hight;
}

//这个函数返回高度
float return_hight(void)
{
	return hight;
}