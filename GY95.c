#include "r_cg_macrodriver.h"
#include "r_cg_serial.h"
#include "delay.h"
#include "GY95.h"
#include "system.h"

//宏定义实现高低位交换
#define SWAP8(x) (((x&0xff00)>> 8)|((x&0xff)<<8))

gy95_frame sensordata;

angle angin；//陀螺仪原始数据
angle angnow；//滤波之后数据
angle angle_before;//前一周期的数据

const unsigned char output_angle[3]={0xa5,0x95,0x3a};

//UART0收发标志
extern bool sendwait0,receivewait0;

//这个函数向陀螺仪发送复位指令
void sensorinit(void)
{
	unsigned char tx_dat[3]={0xa5,0x57,0xfc};//磁场校准指令
	
	R_UART0_Start();
	
	sendwait0=true;
	R_UART0_Send(tx_dat,3);
	while(sendwait0){NOP();}
	
	delay_ms(3000);
	
	tx_dat[0]=0xa5,tx_dat[1]=0x58,tx_dat[2]=0xfd;//加速度计陀螺仪校准指令
	sendwait0=true;
	R_UART0_Send(tx_dat,3);
	while(sendwait0){NOP();}
	
	R_UART0_Stop();
}

//这个函数回传陀螺仪数据
void getangledata(void)
{
	R_UART0_Start();
	sendwait0=true;
	R_UART0_Send(output_angle,3);
	receivewait0=true;
	R_UART0_Receive((uint8_t *)&sensordata,11);
	while(sendwait0){NOP();}
	
#ifndef OPTIMIZE
	while(receivewait0){NOP();}
	R_UART0_Stop();
#endif
}

//这个函数解析陀螺仪数据并滤波（陀螺仪自带卡尔曼滤波，此函数只负责滤除偏航角的异常值）
void angle_filter(void)
{
	angle deltaang;
	
#ifdef OPTIMIZE
	while(receivewait0){NOP();}
	R_UART0_Stop();
#endif
	
	getangledata();
	
	sensordata.x=SWAP8(sensordata.x);
	sensordata.y=SWAP8(sensordata.y);
	sensordata.z=SWAP8(sensordata.z);
	
	angin.x=sensordata.x/100.0;
	angin.y=sensordata.y/100.0;
	angin.z=sensordata.z/100.0;
	
	angnow.x=angin.x;
	angnow.y=angin.y;
	deltaang.z=angin.z-angle_before.z;
	if(deltaang.z>-5&&deltaang.z<5)angnow.z=angin.z;//(angin.z*0.8)+(angle_before.z*0.2);
	else angnow.z=angle_before.z;
	
	angnow.z=angin.z；//调试用！！！
	
	angle_before.x=angnow.x;
	angle_before.y=angnow.y;
	angle_before.z=angnow.z;
}

//这个函数返回欧拉角
angle *return_angle(void)
{
	return &angnow;
}