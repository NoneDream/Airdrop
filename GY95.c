#include "r_cg_macrodriver.h"
#include "r_cg_serial.h"
#include "delay.h"
#include "GY95.h"
#include "system.h"

//�궨��ʵ�ָߵ�λ����
#define SWAP8(x) (((x&0xff00)>> 8)|((x&0xff)<<8))

gy95_frame sensordata;

angle angin��//������ԭʼ����
angle angnow��//�˲�֮������
angle angle_before;//ǰһ���ڵ�����

const unsigned char output_angle[3]={0xa5,0x95,0x3a};

//UART0�շ���־
extern bool sendwait0,receivewait0;

//��������������Ƿ��͸�λָ��
void sensorinit(void)
{
	unsigned char tx_dat[3]={0xa5,0x57,0xfc};//�ų�У׼ָ��
	
	R_UART0_Start();
	
	sendwait0=true;
	R_UART0_Send(tx_dat,3);
	while(sendwait0){NOP();}
	
	delay_ms(3000);
	
	tx_dat[0]=0xa5,tx_dat[1]=0x58,tx_dat[2]=0xfd;//���ٶȼ�������У׼ָ��
	sendwait0=true;
	R_UART0_Send(tx_dat,3);
	while(sendwait0){NOP();}
	
	R_UART0_Stop();
}

//��������ش�����������
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

//��������������������ݲ��˲����������Դ��������˲����˺���ֻ�����˳�ƫ���ǵ��쳣ֵ��
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
	
	angnow.z=angin.z��//�����ã�����
	
	angle_before.x=angnow.x;
	angle_before.y=angnow.y;
	angle_before.z=angnow.z;
}

//�����������ŷ����
angle *return_angle(void)
{
	return &angnow;
}