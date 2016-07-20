#include "r_cg_macrodriver.h"
#include "r_cg_serial.h"
#include "delay.h"
#include "GY95.h"

#define SWAP8(x) (((x&0xff00)>> 8)|((x&0xff)<<8))

gy95_frame sensordata;
angle angin,angnow,angle_before;
//const unsigned char bps_115200[3]={0xa5,0xaf,0x54};
const unsigned char output_angle[3]={0xa5,0x95,0x3a};
extern bool sendwait0,receivewait0;

void sensorinit(void)
{
	unsigned char tx_dat[3]={0xa5,0x57,0xfc};
	R_UART0_Start();
	R_UART0_Send(tx_dat,3);
	delay_ms(3000);
	tx_dat[0]=0xa5,tx_dat[1]=0x58,tx_dat[2]=0xfd;
	sendwait0=true;
	R_UART0_Send(tx_dat,3);
	while(sendwait0){NOP();}
	R_UART0_Stop();
}
void getangledata(void)
{
	R_UART0_Start();
	sendwait0=true;
	R_UART0_Send(output_angle,3);
	while(sendwait0){NOP();}
	receivewait0=true;
	R_UART0_Receive((uint8_t *)&sensordata,11);
	while(receivewait0){NOP();}
	R_UART0_Stop();
}

angle * angle_filter(void)
{
	angle deltaang;
	
	getangledata();
	
	sensordata.x=SWAP8(sensordata.x);
	sensordata.y=SWAP8(sensordata.y);
	sensordata.z=SWAP8(sensordata.z);
	
	angin.x=sensordata.x/100.0;
	angin.y=sensordata.y/100.0;
	angin.z=sensordata.z/100.0;
	
	/*deltaang.x=angin.x-angle_before.x;
	if(deltaang.x>-5&&deltaang.x<5)angnow.x=(angin.x*0.8)+(angle_before.x*0.2);
	else angnow.x=angle_before.x;
	deltaang.y=angin.y-angle_before.y;
	if(deltaang.y>-5&&deltaang.y<5)angnow.y=(angin.y*0.8)+(angle_before.y*0.2);
	else angnow.y=angle_before.y;*/
	angnow.x=angin.x;
	angnow.y=angin.y;
	deltaang.z=angin.z-angle_before.z;
	if(deltaang.z>-5&&deltaang.z<5)angnow.z=(angin.z*0.8)+(angle_before.z*0.2);
	else angnow.z=angle_before.z;
	
	angle_before.x=angnow.x;
	angle_before.y=angnow.y;
	angle_before.z=angnow.z;
	
	return &angle_before;
}