#include "r_cg_macrodriver.h"
#include "r_cg_timer.h"
#include "px4flow.h"
#include "attitude_control.h"
#include "raspberry_pi.h"
#include "r_cg_serial.h"
#include "delay.h"

const char adr=0x84;
const char flow_tx=0x00;
float hight_before=0,init_hight;
px4flow_frame px4flow_data;
extern bool iicsend,iicreceive;

void send_com()
{
	iicsend=1;
	R_IICA0_Master_Send(adr, （uint8_t *）&flow_tx, 1, 40);
	while(iicsend){NOP();}
}

void get_data()
{
	iicsend=1;
	R_IICA0_Master_Send(adr, （uint8_t *）&flow_tx, 1, 40);
	while(iicsend){NOP();}
	delay_ms(2);
	iicreceive=1;
	R_IICA0_Master_Receive(adr, (uint8_t *)&px4flow_data, 22, 40);
	while(iicreceive){NOP();}
}

float hight_filter()
{
	float hightbuf;
	float hight;

	hightbuf=（(float)px4flow_data.ground_distance）/10;
	hight=(hightbuf-init_hight)*0.8;//(hight_before*0.2)+((hightbuf-init_hight)*0.8);/*减去初始高度*/
	
	hight_before=hight;
	
	return hight;
}