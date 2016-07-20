#include "r_cg_macrodriver.h"
#include "system.h"
#include "r_cg_serial.h"
#include "r_cg_timer.h"
#include "r_cg_it.h"
#include "attitude_control.h"
#include "px4flow.h"
#include "GY95.h"
#include "raspberry_pi.h"

extern motor motout;
motor output_buffer;
int hight_buffer;
extern float init_hight;
bool sign_stop=1;
extern px4flow_frame px4flow_data;
extern remotedata command_buf;
extern float integral_h;
extern int mot_min_h;
extern angle integral;

void sysinit(void)
{
	sensorinit();
	get_data();
	init_hight=px4flow_data.ground_distance/10.0-3;
	R_TAU0_Channel0_Start();
}

void start(void)
{
	//R_TAU0_Channel0_Start();
	mot_min_h=1000;
	command_buf.h=10;
	R_TAU0_Channel5_Start();
	//R_TAU0_Channel6_Start();
	//R_IT_Start();
	sign_stop=0;
}

void stop(void)
{
	sign_stop=1;
	R_TAU0_Channel5_Stop();
	command_buf.h=0;
	integral_h=0;
	integral.x=0;
	integral.y=0;
	integral.z=0;
	mot_min_h=100;
	//R_TAU0_Channel6_Stop();
	//R_IT_Stop();
	TDR01=2000;TDR02=2000;TDR03=2000;TDR04=2000;
}

void mot_preoutput(void)
{
	if(motout.TDR1<MOT_MIN)output_buffer.TDR1=MOT_MIN;
	else if(motout.TDR1>MOT_MAX)output_buffer.TDR1=MOT_MAX;
	else output_buffer.TDR1=motout.TDR1;
	
	if(motout.TDR2<MOT_MIN)output_buffer.TDR2=MOT_MIN;
	else if(motout.TDR2>MOT_MAX)output_buffer.TDR2=MOT_MAX;
	else output_buffer.TDR2=motout.TDR2;
	
	if(motout.TDR3<MOT_MIN)output_buffer.TDR3=MOT_MIN;
	else if(motout.TDR3>MOT_MAX)output_buffer.TDR3=MOT_MAX;
	else output_buffer.TDR3=motout.TDR3;
	
	if(motout.TDR4<MOT_MIN)output_buffer.TDR4=MOT_MIN;
	else if(motout.TDR4>MOT_MAX)output_buffer.TDR4=MOT_MAX;
	else output_buffer.TDR4=motout.TDR4;
}

void mot_output(void)
{
	TDR01=output_buffer.TDR1;
	TDR02=output_buffer.TDR2;
	TDR03=output_buffer.TDR3;
	TDR04=output_buffer.TDR4;
}

void report()
{
	//R_UART1_Send();
}