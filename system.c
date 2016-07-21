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
extern angle angin;

//系统初始化
void sysinit(void)
{
	sensorinit();//陀螺仪复位
	get_data();//高度初始化
	init_hight=px4flow_data.ground_distance/10.0-3;
	R_TAU0_Channel0_Start();//开启pwm输出
}

//启动
void start(void)
{
	//积分项清零
	integral_h=0;
	integral.x=0;
	integral.y=0;
	integral.z=0;
	
	//设置初始目标值
	mot_min_h=1000;
	command_buf.h=10;
	
	//开启定时器中断
	R_TAU0_Channel5_Start();
	
	//设置标志位
	sign_stop=0;
}

//停止
void stop(void)
{
	//设置标志位
	sign_stop=1;
	
	//关闭定时器中断
	R_TAU0_Channel5_Stop();
	
	//积分项清零
	command_buf.h=0;
	integral_h=0;
	integral.x=0;
	integral.y=0;
	integral.z=0;
	mot_min_h=100;
	
	//关闭电机
	TDR01=2000;TDR02=2000;TDR03=2000;TDR04=2000;
}

//这个函数对pwm进行最终限幅
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

//这个函数真正输出pwm变化
void mot_output(void)
{
	TDR01=output_buffer.TDR1;
	TDR02=output_buffer.TDR2;
	TDR03=output_buffer.TDR3;
	TDR04=output_buffer.TDR4;
}

//侧翻保护
void protect()
{
    if(angin.x>PROTECT)sign_stop=true;
    if(angin.x<-PROTECT)sign_stop=true;
    if(angin.y>PROTECT)sign_stop=true;
    if(angin.y<-PROTECT)sign_stop=true;
}