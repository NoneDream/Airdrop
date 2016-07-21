/***********************************************************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only 
* intended for use with Renesas products. No other uses are authorized. This 
* software is owned by Renesas Electronics Corporation and is protected under 
* all applicable laws, including copyright laws.
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING 
* THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT 
* LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE 
* AND NON-INFRINGEMENT.  ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.
* TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS 
* ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE 
* FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR 
* ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE 
* BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software 
* and to discontinue the availability of this software.  By using this software, 
* you agree to the additional terms and conditions found by accessing the 
* following link:
* http://www.renesas.com/disclaimer
*
* Copyright (C) 2011, 2013 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/

/***********************************************************************************************************************
* File Name    : r_cg_userdefine.h
* Version      : CodeGenerator for RL78/G13 V2.00.00.07 [22 Feb 2013]
* Device(s)    : R5F100LE
* Tool-Chain   : CA78K0R
* Description  : This file includes user definition.
* Creation Date: 2016/7/21
***********************************************************************************************************************/

#ifndef _USER_DEF_H
#define _USER_DEF_H

/***********************************************************************************************************************
User definitions
***********************************************************************************************************************/

/* Start user code for function. Do not edit comment generated here */
#define LED P13.0

//布尔型定义
typedef enum {true=1, false=0} bool;

//双环pid系数
typedef struct
{
	float waihuan_p;
	float neihuan_p;
	float neihuan_i;
	float neihuan_d;
	int i_limit;//积分项限幅
}ppid;

//角度数据
typedef struct
{
	float x;
	float y;
	float z;
}angle;

//pwm参数
typedef struct
{
	int TDR1;
	int TDR2;
	int TDR3;
	int TDR4;
}motor;

//控制数据
typedef struct
{
	float x;
	float y;
	float h;
	float vz;
}remotedata;

//陀螺仪数据
typedef struct
{
	char head[2];//固定0x5A,0x5A
	char type;//0x45表示欧拉角数据
	unsigned char len;//数据长度，应为6
	int16_t x;
	int16_t y;
	int16_t z;
	unsigned char checksum;//数据的校验和
}gy95_frame;

//光流数据
typedef struct
{
	uint16_t frame_count;// counts created I2C frames [#frames]
	int16_t pixel_flow_x_sum;// latest x flow measurement in pixels*10 [pixels]
	int16_t pixel_flow_y_sum;// latest y flow measurement in pixels*10 [pixels]
	int16_t flow_comp_m_x;// x velocity*1000 [meters/sec]
	int16_t flow_comp_m_y;// y velocity*1000 [meters/sec]
	int16_t qual;// Optical flow quality / confidence [0: bad, 255: maximum quality]
	int16_t gyro_x_rate; // latest gyro x rate [rad/sec]
	int16_t gyro_y_rate; // latest gyro y rate [rad/sec]
	int16_t gyro_z_rate; // latest gyro z rate [rad/sec]
	uint8_t gyro_range; // gyro range [0 .. 7] equals [50 deg/sec .. 2000 deg/sec] 
	uint8_t sonar_timestamp;// time since last sonar update [milliseconds]
	int16_t ground_distance;// Ground distance in meters*1000 [meters]. Positive value: distance known. Negative value: Unknown distance
} px4flow_frame;

//上位机数据
typedef struct
{
	char head[2];//0xff,0x00
	char flag;//0x01:command; 0x02:parameter_r; 0x03:parameter_z; 0x04:parameter_h; 0x11:start; 0x22:stop;
	int16_t data1;//x; op;
	int16_t data2;//y; ip;
	int16_t data3;//vz; ii;
	int16_t data4;//h; id
	unsigned char end;
}pi_frame;
/* End user code. Do not edit comment generated here */
#endif
