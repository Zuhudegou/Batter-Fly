#ifndef _flow_h_
#define _flow_h_

#include "stm32f10x.h"
#include "math.h"
#include "imu.h"
#include "Uart1.h"

#define ANG_2_RAD 0.0174f

//光流输出数据结构体
struct _pixel_flow_
{
	float x_i;			//x轴积分原始值
	float y_i;			//y轴积分原始值
	float fix_x_i;		//x轴积分滤波值
	float fix_y_i;		//y轴积分滤波值
	float ang_x;			
	float ang_y;		
	float gyr_x;			
	float gyr_y;		
	float out_x_i;			//x轴积分输出值
	float out_y_i;			//y轴积分输出值
	float out_x_i_o;			
	float out_y_i_o;		
	
	float dx;		
	float dy;		
	
	float x;				//x轴速度原始值
	float y;				//y轴速度原始值
	float fix_x;		//x轴速度融合值
	float fix_y;		//y轴速度融合值
	
	float loc_x;  //光流计算的当前坐标，作为位置pid的测量值和期望值
	float loc_y;  //光流计算的当前坐标，作为位置pid的测量值和期望值
	float loc_xs;
	float loc_ys;
	
	float loc_ang_x; //角度补偿换算为厘米值
	float loc_ang_y; //光流复位后使用
	
	float fix_High;
	
	
	u8 flow_pause; //光流暂停，替代利用flow_x_i=0来复位。
	float pause_desired_x;//修正的值，这个值将要变为0
	float pause_desired_y;//修正的值，这个值将要变为0
	
	u8 err1_cnt;
	u8 err2_cnt;
	

};
extern struct _pixel_flow_ pixel_flow;

//光流数据结构体
struct _flow_
{
	float flow_x; 	//原始数据
	float flow_y;		//原始数据
	float flow_x_i;	//原始数据积分值，屏蔽光流时，只是清空这个有用吗？角度补偿是否也要清空？
	float flow_y_i;
	
	float flow_x_iOUT;
	float flow_y_iOUT;
	
	
	u8 qual;
	u8 ok;
	float flow_High;
	u8 ssi;
	u8 ssi_cnt;
	

	
//	u8 err;
};


extern struct _flow_ mini;
extern struct _flow_ locat;
extern void Flow_Receive(u8 data);
extern void Flow_Receive1(u8 data);
extern void Pixel_Flow_Fix(float dT);
#endif


