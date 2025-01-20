#ifndef _flow_h_
#define _flow_h_

#include "stm32f10x.h"
#include "math.h"
#include "imu.h"
#include "Uart1.h"

#define ANG_2_RAD 0.0174f

//����������ݽṹ��
struct _pixel_flow_
{
	float x_i;			//x�����ԭʼֵ
	float y_i;			//y�����ԭʼֵ
	float fix_x_i;		//x������˲�ֵ
	float fix_y_i;		//y������˲�ֵ
	float ang_x;			
	float ang_y;		
	float gyr_x;			
	float gyr_y;		
	float out_x_i;			//x��������ֵ
	float out_y_i;			//y��������ֵ
	float out_x_i_o;			
	float out_y_i_o;		
	
	float dx;		
	float dy;		
	
	float x;				//x���ٶ�ԭʼֵ
	float y;				//y���ٶ�ԭʼֵ
	float fix_x;		//x���ٶ��ں�ֵ
	float fix_y;		//y���ٶ��ں�ֵ
	
	float loc_x;  //��������ĵ�ǰ���꣬��Ϊλ��pid�Ĳ���ֵ������ֵ
	float loc_y;  //��������ĵ�ǰ���꣬��Ϊλ��pid�Ĳ���ֵ������ֵ
	float loc_xs;
	float loc_ys;
	
	float loc_ang_x; //�ǶȲ�������Ϊ����ֵ
	float loc_ang_y; //������λ��ʹ��
	
	float fix_High;
	
	
	u8 flow_pause; //������ͣ���������flow_x_i=0����λ��
	float pause_desired_x;//������ֵ�����ֵ��Ҫ��Ϊ0
	float pause_desired_y;//������ֵ�����ֵ��Ҫ��Ϊ0
	
	u8 err1_cnt;
	u8 err2_cnt;
	

};
extern struct _pixel_flow_ pixel_flow;

//�������ݽṹ��
struct _flow_
{
	float flow_x; 	//ԭʼ����
	float flow_y;		//ԭʼ����
	float flow_x_i;	//ԭʼ���ݻ���ֵ�����ι���ʱ��ֻ�������������𣿽ǶȲ����Ƿ�ҲҪ��գ�
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


