/******************** (C) COPYRIGHT 2016 ANO Tech ***************************

*****************************************************************************/
#include "ANO_Data_Transfer.h"
#include "ALL_DATA.h"
#include "nrf24l01.h"
#include "SPI.h"
#include <string.h>
#include "LED.h" 
#include "ANO_DT.h"
#include "Remote.h"
#include "spl06.h"
#include "flow.h"
#include "ADC.h"

dt_flag_t f;					//需要发送数据的标志
u8 data_to_send[50];			//发送数据缓存



//Send_Data函数是协议中所有发送数据功能使用到的发送函数
//移植时，用户应根据自身应用的情况，根据使用的通信方式，实现此函数

void ANO_DT_Send_Data(u8 *dataToSend , u8 length)
{
//	switch(flag.NS)
//	{
//		case 1://已连接遥控器
			ANO_NRF_TxPacket_AP(dataToSend,length);
//		break;
//		
//		case 3://已连接蓝牙模块
////			ANO_UART3_Put_Buf(dataToSend,length);
//		break;
//		
////		default:break;
//	}
//	Usb_Hid_Adddata(dataToSend,length);
//	Usb_Hid_Send();
}



///////////////////////////////////////////////////////////////////////////////////////
//飞机发送数据到遥控器上传到上位机   ，
//由 scheduler.c 的 Duty_4ms()调用，已改到Duty_20ms()
//以前在 Duty_4ms()，但实际上遥控射频只能10ms接收一次，4ms发送一次会导致丢失数据
u8 cnt = 0,yaw_lock = 0,Send_Check = 0;

void ANO_DT_Data_Exchange(void)  //飞机发送数据到遥控器上传到上位机   ，由 scheduler.c 的 Duty_4ms()调用，已改到Duty_20ms()
{
	
	//当 Send_Check=1，会触发射频发送校验码给上位机
	//这个仅仅在 上位机修改飞控的pid参数时，会使用到
	//data_to_send 数据在 ANO_DT.c 的 串口3发送校验码 ANO_DT_Send_Check() 中产生
	if(Send_Check)
	{
		Send_Check = 0;
		ANO_DT_Send_Data(data_to_send, 7); //多发几次，防止因为满63字节一起发送，导致延误
		ANO_DT_Send_Data(data_to_send, 7);
		ANO_DT_Send_Data(data_to_send, 7);
	}
	
	//f.send_pid 在 ANO_DT.c的 ANO_DT_Data_Receive_Anl() 更新
	else if(f.send_pid)  //给上位机发送PID参数
	{
		cnt++;
		switch(cnt)
		{
			case 5:
			case 1:	ANO_DT_Send_PID(1,pidRateX.kp*1000,pidRateX.ki*1000,pidRateX.kd*1000,
																pidRateY.kp*1000,pidRateY.ki*1000,pidRateY.kd*1000,
																pidRateZ.kp*1000,pidRateZ.ki*1000,pidRateZ.kd*1000);	
			break;
			
			case 6:
			case 2:	ANO_DT_Send_PID(2,pidRoll.kp*1000,pidRoll.ki*1000,pidRoll.kd*1000,
																pidPitch.kp*1000,pidPitch.ki*1000,pidPitch.kd*1000,
																pidYaw.kp*1000,pidYaw.ki*1000,pidYaw.kd*1000);
//		cnt = 0;	f.send_pid = 0;	
			break;
			
			case 7:
			case 3:	ANO_DT_Send_PID(3,pidHeightRate.kp*1000,pidHeightRate.ki*1000,pidHeightRate.kd*1000,
																pidHeightHigh.kp*1000,pidHeightHigh.ki*1000,pidHeightHigh.kd*1000,
																Flow_SpeedPid_x.kp*1000,Flow_SpeedPid_x.ki*1000,Flow_SpeedPid_x.kd*1000);
			//cnt = 0;	f.send_pid = 0;	
			break;
			
			case 8:
			case 4:	ANO_DT_Send_PID(4,Flow_PosPid_x.kp*1000,Flow_PosPid_x.ki*1000,Flow_PosPid_x.kd*1000,																														
																Flow_SpeedPid_y.kp*1000,Flow_SpeedPid_y.ki*1000,Flow_SpeedPid_y.kd*1000,
																Flow_PosPid_y.kp*1000,Flow_PosPid_y.ki*1000,Flow_PosPid_y.kd*1000);
			//cnt = 0;	f.send_pid = 0;	
			break;
			
			case 9:
						//到这一帧时，遥控usb需要把所有未发送的数据全部发出给上位机，方便单独调试
						data_to_send[0]=0xAA;
						data_to_send[1]=0xAA;
						data_to_send[2]=0xEF; //模仿pid设置时的校验回应，遥控器那边需要立即发送，而不是等63个字节
						data_to_send[3]=1;
						data_to_send[4]=0;
						data_to_send[5]=0x44; //sum
			
						ANO_DT_Send_Data(data_to_send, 6); //AAAAEF010044 //遥控遇到特征字 0xEF需要立即发送
			
			cnt = 0;	f.send_pid = 0;	
			break;
			
		}				
	}
	
	
	else if(f.send_version)  //上位机获取飞机版本
	{
		f.send_version = 0;
		//ANO_DT_Send_Version(1,ANO_Param.hardware,ANO_Param.software,510,0);
	}
	
	
	else
	{		
		cnt++;  //每次发送一种任务
		switch(cnt)
		{
			case 1:	 //01  逆时针为正
				//ANO_DT_Send_Status(-Angle.roll,Angle.pitch,-Angle.yaw,mini.flow_High,ALL_flag.slock_flag,ALL_flag.unlock);	 // 不需要反向，逆时针为正
				ANO_DT_Send_Status(Angle.roll,Angle.pitch,Angle.yaw,mini.flow_High,ALL_flag.slock_flag,ALL_flag.unlock);	 // 飞控状态    10001
			
			break;
			
			case 2:	 //02
							ANO_DT_Send_Senser(MPU6050.accX,MPU6050.accY,MPU6050.accZ,					//	陀螺仪加速度数据
																 MPU6050.gyroX,MPU6050.gyroY,MPU6050.gyroZ, 			//	陀螺仪角度数据
																 mini.flow_x_iOUT,mini.flow_y_iOUT , mini.qual);  //	光流原使数据		        
			break;
			
			case 3:   //03
				ANO_DT_Send_RCData(Remote.thr,Remote.yaw,Remote.roll,Remote.pitch,Remote.AUX1,Remote.AUX2,Remote.AUX3,Remote.AUX4,NRF_SSI,Remote.AUX6);  // 遥控器通道数据
			break;
			
			case 4:	 //05
				ANO_DT_Send_Power(voltage/10,Remote.AUX5,1,NRF_SSI,test_flag,set_flag,mini.qual);   //飞机电压/遥控器电压/无线模块的信号值/传感器状态/飞行模式（菜单飞控设置）/光流质量
			break;
			
			case 5:	//07
				ANO_DT_Send_Senser2(FlightData.High.bara_height, Control_high, pidHeightHigh.desired);  //气压计/实时控制的高度
			break;
			
			case 6:	//09,发送调试数据给遥控器，想传回给屏幕看的都可以放这里,可以放8个双字节
				
				ANO_DT_Send_debug(Aux_Rc.debug1, Aux_Rc.debug2,Aux_Rc.debug3,Aux_Rc.debug4,Aux_Rc.debug5,((u16)pixel_flow.err1_cnt<<8) | (pixel_flow.err2_cnt), 
																	pidRoll.desired*10, pidPitch.desired*10);  //调试信息
			
				//ANO_DT_Send_debug(Aux_Rc.debug1,Aux_Rc.debug2,Aux_Rc.debug3,Aux_Rc.debug4,Aux_Rc.debug5,Aux_Rc.debug6,Aux_Rc.debug7,Aux_Rc.debug8);  //调试信息
			break;
			
			
			case 7:	//0x0B  //光流模块数据	
				ANO_DT_Send_speed(pixel_flow.loc_x, pixel_flow.loc_y, FlightData.High.bara_height,Flow_PosPid_x.desired,Flow_PosPid_y.desired,Flow_SpeedPid_x.out,Flow_SpeedPid_y.out);  
				//cnt = 0;	//结束
			break;
			

			case 8:	//0xF1  //匿名上位机 用户数据（帧1）光流姿态角补偿系数调试用, 转为毫米，且 float(4字节) 转为 int16_t(2字节)
				ANO_DT_Send_UserData_Frm1(pixel_flow.fix_x_i*10.0f, pixel_flow.fix_y_i*10.0f, pixel_flow.ang_x*10.0f, pixel_flow.ang_y*10.0f);  
				cnt = 0;	//结束
			break;
			 
			
			//给遥控器详细数据，第二页传输的内容
			//飞控电量 voltage/10
			//光流 pixel_flow.loc_x, pixel_flow.loc_y
			//姿态角 ALL_flag.unlock | 期望姿态角，Angle.roll,| Angle.pitch，
			//油门构成
			//定高模式，期望高度，当前高度Control_high| yaw角速度 MPU6050.gyroZ

		}
	}
}




///////////////////////////////////////////////////////////////////////////////////////
////Data_Receive_Anl函数是协议数据解析函数，函数参数是符合协议格式的一个数据帧，该函数会首先对协议数据进行校验
////校验通过后对数据进行解析，实现相应功能
////此函数可以不用用户自行调用，由函数Data_Receive_Prepare自动调用

extern u16 save_pid_en;

void ANO_DT_Send_Version(u8 hardware_type, u16 hardware_ver,u16 software_ver,u16 protocol_ver,u16 bootloader_ver)
{
	u8 _cnt=0;
	u8 sum = 0;
	u8 i;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x00;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=hardware_type;
	data_to_send[_cnt++]=BYTE1(hardware_ver);
	data_to_send[_cnt++]=BYTE0(hardware_ver);
	data_to_send[_cnt++]=BYTE1(software_ver);
	data_to_send[_cnt++]=BYTE0(software_ver);
	data_to_send[_cnt++]=BYTE1(protocol_ver);
	data_to_send[_cnt++]=BYTE0(protocol_ver);
	data_to_send[_cnt++]=BYTE1(bootloader_ver);
	data_to_send[_cnt++]=BYTE0(bootloader_ver);
	
	data_to_send[3] = _cnt-4;
	
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}

void ANO_DT_Send_Speed(float x_s,float y_s,float z_s)
{
	u8 _cnt=0;
	vs16 _temp;
	u8 sum = 0;
	u8 i;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x0B;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(0.1f *x_s);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(0.1f *y_s);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(0.1f *z_s);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
	data_to_send[3] = _cnt-4;
	
	
	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);

}

void ANO_DT_Send_Location(void)
{
	u8 _cnt=0;
	vs16 _temp;
  u8 sum = 0;
	u8 i;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x32;
	data_to_send[_cnt++]=0;

	data_to_send[_cnt++]=0;
	
//	_temp = (s16)(-Rol_Data*100);
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	
//	_temp = (s16)(Pit_Data*100);
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);

//	_temp = (s16)(Line_Angle*100);
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	ANO_DT_Send_Data(data_to_send, _cnt);
}

void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed)
{
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp2 = alt;
	u8 sum = 0;
	u8 i;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;    //功能字，飞机飞行数据，姿态角等
	data_to_send[_cnt++]=0;
	
	_temp = (int)(angle_rol*100); // +/-180度
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_pit*100); // +/-180度 
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	while(angle_yaw>180) angle_yaw -=360; //新增
	while(angle_yaw<-180) angle_yaw +=360;//新增
	_temp = (int)(angle_yaw*100); //+/-32767需要先限制到+/-180度以内
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	data_to_send[_cnt++] = fly_model;
	
	data_to_send[_cnt++] = armed;
	
	data_to_send[3] = _cnt-4;
	
	
	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z)
{
	u8 _cnt=0;
	vs16 _temp;
	u8 sum = 0;
	u8 i;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	
	_temp = a_x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = g_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = m_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
/////////////////////////////////////////
	_temp = 0;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);	
	
	data_to_send[3] = _cnt-4;
	
	
	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}



////////////////////////////////////////////////////////
//void ANO_DT_Send_Senser2(s32 bar_alt, u16 csb_alt, u16 high_desired)
void ANO_DT_Send_Senser2(s32 bar_alt, s16 csb_alt, s16 high_desired)
{
	u8 _cnt=0;
	u8 sum = 0;
	u8 i;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x07;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE3(bar_alt);
	data_to_send[_cnt++]=BYTE2(bar_alt);
	data_to_send[_cnt++]=BYTE1(bar_alt);
	data_to_send[_cnt++]=BYTE0(bar_alt);

	data_to_send[_cnt++]=BYTE1(csb_alt);  //Control_high,为folat，当气压优先时，有可能是负数
	data_to_send[_cnt++]=BYTE0(csb_alt);
	
	data_to_send[_cnt++]=BYTE1(high_desired);
	data_to_send[_cnt++]=BYTE0(high_desired);
	
	data_to_send[3] = _cnt-4;
	
	
	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}




////////////////////////////////////////////////////////
void ANO_DT_Send_debug(s16 d1,s16 d2,s16 d3,s16 d4,s16 d5,s16 d6,s16 d7,s16 d8)
{
	u8 _cnt=0;
	u8 sum = 0;
	u8 i;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x09; //功能字
	data_to_send[_cnt++]=0; //长度
	

	data_to_send[_cnt++]=BYTE1(d1);
	data_to_send[_cnt++]=BYTE0(d1);

	data_to_send[_cnt++]=BYTE1(d2);
	data_to_send[_cnt++]=BYTE0(d2);
	
	data_to_send[_cnt++]=BYTE1(d3);
	data_to_send[_cnt++]=BYTE0(d3);

	data_to_send[_cnt++]=BYTE1(d4);
	data_to_send[_cnt++]=BYTE0(d4);
	
	data_to_send[_cnt++]=BYTE1(d5);
	data_to_send[_cnt++]=BYTE0(d5);

	data_to_send[_cnt++]=BYTE1(d6);
	data_to_send[_cnt++]=BYTE0(d6);
	
	data_to_send[_cnt++]=BYTE1(d7);
	data_to_send[_cnt++]=BYTE0(d7);

	data_to_send[_cnt++]=BYTE1(d8);
	data_to_send[_cnt++]=BYTE0(d8);
	
	data_to_send[3] = _cnt-4; //重写长度
	
	
	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
		
	data_to_send[_cnt++] = sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}




///////////////////////////////////////////////////////////////////////////////////
void ANO_DT_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6)
{
	u8 _cnt=0;
	u8 sum = 0;
	u8 i;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x03;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTE1(thr);
	data_to_send[_cnt++]=BYTE0(thr);
	data_to_send[_cnt++]=BYTE1(yaw);
	data_to_send[_cnt++]=BYTE0(yaw);
	data_to_send[_cnt++]=BYTE1(rol);
	data_to_send[_cnt++]=BYTE0(rol);
	data_to_send[_cnt++]=BYTE1(pit);
	data_to_send[_cnt++]=BYTE0(pit);
	data_to_send[_cnt++]=BYTE1(aux1);
	data_to_send[_cnt++]=BYTE0(aux1);
	data_to_send[_cnt++]=BYTE1(aux2);
	data_to_send[_cnt++]=BYTE0(aux2);
	data_to_send[_cnt++]=BYTE1(aux3);
	data_to_send[_cnt++]=BYTE0(aux3);
	data_to_send[_cnt++]=BYTE1(aux4);
	data_to_send[_cnt++]=BYTE0(aux4);
	data_to_send[_cnt++]=BYTE1(aux5);
	data_to_send[_cnt++]=BYTE0(aux5);
	data_to_send[_cnt++]=BYTE1(aux6);
	data_to_send[_cnt++]=BYTE0(aux6);

	data_to_send[3] = _cnt-4;
	
	
	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}

void ANO_DT_Send_Power(u16 votage, u16 current, u8 flag0, u8 flag1, u16 flag2, u16 flag3, u8 qual)
{
	u8 _cnt=0;
	u16 temp;
	u8 sum = 0;
	u8 i;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x05;   //传感器状态
	data_to_send[_cnt++]=0;
	
	temp = votage;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	
	temp = current;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	
	data_to_send[_cnt++]=flag0;
	data_to_send[_cnt++]=flag1;
	
	temp = flag2;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	
	temp = flag3;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	
	data_to_send[_cnt++]=qual;
	
	data_to_send[3] = _cnt-4;
	
	
	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
	
	
}
void ANO_DT_Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8)
{
	u8 _cnt=0;
	u8 sum = 0;
	u8 i;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x06;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE1(m_1);
	data_to_send[_cnt++]=BYTE0(m_1);
	data_to_send[_cnt++]=BYTE1(m_2);
	data_to_send[_cnt++]=BYTE0(m_2);
	data_to_send[_cnt++]=BYTE1(m_3);
	data_to_send[_cnt++]=BYTE0(m_3);
	data_to_send[_cnt++]=BYTE1(m_4);
	data_to_send[_cnt++]=BYTE0(m_4);
	data_to_send[_cnt++]=BYTE1(m_5);
	data_to_send[_cnt++]=BYTE0(m_5);
	data_to_send[_cnt++]=BYTE1(m_6);
	data_to_send[_cnt++]=BYTE0(m_6);
	data_to_send[_cnt++]=BYTE1(m_7);
	data_to_send[_cnt++]=BYTE0(m_7);
	data_to_send[_cnt++]=BYTE1(m_8);
	data_to_send[_cnt++]=BYTE0(m_8);
	
	data_to_send[3] = _cnt-4;
	
	
	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}

void ANO_DT_Send_speed(int16_t speed_rol_x, int16_t speed_pit_y, u16 speed_z, int16_t x_desired, int16_t y_desired, int16_t x_out, int16_t y_out)
{
	u8 _cnt=0;
	//u16 temp;
	int16_t temp;//改为有符号
	u8 sum = 0;
	u8 i;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x0B;
	data_to_send[_cnt++]=0;
	
	temp = speed_rol_x;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	
	temp = speed_pit_y;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	
	temp = speed_z;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	
	temp = x_desired;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	
	temp = y_desired;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	
	temp = x_out;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	
	temp = y_out;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	
	
	
	data_to_send[3] = _cnt-4;
	
	
	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}



// 匿名上位机 “高级收码”
// 帧格式设置“使能该帧” “显示该帧”，功能码 F1，数据类型 int16
// 数据容器设置：数据来源，选择“帧1”，数据位置，标签1在1处打勾，标签2在2处打勾，1~4都要选择正确
// 匿名上位机 “数据波形”
//“设置” -》“波形快速选择”-》“用户数据波形”-》点击确定
// 打勾 “UserData_1 ~ UserData_1”
// 点击“开始显示”

////////////////////////////////////////////////////////
void ANO_DT_Send_UserData_Frm1(int16_t fix_x_i, int16_t fix_y_i, int16_t ang_x, int16_t ang_y)
{
	u8 _cnt=0;
	u8 sum = 0;
	u8 i;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xF1;
	data_to_send[_cnt++]=0;
	

	data_to_send[_cnt++]=BYTE1(fix_x_i);
	data_to_send[_cnt++]=BYTE0(fix_x_i);

	data_to_send[_cnt++]=BYTE1(fix_y_i);
	data_to_send[_cnt++]=BYTE0(fix_y_i);
	
	data_to_send[_cnt++]=BYTE1(ang_x);
	data_to_send[_cnt++]=BYTE0(ang_x);
	
	data_to_send[_cnt++]=BYTE1(ang_y);
	data_to_send[_cnt++]=BYTE0(ang_y);
	
	data_to_send[3] = _cnt-4;
	
	
	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}




void ANO_DT_Send_PID(u8 group,s16 p1_p,s16 p1_i,s16 p1_d,s16 p2_p,s16 p2_i,s16 p2_d,s16 p3_p,s16 p3_i,s16 p3_d)
{
	u8 _cnt=0;
	vs16 _temp;
	u8 sum = 0;
	u8 i;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x10+group-1;
	data_to_send[_cnt++]=0;
	
	
	_temp = p1_p ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_i ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_d ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_p ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_i ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_d ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_p ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_i ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_d ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	
	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	ANO_DT_Send_Data(data_to_send, _cnt);
}


void Flag_Check(void)
{
	test_flag = 0;
	
	if(!MPU_Err)		test_flag |= BIT0; //陀螺仪
	if(!SPL_Err)		test_flag |= BIT1; //气压计
	if(!NRF_Err)		test_flag |= BIT2; //2.4G
	if(ult_ok)    	test_flag |= BIT3; //超声波
	if(!Locat_Err)  test_flag |= BIT4; //视觉定位
	//if(LED_warn==1)	test_flag |= BIT5;//低压
	if(!Flow_Err)		test_flag |= BIT6; //光流模块
	
	//低压
	if(ALL_flag.unlock && Remote.thr>1200){ //飞行中
			if(voltage<3310 && voltage>1000) test_flag |= BIT5;//低压,小于3.3v
	}
	else{																		//待机
			if(voltage<3510 && voltage>1000) test_flag |= BIT5;//低压,小于3.5v
	
	}
	
}

///******************* (C) COPYRIGHT 2016 ANO TECH *****END OF FILE************/

