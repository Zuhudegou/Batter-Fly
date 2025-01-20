#include "ALL_DEFINE.h"
#include "scheduler.h"
#include "spl06.h"
#include "SPL06_cankao.h" //新的
#include "ANO_Data_Transfer.h"
#include "WIFI_UFO.h"
#include "flow.h"
#include "ADC.h"

loop_t loop; 
//u32 time[10],time_sum;
int32_t time[10],time_sum;
 
//2ms执行一次
//在delay.c调用，2ms一次
void Loop_check()
{
	loop.cnt_2ms++;
	loop.cnt_4ms++;
	loop.cnt_6ms++;
	loop.cnt_10ms++;
	loop.cnt_20ms++;
	loop.cnt_50ms++;
	loop.cnt_1000ms++;

	if( loop.check_flag >= 1)
	{
		loop.err_flag ++;// 2ms 
	}
	else
	{
		loop.check_flag += 1;   //该标志位在循环后面清0
	}
	
}



void main_loop()
{
	if( loop.check_flag >= 1 )
	{
		
		if( loop.cnt_2ms >= 1 )
		{
			loop.cnt_2ms = 0;
			
			Duty_2ms();	 					//周期2ms的任务
		}
		if( loop.cnt_4ms >= 2 )
		{
			loop.cnt_4ms = 0;
			Duty_4ms();						//周期4ms的任务
		}
		if( loop.cnt_6ms >= 3 )
		{
			loop.cnt_6ms = 0;
			Duty_6ms();						//周期6ms的任务
		}
		if( loop.cnt_10ms >= 5 )
		{
			loop.cnt_10ms = 0;
			Duty_10ms();					//周期10ms的任务
		} 
		if( loop.cnt_20ms >= 10 )
		{
			loop.cnt_20ms = 0;
			Duty_20ms();					//周期20ms的任务
		}
		if( loop.cnt_50ms >= 25 )
		{
			loop.cnt_50ms = 0;
			Duty_50ms();					//周期50ms的任务
		}
		if( loop.cnt_1000ms >= 500)
		{
			loop.cnt_1000ms = 0;
			Duty_1000ms();				//周期1s的任务
		}
		loop.check_flag = 0;		//循环运行完毕标志
	}
}

/////////////////////////////////////////////////////////
void Duty_2ms()
{
	time[0] = GetSysTime_us();
	
	MpuGetData();				          //读取陀螺仪数据，获取到角速度直接会作用于姿态控制的pid内环测量值（2ms一次），但外环测量值要经过 GetAngle()解算才有更新（6ms一次），另外定高时（10ms一次）的运动加速度GetAccz()也需要经过 GetAngle()解算（6ms一次）
	FlightPidControl(0.002f);     //姿态控制
	MotorControl();               //电机控制
	CloseLED(); 
	
	time[0] = GetSysTime_us() - time[0];
}

//////////////////////////////////////////////////////////
void Duty_4ms()
{
	time[1] = GetSysTime_us();
	
	ANO_NRF_Check_Event();    //扫描接收2.4G信号，，存在数据则标记 Nrf_Erro=0
	//ANO_DT_Data_Exchange();		//已转移到Duty_20ms()，通过射频发送飞控数据给遥控器, 接收频率要快（4ms一次，避免以后遥控数据），但发送频率要慢，不然收不全
	
	//这个是接收遥控器数据，接收完数据，必须马上 调用RC_Analy();，否则会遗漏掉遥控器的数据，特别是锁定与解锁命令
	Rc_Connect();  						//1.0 解析遥控器数据,放到 Remote，，，解析到数据则Nrf_Erro++
	
	Mode_Controler(0.004f); 	//1.1 分析处理收到的Remote数据；定高定点模式会调用Flow_mode_two()获取期望姿态角
	
	RC_Analy();	     					//1.2 遥控器控制指令处理，进一步处理 Remote内的数据，比如锁定和解锁和陀螺仪校准
	
	time[1] = GetSysTime_us() - time[1];
}

//////////////////////////////////////////////////////////
void Duty_6ms()
{
	time[2] = GetSysTime_us(); 
	
	GetAngle(&MPU6050,&Angle,0.006f);   //解算陀螺仪数据，生成姿态数据，更新Angle;;; 注意：z轴运动加速度（消除重力影响）NormAccz也在这里更新（6ms一次）, 定高算法HeightPidControl需要用到这个值（10ms一次）。
	
	time[2] = GetSysTime_us() - time[2];
}

/////////////////////////////////////////////////////////
void Duty_10ms()
{
	time[3] = GetSysTime_us();

	//这个要移到4ms任务，与Rc_Connect()同频率，
	//RC_Analy();	     					//1.2 遥控器控制指令处理，进一步处理 Remote内的数据
	
	//光流数据采集，在串口1的回调函数。调用 Flow_Receive1()，执行频率由光流模块自身决定。
	Pixel_Flow_Fix(0.006f);  			//mini光流数据融合处理，光流以及激光高度数据接收在串口1回调函数自动接收
	Flow_Pos_Controler(0.006f);		//光流定点控制	
	
	Height_Get(0.01f);			//获取气压高度数据，气压原始数据获取
	High_Data_Calc(10);			//气压高度数据融合，刷新 FlightData.High.bara_height，用于大于激光量程时（4米）
	
	//Height_Get_New(0.01f);			//获取气压高度数据，气压原始数据获取
	
	
	HeightPidControl(0.006f); 		//刷新Control_high(激光或气压), 以及定高模式下的高度控制程序，非定高模式退出
	
	time[3] = GetSysTime_us() - time[3];
}

/////////////////////////////////////////////////////////
void Duty_20ms()
{
	time[4] = GetSysTime_us();

	ANTO_polling(); 	//通过飞控串口3发送数据到上位机  //同时也会更新来自上位机的pid参数给飞控，这个pid参数不一定从串口接收而来。也可能是遥控转送
	ANO_DT_Data_Exchange();		//发送数据给遥控器, 接收频率要快（4ms一次，避免以后遥控数据），但发送频率要慢，不然收不全，遥控接收为10ms一次
	
	time[4] = GetSysTime_us() - time[4];
}

//////////////////////////////////////////////////////////
void Duty_50ms()
{
	time[5] = GetSysTime_us();
	
	PilotLED(); 						//LED刷新
	
	Flag_Check();   			 //传感器状态标志
	
	Voltage_Check();			//飞控电压检测
	
	time[5] = GetSysTime_us() - time[5];
}

/////////////////////////////////////////////////////////////
void Duty_1000ms()
{
	u8 i;
  NRF_SSI = NRF_SSI_CNT;  //NRF信号强度
	NRF_SSI_CNT = 0;
	
	WIFI_SSI = WIFI_SSI_CNT;//WiFi信号强度
	WIFI_SSI_CNT = 0;
	
	Locat_SSI = Locat_SSI_CNT;//视觉位置数据频率
	Locat_SSI_CNT = 0;
	
	Flow_SSI = Flow_SSI_CNT;  //光流数据频率 Flow_SSI_CNT在串口1回调函数（光流数据接收）会自增
	Flow_SSI_CNT = 0;
	
		//检测视觉定位模块是否插入
	if(Locat_SSI>10)  Locat_Err = 0;
	else Locat_Mode=0,Locat_Err = 1;
	
	  //检测光流模块是否插入
	if(Flow_SSI>10)  Flow_Err = 0;  //光流正常，串口收到光流模块10条数据，即表示光流正常
	else 						 Flow_Err = 1;	//光流异常，此时在 Mode_Controler()会自动改为姿态模式
	
	time_sum = 0;
	for(i=0;i<6;i++)	time_sum += time[i];
}



//////////////////////////end///////////////////////////////////////////
