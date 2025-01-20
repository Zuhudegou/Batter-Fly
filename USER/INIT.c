

#include "ALL_DEFINE.h" 
#include "ALL_DATA.h"   //在这里向外发布
#include "spl06.h" 
#include "SPL06_cankao.h" //新的
#include "LED.h"
#include "Uart1.h"
#include "USART2.h"
#include "ANO_Data_Transfer.h"
#include "ADC.h"

volatile uint32_t SysTick_count; //系统时间计数
volatile uint8_t spl_flag; //系统时间计数
_st_Mpu MPU6050;   //MPU6050原始数据
_st_Mag AK8975;   
_st_AngE Angle;    //当前角度姿态值
_st_Remote Remote; //遥控通道值

//这里定义的值，还要在  all_data.h  定义

//新增
struct _Aux_Rc Aux_Rc; //当前摇杆值解析

volatile uint32_t ST_CpuID;
 
 
_st_ALL_flag ALL_flag; //系统标志位，包含解锁标志位等



 _st_FlightData FlightData;
 //飞控命令
st_Command Command;

PidObject pidRateX; //内环PID数据
PidObject pidRateY;
PidObject pidRateZ;

PidObject pidPitch; //外环PID数据
PidObject pidRoll;
PidObject pidYaw;

PidObject pidHeightRate; //高度内环pid
PidObject pidHeightHigh; //高度外环pid



PidObject pidHeightThr; //用于平衡油门动态调整
PidObject pidOffsetRoll; //用于零漂动态调整



PidObject Flow_PosPid_x;    //外环光流
PidObject Flow_PosPid_y;

PidObject Flow_SpeedPid_x;  //内环光流
PidObject Flow_SpeedPid_y;

_st_IMU IMU;

void pid_param_Init(void); //PID控制参数初始化，改写PID并不会保存数据，请调试完成后直接在程序里更改 再烧录到飞控


//获取CPU的ID
void GetLockCode(void)
{
	ST_CpuID = *(vu32*)(0x1ffff7e8);//低字节芯片ID用来做通讯对频通道
}

u8 count1=0;

///////////////全部初始化//////////////////////////////////
void ALL_Init(void)
{
	float STBy;
	
	//移到最前面初始化，因为有时初始化之前会有浮动电压1v左右导致电机转动, 下拉电阻减小为4.7k
	//TIM2_PWM_Config();			//2路PWM初始化		
	//TIM3_PWM_Config();			//2路PWM初始化		
	

	
	//while(1);


	IIC_Init();             //I2C初始化
	count1=1;
		
	pid_param_Init();       //PID参数初始化
	count1=2;
	  
		
	
	
	LEDInit();              //LED闪灯初始化
	count1=3;




	MpuInit();              //MPU6050初始化，初始化开始，红灯亮，蓝灯灭，
	count1=4;


 


		//ADC初始化
	ADC1_Init();
	count1=5;
	
	ANO_Uart1_Init(19200);   //接光流模块
	//ANO_Uart1_Init(115200);   //接光流模块
	count1=6;
	
//	printf("ANO_Uart1_Init  \r\n")	
	//if (FLY_TYPE == 2) 
	//{UART2_Init(115200); }      //
	
	USART3_Config(500000);        //上位机串口初始化
//	printf("USART3_Config  \r\n");
	count1=7;
	
		


	NRF24L01_init();				//2.4G遥控通信初始化,, 初始化完成后，蓝灯亮，红灯灭，相当于AlternateFlash
	count1=8;
	
	//蓝灯交替表示陀螺仪err，红灯交替表示气压计err
	if(MPU_Err){ //陀螺仪异常
			fLED_H(); //右上角蓝灯熄灭，表示陀螺仪失败（左上角蓝灯是亮的），蓝灯交替闪烁，而不是同步闪烁

	}

	
	//if(SPL_init() >=1)	 //气压计初始化, 0表示 SUCCESS
	if(spl0601_init() >=1)	 //气压计初始化, 0表示 SUCCESS
	{//失败，红色只亮一个灯
		  while(1)
			{ 
				STBy++;
				//GPIOB->BSRR = GPIO_Pin_9;  //亮一个LED
				GPIOB->BRR =bLED_io;//仅点亮右下角红灯，导致红灯交替，左下角在rf初始化时已经关闭
				
				if(STBy>=100000)//失败
				{
						spl_flag=0; 
						SPL_Err = 1;
					  break;
				}	
			}
	}
	
	
	//气压计初始化成功，延续2.4G成功后蓝灯亮，红灯灭
	else 
	{
			spl_flag=1;
		  SPL_Err = 0;
		
		
		
	}
	
	count1=9;
	
	
	
	//两个蓝灯不同时亮，表示陀螺仪异常；但仅限没有连接遥控。
	//两个红灯不同时亮，表示气压计异常；但仅限没有连接遥控。


	//while(1);
	
	//移到最前面初始化，因为有时初始化之前会有浮动电压1v左右导致电机转动
	TIM2_PWM_Config();			//2路PWM初始化		
	TIM3_PWM_Config();			//2路PWM初始化		
	delay_ms(10); //10ms
	
	//while(1);

	
//最后，空心杯马达响几下	
#if ( FLY_TYPE !=3 ) //非无刷电机，无刷电机开机响应由电调负责。

	//#define PWM0 TIM2->CCR1   //定时器2 通道1    M1
	//#define PWM1 TIM2->CCR2		//定时器2 通道2		M2
	//#define PWM2 TIM3->CCR1   //定时器3 通道1		M3
	//#define PWM3 TIM3->CCR2		//定时器3 通道2		M4

	TIM_Cmd(TIM2,DISABLE);	//调整频率，为800Hz
	TIM_PrescalerConfig(TIM2, SystemCoreClock/(TIM2_PWM_MAX+1)/800 -1 ,TIM_PSCReloadMode_Immediate);
	TIM_Cmd(TIM2,ENABLE);
	
	TIM_Cmd(TIM3,DISABLE);	//
	TIM_PrescalerConfig(TIM3, SystemCoreClock/(TIM3_PWM_MAX+1)/800 -1 ,TIM_PSCReloadMode_Immediate);
	TIM_Cmd(TIM3,ENABLE);
	
	for(u8 i=0; i<3; i++){
		//调整占空比1.3%（13/1000），太大了会转；跟无刷不同，无刷电机可以完全替代蜂鸣器，很响但不转。
		TIM2->CCR1=13;	TIM2->CCR2=13; TIM3->CCR1=13; TIM3->CCR2=13; 
		delay_ms(30); //响30ms
		
		//恢复占空比为0
		TIM2->CCR1=0;	TIM2->CCR2=0; TIM3->CCR1=0; TIM3->CCR2=0; 
		delay_ms(120); //暂停0.12s
	}

		
	TIM_Cmd(TIM2,DISABLE);	//恢复正常频率
	TIM_PrescalerConfig(TIM2, SystemCoreClock/(TIM2_PWM_MAX+1)/TIM2_PWM_HZ -1 ,TIM_PSCReloadMode_Immediate);
	TIM_Cmd(TIM2,ENABLE);
	
	TIM_Cmd(TIM3,DISABLE);	//
	TIM_PrescalerConfig(TIM3, SystemCoreClock/(TIM3_PWM_MAX+1)/TIM3_PWM_HZ -1 ,TIM_PSCReloadMode_Immediate);
	TIM_Cmd(TIM3,ENABLE);
	
#endif
	
	
}


volatile u8 flow_pid_param_type=0;


//设置pid参数
void pid_param_Init(void)
{
	
	/////////////////////////////  1. 姿态角 pid  ////////////////////////////////////
	
	
//////////////////1.1 内环速度PID///////////////////////	
	
	#if (FLY_TYPE == 1 || FLY_TYPE == 2) //空心杯
			//pidRateX.kp = 2.0f;
			//pidRateY.kp = 2.0f;
			pidRateX.kp = 1.8f;
			pidRateY.kp = 1.8f;
			pidRateZ.kp = 3.0f; //yaw
			
			pidRateX.ki = 0.0f;
			pidRateY.ki = 0.0f;
			pidRateZ.ki = 0.0f;	
			
			pidRateX.kd = 0.08f;
			pidRateY.kd = 0.08f;
			pidRateZ.kd = 0.05f;	
	
	
	
	#else 		//3 、无刷330   //无刷强劲，力度反应迅速，pid需要小一点，太大容易抖
	
			pidRateX.kp = 1.2f;
			pidRateY.kp = 1.2f;
			pidRateZ.kp = 2.0f; //yaw
			
			pidRateX.ki = 0.0f;
			pidRateY.ki = 0.0f;
			pidRateZ.ki = 0.0f;	
			
			pidRateX.kd = 0.050f;
			pidRateY.kd = 0.050f;
			pidRateZ.kd = 0.030f;	
	
	
	#endif
	
	
	
/////////////1.2 外环角度PID///////////////////////////
	
	pidPitch.kp = 7.0f;
	pidRoll.kp = 7.0f;
	pidYaw.kp = 7.0f;	
	
	pidPitch.ki = 0.0f;
	pidRoll.ki = 0.0f;
	pidYaw.ki = 0.0f;	
	
	pidPitch.kd = 0.0f;
	pidRoll.kd = 0.0f;
	pidYaw.kd = 0.0f;	
	
	
	
	
	/////////////////////////////  2. 定高  ////////////////////////////////////
	
	
	
	//////////////////2.1 高度 内环 PID 参数///////////////////////	
	
#if (FLY_TYPE == 1 || FLY_TYPE == 2) //空心杯
	pidHeightRate.kp = 1.2f; //1.2f
	pidHeightRate.ki = 0.04f;
	pidHeightRate.kd = 0.085f;
#else 		//3 、无刷330
	pidHeightRate.kp = 1.0f; //1.2f
	pidHeightRate.ki = 0.04f;
	pidHeightRate.kd = 0.06f;
#endif


	//////////////////2.2 高度 外环PID参数///////////////////////	
	pidHeightHigh.kp = 1.2f;//1.2f
	pidHeightHigh.ki = 0.00f;
	pidHeightHigh.kd = 0.085f;//0.085f



	/*
		//高度 内环PID参数 速度
	pidHeightRate.kp = 1.2f; //1.2f
	pidHeightRate.ki = 0.04f;
	pidHeightRate.kd = 0.2f;

		//高度 外环PID参数
	pidHeightHigh.kp = 0.6f;//1.2f
	pidHeightHigh.ki = 0.00f;
	pidHeightHigh.kd = 0.1f;//0.085f
	*/

	//基础油门pid参数
	
	pidHeightThr.kp =1.0f;
	pidHeightThr.ki =0.0f;
	pidHeightThr.kd =0.1f;
	
	
	
	
	
	///////////////////////////////  3. 定点  ////////////////////////////////////
	
#if (FLY_TYPE == 1 || FLY_TYPE == 2) //空心杯
	
//////////////////////////3.1.1光流 x内环///////////////////////////////////////////

	//X内环光流PID参数  速度
	//Flow_SpeedPid_x.kp = 0.610f;//比例  0.600f
	//Flow_SpeedPid_x.ki = 0.000f;//积分
	//Flow_SpeedPid_x.kd = 0.400f;//微分
	
	//X内环光流速度PID参数  速度
	//Flow_SpeedPid_x.kp = 0.610f;//比例  0.600f
	Flow_SpeedPid_x.kp = 0.500f;//比例  0.600f
	Flow_SpeedPid_x.ki = 0.000f;//积分
	Flow_SpeedPid_x.kd = 0.400f;//微分
	

	////////////////////////3.1.2光流 y内环//////////////////////////////////
	
	//Y内环光流PID参数 速度
	//Flow_SpeedPid_y.kp = 0.610f;//比例
	//Flow_SpeedPid_y.ki = 0.000f;//积分
	//Flow_SpeedPid_y.kd = 0.400f;//微分
	
	//Y内环光流速度PID参数 速度
	//Flow_SpeedPid_y.kp = 0.610f;//比例
	Flow_SpeedPid_y.kp = 0.500f;//比例
	Flow_SpeedPid_y.ki = 0.000f;//积分
	Flow_SpeedPid_y.kd = 0.400f;//微分
	
	
	
	/////////////////////////3.2.1 光流 x外环/////////////////////////////////

	//X外环光流位置PID参数  位置
	//Flow_PosPid_x.kp = 2.200f;//比例  2.000f
	Flow_PosPid_x.kp = 2.000f;//比例  2.000f
	Flow_PosPid_x.ki = 0.000f;//积分
	Flow_PosPid_x.kd = 0.006f;//微分
	
	/////////////////////////3.2.2 光流 y外环/////////////////////////////////
	
	//Y外环光流位置PID参数 位置 
	//Flow_PosPid_y.kp = 2.200f;//比例
	Flow_PosPid_y.kp = 2.000f;//比例
	Flow_PosPid_y.ki = 0.00f;//积分
	Flow_PosPid_y.kd = 0.006f;//微分

///////////////////////////////////////////////////////////////////
	


#else 		//3 、无刷330


/*
//////////////////////////光流 内环///////////////////////////////////////////

	//X内环光流速度PID参数 速度
	Flow_SpeedPid_x.kp = 0.410f;//比例  0.600f
	Flow_SpeedPid_x.ki = 0.000f;//积分
	Flow_SpeedPid_x.kd = 0.300f;//微分
	//Y内环光流速度PID参数 速度
	Flow_SpeedPid_y.kp = 0.410f;//比例
	Flow_SpeedPid_y.ki = 0.000f;//积分
	Flow_SpeedPid_y.kd = 0.300f;//微分
	
/////////////////////////光流 外环/////////////////////////////////

	//X外环光流位置PID参数 位置
	Flow_PosPid_x.kp = 2.200f;//比例  2.000f
	Flow_PosPid_x.ki = 0.000f;//积分
	Flow_PosPid_x.kd = 0.006f;//微分
	//Y外环光流位置PID参数 位置 
	Flow_PosPid_y.kp = 2.200f;//比例
	Flow_PosPid_y.ki = 0.00f;//积分
	Flow_PosPid_y.kd = 0.006f;//微分

///////////////////////////////////////////////////////////////////
*/

	flow_pid_param_default(); //无刷电机光流pid，定点悬停时，pid参数较小


#endif

	Command.FlightMode = NORMOL;  //初始化为姿态飞行模式
}


/*



#if ( FLY_TYPE==3 ) //无刷 光流pid默认参数，手动定高起飞或一键定高起飞悬停中
  //退出模式3，或者方向摇杆动作结束，则调入光流默认pid参数
	if(flow_pid_param_type!=1) flow_pid_param_default(); //无刷电机光流pid，定点悬停时，pid参数较小

#endif


#if ( FLY_TYPE==3 ) //无刷 光流pid运动参数，一键定高起飞或方向摇杆移动中
  //进入模式3，或者检测到方向摇杆动作，则调入光流运动pid参数
	if(flow_pid_param_type!=2) flow_pid_param_move();//无刷电机光流pid，起飞和手动移动时，pid参数较大

#endif

*/



//无刷电机光流pid，定点悬停或平时，pid参数较小，追求平稳，防止抖动；PID参数可以在遥控连接上位机后实时查看
void flow_pid_param_default(void){
	
		//if(flow_pid_param_type!=0) return;  //这一句可以防止飞行过程pid参数被自动更改，方便使用上位机修改pid参数
	
		flow_pid_param_type=1;
	
#if (FLY_TYPE ==3) //无刷
		/*
		//X内环光流速度PID参数  速度
		Flow_SpeedPid_x.kp = 0.400f;//比例  0.380f
		Flow_SpeedPid_x.ki = 0.000f;//积分
		Flow_SpeedPid_x.kd = 0.300f;//微分  0.300f
		//Y内环光流速度PID参数 速度
		Flow_SpeedPid_y.kp = Flow_SpeedPid_x.kp;//比例
		Flow_SpeedPid_y.ki = Flow_SpeedPid_x.ki;//积分
		Flow_SpeedPid_y.kd = Flow_SpeedPid_x.kd;//微分
		
		//X外环光流位置PID参数  位置
		Flow_PosPid_x.kp = 1.900f;//比例  1.700f
		Flow_PosPid_x.ki = 0.000f;//积分
		Flow_PosPid_x.kd = 0.006f;//微分  0.006f;
		//Y外环光流位置PID参数 位置 
		Flow_PosPid_y.kp = Flow_PosPid_x.kp;//比例
		Flow_PosPid_y.ki = Flow_PosPid_x.ki;//积分
		Flow_PosPid_y.kd = Flow_PosPid_x.kd;//微分
		*/
		
		
		
		
		//X内环光流速度PID参数  速度
		Flow_SpeedPid_x.kp = 0.410f;//比例  0.600f
		Flow_SpeedPid_x.ki = 0.000f;//积分
		Flow_SpeedPid_x.kd = 0.300f;//微分
		//Y内环光流速度PID参数 速度
		Flow_SpeedPid_y.kp = Flow_SpeedPid_x.kp;//比例
		Flow_SpeedPid_y.ki = Flow_SpeedPid_x.ki;//积分
		Flow_SpeedPid_y.kd = Flow_SpeedPid_x.kd;//微分
		
		//X外环光流位置PID参数  位置
		Flow_PosPid_x.kp = 2.000f;//比例  2.000f
		Flow_PosPid_x.ki = 0.000f;//积分
		Flow_PosPid_x.kd = 0.006f;//微分
		//Y外环光流位置PID参数 位置 
		Flow_PosPid_y.kp = Flow_PosPid_x.kp;//比例
		Flow_PosPid_y.ki = Flow_PosPid_x.ki;//积分
		Flow_PosPid_y.kd = Flow_PosPid_x.kd;//微分
	
		
		
		
		
		

	
#endif
}

//无刷电机光流pid，起飞或者方向摇杆控制移动时，pid参数较大，追求快速响应期望值；PID参数可以在遥控连接上位机后实时查看
void flow_pid_param_move(void){
	
	//if(flow_pid_param_type!=0) return;  //这一句可以防止飞行过程pid参数被自动更改，方便使用上位机修改pid参数
	
	flow_pid_param_type=2;
	
#if (FLY_TYPE ==3) //无刷

		//X内环光流速度PID参数  速度
		Flow_SpeedPid_x.kp = 0.410f;//比例  0.600f
		Flow_SpeedPid_x.ki = 0.000f;//积分
		Flow_SpeedPid_x.kd = 0.300f;//微分
		//Y内环光流速度PID参数 速度
		Flow_SpeedPid_y.kp = Flow_SpeedPid_x.kp;//比例
		Flow_SpeedPid_y.ki = Flow_SpeedPid_x.ki;//积分
		Flow_SpeedPid_y.kd = Flow_SpeedPid_x.kd;//微分
		
		//X外环光流位置PID参数  位置
		Flow_PosPid_x.kp = 2.000f;//比例  2.000f
		Flow_PosPid_x.ki = 0.000f;//积分
		Flow_PosPid_x.kd = 0.006f;//微分
		//Y外环光流位置PID参数 位置 
		Flow_PosPid_y.kp = Flow_PosPid_x.kp;//比例
		Flow_PosPid_y.ki = Flow_PosPid_x.ki;//积分
		Flow_PosPid_y.kd = Flow_PosPid_x.kd;//微分
	
#endif
}





