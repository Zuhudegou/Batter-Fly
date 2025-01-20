/**********************STM32 开源无人机*******************************************************/
//  V1.0 开源作者：小南&zin；日期：2016.11.21
//           STM32F103C8飞控以及遥控基础功能以及核心代码实现；
//  V2.0 开源作者：小刘；日期：2020.05.17
//           scheduler任务架构调整，增加屏幕以及气压计，新增PID在线调整功能；
//  V3.0 开源作者：zhibo_sz&sunsp；日期：2024.06.01
//           新增一键定高起飞，悬停运动控制以及刹车优化，气压遥控屏幕陀螺仪等模块优化，新增无刷电机；
/********************************************************************************************/

//声明：
//      本程序仅对购机用户开源，学习使用，所有权归以上作者所有；
//      未经许可，不得传阅、转载、公开、转卖本代码。


#include "ALL_DATA.h"
#include "nrf24l01.h"
#include "control.h"
#include <math.h>
#include "myMath.h"
#include "LED.h"
#include "Remote.h"
#include "WIFI_UFO.h"
#include "ALL_DATA.h"
#include "ANO_DT.h"
#include "flow.h"
#include "spl06.h"
#include "STM32F10x_IWDG.h"

#define SUCCESS 0
#undef FAILED
#define FAILED  1

u16 test_flag,set_flag;

void Rc_Connect(void)
{
	//控制数据优先级
	//1、遥控器  2、WiFi图传模块 
	if(NRF_Connect()==0)  //射频接收并解析数据
	{
		if(WIFI_UFO_Connect()==0)//wifi接收并解析数据
		{
			
		}
	}
}





/*****************************************************************************************
 *  通道数据处理
 * @param[in] 
 * @param[out] 
 * @return     
 ******************************************************************************************/	
uint8_t RC_rxData[32];
void remote_unlock(void);	
void mpu_adjust(void);
void motor_test(void);

//4ms一次
void RC_Analy(void)  
{
		static uint16_t cnt,cnt_tmp;
/*             Receive  and check RC data                               */	
//	static u8 Connect_flag;
	 ///模式选择飞行程序
	
	remote_unlock(); // 解锁判断,移到外面，遥控有无数据都要进这个函数
		
	//控制数据优先级
	if(Nrf_Erro==1 || WIFI_UFO_Err==1) //Nrf_Erro为1，表示2.4G已经接收到数据
	{ 	
				//新增，
				cnt=0;//新增，收到数据了，错误计数要清零，防止Remote数据被复位
				//if(Remote.thr!=0)cnt=0;//确实收到数据了，失联后，thr会自动赋值
				//if(Remote.AUX1!=0)cnt=0;//确实收到数据了，应该是1000~2000，而不是0
				//不能用这个判断有无收到数据
		
				//数据在 ANO_DT.c中接收，并更新 结构体 Remote


				//在control.c 的 Mode_Controler()函数，做摇杆值的进一步解析，得到目标姿态角度。
		
				{
//							const float roll_pitch_ratio = 0.04f;
								const float yaw_ratio =  0.0015f;    
					
//							pidPitch.desired =-(Remote.pitch-1500)*roll_pitch_ratio;	 //将遥杆值作为飞行角度的期望值
//							pidRoll.desired = -(Remote.roll-1500)*roll_pitch_ratio;
					
//							if((Flow_Err ==0) && (mini.ok == 1) && (FlightData.High.bara_height < 40))//光流模块在位  数据有效   //直接控制速度 并且关掉外环计算
//							{
//								Flow_SpeedPid_y.desired = -(Remote.pitch-1500)*roll_pitch_ratio;   //摇杆控制速度期望值 
//								Flow_SpeedPid_x.desired = -(Remote.roll-1500) *roll_pitch_ratio;
//								pidPitch.desired = -(Remote.pitch-1500)*0.02f;  //遥控器摇杆控制姿态外环期望值
//								pidRoll.desired  = -(Remote.roll-1500) *0.02f;  //遥控器摇杆控制姿态外环期望值 								
//							}
//							else
//							{
//								pidPitch.desired = -(Remote.pitch-1500)*roll_pitch_ratio;
//								pidRoll.desired  = -(Remote.roll-1500) *roll_pitch_ratio;  
//							}
//					
					
					
					
							// 注意： pidPitch.desired 和 pidRoll.desired  在 control.c 的 Mode_Controler()
							//偏航角 摇杆值与pid期望角度换算，也转移到  control.c 的 Mode_Controler()
							/*
					    if(Remote.yaw>1820)
							{
								pidYaw.desired -= 0.75f;	
							}
							else if(Remote.yaw <1180)
							{
								pidYaw.desired += 0.75f;	
							}	
							*/
							
							
				}
				
				//remote_unlock(); // 解锁判断,移到外面，遥控有无数据都要进这个函数
				
				//以下新增
				mpu_adjust();//陀螺仪校准判断
				motor_test();//马达测试判断,,,连按4次紧急锁定
				
				
				if(AUX86_isH()) LED.off=1; //关闭LED
				else LED.off=0;//LED正常
				
  }
//如果3秒没收到遥控数据，则判断遥控信号丢失，飞控在任何时候停止飞行，避免伤人。
//意外情况，使用者可紧急关闭遥控电源，飞行器会在3秒后立即关闭，避免伤人。
//立即关闭遥控，如果在飞行中会直接掉落，可能会损坏飞行器。
  else
	{					
				cnt++;
				if(cnt>500)						//2秒没有遥控器信号 判断遥控器失联 信号断线 自动下降保护
				{	
					Remote.rst_count++;//记录失联次数，如果没有开遥控，每4秒记录会增加一次。
					
					//1499，1500，1501表示通讯断了
					Remote.roll = 1499;  //通道1    数据归中
					LIMIT(Remote.roll,1000,2000);
					Remote.pitch = 1500;  //通道2		数据归中
					LIMIT(Remote.pitch,1000,2000);
					Remote.yaw =  1501;   //通道4		数据归中
					LIMIT(Remote.yaw,1000,2000);		
					
					
					//重启rf模块
					//if(Remote.thr < 1030)						//判断油门
					if( Remote.thr < 1030 || !ALL_flag.unlock )// 不在飞行状态可以立即重启rf模块，灯也会随之变成交替闪烁
					{
							cnt = 0;
							Remote.thr =1000;						//关闭油门
							ALL_flag.unlock = 0; 				//退出控制
							//LED.status = AllFlashLight; //开始闪灯,表示失联,在2.4G复位时，会更改灯的状态
							NRF24L01_init();						//复位一下2.4G模块，复位时会变为交替	
					}
					//逐渐关闭油门
					else
					{	
						cnt = 810;
						if(cnt_tmp++>100)                 //控制油门减小的时间
						{
							cnt_tmp=0;
//							printf("Remote.thr: %d  \r\n",Remote.thr);				//串口1的打印			
							Remote.thr = 	Remote.thr-20;   //通道3 油门通道在原来的基础上自动慢慢减小  起到飞机慢慢下降
						}				
					}
					LIMIT(Remote.thr,1000,2000);		
					
					
				} 
	}	
}


void mpu_adjust(void)//陀螺仪校准判断
{
	
	//校准按键被按下
	if(AUX811_isH()){
			if(Aux_Rc.setMpu==0){ //还没开始校准
						Aux_Rc.setMpu=1; //开始校准
						//校准MPU6050，水平以及静止校准
				
						//灭灯
						bLED_H();	 	//前灯灭
						aLED_H();		//前灯灭
						fLED_H();		//后灯灭
						hLED_H();		//后灯灭
				
						IWDG_ReloadCounter();//喂狗	
						//有个别mcu超过0.5秒就会重启
						delay_ms(500); //熄灭1秒
						IWDG_ReloadCounter();//喂狗	
						//delay_ms(500); //熄灭1秒
						//IWDG_ReloadCounter();//喂狗	
				
						MpuGetOffset_save(); //校准陀螺仪并保存数据
						IWDG_ReloadCounter();//喂狗	
				
						//delay_ms(100);
						//成功后闪烁
						for(u8 i=0; i<10; i++){
									//亮灯
									bLED_L();	 	//前灯亮
									aLED_L();		//前灯亮
									fLED_L();		//后灯亮
									hLED_L();		//后灯亮
							
									IWDG_ReloadCounter();//喂狗	
									delay_ms(30); //打开1秒
							
									bLED_H();	 	//前灯灭
									aLED_H();		//前灯灭
									fLED_H();		//后灯灭
									hLED_H();		//后灯灭
							
									IWDG_ReloadCounter();//喂狗	
									delay_ms(30); //打开1秒
						}
						
							
			}
	
	
	}
	else{
				Aux_Rc.setMpu=0; //已经松开
	
	}

}





void motor_test(void)//马达自动测试判断,,,连按3次紧急锁定
{
	
	//测试按键被按下
	if(AUX822_isH()){
			if(Aux_Rc.testMotor==0){ //还没开始校准
						Aux_Rc.testMotor=1; //开始校准
						//校准MPU6050，水平以及静止校准

						u8 motor=100; //马达测试力度，0~1000
		
						//灭灯
						bLED_H();	 	//前灯亮
						aLED_H();		//前灯亮
						fLED_H();		//后灯灭
						hLED_H();		//后灯灭
		
						//PWM0 = 0;
						//PWM1 = 0;
						//PWM2 = 0;
						//PWM3 = 0;
						PWM0 = TIM2_DUTY + 0;
						PWM1 = TIM2_DUTY + 0;
						PWM2 = TIM3_DUTY + 0;
						PWM3 = TIM3_DUTY + 0;

						//默认占空比，空心杯为0，无刷为1000，
						//TIM2_DUTY, TIM3_DUTY	

						bLED_L(); //1，红，右下
						IWDG_ReloadCounter();//喂狗	
						PWM0 = TIM2_DUTY + motor; //转动
		
						delay_ms(500); //轮流点亮0.5s
						IWDG_ReloadCounter();//喂狗	
						PWM0 = 0; //无刷重启
						
						delay_ms(500); //暂停0.5s
						IWDG_ReloadCounter();//喂狗	
						
						delay_ms(500); //暂停0.5s
						IWDG_ReloadCounter();//喂狗	
						PWM0 = TIM2_DUTY + 0; //结束后无刷响，灯灭
						bLED_H(); //1，红，右下

		
		

						fLED_L(); //2，蓝，右上
						IWDG_ReloadCounter();//喂狗	
						PWM1 = TIM2_DUTY + motor;
						
						delay_ms(500); //轮流点亮1ms
						IWDG_ReloadCounter();//喂狗	
						PWM1 = 0;
						
						delay_ms(500); //暂停1ms
						IWDG_ReloadCounter();//喂狗	
						
						delay_ms(500); //暂停1ms
						IWDG_ReloadCounter();//喂狗	
						PWM1 = TIM2_DUTY + 0;
						fLED_H(); //2，蓝，右上

		
		

						hLED_L(); //3，蓝，左上
						IWDG_ReloadCounter();//喂狗	
						PWM2 = TIM3_DUTY + motor;
						
						delay_ms(500); //轮流点亮1ms
						IWDG_ReloadCounter();//喂狗	
						PWM2 = 0;
						
						delay_ms(500); //暂停1ms
						IWDG_ReloadCounter();//喂狗	
						
						delay_ms(500); //暂停1ms
						IWDG_ReloadCounter();//喂狗	
						PWM2 = TIM3_DUTY + 0;
						hLED_H(); //3，蓝，左上

		
		

						aLED_L(); //4，红，左下
						IWDG_ReloadCounter();//喂狗	
						PWM3 = TIM3_DUTY + motor;
						
						delay_ms(500); //轮流点亮1ms
						IWDG_ReloadCounter();//喂狗	
						PWM3 = 0;
						
						delay_ms(500); //暂停1ms
						IWDG_ReloadCounter();//喂狗	
						
						delay_ms(500); //暂停1ms
						IWDG_ReloadCounter();//喂狗	
						PWM3 = TIM3_DUTY + 0;
						aLED_H(); //4，红，左下
						
						

	
	

			}
			
			
			

	}
	else{
				Aux_Rc.testMotor=0; //已经松开
	}
	
}
	

/*****************************************************************************************
 *  解锁判断
 * @param[in] 
 * @param[out] 
 * @return     
 ******************************************************************************************/	
//RC_Analy()调用，4ms一次,,,,,//有数据Nrf_Erro=1,无数据Nrf_Erro>1。
//遥控那边发数据，差不都10ms一次
void remote_unlock(void)   //解锁数据解析
{

	volatile static uint8_t status=WAITING_1;
	static uint16_t cnt=0; //6s未起飞自动上锁
	static uint8_t status_old=WAITING_1;
	
	//尚未解锁飞控时，当遥控器疑似失联后，立即重设解锁三部曲，
	//防止多次重启遥控器触发解锁(当油门摇杆正好在最高位时，容易误触发)
	if(Nrf_Erro>10 && !ALL_flag.unlock) {//等待解锁中

				//尚未解锁飞控时，如果遥控器失联了，则重新解锁三部曲，	
				status = EXIT_255; //等待重新解锁三部曲，防止误触发
		
		
				if(status_old!=EXIT_255){
						//遥控疑似失联，会闪一下，如果当前尚未解锁，正在解锁程序，需要重新开始
						//此时尚未判断为正式失联，如果在飞行中，也有可能是因为距离太远，导致信号不是太好。
						//但解锁飞控时，一般都离得很近，如果解锁过程Nrf_Erro>10，多半是遥控关机了。
						fLED_Toggle(); hLED_Toggle();bLED_Toggle();aLED_Toggle();
						delay_ms(50); //1ms
						fLED_Toggle(); hLED_Toggle();bLED_Toggle();aLED_Toggle();
						delay_ms(50); //1ms
				
				}

		
	
				

	}
	
	status_old = status;
	
	//有遥控数据才继续
	if(Nrf_Erro !=1 ) return; //当前结构体Remote数据不是遥控发来的最新数据。
	
	
	//if(Remote.thr<1200 &&Remote.yaw>1800)                         //油门遥杆右下角锁定飞机
	//{
	//	status = EXIT_255;
	//}
	
	//左下角紧急锁定（兼容第一代上锁功能）
	if(Remote.thr<1050 &&Remote.yaw<1150)                         //油门遥杆左下角锁定
	{
		
			if( LED.status == SimpleFlash ) { //已连上遥控
						bLED_H(); fLED_H(); aLED_H(); hLED_H(); //紧急锁定会导致飞机掉落，通过LED状态提醒，触发了紧急锁定
			
			}

			if(ALL_flag.unlock) status = EXIT_255; //上锁	
		
	}
	
	
	
	//新增解锁技能
	//AUX82_isH代表进入了油门屏蔽, yaw:2000; thr:1000 -> 自动解锁
	if(AUX82_isH() && Remote.thr==1000 && Remote.yaw==2000 )      
	{
			status = WAITING_4;
	}
	
	
	//能走到这里，Nrf_Erro都是1，除了飞控刚开机时
	switch(status)
	{
		
		//解锁三步奏，油门最低->油门最高->油门最低 看到LED灯不闪了 即完成解锁
		case WAITING_1: //默认值  等待解锁

			//if(Remote.thr<1200)          //1.拉低油门 
			if(Remote.thr<1200)          //1.拉低油门
			{			 
					 status = WAITING_2;			//等待2拉高油门	 
			}		
			
			

			break; //新增
			
			
		case WAITING_2:
			//if(Remote.thr>1850 )        //2.拉高油门  
			if(Remote.thr>1850)        //2.拉高油门  
			{		
						static uint8_t cnt = 0;
					 	cnt++;		
						if(cnt>5) //最高油门需保持200ms以上，防止遥控开机初始化未完成的错误数据
						{	
								cnt=0;
								status = WAITING_3; //等待3拉低油门，需要在3秒内拉低油门，否则需要重新解锁三部曲
						}
			}			
			else{
						//尚未拉高油门
						/*
						static uint8_t cnt1 = 0;
					 	cnt1++;		
						if(cnt1>210) //如果等待2秒未拉高油门，需要重新拉低油门，重启解锁三部曲
						{	
								cnt1=0;
								status = WAITING_1; //重回 WAITING_1
						}
						*/

			}
			break;
			
			
		case WAITING_3:
			//if(Remote.thr<1150)     //3.拉低油门解锁   
			if(Remote.thr<1150)          //3.拉低油门解锁, 防止遥控连续开关机误触发Remote.thr>=1000
			{			 
						//取消解锁，需要松开H+后，重新打高打低
						if(AUX87_isH() && Remote.test_motor!=0){ //正在手动测试电机，且电机正在转动（按了H+）
								status = EXIT_255; //等待重新解锁三部曲，	 进入EXIT_255会闪一下
							
						}
						
						else{
								status = WAITING_4;			//将进入4解锁成功程序
							
						}

			}		
			else{
						//尚未拉低油门
						static uint16_t cnt2 = 0;
					 	cnt2++;		
						//操作超时
						if( cnt2>310 ) //3s内没有拉低油门则重新回到等待状态
						{	
								cnt2=0;
								status = EXIT_255; //等待重新解锁三部曲，	
							
								//解锁超时，会闪一下
							
								fLED_Toggle(); hLED_Toggle();bLED_Toggle();aLED_Toggle();
								delay_ms(50); //1ms
								fLED_Toggle(); hLED_Toggle();bLED_Toggle();aLED_Toggle();
								delay_ms(50); //1ms
							
						}
						
						

						
				
			}
			break;	

			
		case WAITING_4:	//解锁成功
				ALL_flag.unlock = 1;  //解锁
		

				status = PROCESS_31; //将进入上锁检测程序
				LED.status = AlwaysOn;	
				Aux_Rc.yaw_stick = 0.0f; //摇杆目标偏航角清零
		
				//复位光流，这样可以举在空中飞，地面起飞时，光流为0
				mini.flow_x_i=0;
				mini.flow_y_i=0;
				pixel_flow.fix_x = 0;
				pixel_flow.fix_y = 0;	
				
				//重启光流模块，出现问题后，重启后下次飞行恢复
				ANO_Uart1_Put_String( (u8 *)"\r\nr_e_s_t_a_r_t\r\n" );
		
				//新增每次解锁时，重启陀螺仪
				MPU_Err=1;
				SysTick_count=0;
				IIC_Init();//有个别单片机需要重新初始化单片机io,陀螺仪才能恢复正常。
				delay_ms(100);
				MpuInit();//每次解锁重启陀螺仪，注意有的陀螺仪有设置重启时暂时无法输出最新数值时，是输出上次数值，还是输出0xff，这里需要陀螺仪输出上次值。避免数值出现太大波动
				
				//气压清零
				baro_start=1;  //气压清零标志, 重设气压零坐标
				
				break;		
		
		
		case PROCESS_31:	//进入解锁状态，检测何时需要重新上锁
			
				//未起飞，6s后自动上锁
				if(Remote.thr<1020)
				{
					if(cnt++ > 700)      // 解锁后  不动油门遥杆处于最低6S自动上锁 注意，有遥控数据才会调用 700*10ms=7秒
					{								
						status = EXIT_255;								
					}
				}
				else					
					cnt = 0; //油门过小计数清零
				
				
				//else if(!ALL_flag.unlock)   //判断其他锁定不应只在油门>1020时。 
				if(!ALL_flag.unlock)          //Other conditions lock 
				{
					status = EXIT_255;				
				}

				
				
			break;
				
		case EXIT_255: //进入锁定
			
			//解锁超时，转移到  WAITING_3 ，这里有可能是从飞行状态转过来，飞机还在飞行中
		  //fLED_Toggle(); hLED_Toggle();bLED_Toggle();aLED_Toggle();
			//delay_ms(50); //1ms
		  // fLED_Toggle(); hLED_Toggle();bLED_Toggle();aLED_Toggle();
			//delay_ms(50); //1ms 此时可能飞机还在飞，不要用延时，转到 WAITING_1
		
			status_old = EXIT_255;
		
			//指示灯切换为锁定状态, AllFlashLight表示LED闪烁，代表上锁状态; 有可能是同步闪烁SimpleFlash；也有可能是交替闪烁AlternateFlash
			//AlwaysOn表示解锁常亮，DANGEROURS表示解锁双闪。
			if( LED.status == AlwaysOn || LED.status == DANGEROURS ) LED.status = AllFlashLight;	 //LED状态 从解锁状态，变化为上锁状态
			
			cnt = 0;
			LED.FlashTime = 300; //300*3ms		
			ALL_flag.unlock = 0;
			status = WAITING_1;
			break;
		
		
		default:
			status = EXIT_255;
			break;
	}
}
/***********************END OF FILE*************************************/







