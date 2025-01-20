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
#include "ALL_DEFINE.h" 
#include "control.h"
#include "pid.h"
#include "attitude_process.h"
#include "flow.h"
#include "spl06.h"
#include "ADC.h" //获取电池电压
//------------------------------------------------------------------------------
#undef  NULL
#define NULL 0
#undef  DISABLE 
#define DISABLE 0
#undef  ENABLE 
#define ENABLE 1
#undef  REST
#define REST 0
#undef  SET 
#define SET 1 
#undef  EMERGENT
#define EMERGENT 0

#define REMOTE_THR Remote.thr
#define REMOTE_PITCH Remote.pitch
#define REMOTE_ROLL Remote.roll
#define REMOTE_YAW Remote.yaw
//#define measured FeedBack
#define Expect desired 	

#define HIGH_LASER		1  //高度数据来源 激光
#define HIGH_BAR			2  //高度数据来源 气压

float Throttle_out; //飞控油门输出值 //遥控 + 定高 输出值
//------------------------------------------------------------------------------
PidObject *(pPidObject[])={&pidRateX,&pidRateY,&pidRateZ,&pidRoll,&pidPitch,&pidYaw   //结构体数组，将每一个数组放一个pid结构体，这样就可以批量操作各个PID的数据了  比如解锁时批量复位pid控制数据，新手明白这句话的作用就可以了
		,&pidHeightRate,&pidHeightHigh,&Flow_SpeedPid_x,&Flow_PosPid_x,&Flow_SpeedPid_y,&Flow_PosPid_y
	
		//一共12个，姿态模式6个，悬停模式增加6个（定高2个，定点4个）
};

/**************************************************************
 *  Height control
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/

//static uint8_t set_high_desired = 0; //定高高度已设定

//int16_t  HIGH_START =180;   //一键起飞目标高度
int16_t  HIGH_START =100;   //一键起飞初步高度

//uint32_t Control_high = 0; //当前高度  ，，
float Control_high = 0; //当前高度  ，，需要改成浮点；；mini.flow_High与FlightData.High.bara_height都是float单位厘米
u8 Control_high_type =0;  //高度数据来源 


//float thr_balance=500; //能维持高度的自动平衡油门，当非定高模式进来时，也会把当前油门当作自动平衡油门


//新增
int8_t thr_isChange=0;//摇杆拨动大于 +/-10，0.2秒后设置标识
int16_t thr_raw_adds=0; //本次增加的摇杆值

float voltage_static = 4.0;//单位 1.0v,频闭油门时的电池电压

float thr_adjust=0;//油门平衡修正值,一定要浮点。
float thr_hight_modify=0;//油门暂时调整，高度加或减按键按下。
float thr_base_add=0;//一键定高起飞，补偿用户油门摇杆值

u8 hight_mode=0;//1：定高飞行模式，由用户手动调整平衡油门，不会自动调整平衡油门，手动定高起飞全程都为模式1；在模式2下，只要动了油门摇杆就会变为模式1
								//2: 定高飞行中，且程序自动调整平衡油门，一键起飞完成模式3会转成模式2，在飞行过程中，从姿态模式切换到定高模式，也会从模式0进入模式2
								//3：一键定高起飞中，按住key2键，定高起飞进行中为模式3；松开k2，变为模式2
								

int8_t hight_modify=0;//高度调整模式，此模式下，不要修改平衡油门


int8_t modify_thr_adjust=0;//当我们在修改 平衡油门thr_adjust时，应该对 内环速度pid输出限幅



/**************************************************************
 *  //高度控制器     气压
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
//10ms调用一次
void HeightPidControl(float dt)  //高度控制器     气压
{
	
	
		volatile static uint8_t status=WAITING_1;
		int16_t acc;       //当前飞行器的加速度值
   	int16_t acc_error; //当前加速度减去重力加速度则为上下移动的加速度
		static int16_t acc_offset;//重力加速度值	 
	
		//已弃用 这3个标志
		//static float thr_hold = 0; //进入高度时记录当前油门值
		//static uint8_t set_high = 0,High_breaktime;
	

		//if(mini.flow_High<400 && !AUX833_isH())//4米以下，并且默认激光优先(非气压优先)
		if(Flow_Err == 0 && mini.flow_High<400 && !AUX833_isH())//4米以下，并且默认激光优先(非气压优先)
		{
				Control_high=mini.flow_High;      //更新激光高度
				Control_high_type = HIGH_LASER;		//高度数据来源 激光
			
		}
		//4米以上，或者气压优先
		else if(SPL_Err == 0)
		{
				Control_high=FlightData.High.bara_height ;	 //更新气压高度
				Control_high_type = HIGH_BAR;			//高度数据来源 气压
			
		}
		 
		
		
		//----------------------------------------------	
		{ //获取垂直速度数据

			
				acc = (int16_t)GetAccz(); //获取无人机垂直于地面的加速度，不一定与无人机Z轴平行；通过陀螺仪3轴各自加速度 计算GetAccz()
			

				if(!ALL_flag.unlock)      //取得静态ACC值 
				{
						acc_offset = acc; //记录无人机在地面垂线方向的静态加速度(数值通常与重力加速度相等 8192 )
				}
				acc_error = acc - acc_offset;	  //获取无人机在地面垂线方向的运动加速度（不含重力），与无人机上升下降有关，与哪个轴指向地面无关。并非xyz某个轴单独的运动加速度
					
				//注意加速度单位，与结构体 MPU6050 的XYZ加速度相同，重力加速度g为8192，并没有换算为公制加速度单位 9.8 m/s2
				//从数量级看，相当于 mm/s2的加速度单位。没有严格换算为了简化计算量，单片机的浮点乘除法运算速度有限，避免响应迟钝。
			

				
				{//此处做一个速度与高度的互补滤波 
						static float last_high;
						//acc单位是毫米，气压计以及激光高度Control_high厘米
						pidHeightRate.measured = (pidHeightRate.measured + acc_error * dt)*0.98f + 0.02f*(Control_high - last_high)/dt; //速度环永远是主调，所以互补滤波关键就抓在速度环这里
						
						//pidHeightRate.measured = (pidHeightRate.measured + acc_error * 0.1f * dt)*0.98f + 0.02f*(Control_high - last_high)/dt; //速度环永远是主调，所以互补滤波关键就抓在速度环这里
					
					
						last_high =  pidHeightHigh.measured = Control_high;  //实时高度反馈给外环
					

					
				}	
				
				
				
				
				
		}
		
		
		
		//----------------------------------------------紧急终止飞行
		if(ALL_flag.unlock == EMERGENT) //意外情况，请使用遥控紧急上锁，飞控就可以在任何情况下紧急中止飞行，锁定飞行器，退出PID控制
				status = EXIT_255;
		//----------------------------------------------控制
		
		//static u8 thr_stop_3s=1;//已经停止了3s
		//static uint16_t stop_count=0;//摇杆停止计数
		
		static u8 enter_1s=0;
		static uint16_t enter_count=0;
		
		static u8 hight_modify_cnt=0;
		static uint16_t fly_key2_cnt=0; //一键定高起飞刚开始1秒，计时
		
		
		//当屏蔽油门时，记录电池电压，为进入一键定高起飞做准备。
		if(Aux_Rc.thr_mask  && ALL_flag.unlock){ //当前尚未起飞。
			
					//voltage_static = (float)voltage/1000.0f;
					voltage_static += ((float)voltage/1000.0f - voltage_static)*0.8;
		
		}

		
		//Aux_Rc.debug8 = status; //调试
		
		switch(status)
		{
			
			

			
				//当前为非定高模式，是否进入定高
				case WAITING_1: //检测定高
						if(  (ALL_flag.height_lock || ALL_flag.height_lock_qiya) && ALL_flag.unlock)   //已解锁，并在定高模式，但有可能待飞，也有可能正在飞行
						{
							if(ALL_flag.height_lock) LED.status = DANGEROURS;//常亮 2秒 闪烁 1秒, 只有悬停才改变灯状态，只是气压定高不改变灯
							status = WAITING_2;  //进入定高模式
							
							//thr_hold=0; 				        //清除记录定高时的油门
							
						}
						

						hight_mode=0; //已退出定高待重新进入，清除定高具体模式

						
						break;
						
					
					
				//从别的模式第一次进入定高
				case WAITING_2: //定高前准备
						//如果是在飞机锁定状态 或者落地状态，切换的定高模式呢？？？？？
						//此时的油门是0，那么就使用500（1500）作为默认油门
						//另外这个油门要记录带油门比例的真实油门，而不是摇杆值，
						//但后面控制方向时，又需要看纯摇杆值。
				
						//解锁状态，并且非屏蔽状态，并且油门大于1400，
						//这种时候进来基本上可以判断飞机正在姿态飞行，这个油门可以作为平衡油门
						//否则，其他情形进来，飞机在落地状态，此时油门无法保证可以作为平衡油门
				

				
						//thr_hold = Remote.thr -1000;  //记录定高时的油门
						//set_high = 0;
				
						pidHeightHigh.desired = Control_high;//默认在飞行中，定高时把进入时的高度作为期望高度
						
						enter_1s=1; //进入模式2计时，启用1秒刹车程序

						//thr_stop_3s=1;//未拨动超3秒
				
						hight_mode=2;//先假设已经在飞行中(自动修正平衡油门)
						thr_adjust=0;//平衡修正复位，原来飞行在姿态模式，油门摇杆值已经就是平衡位置。
						thr_base_add=0;//基础油门补偿复位，一键定高起飞，补偿用户油门摇杆值
						thr_hight_modify=0; //用于上升/下降的临时增加油门复位
				
						//也可以由油门屏蔽状态加油门值来判断，走到这里都是解锁状态
						//如果油门值<1200或者油门屏蔽状态，进入定高，那么一定时待飞状态。
						//if(Control_high<30){ //当前高度很小（小于30cm），说明尚未起飞。
						//if(Aux_Rc.thr_mask || Remote.thr<1200){ //当前尚未起飞。
						if(Remote.thr<1200){ //当前尚未起飞。正处于定高模式	

									pidHeightHigh.desired = 0; //此时最好设为0，否则高度内环速度pid是有输出油门的。
									thr_adjust=0; //这种情形，用户需要手动定高起飞，平衡油门复位

									hight_mode=0;//尚未起飞,  0 代表姿态模式，或者定高模式但没有起飞。屏蔽状态下是姿态模式。

							


						}
				
						//按下一键定高起飞时，一定会从姿态切换到定高，所以一定会走到这里。
						
						
						//一键定高起飞模式，长按key2,只有油门在1400~1600，才能进入一键起飞，遥控那边设定的。
						if(Aux_Rc.fly_key2 && hight_mode!=3 && Remote.thr>1200 ) //正在一键起飞，一键起飞首先要set+key2屏蔽油门（进入姿态模式），再长按key2，进入定高模式
						{
							
								//Aux_Rc.debug7 += 1; //调试
								
								hight_mode=3; //进入一键定高起飞
								pidHeightHigh.desired = HIGH_START;  //设定一键定高起飞最低高度，用户按住起飞键不放会不断增加高度
						

								//这个值，1s内减为100，每次减1
#if (FLY_TYPE < 3)		//空心杯
											thr_hight_modify = 280; // 有刷 开始上升临时油门再增加一些，有刷爆发力弱
#else //无刷
											//无刷起飞相比与平时悬停不用大太多，爆发力好
											thr_hight_modify = 200; //用于上升的临时增加油门，相当于按下 H+					
#endif
							
							
								//这个值，1s内减为50，每次减1.5
								pidHeightRate.desired = 200;//期望速度 2米/秒，逐渐减为0.5米/秒
							
								fly_key2_cnt=200;//起飞2秒内，逐渐降速
							
							
							
								//进入一键定高起飞过程，光流pid参数设置为运动模式，等起飞刹车结束时，恢复为默认模式。
#if ( FLY_TYPE==3 ) //无刷 光流pid运动参数，一键定高起飞或方向摇杆移动中
								//进入模式3，或者检测到方向摇杆动作，则调入光流运动pid参数
								if(flow_pid_param_type!=2) flow_pid_param_move();//无刷电机光流pid，起飞和手动移动时，pid参数较大

#endif

							
								
												
								//1400+/-100. 1300~1500之间，用户摇杆1300~1700，我们把他调为1400
								thr_base_add = 1400 - Remote.thr; //定高起飞的基础油门默认1400，如果摇杆值是其他值，需要补偿。一键定高起飞，补偿用户油门摇杆值
								
								//根据油门屏蔽时的电池电压，调整基础油门补偿值
								
								//无刷桨叶：3种规格，58/64/70，最小58桨叶稳定性好，但升力小，需要把基础油门设为600（1600）
								//或者不要使用一键定高起飞，而使用手动定高起飞，这样就可以查看到每个电池电压状态下的的最佳油门位置。
								
								

								#if (FLY_TYPE >= 3) 
											thr_base_add += 100;//基础油门从1400加到1500；现在用的58mm小桨叶更稳定，但升力略小，需要在这里增加100油门
								#endif
								
								
								//实际 3.5v 以下就飞不起来了(3.5v起飞后电压为3.1v，有0.4v压降)。							
								
								/*  待机时3.5v提示电压低，飞行时3.3v提示电压低
								 4.2v  ~4.0v   ~3.8v    ~3.5v    ~3.2v    ~3.0
								 -40    +0     +40     +150     +300      +200 
								>4.0v  + (4.0-v)*200
								>3.8v  + (4.0-v)*200
								>3.5v  + (4.0-3.8)*200 + (3.8-v)*500
								>3.2v  + (4.0-3.8)*200 + (3.8-3.5)*500 + (3.5-v)*1000
								>3.0v  + (4.0-3.8)*200 + (3.8-3.5)*500 + (3.5-3.2)*1000 + (3.2-v)*1000
								*/
								
								//thr_base_add =100;   //测试
								//voltage_static = 2000.0f/1000.0f;   //测试
								
								if(voltage_static>4.0){  // >3.9
										thr_base_add -= (voltage_static-4.0)*200; //补偿4.0~4.2v,    4.2v时补偿-40
									
								}
								else{ //<=4.0

										if(voltage_static>3.8){  
											
													thr_base_add += (4.0-voltage_static)*200; //补偿 3.8~4.0v
										}
										else{
													thr_base_add += (4.0-3.8)*200;  //+40   //补偿 3.8~4.0v
											
											
											
													if(voltage_static>3.5){
														
																thr_base_add += (3.8-voltage_static)*500; //补偿 3.5~3.8v
													}
													else{
																thr_base_add += (3.8-3.5)*500; //+150  //补偿 3.5~3.8v
														
																
														
														
																if(voltage_static>3.2){
																	
																			thr_base_add += (3.5-voltage_static)*1000; //补偿 3.3~3.5
																}
																else{//3.2v以下
																			thr_base_add += (3.5-3.2)*1000;  //+300  //补偿 3.3~3.5
																	
																			thr_base_add += (3.2-voltage_static)*1000; //补偿3.3以下

																}
													}
										}
										
								}
								


								
							
						
						}


				
						
						
						
						
						
				
						status = PROCESS_31;
				
				
				
				

				
				
				
				
				
				
				
				
				
						break; 
					
				//已经在定高模式，维持高度，10ms执行一次
				case PROCESS_31://进入定高	

						if(!ALL_flag.height_lock && !ALL_flag.height_lock_qiya)     //退出定高,  非悬停模式，并且非气压定高
						{
								LED.status = AlwaysOn ;
								status = EXIT_255; //退出定高
						}
						
						//下降到接近地面时，按一下遥控 【LOCK】 键，即可着落。
						//此处可以实现降落过程的自动熄火，即当用户按着遥控器"下"按键，
						//如果突然无人机在地面垂线方向的加速度 acc_error 变为很大的向上数值（向上为负值，与重力方向相反）
						//那么代表无人机已经碰触到地面了，此时就可以熄火了（锁定无人机）
						//用户按下遥控“下”键
						//if( AUX88_isH() && acc_error<-1000){ //重力方向上的运动加速度 acc_error<-1000，-1000表示运动加速度向上，重力为8192
						//			ALL_flag.unlock = EMERGENT;//0 紧急锁定

						//}
						
						
						
				
						//定高一键起飞，单独判断，如果高度已经快达到，
						//并且key2按键按着（一键起飞状态），不断增加期望高度, 以一个稍快的速度每秒25厘米调整

						//如果有按高度增加或者减少键，则相应不断增加或减少期望高度。
						//以一个不太快的速度每秒25厘米调整
				
						//用户按动了高度调整按键 H+/-
						//AUX87_isH()
						//AUX88_isH()
						//按下按键
						if(AUX87_isH() || AUX88_isH()){
							
							
								//进入高度调整模式
								if (AUX87_isH()) hight_modify=1;//高度调整模式，此模式下，不要修改平衡油门
								if (AUX88_isH()) hight_modify=-1;//高度调整模式，此模式下，不要修改平衡油门
							
								hight_modify_cnt=0;
								
								//按下按键时，会产生一个油门临时增量，thr_hight_modify约为+/-100左右
								pidHeightHigh.desired = Control_high;//调整高度期间时把进入时的高度作为期望高度，
								//或者 +/- (0.1cm/0.01s)也可以
							
						}
						//已经松开按键
						else{
							
								//高度调整模式结束，如果时刚松开，那么重设期望高度
								if(hight_modify!=0){ //松开后第一次进来
									
											hight_modify=0; //已松开按键，进来31流程才能复位，有时会因为无法即使复位，油门那边有问题
											hight_modify_cnt=50; //高度调整结束后的1s需要再次刷新期望高度
								}
								
								
								//刹车过程，用当前高度重写期望高度。刹车完成，维持期望高度。
								if(hight_modify_cnt){ //1s计时
											hight_modify_cnt--; //从50到0
											pidHeightHigh.desired = Control_high;//重写期望高度
								}

							
						}
				
						
						
						
						
						
						
						
						
						
						


						
						



										
						//一键定高起飞被按下
						if(Aux_Rc.fly_key2){
										
													
									//一键定高起飞中
									if(hight_mode==3){
																			
												//油门构成   1.动态（小幅维持），2.基础（摇杆）,2.1基础油门补偿，3.平衡（修正），4.临时增减（大幅运动）
												//1. 实现定高维持，内环速度pid输出的油门，当2+3平衡油门适合，1直接与平衡时的加速度有关。
												//2+3. 抵消飞机重力的油门（用3的变化来弥补基础油门2的不足）
												//4：实现上下大幅运动。当按下H+/-或一键定高起飞才需要
												//2.1只有在一键定高起飞时才有，进入模式1的时候，会逐渐转移到3平衡油门
										
										
												//1由内环pid输出决定，2由摇杆位置决定，3由期望加速度和实测加速度差异的积分决定。4由上升还是下降需求决定
												//假设我们现在电压下，希望基础=1400，那么当用户设定了1500，平衡预设值=-100
												//这里导致平衡修正值的范围与模式2不同，范围要大很多，这个范围应该时动态的，平衡预设值+/-200
										
												//4.临时增加值，我们暂时设为100（油门）
										
												//当前实测加速度 VS 1+4  作为pid的测量值与期望值，这点与模式2不同，模式2为 实测加速度 VS 1动态油门
												//用这个pid的输出，来修正3平衡油门。

												//期望高度以及期望速度设定
												//2种方法，看哪种好？
												//优选位置方案，预设一个位置，然后起飞期间不断增加。
												//1.设置基础补偿，2.设置临时增减，3:设置期望高度 按一定速度递增。初始期望高度已经在上面进入模式3时设置
												
												//1400+/-100. 1300~1500之间，用户摇杆1300~1600，我们把他调为1400
												//thr_base_add = 1400 - Remote.thr; //定高起飞的基础油门默认1400，如果摇杆值是其他值，需要补偿。一键定高起飞，补偿用户油门摇杆值
												
												//根据油门屏蔽时的电池电压，调整基础油门补偿值
												
												/*
												4.2v   ~3.8v   ~3.5v    ~3.2  	~3.0V
												 +0     +24      +48     +96     +146
												>3.8v  + (4.2-v)*60
												>3.5v  + (4.2-3.8)*60 + (3.8-v)*80
												>3.2v  + (4.2-3.8)*60 + (3.8-3.5)*80 + (3.5-v)*160
												>3.0v  + (4.2-3.8)*60 + (3.8-3.5)*80 + (3.5-3.2)*160 + (3.2-v)*250
												*/
												
												
												//thr_hight_modify = 200; //用于上升的临时增加油门，相当于按下 H+
												//pidHeightHigh.desired += 0.8;   //每秒80cm，10ms加0.8cm，刚开始的起始高度为100cm
												
												//初始 临时油门  200，减到 160
												//初始速度，200厘米/秒，减到 100厘米/秒
												//刚起飞瞬间，要快要大油门，因为这段时间获得加速度，并且30厘米以内没有光流数据，容易偏。
												
												if(fly_key2_cnt>0){ //cnt从200减为0，共2秒; //每秒进来100次
															//fly_key2_cnt=200;						//起飞1秒内，逐渐降速
															fly_key2_cnt--;
															
															

#if (FLY_TYPE < 3)		//空心杯
															if(thr_hight_modify>100) thr_hight_modify -=0.5;				//油门每10ms减0.5，一共进来200次，减100，空心杯刚开始的thr_hight_modify起飞值大, 从280到180。
#else // >=3  无刷
															if(thr_hight_modify>100) thr_hight_modify -=0.2;				//油门每10ms减0.2，一共进来200次，减40，无刷刚开始的thr_hight_modify起飞值小，从200到160。								
#endif
													
													
															if(pidHeightRate.desired>50) pidHeightRate.desired -= 0.5;	//每10ms减1.5，期望速度 200厘米/秒，逐渐减为50厘米/秒
												
												}
												
												



									}
										
									
						}
						//定高起飞按键松开了
						else{
							
									//thr_base_add 需要慢慢归零(在模式1拨动油门摇杆时才会慢慢归零)
									thr_hight_modify = 0; //用于上升的临时增加油门取消
									
							
									//刚松开，模式3会变为模式2，并启用1s刹车程序
									if(hight_mode==3){
												hight_mode=2; //退出一键定高起飞，进入定高模式
												enter_1s=1; 	//启用1s刹车程序
												pidHeightHigh.desired = Control_high;  //重写期望高度

									}

						}

						//起飞过程是否需要修正平衡油门

							
							
						
						
						
						
						
						//切换到模式2，或者刚刚进入模式2
						//刚刚进入模式2（刚从姿态模式切换到定高模式）
				
						//从正在飞行的姿态模式切换到定高模式，此时应该自动修正 平衡油门
						//高度调整时，应该暂停平衡油门自动修正。
						if(hight_mode==2 && hight_modify==0){  //模式2（基础油门自动优化的定高）且未按高度调整按键
							
							
													//延时，重设期望高度
													//从姿态模式转过来时，飞机可能在上下运动，需要刹车过程，而不是一直维持进来时的高度，那样回撤太多。
													//刹车过程，用当前高度重写期望高度。刹车完成，维持期望高度不变。
													if(enter_1s){
																enter_count++; //最大到120，1.2s刹车
														
														
																//激光优先，并且高度4米以下，默认是这个模式，注意阳光直射下会干扰激光，激光模式最好在室内使用
																//此时高度数据来自于光流模块上的激光ic，数据延迟很小，刹车程序比较简单，刹车只需要1.2秒
																if(Control_high_type == HIGH_LASER){
																	
																				//刹车过程继续上升，则减小临时油门
																				if( Control_high > pidHeightHigh.desired ){//还在上升
																							thr_hight_modify = 0.0f-(Control_high-pidHeightHigh.desired)*5.0f;//10ms内实际高度每高出1cm临时油门减5
																				}
																				else{
																							thr_hight_modify = 0.0f; //临时刹车油门清零
																				}
																
																		
																		
																				pidHeightHigh.desired = Control_high;//刹车时更新期望高度，
																		
																				pidHeightHigh.desired -= 30.0f;//比当前高度小30cm，加强起飞结束时的刹车作用，防止上冲
																				pidHeightHigh.desired += enter_count/(120.0f/30.0f); //当计时到达120时，目标高度逐步恢复为当前测量到的高度
																				//if(pidHeightHigh.desired<=0.0f) pidHeightHigh.desired = Control_high; //异常高度，则取消刹车加强
																		
																				//if(enter_count>100){ //超过1s，退出刹车程序
																				if(enter_count>120){ //刹车时间增加到1.2s，超过1.2s，退出刹车程序
																							enter_1s=0;
																							enter_count=0;
																							pidHeightHigh.desired = Control_high; //刹车结束，锁定期望高度
																							thr_hight_modify=0;//刹车结束，临时刹车油门清零
																					
																				}
																
																}
																
																
																
																//气压优先，或者起飞松开高度超过4米，或者没有光流模块时；当室外阳光直射飞行时，可以采用这种模式避免激被干扰，引起的高度不稳定
																//但气压计数据高度数据，滞后很多。特别在高度迅速变化的起飞阶段，高度数据滞后非常大，估计有0.5s以上，测量高度与实际高度差异巨大
																//此时刹车需要分为如下3个阶段，共3.8秒
																else if(Control_high_type == HIGH_BAR){
																	
																	
																				//0~1.2秒, 气压高度，刹车程序，第一阶段；
																				if(enter_count<=120){
																					
																								//if( Control_high > pidHeightHigh.desired ){//无法简单判断是否在上升，因为气压高度数据滞后很大。即使已经在下降，测量值可能仍然在上升。
																								if( FlightData.High.ultra_baro_height > Control_high +30 ){ //还在上升; 气压原始高度 远大于 气压融合滤波高度。
																											//thr_hight_modify=0.0f-(Control_high-pidHeightHigh.desired)*5.0f;//10ms内实际高度每高出1cm临时油门减5
																											thr_hight_modify = (FlightData.High.ultra_baro_height - Control_high) * 10.0f; //这是刹车粗调，原始气压高度远大于融合滤波高度，说明在快速上升。
																								}
																								else{
																											thr_hight_modify=0.0f; //临时刹车油门清零
																								}
																								
																								//以下3句为刹车精调，与激光一样
																								pidHeightHigh.desired = Control_high;//刹车时更新期望高度，气压模式时，当前实测高度实际是0.5s以前的，严重滞后。
																								//但在刹车阶段只需要把期望高度设置的比测量高度低即可实现刹车，不用考虑测量高度与实际高度之间的滞后性
																								pidHeightHigh.desired -= 30.0f;//期望比当前测量高度小30cm，加强起飞结束时的刹车作用，防止上冲
																								pidHeightHigh.desired += enter_count/(120.0f/30.0f); //当计时到达120时，目标高度恢复为当前测量到的高度
																								//if(pidHeightHigh.desired<=0.0f) pidHeightHigh.desired = Control_high; //异常高度，则取消刹车加强
		
																								
																				}
																				
																				
																				//1.2秒~2.2秒 第二阶段 enter_count=120~220，这个阶段开始会重设一下期望高度
																				//高度不再急剧变化，测量高度与实际高度差异逐步缩小
																				else if(enter_count<=220){
																					
																								//刹车程序结束时，需要修正高度期望值到 真实高度附近，以免因为期望高度滞后导致期望高度太小而下落。
																								if(enter_count==121){
																											thr_hight_modify=0;//刹车结束，临时刹车油门清零
																											//由于气压数据滞后严重，随着时间过去，虽然实际高度未上升，但测量高度会快速上升。越来越接近真实高度
																											//而第一阶段刹车的期望高度是当时的测量高度，比实际高度低很多，所以在这一阶段需要修正期望高度，让他接近无人机当前实际高度
																											pidHeightHigh.desired = Control_high + 20.0f; // 期望高度+20, 大概就是现在的实际高度，防止无人机下落。
																								}

																								//除了期望高度值外，高度测量值也需要修正，修正实测高度，直到测量高度赶上真实高度, 测量修正值逐渐变小
																								//pidHeightHigh.measured = Control_high + 10.0f; // 测量高度+10, 大概就是现在的实际高度。此时(1.2秒)测量高度比实际低
																								pidHeightHigh.measured = Control_high  + 70.0f - enter_count/2.0f; //由于高度数据滞后，最开始测量高度比实际低10，逐渐变为测量高度比实际高40，从+10变为-40； 
																																																	//-60~-110       //开始为+10，比期望高度低20，防止1.2秒后快速下降；结束为-40，比即将结束后新的期望值低30，防止2.2秒后快速下降。
																				}
																				
																				//2.2秒~3.8秒 第三阶段，enter_count=220~380，，这个阶段开始会重设一下期望高度，这个高度就是刹车结束后的最终定高高度。
																				//测量高度进一步贴近实际高度，同时给出充分的时间，让后面的程序及时调整平衡油门 thr_adjust
																				else{	
																								if(enter_count==221){
																											pidHeightHigh.desired = Control_high-10.0f; //期望高度-10, 大概就是现在的实际高度。此时(2.21秒)测量高度比实际高
																											//pidHeightHigh.desired = Control_high; // 
																								}
																					
																								pidHeightHigh.measured = Control_high - 95.0f + enter_count/4.0f;  //由于高度数据滞后，最开始测量高度比实际高度高，逐渐缩小差异， 从-40变为0，一开始测量高度为-40，比期望高度低30，防止刚开始快速下降，后面逐渐变为0，能防止数据滞后导致的慢速上升
																																															  	//+55~+95  
																					
																				}	
																				
																				//刹车程序三个阶段，都已经走完。不再重设期望高度。
																				//if(enter_count>100){ //超过1s，退出刹车程序
																				if(enter_count>380){ //高度来自气压数据, 除了要1.2s的刹车程序，还需要再等1s，使得滞后的测量高度与实际高度逼近
																							enter_1s=0;
																							enter_count=0;
																							//pidHeightHigh.desired = Control_high; //。不再重设期望高度。
																							//thr_hight_modify=0;//刹车结束，清除临时油门; 在1.21s时已经清除
																					
																				}
																	
																	
																	
																	
																}
																
																
																
																
																
															 //一键定高起飞结束，光流pid参数恢复到默认状态。
#if ( FLY_TYPE==3 ) //无刷 光流pid默认参数，手动定高起飞或一键定高起飞悬停中
																			//退出模式3，或者方向摇杆动作结束，则调入光流默认pid参数
															 if(flow_pid_param_type!=1) flow_pid_param_default(); //无刷电机光流pid，定点悬停时，pid参数较小

#endif
																
																
																
																
													
													}
					
													
														
						}
				
						
						
						//在模式2下，自动修正平衡油门
						
						modify_thr_adjust=0;//修改平衡补偿标志，复位；
						//油门构成（1.动态，2.基础，3.平衡，4.临时增减）
						
						//每10ms修正一次基础油门,,,这个思路也可以用在自动漂移修正上，
						//当姿态角为0，预示着我们希望这个方向加速度为零,当加速度不为0，则修改漂移值（姿态角修正）
						if(hight_mode==2 && hight_modify==0){  //模式2（基础油门自动优化的定高）且未按高度调整按键
							
									
									modify_thr_adjust=1;//当我们在修改 平衡油门thr_adjust时，应该对 内环速度pid输出限幅，防止最终油门过大
									//比如，我们把手伸到无人机下面，在模式1时，平衡油门是不修改的。此时飞机仅凭内环pid输出，已经能够快速上升。
							    //而在 模式2 时，除了内环速度pid输出大幅增加外，平衡油门也在大幅增加，导致油门突变混乱。
									//所以我们在修改平衡时，应该对 内环速度pid输出限幅。这个限幅不仅仅在这里，而是在最终输出给电机的油门那边pid刚计算后
							
							
									//基础油门pid参数
									pidHeightThr.kp =1.0f; //p决定了修正力度
									pidHeightThr.ki =0.0f;
									pidHeightThr.kd =0.1f; //d防止过冲震荡
				
									
									pidHeightThr.desired = pidHeightRate.out; //上次的！内环速度的输出(即加速度)
							
									//当pidHeightRate.out突然大幅增加时，
							
							
									//当手伸到飞机下面，导致测量高度突变的情况，其实，只靠pidHeightRate.out自身对油门的影响已经足够了
									//我们这里调整平衡油门，主要解决的是没有遇到突发情况时，防止因为平衡基础油门不足，导致的飞机缓缓上升或者缓缓下降
									//手伸到飞机下面这种情况，我们飞机实际加速度，肯定是追不上pidHeightRate.out，也不用追上
									//所以我们这里需要对 我们的输入（期望加速度）限幅处理，否则电机功率突然拉满，有时飞机反而会失去动力下坠
							

									//pidHeightThr.measured = acc_error;//高度方向的运动加速度 （去除了重力加速度）10cm/s2的加速的，达到10mm/s2就认为达到了
									pidHeightThr.measured = acc_error*0.1f;//高度方向的运动加速度 （去除了重力加速度）
									
									
									//=LIMIT(pidHeightThr.desired, -50.0f, 50.0f); //每10ms调用一次，1s最多改变100次
									//=LIMIT(pidHeightThr.measured, -50.0f, 50.0f); //每10ms调用一次，1s最多改变100次
									//不能在这里限幅，而是在内环pid刚计算完成时
									
									
									//当都达到最大限幅时，pidHeightThr.out为零，平衡油门维持不变，但实际最终油门是在增加的，因为pidHeightRate.out很大
							
									//当单位不一致时，会导致，实际加速度已经达到了，pid却不知道，或者实际没达到，pid却以为达到了
									//比如，我们期望加速度 10cm/s2, 测量加速度10mm/s2, 导致加速度才1/10，就以为达到了期望加速度
									//如果测量加速度用分米单位，那么要等到10dm/s2，才以为加速度达到了。
									//所以，期望单位与测量反馈单位最好要一致。不然即使效果达到了，也是负负得正的效果。
							
									//外环单位一般不会出错，因为出错了马上就直到了，比如期望运动10cm，实际运动了10mm，我们马上就会发现并纠正
									//但内环单位弄错了，我们不一定知道，比如期望速度10cm/s，实际速度到达10mm/s就认为已经到了速度，
									//最后导致，要很长时间才能到达目标位置。我们就会怀疑是不是pid的p参数小了，于是调大上级p参数，结果果然快了很多
									//原因是，本来移动10cm，计算出来的速度是20cm/s，p参数调大5倍，计算出来的速度100cm/s,等实际速度到达100mm/s
									//认为是达到预计速度了，实际速度10cm/s好像也没有什么大问题，但实际上是错上加错，达到了负负得正的效果。
							
									//高度单位和角度单位是统一的，都是厘米
									//速度单位：mcu和光流使用厘米，陀螺仪使用毫米
									//加速度单位，mcu使用厘米，陀螺仪使用毫米
							
									pidUpdate(&pidHeightThr,dt); //再调用基础油门pid
									
									//这个输出是由加速度没达到期望值导致的。每秒调整量
									//如果加速度相差+/-50cm/s2,那么当p=1时，out结果大约 -50~50
									//实际值超过100甚至更多是会出现的
									pidHeightThr.out = LIMIT(pidHeightThr.out,-200.0f,200.0f); //每10ms调用一次，1s最多改变100次
									
									

									//速度不准，但加速度和高度都是准的，一个来自mpu，一个来自光流和激光，但高度带有滞后性，会导致来回震荡

									
									//调整的目的是让测量加速度能跟得上 期望加速度


									//往上调，现在位置太靠下，并且高度内环输出加速度大于实际加速度，自动往上修正平衡油门
									//if(thr_adjust_count_up>0){
														//thr_adjust_count_up--; //有上面的判断程序自动清空


									if(pidHeightRate.out>20){ //上次希望向上，内环输出限幅+/-150，出现很大的值，说明平衡油门不够，或者突发情况

										
												//同时要判断外环，当前高度不能太高
												//if(pidHeightHigh.measured < pidHeightHigh.desired){
											 

										
														//基础油门pid的输出，来自动修正基础油门
														if( ( pidHeightThr.out>1.0f && thr_adjust <200.0f ) ){//+/-200
																			
															
																			//如果当前高度比期望高度+10cm还要高，不允许上调
																			//解决测量高度突然变高，飞机不会自动下落
																			//if( !((float)Control_high > (pidHeightHigh.desired  + 10.0f)) ){
															
																						//调整好系数，每次最多修正1.0，每10ms调整量，每秒200油门
																						//thr_adjust += 0.005f + pidHeightThr.out*0.01f; 
																						thr_adjust += 0.000f + pidHeightThr.out*0.01f; //如果pid输出为零，说明基础油门正好，不需要调整平衡油门
																			//}
																				
																				
														}
												//}
														
														
														
									}

									
									if(pidHeightRate.out<-20){ //上次希望向下，内环输出限幅+/-150，出现很大的负值，说明平衡油门太大，或者突发情况
										
										
												//同时要判断外环，当前高度不能太低
												//if(pidHeightHigh.measured > pidHeightHigh.desired){
											
										
														//基础油门pid的输出，来自动修正基础油门
														if( ( pidHeightThr.out<-1.0f && thr_adjust >-200.0f ) ){//+/-200
																			
																			//如果当前高度比期望高度-10cm还要低，不允许下调
																			//解决测量高度突然变矮，飞机不会自动上跳
																			//if( !(pidHeightHigh.desired > ((float)Control_high + 10.0f)) ){
																								//调整好系数，每次最多修正0.5，每10ms调整量，每秒50油门
																								//thr_adjust += -0.005f + pidHeightThr.out*0.01f; 
																								thr_adjust += -0.000f + pidHeightThr.out*0.01f; //如果pid输出为零，说明基础油门正好，不需要调整平衡油门
																			//}
															

															
															
														}
												//}
														
														
														
														
									}
														
														
														

									//}




						}
						
						
						
						
						
						
						
						
						
						
						
						
						
						
						//一键定高起飞被按下，一键定高起飞中
						if(hight_mode==3 && Aux_Rc.fly_key2){
							
							
							
									//基础油门pid参数
									pidHeightThr.kp =1.0f; //p决定了修正力度
									pidHeightThr.ki =0.0f;
									pidHeightThr.kd =0.1f; //d防止过冲震荡
				
									
									pidHeightThr.desired = pidHeightRate.out + thr_hight_modify; //上次的！内环速度的输出(即加速度)
									pidHeightThr.measured = acc_error*0.1f;//高度方向的运动加速度 （去除了重力加速度）
							
							
									pidUpdate(&pidHeightThr,dt); //再调用基础油门pid
									pidHeightThr.out = LIMIT(pidHeightThr.out,-200.0f,200.0f); //每10ms调用一次，1s最多改变100次
							

									if(pidHeightRate.out>20){ //上次希望向上，内环输出限幅+/-150，出现很大的值，说明平衡油门不够，或者突发情况


														//基础油门pid的输出，来自动修正基础油门
														if( ( pidHeightThr.out>1.0f && thr_adjust <200.0f ) ){//+/-200
																			
															
																			//如果当前高度比期望高度+10cm还要高，不允许上调
																			//解决测量高度突然变高，飞机不会自动下落
																			//if( !((float)Control_high > (pidHeightHigh.desired  + 10.0f)) ){
															
																						//调整好系数，每次最多修正1.0，每10ms调整量，每秒200油门
																						//thr_adjust += 0.005f + pidHeightThr.out*0.01f; 
																						thr_adjust += 0.000f + pidHeightThr.out*0.01f; //如果pid输出为零，说明基础油门正好，不需要调整平衡油门
																			//}
																				
																				
														}
									}

									
									if(pidHeightRate.out<-20){ //上次希望向下，内环输出限幅+/-150，出现很大的负值，说明平衡油门太大，或者突发情况
										
														//基础油门pid的输出，来自动修正基础油门
														if( ( pidHeightThr.out<-1.0f && thr_adjust >-200.0f ) ){//+/-200
																			
																			//如果当前高度比期望高度-10cm还要低，不允许下调
																			//解决测量高度突然变矮，飞机不会自动上跳
																			//if( !(pidHeightHigh.desired > ((float)Control_high + 10.0f)) ){
																								//调整好系数，每次最多修正0.5，每10ms调整量，每秒50油门
																								//thr_adjust += -0.005f + pidHeightThr.out*0.01f; 
																								thr_adjust += -0.000f + pidHeightThr.out*0.01f; //如果pid输出为零，说明基础油门正好，不需要调整平衡油门
																			//}
															

															
															
														}
														
									}
													
							
							
							
						}
						
						
						
						
						
						
						
						
						
						
						
						
						
						
						
						
						
						
						
						
						//切换到模式1，或者拨动了油门
						//拨动了油门，即油门累计变化超过阈值后，又过了0.2s，重设期望高度，并切换到模式1
						if(thr_isChange!=0){
							
													hight_mode=1; //定高飞行时，动了油门,并且又过了0.2s，代表模式1
													thr_isChange=0; //标志复位
							
													pidHeightHigh.desired = Control_high;//重设期望高度
							
						}

						
						
						//在模式1下，当拨动油门时，要让平衡油门补偿值，逐渐回零
						//引入 thr_base_add 基础油门补偿，thr_adjust全都转移到  thr_base_add
						if(hight_mode==1 && thr_raw_adds!=0){

													if(thr_adjust!=0){
																		//平衡油门调整，与基础油门补偿，全都转移到基础油门补偿。
																		thr_base_add = thr_base_add + thr_adjust; //thr_adjust转移到thr_base_add
																		thr_adjust=0;
													}
							
							
													//如果用户希望上升，摇杆正在往上拨
													if(thr_raw_adds>0){//向上修正（此时平衡油门为负），平衡油门范围+/-200
																		//要避免修正值太靠下，导致无法上升
																		//if(thr_adjust<0.0f) thr_adjust+=0.05; //2ms一次，摇杆值10ms变化一次，一次变化累加了5次
																		//if(thr_adjust <= -thr_raw_add1*0.2 && thr_raw_add1> 0.0f) thr_adjust += thr_raw_add1*0.2;//确保向0靠拢
																		//这里允许摇杆漂移值，触发回零
													
																		if(thr_raw_adds<25) thr_raw_adds=25; // >10就能进来，每次至少修正25*0.3=7.5, 总油门+17.5
														
																		/*
																		if(thr_adjust <= -thr_raw_adds*0.3){//防止修正过头
																							thr_adjust += thr_raw_adds*0.3; //向上修正，油门往上200，平衡油门往上60，防止平衡油门太靠下，导致总油门无法往上调整
																		}
																		else if(thr_adjust<0){//确保向0靠拢
																							thr_adjust=0; //直接向上回零
																		}*/
																		
																		if( thr_base_add <= -thr_raw_adds*0.3 ){//防止修正过头
																							thr_base_add += thr_raw_adds*0.3; //向上修正，油门往上200，平衡油门往上60，防止平衡油门太靠下，导致总油门无法往上调整
																		}
																		else if( thr_base_add <0 ){//确保向0靠拢
																							thr_base_add=0; //直接向上回零
																		}
													}
													
													
													
													
													//如果用户希望下降，摇杆正在往下拨
													else if(thr_raw_adds<0){//向下修正（此时平衡油门为正），平衡油门范围+/-200
																		//要避免修正值太靠上，导致无法下降
																		//if(thr_adjust>0.0f) thr_adjust-=0.05; //2ms一次，摇杆值10ms变化一次，一次变化累加了5次
																		//if(thr_adjust >= -thr_raw_add1*0.2 && thr_raw_add1 < 0.0f) thr_adjust += thr_raw_add1*0.2;//确保向0靠拢
																		//这里允许摇杆漂移值，触发回零

																		if(thr_raw_adds>-25) thr_raw_adds=-25; // >10就能进来，每次至少修正-25*0.3=-9，总油门-17.5
														
																		/*
																		if(thr_adjust >= -thr_raw_adds*0.3){//防止修正过头
																							thr_adjust += thr_raw_adds*0.3; //向下修正
																		}
																		else if(thr_adjust>0){//确保向0靠拢
																							thr_adjust=0; //直接向下回零
																		}
																		*/
																		
																		if( thr_base_add >= -thr_raw_adds*0.3 ){//防止修正过头
																							thr_base_add += thr_raw_adds*0.3; //向下修正
																		}
																		else if( thr_base_add > 0 ){//确保向0靠拢
																							thr_base_add = 0; //直接向下回零
																		}
													}
													
													

													thr_raw_adds=0;//本次油门增加值清空
						
						}
						
						

						
						//每10ms维持一下现有高度
						//维持高度的能力不用太强
						//当同时操作油门摇杆时，无法维持高度
						//飞机会上下飞行，0.2s后，更新目标高度。
						//当油门摇杆没有变化，目标高度不会变更。
						pidUpdate(&pidHeightHigh,dt);    //调用PID处理函数来处理外环	俯仰角PID	
						
						//速度调整限制
						//=LIMIT(pidHeightHigh.out,-30,60); //外环pid输出：速度限幅 100cm/s
						
						//当一键起飞，期望速度，上面已直接给出（逐渐降速），所以要屏蔽外环计算
						if(!(hight_mode==3 && Aux_Rc.fly_key2)){
										pidHeightRate.desired = pidHeightHigh.out;  //高度环输出高度速度设定值
						}
								
						
						pidUpdate(&pidHeightRate,dt); //再调用高度速度内环
						
						
						
						

						
						
						if(modify_thr_adjust==1){//总油门 = 摇杆基础油门 + 内环pid输出加速度 + 平衡油门补偿
										//在自动调整平衡油门时，这里可能有双重 油门提升，导致混乱，有时太大反而导致失效，需要限幅
										pidHeightRate.out=LIMIT(pidHeightRate.out,-120.0f,120.0f); //内环pid输出限幅，即内环pid输出的油门限幅
						}
						
						else if(hight_mode==3 && Aux_Rc.fly_key2){
										//在自动调整平衡油门时，这里可能有双重 油门提升，导致混乱，有时太大反而导致失效，需要限幅
										pidHeightRate.out=LIMIT(pidHeightRate.out,-200.0f,200.0f); //内环pid输出限幅，即内环pid输出的油门限幅
						}
						
						else{
										//其他情形，动态油门(加速度) +/-400，总油门=摇杆基础油门+内环pid输出加速度，实际不会到400，最多200~300
										pidHeightRate.out=LIMIT(pidHeightRate.out,-400.0f,400.0f); //内环pid输出限幅，即内环pid输出的油门限幅
						
						}
						
						
						
						//油门调整限制
						//=LIMIT(pidHeightRate.out,-50,50); //内环pid输出：加速度限幅 100cm/s2，1秒内加速到100cm/s，这个输出同时也是油门更改值
						
						//加速度和油门成正比例关系（合适的平衡油门条件下）比例系数由内环pid的p参数来匹配。
						
						
						//没有调整油门时，飞机也缓缓下降，或者缓缓上升，
						//说明高度内环的输出与油门之间的系数有问题。
						//系数太小，定高能力偏弱，油门不动，也没法保持平衡，必须把油门调到一个最佳值，才能平衡。
						//系数太大，定高能力太强，很难通过油门来调整定高高度。
						
						//高度的速度内环pid计算，输出结果是油门值。
						//注意：最终油门值，在MotorControl()中给出，2ms一次。
						//这里，全是维持当前高度，算出的油门值。10ms一次
						
							 
						//pidHeightRate.out += Remote.thr -1000;//加入悬停时的油门
							 

						
						break;	
						
				case EXIT_255: //退出定高
					pidRest(&pPidObject[6],1);	//清除当前的定高输出值
					status = WAITING_1;//回到等待进入定高
					break;
				
				default:
					status = WAITING_1;
					break;	
		}	
				
}



//4ms一次，紧随Rc_Connect()，接收到遥控数据就开始进入模式选择。
//但是 遥控数据并不是 4ms发送一次。
void Mode_Controler(float dt)
{
	
		//新增，
		//如果没有收到新的控制数据，直接退出，防止一次数据，多次重复处理
		if(!Remote.new_data){
							//悬停模式 .定点定高，这里也要调用一下，确保调用周期为0.004s
							//如果收到数据，在数据解析后，还要再调用一次。
							if(ALL_flag.height_lock==1){  //悬停模式
										//取消这个函数,,移动到Mode_Controler()最后调用
										Flow_mode_two();       // 遥控-光流控制姿
							}
							
							return;
		} 

		
		Remote.new_data=0;
		

	
	
		const float roll_pitch_ratio = 0.04f;
	
//	if(ALL_flag.unlock == 1)  //判断解锁
		{
			
							//电机手动测试, 一旦遥控按过，AUX844_isH()永远位高电平，直到遥控重启
							if(Remote.test_motor==0 && AUX844_isH()) Remote.test_motor=1; //test_motor代表当前测试第几个马达，0代表非测试模式
							else if(Remote.test_motor!=0 && AUX844_isH()==0) Remote.test_motor=0; //重启遥控可以关闭测试模式
			
							//无头模式		
							Aux_Rc.headless=!AUX83_isH();//默认为无头模式
							//油门屏蔽
							Aux_Rc.thr_mask=AUX82_isH();
			
			
							//一键定高起飞
							Aux_Rc.fly_key2=AUX81_isH();//一键定高起飞
			
							//定高起飞流程：先set+key2，进入油门屏蔽（姿态模式）
							//长按 key2，一键起飞fly_key2=1，定高模式AUX2 =1000，thr_mask=0
							//松开key2，达到指定高度，维持高度。fly_key2=0
			
							
							//偏航角控制，不用区分是否是悬停模式（定高+定点）
							{
								
										//const float yaw_ratio =   0.5f;		//偏航角灵敏度度
										const float yaw_ratio =   0.4f;	//转动速度  0.4/4ms, 1秒100度
								
								
										//通过油门摇杆控制，这是原来的方式，不建议使用。实际操作时无法保证油门摇杆值不变
										//如果用户拨动的偏航角摇杆, 这里的yaw摇杆值，不包含偏航角漂移修正。 偏航修正直接作用于陀螺仪修正
										if(Remote.yaw>1820) //+320
										{
												Aux_Rc.yaw_stick -= yaw_ratio;	//符号控制偏航角方向
										}
										else if(Remote.yaw <1180) //-320
										{
												Aux_Rc.yaw_stick += yaw_ratio;	//符号控制偏航角方向
										}	

										
										//由于用户拨动摇杆导致的偏航角变化，这个值在解锁时要归零，Aux_Rc.yaw_stick
								
								
								
										//偏航角度，用户在遥控端按了旋转按钮，使用目标偏航角度
										//遥控每按一次，目标yaw角度会增加或减少45度
										int8_t n_45=AUX73();
								
								
										//慢慢到达目标角度，太快了会乱动。
										if( (float)n_45*45 + Aux_Rc.yaw_stick - pidYaw.desired >= yaw_ratio ) {
													pidYaw.desired += yaw_ratio;

										}
										else if( (float)n_45*45 + Aux_Rc.yaw_stick - pidYaw.desired <= -yaw_ratio ){
												pidYaw.desired -= yaw_ratio;
											
										}
										else{
												pidYaw.desired = (float)n_45*45 + Aux_Rc.yaw_stick;  //最终达到目标角度
											
											
										}
								


	
							}		
			
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
							
							
							
							
							//遥控默认开机aux2为1000，默认为 悬停模式； 当aux2 =2000表示姿态模式 ;;;
							//油门屏蔽时也是姿态模式, 长按KEY2才会进入定高模式。
							//if(Remote.AUX2 < 1700)   //如果小于1700 则进入定点模式显示
							//光流模块1s内无数据时，会自动改为姿态模式
							//set_flag：       空7 | 气压定高6 | 光流定点5 | 视觉定位4     |     切换机头3 | 无头模式2 | 抛飞1 |  激光定高0 
							//油门屏蔽后，长按k2为定高起飞，如果有光流就是默认激光定高，没有光流模块就是默认气压定高。
							//油门屏蔽后，按向上键，表示姿态起飞。
							if(Remote.AUX2 < 1700 && Flow_Err == 0)   //如果小于1700并且光流模块无异常时，才会进入悬停模式
							
							{
											Command.FlightMode = HEIGHT;  //用于传输给遥控器或者上位机
								
											ALL_flag.height_lock = 1;//悬停模式标志符：光流模式+定高
											ALL_flag.height_lock_qiya = 0;//非定点模式下的气压定高，只是定高，没有光流
											
											//这个函数要放到最后面,要等摇杆值处理之后再调用
											//Flow_mode_two();       // 遥控-光流控制姿
								
											// 0b 0010 0001
											set_flag=0x21;         // 用于OLED定高定点模式显示( 在飞控设置界面 )，激光定高（超过4米启动气压）
											
											if(Aux_Rc.headless){
													set_flag = set_flag | 0x04; //无头模式
											}
								
											if(AUX833_isH()){ //激光定高时，主动选择了气压优先，默认是激光优先AUX833_isH()==0
													set_flag = set_flag | 0x40; //气压定高开启
													set_flag = set_flag & ~0x01; //关闭激光定高
											}

											
											
											//如果是刚从姿态模式切换过来，要不要复位 光流？？？
											//Aux_Rc.position=1;//悬停模式不更新 Aux_Rc

								
								
								
							}
							
							//set_flag：       空7 | 气压定高6 | 光流定点5 | 视觉定位4     |     切换机头3 | 无头模式2 | 抛飞1 |  激光定高0 
							//没有装光流模块，则aux2意味着气压定高模式 height_lock_qiya，此时没有激光定点功能。单纯的定高功能
							//上面的定高模式 height_lock，表示悬停模式（激光定高+光流定点）注：也可以选择气压高度
							
							else if(Remote.AUX2 < 1700 && SPL_Err == 0)   //如果小于1700并且无光流模块，气压模块正常，才会进入气压定高模式
							{
											Command.FlightMode = HEIGHT;  //用于传输给遥控器或者上位机
								
											ALL_flag.height_lock = 0;//悬停模式标志符：光流模式+定高
											ALL_flag.height_lock_qiya = 1;//非定点模式下的气压定高，只是定高，没有光流

								
											// 0b 0100 0000
											set_flag=0x40;         // 当用户选择定高模式（aux2=1000），并且光流失效时，自动开启气压定高
											
											if(Aux_Rc.headless){
													set_flag = set_flag | 0x04; //无头模式
											}
								
											//当光流模块未安装，同时安装了气压模块，不管有无选择气压优先，都会启动气压定高。
											//if(AUX833_isH()){
													//set_flag = set_flag | 0x40; //气压定高开启
													//set_flag = set_flag & ~0x01; //关闭激光定高
											//}
								
								
								
							}
							
							//光流和气压都不存在，或者用户 key2按了3下，主动选择了姿态模式。
							else //姿态模式，aux2为2000，此时height_lock和height_lock_qiya都为0
							{
								
											Command.FlightMode = NORMOL;	//用于传输给遥控器或者上位机
											
											ALL_flag.height_lock = 0; //悬停模式标志符：光流模式+定高
											ALL_flag.height_lock_qiya = 0; //非定点模式下的气压定高，只是定高，没有光流
								
											// 0b 0000 0000
											set_flag=0x00;        //OLED姿态模式显示 ( 在飞控设置界面 )
								
											if(Aux_Rc.headless){ 
													set_flag = set_flag | 0x04; //无头模式
											}
											
											//当用户并未开启定高模式(aux2=2000)，无论用户有无选择气压优先，气压模块都不会开启
											//if(AUX833_isH()){
													//set_flag = set_flag | 0x40; //气压定高开启
											//}
								

											//Aux_Rc.position=0;//姿态模式下，才更新 Aux_Rc
								
							}
								
							
							
							
							//记录纯油门摇杆值，用于定高时，通过摇杆调整高度判断
							Aux_Rc.thr_raw = Remote.thr;
							Aux_Rc.roll_raw = Remote.roll;
							Aux_Rc.pitch_raw = Remote.pitch;	
							
							
							//3.4 零飘修正，需要在力度与无头转换之后（永远是修正飞机自身xy，不应坐标转换，与摇杆中值误差不同）
							Aux_Rc.offse_roll 	= Remote.AUX1-1500; //要注意，悬停模式，这个不能注销。。。不然悬停模式读不到零漂修正值
							Aux_Rc.offse_pitch 	= Remote.AUX3-1500;
							//offse_yaw在遥控设置方法：SET+P 或者 SET+M
							Aux_Rc.offse_yaw 		= Remote.AUX4>>8; 	//高字节为yaw漂移修正, 直接作用于 陀螺仪MPU6050.gyroZ_offset1（在imu.c和mpu6050.c用到）
							
							
							
							//悬停模式，使用的摇杆原始值以及各种按键值，摇杆值只需要做一下无头模式的坐标转换
							//不含用户零漂修正值
							if(ALL_flag.height_lock==1){ //悬停模式
								
										//提取摇杆值
										Aux_Rc.thr = Remote.thr;
										Aux_Rc.yaw = Remote.yaw;
										Aux_Rc.roll= Remote.roll;
										Aux_Rc.pitch = Remote.pitch;
					
			


										//1. 取出各种参数
										//1.1 取出力度
										Aux_Rc.lidu=(8 - AUX71());  //( 8 - AUX71);  //0~7变成8~1, 同时也是显示值
								
										//1.2 取出Z增量方向，用户计算油门临时增加值
										Aux_Rc.move_fx_z=0;
										if(AUX88_isH()){
												Aux_Rc.move_fx_z=-1;
										}
										if(AUX87_isH()){
												Aux_Rc.move_fx_z=1;
										}
										
										//1.3 取出油门比例，用于计算基础油门
										Aux_Rc.thr_ratio=0;
										Aux_Rc.thr_ratio =  1 + 0.05*(AUX72()-2);//  1 + 0.05*(AUX72-2); (0~15)分别为(90%~165%) 油门比例显示值 AUX72()+1,,1~16
										
															

										//2.更新油门摇杆值，修改好后，control.c里面电机控制会用到油门摇杆值
										//2.1 更新油门基础值,
										Aux_Rc.thr = (Aux_Rc.thr-1000) * Aux_Rc.thr_ratio +1000; //与姿态模式保持一致，防止切换时造成波动
										
										
										
										/////////////////////////////////////////////////////////////////////////////////////////////
										/*///////////悬停模式时，油门值没有临时增加量，不是靠临时增加值上升或下降////////////////////////
										/////////////////////////////////////////////////////////////////////////////////////////////
										
										//2.2 计算油门临时增加值, 当力度为8时，移动量为当前值的50%
										//Aux_Rc.move_thr = ( (Aux_Rc.thr-1000) * Aux_Rc.lidu * Aux_Rc.move_fx_z )/8/2; //油门临时移动量与当前油门有关
										Aux_Rc.move_thr = ( (Aux_Rc.thr-1000) * 3 * Aux_Rc.move_fx_z )/8/2; //油门临时移动量与当前油门有关

										//修改临时增加值，防止油门最终出现负数
										if((Aux_Rc.thr-1000) + Aux_Rc.move_thr <=0) Aux_Rc.move_thr = -(Aux_Rc.thr-1000);
										
										//2.3 最终修改油门值，如果收到的油门为1000，则不会做任何更改
										Aux_Rc.thr = Aux_Rc.thr + Aux_Rc.move_thr;
										*/
		                /////////////////////////////////////////////////////////////////////////////////////////////
															
															
															
										//3.更新xy姿态角摇杆值	
										
										//3.1 已取消，摇杆值中值误差修正：现在的摇杆值 - 摇杆在解锁时的中值误差
										//这里的_offset修正，是由于摇杆自身每次回中误差导致的差异。这个差异在解锁无人机时会记录。
										//Remote.pitch -= pitch_offset;
										//Remote.roll -= roll_offset;
										

						
										//3.2.姿态角摇杆值力度修正
										Aux_Rc.roll = (Aux_Rc.roll-1500) * Aux_Rc.lidu/8 + 1500; // 1/8
										Aux_Rc.pitch = (Aux_Rc.pitch-1500) * Aux_Rc.lidu/8 + 1500; // 1/8


										//3.3.坐标转换，如果是无头模式		
										//Angle.yaw为当前飞机的偏航旋转角度（相对于地面坐标解锁时刻）
										if(Aux_Rc.headless){
										//姿态模式与悬停模式都需要无头模式
											
											
															int16_t x1 = (Aux_Rc.roll-1500)*cos(Angle.yaw*PI/180) + (Aux_Rc.pitch-1500)*sin(Angle.yaw*PI/180);
															int16_t y1 =-(Aux_Rc.roll-1500)*sin(Angle.yaw*PI/180) + (Aux_Rc.pitch-1500)*cos(Angle.yaw*PI/180);
															Aux_Rc.roll = x1+1500;
															Aux_Rc.pitch= y1+1500;
											
											
										}
															
										
										
										
										/////////////////////这里不能注销，非常重要///////////////////////////
										//3.4 零飘修正，需要在力度与无头转换之后（永远是修正飞机自身xy，不应坐标转换，与摇杆中值误差不同）
										//Aux_Rc.offse_roll = Remote.AUX1-1500;		//要注意，悬停模式，这个不能注销。。。不然悬停模式读不到零漂修正值
										//Aux_Rc.offse_pitch = Remote.AUX3-1500;  //转移到共用区域
										
										
										//从这里开始，正常的悬停模式不需要，但当高度<20cm时，光流不起作用，此时就需要零漂修正！！
										
										/*	 悬停模式也要受漂移修正影响，但不要用漂移修正更改摇杆值。要在光流内环计算结果出来更新期望姿态角之后，再来修正。
										Aux_Rc.roll = Aux_Rc.roll + Aux_Rc.offse_roll; 
										Aux_Rc.pitch = Aux_Rc.pitch + Aux_Rc.offse_pitch;
										*/

										//3.5 取消PID里面的offset修正摇杆自身中值误差 ，现在没有在解锁时测量中位值，但增加了摇杆自身校准。
										//pidRoll.offset  = 0;   
										//pidPitch.offset = 0; 


                    ///////////////////////////////////////////////////////////////////////////////
										// 悬停模式时，只有大幅拨动(>+/-150)才会屏蔽光流，导致直接用摇杆值转换姿态角。 /////
										// 漂移修正值在摇杆中位（光流有效）时，不应转为姿态角                         /////
										// 。。。但漂移修正值在拨动结束后的1s内很关键。因为那时继续屏蔽光流            /////
										////////////////////////////////////////////////////////////////////////////////
										/*  
										//3.6 更新xy姿态期望角度，，，姿态摇杆值，转为姿态角。
										//摇杆中值误差已经在3.1里面提前修正了，漂移修正已经在3.4里面提前修正了
										
										pidRoll.desired  = -(Aux_Rc.roll-1500) *roll_pitch_ratio;  //将遥杆值作为飞行角度的期望值
										pidPitch.desired = -(Aux_Rc.pitch-1500)*roll_pitch_ratio;  
										*/
				            ///////////////////////////////////////////////////////////////////////////////
									
										//更新摇杆值
										Remote.thr = Aux_Rc.thr;
										Remote.yaw = Aux_Rc.yaw;
										Remote.roll = Aux_Rc.roll;
										Remote.pitch = Aux_Rc.pitch;
								
							
							}
							
							
							//姿态模式(非定点悬停模式)，但有可能是非悬停模式下的气压定高 ALL_flag.height_lock_qiya
							else{//姿态模式，计算期望姿态角
							

								
								
								
								
								
								
								
								
								
								
								
								
								
								
								
								
								
								
								
								
								
								
								
								
								
								
								
										//提取摇杆值
										Aux_Rc.thr = Remote.thr;
										Aux_Rc.yaw = Remote.yaw;
										Aux_Rc.roll= Remote.roll;
										Aux_Rc.pitch = Remote.pitch;
					
			


										//1. 取出各种参数
										//1.1 取出力度
										Aux_Rc.lidu=(8 - AUX71());  //( 8 - AUX71);  //0~7变成8~1, 同时也是显示值
								
										//1.2 取出Z增量方向，用户计算油门临时增加值
										Aux_Rc.move_fx_z=0;
										if(AUX88_isH()){
												Aux_Rc.move_fx_z=-1;
										}
										if(AUX87_isH()){
												Aux_Rc.move_fx_z=1;
										}
										
										//1.3 取出油门比例，用于计算基础油门
										Aux_Rc.thr_ratio=0;
										Aux_Rc.thr_ratio =  1 + 0.05*(AUX72()-2);//  1 + 0.05*(AUX72-2); (0~15)分别为(90%~165%) 油门比例显示值 AUX72()+1,,1~16
										
															
											
										//2.更新油门摇杆值，修改好后，control.c里面电机控制会用到油门摇杆值
										//2.1 更新油门基础值,
										Aux_Rc.thr = (Aux_Rc.thr-1000) * Aux_Rc.thr_ratio +1000;
										
										
										//当气压定高时，油门值也不用包含 临时增加值，而是有一系列油门构成
										
										if(!ALL_flag.height_lock_qiya){ //新增，非悬停模式，并且非气压定高，油门变量就直接包含了临时增加值
										
												//2.2 计算油门临时增加值, 当力度为8时，移动量为当前值的50%
												//Aux_Rc.move_thr = ( (Aux_Rc.thr-1000) * Aux_Rc.lidu * Aux_Rc.move_fx_z )/8/2; //油门临时移动量与当前油门有关
												Aux_Rc.move_thr = ( (Aux_Rc.thr-1000) * 3 * Aux_Rc.move_fx_z )/8/2; //油门临时移动量与当前油门有关

												//修改临时增加值，防止油门最终出现负数
												if((Aux_Rc.thr-1000) + Aux_Rc.move_thr <=0) Aux_Rc.move_thr = -(Aux_Rc.thr-1000);
												
												//2.3 最终修改油门值，如果收到的油门为1000，则不会做任何更改
												Aux_Rc.thr = Aux_Rc.thr + Aux_Rc.move_thr;
										}
		
															
										//3.更新xy姿态角摇杆值	
										
										//3.1 已废弃，摇杆值中值误差修正：现在的摇杆值 - 摇杆在解锁时的中值误差
										//这里的_offset修正，是由于摇杆自身每次回中误差导致的差异。这个差异在解锁无人机时会记录。
										//Remote.pitch -= pitch_offset;
										//Remote.roll -= roll_offset;
										

						
										//3.2.姿态角摇杆值力度修正
										Aux_Rc.roll = (Aux_Rc.roll-1500) * Aux_Rc.lidu/8 + 1500; // 1/8
										Aux_Rc.pitch = (Aux_Rc.pitch-1500) * Aux_Rc.lidu/8 + 1500; // 1/8


										//3.3.坐标转换，如果是无头模式		
										//Angle.yaw为当前飞机的偏航旋转角度（相对于地面坐标解锁时刻）
										if(Aux_Rc.headless){
										//姿态模式与悬停模式都需要无头模式
											
															int16_t x1 = (Aux_Rc.roll-1500)*cos(Angle.yaw*PI/180) + (Aux_Rc.pitch-1500)*sin(Angle.yaw*PI/180);
															int16_t y1 =-(Aux_Rc.roll-1500)*sin(Angle.yaw*PI/180) + (Aux_Rc.pitch-1500)*cos(Angle.yaw*PI/180);
															Aux_Rc.roll = x1+1500;
															Aux_Rc.pitch= y1+1500;
											
										}
										
	

										//3.4 零飘修正，需要在力度与无头转换之后（永远是修正飞机自身xy，不应坐标转换，与摇杆中值误差不同）
										//Aux_Rc.offse_roll = Remote.AUX1-1500; 	//要注意，悬停模式，这个不能注销。。。不然悬停模式读不到零漂修正值
										//Aux_Rc.offse_pitch = Remote.AUX3-1500;	//转移到共用区域
										
										
										//从这里开始，正常的悬停模式不需要，但当高度<20cm时，光流不起作用，此时就需要零漂修正！！
										//姿态模式时，零漂修正直接影响 期望角；而在悬停模式时，需要先通过光流内环的输出值，计算出期望角之后，再用零漂修正这个期望姿态角
										
										Aux_Rc.roll = Aux_Rc.roll + Aux_Rc.offse_roll;  //漂移 静态修正(用户修正值，应该在电池充满时修正)
										Aux_Rc.pitch = Aux_Rc.pitch + Aux_Rc.offse_pitch;
										//有问题，45应该是1.8，但有时1.8，有时3.6

										//Aux_Rc.roll = Aux_Rc.roll + Aux_Rc.offset_roll_adjust;  //漂移 动态修正值，电池电压下降等原因导致的某个电机推力下降
										//Aux_Rc.pitch = Aux_Rc.pitch + Aux_Rc.offset_pitch_adjust; 
					

										//3.5 已废弃，取消PID里面的offset修正摇杆自身中值误差 ，现在没有在解锁时测量中位值，但增加了摇杆自身校准。
										//pidRoll.offset  = 0;   
										//pidPitch.offset = 0; 



										//姿态模式下，通过方向摇杆变化调整姿态角
										//3.6 更新xy姿态期望角度，，，姿态摇杆值，转为姿态角。
										//摇杆中值误差已经在3.1里面提前修正了，漂移修正已经在3.4里面提前修正了
										pidRoll.desired  = -(Aux_Rc.roll-1500) *roll_pitch_ratio;  //将遥杆值作为飞行角度的期望值,,,假设-330*0.04
										pidPitch.desired = -(Aux_Rc.pitch-1500)*roll_pitch_ratio;  



										
										//更新摇杆值
										Remote.thr = Aux_Rc.thr;
										Remote.yaw = Aux_Rc.yaw;
										Remote.roll = Aux_Rc.roll;
										Remote.pitch = Aux_Rc.pitch;
										
										
										
							}
							
							
							//悬停模式 .定点定高
							if( ALL_flag.height_lock==1 ){ 
										//悬停模式时，要通过这个来提取光流pid计算结果，到期望姿态角
										Flow_mode_two();       // 遥控-光流控制姿
							
							}

							
							
							
							
							
			
			
		}				
}




// .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. ..  1ms
//     .     .     .     .     .     .     .     .     .     .  4ms
//              .              .              .              .  10ms

//u32 altHold_Pos_Save = 0, Pos_breaktime = 0, Pos_break_save = 0;
u16 altHold_Pos_cnt =0; //刹车倒计时
u8  altHold_Pos_Save;		//方向摇杆正在拨动


///////////////////////// 遥控-光流控制姿态////////////////////////////////////////////////
//最终为了计算出 姿态角
//摇杆值  - > 移动速度期望 -> ..... ->移动速度pid输出 ->姿态角
//摇杆值  - > 姿态角
//只有悬停模式才会进来，Mode_Controler()调用，4ms一次，目的：提取光流pid的最终计算结果，给到期望姿态角。
void Flow_mode_two(void)  // 有Mode_Controler()调用，4ms一次，目的：提取光流pid的最终计算结果（10ms更新一次）
{
	
	////////////////////////////////////////////////////////////////////////////
	///////////这个函数的作用是：在悬停模式时，提取光流计算结果////////////////////
	////////////////////////////////////////////////////////////////////////////
	
	
		//此时的目标是让无人机姿态角到达 摇杆值+漂移修正值 
		//const float roll_pitch_ratio = 8.00f;	//
		//const float roll_pitch_ratio = 3.00f;	//
	
		//只有悬停模式才会进来，并且高度在范围内，并且光流数据正常。
		//提取光流计算结果，给期望姿态角
		//if((mini.ok == 1) && ((Control_high>20)   && (Control_high<400))  ) //高度范围判断改为指定激光高度，这样光流才可以配合气压优先模式。
		if( (mini.ok == 1)  &&  (mini.flow_High>20) && (mini.flow_High<400)  )//判断是否存在光流  高度高于20CM 光流才可以定点
		{//悬停模式 必须要有激光，但可以设定气压优先，用来测试气压定高效果

					
					//这里应该包含漂移修正，光流速度pid计算结果不含漂移修正，但是我们需要。
					//这里是不含漂移修正的，不过没有关系，误差累积大了，期望速度也会逐渐增加。
					//pidPitch.desired = LIMIT(Flow_SpeedPid_x.out*0.1,-15,15) ; //姿态外环期望值 (Remote.pitch-1500)*8*0.006
					//pidRoll.desired  = LIMIT(Flow_SpeedPid_y.out*0.1,-15,15) ; //姿态外环期望值  实际系数为0.048f，与姿态模式类似。
					//pidRoll.desired = -LIMIT(Flow_SpeedPid_y.out*0.1,-15,15) ; //姿态外环期望值  实际系数为0.048f，与姿态模式类似。
					//改成对最终的期望姿态角限幅，之前的光流输出姿态角有可能因为零漂原因，姿态角期望输出中心会偏向某一边，限幅后会导致有效值范围不足
					pidPitch.desired = Flow_SpeedPid_x.out*0.1; 
					pidRoll.desired = Flow_SpeedPid_y.out*0.1; 
			

			
					//   目前坐标系设置，只是通过无数实验测试出来实现了较好效果的一种方法，并不是唯一方法；
					//   如用户有自认为在理论上存在更为合理的方法可自行测试，如不理解但又无能力更改请保持现状即可。
					//
					//   光流坐标   x-     机体加速度坐标   x-	       机体姿态角 p-（顺时针为负值）   遥控姿态角 p+（为了符合用户习惯）
					//             /                      /                    /                             /
					//      y+ ___/___ y-          y- ___/___ y+        r+ ___/___ r-                 r- ___/___ r+   
					//           /|                     /|                   /|                            /|
					//          / |                    / |                  / |                           / |
					//        x+	|                  x+  |                p+  |                         p-  |
					//            z-                     z-                    
			


					//期望姿态角限幅，防止抖动; 
					//对于非常好的机器，可能这里设置大一些无所谓，但大多数机器，期望角度过大会抖动。在悬停自稳阶段设置限幅为+/-15度左右

#if ( FLY_TYPE==3 ) //无刷电机。 光流的速度pid输出限幅（期望姿态角限幅），避免抖动。
				

				
					if(altHold_Pos_cnt){ //正在刹车
								//Flow_SpeedPid_y.out = LIMIT(Flow_SpeedPid_y.out,-40,40);//速度PID输出，改写期望姿态角
								//Flow_SpeedPid_x.out = LIMIT(Flow_SpeedPid_x.out,-40,40);//速度PID输出，改写期望姿态角
								pidPitch.desired = LIMIT(pidPitch.desired, -20,20) ; //期望姿态角限幅
								pidRoll.desired  = LIMIT(pidRoll.desired,  -20,20) ; //期望姿态角限幅
						
					}
					if(altHold_Pos_Save){ //摇杆正在拨动
								//Flow_SpeedPid_y.out = LIMIT(Flow_SpeedPid_y.out,-60,60);//速度PID输出，改写期望姿态角
								//Flow_SpeedPid_x.out = LIMIT(Flow_SpeedPid_x.out,-60,60);//速度PID输出，改写期望姿态角
								pidPitch.desired = LIMIT(pidPitch.desired, -25,25) ; //期望姿态角限幅
								pidRoll.desired  = LIMIT(pidRoll.desired,  -25,25) ; //期望姿态角限幅
						
					}
					else{//平时
								//防止光流环境质量不足时，pid输出值过大，引起抖动；仅针对无刷电机(力大)。空心杯似乎没有这个问题
								//Flow_SpeedPid_y.out = LIMIT(Flow_SpeedPid_y.out,-15,15);//速度PID输出，改写期望姿态角
								//Flow_SpeedPid_x.out = LIMIT(Flow_SpeedPid_x.out,-15,15);//速度PID输出，改写期望姿态角
								//pidPitch.desired = LIMIT(pidPitch.desired, -15, 15) ; //期望姿态角限幅
								//pidRoll.desired  = LIMIT(pidRoll.desired,  -15, 15) ; //期望姿态角限幅
						
								//如果下面两句生效后就不抖了，说明不仅仅与光流pid有关，而且内环角速度PID参数也不适合，无刷电机强劲快速，p值，d值都可以适当减小
								//没有光流时，不抖动时因为平时姿态角期望值比较稳定，很少变化。
								//pidPitch.desired = Flow_SpeedPid_x.out*0.07; 
								//pidRoll.desired = Flow_SpeedPid_y.out*0.07; 	
									
								pidPitch.desired = LIMIT(pidPitch.desired, -15, 15) ; //期望姿态角限幅
								pidRoll.desired  = LIMIT(pidRoll.desired,  -15, 15) ; //期望姿态角限幅
						

						
					}
#else
					
					pidPitch.desired = LIMIT(pidPitch.desired, -15,15) ; //期望姿态角限幅
					pidRoll.desired  = LIMIT(pidRoll.desired,  -15,15) ; //期望姿态角限幅
					
					
#endif
				



					
					//注意这里通过光流速度pid的输出得到的姿态角期望值，还是要用零漂修正，不然零漂太大，会先飘出去再飞回来，上升呈现弧线。
					//如果没有零漂修正，正反方向的移动速度会不一致，比如 X+移动的快，X-移动的慢
					//没有漂移修正，会导致刚启动光流时，有点抖；在姿态模式或者<20cm时姿态角有漂移修正，现在也有，过渡会更顺畅。
					//注意：遥控器姿态角正负方向正好与机体姿态角正负相反。
					//     遥控器姿态角为正代表向前向右（符合使用体验）。而机体姿态角为正，代表逆时针，无人机向左向后
					pidPitch.desired += (-Aux_Rc.offse_pitch)*0.04f; //漂移 静态修正(用户修正值)
					pidRoll.desired += (-Aux_Rc.offse_roll)*0.04f;//已经转移到：Flow_mode_two()
					



			
		}
		
		//这里也需要漂移修正
		//进入这里是因为启用了光流，光流模块通讯也正常，只是数值可信度不高，数据被取消了
		//没有启用悬停模式，在姿态模式，是上面3.6直接给出姿态角期望值。那里也包含修正了
		
		//只有悬停模式才会进来，并且高度不在范围内，或者光流数据异常。
		//注意此时为悬停模式，有可能开机后从没进入过姿态模式，没有姿态模式的数据。Aux_Rc.offse_pitch不能到了姿态模式才获取！！！！！！！！
	  //注意，虽然这里的控制方式和姿态模式一样，但这里不能省略，有时开机后直接手动定高起飞，开机后从没有经历过油门屏蔽和姿态模式，所以默认姿态角期望值仍为0，不含修正值。
		else//光流失效，或者高度（0.2~4米）不在范围内，则使用遥控器直接控制姿态角期望值（要包含零漂修正）。
		{

					//在悬停模式，这个值已经包含力度修正，但不包含零漂修正
					//以下非常重要，不能注销，悬停模式在光流失效时，Mode_Controler()里面从未计算过姿态角
					pidPitch.desired = -(Remote.pitch-1500)*0.04f; //(Remote.pitch-1500)*0.04
					pidRoll.desired  = -(Remote.roll-1500)*0.04f;  //已经转移到：Flow_mode_two()
					
					pidPitch.desired += (-Aux_Rc.offse_pitch)*0.04f; //零漂修正
					pidRoll.desired += (-Aux_Rc.offse_roll)*0.04f;
			
		}
}



/**************************************************************
 * //位置定点控制器  光流pid计算 10ms一次, 姿态模式下也在计算光流，只是不影响姿态角
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/


//这里只是光流计算结果（姿态模式也会计算） 暂时不影响 pidPitch.desired
//等到 进入悬停模式，会调用Flow_mode_two(),才会把光流的计算结果给到 期望姿态角。
//光流一直都在计算，不管当前是悬停模式，还是姿态模式。
void Flow_Pos_Controler(float dt)  //位置定点控制器  光流，，10ms一次，要在光流融合数据计算后
{
		//const uint16_t DEADBAND=150;	
		const uint16_t DEADBAND=50;	//最小1级力度时，最大+/-60   不含修正值
		static u8 Pos_breaktime_max=100;
	
		static float  Flow_SpeedPid_out_old_x=0; //实现输出滤波
		static float  Flow_SpeedPid_out_old_y=0;
	
	  static uint8_t move_cnt=0; //记录移动时间
	
	
		//光流只在20~400cm生效，小于20cm或者大于4米，依旧靠 零漂修正 直接影响期望姿态角。
		//注意：姿态模式也会进来
		//if((mini.ok == 1) && ((Control_high>20)  && (Control_high<400))  ) //高度范围判断改为指定激光高度，这样光流才可以配合气压优先模式。
		if( (mini.ok == 1) &&  (mini.flow_High>20) && (mini.flow_High<400)  ) //判断是否存在光流  高度高于20CM 光流才可以定点
		{//悬停模式 必须要有激光，但可以设定气压优先，用来测试气压定高效果

				//正在拨动方向摇杆，此时期望速度，有摇杆决定，而非通过位置pid计算。
				//摇杆值不能包含漂移修正
				//注意：姿态模式也会进来，此时Remote.pitch和Remote.roll包含了零漂修正值，我们要力度，但不要零漂
				//if((Remote.pitch>(1500+DEADBAND))||(Remote.pitch<(1500-DEADBAND))||(Remote.roll>(1500+DEADBAND))||(Remote.roll<(1500-DEADBAND))) //拨动遥杆只进行水平速度控制
				if((Aux_Rc.pitch_raw>(1500+DEADBAND))||(Aux_Rc.pitch_raw<(1500-DEADBAND))||(Aux_Rc.roll_raw>(1500+DEADBAND))||(Aux_Rc.roll_raw<(1500-DEADBAND))) //拨动遥杆只进行水平速度控制
				{
							//拨动摇杆不要关闭光流
							altHold_Pos_Save = 1;  			//代表摇杆正在拨动
							if(move_cnt<250)move_cnt++; //记录移动时间，0.5s后快速移动
					
							//摇杆向前 = 摇杆姿态角（pitch+) = 光流期望坐标x-（向前运动） = 机体期望姿态角（pitch-，顺时针）
							//摇杆向右 = 摇杆姿态角（ roll+) = 光流期望坐标y-（向右运动） = 机体期望姿态角（ roll-，顺时针）
							//注意遥控姿态角方向与机体姿态角方向的关系：遥控为了符合用户习惯，摇杆姿态角与机体姿态角的方向是反的，摇杆姿态角为正代表用户想向右向上运动，此时机体姿态角应该是负值（顺时针转动）
							//注意光流坐标与机体姿态角方向的关系：光流坐标x-,y-分别代表向前和向右运动。此时机体姿态角分别为r-，p-；姿态角数值正负号与光流坐标一致；姿态角负代表逆时针。
							//注意机体加速度坐标：机体加速度方向要符合加速度计的算法习惯，机体加速度坐标系与光流坐标系可以不一致，因为只有在高度方向的定高算法中使用到加速度。
							//并不存在当前机体位置坐标系，只有当前光流坐标系和机体加速度坐标系。而机体加速度坐标系主要用于定高，与定点无关。定点依靠的光流坐标和机体姿态角

					
#if ( FLY_TYPE==3 ) //无刷 
							if(move_cnt<30){//0.3s内触发慢速移动，用于位置修正，摇杆拨动方法：快速点拨后松开
										Flow_PosPid_x.desired	-= (Remote.pitch-1500)*0.25*0.01;//8级力度500对应每秒50cm，3级力度200对应每秒20cm，1级力度60对应每秒6cm
										Flow_PosPid_y.desired -= (Remote.roll-1500)*0.25*0.01;//这里的0.01是本函数0.01秒执行一次。

							}
							else{//拨动0.5s以上触发快速移动，用于快速移动，摇杆拨动方法：按着不放，注意要提前松开，防止撞到障碍物
										Flow_PosPid_x.desired	-= (Remote.pitch-1500)*0.30*0.01;//8级力度500对应每秒50cm，3级力度200对应每秒20cm，1级力度60对应每秒6cm
										Flow_PosPid_y.desired -= (Remote.roll-1500)*0.30*0.01;//这里的0.01是本函数0.01秒执行一次。
							
							}
#else //空心杯
										Flow_PosPid_x.desired	-= (Remote.pitch-1500)*0.2*0.01;//8级力度500对应每秒50cm，3级力度200对应每秒20cm，1级力度60对应每秒6cm
										Flow_PosPid_y.desired -= (Remote.roll-1500)*0.2*0.01;//这里的0.01是本函数0.01秒执行一次。
							
#endif


					
							//Flow_PosPid_y.desired += (Remote.roll-1500)*0.1*0.01;//这里的0.01是本函数0.01秒执行一次。		
							
							//方向摇杆拨动中，光流pid参数设置为运动模式
#if ( FLY_TYPE==3 ) //无刷 光流pid运动参数，一键定高起飞或方向摇杆移动中
							//进入模式3，或者检测到方向摇杆动作，则调入光流运动pid参数
							if(flow_pid_param_type!=2) flow_pid_param_move();//无刷电机光流pid，起飞和手动移动时，pid参数较大

#endif

							//   目前坐标系设置，只是通过无数实验测试出来实现了较好效果的一种方法，并不是唯一方法；
							//   如用户有自认为在理论上存在更为合理的方法可以自行测试，如不理解但又无能力更改请保持现状即可。
							//
							//   光流坐标   x-     机体加速度坐标   x-	       机体姿态角 p-（顺时针为负值）   遥控姿态角 p+（为了符合用户习惯）
							//             /                      /                    /                             /
							//      y+ ___/___ y-          y- ___/___ y+        r+ ___/___ r-                 r- ___/___ r+   
							//           /|                     /|                   /|                            /|
							//          / |                    / |                  / |                           / |
							//        x+	|                  x+  |                p+  |                         p-  |
							//            z-                     z-                    
							//共有四个坐标系：遥控器坐标系（操作者视角），光流坐标系，机体加速度坐标系，机体姿态角坐标系。
							//不包含光流模块原始坐标系和陀螺仪ic原始坐标系，因为这2个坐标系需要分别转换为光流坐标系和机体加速度坐标系/机体姿态角坐标系
							//要注意光流坐标方向，与遥控姿态角方向不同，与机体姿态角方向也不同。这3者的对应关系一定要转换正确。
							//而光流与机体加速度坐标之间不需要对应关系，也不用转换；
							//最终实现无人机飞行方向的是机体期望姿态角坐标系与光流坐标系之间的关系，与机体加速度坐标系方向关系不大。机体加速度主要用于定高程序。

								
				}
				//方向摇杆已回中
				else	
				{
							move_cnt=0; //移动时间清零
					
					
							//设置刹车计数器初值。
							if(altHold_Pos_Save == 1)   //第一次回中
							{
											altHold_Pos_Save = 0; 	//摇杆已经回中
											altHold_Pos_cnt =100;	  //1s刹车程序，等待慢慢停止水平移动

							}

							//这段时间，期望坐标不再增加，实现刹车过程。
							if(altHold_Pos_cnt>0){
											altHold_Pos_cnt--;
								
								
											//刹车结束时，这时应该把期望坐标复位成0，最好的办法是移动坐标轴，这样才能确保不会造成任何抖动和回撤。
											//以后万一有光流异常mini.ok != 1时，需要复位光流，才不会抖动。
											if(altHold_Pos_cnt==0){ //

																//已经刹车1s了，速度慢下来了，此时移动坐标轴，影响更小。目的期望坐标归零，并且不会导致任何抖动和回撤
																pixel_flow.flow_pause=1;//这个标志的作用就是，让光流程序实现这个愿望，目的是在没有任何影响的情况下复位期望坐标
																//根据pixel_flow.loc_x的值，倒过来推算出mini.flow_x_i值。并重设mini.flow_x_i
																pixel_flow.pause_desired_x = Flow_PosPid_x.desired;//移动坐标轴，在不改变任何飞机状态下，实现期望坐标归零
																pixel_flow.pause_desired_y = Flow_PosPid_y.desired;
																
																//下次进来前，这4句都在flow.c实现了
																//pixel_flow.loc_x = pixel_flow.loc_x - pixel_flow.pause_desired_x;
																//pixel_flow.loc_y = pixel_flow.loc_y - pixel_flow.pause_desired_y;
																
																//这2句，由pixel_flow.flow_pause=1来实现
																//Flow_PosPid_x.desired=0; //期望坐标复位
																//Flow_PosPid_y.desired=0; //期望坐标复位
												

											}
											
											
												
											//方向摇杆拨动停止，光流pid参数设置为默认模式
#if ( FLY_TYPE==3 ) //无刷 光流pid默认参数，手动定高起飞或一键定高起飞悬停中
											//退出模式3，或者方向摇杆动作结束，则调入光流默认pid参数
											if(flow_pid_param_type!=1) flow_pid_param_default(); //无刷电机光流pid，定点悬停时，pid参数较小

#endif

											
							
							}

						
				}
				

				
				


				//外环pid计算，根据位置差异，计算出期望速度
				Flow_PosPid_y.measured = pixel_flow.loc_y;//实时位置反馈，在回中后的1秒内都是0，因为每次都要复位
				pidUpdate(&Flow_PosPid_y,dt);//位置PID运算
				Flow_PosPid_x.measured = pixel_flow.loc_x;//实时位置反馈
				pidUpdate(&Flow_PosPid_x,dt);//位置PID运算
				
				
				//用外环输出，改写内环期望  
				Flow_SpeedPid_y.desired = LIMIT(Flow_PosPid_y.out,-1000,1000);//位置PID输出，改写期望速度
				Flow_SpeedPid_x.desired = LIMIT(Flow_PosPid_x.out,-1000,1000);//位置PID输出，改写期望速度
				

				
				//正在拨动方向摇杆，用真实的速度反馈。//也可以用实测坐标-期望坐标，替代当前速度反馈；
				if(altHold_Pos_Save>0){
								Flow_SpeedPid_y.measured = pixel_flow.loc_ys*0.06;
								Flow_SpeedPid_x.measured = pixel_flow.loc_xs*0.06;//速度反馈
				}
				//正在刹车，此时期望坐标并不为零，不太建议用实测坐标替代实测速度
				else if(altHold_Pos_cnt>1){
								Flow_SpeedPid_y.measured = pixel_flow.loc_ys*0.06;
								Flow_SpeedPid_x.measured = pixel_flow.loc_xs*0.06;//速度反馈
				}
				//平时，期望坐标为0，此时才可以用实测坐标替代速度反馈，此时比真实速度反馈效果稍好
				else{
								//在拨动阶段和刹车阶段，无人机运动速度和运动方向比较稳定，不会短时间内不停的更换方向。这样的速度反馈，可以计算出一个平稳的速度pid输出（即期望姿态角）
								//但在平时的悬停阶段，无人机运动多为随机的微小运动，尤其是方向上忽左忽右，忽前忽后，以这样的速度反馈，很难得到一个平稳的pid输出，
								//所以我们选用当前无人机最终位置偏移坐标来替代当前漂移速度。这样计算出来的pid输出，正好使得无人机超回归中心方向运动，不会导致加大漂移坐标的远离中心移动，实现稳定快速回位中心。
								//这只是通过无数实验测试出来实现了较好效果的一种方法，并不是唯一方法；如用户认为在理论上存在更为合理的方法可自行测试。
								Flow_SpeedPid_x.measured = pixel_flow.loc_x- Flow_PosPid_x.desired;//以偏移期望的位移，替代当前速度反馈，相当于测量速度与期望速度相反。
								Flow_SpeedPid_y.measured = pixel_flow.loc_y- Flow_PosPid_y.desired;//以偏移期望的位移，替代当前速度反馈

				}
				
				

				
				pidUpdate(&Flow_SpeedPid_y,dt);//速度PID运算，得到 Flow_SpeedPid_x.out
				pidUpdate(&Flow_SpeedPid_x,dt);//速度PID运算
				
				
				
				
				
//以下限幅 已经转移到：Flow_mode_two()
				/*
#if ( FLY_TYPE==3 ) //无刷电机。 光流的速度pid输出限幅（期望姿态角限幅），避免抖动。
				
				if(altHold_Pos_cnt){ //正在刹车
							Flow_SpeedPid_y.out = LIMIT(Flow_SpeedPid_y.out,-40,40);//速度PID输出，改写期望姿态角
							Flow_SpeedPid_x.out = LIMIT(Flow_SpeedPid_x.out,-40,40);//速度PID输出，改写期望姿态角
				}
				if(altHold_Pos_Save){ //摇杆正在拨动
							Flow_SpeedPid_y.out = LIMIT(Flow_SpeedPid_y.out,-60,60);//速度PID输出，改写期望姿态角
							Flow_SpeedPid_x.out = LIMIT(Flow_SpeedPid_x.out,-60,60);//速度PID输出，改写期望姿态角
				}
				else{//平时
							//防止光流环境质量不足时，pid输出值过大，引起抖动；仅针对无刷电机(力大)。空心杯似乎没有这个问题
							Flow_SpeedPid_y.out = LIMIT(Flow_SpeedPid_y.out,-25,25);//速度PID输出，改写期望姿态角
							Flow_SpeedPid_x.out = LIMIT(Flow_SpeedPid_x.out,-25,25);//速度PID输出，改写期望姿态角
				}

#endif
			*/
				
				
				
				//cpu线程资源快耗光了，最好不要再做滤波了
				
				/*
				float ratio_speed=0.5;
				if(altHold_Pos_cnt){ //正在刹车
					ratio_speed=0.5;  //姿态角变化不用太快，太快了抖动
				}
				if(altHold_Pos_Save){ //摇杆正在拨动
						ratio_speed=0.5;  //姿态角缓慢变化，太快了，到了目标速度姿态角立即为零，给人感觉不稳，
				}
				else{//平时
						ratio_speed=1.0;  //平时不滤波，和原来的程序一样
				}

				//对输出滤波
				
				Flow_SpeedPid_x.out = Flow_SpeedPid_out_old_x + (Flow_SpeedPid_x.out- Flow_SpeedPid_out_old_x) * ratio_speed;
				Flow_SpeedPid_y.out = Flow_SpeedPid_out_old_y + (Flow_SpeedPid_y.out- Flow_SpeedPid_out_old_y) * ratio_speed;
				Flow_SpeedPid_out_old_x=Flow_SpeedPid_x.out;
				Flow_SpeedPid_out_old_y=Flow_SpeedPid_y.out;
				*/
				
			

				//以下4行已经转移到：Flow_mode_two()
				
				//pidPitch.desired =  LIMIT(Flow_SpeedPid_x.out*0.1,-15,15) ; //姿态外环期望值 (Remote.pitch-1500)*8*0.006
				//pidRoll.desired = LIMIT(Flow_SpeedPid_y.out*0.1,-15,15) ; //姿态外环期望值  实际系数为0.048f，与姿态模式类似。

				//pidPitch.desired += (-Aux_Rc.offse_pitch)*0.04f; //漂移 静态修正(用户修正值)
				//pidRoll.desired += (-Aux_Rc.offse_roll)*0.04f;//已经转移到：Flow_mode_two()

				
							
				
		
		}
		
		
		//光流只在20~400cm生效，小于20cm或者大于4米，依旧靠 零漂修正 直接影响期望姿态角。
		//注意：姿态模式也会进来
		else//光流失效，或者高度（0.2~4米）不在范围内，清空光流数据。
		{
			

						mini.flow_x_i = 0;				//光流未融合位置复位
						mini.flow_y_i = 0;				//
			
						pixel_flow.loc_y=0; 			//已融合光流位置复位
						pixel_flow.loc_x=0;
			
						pixel_flow.loc_ys=0; 			//当前速度
						pixel_flow.loc_xs=0;
												
						//新增  //高度大于20cm，才开启光流
						//mini.flow_pause=1;//新增。防止姿态角变化，导致光流坐标不断累计。
							
						//这里的姿态角应该是要包含xy漂移补偿,而不是简单的清零
						//直接修改期望姿态角 pidPitch.desired
						Flow_SpeedPid_x.out = 0;	//速度pid输出，决定着姿态角
						Flow_SpeedPid_y.out = 0;	//速度pid输出，决定着姿态角
						
						Flow_SpeedPid_out_old_x =0; //滤波复位，这个要手动复位，不跟跟着pid复位
						Flow_SpeedPid_out_old_y =0;
			

						Flow_PosPid_x.desired=pixel_flow.loc_x = 0;
						Flow_PosPid_y.desired=pixel_flow.loc_y = 0;


						
						//以下4行，已经转移到：Flow_mode_two()
						//在悬停模式，这个值已经包含力度修正，但不包含零漂修正
						//以下非常重要，不能注销，悬停模式在光流失效时，Mode_Controler()里面从未计算过姿态角
						//pidPitch.desired = -(Remote.pitch-1500)*0.04f; //(Remote.pitch-1500)*0.04
						//pidRoll.desired  = -(Remote.roll-1500)*0.04f;  //已经转移到：Flow_mode_two()
						
						//pidPitch.desired += (-Aux_Rc.offse_pitch)*0.04f; //零漂修正
						//pidRoll.desired += (-Aux_Rc.offse_roll)*0.04f;
					
					
					
			
			
		}
		

	
}




/**************************************************************
 * 姿态控制
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
//2ms一次
void FlightPidControl(float dt)
{
	volatile static uint8_t status=WAITING_1;

	switch(status)
	{		
		case WAITING_1: //等待解锁
			if(ALL_flag.unlock)
			{
				status = READY_11;	
			}			
			break;
			
		case READY_11:  //准备进入控制
			//pidRest(pPidObject,8); //批量复位PID数据，防止上次遗留的数据影响本次控制
			pidRest(pPidObject,12); //批量复位PID数据，防止上次遗留的数据影响本次控制，不要把4个定点pid遗漏了，否则下次手动定高起飞会偏
		

		
			//偏航角清零
			Angle.yaw = pidYaw.desired =  pidYaw.measured = 0;   //锁定偏航角, 当前偏航角清零
		
		
			status = PROCESS_31;
		
			break;			
		
		case PROCESS_31: //正式进入控制
			

			//角速度
      pidRateX.measured = MPU6050.gyroX * Gyro_G; //内环测量值 角度/秒
			pidRateY.measured = MPU6050.gyroY * Gyro_G; //内环测量值 角度/秒
			pidRateZ.measured = MPU6050.gyroZ * Gyro_G; //内环测量值 角度/秒
		
			//角度
			pidPitch.measured = Angle.pitch; 		//外环测量值 单位：角度
		  pidRoll.measured = Angle.roll;			//外环测量值 单位：角度
			pidYaw.measured = Angle.yaw;				//外环测量值 单位：角度
		
			//roll
		 	pidUpdate(&pidRoll,dt);    //调用PID处理函数来处理外环	横滚角PID		
			//角度偏差，得到期望角速度m 如果在起飞时，pidRoll.out持续大的输出，
			//说明现在的电池电量下降，无法快速达到零漂修正姿态角。将出现大幅零漂, 此时应该调整动态零漂修正，使得当前零漂姿态角尽快恢复
			//用户零漂修正是通过更改摇杆值->产生零漂姿态角->...->产生油门调整到期望姿态角为止。
			
			//用户零漂姿态角与当前姿态角做对比，如果有差异，则调整期望姿态角（通过更改动态漂移修正实现）。直到零漂姿态角与当前姿态角相同。
			//只有当姿态角pid输出很大时（说明动力不足，实际姿态角一直追不上期望姿态角），
			//并且零漂姿态角与当前姿态角有差异，才调整动态漂移修正值。
			//期望姿态角= 用户零漂姿态角 + 动态零漂姿态角
			//总结：引入动态零漂姿态角的目的，是让当前姿态角尽快与用户零漂姿态角一致。避免漂移
		
			pidRateX.desired = pidRoll.out; //将外环的PID输出作为内环PID的期望值即为串级PID
			pidUpdate(&pidRateX,dt);  //再调用内环
			//角速度偏差，得到期望油门

			//Pitch
		 	pidUpdate(&pidPitch,dt);    //调用PID处理函数来处理外环	俯仰角PID	
			pidRateY.desired = pidPitch.out;  
			pidUpdate(&pidRateY,dt); //再调用内环
			
			//Yaw
			CascadePID(&pidRateZ,&pidYaw,dt);	//也可以直接调用串级PID函数来处理
			//对pidRateZ的输出结果进行修正，解决yaw漂移。修正值+/-128
		
			//pidRateZ.out= pidRateZ.out*(1+0.0001f*1.0f); //放到电机输出之后，要用当时的电机输出微调修正量。
			
		
			break;
			
		case EXIT_255:  						//退出控制
			pidRest(pPidObject,8);		//复位PID参数
			status = WAITING_1;				//返回等待解锁
		  break;
		
		default:
			status = EXIT_255;
			break;
		
	}
	
	if(ALL_flag.unlock == EMERGENT) //意外情况，请使用遥控紧急上锁，飞控就可以在任何情况下紧急中止飞行，锁定飞行器，退出PID控制
		status = EXIT_255;
	
	
}


int16_t motor[4];
#define MOTOR1 motor[0] 
#define MOTOR2 motor[1] 
#define MOTOR3 motor[2] 
#define MOTOR4 motor[3] 


//Aux_Rc.thr_raw



//2ms一次
void MotorControl(void) //电机控制
{	
	
	static uint16_t thr_count=0;
	static int16_t thr_raw_old=0; //上次摇杆值，用于比对

	static uint8_t AUX88_old=0;

	volatile static uint8_t status=WAITING_1;
	
	
	if(ALL_flag.unlock == EMERGENT) //=0， 意外情况，请使用遥控紧急上锁，飞控就可以在任何情况下紧急中止飞行，锁定飞行器，退出PID控制
			status = EXIT_255;	
	

	
	
	switch(status)
	{
		
		
		case WAITING_1: 	     //等待解锁	
			MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = 0;  //如果锁定，则电机输出都为0
		
			if(ALL_flag.unlock) //进入解锁
			{
					status = WAITING_2;

			}
			
			break;
			
			
		case WAITING_2: //解锁完成后判断使用者是否开始拨动遥杆进行飞行控制
			if(Remote.thr>1100)
			{
					status = PROCESS_31;
				
					thr_isChange=0;//摇杆变化状态，复位
					thr_raw_old=0;//摇杆拨动比较基准复位
				
				
					//这里一定要调用一下，虽然在FlightPidControl() ready11里面有pid复位。
					//但我们使用set+key2方式解锁走不到那里
					pidRest(pPidObject,12); //批量复位PID数据，防止上次遗留的数据影响本次控制，不要把4个定点pid遗漏了，否则下次手动定高起飞会偏
					//mini.flow_pause=1;//复位光流
				

				
			}
			break;
			

			
			
			
		case PROCESS_31:
		{
					int16_t thr_temp;



					//判断是否在拨动摇杆	
						
					//这里虽然2ms一次判断，但实际遥控那边油门变化是10ms发送一次，飞控这边4ms接收一次遥控值
						
					//比上次 更新期望高度时的油门，大 +/-10，说明用户拨动了油门，在模式1用户可以通过油门调整定高高度，如果此时在模式2，油门动作则会使得模式转为模式1
					if(  ((Aux_Rc.thr_raw > thr_raw_old+10 || Aux_Rc.thr_raw < thr_raw_old-10) &&  hight_mode!=2 &&  hight_mode!=3 )
							//当处于模式3(一键定高起飞)或者模式2(一键定高飞行中)，需要提高阈值，油门变化会导致模式2/3变为模式1(手动油门辅助定高)，有时遥控抖动或轻微振动也会引起的油门变化，一般会小于50
							|| ((Aux_Rc.thr_raw > thr_raw_old+50 || Aux_Rc.thr_raw < thr_raw_old-50) && (hight_mode==2 ||  hight_mode==3) )
					) //
					{
								if(thr_count++>100 && thr_isChange==0){ //自上次更新高度以来，油门有变化，并且达到阈值后过了0.2s，thr_count作为滤除干扰或毛刺信号
									
									
												thr_raw_adds = Aux_Rc.thr_raw - thr_raw_old; //记录本次摇杆变化，用于平衡油门归零
									
												//要防止油门屏蔽动作，误认为摇杆动作，导致模式3变成模式1
												//防止飞控刚开机第一次进入一键定高起飞（模式3），误认为是摇杆动作，直接变成了模式1
												if( thr_raw_old>1000 && Aux_Rc.thr_raw>1000 ){//刚开机为0，屏蔽为1000，正常操作>1000
															if( thr_raw_adds>0 ){
																			thr_isChange=1;			//有往上拨动摇杆，0.2秒后触发 标识位
															}
															else{
																			thr_isChange=-1;		//有往下拨动摇杆，0.2秒后触发 标识位
															}		
												}
												
												thr_raw_old = Aux_Rc.thr_raw;	//更新上次油门
												thr_count=0; 									//计时复位

								}

					}
					else{
							thr_count=0; //计时清零，防止多次毛刺干扰信号，误认为用户拨动了油门。
					
					}
						
						

					//姿态模式，不要识别油门摇杆变化
					if(!ALL_flag.height_lock  && !ALL_flag.height_lock_qiya ) thr_isChange=0;//非悬停模式，并且非气压定高，清空油门变化判断
					
					
					//一键定高起飞以外，先清空临时油门增加值
					if(!(hight_mode==3 && Aux_Rc.fly_key2)){
									thr_hight_modify=0; //临时油门增减值清零（按键触发）
					
					}
					
					
					//能到这里，说明在定高起飞或飞行，
					if(ALL_flag.height_lock || ALL_flag.height_lock_qiya) //悬停模式或气压定高模式 油门遥杆作为调整高度使用   
					{
						
								//按键H+/-被按下， 油门临时增加值：thr_hight_modify
								if(AUX87_isH() || AUX88_isH()){//这里要做一下判断，当按着H+/-，再按紧急解锁时这个值来不及复位，因为锁定后无法及时复位hight_modify
								
												if(hight_modify==-1){//下降
															//thr_hight_modify=-100;
															thr_hight_modify= -60-((Remote.thr -1000) + thr_adjust)*0.1;

												} 
												else if(hight_modify==1){//上升
																//thr_hight_modify=100;
																//thr_hight_modify =60+((Remote.thr -1000) + thr_adjust)*0.1;
																thr_hight_modify =100+((Remote.thr -1000) + thr_adjust)*0.1; //上升加大
												}
								}
								
								

								//最终油门，当hight_mode=2时，平衡油门会自动修正基础油门的不足，以及定高过程由于电池电量下降等原因导致的的自动修正
								thr_temp = pidHeightRate.out  +  (Remote.thr -1000)  +  thr_base_add  +   thr_adjust   +   thr_hight_modify; //这是定高油门，还要限幅，留余量给姿态调整
								//         内环pid输出(加速度)     当前基础油门(摇杆)       基础补偿       平衡油门修正值    油门临时增减(按键H+/-)
			
			
								//当油门为0时，应该屏蔽最终油门输出，否则还没起飞就会乱动（因为其他2个值的原因）。
								if(Remote.thr<1020) thr_temp=0;//油门摇杆为零，则油门输出为0

						
						
						
					}
					else  //姿态模式，油门正常使用
					{
								thr_temp = Remote.thr -1000; //输出给电机的是油门输出值，只有基础油门
					}
					

					
					//油门太低了，则限制输出  不然飞机乱转	
					//要连续多次才停止输出
					static uint16_t low_thr_cnt=0;
					if(Remote.thr<1020)		//油门太低了，则限制输出  不然飞机乱转												
					{
								low_thr_cnt++;
								if(low_thr_cnt>=500)//500*2=1000ms
								{
									
											//连续1秒，油门为零，输出为0
											MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4=0;
									
											//退出PROCESS_31:
											status = WAITING_1;	//退出PROCESS_31: 
											low_thr_cnt=0; //低油门计时复位， >1100才能重新进来
											//在FlightPidControl() ready11里面有pid复位。
											//这里也可以再调用一下pid复位，但要注意不是6个，而是12个，最后4个是定点pid

											//退出后重新进入WAITING_2时，会复位光流和pid

									
											//如果是用户持续把油门放到最低位，说明用户希望锁定飞机
											if(!Aux_Rc.thr_mask){ //如果是按了SET+key2屏蔽油门导致的，油门突然为零，只退出PROCESS_31油门生成程序，但不能锁定飞机
													ALL_flag.unlock = 0; //紧急锁定,起飞后，油门又长时间为0的，这时应该1s锁定，不用等6s
											}
											//油门屏蔽动作导致的油门为零，不要锁定飞机
											else{
											}
											
											
									
											break;
								}
					}
					else low_thr_cnt=0; //复位
					
					
					
					//高度控制
					//MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = LIMIT(thr_temp,0,900); //油门占的太多，调高度时会影响到姿态角
					MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = LIMIT(thr_temp,0,800); //留200给姿态控制
					
					//这里是飞行过程修正，由用户设置修正量；而静止状态下的修正在imu.c, MPU6050.gyroZ_offset，在静止状态会自动实时修正yaw。
					//偏航修正, 在不同电池电量下，相同电机数值代表的输出推力是不同的，电机基础数值大说明代表的推力减弱了。此时偏航修正数值也要加大
					//pidRateZ.out += (Aux_Rc.offse_yaw + Aux_Rc.offse_yaw*((float)MOTOR1/800.0f))*0.5f; //偏航修正值不是值越大调整效果越大，修正值太大了z角速度被检测出来，就失去修正效果了
					//修正这里没有用，还是要修正测量值，在imu.c和mpu6050.c
					
					//姿态控制
					MOTOR1 +=    + pidRateX.out - pidRateY.out - pidRateZ.out;//; 姿态输出分配给各个电机的控制量
					MOTOR2 +=    + pidRateX.out + pidRateY.out + pidRateZ.out ;//;
					MOTOR3 +=    - pidRateX.out + pidRateY.out - pidRateZ.out;
					MOTOR4 +=    - pidRateX.out - pidRateY.out + pidRateZ.out;//;
					

					

					
					//             蓝3(逆)     ↑      2蓝(顺)
					//
					//
					//                       X
					//
					// 
					//             红4(顺)            1红(逆)
					
					
					
				}	
				break;
			
			
		case EXIT_255:
				MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = 0;  //如果锁定，则电机输出都为0
				status = WAITING_1;	 //如果当前为锁定状态，下次刚进入函数就会被重设为  EXIT_255，并不是进入 WAITING_1

				//新增电机手动测试，1.上位机点击 罗盘校准（或者遥控器按 SET+“下”），2.遥控按H+转动，松开停止，3.遥控按H-切换测试电机
				//增加马达手动测试，可调油门，观察哪个马达震动大，主要看有无1Khz以下的低频振动，尤其是能与壳体共振的那种低频振动。会导致陀螺仪和加速度计不准。
				//油门摇杆设置为1400左右，无刷最容易振动（速度很快或者很慢反而没有振动）。看此时哪个电机会与壳体共振。
				if(Remote.test_motor){ //电机手动测试；进入方法1：上位机点击“罗盘校准”，进入方法2：遥控器按 SET+“下”
							//油门 1000~2000 H+被按下，电机转动，转速与当前油门摇杆位置有关; 提高易用性，按了H+，当油门为零则会以最低速转动。
							if(     AUX87_isH() && Remote.test_motor==1){ //H+
									MOTOR1 = LIMIT(Remote.thr-1000, 140, 1000);
							}
							else if(AUX87_isH() && Remote.test_motor==2){ //H+
									MOTOR2 = LIMIT(Remote.thr-1000, 140, 1000);
							}
							else if(AUX87_isH() && Remote.test_motor==3){ //H+
									MOTOR3 = LIMIT(Remote.thr-1000, 140, 1000);
							}
							else if(AUX87_isH() && Remote.test_motor==4){ //H+
									MOTOR4 = LIMIT(Remote.thr-1000, 140, 1000);
							}
							else{
									MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4=0;
							}

							if(AUX88_isH() && AUX88_old==0){ //H-被按下，切换电机
									Remote.test_motor++;
									if(Remote.test_motor>=5) Remote.test_motor=1;//1~4循环

							}
					
							AUX88_old = AUX88_isH();//更新 H-, 下次按下才能触发电机切换
				}
				
				else{
							//MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = 0;  //如果锁定，则电机输出都为0
				}
		
		
		
				break;
				
				
		
		default:
			break;

	}

	
	

	//返回给屏幕显示在详细数据界面
	Aux_Rc.debug1 = hight_mode;//详细定高模式，0为姿态
	Aux_Rc.debug2 = Remote.thr -1000;// 基础油门，摇杆值比例修正后。只有这个是正数，其他都是有正有负
	Aux_Rc.debug3 = thr_base_add;//一键定高起飞的基础油们补偿
	Aux_Rc.debug4 = thr_adjust;//			普通定高模式下，自动调整 平衡油门，平衡油门指用于抵消飞机重力的油门
	Aux_Rc.debug5 = thr_hight_modify;// 临时油门增减，用于高度上升或下降过程。

	
	
	
//	TIM2->CCR1 = LIMIT(MOTOR1,0,1000);  //更新PWM1
//	TIM2->CCR2 = LIMIT(MOTOR2,0,1000);  //更新PWM2
//	TIM3->CCR1 = LIMIT(MOTOR3,0,1000);  //更新PWM3
//	TIM3->CCR2 = LIMIT(MOTOR4,0,1000);  //更新PWM4
////	TIM2->CCR3 = LIMIT(MOTOR3,0,1000);  //更新PWM3
////	TIM2->CCR4 = LIMIT(MOTOR4,0,1000);  //更新PWM4
	
#if (FLY_TYPE == 1 || FLY_TYPE == 2)
	
	PWM0 = LIMIT(MOTOR1,0,1000);  //更新PWM1
	PWM1 = LIMIT(MOTOR2,0,1000);  //更新PWM2
	PWM2 = LIMIT(MOTOR3,0,1000);  //更新PWM3
	PWM3 = LIMIT(MOTOR4,0,1000);  //更新PWM4
	
#elif (FLY_TYPE >= 3)
	
	//PWM0 = 1000 + LIMIT(MOTOR1,0,1000);  //更新PWM1
	//PWM1 = 1000 + LIMIT(MOTOR2,0,1000);  //更新PWM2
	//PWM2 = 1000 + LIMIT(MOTOR3,0,1000);  //更新PWM3] ;     
	//PWM3 = 1000 + LIMIT(MOTOR4,0,1000);  //更新PWM4
	
	
	//无刷解锁后电机要以最低速度转动
	if(ALL_flag.unlock && Remote.thr==1000){ //1000表示油门为0，油门范围1000~2000
			//	1075/2500 = 43%, 至少要75才能勉强转动 100为44%，120更可靠（1120/2500=44.8%）
			PWM0 = TIM2_DUTY + 120;  //更新PWM1
			PWM1 = TIM2_DUTY + 120;  //更新PWM2
			PWM2 = TIM3_DUTY + 120;  //更新PWM3   
			PWM3 = TIM3_DUTY + 120;  //更新PWM4
	
	
	}
	else if(ALL_flag.unlock){ //已解锁，且油门大于1000，表示飞行中
			//注意：无刷电机的螺旋桨比较特殊，一旦停转，无法立即恢复，
			//所以这里把 LIMIT(MOTORn,0,1000) 改为 LIMIT(MOTORn,140,1000), 1140/2500=45.6%
		  //最低转速比解锁待机时转速稍高
			PWM0 = TIM2_DUTY + LIMIT(MOTOR1,140,1000);  //更新PWM1
			PWM1 = TIM2_DUTY + LIMIT(MOTOR2,140,1000);  //更新PWM2
			PWM2 = TIM3_DUTY + LIMIT(MOTOR3,140,1000);  //更新PWM3
			PWM3 = TIM3_DUTY + LIMIT(MOTOR4,140,1000);  //更新PWM4
	
	}
	else{ //飞机锁定中，无刷处于不转的待机状态
			PWM0 = TIM2_DUTY + LIMIT(MOTOR1,0,1000);  //更新PWM1
			PWM1 = TIM2_DUTY + LIMIT(MOTOR2,0,1000);  //更新PWM2
			PWM2 = TIM3_DUTY + LIMIT(MOTOR3,0,1000);  //更新PWM3   
			PWM3 = TIM3_DUTY + LIMIT(MOTOR4,0,1000);  //更新PWM4
		
	}
	
	
	
	
	
	
	
#else
	#error Please define FLY_TYPE!
		
#endif

} 


/************************************END OF FILE********************************************/ 



