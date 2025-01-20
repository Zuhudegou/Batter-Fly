#ifndef _ALL_USER_DATA_H_
#define _ALL_USER_DATA_H_

typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       long long int64_t;

    /* exact-width unsigned integer types */
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       long long uint64_t;


#define NULL 0
extern volatile uint32_t SysTick_count;
extern volatile uint8_t spl_flag;
extern volatile uint32_t ST_CpuID;

typedef struct
{
 float Input_Butter[3];
 float Output_Butter[3];
}Butter_BufferData;

typedef struct
{
 const float a[3];
 const float b[3];
}Butter_Parameter;


typedef struct{
	int16_t accX;
	int16_t accY;
	int16_t accZ;
	int16_t gyroX;
	int16_t gyroY;
	int16_t gyroZ;
	
	//以上顺序不能更改增删，pMpu数组和MPU6050共用
	
	int16_t gyroZ_raw; //没有经过修正的值，用作调试比对
	float gyroZ_offset; //一定要float，计算yaw角速度偏移，这个偏移无法通过MUP6050校准消除。
	float gyroZ_offset1; //飞行中，偏航角测量偏差=用户设置的零漂修正。 遥控设置方法 SET+P 或者 SET+M
	uint16_t gyroZ_cnt;	//静态修正次数累计，此时gyroZ_offset1=0，飞行中,才有gyroZ_offset1。

	int16_t gyroZ_sum; //没有经过修正的值，用作调试比对
	uint16_t gyroZ_sum_cnt;//用来求均值
	
	//新增，起飞1s内的积分速度
	//float speedX; //MPU提供的水平方向的测量速度并无参考意义
	//float speedY; //MPU提供的水平方向的测量速度并无参考意义
}_st_Mpu;


typedef struct{
	int16_t magX;
	int16_t magY;
	int16_t magZ;
}_st_Mag;


typedef struct{
	float rate;
	float height;
}High;


typedef struct{
	float roll;
	float pitch;
	float yaw;
	float tmp;
}_st_AngE;

 typedef struct
{	
//		 struct{  //角度数据 
//			float roll;
//			float pitch;
//			float yaw;
//		}Angle;
		 struct{  //高度数据
			float rate;
			float bara_height; 
			float ultra_height;
			float ultra_baro_height;
		}High;		 
}_st_FlightData;


typedef struct
{
	uint16_t roll;
	uint16_t pitch;
	uint16_t thr;
	uint16_t yaw;
	
	//uint16_t raw_data1; //测试
	//uint16_t raw_data2; //测试
	
	uint16_t AUX1;
	uint16_t AUX2;
	uint16_t AUX3;
	uint16_t AUX4;	
	uint16_t AUX5;
	uint16_t AUX6;
	
	//新增
	uint16_t AUX7;
	uint16_t AUX8;
	
	uint8_t new_data; //来了新数据
	uint8_t rst_count; //失联次数
	
	uint8_t test_motor; //当前动平衡测试电机序号（1~4）
	
}_st_Remote;



//新增
struct _Aux_Rc//当前摇杆值解析
{
	//int8_t position;	//悬停模式，，，与 ALL_flag.height_lock 一样，都是代表悬停模式
	uint8_t headless;			//无头模式
	uint8_t thr_mask; 			//油门屏蔽
	uint8_t fly_key2;			//一键定高起飞
	
	uint8_t setMpu;				//校准判断
	uint8_t testMotor;		//测试马达
	float yaw_stick; //摇杆控制的角度，最终偏航角=摇杆控制角度+按键控制角度
	
	int8_t lidu;
	int8_t move_fx_z;
	double thr_ratio;
	int16_t move_thr; 	//悬停模式下，没有油门临时增量

	int16_t offse_roll; 	//零漂修正 改为双字节 +/-256
	int16_t offse_pitch;
	
	int8_t offse_yaw; //偏航动态修正，直接修正内环输出。漂移原因：z角速度积分成yaw时，小数据被是做干扰丢弃了；直接作用于陀螺仪 gyroZ_offset1
	

	//float offset_roll_adjust; 	//动态零漂修正
	//float offset_pitch_adjust; //必须为浮点，不然无法响应每次微小变化。
	
	
	int16_t thr_raw;		//纯油门摇杆值，不含油门比例和临时增加值
	int16_t roll_raw;				//纯方向摇杆值
	int16_t pitch_raw;		//纯方向摇杆值
	
  //以下与 Remote 中的一致
	int16_t thr;				//两种模式都有包含油门比例，但悬停模式油门不含临时增加值
	int16_t yaw;
	int16_t roll; 			//悬停模式下，含力度，但不含零漂修正值
	int16_t pitch;			//姿态模式下，含力度，也包含零漂修正值
	
	
	int16_t debug1,debug2,debug3,debug4,debug5,debug6,debug7,debug8;//传给遥控的调试数据，调试数据 
	
};







typedef volatile struct
{
	float desired;     ///期望
	float offset;      //原本用来放置摇杆中位误差导致的修正，现在没有用到。
	float prevError;    // 上次偏差
	float integ;        //误差积分累加值
	float kp;           //p参数
	float ki;           //i参数
	float kd;           //d参数
	float IntegLimitHigh;       //< integral limit
	float IntegLimitLow;
	float measured;     ////pid反馈量
	float out;
	float OutLimitHigh;
	float OutLimitLow;	
	float Control_OutPut;//控制器总输出
	float Last_Control_OutPut;//上次控制器总输出
	float Control_OutPut_Limit;//输出限幅
		/***************************************/
	float Last_FeedBack;//上次反馈值
	float Dis_Err;//微分量
	float Dis_Error_History[5];//历史微分量
	float Err_LPF;
	float Last_Err_LPF;
	float Dis_Err_LPF;

//	int8_t Err_Limit_Flag :1;//偏差限幅标志
//	int8_t Integrate_Limit_Flag :1;//积分限幅标志
//	int8_t Integrate_Separation_Flag :1;//积分分离标志		
  Butter_BufferData Control_Device_LPF_Buffer;//控制器低通输入输出缓冲	
}PidObject;


typedef volatile struct
{
	uint8_t 	unlock;					//已解锁
	uint32_t  slock_flag;
	uint8_t 	height_lock:1;  //悬停模式 定高+光流
	uint8_t 	height_lock_qiya:1;  //气压定高模式，光流失效时
	uint8_t 	take_off:1;
	uint8_t 	take_down:1;

}_st_ALL_flag;



typedef volatile struct
{
	uint8_t AccOffset :1;  //校准命令
	uint8_t GyroOffset :1;
	uint8_t MagOffset :1;
	uint8_t six_acc_offset; //六面校准
	enum{ 								 //飞行模式切换命令
			LOCK = 0x00,				//锁定模式
			NORMOL, 		//基本模式		
			HEIGHT,			//定高模式
			Flow_POSITION,   //GPS定点模式
	}FlightMode; //飞行模式
}st_Command;



extern struct _Aux_Rc Aux_Rc; //当前摇杆值解析

extern _st_Remote Remote;
extern _st_Mpu MPU6050;
extern _st_Mag AK8975; //保留，需外接磁力计
extern _st_AngE Angle;


extern _st_ALL_flag ALL_flag;


extern	PidObject pidRateX;
extern	PidObject pidRateY;
extern	PidObject pidRateZ;

extern	PidObject pidPitch;
extern	PidObject pidRoll;
extern	PidObject pidYaw;

extern	PidObject pidHeightRate;
extern	PidObject pidHeightHigh;

extern	PidObject pidHeightThr; //用于平衡油门动态调整
extern	PidObject pidOffsetRoll; //用于零漂动态调整

extern PidObject Flow_PosPid_x; 
extern PidObject Flow_PosPid_y;

extern PidObject Flow_SpeedPid_x;
extern PidObject Flow_SpeedPid_y;

extern _st_FlightData FlightData;
//飞控命令
extern st_Command Command;



void GetLockCode(void);

//载入pid参数 在int.c
void pid_param_Init(void);
void flow_pid_param_default(void);//无刷电机光流pid参数，定点悬停时，pid参数较小，追求平稳，防止抖动；
void flow_pid_param_move(void);//无刷电机光流pid参数，起飞和手动移动时，pid参数较大，追求快速响应期望值；
extern volatile uint8_t flow_pid_param_type;//当前无刷电机光流pid参数类型

#endif

