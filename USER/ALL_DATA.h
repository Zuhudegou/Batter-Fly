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
	
	
	
	int16_t gyroZ_raw; 
	float gyroZ_offset; 
	float gyroZ_offset1; 
	uint16_t gyroZ_cnt;	

	int16_t gyroZ_sum; 
	uint16_t gyroZ_sum_cnt;
	
	
	//float speedX; 
	//float speedY;
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
//		 struct{ 
//			float roll;
//			float pitch;
//			float yaw;
//		}Angle;
		 struct{ 
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
	
	//uint16_t raw_data1;
	//uint16_t raw_data2; 

	uint16_t AUX1;
	uint16_t AUX2;
	uint16_t AUX3;
	uint16_t AUX4;	
	uint16_t AUX5;
	uint16_t AUX6;
	

	uint16_t AUX7;
	uint16_t AUX8;
	
	uint8_t new_data; 
	uint8_t rst_count; 
	
	uint8_t test_motor;
	
}_st_Remote;




struct _Aux_Rc
{
	//int8_t position;	
	uint8_t headless;			
	uint8_t thr_mask; 			
	uint8_t fly_key2;			
	
	uint8_t setMpu;				
	uint8_t testMotor;		
	float yaw_stick;
	
	int8_t lidu;
	int8_t move_fx_z;
	double thr_ratio;
	int16_t move_thr; 	

	int16_t offse_roll; 	
	int16_t offse_pitch;
	
	int8_t offse_yaw;
	

	//float offset_roll_adjust; 	
	//float offset_pitch_adjust; 
	
	
	int16_t thr_raw;		
	int16_t roll_raw;				
	int16_t pitch_raw;		
	

	int16_t thr;				
	int16_t yaw;
	int16_t roll; 			
	int16_t pitch;			
	
	
	int16_t debug1,debug2,debug3,debug4,debug5,debug6,debug7,debug8;
	
};







typedef volatile struct
{
	float desired;    
	float offset;      
	float prevError;    
	float integ;        
	float kp;           
	float ki;           
	float kd;           
	float IntegLimitHigh;       //< integral limit
	float IntegLimitLow;
	float measured;     
	float out;
	float OutLimitHigh;
	float OutLimitLow;	
	float Control_OutPut;
	float Last_Control_OutPut;
	float Control_OutPut_Limit;
		/***************************************/
	float Last_FeedBack;
	float Dis_Err;
	float Dis_Error_History[5];
	float Err_LPF;
	float Last_Err_LPF;
	float Dis_Err_LPF;

//	int8_t Err_Limit_Flag :1;
//	int8_t Integrate_Limit_Flag :1;
//	int8_t Integrate_Separation_Flag :1;		
  Butter_BufferData Control_Device_LPF_Buffer;
}PidObject;


typedef volatile struct
{
	uint8_t 	unlock;					
	uint32_t  slock_flag;
	uint8_t 	height_lock:1;  
	uint8_t 	height_lock_qiya:1; 
	uint8_t 	take_off:1;
	uint8_t 	take_down:1;

}_st_ALL_flag;



typedef volatile struct
{
	uint8_t AccOffset :1;  
	uint8_t GyroOffset :1;
	uint8_t MagOffset :1;
	uint8_t six_acc_offset; 
	enum{ 								
			LOCK = 0x00,				
			NORMOL, 				
			HEIGHT,			
			Flow_POSITION,  
	}FlightMode; 
}st_Command;



extern struct _Aux_Rc Aux_Rc; 

extern _st_Remote Remote;
extern _st_Mpu MPU6050;
extern _st_Mag AK8975; 
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

extern	PidObject pidHeightThr; 
extern	PidObject pidOffsetRoll;

extern PidObject Flow_PosPid_x; 
extern PidObject Flow_PosPid_y;

extern PidObject Flow_SpeedPid_x;
extern PidObject Flow_SpeedPid_y;

extern _st_FlightData FlightData;

extern st_Command Command;



void GetLockCode(void);


void pid_param_Init(void);
void flow_pid_param_default(void);
void flow_pid_param_move(void);
extern volatile uint8_t flow_pid_param_type;

#endif

