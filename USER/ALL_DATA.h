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
	
	//����˳���ܸ�����ɾ��pMpu�����MPU6050����
	
	int16_t gyroZ_raw; //û�о���������ֵ���������Աȶ�
	float gyroZ_offset; //һ��Ҫfloat������yaw���ٶ�ƫ�ƣ����ƫ���޷�ͨ��MUP6050У׼������
	float gyroZ_offset1; //�����У�ƫ���ǲ���ƫ��=�û����õ���Ư������ ң�����÷��� SET+P ���� SET+M
	uint16_t gyroZ_cnt;	//��̬���������ۼƣ���ʱgyroZ_offset1=0��������,����gyroZ_offset1��

	int16_t gyroZ_sum; //û�о���������ֵ���������Աȶ�
	uint16_t gyroZ_sum_cnt;//�������ֵ
	
	//���������1s�ڵĻ����ٶ�
	//float speedX; //MPU�ṩ��ˮƽ����Ĳ����ٶȲ��޲ο�����
	//float speedY; //MPU�ṩ��ˮƽ����Ĳ����ٶȲ��޲ο�����
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
//		 struct{  //�Ƕ����� 
//			float roll;
//			float pitch;
//			float yaw;
//		}Angle;
		 struct{  //�߶�����
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
	
	//uint16_t raw_data1; //����
	//uint16_t raw_data2; //����
	
	uint16_t AUX1;
	uint16_t AUX2;
	uint16_t AUX3;
	uint16_t AUX4;	
	uint16_t AUX5;
	uint16_t AUX6;
	
	//����
	uint16_t AUX7;
	uint16_t AUX8;
	
	uint8_t new_data; //����������
	uint8_t rst_count; //ʧ������
	
	uint8_t test_motor; //��ǰ��ƽ����Ե����ţ�1~4��
	
}_st_Remote;



//����
struct _Aux_Rc//��ǰҡ��ֵ����
{
	//int8_t position;	//��ͣģʽ�������� ALL_flag.height_lock һ�������Ǵ�����ͣģʽ
	uint8_t headless;			//��ͷģʽ
	uint8_t thr_mask; 			//��������
	uint8_t fly_key2;			//һ���������
	
	uint8_t setMpu;				//У׼�ж�
	uint8_t testMotor;		//�������
	float yaw_stick; //ҡ�˿��ƵĽǶȣ�����ƫ����=ҡ�˿��ƽǶ�+�������ƽǶ�
	
	int8_t lidu;
	int8_t move_fx_z;
	double thr_ratio;
	int16_t move_thr; 	//��ͣģʽ�£�û��������ʱ����

	int16_t offse_roll; 	//��Ư���� ��Ϊ˫�ֽ� +/-256
	int16_t offse_pitch;
	
	int8_t offse_yaw; //ƫ����̬������ֱ�������ڻ������Ư��ԭ��z���ٶȻ��ֳ�yawʱ��С���ݱ��������Ŷ����ˣ�ֱ�������������� gyroZ_offset1
	

	//float offset_roll_adjust; 	//��̬��Ư����
	//float offset_pitch_adjust; //����Ϊ���㣬��Ȼ�޷���Ӧÿ��΢С�仯��
	
	
	int16_t thr_raw;		//������ҡ��ֵ���������ű�������ʱ����ֵ
	int16_t roll_raw;				//������ҡ��ֵ
	int16_t pitch_raw;		//������ҡ��ֵ
	
  //������ Remote �е�һ��
	int16_t thr;				//����ģʽ���а������ű���������ͣģʽ���Ų�����ʱ����ֵ
	int16_t yaw;
	int16_t roll; 			//��ͣģʽ�£������ȣ���������Ư����ֵ
	int16_t pitch;			//��̬ģʽ�£������ȣ�Ҳ������Ư����ֵ
	
	
	int16_t debug1,debug2,debug3,debug4,debug5,debug6,debug7,debug8;//����ң�صĵ������ݣ��������� 
	
};







typedef volatile struct
{
	float desired;     ///����
	float offset;      //ԭ����������ҡ����λ���µ�����������û���õ���
	float prevError;    // �ϴ�ƫ��
	float integ;        //�������ۼ�ֵ
	float kp;           //p����
	float ki;           //i����
	float kd;           //d����
	float IntegLimitHigh;       //< integral limit
	float IntegLimitLow;
	float measured;     ////pid������
	float out;
	float OutLimitHigh;
	float OutLimitLow;	
	float Control_OutPut;//�����������
	float Last_Control_OutPut;//�ϴο����������
	float Control_OutPut_Limit;//����޷�
		/***************************************/
	float Last_FeedBack;//�ϴη���ֵ
	float Dis_Err;//΢����
	float Dis_Error_History[5];//��ʷ΢����
	float Err_LPF;
	float Last_Err_LPF;
	float Dis_Err_LPF;

//	int8_t Err_Limit_Flag :1;//ƫ���޷���־
//	int8_t Integrate_Limit_Flag :1;//�����޷���־
//	int8_t Integrate_Separation_Flag :1;//���ַ����־		
  Butter_BufferData Control_Device_LPF_Buffer;//��������ͨ�����������	
}PidObject;


typedef volatile struct
{
	uint8_t 	unlock;					//�ѽ���
	uint32_t  slock_flag;
	uint8_t 	height_lock:1;  //��ͣģʽ ����+����
	uint8_t 	height_lock_qiya:1;  //��ѹ����ģʽ������ʧЧʱ
	uint8_t 	take_off:1;
	uint8_t 	take_down:1;

}_st_ALL_flag;



typedef volatile struct
{
	uint8_t AccOffset :1;  //У׼����
	uint8_t GyroOffset :1;
	uint8_t MagOffset :1;
	uint8_t six_acc_offset; //����У׼
	enum{ 								 //����ģʽ�л�����
			LOCK = 0x00,				//����ģʽ
			NORMOL, 		//����ģʽ		
			HEIGHT,			//����ģʽ
			Flow_POSITION,   //GPS����ģʽ
	}FlightMode; //����ģʽ
}st_Command;



extern struct _Aux_Rc Aux_Rc; //��ǰҡ��ֵ����

extern _st_Remote Remote;
extern _st_Mpu MPU6050;
extern _st_Mag AK8975; //����������Ӵ�����
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

extern	PidObject pidHeightThr; //����ƽ�����Ŷ�̬����
extern	PidObject pidOffsetRoll; //������Ư��̬����

extern PidObject Flow_PosPid_x; 
extern PidObject Flow_PosPid_y;

extern PidObject Flow_SpeedPid_x;
extern PidObject Flow_SpeedPid_y;

extern _st_FlightData FlightData;
//�ɿ�����
extern st_Command Command;



void GetLockCode(void);

//����pid���� ��int.c
void pid_param_Init(void);
void flow_pid_param_default(void);//��ˢ�������pid������������ͣʱ��pid������С��׷��ƽ�ȣ���ֹ������
void flow_pid_param_move(void);//��ˢ�������pid��������ɺ��ֶ��ƶ�ʱ��pid�����ϴ�׷�������Ӧ����ֵ��
extern volatile uint8_t flow_pid_param_type;//��ǰ��ˢ�������pid��������

#endif

