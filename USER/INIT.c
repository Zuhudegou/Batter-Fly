/**********************STM32 ��Դ���˻�*******************************************************/
//  V1.0 ��Դ���ߣ�С��&zin�����ڣ�2016.11.21
//           STM32F103C8�ɿ��Լ�ң�ػ��������Լ����Ĵ���ʵ�֣�
//  V2.0 ��Դ���ߣ�С�������ڣ�2020.05.17
//           scheduler����ܹ�������������Ļ�Լ���ѹ�ƣ�����PID���ߵ������ܣ�
//  V3.0 ��Դ���ߣ�zhibo_sz&sunsp�����ڣ�2024.06.01
//           ����һ��������ɣ���ͣ�˶������Լ�ɲ���Ż�����ѹң����Ļ�����ǵ�ģ���Ż���������ˢ�����
/********************************************************************************************/

//������
//      ��������Թ����û���Դ��ѧϰʹ�ã�����Ȩ�������������У�
//      δ����ɣ����ô��ġ�ת�ء�������ת�������롣


#include "ALL_DEFINE.h" 
#include "ALL_DATA.h"   //���������ⷢ��
#include "spl06.h" 
#include "SPL06_cankao.h" //�µ�
#include "LED.h"
#include "Uart1.h"
#include "USART2.h"
#include "ANO_Data_Transfer.h"
#include "ADC.h"

volatile uint32_t SysTick_count; //ϵͳʱ�����
volatile uint8_t spl_flag; //ϵͳʱ�����
_st_Mpu MPU6050;   //MPU6050ԭʼ����
_st_Mag AK8975;   
_st_AngE Angle;    //��ǰ�Ƕ���ֵ̬
_st_Remote Remote; //ң��ͨ��ֵ

//���ﶨ���ֵ����Ҫ��  all_data.h  ����

//����
struct _Aux_Rc Aux_Rc; //��ǰҡ��ֵ����

volatile uint32_t ST_CpuID;
 
 
_st_ALL_flag ALL_flag; //ϵͳ��־λ������������־λ��



 _st_FlightData FlightData;
 //�ɿ�����
st_Command Command;

PidObject pidRateX; //�ڻ�PID����
PidObject pidRateY;
PidObject pidRateZ;

PidObject pidPitch; //�⻷PID����
PidObject pidRoll;
PidObject pidYaw;

PidObject pidHeightRate; //�߶��ڻ�pid
PidObject pidHeightHigh; //�߶��⻷pid



PidObject pidHeightThr; //����ƽ�����Ŷ�̬����
PidObject pidOffsetRoll; //������Ư��̬����



PidObject Flow_PosPid_x;    //�⻷����
PidObject Flow_PosPid_y;

PidObject Flow_SpeedPid_x;  //�ڻ�����
PidObject Flow_SpeedPid_y;

_st_IMU IMU;

void pid_param_Init(void); //PID���Ʋ�����ʼ������дPID�����ᱣ�����ݣ��������ɺ�ֱ���ڳ�������� ����¼���ɿ�


//��ȡCPU��ID
void GetLockCode(void)
{
	ST_CpuID = *(vu32*)(0x1ffff7e8);//���ֽ�оƬID������ͨѶ��Ƶͨ��
}

u8 count1=0;

///////////////ȫ����ʼ��//////////////////////////////////
void ALL_Init(void)
{
	float STBy;
	
	//�Ƶ���ǰ���ʼ������Ϊ��ʱ��ʼ��֮ǰ���и�����ѹ1v���ҵ��µ��ת��, ���������СΪ4.7k
	//TIM2_PWM_Config();			//2·PWM��ʼ��		
	//TIM3_PWM_Config();			//2·PWM��ʼ��		
	

	
	//while(1);


	IIC_Init();             //I2C��ʼ��
	count1=1;
		
	pid_param_Init();       //PID������ʼ��
	count1=2;
	  
		
	
	
	LEDInit();              //LED���Ƴ�ʼ��
	count1=3;




	MpuInit();              //MPU6050��ʼ������ʼ����ʼ���������������
	count1=4;


 


		//ADC��ʼ��
	ADC1_Init();
	count1=5;
	
	ANO_Uart1_Init(19200);   //�ӹ���ģ��
	//ANO_Uart1_Init(115200);   //�ӹ���ģ��
	count1=6;
	
//	printf("ANO_Uart1_Init  \r\n")	
	//if (FLY_TYPE == 2) 
	//{UART2_Init(115200); }      //
	
	USART3_Config(500000);        //��λ�����ڳ�ʼ��
//	printf("USART3_Config  \r\n");
	count1=7;
	
		


	NRF24L01_init();				//2.4Gң��ͨ�ų�ʼ��,, ��ʼ����ɺ���������������൱��AlternateFlash
	count1=8;
	
	//���ƽ����ʾ������err����ƽ����ʾ��ѹ��err
	if(MPU_Err){ //�������쳣
			fLED_H(); //���Ͻ�����Ϩ�𣬱�ʾ������ʧ�ܣ����Ͻ����������ģ������ƽ�����˸��������ͬ����˸

	}

	
	//if(SPL_init() >=1)	 //��ѹ�Ƴ�ʼ��, 0��ʾ SUCCESS
	if(spl0601_init() >=1)	 //��ѹ�Ƴ�ʼ��, 0��ʾ SUCCESS
	{//ʧ�ܣ���ɫֻ��һ����
		  while(1)
			{ 
				STBy++;
				//GPIOB->BSRR = GPIO_Pin_9;  //��һ��LED
				GPIOB->BRR =bLED_io;//���������½Ǻ�ƣ����º�ƽ��棬���½���rf��ʼ��ʱ�Ѿ��ر�
				
				if(STBy>=100000)//ʧ��
				{
						spl_flag=0; 
						SPL_Err = 1;
					  break;
				}	
			}
	}
	
	
	//��ѹ�Ƴ�ʼ���ɹ�������2.4G�ɹ����������������
	else 
	{
			spl_flag=1;
		  SPL_Err = 0;
		
		
		
	}
	
	count1=9;
	
	
	
	//�������Ʋ�ͬʱ������ʾ�������쳣��������û������ң�ء�
	//������Ʋ�ͬʱ������ʾ��ѹ���쳣��������û������ң�ء�


	//while(1);
	
	//�Ƶ���ǰ���ʼ������Ϊ��ʱ��ʼ��֮ǰ���и�����ѹ1v���ҵ��µ��ת��
	TIM2_PWM_Config();			//2·PWM��ʼ��		
	TIM3_PWM_Config();			//2·PWM��ʼ��		
	delay_ms(10); //10ms
	
	//while(1);

	
//��󣬿��ı�����켸��	
#if ( FLY_TYPE !=3 ) //����ˢ�������ˢ���������Ӧ�ɵ������

	//#define PWM0 TIM2->CCR1   //��ʱ��2 ͨ��1    M1
	//#define PWM1 TIM2->CCR2		//��ʱ��2 ͨ��2		M2
	//#define PWM2 TIM3->CCR1   //��ʱ��3 ͨ��1		M3
	//#define PWM3 TIM3->CCR2		//��ʱ��3 ͨ��2		M4

	TIM_Cmd(TIM2,DISABLE);	//����Ƶ�ʣ�Ϊ800Hz
	TIM_PrescalerConfig(TIM2, SystemCoreClock/(TIM2_PWM_MAX+1)/800 -1 ,TIM_PSCReloadMode_Immediate);
	TIM_Cmd(TIM2,ENABLE);
	
	TIM_Cmd(TIM3,DISABLE);	//
	TIM_PrescalerConfig(TIM3, SystemCoreClock/(TIM3_PWM_MAX+1)/800 -1 ,TIM_PSCReloadMode_Immediate);
	TIM_Cmd(TIM3,ENABLE);
	
	for(u8 i=0; i<3; i++){
		//����ռ�ձ�1.3%��13/1000����̫���˻�ת������ˢ��ͬ����ˢ���������ȫ��������������쵫��ת��
		TIM2->CCR1=13;	TIM2->CCR2=13; TIM3->CCR1=13; TIM3->CCR2=13; 
		delay_ms(30); //��30ms
		
		//�ָ�ռ�ձ�Ϊ0
		TIM2->CCR1=0;	TIM2->CCR2=0; TIM3->CCR1=0; TIM3->CCR2=0; 
		delay_ms(120); //��ͣ0.12s
	}

		
	TIM_Cmd(TIM2,DISABLE);	//�ָ�����Ƶ��
	TIM_PrescalerConfig(TIM2, SystemCoreClock/(TIM2_PWM_MAX+1)/TIM2_PWM_HZ -1 ,TIM_PSCReloadMode_Immediate);
	TIM_Cmd(TIM2,ENABLE);
	
	TIM_Cmd(TIM3,DISABLE);	//
	TIM_PrescalerConfig(TIM3, SystemCoreClock/(TIM3_PWM_MAX+1)/TIM3_PWM_HZ -1 ,TIM_PSCReloadMode_Immediate);
	TIM_Cmd(TIM3,ENABLE);
	
#endif
	
	
}


volatile u8 flow_pid_param_type=0;


//����pid����
void pid_param_Init(void)
{
	
	/////////////////////////////  1. ��̬�� pid  ////////////////////////////////////
	
	
//////////////////1.1 �ڻ��ٶ�PID///////////////////////	
	
	#if (FLY_TYPE == 1 || FLY_TYPE == 2) //���ı�
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
	
	
	
	#else 		//3 ����ˢ330   //��ˢǿ�������ȷ�ӦѸ�٣�pid��ҪСһ�㣬̫�����׶�
	
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
	
	
	
/////////////1.2 �⻷�Ƕ�PID///////////////////////////
	
	pidPitch.kp = 7.0f;
	pidRoll.kp = 7.0f;
	pidYaw.kp = 7.0f;	
	
	pidPitch.ki = 0.0f;
	pidRoll.ki = 0.0f;
	pidYaw.ki = 0.0f;	
	
	pidPitch.kd = 0.0f;
	pidRoll.kd = 0.0f;
	pidYaw.kd = 0.0f;	
	
	
	
	
	/////////////////////////////  2. ����  ////////////////////////////////////
	
	
	
	//////////////////2.1 �߶� �ڻ� PID ����///////////////////////	
	
#if (FLY_TYPE == 1 || FLY_TYPE == 2) //���ı�
	pidHeightRate.kp = 1.2f; //1.2f
	pidHeightRate.ki = 0.04f;
	pidHeightRate.kd = 0.085f;
#else 		//3 ����ˢ330
	pidHeightRate.kp = 1.0f; //1.2f
	pidHeightRate.ki = 0.04f;
	pidHeightRate.kd = 0.06f;
#endif


	//////////////////2.2 �߶� �⻷PID����///////////////////////	
	pidHeightHigh.kp = 1.2f;//1.2f
	pidHeightHigh.ki = 0.00f;
	pidHeightHigh.kd = 0.085f;//0.085f



	/*
		//�߶� �ڻ�PID���� �ٶ�
	pidHeightRate.kp = 1.2f; //1.2f
	pidHeightRate.ki = 0.04f;
	pidHeightRate.kd = 0.2f;

		//�߶� �⻷PID����
	pidHeightHigh.kp = 0.6f;//1.2f
	pidHeightHigh.ki = 0.00f;
	pidHeightHigh.kd = 0.1f;//0.085f
	*/

	//��������pid����
	
	pidHeightThr.kp =1.0f;
	pidHeightThr.ki =0.0f;
	pidHeightThr.kd =0.1f;
	
	
	
	
	
	///////////////////////////////  3. ����  ////////////////////////////////////
	
#if (FLY_TYPE == 1 || FLY_TYPE == 2) //���ı�
	
//////////////////////////3.1.1���� x�ڻ�///////////////////////////////////////////

	//X�ڻ�����PID����  �ٶ�
	//Flow_SpeedPid_x.kp = 0.610f;//����  0.600f
	//Flow_SpeedPid_x.ki = 0.000f;//����
	//Flow_SpeedPid_x.kd = 0.400f;//΢��
	
	//X�ڻ������ٶ�PID����  �ٶ�
	//Flow_SpeedPid_x.kp = 0.610f;//����  0.600f
	Flow_SpeedPid_x.kp = 0.500f;//����  0.600f
	Flow_SpeedPid_x.ki = 0.000f;//����
	Flow_SpeedPid_x.kd = 0.400f;//΢��
	

	////////////////////////3.1.2���� y�ڻ�//////////////////////////////////
	
	//Y�ڻ�����PID���� �ٶ�
	//Flow_SpeedPid_y.kp = 0.610f;//����
	//Flow_SpeedPid_y.ki = 0.000f;//����
	//Flow_SpeedPid_y.kd = 0.400f;//΢��
	
	//Y�ڻ������ٶ�PID���� �ٶ�
	//Flow_SpeedPid_y.kp = 0.610f;//����
	Flow_SpeedPid_y.kp = 0.500f;//����
	Flow_SpeedPid_y.ki = 0.000f;//����
	Flow_SpeedPid_y.kd = 0.400f;//΢��
	
	
	
	/////////////////////////3.2.1 ���� x�⻷/////////////////////////////////

	//X�⻷����λ��PID����  λ��
	//Flow_PosPid_x.kp = 2.200f;//����  2.000f
	Flow_PosPid_x.kp = 2.000f;//����  2.000f
	Flow_PosPid_x.ki = 0.000f;//����
	Flow_PosPid_x.kd = 0.006f;//΢��
	
	/////////////////////////3.2.2 ���� y�⻷/////////////////////////////////
	
	//Y�⻷����λ��PID���� λ�� 
	//Flow_PosPid_y.kp = 2.200f;//����
	Flow_PosPid_y.kp = 2.000f;//����
	Flow_PosPid_y.ki = 0.00f;//����
	Flow_PosPid_y.kd = 0.006f;//΢��

///////////////////////////////////////////////////////////////////
	


#else 		//3 ����ˢ330


/*
//////////////////////////���� �ڻ�///////////////////////////////////////////

	//X�ڻ������ٶ�PID���� �ٶ�
	Flow_SpeedPid_x.kp = 0.410f;//����  0.600f
	Flow_SpeedPid_x.ki = 0.000f;//����
	Flow_SpeedPid_x.kd = 0.300f;//΢��
	//Y�ڻ������ٶ�PID���� �ٶ�
	Flow_SpeedPid_y.kp = 0.410f;//����
	Flow_SpeedPid_y.ki = 0.000f;//����
	Flow_SpeedPid_y.kd = 0.300f;//΢��
	
/////////////////////////���� �⻷/////////////////////////////////

	//X�⻷����λ��PID���� λ��
	Flow_PosPid_x.kp = 2.200f;//����  2.000f
	Flow_PosPid_x.ki = 0.000f;//����
	Flow_PosPid_x.kd = 0.006f;//΢��
	//Y�⻷����λ��PID���� λ�� 
	Flow_PosPid_y.kp = 2.200f;//����
	Flow_PosPid_y.ki = 0.00f;//����
	Flow_PosPid_y.kd = 0.006f;//΢��

///////////////////////////////////////////////////////////////////
*/

	flow_pid_param_default(); //��ˢ�������pid��������ͣʱ��pid������С


#endif

	Command.FlightMode = NORMOL;  //��ʼ��Ϊ��̬����ģʽ
}


/*



#if ( FLY_TYPE==3 ) //��ˢ ����pidĬ�ϲ������ֶ�������ɻ�һ�����������ͣ��
  //�˳�ģʽ3�����߷���ҡ�˶�����������������Ĭ��pid����
	if(flow_pid_param_type!=1) flow_pid_param_default(); //��ˢ�������pid��������ͣʱ��pid������С

#endif


#if ( FLY_TYPE==3 ) //��ˢ ����pid�˶�������һ��������ɻ���ҡ���ƶ���
  //����ģʽ3�����߼�⵽����ҡ�˶��������������˶�pid����
	if(flow_pid_param_type!=2) flow_pid_param_move();//��ˢ�������pid����ɺ��ֶ��ƶ�ʱ��pid�����ϴ�

#endif

*/



//��ˢ�������pid��������ͣ��ƽʱ��pid������С��׷��ƽ�ȣ���ֹ������PID����������ң��������λ����ʵʱ�鿴
void flow_pid_param_default(void){
	
		//if(flow_pid_param_type!=0) return;  //��һ����Է�ֹ���й���pid�������Զ����ģ�����ʹ����λ���޸�pid����
	
		flow_pid_param_type=1;
	
#if (FLY_TYPE ==3) //��ˢ
		/*
		//X�ڻ������ٶ�PID����  �ٶ�
		Flow_SpeedPid_x.kp = 0.400f;//����  0.380f
		Flow_SpeedPid_x.ki = 0.000f;//����
		Flow_SpeedPid_x.kd = 0.300f;//΢��  0.300f
		//Y�ڻ������ٶ�PID���� �ٶ�
		Flow_SpeedPid_y.kp = Flow_SpeedPid_x.kp;//����
		Flow_SpeedPid_y.ki = Flow_SpeedPid_x.ki;//����
		Flow_SpeedPid_y.kd = Flow_SpeedPid_x.kd;//΢��
		
		//X�⻷����λ��PID����  λ��
		Flow_PosPid_x.kp = 1.900f;//����  1.700f
		Flow_PosPid_x.ki = 0.000f;//����
		Flow_PosPid_x.kd = 0.006f;//΢��  0.006f;
		//Y�⻷����λ��PID���� λ�� 
		Flow_PosPid_y.kp = Flow_PosPid_x.kp;//����
		Flow_PosPid_y.ki = Flow_PosPid_x.ki;//����
		Flow_PosPid_y.kd = Flow_PosPid_x.kd;//΢��
		*/
		
		
		
		
		//X�ڻ������ٶ�PID����  �ٶ�
		Flow_SpeedPid_x.kp = 0.410f;//����  0.600f
		Flow_SpeedPid_x.ki = 0.000f;//����
		Flow_SpeedPid_x.kd = 0.300f;//΢��
		//Y�ڻ������ٶ�PID���� �ٶ�
		Flow_SpeedPid_y.kp = Flow_SpeedPid_x.kp;//����
		Flow_SpeedPid_y.ki = Flow_SpeedPid_x.ki;//����
		Flow_SpeedPid_y.kd = Flow_SpeedPid_x.kd;//΢��
		
		//X�⻷����λ��PID����  λ��
		Flow_PosPid_x.kp = 2.000f;//����  2.000f
		Flow_PosPid_x.ki = 0.000f;//����
		Flow_PosPid_x.kd = 0.006f;//΢��
		//Y�⻷����λ��PID���� λ�� 
		Flow_PosPid_y.kp = Flow_PosPid_x.kp;//����
		Flow_PosPid_y.ki = Flow_PosPid_x.ki;//����
		Flow_PosPid_y.kd = Flow_PosPid_x.kd;//΢��
	
		
		
		
		
		

	
#endif
}

//��ˢ�������pid����ɻ��߷���ҡ�˿����ƶ�ʱ��pid�����ϴ�׷�������Ӧ����ֵ��PID����������ң��������λ����ʵʱ�鿴
void flow_pid_param_move(void){
	
	//if(flow_pid_param_type!=0) return;  //��һ����Է�ֹ���й���pid�������Զ����ģ�����ʹ����λ���޸�pid����
	
	flow_pid_param_type=2;
	
#if (FLY_TYPE ==3) //��ˢ

		//X�ڻ������ٶ�PID����  �ٶ�
		Flow_SpeedPid_x.kp = 0.410f;//����  0.600f
		Flow_SpeedPid_x.ki = 0.000f;//����
		Flow_SpeedPid_x.kd = 0.300f;//΢��
		//Y�ڻ������ٶ�PID���� �ٶ�
		Flow_SpeedPid_y.kp = Flow_SpeedPid_x.kp;//����
		Flow_SpeedPid_y.ki = Flow_SpeedPid_x.ki;//����
		Flow_SpeedPid_y.kd = Flow_SpeedPid_x.kd;//΢��
		
		//X�⻷����λ��PID����  λ��
		Flow_PosPid_x.kp = 2.000f;//����  2.000f
		Flow_PosPid_x.ki = 0.000f;//����
		Flow_PosPid_x.kd = 0.006f;//΢��
		//Y�⻷����λ��PID���� λ�� 
		Flow_PosPid_y.kp = Flow_PosPid_x.kp;//����
		Flow_PosPid_y.ki = Flow_PosPid_x.ki;//����
		Flow_PosPid_y.kd = Flow_PosPid_x.kd;//΢��
	
#endif
}





