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
#include "scheduler.h"
#include "spl06.h"
#include "SPL06_cankao.h" //�µ�
#include "ANO_Data_Transfer.h"
#include "WIFI_UFO.h"
#include "flow.h"
#include "ADC.h"

loop_t loop; 
//u32 time[10],time_sum;
int32_t time[10],time_sum;
 
//2msִ��һ��
//��delay.c���ã�2msһ��
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
		loop.check_flag += 1;   //�ñ�־λ��ѭ��������0
	}
	
}



void main_loop()
{
	if( loop.check_flag >= 1 )
	{
		
		if( loop.cnt_2ms >= 1 )
		{
			loop.cnt_2ms = 0;
			
			Duty_2ms();	 					//����2ms������
		}
		if( loop.cnt_4ms >= 2 )
		{
			loop.cnt_4ms = 0;
			Duty_4ms();						//����4ms������
		}
		if( loop.cnt_6ms >= 3 )
		{
			loop.cnt_6ms = 0;
			Duty_6ms();						//����6ms������
		}
		if( loop.cnt_10ms >= 5 )
		{
			loop.cnt_10ms = 0;
			Duty_10ms();					//����10ms������
		} 
		if( loop.cnt_20ms >= 10 )
		{
			loop.cnt_20ms = 0;
			Duty_20ms();					//����20ms������
		}
		if( loop.cnt_50ms >= 25 )
		{
			loop.cnt_50ms = 0;
			Duty_50ms();					//����50ms������
		}
		if( loop.cnt_1000ms >= 500)
		{
			loop.cnt_1000ms = 0;
			Duty_1000ms();				//����1s������
		}
		loop.check_flag = 0;		//ѭ��������ϱ�־
	}
}

/////////////////////////////////////////////////////////
void Duty_2ms()
{
	time[0] = GetSysTime_us();
	
	MpuGetData();				          //��ȡ���������ݣ���ȡ�����ٶ�ֱ�ӻ���������̬���Ƶ�pid�ڻ�����ֵ��2msһ�Σ������⻷����ֵҪ���� GetAngle()������и��£�6msһ�Σ������ⶨ��ʱ��10msһ�Σ����˶����ٶ�GetAccz()Ҳ��Ҫ���� GetAngle()���㣨6msһ�Σ�
	FlightPidControl(0.002f);     //��̬����
	MotorControl();               //�������
	CloseLED(); 
	
	time[0] = GetSysTime_us() - time[0];
}

//////////////////////////////////////////////////////////
void Duty_4ms()
{
	time[1] = GetSysTime_us();
	
	ANO_NRF_Check_Event();    //ɨ�����2.4G�źţ��������������� Nrf_Erro=0
	//ANO_DT_Data_Exchange();		//��ת�Ƶ�Duty_20ms()��ͨ����Ƶ���ͷɿ����ݸ�ң����, ����Ƶ��Ҫ�죨4msһ�Σ������Ժ�ң�����ݣ���������Ƶ��Ҫ������Ȼ�ղ�ȫ
	
	//����ǽ���ң�������ݣ����������ݣ��������� ����RC_Analy();���������©��ң���������ݣ��ر����������������
	Rc_Connect();  						//1.0 ����ң��������,�ŵ� Remote������������������Nrf_Erro++
	
	Mode_Controler(0.004f); 	//1.1 ���������յ���Remote���ݣ����߶���ģʽ�����Flow_mode_two()��ȡ������̬��
	
	RC_Analy();	     					//1.2 ң��������ָ�����һ������ Remote�ڵ����ݣ����������ͽ�����������У׼
	
	time[1] = GetSysTime_us() - time[1];
}

//////////////////////////////////////////////////////////
void Duty_6ms()
{
	time[2] = GetSysTime_us(); 
	
	GetAngle(&MPU6050,&Angle,0.006f);   //�������������ݣ�������̬���ݣ�����Angle;;; ע�⣺z���˶����ٶȣ���������Ӱ�죩NormAcczҲ��������£�6msһ�Σ�, �����㷨HeightPidControl��Ҫ�õ����ֵ��10msһ�Σ���
	
	time[2] = GetSysTime_us() - time[2];
}

/////////////////////////////////////////////////////////
void Duty_10ms()
{
	time[3] = GetSysTime_us();

	//���Ҫ�Ƶ�4ms������Rc_Connect()ͬƵ�ʣ�
	//RC_Analy();	     					//1.2 ң��������ָ�����һ������ Remote�ڵ�����
	
	//�������ݲɼ����ڴ���1�Ļص����������� Flow_Receive1()��ִ��Ƶ���ɹ���ģ�����������
	Pixel_Flow_Fix(0.006f);  			//mini���������ںϴ��������Լ�����߶����ݽ����ڴ���1�ص������Զ�����
	Flow_Pos_Controler(0.006f);		//�����������	
	
	Height_Get(0.01f);			//��ȡ��ѹ�߶����ݣ���ѹԭʼ���ݻ�ȡ
	High_Data_Calc(10);			//��ѹ�߶������ںϣ�ˢ�� FlightData.High.bara_height�����ڴ��ڼ�������ʱ��4�ף�
	
	//Height_Get_New(0.01f);			//��ȡ��ѹ�߶����ݣ���ѹԭʼ���ݻ�ȡ
	
	
	HeightPidControl(0.006f); 		//ˢ��Control_high(�������ѹ), �Լ�����ģʽ�µĸ߶ȿ��Ƴ��򣬷Ƕ���ģʽ�˳�
	
	time[3] = GetSysTime_us() - time[3];
}

/////////////////////////////////////////////////////////
void Duty_20ms()
{
	time[4] = GetSysTime_us();

	ANTO_polling(); 	//ͨ���ɿش���3�������ݵ���λ��  //ͬʱҲ�����������λ����pid�������ɿأ����pid������һ���Ӵ��ڽ��ն�����Ҳ������ң��ת��
	ANO_DT_Data_Exchange();		//�������ݸ�ң����, ����Ƶ��Ҫ�죨4msһ�Σ������Ժ�ң�����ݣ���������Ƶ��Ҫ������Ȼ�ղ�ȫ��ң�ؽ���Ϊ10msһ��
	
	time[4] = GetSysTime_us() - time[4];
}

//////////////////////////////////////////////////////////
void Duty_50ms()
{
	time[5] = GetSysTime_us();
	
	PilotLED(); 						//LEDˢ��
	
	Flag_Check();   			 //������״̬��־
	
	Voltage_Check();			//�ɿص�ѹ���
	
	time[5] = GetSysTime_us() - time[5];
}

/////////////////////////////////////////////////////////////
void Duty_1000ms()
{
	u8 i;
  NRF_SSI = NRF_SSI_CNT;  //NRF�ź�ǿ��
	NRF_SSI_CNT = 0;
	
	WIFI_SSI = WIFI_SSI_CNT;//WiFi�ź�ǿ��
	WIFI_SSI_CNT = 0;
	
	Locat_SSI = Locat_SSI_CNT;//�Ӿ�λ������Ƶ��
	Locat_SSI_CNT = 0;
	
	Flow_SSI = Flow_SSI_CNT;  //��������Ƶ�� Flow_SSI_CNT�ڴ���1�ص��������������ݽ��գ�������
	Flow_SSI_CNT = 0;
	
		//����Ӿ���λģ���Ƿ����
	if(Locat_SSI>10)  Locat_Err = 0;
	else Locat_Mode=0,Locat_Err = 1;
	
	  //������ģ���Ƿ����
	if(Flow_SSI>10)  Flow_Err = 0;  //���������������յ�����ģ��10�����ݣ�����ʾ��������
	else 						 Flow_Err = 1;	//�����쳣����ʱ�� Mode_Controler()���Զ���Ϊ��̬ģʽ
	
	time_sum = 0;
	for(i=0;i<6;i++)	time_sum += time[i];
}



//////////////////////////end///////////////////////////////////////////