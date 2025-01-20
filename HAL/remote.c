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
	//�����������ȼ�
	//1��ң����  2��WiFiͼ��ģ�� 
	if(NRF_Connect()==0)  //��Ƶ���ղ���������
	{
		if(WIFI_UFO_Connect()==0)//wifi���ղ���������
		{
			
		}
	}
}





/*****************************************************************************************
 *  ͨ�����ݴ���
 * @param[in] 
 * @param[out] 
 * @return     
 ******************************************************************************************/	
uint8_t RC_rxData[32];
void remote_unlock(void);	
void mpu_adjust(void);
void motor_test(void);

//4msһ��
void RC_Analy(void)  
{
		static uint16_t cnt,cnt_tmp;
/*             Receive  and check RC data                               */	
//	static u8 Connect_flag;
	 ///ģʽѡ����г���
	
	remote_unlock(); // �����ж�,�Ƶ����棬ң���������ݶ�Ҫ���������
		
	//�����������ȼ�
	if(Nrf_Erro==1 || WIFI_UFO_Err==1) //Nrf_ErroΪ1����ʾ2.4G�Ѿ����յ�����
	{ 	
				//������
				cnt=0;//�������յ������ˣ��������Ҫ���㣬��ֹRemote���ݱ���λ
				//if(Remote.thr!=0)cnt=0;//ȷʵ�յ������ˣ�ʧ����thr���Զ���ֵ
				//if(Remote.AUX1!=0)cnt=0;//ȷʵ�յ������ˣ�Ӧ����1000~2000��������0
				//����������ж������յ�����
		
				//������ ANO_DT.c�н��գ������� �ṹ�� Remote


				//��control.c �� Mode_Controler()��������ҡ��ֵ�Ľ�һ���������õ�Ŀ����̬�Ƕȡ�
		
				{
//							const float roll_pitch_ratio = 0.04f;
								const float yaw_ratio =  0.0015f;    
					
//							pidPitch.desired =-(Remote.pitch-1500)*roll_pitch_ratio;	 //��ң��ֵ��Ϊ���нǶȵ�����ֵ
//							pidRoll.desired = -(Remote.roll-1500)*roll_pitch_ratio;
					
//							if((Flow_Err ==0) && (mini.ok == 1) && (FlightData.High.bara_height < 40))//����ģ����λ  ������Ч   //ֱ�ӿ����ٶ� ���ҹص��⻷����
//							{
//								Flow_SpeedPid_y.desired = -(Remote.pitch-1500)*roll_pitch_ratio;   //ҡ�˿����ٶ�����ֵ 
//								Flow_SpeedPid_x.desired = -(Remote.roll-1500) *roll_pitch_ratio;
//								pidPitch.desired = -(Remote.pitch-1500)*0.02f;  //ң����ҡ�˿�����̬�⻷����ֵ
//								pidRoll.desired  = -(Remote.roll-1500) *0.02f;  //ң����ҡ�˿�����̬�⻷����ֵ 								
//							}
//							else
//							{
//								pidPitch.desired = -(Remote.pitch-1500)*roll_pitch_ratio;
//								pidRoll.desired  = -(Remote.roll-1500) *roll_pitch_ratio;  
//							}
//					
					
					
					
							// ע�⣺ pidPitch.desired �� pidRoll.desired  �� control.c �� Mode_Controler()
							//ƫ���� ҡ��ֵ��pid�����ǶȻ��㣬Ҳת�Ƶ�  control.c �� Mode_Controler()
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
				
				//remote_unlock(); // �����ж�,�Ƶ����棬ң���������ݶ�Ҫ���������
				
				//��������
				mpu_adjust();//������У׼�ж�
				motor_test();//�������ж�,,,����4�ν�������
				
				
				if(AUX86_isH()) LED.off=1; //�ر�LED
				else LED.off=0;//LED����
				
  }
//���3��û�յ�ң�����ݣ����ж�ң���źŶ�ʧ���ɿ����κ�ʱ��ֹͣ���У��������ˡ�
//���������ʹ���߿ɽ����ر�ң�ص�Դ������������3��������رգ��������ˡ�
//�����ر�ң�أ�����ڷ����л�ֱ�ӵ��䣬���ܻ��𻵷�������
  else
	{					
				cnt++;
				if(cnt>500)						//2��û��ң�����ź� �ж�ң����ʧ�� �źŶ��� �Զ��½�����
				{	
					Remote.rst_count++;//��¼ʧ�����������û�п�ң�أ�ÿ4���¼������һ�Ρ�
					
					//1499��1500��1501��ʾͨѶ����
					Remote.roll = 1499;  //ͨ��1    ���ݹ���
					LIMIT(Remote.roll,1000,2000);
					Remote.pitch = 1500;  //ͨ��2		���ݹ���
					LIMIT(Remote.pitch,1000,2000);
					Remote.yaw =  1501;   //ͨ��4		���ݹ���
					LIMIT(Remote.yaw,1000,2000);		
					
					
					//����rfģ��
					//if(Remote.thr < 1030)						//�ж�����
					if( Remote.thr < 1030 || !ALL_flag.unlock )// ���ڷ���״̬������������rfģ�飬��Ҳ����֮��ɽ�����˸
					{
							cnt = 0;
							Remote.thr =1000;						//�ر�����
							ALL_flag.unlock = 0; 				//�˳�����
							//LED.status = AllFlashLight; //��ʼ����,��ʾʧ��,��2.4G��λʱ������ĵƵ�״̬
							NRF24L01_init();						//��λһ��2.4Gģ�飬��λʱ���Ϊ����	
					}
					//�𽥹ر�����
					else
					{	
						cnt = 810;
						if(cnt_tmp++>100)                 //�������ż�С��ʱ��
						{
							cnt_tmp=0;
//							printf("Remote.thr: %d  \r\n",Remote.thr);				//����1�Ĵ�ӡ			
							Remote.thr = 	Remote.thr-20;   //ͨ��3 ����ͨ����ԭ���Ļ������Զ�������С  �𵽷ɻ������½�
						}				
					}
					LIMIT(Remote.thr,1000,2000);		
					
					
				} 
	}	
}


void mpu_adjust(void)//������У׼�ж�
{
	
	//У׼����������
	if(AUX811_isH()){
			if(Aux_Rc.setMpu==0){ //��û��ʼУ׼
						Aux_Rc.setMpu=1; //��ʼУ׼
						//У׼MPU6050��ˮƽ�Լ���ֹУ׼
				
						//���
						bLED_H();	 	//ǰ����
						aLED_H();		//ǰ����
						fLED_H();		//�����
						hLED_H();		//�����
				
						IWDG_ReloadCounter();//ι��	
						//�и���mcu����0.5��ͻ�����
						delay_ms(500); //Ϩ��1��
						IWDG_ReloadCounter();//ι��	
						//delay_ms(500); //Ϩ��1��
						//IWDG_ReloadCounter();//ι��	
				
						MpuGetOffset_save(); //У׼�����ǲ���������
						IWDG_ReloadCounter();//ι��	
				
						//delay_ms(100);
						//�ɹ�����˸
						for(u8 i=0; i<10; i++){
									//����
									bLED_L();	 	//ǰ����
									aLED_L();		//ǰ����
									fLED_L();		//�����
									hLED_L();		//�����
							
									IWDG_ReloadCounter();//ι��	
									delay_ms(30); //��1��
							
									bLED_H();	 	//ǰ����
									aLED_H();		//ǰ����
									fLED_H();		//�����
									hLED_H();		//�����
							
									IWDG_ReloadCounter();//ι��	
									delay_ms(30); //��1��
						}
						
							
			}
	
	
	}
	else{
				Aux_Rc.setMpu=0; //�Ѿ��ɿ�
	
	}

}





void motor_test(void)//����Զ������ж�,,,����3�ν�������
{
	
	//���԰���������
	if(AUX822_isH()){
			if(Aux_Rc.testMotor==0){ //��û��ʼУ׼
						Aux_Rc.testMotor=1; //��ʼУ׼
						//У׼MPU6050��ˮƽ�Լ���ֹУ׼

						u8 motor=100; //���������ȣ�0~1000
		
						//���
						bLED_H();	 	//ǰ����
						aLED_H();		//ǰ����
						fLED_H();		//�����
						hLED_H();		//�����
		
						//PWM0 = 0;
						//PWM1 = 0;
						//PWM2 = 0;
						//PWM3 = 0;
						PWM0 = TIM2_DUTY + 0;
						PWM1 = TIM2_DUTY + 0;
						PWM2 = TIM3_DUTY + 0;
						PWM3 = TIM3_DUTY + 0;

						//Ĭ��ռ�ձȣ����ı�Ϊ0����ˢΪ1000��
						//TIM2_DUTY, TIM3_DUTY	

						bLED_L(); //1���죬����
						IWDG_ReloadCounter();//ι��	
						PWM0 = TIM2_DUTY + motor; //ת��
		
						delay_ms(500); //��������0.5s
						IWDG_ReloadCounter();//ι��	
						PWM0 = 0; //��ˢ����
						
						delay_ms(500); //��ͣ0.5s
						IWDG_ReloadCounter();//ι��	
						
						delay_ms(500); //��ͣ0.5s
						IWDG_ReloadCounter();//ι��	
						PWM0 = TIM2_DUTY + 0; //��������ˢ�죬����
						bLED_H(); //1���죬����

		
		

						fLED_L(); //2����������
						IWDG_ReloadCounter();//ι��	
						PWM1 = TIM2_DUTY + motor;
						
						delay_ms(500); //��������1ms
						IWDG_ReloadCounter();//ι��	
						PWM1 = 0;
						
						delay_ms(500); //��ͣ1ms
						IWDG_ReloadCounter();//ι��	
						
						delay_ms(500); //��ͣ1ms
						IWDG_ReloadCounter();//ι��	
						PWM1 = TIM2_DUTY + 0;
						fLED_H(); //2����������

		
		

						hLED_L(); //3����������
						IWDG_ReloadCounter();//ι��	
						PWM2 = TIM3_DUTY + motor;
						
						delay_ms(500); //��������1ms
						IWDG_ReloadCounter();//ι��	
						PWM2 = 0;
						
						delay_ms(500); //��ͣ1ms
						IWDG_ReloadCounter();//ι��	
						
						delay_ms(500); //��ͣ1ms
						IWDG_ReloadCounter();//ι��	
						PWM2 = TIM3_DUTY + 0;
						hLED_H(); //3����������

		
		

						aLED_L(); //4���죬����
						IWDG_ReloadCounter();//ι��	
						PWM3 = TIM3_DUTY + motor;
						
						delay_ms(500); //��������1ms
						IWDG_ReloadCounter();//ι��	
						PWM3 = 0;
						
						delay_ms(500); //��ͣ1ms
						IWDG_ReloadCounter();//ι��	
						
						delay_ms(500); //��ͣ1ms
						IWDG_ReloadCounter();//ι��	
						PWM3 = TIM3_DUTY + 0;
						aLED_H(); //4���죬����
						
						

	
	

			}
			
			
			

	}
	else{
				Aux_Rc.testMotor=0; //�Ѿ��ɿ�
	}
	
}
	

/*****************************************************************************************
 *  �����ж�
 * @param[in] 
 * @param[out] 
 * @return     
 ******************************************************************************************/	
//RC_Analy()���ã�4msһ��,,,,,//������Nrf_Erro=1,������Nrf_Erro>1��
//ң���Ǳ߷����ݣ����10msһ��
void remote_unlock(void)   //�������ݽ���
{

	volatile static uint8_t status=WAITING_1;
	static uint16_t cnt=0; //6sδ����Զ�����
	static uint8_t status_old=WAITING_1;
	
	//��δ�����ɿ�ʱ����ң��������ʧ�����������������������
	//��ֹ�������ң������������(������ҡ�����������λʱ�������󴥷�)
	if(Nrf_Erro>10 && !ALL_flag.unlock) {//�ȴ�������

				//��δ�����ɿ�ʱ�����ң����ʧ���ˣ������½�����������	
				status = EXIT_255; //�ȴ����½�������������ֹ�󴥷�
		
		
				if(status_old!=EXIT_255){
						//ң������ʧ��������һ�£������ǰ��δ���������ڽ���������Ҫ���¿�ʼ
						//��ʱ��δ�ж�Ϊ��ʽʧ��������ڷ����У�Ҳ�п�������Ϊ����̫Զ�������źŲ���̫�á�
						//�������ɿ�ʱ��һ�㶼��úܽ��������������Nrf_Erro>10�������ң�عػ��ˡ�
						fLED_Toggle(); hLED_Toggle();bLED_Toggle();aLED_Toggle();
						delay_ms(50); //1ms
						fLED_Toggle(); hLED_Toggle();bLED_Toggle();aLED_Toggle();
						delay_ms(50); //1ms
				
				}

		
	
				

	}
	
	status_old = status;
	
	//��ң�����ݲż���
	if(Nrf_Erro !=1 ) return; //��ǰ�ṹ��Remote���ݲ���ң�ط������������ݡ�
	
	
	//if(Remote.thr<1200 &&Remote.yaw>1800)                         //����ң�����½������ɻ�
	//{
	//	status = EXIT_255;
	//}
	
	//���½ǽ������������ݵ�һ���������ܣ�
	if(Remote.thr<1050 &&Remote.yaw<1150)                         //����ң�����½�����
	{
		
			if( LED.status == SimpleFlash ) { //������ң��
						bLED_H(); fLED_H(); aLED_H(); hLED_H(); //���������ᵼ�·ɻ����䣬ͨ��LED״̬���ѣ������˽�������
			
			}

			if(ALL_flag.unlock) status = EXIT_255; //����	
		
	}
	
	
	
	//������������
	//AUX82_isH�����������������, yaw:2000; thr:1000 -> �Զ�����
	if(AUX82_isH() && Remote.thr==1000 && Remote.yaw==2000 )      
	{
			status = WAITING_4;
	}
	
	
	//���ߵ����Nrf_Erro����1�����˷ɿظտ���ʱ
	switch(status)
	{
		
		//���������࣬�������->�������->������� ����LED�Ʋ����� ����ɽ���
		case WAITING_1: //Ĭ��ֵ  �ȴ�����

			//if(Remote.thr<1200)          //1.�������� 
			if(Remote.thr<1200)          //1.��������
			{			 
					 status = WAITING_2;			//�ȴ�2��������	 
			}		
			
			

			break; //����
			
			
		case WAITING_2:
			//if(Remote.thr>1850 )        //2.��������  
			if(Remote.thr>1850)        //2.��������  
			{		
						static uint8_t cnt = 0;
					 	cnt++;		
						if(cnt>5) //��������豣��200ms���ϣ���ֹң�ؿ�����ʼ��δ��ɵĴ�������
						{	
								cnt=0;
								status = WAITING_3; //�ȴ�3�������ţ���Ҫ��3�����������ţ�������Ҫ���½���������
						}
			}			
			else{
						//��δ��������
						/*
						static uint8_t cnt1 = 0;
					 	cnt1++;		
						if(cnt1>210) //����ȴ�2��δ�������ţ���Ҫ�����������ţ���������������
						{	
								cnt1=0;
								status = WAITING_1; //�ػ� WAITING_1
						}
						*/

			}
			break;
			
			
		case WAITING_3:
			//if(Remote.thr<1150)     //3.�������Ž���   
			if(Remote.thr<1150)          //3.�������Ž���, ��ֹң���������ػ��󴥷�Remote.thr>=1000
			{			 
						//ȡ����������Ҫ�ɿ�H+�����´�ߴ��
						if(AUX87_isH() && Remote.test_motor!=0){ //�����ֶ����Ե�����ҵ������ת��������H+��
								status = EXIT_255; //�ȴ����½�����������	 ����EXIT_255����һ��
							
						}
						
						else{
								status = WAITING_4;			//������4�����ɹ�����
							
						}

			}		
			else{
						//��δ��������
						static uint16_t cnt2 = 0;
					 	cnt2++;		
						//������ʱ
						if( cnt2>310 ) //3s��û���������������»ص��ȴ�״̬
						{	
								cnt2=0;
								status = EXIT_255; //�ȴ����½�����������	
							
								//������ʱ������һ��
							
								fLED_Toggle(); hLED_Toggle();bLED_Toggle();aLED_Toggle();
								delay_ms(50); //1ms
								fLED_Toggle(); hLED_Toggle();bLED_Toggle();aLED_Toggle();
								delay_ms(50); //1ms
							
						}
						
						

						
				
			}
			break;	

			
		case WAITING_4:	//�����ɹ�
				ALL_flag.unlock = 1;  //����
		

				status = PROCESS_31; //����������������
				LED.status = AlwaysOn;	
				Aux_Rc.yaw_stick = 0.0f; //ҡ��Ŀ��ƫ��������
		
				//��λ�������������Ծ��ڿ��зɣ��������ʱ������Ϊ0
				mini.flow_x_i=0;
				mini.flow_y_i=0;
				pixel_flow.fix_x = 0;
				pixel_flow.fix_y = 0;	
				
				//��������ģ�飬����������������´η��лָ�
				ANO_Uart1_Put_String( (u8 *)"\r\nr_e_s_t_a_r_t\r\n" );
		
				//����ÿ�ν���ʱ������������
				MPU_Err=1;
				SysTick_count=0;
				IIC_Init();//�и���Ƭ����Ҫ���³�ʼ����Ƭ��io,�����ǲ��ָܻ�������
				delay_ms(100);
				MpuInit();//ÿ�ν������������ǣ�ע���е�����������������ʱ��ʱ�޷����������ֵʱ��������ϴ���ֵ���������0xff��������Ҫ����������ϴ�ֵ��������ֵ����̫�󲨶�
				
				//��ѹ����
				baro_start=1;  //��ѹ�����־, ������ѹ������
				
				break;		
		
		
		case PROCESS_31:	//�������״̬������ʱ��Ҫ��������
			
				//δ��ɣ�6s���Զ�����
				if(Remote.thr<1020)
				{
					if(cnt++ > 700)      // ������  ��������ң�˴������6S�Զ����� ע�⣬��ң�����ݲŻ���� 700*10ms=7��
					{								
						status = EXIT_255;								
					}
				}
				else					
					cnt = 0; //���Ź�С��������
				
				
				//else if(!ALL_flag.unlock)   //�ж�����������Ӧֻ������>1020ʱ�� 
				if(!ALL_flag.unlock)          //Other conditions lock 
				{
					status = EXIT_255;				
				}

				
				
			break;
				
		case EXIT_255: //��������
			
			//������ʱ��ת�Ƶ�  WAITING_3 �������п����Ǵӷ���״̬ת�������ɻ����ڷ�����
		  //fLED_Toggle(); hLED_Toggle();bLED_Toggle();aLED_Toggle();
			//delay_ms(50); //1ms
		  // fLED_Toggle(); hLED_Toggle();bLED_Toggle();aLED_Toggle();
			//delay_ms(50); //1ms ��ʱ���ܷɻ����ڷɣ���Ҫ����ʱ��ת�� WAITING_1
		
			status_old = EXIT_255;
		
			//ָʾ���л�Ϊ����״̬, AllFlashLight��ʾLED��˸����������״̬; �п�����ͬ����˸SimpleFlash��Ҳ�п����ǽ�����˸AlternateFlash
			//AlwaysOn��ʾ����������DANGEROURS��ʾ����˫����
			if( LED.status == AlwaysOn || LED.status == DANGEROURS ) LED.status = AllFlashLight;	 //LED״̬ �ӽ���״̬���仯Ϊ����״̬
			
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







