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


#include "stm32f10x.h"
#include "LED.h"
#include "ALL_DATA.h"
#include "ALL_DEFINE.h" 
//---------------------------------------------------------
/*     you can select the LED statue on enum contains            */

//Ĭ��Ϊ0.3��Ŀ���
sLED LED = {300,AllFlashLight};  //LED initial statue is off;
u8 LED_warn;                             //default 300ms flash the status

u16 falg_Time=0;

/**************************************************************
 *  LED Init
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
void LEDInit(void)	
{	
		GPIO_InitTypeDef GPIO_InitStructure;
		//AFIO->MAPR = 0X02000000; //ʹ��4����д �ͷ�ĳЩ����д��ص�����,,,���Ҫע�͵�����Ȼ��Ҫ����ִ�� TIM3_PWM_Config()
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO , ENABLE);
	  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
  	GPIO_InitStructure.GPIO_Pin = fLED_io | hLED_io|aLED_io | bLED_io;		     //LED12
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(LED_GPIOB, &GPIO_InitStructure); 	 
	
}
/**************************************************************
 *  LED system
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
//����
void CloseLED() //2ms����1��
{
	if( LED.to_off && LED.off ){
		LED.to_off=0;
		bLED_H();
		fLED_H();
		aLED_H();
		hLED_H();
	}
	

}



/**************************************************************
 *  LED system
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/	
void PilotLED() //flash 300MS interval
{
	static uint32_t LastTime = 0;
  		
	


	
	if(SysTick_count - LastTime < LED.FlashTime)
	{
			//δ���л����ʱ��
			return;
	}
	else
			LastTime = SysTick_count;
	
	
	//50ms ����һ��
	if(LED.off){
			static u16 cnt;
			cnt++;
			//ʹ��delay1ms��Ӱ�����
			switch(cnt%24)
			{
				//�����½ǿ�ʼ������ʱ�룬��˳ʱ��
				case 0:
				case 12:
					bLED_L(); //1���죬����
					LED.to_off=1; //������2ms��ȡ��delay_ms(1)
					//delay_ms(1); //��������1ms
					break;
				case 3:
				case 21:
					fLED_L(); //2����������
					LED.to_off=1; //������2ms��ȡ��delay_ms(1)
					//delay_ms(1); //��������1ms
					break;
				case 6:
				case 18:
					hLED_L(); //3����������
					LED.to_off=1; //������2ms��ȡ��delay_ms(1)
					//delay_ms(1); //��������1ms
					break;
				case 9:
				case 15:
					aLED_L(); //4���죬����
					LED.to_off=1; //������2ms��ȡ��delay_ms(1)
					//delay_ms(1); //��������1ms
					break;
			}
			
			
			
	}
	
	
	
	if(LED.off){
		//����ת�Ƶ�CloseLED()
		//bLED_H();
		//fLED_H();
		//aLED_H();
		//hLED_H();
		
		return; //��ǰ����
	}
	
	
	
	switch(LED.status)
	{
		case AlwaysOff:      //����   
			bLED_H();
			fLED_H();
			aLED_H();
			hLED_H();
			break;
		
		//�������״̬��������̬ģʽ
		case AlwaysOn:  //����
		  bLED_L();
			fLED_L();
			aLED_L();
			hLED_L();
		  break;
		
		//�����п���ʱͬ����˸��Ҳ�п���ʱ������˸
		//����˸ǰ�����Ƶ�״̬��
		case AllFlashLight:				  //ȫ��ͬʱ��˸
			if(falg_Time++%3==0){
					fLED_Toggle();			
					bLED_Toggle();
					hLED_Toggle();			
					aLED_Toggle();
			}				
		  break;

		//2ǰ����2�������
		//rf�ճ�ʼ��������ʧ��ң��3���
		case AlternateFlash: //������˸
			//�Լ��ȵ�������һ��
			//bLED_H();aLED_H();fLED_L();hLED_L(); //����״̬��2ǰ����2�������
			if(falg_Time++%3==0){
					fLED_Toggle();
					bLED_Toggle();
					hLED_Toggle();
					aLED_Toggle();
			
			}

			//LED.status = AllFlashLight; //�л�Ϊ��˸
			break;
		
		//ͬ����˸����ʾ������ң�أ�����δ����
		case SimpleFlash: //ͬ����˸
			//�Լ��ȵ�������һ��
			//bLED_H();aLED_H();fLED_H();hLED_H(); //ͬ��״̬
		 
			fLED_Toggle();			
			bLED_Toggle();
			hLED_Toggle();			
			aLED_Toggle();
			//LED.status = AllFlashLight; //���л�Ϊ��˸
			break;	
		
		//0.3��Ĭ�Ͽ�������Ϊ0.8������
		case WARNING:
		  fLED_Toggle();
		  hLED_Toggle();
			bLED_Toggle();
		  aLED_Toggle();
			LED.FlashTime = 800;
			break;
		
		//�������״̬��������ͣģʽ
		case DANGEROURS:   //���� 2�� ��˸ 1��
			falg_Time++;
		  if(falg_Time>11)
					falg_Time=0;
				
			if(falg_Time>8)
			{
					bLED_Toggle(); //��˸
					aLED_Toggle();
					fLED_Toggle();
					hLED_Toggle();
			}
			else  if((falg_Time<7))
			{
					bLED_L();  //����
					fLED_L();
					aLED_L();
					hLED_L();
			}
		 
			LED.FlashTime = 150;
			
			break;
			
		//Ĭ�Ϲر�
		default:
			LED.status = AlwaysOff;
			break;
	}
}


/*
��������ʼ��rf�󣬽�����˸
������ң�أ�ͬ����˸
��̬������������
��ͣ����������2s����˸1s
ʧ����������˸��
��������ң�أ�ͬ����˸��


*/

/**************************END OF FILE*********************************/



