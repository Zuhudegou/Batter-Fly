/******************** (C) COPYRIGHT 2020 ANO Tech ***************************	
 * ��ֲ��  �������ƴ�
 * �Ա�    ��
 * ����QȺ ��
**********************************************************************************/
#include "WIFI_UFO.h"
#include "Remote.h"
#include "ANO_DT.h"
#include "ALL_DATA.h"
#include "myMath.h"


////////////////////////////////////////////////////
//������WIFI_UFOͨ��Э�����
////////////////////////////////////////////////////
static uint8_t wifi_RxBuffer[8];
uint16_t WIFI_UFO_Err;
uint8_t WIFI_SSI,WIFI_SSI_CNT;//WIFI�ź�

static uint8_t wifi_PUI=0;

//WIFI_UFO���ݽ��պ���
void WIFI_UFO_Data_Receive_Prepare(uint8_t data)
{
	static uint8_t _data_cnt = 0;
	static uint8_t state = 0;
//	ANO_Uart1_Put_Char(data);  //����1���
	
	switch(state)
	{
		case 0:
			if(data==0x66)  //��ͷ
			{
				state=1;
				wifi_RxBuffer[_data_cnt++]=data;
			}else state = 0;
		break;
		case 1:
			wifi_RxBuffer[_data_cnt++]=data;
			if(_data_cnt==7 && wifi_RxBuffer[7]==0x99)
			{
					state = 0;
					_data_cnt = 0;
					if( wifi_RxBuffer[7] == 0x99){ WIFI_UFO_Err=0;		}			//���ձ�־λ��0	
					
					//��ȡ����
					Remote.thr = ((float)(wifi_RxBuffer[3])/256) * 1000 + 1000;    //����
					LIMIT(Remote.thr,1000,2000);
					
					Remote.yaw  = ((float)(wifi_RxBuffer[4])*3.90625) + 1000;      //����
					LIMIT(Remote.yaw,1000,2000);
					//���ԣ��Ƿ�ֻ���������� Remote
					//Remote.raw_data2 = Remote.yaw;
					
					Remote.roll = (((float)(wifi_RxBuffer[1])/256)*1000) + 1000;   //����
					LIMIT(Remote.roll,1000,2000);
					Remote.pitch = (((float)(wifi_RxBuffer[2])/256)*1000)+ 1000;   //ǰ�� 
					LIMIT(Remote.pitch,1000,2000);
						
					//////////////////////////////////////////////////////
					//��־λ
					Remote.AUX2 = (wifi_RxBuffer[5]);
					LIMIT(Remote.AUX2,0,128);											
			}
		break;
		default:
			state = 0;
			_data_cnt = 0;
		break;
	}
	
}


//WIFI_UFO���ݴ�����   66 80 80 00 80 00 80 99
void WIFI_UFO_Data_Receive_Anl(void)   
{
	static u8 temp;
	static u8 count10=0;
  flag.NS = 2;//ң��������Դ��1��ң������2��WiFiͼ��ģ�顣3������ģ�飩  

	if(temp != Remote.AUX2)//��־λ�ж�
	{
			temp = Remote.AUX2;
			switch(temp)
			{
				case 1://һ�����  �ֻ��򿪶���ģʽ����Ч
							wifi_PUI = 1;
					break;
				case 2://һ������  �ֻ��򿪶���ģʽ����Ч
							wifi_PUI = 1;
					break;
				case 4://����ͣ��   ���ð�������һ�½�3��128
							ALL_flag.unlock = 0; 				//�˳�����
							LED.status = AllFlashLight; //��ʼ����
							wifi_PUI = 1;
					break;
				case 8://3D����   ��һ�� Ҫ���Ʒ�������8 
							wifi_PUI = 1;
					break;
				case 16: //��ͷģʽ ��һ������16 �ڰ�����0
							wifi_PUI = 1;
					break;
				case 128://���ð���  ����һ�½�3��128
							wifi_PUI = 1;
					break;
				default:	
							wifi_PUI = 0;
				break;
			}		
	}
	
	  count10++;
		if(count10>=2)
		{
			count10=0;		
			if(wifi_PUI==1)
			{	
//				printf("%d  \r\n",Remote.AUX2);  //����1���
			}
//			printf("   %d   %d   %d   %d   %d   %d     \r\n",wifi_RxBuffer[1],wifi_RxBuffer[2],wifi_RxBuffer[3],wifi_RxBuffer[4],wifi_RxBuffer[5],wifi_RxBuffer[6]);  //����1���
			
			printf("   %d   %d   %d   %d   %d        \r\n",Remote.thr,Remote.yaw,Remote.roll,Remote.pitch,Remote.AUX2);  //����1���
		}
				
}

//WIFI_UFO���Ӻ���
 u8 WIFI_UFO_Connect(void)
{
	static u8 Connect_flag;
	WIFI_UFO_Err ++;
	if(WIFI_UFO_Err==1)   //�ڽ��ճ���������
	{
		WIFI_UFO_Data_Receive_Anl();
		WIFI_SSI_CNT++;
		Connect_flag = 1;
		
	}
	if(WIFI_UFO_Err>=30)
	{
		//WIFI_UFO_Err = 1;//=1��ʾ�յ�������,û���ź�Ҳ����һ��,����Ϊ1��Ҳ�����ȥ�����ݵ�
		WIFI_UFO_Err = 2;//=1��ʾ�յ������ˣ�������Ҫ��λ��2
		Connect_flag = 0;
		flag.NS = 0;
//		printf("wifi©�� \r\n");  //����1���	
	}
	return Connect_flag;
}













