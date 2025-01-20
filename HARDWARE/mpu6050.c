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
#include "mpu6050.h"
#include "I2C.h"
#include "filter.h"
#include <string.h>
#include "LED.h"
#include "myMath.h"
#include "kalman.h"
#include "ANO_Data_Transfer.h"
#include "flash.h" //����������������У׼����


#define	SMPLRT_DIV		0x19	//�����ǲ����ʣ�����ֵ��0x07(125Hz)
#define	CONFIGL			0x1A	//��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
#define	ACCEL_CONFIG	0x1C	//���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz)
#define	ACCEL_ADDRESS	0x3B
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define GYRO_ADDRESS  0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define	PWR_MGMT_1		0x6B	//��Դ��������ֵ��0x00(��������)
#define	WHO_AM_I		  0x75	//IIC��ַ�Ĵ���(Ĭ����ֵ0x68��ֻ��)
#define MPU6050_PRODUCT_ID 0x68 //����һ�ֺ��ϵ�˿ӡһ������id��0x98������Ч�����á�
#define MPU6052C_PRODUCT_ID 0x72

//#define BMI160_PRODUCT_ID 		0xD1
#define ICM42688_PRODUCT_ID			0x47	

//ICM42688 �Ĵ���
#define ICM42688_DEVICE_CONFIG             0x11				//�������
#define ICM42688_GYRO_ACCEL_CONFIG0        0x52				//���õ�ͨ�˲���
#define ICM42688_GYRO_CONFIG0              0x4F				//�������̺�����
#define ICM42688_ACCEL_CONFIG0             0x50				//�������̺�����
#define ICM42688_PWR_MGMT0                 0x4E				//���ù���ģʽ
#define ICM42688_INTF_CONFIG0              0x4C

//���ٶ�����
#define AFS_2G  0x03
#define AFS_4G  0x02
#define AFS_8G  0x01
#define AFS_16G 0x00  // default
//���ٶ�����
#define GFS_2000DPS   0x00 // default
#define GFS_1000DPS   0x01
#define GFS_500DPS    0x02
#define GFS_250DPS    0x03
#define GFS_125DPS    0x04
#define GFS_62_5DPS   0x05
#define GFS_31_25DPS  0x06
#define GFS_15_625DPS 0x07



//#define   MPU6050_is_DRY()      GPIO_ReadOutBit(HT_GPIOC, GPIO_PIN_0)//IRQ������������
#ifdef  	USE_I2C_HARDWARE
		#define MPU6050_ADDRESS 0xD0//0x68
		
#else
		#define  MPU6050_ADDRESS 0xD0   //IICд��ʱ�ĵ�ַ�ֽ����ݣ�+1Ϊ��ȡ
		
		#define ICM42688_ADDRESS_H  0xD2 //�ߵ�ƽʱ
		#define ICM42688_ADDRESS_L  0xD0 //�͵�ƽʱ
		
		#define ICM42688_ADDRESS(T)  ( (T)==ICM42688_H? (ICM42688_ADDRESS_H):(ICM42688_ADDRESS_L) ) //	
		
#endif


//�������� sensor_type
#define MPU6050_L 1				//PCB���棬MPU6050����ģ����pcb���棬ADOΪ�͵�ƽ
#define ICM42688_H 2			//PCB���棬ICM42688ģ�鰲װ��pcb���棬ADOΪ�ߵ�ƽ
#define ICM42688_L 3			//PCB���棬ICM42688��Ƭ��pcb���棬ADOΪ�͵�ƽ


int16_t MpuOffset[7] = {0}; //6��У׼ֵ��ǰ3�����ٶȣ���3�����ٶ�; ���һ�����Ƿ�У׼��

static volatile int16_t *pMpu = (int16_t *)&MPU6050;
//u8 id_tmp;//��¼оƬid�����ڵ���

uint8_t product_id=0;  //��¼оƬid�����ڵ���
uint8_t sensor_type=0; //�Զ��ж�оƬ���ͣ�����xyz���ݶ�ȡ�Լ�������

/****************************************************************************************
*@brief  
*@brief   
*@param[in]
*****************************************************************************************/
int8_t mpu6050_rest(void) //û���õ�
{
	if(IIC_Write_One_Byte(MPU6050_ADDRESS, PWR_MGMT_1, 0x80) == FAILED)
		return FAILED;	//��λ
	delay_ms(20);
	return SUCCESS;
}


/****************************************************************************************
*@brief   
*@brief  
*@param[in]
*****************************************************************************************/
int8_t MpuInit(void) //��ʼ��
{
	uint8_t date = SUCCESS;
	
	//���������һ��ʼ��ʧ�ܣ�ֻ���ұ�����
	fLED_L();	 	//ǰ����������
	hLED_H();		//ǰ��������
	
	bLED_H();		//������Һ�
	aLED_H();		//��������
	
	 
	
	
	//�����Ǹ�λ
	delay_ms(10);
	IIC_Write_One_Byte(MPU6050_ADDRESS, PWR_MGMT_1, 0x80);	//��λ
	//IIC_Write_One_Byte(BMI160_ADDRESS, 0x7e, 0xb6);		//��λ����λʱ��15ms��
	IIC_Write_One_Byte(ICM42688_ADDRESS_H, ICM42688_DEVICE_CONFIG, 0x01); //��λ���������˺���Ҫ���ٵȴ�1ms��
	IIC_Write_One_Byte(ICM42688_ADDRESS_L, ICM42688_DEVICE_CONFIG, 0x01); //��λ���������˺���Ҫ���ٵȴ�1ms��
	
	delay_ms(10);
	
	
	
	//�ж�������оƬ����
	product_id=IIC_Read_One_Byte(MPU6050_ADDRESS, 0x75);

	if( product_id == MPU6050_PRODUCT_ID ){ //0x68
			//��ǰ������Ϊ MPU6050
			sensor_type=MPU6050_L; //1
	}
	else if( product_id == ICM42688_PRODUCT_ID ){ //0x47	
			//��ǰ������Ϊ ICM42688, �Ұ�װ��PCB���棨ADOΪ�͵�ƽ��
			sensor_type=ICM42688_L; //3
	}
	else{
			product_id=0;
			product_id=IIC_Read_One_Byte(ICM42688_ADDRESS_H, 0x75);
			if(product_id == ICM42688_PRODUCT_ID){//0x47	
					//��ǰ������Ϊ ICM42688, �Ұ�װ��PCB���棨ADOΪ�ߵ�ƽ��
					sensor_type=ICM42688_H; //2
			}
			else{
					sensor_type=0;//δ�ҵ�����
					//product_id = 0; //��Ʒid������
			}
	}
	//while(1){};
	//**************************************���µ��������ͺ������˻��ϵ�ʹ���ܽ�********************************************************
	// ��������BMI160(Bosch) << BMI270 = MPU6050(InvenSense) < ICM20602(InvenSense) < ICM42688 << ICM45686 < ADXL355
	// ������ BMI160 �����ݲ��ȶ������۸�����ˣ�ֻҪ3��ࣩ����װΪ LGA14��ʡ�ռ䣬������˻���ѡ�����֣�
	// Ӧ��ʢ�� MPU6050 �����ȶ������ϺͿ�Դ������Ϊ�ḻ��������������磬ֻ��I2C�ӿڣ����Ӹ���Ҳ�׵��²��ȶ������ཨ��ѡ��180�ȣ���Ƭ�¶�210�ȡ�
	// BMIϵ������BMI088(BMI160��2~3���۸���ICM42688�൱)��˵Ч��Ҳ����������װ�Ƚ����⣨LGA16-�����Σ������ֳ�ģ�飬��δ���ԡ�
	// ICM42688 ��װҲ��LGA14����MPU6050��һЩ���ܽŲ���벩�����BMI���ݣ�����ָ��Ϻã����ҿ���Ч���ȽϺã��ܶ������ƿ�Դ���˻�ʹ��(BMI088Ҳ����)��
  // ICM45686 �۸�ϸߣ���оƬ�ͽӽ�20Ԫ�����ܷǳ��ã��ӽ��ڼ۸���ٵ�ADXL355�����˻���ѡ�õĲ��ࣻ
	// ADXL355  �����ڹ�ҵ���豸���Ѿ������ڵͳɱ�IMU���۸���һ�ٶ�Ԫ�����Ҳ�ܴ����˻�δ����ʹ�ð�����
	// ���ϸ��ͺ����������ǣ����������������۸�Ҳ������Ӧ������
	//********************************************************************************************************************************
	
	
	//������ ��ʼ������
	MPU_Err = 1;	//Ĭ�����ò��ɹ�	
	
	
	if( sensor_type == MPU6050_L ){
						
					do
					{
							//hLED_Toggle();		//ǰ��� ����
							
							//������ģ��ֱ��5v������Ҫ�ӳٳ�ʼ���ˡ�
							//delay_ms(500); //��Դ�������ض����Ƚ��������Լ�����ģ���Լ������ϵ磬Ҫ���һ�ᣬ��Ȼ����ʹ��ʶ��оƬidҲ������Ե������쳣��
							//delay_ms(100); //����ÿ�ν���ʱ����ʼ�������ǡ�
							date = IIC_Write_One_Byte(MPU6050_ADDRESS, PWR_MGMT_1, 0x80);	//��λ
						
							hLED_L();		//ǰ��� ����
							delay_ms(1);
						
							hLED_H();		//ǰ��� ��
							delay_ms(50);
						
							date += IIC_Write_One_Byte(MPU6050_ADDRESS, SMPLRT_DIV, 0x02); //�����ǲ����ʣ�0x00(500Hz)��2ms����һ������
							date += IIC_Write_One_Byte(MPU6050_ADDRESS, PWR_MGMT_1, 0x03);	//�����豸ʱ��Դ��������Z��
							date += IIC_Write_One_Byte(MPU6050_ADDRESS, CONFIGL, 0x03);   //��ͨ�˲�Ƶ�ʣ�0x03(42Hz)
							date += IIC_Write_One_Byte(MPU6050_ADDRESS, GYRO_CONFIG, 0x18);//+-2000deg/s
							date += IIC_Write_One_Byte(MPU6050_ADDRESS, ACCEL_CONFIG, 0x09);//+-4G
							MPU_Err = 1;	
					}
					while(date != SUCCESS);
	
					
					
	}
	
	
	
	else if( sensor_type == ICM42688_L || sensor_type == ICM42688_H ){	
				//����������Ŀ��Դ�ڹٷ�����飬��DS-000347-ICM-42688-P-v1.7.pdf��

				do
				{

						//IIC_Write_One_Byte(BMI160_ADDRESS, 0x7e, 0xb6);		//��λ����λʱ��15ms����λ�������ȷ��д�Ĵ���������ģʽ��ΪSuspend��0����
						IIC_Write_One_Byte(ICM42688_ADDRESS(sensor_type), ICM42688_DEVICE_CONFIG, 0x01); //��λ���������˺���Ҫ���ٵȴ�1ms��
					
					
						hLED_L();		//ǰ��� ����
						delay_ms(15);
					
						hLED_H();		//ǰ��� ��
						delay_ms(50);
					
					
					  uint8_t date_tmp =0xFF;
						date = SUCCESS;//0
		
		
						 //1. ���õ�ͨ�˲����˳���Ƶ�񶯣�Ĭ���ź�ͨ������125Hz��ODR=500Hzʱ�������ڽ��ٶ����ʺϵģ������ڼ��ٶ�̫���ˣ�80ҳ
						 //ICM42688_GYRO_ACCEL_CONFIG0
						//IIC_Write_One_Byte(ICM42688_ADDRESS, ICM42688_GYRO_ACCEL_CONFIG0, 0x44);  //���ٶȺͽ��ٶ� ��ͨ�˲�����ѡ��ͨ������0~25Hz���˳���Ƶ
						//���ٶ�ֵ�ǳ����ױ���Ӱ�죬�������Դ���˲�Ч����һ����С����ٶȶ����񶯲����У�����Ҫ��ʱ�Ժܺã������˲�̫�ã���ֹ���ٶ��ӳ١�
						//���Լ��ٶ�����ٶȷֱ�ѡ��12.5Hz��50Hz, ����ǳ��ؼ�������������������ƽ����񶯸���ֻҪ��΢��һ�㣬���ᵼ����ͣЧ�����á�
						//IIC_Write_One_Byte(ICM42688_ADDRESS(sensor_type), ICM42688_GYRO_ACCEL_CONFIG0, 0x73);  //���ٶ�����ٶ� ��ͨ�˲����ֱ�ѡ��ͨ������12.5Hz�Լ�50Hz���������Ƶ�ʽ����˳�
						IIC_Write_One_Byte(ICM42688_ADDRESS(sensor_type), ICM42688_GYRO_ACCEL_CONFIG0, 0x71);    //���ٶ�����ٶ� ��ͨ�˲����ֱ�ѡ��ͨ������12.5Hz�Լ�125Hz���������Ƶ�ʽ����˳�
						//���ٶȲ���ѡ��̫��Ƶ�ʵĵ�ͨ�˲����ᵼ����̬���ź��ͺ����أ�����PID�����ͺ��񵴣��ῴ��������������һ�����񶯣�
						//��������Ӱ�����ء�1�����������������Լ�������2��PID������3�������ƽ�⣻4�����������λ����� 5����·���Լ������Ǹ���Ч����
						
						//�Ƿ����óɹ�
						date_tmp=IIC_Read_One_Byte(ICM42688_ADDRESS(sensor_type), ICM42688_GYRO_ACCEL_CONFIG0); //��ѯ
						//product_id=date_tmp;//���ڵ���
						if(date_tmp!=0x71){ //��ͨ�˲����ֱ�Ϊ12.5Hz���Լ�125Hz
								date=1; 
						}
						
						
						

						
						//2. ���ü��ٶȵ����̺Ͳ����ٶȣ�����78ҳ
						date_tmp = IIC_Read_One_Byte(ICM42688_ADDRESS(sensor_type), ICM42688_ACCEL_CONFIG0);  //��ѯ
						date_tmp |= (AFS_4G << 5);   						//���� ��4g��Ĭ��16g����Ϊ4��
						date_tmp |= (0x0F);     								//���ODR���� 500HZ
						IIC_Write_One_Byte(ICM42688_ADDRESS(sensor_type), ICM42688_ACCEL_CONFIG0, date_tmp);
						//�Ƿ����óɹ�
						date_tmp=IIC_Read_One_Byte(ICM42688_ADDRESS(sensor_type), ICM42688_ACCEL_CONFIG0); //��ѯ
						//product_id=date_tmp;//���ڵ���
						if(date_tmp!=0x4F){ //+/-4g��500hz
								date=2;
						}
						
						
						
						
						//3. ���ý��ٶȵ����̺Ͳ����ٶȣ�����77ҳ
						date_tmp=IIC_Read_One_Byte(ICM42688_ADDRESS(sensor_type), ICM42688_GYRO_CONFIG0);  //��ѯ
						date_tmp |= (GFS_2000DPS << 5);   				//���� +-2000deg/s������Ĭ��ֵ
						date_tmp |= (0x0F);     									//���ODR���� 500HZ
						IIC_Write_One_Byte(ICM42688_ADDRESS(sensor_type), ICM42688_GYRO_CONFIG0, date_tmp);
						
						//�Ƿ����óɹ�
						date_tmp=IIC_Read_One_Byte(ICM42688_ADDRESS(sensor_type), ICM42688_GYRO_CONFIG0); //��ѯ
						//product_id=date_tmp;//���ڵ���
						if(date_tmp!=0x0F){ //+-2000deg/s��500hz
								date=3;
						}




						//4. ���ù���ģʽ 76ҳ
						date_tmp=IIC_Read_One_Byte(ICM42688_ADDRESS(sensor_type), ICM42688_PWR_MGMT0);  //��ѯ
						date_tmp &= ~(1 << 5);								//ʹ���¶Ȳ���, 0��ʾ�¶�ʹ�ܣ�Ҳ��Ĭ��ֵ
						date_tmp |= ((3) << 2);								//����GYRO_MODE  0:�ر� 1:���� 2:Ԥ�� 3:������
						date_tmp |= (3);										//����ACCEL_MODE 0:�ر� 1:�ر� 2:�͹��� 3:������
						IIC_Write_One_Byte(ICM42688_ADDRESS(sensor_type), ICM42688_PWR_MGMT0, date_tmp);
						
						//�Ƿ����óɹ�
						date_tmp=IIC_Read_One_Byte(ICM42688_ADDRESS(sensor_type), ICM42688_PWR_MGMT0); //��ѯ
						//product_id=date_tmp;//���ڵ���
						if(date_tmp!=0x0F){ //�¶�+A����+G����
								date=4;
						}
						
						
						
						
						//5. ����û������ʱ����ʾ�ϴβ���ֵ����ֹ��ʾΪ 0xFFFF(-32768),,,,,FIFO_HOLD_LAST_DATA_EN=1,,,,74ҳ 
						//���ǽ����ɿ�ʱ��remote.c �����һ�� ����i2c �Լ� ��������������
						//���û���������ã������ǽ����ɿ�ʱ�������������׶Σ�����������Ϊ 0xFFFF�������������̬�ǻ����쳣��
						date_tmp=IIC_Read_One_Byte(ICM42688_ADDRESS(sensor_type), ICM42688_INTF_CONFIG0);  //��ѯ
						date_tmp |= (0x80);										//���λ��1
						IIC_Write_One_Byte(ICM42688_ADDRESS(sensor_type), ICM42688_INTF_CONFIG0, date_tmp);
						
						//�Ƿ����óɹ�
						date_tmp=IIC_Read_One_Byte(ICM42688_ADDRESS(sensor_type), ICM42688_INTF_CONFIG0); //��ѯ
						//product_id=date_tmp;//���ڵ���
						if( (date_tmp&0x80)!=0x80 ){ //���λ�Ƿ�Ϊ1
								date=5;
						}
						
						
							
						delay_ms(5);			//������PWR��MGMT0�Ĵ����� 200us�ڲ������κζ�д�Ĵ����Ĳ���
			
						
						//product_id=date;//���ڵ���, ��¼�Ĳ�����
				}
				while(date != SUCCESS); //���óɹ���������
				
	}
	
	
	
	//�������ͺŲ����ϣ�ͣס������������
	else{
				do
				{
						hLED_L();		//ǰ��� ����
						delay_ms(10);
					
						hLED_H();		//ǰ��� ��
						delay_ms(100);
					
					
				}
				while(1); //���óɹ���������
	
	}
	
			
					
	//������ͨѶok��2��������
	fLED_L();	 	//ǰ��������
	hLED_L();		//ǰ��������

	
	//date = IIC_Read_One_Byte(MPU6050_ADDRESS, 0x75);
	
	//оƬȷ��ʧ�ܣ�ֻ����ߺ�ơ�
	//if( date!= MPU6050_PRODUCT_ID ) //��ȡоƬ�ͺţ�ȷ��оƬ�Ƿ���ȷʶ��
	if( sensor_type==0 ) 	
	//if(date!= 0x98) //���µ�MPU6050��ƷidΪ0x68, �����ڵ�MPU6050Ϊ0x98, Ҳ���ã���Ч�����Զ��������ͣ���ܶ��꣬��Щ�̼һ�����˿ӡð�����µģ��۸��������ĵ�һЩ��
	{
				//����ᱻ ���������ģ���״̬���ǡ�
				
				//fLED_H(); //���Ͻ�����Ϩ�𣬱�ʾ������ʧ�ܣ����Ͻ����������ģ������ƽ�����˸��������ͬ����˸
				return FAILED;
	}	

	
	//MpuGetOffset(); //����ȡ��MPU6050У׼������ң����key2������ʱУ׼���ǽ���ģʽ���Ȱ����Ŵ���ͣ�
	FLASH_read(MpuOffset,7);//��mcu��FLASH�ж�ȡMPU6050��ˮƽ��ֹ�궨У׼ֵ
			
	
	MPU_Err = 0;
	

	
/*
	//����
	while(1){
			delay_ms(2); //ģ��ÿ��2ms��ȡһ������
			MpuGetData();

		//���������ǿɿ��ԣ����ٶ�012�����ٶ�345
		if(  		MPU6050.gyroX>30 || MPU6050.gyroX<-30
				 || MPU6050.gyroY>30 || MPU6050.gyroY<-30
				 || MPU6050.gyroZ>30 || MPU6050.gyroZ<-30
		){
				//����������ھ�ֹ״̬����Ӧ�ý������
				//�����ֹ״̬��������������ӣ�˵��ͨѶ�������ˣ����������Ǳ������ˡ�
				product_id++; //
		
		}
		
		if(MPU6050.accZ<1000){
				product_id=100;
		
		}

	}

*/

	
	return SUCCESS;

	
}


//У׼+���棬
//����ң����key2�������ǽ���ģʽ���Ȱ����Ŵ���ͣ�
void MpuGetOffset_save(void) //�û�����У׼�Żᱣ�浽flash
{
	
	MpuGetOffset(); //���������ʱҲҪִ�У���Ҫ��������flash������Ӱ��flash������
	
	MpuOffset[6]=1;//�Ѿ�У׼��0~5Ϊ6��У׼���ݣ����һλΪ�µ�mcu�Ƿ��Ѿ�У׼mpu6050������
	
	FLASH_write(MpuOffset,7);//������д��FLASH�У�һ����6��int16����

	
}





/****************************************************************************************
*@brief    
*@brief   
*@param[in]
*****************************************************************************************/

#define  Acc_Read_6050() IIC_read_Bytes(MPU6050_ADDRESS, 0X3B, buffer, 6 )
#define  Gyro_Read_6050() IIC_read_Bytes(MPU6050_ADDRESS, 0x43, &buffer[6], 6 )

#define	Acc_Read_ICM42688(T)	IIC_read_Bytes(ICM42688_ADDRESS(T), 0x1F, buffer, 6 ) 	//ǰ6���ֽڷ�3����ٶ�
#define	Gyro_Read_ICM42688(T)	IIC_read_Bytes(ICM42688_ADDRESS(T), 0x25, &buffer[6], 6 );	//��6���ֽڷ�3����ٶ�

u8 MPU_empty_cnt;//��������
u8 MPU_empty_cnt_all;//�ܴ���

//2ms����һ�Σ�pMpu�����ڴ��ַ��MPU6050��ʼ��ַһ��
//��ȡ����2ms����������6ms��Ҳ���������������������������㡣
void MpuGetData(void) //��ȡ���������ݼ��˲�
{
	  uint8_t i;
    uint8_t buffer[12];
		memset(buffer, 0, 12);  //16�ֽڳ�ʼ��Ϊ0
	
		if( sensor_type == MPU6050_L ){
					Acc_Read_6050();
					Gyro_Read_6050();
		}

		else if( sensor_type == ICM42688_L || sensor_type == ICM42688_H){	
					Acc_Read_ICM42688(sensor_type);
					Gyro_Read_ICM42688(sensor_type);
		}
		

		

		//8λ���ݺϲ�Ϊ16λ����
		for(i=0;i<6;i++)  //3�����ٶȣ�3�����ٶ�
		{
			//ԭʼֵ
			pMpu[i] = (((int16_t)buffer[i<<1] << 8) | buffer[(i<<1)+1]);	
		}
	
		
		//XY�Ե����������ICM42688��PCB����������	//x(0,3) y(1,4) 
		if( sensor_type == ICM42688_H ){	
					int16_t tmp=0;
					tmp=pMpu[0]; pMpu[0]=pMpu[1]; pMpu[1]=tmp; //xy���ٶȶԵ�
					tmp=pMpu[3]; pMpu[3]=pMpu[4]; pMpu[4]=tmp; //xy���ٶȶԵ�
		}
		
	
		//���3�����������ٶȶ�Ϊ����߶�Ϊ-1(0xFFFF)����˵�������쳣
		//��������״̬����������Ϩ��
		//fLED_H(); //2���������ϣ�Ϩ��
		//��������ʧЧ�����ݣ����ٶ� -233�� -138�� 2762, ���ٶ� 29��7��0��
		//��ʱ��offset ���ݣ����ٶ�  234�� 139��-2763�����ٶ� -29��-7��1
		//��������ʧЧ�����ݣ����ٶ� -234�� -130�� 2762, ���ٶ� 24��6��0��
		//��ʱ��offset ���ݣ����ٶ�  234�� 139��-2763�����ٶ� -29��-7��1
		
		
		
		if(MPU_Err==0 && SysTick_count>1000 ){//�տ����������������쳣ʱ�����ж�������Ч��
				if(ABS(pMpu[0])<=1 && ABS(pMpu[1])<=1 && ABS(pMpu[2])<=1){//��Ϊ���ڼ��ٶ��������������ܷɻ���̬��Σ�������3������ٶ�ͬʱ��С��
				//if(ABS(pMpu[0])<=1 && ABS(pMpu[1])<=1 && ABS(pMpu[2])<=1 && SysTick_count>1000 ){//��Ϊ���ڼ��ٶ��������������ܷɻ���̬��Σ�������3������ٶ�ͬʱ��С��

							MPU_empty_cnt_all++; //�쳣�����ܴ������������
							MPU_empty_cnt++; //�����쳣���ݵĴ��������ʱbmi088������(����Ƶ����Ϊ400Hz)��������ÿ2ms��ȡһ�Σ�����ÿ�ζ��ܶ�ȡ������ֻҪ��������������1���Ͳ����쳣��
							//fLED_H(); //2���������ϣ�Ϩ�� �� ���������������Ʋ�ͬ����ʾ�����������쳣��
							//��ʱi2c�ź��ܸ��ţ�����������������������ߣ�Ҳ��ż����ʱ�쳣��������һ�㲻������ô��ĸ��ţ�icm42688��mpu6050�������������������ţ������Ǹ�ʡ�絼���ź�������������
							if(MPU_empty_cnt>0){//2 �� ����������������������������ʾ��������������ȡ�׶������쳣������ʼ���׶���������������
											//ȷ����������״̬��ͬ����ʾ�����������쳣��
											fLED_H(); //����
											hLED_L(); //����
							} 
							//�����������ݣ�Ӧ��Ҫ�����������ݡ��������������ﵽһ��������ͨ��LED��ʾ������������������2�����Ǻ������ģ���Ϊ������500Hz��ÿ2ms��ȡһ�β�һ������������
							//�����Ǹ����ϰ汾MPU6050ģ������⣨������ģ�鹩Ӧ�����˶���ic���ߺ����¶ȹ��ߵ��£������µ�ic�ͽ���ˣ�
							//��ʱ����ܾõ�һ�ο���ʱ����������ʶ�𵽣���������ȫ��0����F��
							//��ʱ��Ҫָʾ����ʾ�쳣���ɿذ������¿������ɡ�

				}
				else{
							if(MPU_empty_cnt>0){ //�մ��쳣�лָ�����ʱ�������Ʋ�ͬ����
								
									//����״̬�ָ���ң�ؽ�������߰�һ��LOCK���ƶ��ָܻ���
									MPU_empty_cnt=0; //��������
							}
							
				
				}
				
		}
		
	
	
	
		for(i=0;i<6;i++)  //3�����ٶȣ�3�����ٶ�
		{
			
			//ԭʼֵ,�Ѿ��Ƶ���һ����Ҫ�ж�һ��ԭʼֵ�Ƿ���Ч
			//pMpu[i] = (((int16_t)buffer[i<<1] << 8) | buffer[(i<<1)+1]);	

			
			//MPU ��Ƭλ�ã�������ĵ�����
			//0,3->x,,,1,4->y,,,,2,5->Z
			//if(i==2||i==5 || i==0 || i==3) pMpu[i]= -pMpu[i];
			//if(i==5) pMpu[i]= -pMpu[i];
			
			//y(1,4) z(2,5)ת������Ҫ���ڲ���֮ǰ��
			//if(i==2||i==5 || i==1 || i==4) pMpu[i]= -pMpu[i];
			//if(i==2) pMpu[i] = -pMpu[i]; //ת������Ҫ���ڲ���֮ǰ��
			
			
			//����ת����MPU6050��PCB����
			if( sensor_type == MPU6050_L ){   //���� 
						//z(2,5)ת������Ҫ���ڲ���֮ǰ��
						if(i==2||i==5 ) pMpu[i]= -pMpu[i];
						
						//y(1,4) ת������Ҫ���ڲ���֮ǰ��
						if(i==1||i==4 ) pMpu[i]= -pMpu[i];
						
						//x(0,3) ת������Ҫ���ڲ���֮ǰ��
						//if(i==0||i==3 ) pMpu[i]= -pMpu[i];
			}

			//����ת����ICM42688��PCB����
			else if( sensor_type == ICM42688_L){	 //����
						//z(2,5)ת������Ҫ���ڲ���֮ǰ��
						//if(i==2||i==5 ) pMpu[i]= -pMpu[i];
						
						//y(1,4) ת������Ҫ���ڲ���֮ǰ��
						if(i==1||i==4 ) pMpu[i]= -pMpu[i];
						
						//x(0,3) ת������Ҫ���ڲ���֮ǰ��
						if(i==0||i==3 ) pMpu[i]= -pMpu[i];
						
			}
			
			//����ת����ICM42688��PCB������
			else if( sensor_type == ICM42688_H ){	 //����
						//z(2,5)ת������Ҫ���ڲ���֮ǰ��
						if(i==2||i==5 ) pMpu[i]= -pMpu[i];
						
						//y(1,4) ת������Ҫ���ڲ���֮ǰ��
						if(i==1||i==4 ) pMpu[i]= -pMpu[i];
						
						//x(0,3) ת������Ҫ���ڲ���֮ǰ��
						if(i==0||i==3 ) pMpu[i]= -pMpu[i];
			}
				
			

			
			
			//�����Ǿ�ֹУ׼�Ĳ���
			pMpu[i] = pMpu[i] - MpuOffset[i];		//����
			
			

			
			//��ǰΪƫ�����ٶȣ�����2��������������ֹ��Ư�ͷ�������Ư��
			if(i==5){
					MPU6050.gyroZ_raw=pMpu[i]; //������ֵ
					//��һ������zƫ�����ٶȣ�����΢СƯ�ƣ�Ҫ�Խ���������룻ֱ�Ӹ�ֵ������ȡ��
					pMpu[i] = round( (float)pMpu[i] +  MPU6050.gyroZ_offset ); //��ֹ״̬ȫ��ƫ�� +/-10���ڣ����ֵ�Զ���������imu.c��
					pMpu[i] = round( (float)pMpu[i] +  MPU6050.gyroZ_offset1 ); //�����в���ƫ�� ���û��趨YAWƯ������+/-128����
			}

			
			
			
			if(i < 3) //=0 1 2�����ٶ� �˲�
			{
				{
					static struct _1_ekf_filter ekf[3] = {{0.02,0,0,0,0.001,0.543},{0.02,0,0,0,0.001,0.543},{0.02,0,0,0,0.001,0.543}};	
					kalman_1(&ekf[i],(float)pMpu[i]);  //һά������
					pMpu[i] = (int16_t)ekf[i].out;
				}
			}
				
			
			if(i > 2) //=3 4 5, ���ٶ� �˲�
			{
				uint8_t k=i-3; //k:0,1,2
				const float factor = 0.15f;  //�˲�����			
				static float tBuff[3];//��¼�ϴν��ٶ�ֵ

				//pMpu[i] = tBuff[k] = tBuff[k] * (1 - factor) + pMpu[i] * factor;

				tBuff[k] = tBuff[k]+ ( (float)pMpu[i] - tBuff[k] ) * factor; //�˲�
				pMpu[i] = round(tBuff[k]);
				
			}
			
			
			//��ǰΪƫ�����ٶȣ�����2��������������ֹ��Ư�ͷ�������Ư��
			if(i==5){ //��imu.c ���ǶȻ��֣�6msһ��
						MPU6050.gyroZ_sum += pMpu[i];
						MPU6050.gyroZ_sum_cnt++;
				
			}
			
			
		
	}
		
	

	
	
		
			//�ڵ��������£���ֱ���ٶ�
			//Aux_Rc.debug1 = NormAccz;
			//�ڵ��������£�ˮƽy������ٶȣ���̬��roll���£�
			//Aux_Rc.debug2 = pMpu->accZ * sin(Angle.roll*PI/180.0f) - pMpu->accY*cos(Angle.roll*PI/180.0f);
      
			//����Ϊxyz����������xyz���ٶȷ����෴�������������˻���೯�£�����z�ᳯ��ʱ���Ҽ����˶���������y������Ķ�����ֵ
	
			//       ����   x+         ������ٶ�   x-	       ������̬�� p- ��˳ʱ��Ϊ��ֵ��
			//             /                      /                    /
			//      y+ ___/___ y-          y- ___/___ y+        r+ ___/___ r-   
			//           /|                     /|                   /|
			//          / |                    / |                  / |
			//        x-	|                  x+  |                p+  |
			//            z+                     z-                     
			//�ж������Ƿ����Ƿ���ȷ����ĳ����������ָ�����ʱ������ļ��ٶ�Ӧ����ʾλ 8192��
			//һ��Ҫע�⣺������������ٶȷ����෴����������Ҳ�ǹ������ķ���ĳ���ڳ���������˶�ʱ�����䷴��������һ������������������
	
	/*
			float acc_y = MPU6050.accY*cos(Angle.roll*PI/180.0f)-MPU6050.accZ * sin(Angle.roll*PI/180.0f);
	
			MPU6050.speedY += acc_y*0.002;//2ms����һ��
	
			Aux_Rc.debug2=MPU6050.speedY;
			Aux_Rc.debug5=acc_y;	
			*/
	
}

/****************************************************************************************
*@brief   get mpu offset
*@brief   initial and cmd call this
*@param[in]
*****************************************************************************************/
//�����Ǿ�̬У׼�����ɻ��ڷ��й��̣�z���ٶ���Ȼ����΢����⣬���·��м�����֮����һЩ��

void MpuGetOffset(void) //У׼
{
	int32_t buffer[6]={0};
	int16_t i;  
	uint8_t k=30;
	const int8_t MAX_GYRO_QUIET = 5;
	const int8_t MIN_GYRO_QUIET = -5;	
	
	
	
	/*  wait for calm down   */
	int16_t LastGyro[3] = {0};
	int16_t ErrorGyro[3];	
	
	
	
	
	
	/*  set offset initial to zero 	*/
	
	memset(MpuOffset,0,12);
	MpuOffset[2] = 8192;   //set offset from the 8192  
	
	
	
	TIM_ITConfig(  //ʹ�ܻ���ʧ��ָ����TIM�ж�
				TIM3, //TIM2
				TIM_IT_Update ,
				DISABLE  //ʹ��
	);	
	
	
	//�ȴ��ɻ���ֹ����
	while(k--)//30�ξ�ֹ���ж����������ھ�ֹ״̬
	{
				do
				{
							delay_ms(10);
							MpuGetData();
							for(i=0;i<3;i++) //�õ�����2�Σ���3�����ݣ����ٶȣ��Ĳ��졣
							{
										ErrorGyro[i] = pMpu[i+3] - LastGyro[i];
										LastGyro[i] = pMpu[i+3];	
							}
							
				}while ((ErrorGyro[0]  > MAX_GYRO_QUIET )|| (ErrorGyro[0] < MIN_GYRO_QUIET)
					 		 ||(ErrorGyro[1] > MAX_GYRO_QUIET )|| (ErrorGyro[1] < MIN_GYRO_QUIET)
 					 		 ||(ErrorGyro[2] > MAX_GYRO_QUIET )|| (ErrorGyro[2] < MIN_GYRO_QUIET)
							); //�����Сʱ����Ϊ�Ѿ���ֹ

	}
	
	
	

	/* throw first 100  group data and make 256 group average as offset */	
	for(i=0;i<356;i++)	//ˮƽ�Լ���ֹ�궨��ǰ3�����ٶȣ���3�����ٶ�
	{		
				MpuGetData();
				if(100 <= i) //ȡ256�飨i��100��356�����ݽ���ƽ��
				{
							uint8_t k;
							for(k=0;k<6;k++) //ǰ3�����ٶȣ���3�����ٶ�
							{
										buffer[k] += pMpu[k]; //256���������
							}
				}
	}
	

		
	//���������ָֻ�ԭ������
	//��mode=1��ֻ��ˮƽ�궨��У׼���ٶȣ�xy=0��z=8192������ĺ�3��Ԫ��
	//��mode=2��ֻ����ֹ�궨��У׼���ٶȣ�xyz=0�������ǰ3��Ԫ��
	
	for(i=0;i<6;i++) //ȡ����ֹʱ������
	{
				MpuOffset[i] = buffer[i]>>8; // ƽ��ֵ=�ܺ�/256
	}
	
	
	
	
	TIM_ITConfig(  //ʹ�ܻ���ʧ��ָ����TIM�ж�
				TIM3, //TIM2
				TIM_IT_Update,
				ENABLE  //ʹ��
	);
	
	
	
	
	
}

/**************************************END OF FILE*************************************/

