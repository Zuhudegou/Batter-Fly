/******************************************************************************************************
 * Update the PID parameters.
 *
 * @param[in] pid         A pointer to the pid object.
 * @param[in] measured    The measured value
 * @param[in] updateError Set to TRUE if error should be calculated.
 *                        Set to False if pidSetError() has been used.
 * @return PID algorithm output
 *******************************************************************************************************/	
#include "imu.h"
#include "myMath.h"
#include <math.h>


//static float NormAccz;
float NormAccz;  //�����ڴ�ֱ�ڵ��淽���ϵļ��ٶȣ����������Լ��˶����ٶȣ�������ĳ����ļ��ٶȣ�����ͨ��������ļ��ٶȼ��������

typedef volatile struct {
  float q0;
  float q1;
  float q2;
  float q3;
} Quaternion;


//6msһ��
void GetAngle(const _st_Mpu *pMpu,_st_AngE *pAngE, float dt) 
{
	volatile struct V{
				float x;
				float y;
				float z;
				} Gravity,Acc,Gyro,AccGravity;

	static struct V GyroIntegError = {0};
	static  float KpDef = 0.8f ;
	static  float KiDef = 0.0003f;
	static Quaternion NumQ = {1, 0, 0, 0};
	float q0_t,q1_t,q2_t,q3_t;
  //float NormAcc;	
	float NormQuat; 
	float HalfTime = dt * 0.5f;

	

	// ��ȡ��Ч��ת�����е��������� 
	Gravity.x = 2*(NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);								
	Gravity.y = 2*(NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);						  
	Gravity.z = 1-2*(NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);	
	// ���ٶȹ�һ��
 NormQuat = Q_rsqrt(squa(MPU6050.accX)+ squa(MPU6050.accY) +squa(MPU6050.accZ));
	
    Acc.x = pMpu->accX * NormQuat;
    Acc.y = pMpu->accY * NormQuat;
    Acc.z = pMpu->accZ * NormQuat;	
 	//������˵ó���ֵ
	AccGravity.x = (Acc.y * Gravity.z - Acc.z * Gravity.y);
	AccGravity.y = (Acc.z * Gravity.x - Acc.x * Gravity.z);
	AccGravity.z = (Acc.x * Gravity.y - Acc.y * Gravity.x);
	//�������ٶȻ��ֲ������ٶȵĲ���ֵ
    GyroIntegError.x += AccGravity.x * KiDef;
    GyroIntegError.y += AccGravity.y * KiDef;
    GyroIntegError.z += AccGravity.z * KiDef;
	//���ٶ��ںϼ��ٶȻ��ֲ���ֵ
    Gyro.x = pMpu->gyroX * Gyro_Gr + KpDef * AccGravity.x  +  GyroIntegError.x;//������
    Gyro.y = pMpu->gyroY * Gyro_Gr + KpDef * AccGravity.y  +  GyroIntegError.y;
    Gyro.z = pMpu->gyroZ * Gyro_Gr + KpDef * AccGravity.z  +  GyroIntegError.z;		
	// һ�����������, ������Ԫ��

	q0_t = (-NumQ.q1*Gyro.x - NumQ.q2*Gyro.y - NumQ.q3*Gyro.z) * HalfTime;
	q1_t = ( NumQ.q0*Gyro.x - NumQ.q3*Gyro.y + NumQ.q2*Gyro.z) * HalfTime;
	q2_t = ( NumQ.q3*Gyro.x + NumQ.q0*Gyro.y - NumQ.q1*Gyro.z) * HalfTime;
	q3_t = (-NumQ.q2*Gyro.x + NumQ.q1*Gyro.y + NumQ.q0*Gyro.z) * HalfTime;
	
	NumQ.q0 += q0_t;
	NumQ.q1 += q1_t;
	NumQ.q2 += q2_t;
	NumQ.q3 += q3_t;
	// ��Ԫ����һ��
	NormQuat = Q_rsqrt(squa(NumQ.q0) + squa(NumQ.q1) + squa(NumQ.q2) + squa(NumQ.q3));
	NumQ.q0 *= NormQuat;
	NumQ.q1 *= NormQuat;
	NumQ.q2 *= NormQuat;
	NumQ.q3 *= NormQuat;	


	{
		 	/*��������ϵ�µ�Z��������*/  //����������׼ȷӦ���ǵ�������ϵ�µ�z������
		//float vecxZ = 2 * NumQ.q0 *NumQ.q2 - 2 * NumQ.q1 * NumQ.q3 ;/*����(3,1)��*/
		float vecxZ =  2 * NumQ.q1 * NumQ.q3 - 2 * NumQ.q0 *NumQ.q2 ;//����(3,1)��� ���ģ�������䷽���ˣ�ע�⣺pAngE->pitchҲҪ�ķ���
		//Ҫ��ﵽ������Ч�����ھ�ֹ״̬�£��ɻ���������ʱ����Ȼ������z���ٶȲ��ϱ仯�����ڵ��������£��ɻ���ֱ�ڵ���ļ��ٶ� NormAccz ʼ��Ϊ�������ٶ�ֵ��
		
		float vecyZ = 2 * NumQ.q2 *NumQ.q3 + 2 * NumQ.q0 * NumQ.q1;/*����(3,2)��*/
		float veczZ =  1 - 2 * NumQ.q1 *NumQ.q1 - 2 * NumQ.q2 * NumQ.q2;	/*����(3,3)��*/		 
		
			#ifdef	YAW_GYRO
			*(float *)pAngE = atan2f(2 * NumQ.q1 *NumQ.q2 + 2 * NumQ.q0 * NumQ.q3, 1 - 2 * NumQ.q2 *NumQ.q2 - 2 * NumQ.q3 * NumQ.q3) * RtA;  //yaw
			#else
				//����6ms����һ�Σ�ʵ��2ms����һ��
				//float yaw_G = pMpu->gyroZ * Gyro_G;//��Z����ٶ�������ֵ ת��ΪZ�Ƕ�/��      Gyro_G�����ǳ�ʼ������+-2000��ÿ����1 / (65536 / 4000) = 0.03051756*2		
				//6ms�ڼ��β�����ƽ��ֵ
				float yaw_G = pMpu->gyroZ_sum * Gyro_G; //2ms����ֵ���ܺ�
				yaw_G = yaw_G/pMpu->gyroZ_sum_cnt;//��2ms����ƽ��ֵ
				MPU6050.gyroZ_sum=0;//��λ
				MPU6050.gyroZ_sum_cnt=0;//��λ
					
				if((yaw_G > 1.0f) || (yaw_G < -1.0f)) //����̫С������Ϊ�Ǹ��ţ�����ƫ������,��yaw_G=1��/��, gyroZ=16.384
				//if((yaw_G > 0.2f) || (yaw_G < -0.2f)) //����̫С������Ϊ�Ǹ��ţ�����ƫ������,��yaw_G=1��/��, gyroZ=16.384
				{
						//ע�⣺����΢Сyaw���ٶȣ��ᵼ��yaw��Ư�ƣ��������������YAWƫ����Ư����
						//pAngE->yaw  += yaw_G * dt;//���ٶȻ��ֳ�ƫ���ǣ�
						//ʵ��תһȦֻ��319�����ҡ�����ת��2��
						pAngE->yaw  +=   yaw_G * dt /0.89f; //����Ƕ��������ģ����Գ���+/-360��
				}
				
				//��roll��pitch���ٶȺ�С��ʱ�򡣲��ҵ��ɻ���ֹʱ��yaw���ٶ�ҲӦ��Ϊ0��������Ϊyaw���ٶ���Ҫ��������ֵ
				else if( (ABS(pMpu->gyroX)<10 || ABS(pMpu->gyroY)<10 ) && absFloat(pAngE->pitch)<5.0f && absFloat(pAngE->roll)<5.0f){ 
					
						//���ɻ����ھ�ֹ״̬������ˮƽ��̬��ƫ�Ʋ����ʱ��
						//����ֻ��+/-3�Ľ��ٶȲ���Ư�ƣ�Ҳ��0.18��/���ˣ�һ����10.8�ȣ�6����64.8�����
						//����������Ҫ�Ѿ�ֹ״̬ʱ������������
						if(!ALL_flag.unlock || Aux_Rc.thr<1200 ){
							
								//int16_t gyroZ_tmp = round( (float)pMpu->gyroZ + MPU6050.gyroZ_offset ); //��һ���� mpu6050.c����ִ�У������ʱ����
								int16_t gyroZ_tmp = pMpu->gyroZ; //����������ԭʼֵ

								//����������ƫ����෴ //�����Ǿ�̬����
								MPU6050.gyroZ_offset -= ( (float)gyroZ_tmp )*0.005f;  //����pMpu->gyroZʱ�����������ֵ
							
								//������������
								if(MPU6050.gyroZ_offset>10.0f) MPU6050.gyroZ_offset =10.0f;
								if(MPU6050.gyroZ_offset<-10.0f) MPU6050.gyroZ_offset =-10.0f;
								
								MPU6050.gyroZ_cnt++;
						}
				}
				
				
				if(ALL_flag.unlock && Aux_Rc.thr>1300 ){//������,ƫ������ƫ������ɻ����ĸ�����Ư�ƣ��ͳ����ķ����򲹳���
								//����˳ʱ��Ư�ƣ����ǾͰѲ�������Ϊ������ʱ�벹��������ʱ��Ư�ƣ��ͰѲ�������Ϊ����˳ʱ�벹����
								//����ֵ�����÷ɻ�����Ϊ���෴����ת�������յ�������ƫ�����ٶ�Ư��
								MPU6050.gyroZ_offset1 = -Aux_Rc.offse_yaw; //Aux_Rc.offse_yaw=10��ʾ��ʱ�벹��10
				}
				else{
								MPU6050.gyroZ_offset1 = 0; //ƽʱΪ0��ƽʱΪ��ֵ̬�������������ͨ�� ������У׼��������ң�س���K2������ֵ�ŵ���ͣ�
				}
				
				
				
				//
				
				
			#endif
				
			//�����Ƿ�Χֻ�У�-90~90�����޷����ַɻ����ϻ��ǳ��¡�
			//����ʱ	(-90~90)
			//	F   30��            B -30��
			//	   .             .    
			//	 .    B       F   .
			//����ʱ(-90~90)     
		  //       .     F      B     .
			//	       .              .
		  //     B                     F
			//�����ǿ������ֳ��Ϻͳ��£�����ʱ��0 ~ +/-90��������ʱ(+/-90 ~ +/-180)

			//pAngE->pitch  =  asin(vecxZ)* RtA;	 //������	 0~+/-90	
			pAngE->pitch  =  -asin(vecxZ)* RtA;	 //������	 0~+/-90
				
			//pAngE->tmp = pAngE->pitch;
				
			pAngE->roll	= atan2f(vecyZ,veczZ) * RtA;	//����� 0~+/-180

			NormAccz = pMpu->accX* vecxZ + pMpu->accY * vecyZ + pMpu->accZ * veczZ;	/*Z�ᴹֱ�����ϵļ��ٶȣ���ֵ��������бʱ��Z����ٶȵ������ͣ����ǵ���������Ӧ�ó���ֵ*/				
	
			//�ڵ��������£���ֱ���ٶ�
			//Aux_Rc.debug1 = NormAccz;
			//�ڵ��������£�ˮƽy������ٶȣ���̬��roll���£�
			//Aux_Rc.debug2 = pMpu->accZ * sin(Angle.roll*PI/180.0f) - pMpu->accY*cos(Angle.roll*PI/180.0f);
      
			//1.��̬�Ƿ�����ʱ��Ϊ����˳ʱ��Ϊ����
			//2.������������xyzʵ�ʼ��ٶȷ����෴��������������ͬ

			//��ĳ������������˶�ʱ�����䷴�����и��������й��������������ڲ����Ǽ�������������
			//��������һ����Դ���������������������������һ�£�����ٶȷ����෴�� 
			//ĳ�������ܵ���������Ч���䷴�����ܵ����ٶ�; �����Ͼ�ֹ������z���ܵ��Ĺ���������Ч����̫�����Լ��ٶ�9.8m/s2�����˶�ʱ��״̬
			//����Ϊxyz����������xyz���ٶȷ����෴�������������˻���೯�£�����z�ᳯ��ʱ���Ҽ����˶���������y������Ķ�����ֵ
			
			
			//       ����   x+         ������ٶ�   x-	       ������̬�� p- ��˳ʱ��Ϊ��ֵ��
			//             /                      /                    /
			//      y+ ___/___ y-          y- ___/___ y+        r+ ___/___ r-   
			//           /|                     /|                   /|
			//          / |                    / |                  / |
			//        x-	|                  x+  |                p+  |
			//            z+                     z-                     
			
			//     _________________
			//    | ΢����   ΢���� |
			//    |....... O ......|  ��ĳ���������ڼ����˶����������ڲ��䷴������ܵ�һ�����������������ڲ����Ǽ�������������
			//    |~~~~~~~   ~~~~~ |  �����������Ҽ����˶������߹�������������������ʱ����������y�����������ֵ
      //    |__ѹ��____����.__|

				
			//��� imu.c ��Ҫ�Ǽ���ˢ�½Ƕ�Angle��6msһ�Σ�����ת�Ƶ�mpu6050.c ��MpuGetData()��������ٶȣ�2msһ��
			//����������ˮƽ������ٶȺ��ٶ�
			//float acc_y = MPU6050.accY*cos(Angle.roll*PI/180.0f)-MPU6050.accZ * sin(Angle.roll*PI/180.0f);
			//MPU6050.speedY += acc_y*0.006;//6ms����һ��
	
			//Aux_Rc.debug2=MPU6050.speedY;
			//Aux_Rc.debug5=acc_y;	
			

	}
}

float GetAccz(void)
{
	return NormAccz;
}