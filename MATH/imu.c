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
float NormAccz;  //机体在垂直于地面方向上的加速度（包含重力以及运动加速度）。并非某个轴的加速度，而是通过所有轴的加速度计算而来。

typedef volatile struct {
  float q0;
  float q1;
  float q2;
  float q3;
} Quaternion;


//6ms一次
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

	

	// 提取等效旋转矩阵中的重力分量 
	Gravity.x = 2*(NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);								
	Gravity.y = 2*(NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);						  
	Gravity.z = 1-2*(NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);	
	// 加速度归一化
 NormQuat = Q_rsqrt(squa(MPU6050.accX)+ squa(MPU6050.accY) +squa(MPU6050.accZ));
	
    Acc.x = pMpu->accX * NormQuat;
    Acc.y = pMpu->accY * NormQuat;
    Acc.z = pMpu->accZ * NormQuat;	
 	//向量差乘得出的值
	AccGravity.x = (Acc.y * Gravity.z - Acc.z * Gravity.y);
	AccGravity.y = (Acc.z * Gravity.x - Acc.x * Gravity.z);
	AccGravity.z = (Acc.x * Gravity.y - Acc.y * Gravity.x);
	//再做加速度积分补偿角速度的补偿值
    GyroIntegError.x += AccGravity.x * KiDef;
    GyroIntegError.y += AccGravity.y * KiDef;
    GyroIntegError.z += AccGravity.z * KiDef;
	//角速度融合加速度积分补偿值
    Gyro.x = pMpu->gyroX * Gyro_Gr + KpDef * AccGravity.x  +  GyroIntegError.x;//弧度制
    Gyro.y = pMpu->gyroY * Gyro_Gr + KpDef * AccGravity.y  +  GyroIntegError.y;
    Gyro.z = pMpu->gyroZ * Gyro_Gr + KpDef * AccGravity.z  +  GyroIntegError.z;		
	// 一阶龙格库塔法, 更新四元数

	q0_t = (-NumQ.q1*Gyro.x - NumQ.q2*Gyro.y - NumQ.q3*Gyro.z) * HalfTime;
	q1_t = ( NumQ.q0*Gyro.x - NumQ.q3*Gyro.y + NumQ.q2*Gyro.z) * HalfTime;
	q2_t = ( NumQ.q3*Gyro.x + NumQ.q0*Gyro.y - NumQ.q1*Gyro.z) * HalfTime;
	q3_t = (-NumQ.q2*Gyro.x + NumQ.q1*Gyro.y + NumQ.q0*Gyro.z) * HalfTime;
	
	NumQ.q0 += q0_t;
	NumQ.q1 += q1_t;
	NumQ.q2 += q2_t;
	NumQ.q3 += q3_t;
	// 四元数归一化
	NormQuat = Q_rsqrt(squa(NumQ.q0) + squa(NumQ.q1) + squa(NumQ.q2) + squa(NumQ.q3));
	NumQ.q0 *= NormQuat;
	NumQ.q1 *= NormQuat;
	NumQ.q2 *= NormQuat;
	NumQ.q3 *= NormQuat;	


	{
		 	/*机体坐标系下的Z方向向量*/  //这里描述更准确应该是地面坐标系下的z向量。
		//float vecxZ = 2 * NumQ.q0 *NumQ.q2 - 2 * NumQ.q1 * NumQ.q3 ;/*矩阵(3,1)项*/
		float vecxZ =  2 * NumQ.q1 * NumQ.q3 - 2 * NumQ.q0 *NumQ.q2 ;//矩阵(3,1)项，， 更改，上面这句方向反了，注意：pAngE->pitch也要改方向
		//要求达到这样的效果，在静止状态下，飞机翻滚或俯仰时，虽然陀螺仪z加速度不断变化，但在地面坐标下，飞机垂直于地面的加速度 NormAccz 始终为重力加速度值。
		
		float vecyZ = 2 * NumQ.q2 *NumQ.q3 + 2 * NumQ.q0 * NumQ.q1;/*矩阵(3,2)项*/
		float veczZ =  1 - 2 * NumQ.q1 *NumQ.q1 - 2 * NumQ.q2 * NumQ.q2;	/*矩阵(3,3)项*/		 
		
			#ifdef	YAW_GYRO
			*(float *)pAngE = atan2f(2 * NumQ.q1 *NumQ.q2 + 2 * NumQ.q0 * NumQ.q3, 1 - 2 * NumQ.q2 *NumQ.q2 - 2 * NumQ.q3 * NumQ.q3) * RtA;  //yaw
			#else
				//这里6ms调用一次，实际2ms测量一次
				//float yaw_G = pMpu->gyroZ * Gyro_G;//将Z轴角速度陀螺仪值 转换为Z角度/秒      Gyro_G陀螺仪初始化量程+-2000度每秒于1 / (65536 / 4000) = 0.03051756*2		
				//6ms内几次测量的平均值
				float yaw_G = pMpu->gyroZ_sum * Gyro_G; //2ms测量值的总和
				yaw_G = yaw_G/pMpu->gyroZ_sum_cnt;//求2ms测量平均值
				MPU6050.gyroZ_sum=0;//复位
				MPU6050.gyroZ_sum_cnt=0;//复位
					
				if((yaw_G > 1.0f) || (yaw_G < -1.0f)) //数据太小可以认为是干扰，不是偏航动作,当yaw_G=1度/秒, gyroZ=16.384
				//if((yaw_G > 0.2f) || (yaw_G < -0.2f)) //数据太小可以认为是干扰，不是偏航动作,当yaw_G=1度/秒, gyroZ=16.384
				{
						//注意：丢弃微小yaw角速度，会导致yaw逐渐漂移，解决方法是设置YAW偏航零漂修正
						//pAngE->yaw  += yaw_G * dt;//角速度积分成偏航角，
						//实际转一圈只有319度左右。正反转差2度
						pAngE->yaw  +=   yaw_G * dt /0.89f; //这个角度是连续的，可以超过+/-360度
				}
				
				//当roll和pitch角速度很小的时候。并且当飞机静止时，yaw角速度也应该为0，否则认为yaw角速度需要更新修正值
				else if( (ABS(pMpu->gyroX)<10 || ABS(pMpu->gyroY)<10 ) && absFloat(pAngE->pitch)<5.0f && absFloat(pAngE->roll)<5.0f){ 
					
						//当飞机处于静止状态，并且水平姿态角偏移不大的时候
						//哪怕只有+/-3的角速度测量漂移，也有0.18度/秒了，一分钟10.8度，6分钟64.8度误差
						//所以这里需要把静止状态时真正修正到零
						if(!ALL_flag.unlock || Aux_Rc.thr<1200 ){
							
								//int16_t gyroZ_tmp = round( (float)pMpu->gyroZ + MPU6050.gyroZ_offset ); //这一句在 mpu6050.c里面执行，这边临时测试
								int16_t gyroZ_tmp = pMpu->gyroZ; //经过修正的原始值

								//修正方向于偏差方向相反 //仅仅是静态修正
								MPU6050.gyroZ_offset -= ( (float)gyroZ_tmp )*0.005f;  //计算pMpu->gyroZ时，会加上修正值
							
								//限制修正幅度
								if(MPU6050.gyroZ_offset>10.0f) MPU6050.gyroZ_offset =10.0f;
								if(MPU6050.gyroZ_offset<-10.0f) MPU6050.gyroZ_offset =-10.0f;
								
								MPU6050.gyroZ_cnt++;
						}
				}
				
				
				if(ALL_flag.unlock && Aux_Rc.thr>1300 ){//飞行中,偏航测量偏差补偿，飞机朝哪个方向漂移，就朝他的反方向补偿。
								//比如顺时针漂移，我们就把补偿设置为正（逆时针补偿），逆时针漂移，就把补偿设置为负（顺时针补偿）
								//补偿值，能让飞机误以为朝相反方向转动，最终得以修正偏航角速度漂移
								MPU6050.gyroZ_offset1 = -Aux_Rc.offse_yaw; //Aux_Rc.offse_yaw=10表示逆时针补偿10
				}
				else{
								MPU6050.gyroZ_offset1 = 0; //平时为0，平时为静态值，如果有误差可以通过 陀螺仪校准来消除，遥控长按K2（油门值放到最低）
				}
				
				
				
				//
				
				
			#endif
				
			//俯仰角范围只有（-90~90），无法区分飞机朝上还是朝下。
			//朝上时	(-90~90)
			//	F   30度            B -30度
			//	   .             .    
			//	 .    B       F   .
			//朝下时(-90~90)     
		  //       .     F      B     .
			//	       .              .
		  //     B                     F
			//翻滚角可以区分朝上和朝下，朝上时（0 ~ +/-90），朝下时(+/-90 ~ +/-180)

			//pAngE->pitch  =  asin(vecxZ)* RtA;	 //俯仰角	 0~+/-90	
			pAngE->pitch  =  -asin(vecxZ)* RtA;	 //俯仰角	 0~+/-90
				
			//pAngE->tmp = pAngE->pitch;
				
			pAngE->roll	= atan2f(vecyZ,veczZ) * RtA;	//横滚角 0~+/-180

			NormAccz = pMpu->accX* vecxZ + pMpu->accY * vecyZ + pMpu->accZ * veczZ;	/*Z轴垂直方向上的加速度，此值涵盖了倾斜时在Z轴角速度的向量和，不是单纯重力感应得出的值*/				
	
			//在地球坐标下，垂直加速度
			//Aux_Rc.debug1 = NormAccz;
			//在地球坐标下，水平y方向加速度（姿态角roll导致）
			//Aux_Rc.debug2 = pMpu->accZ * sin(Angle.roll*PI/180.0f) - pMpu->accY*cos(Angle.roll*PI/180.0f);
      
			//1.姿态角方向，逆时针为正，顺时针为负，
			//2.惯性力方向：与xyz实际加速度方向相反，与重力方向相同

			//当某个轴正向加速运动时，在其反方向有个假想力叫惯性力，陀螺仪内部就是检测这个惯性力。
			//惯性力另一个来源是重力，重力方向与惯性力方向一致，与加速度方向相反。 
			//某个方向受到重力，等效于其反方向受到加速度; 地球上静止的物体z轴受到的惯性力，等效于在太空中以加速度9.8m/s2向上运动时的状态
			//以下为xyz重力方向，与xyz加速度方向相反。举例：当无人机左侧朝下，或者z轴朝下时向右加速运动，陀螺仪y轴输出的都是正值
			
			
			//       重力   x+         机体加速度   x-	       机体姿态角 p- （顺时针为负值）
			//             /                      /                    /
			//      y+ ___/___ y-          y- ___/___ y+        r+ ___/___ r-   
			//           /|                     /|                   /|
			//          / |                    / |                  / |
			//        x-	|                  x+  |                p+  |
			//            z+                     z-                     
			
			//     _________________
			//    | 微弹簧   微弹簧 |
			//    |....... O ......|  当某个轴正向在加速运动，陀螺仪内部其反方向会受到一个惯性力，陀螺仪内部就是检测这个惯性力。
			//    |~~~~~~~   ~~~~~ |  举例：当朝右加速运动，或者惯性力（含重力）朝左时，陀螺仪在y轴输出的是正值
      //    |__压缩____拉伸.__|

				
			//这个 imu.c 主要是计算刷新角度Angle，6ms一次，我们转移到mpu6050.c 的MpuGetData()里面积分速度，2ms一次
			//新增，测试水平方向加速度和速度
			//float acc_y = MPU6050.accY*cos(Angle.roll*PI/180.0f)-MPU6050.accZ * sin(Angle.roll*PI/180.0f);
			//MPU6050.speedY += acc_y*0.006;//6ms调用一次
	
			//Aux_Rc.debug2=MPU6050.speedY;
			//Aux_Rc.debug5=acc_y;	
			

	}
}

float GetAccz(void)
{
	return NormAccz;
}