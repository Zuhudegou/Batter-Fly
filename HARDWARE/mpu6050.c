/**********************STM32 开源无人机*******************************************************/
//  V1.0 开源作者：小南&zin；日期：2016.11.21
//           STM32F103C8飞控以及遥控基础功能以及核心代码实现；
//  V2.0 开源作者：小刘；日期：2020.05.17
//           scheduler任务架构调整，增加屏幕以及气压计，新增PID在线调整功能；
//  V3.0 开源作者：zhibo_sz&sunsp；日期：2024.06.01
//           新增一键定高起飞，悬停运动控制以及刹车优化，气压遥控屏幕陀螺仪等模块优化，新增无刷电机；
/********************************************************************************************/

//声明：
//      本程序仅对购机用户开源，学习使用，所有权归以上作者所有；
//      未经许可，不得传阅、转载、公开、转卖本代码。


#include "ALL_DATA.h"
#include "mpu6050.h"
#include "I2C.h"
#include "filter.h"
#include <string.h>
#include "LED.h"
#include "myMath.h"
#include "kalman.h"
#include "ANO_Data_Transfer.h"
#include "flash.h" //新增，保存陀螺仪校准参数


#define	SMPLRT_DIV		0x19	//陀螺仪采样率，典型值：0x07(125Hz)
#define	CONFIGL			0x1A	//低通滤波频率，典型值：0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
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
#define	PWR_MGMT_1		0x6B	//电源管理，典型值：0x00(正常启用)
#define	WHO_AM_I		  0x75	//IIC地址寄存器(默认数值0x68，只读)
#define MPU6050_PRODUCT_ID 0x68 //还有一种很老的丝印一样，但id是0x98，那种效果不好。
#define MPU6052C_PRODUCT_ID 0x72

//#define BMI160_PRODUCT_ID 		0xD1
#define ICM42688_PRODUCT_ID			0x47	

//ICM42688 寄存器
#define ICM42688_DEVICE_CONFIG             0x11				//软件重启
#define ICM42688_GYRO_ACCEL_CONFIG0        0x52				//设置低通滤波器
#define ICM42688_GYRO_CONFIG0              0x4F				//设置量程和速率
#define ICM42688_ACCEL_CONFIG0             0x50				//设置量程和速率
#define ICM42688_PWR_MGMT0                 0x4E				//设置工作模式
#define ICM42688_INTF_CONFIG0              0x4C

//加速度量程
#define AFS_2G  0x03
#define AFS_4G  0x02
#define AFS_8G  0x01
#define AFS_16G 0x00  // default
//角速度量程
#define GFS_2000DPS   0x00 // default
#define GFS_1000DPS   0x01
#define GFS_500DPS    0x02
#define GFS_250DPS    0x03
#define GFS_125DPS    0x04
#define GFS_62_5DPS   0x05
#define GFS_31_25DPS  0x06
#define GFS_15_625DPS 0x07



//#define   MPU6050_is_DRY()      GPIO_ReadOutBit(HT_GPIOC, GPIO_PIN_0)//IRQ主机数据输入
#ifdef  	USE_I2C_HARDWARE
		#define MPU6050_ADDRESS 0xD0//0x68
		
#else
		#define  MPU6050_ADDRESS 0xD0   //IIC写入时的地址字节数据，+1为读取
		
		#define ICM42688_ADDRESS_H  0xD2 //高电平时
		#define ICM42688_ADDRESS_L  0xD0 //低电平时
		
		#define ICM42688_ADDRESS(T)  ( (T)==ICM42688_H? (ICM42688_ADDRESS_H):(ICM42688_ADDRESS_L) ) //	
		
#endif


//传感类型 sensor_type
#define MPU6050_L 1				//PCB背面，MPU6050或者模块在pcb背面，ADO为低电平
#define ICM42688_H 2			//PCB背面，ICM42688模块安装在pcb背面，ADO为高电平
#define ICM42688_L 3			//PCB正面，ICM42688贴片在pcb正面，ADO为低电平


int16_t MpuOffset[7] = {0}; //6个校准值，前3个加速度，后3个角速度; 最后一个，是否校准过

static volatile int16_t *pMpu = (int16_t *)&MPU6050;
//u8 id_tmp;//记录芯片id，用于调试

uint8_t product_id=0;  //记录芯片id，用于调试
uint8_t sensor_type=0; //自动判断芯片类型，用于xyz数据读取以及方向处理

/****************************************************************************************
*@brief  
*@brief   
*@param[in]
*****************************************************************************************/
int8_t mpu6050_rest(void) //没有用到
{
	if(IIC_Write_One_Byte(MPU6050_ADDRESS, PWR_MGMT_1, 0x80) == FAILED)
		return FAILED;	//复位
	delay_ms(20);
	return SUCCESS;
}


/****************************************************************************************
*@brief   
*@brief  
*@param[in]
*****************************************************************************************/
int8_t MpuInit(void) //初始化
{
	uint8_t date = SUCCESS;
	
	//如果陀螺仪一开始就失败，只亮右边蓝灯
	fLED_L();	 	//前灯亮，右蓝
	hLED_H();		//前灯灭，左蓝
	
	bLED_H();		//后灯灭，右红
	aLED_H();		//后灯灭，左红
	
	 
	
	
	//陀螺仪复位
	delay_ms(10);
	IIC_Write_One_Byte(MPU6050_ADDRESS, PWR_MGMT_1, 0x80);	//复位
	//IIC_Write_One_Byte(BMI160_ADDRESS, 0x7e, 0xb6);		//复位，复位时间15ms，
	IIC_Write_One_Byte(ICM42688_ADDRESS_H, ICM42688_DEVICE_CONFIG, 0x01); //软复位传感器（此后需要至少等待1ms）
	IIC_Write_One_Byte(ICM42688_ADDRESS_L, ICM42688_DEVICE_CONFIG, 0x01); //软复位传感器（此后需要至少等待1ms）
	
	delay_ms(10);
	
	
	
	//判断陀螺仪芯片类型
	product_id=IIC_Read_One_Byte(MPU6050_ADDRESS, 0x75);

	if( product_id == MPU6050_PRODUCT_ID ){ //0x68
			//当前陀螺仪为 MPU6050
			sensor_type=MPU6050_L; //1
	}
	else if( product_id == ICM42688_PRODUCT_ID ){ //0x47	
			//当前陀螺仪为 ICM42688, 且安装在PCB正面（ADO为低电平）
			sensor_type=ICM42688_L; //3
	}
	else{
			product_id=0;
			product_id=IIC_Read_One_Byte(ICM42688_ADDRESS_H, 0x75);
			if(product_id == ICM42688_PRODUCT_ID){//0x47	
					//当前陀螺仪为 ICM42688, 且安装在PCB背面（ADO为高电平）
					sensor_type=ICM42688_H; //2
			}
			else{
					sensor_type=0;//未找到传感
					//product_id = 0; //产品id不符合
			}
	}
	//while(1){};
	//**************************************大致的陀螺仪型号在无人机上的使用总结********************************************************
	// 性能排序：BMI160(Bosch) << BMI270 = MPU6050(InvenSense) < ICM20602(InvenSense) < ICM42688 << ICM45686 < ADXL355
	// 博世的 BMI160 最差，数据不稳定，但价格最便宜（只要3块多），封装为 LGA14，省空间，玩具无人机多选用这种；
	// 应美盛的 MPU6050 还算稳定，资料和开源代码最为丰富，但开发年代较早，只有I2C接口，焊接高温也易导致不稳定，锡膏建议选择180度，贴片温度210度。
	// BMI系列最贵的BMI088(BMI160的2~3倍价格，与ICM42688相当)听说效果也不错，不过封装比较特殊（LGA16-长条形），无现成模块，尚未测试。
	// ICM42688 封装也是LGA14，比MPU6050贵一些，管脚差不多与博世多款BMI兼容，各项指标较好，而且抗震效果比较好，很多国外大牌开源无人机使用(BMI088也不少)；
  // ICM45686 价格较高，单芯片就接近20元，性能非常好，接近于价格过百的ADXL355，无人机上选用的不多；
	// ADXL355  多用于工业级设备，已经不属于低成本IMU，价格在一百多元，体积也很大，无人机未看到使用案例。
	// 以上各型号六轴陀螺仪，性能有逐步提升，价格也是有相应提升。
	//********************************************************************************************************************************
	
	
	//陀螺仪 初始化设置
	MPU_Err = 1;	//默认设置不成功	
	
	
	if( sensor_type == MPU6050_L ){
						
					do
					{
							//hLED_Toggle();		//前左灯 闪，
							
							//陀螺仪模块直接5v，不需要延迟初始化了。
							//delay_ms(500); //电源拨动开关抖动比较厉害，以及各个模块以及电容上电，要多等一会，不然个别即使能识别芯片id也会持续性的数据异常。
							//delay_ms(100); //新增每次解锁时，初始化陀螺仪。
							date = IIC_Write_One_Byte(MPU6050_ADDRESS, PWR_MGMT_1, 0x80);	//复位
						
							hLED_L();		//前左灯 亮，
							delay_ms(1);
						
							hLED_H();		//前左灯 灭，
							delay_ms(50);
						
							date += IIC_Write_One_Byte(MPU6050_ADDRESS, SMPLRT_DIV, 0x02); //陀螺仪采样率，0x00(500Hz)，2ms更新一次数据
							date += IIC_Write_One_Byte(MPU6050_ADDRESS, PWR_MGMT_1, 0x03);	//设置设备时钟源，陀螺仪Z轴
							date += IIC_Write_One_Byte(MPU6050_ADDRESS, CONFIGL, 0x03);   //低通滤波频率，0x03(42Hz)
							date += IIC_Write_One_Byte(MPU6050_ADDRESS, GYRO_CONFIG, 0x18);//+-2000deg/s
							date += IIC_Write_One_Byte(MPU6050_ADDRESS, ACCEL_CONFIG, 0x09);//+-4G
							MPU_Err = 1;	
					}
					while(date != SUCCESS);
	
					
					
	}
	
	
	
	else if( sensor_type == ICM42688_L || sensor_type == ICM42688_H ){	
				//以下设置项目来源于官方规格书，【DS-000347-ICM-42688-P-v1.7.pdf】

				do
				{

						//IIC_Write_One_Byte(BMI160_ADDRESS, 0x7e, 0xb6);		//复位，复位时间15ms，复位后才能正确读写寄存器，工作模式变为Suspend（0），
						IIC_Write_One_Byte(ICM42688_ADDRESS(sensor_type), ICM42688_DEVICE_CONFIG, 0x01); //软复位传感器（此后需要至少等待1ms）
					
					
						hLED_L();		//前左灯 亮，
						delay_ms(15);
					
						hLED_H();		//前左灯 灭，
						delay_ms(50);
					
					
					  uint8_t date_tmp =0xFF;
						date = SUCCESS;//0
		
		
						 //1. 设置低通滤波，滤除高频振动，默认信号通过带宽125Hz（ODR=500Hz时），对于角速度是适合的，但对于加速度太大了，80页
						 //ICM42688_GYRO_ACCEL_CONFIG0
						//IIC_Write_One_Byte(ICM42688_ADDRESS, ICM42688_GYRO_ACCEL_CONFIG0, 0x44);  //加速度和角速度 低通滤波器都选择通过带宽0~25Hz，滤除高频
						//加速度值非常容易被振动影响，电机是振动源，滤波效果好一点才行。角速度对与振动不敏感，但需要及时性很好，不用滤波太好，防止角速度延迟。
						//所以加速度与角速度分别选择12.5Hz和50Hz, 这个非常关键，否则电机或螺旋桨动平衡或振动干扰只要稍微差一点，都会导致悬停效果不好。
						//IIC_Write_One_Byte(ICM42688_ADDRESS(sensor_type), ICM42688_GYRO_ACCEL_CONFIG0, 0x73);  //加速度与角速度 低通滤波器分别选择通过带宽12.5Hz以及50Hz，超过这个频率将会滤除
						IIC_Write_One_Byte(ICM42688_ADDRESS(sensor_type), ICM42688_GYRO_ACCEL_CONFIG0, 0x71);    //加速度与角速度 低通滤波器分别选择通过带宽12.5Hz以及125Hz，超过这个频率将会滤除
						//角速度不能选择太低频率的低通滤波，会导致姿态角信号滞后严重，引起PID测量滞后振荡（会看到机身像蝴蝶翅膀一样的振动）
						//抖动常见影响因素。1：陀螺仪自身性能以及参数；2，PID参数；3：电机动平衡；4：螺旋桨变形或破损； 5：电路板以及陀螺仪隔振效果；
						
						//是否设置成功
						date_tmp=IIC_Read_One_Byte(ICM42688_ADDRESS(sensor_type), ICM42688_GYRO_ACCEL_CONFIG0); //查询
						//product_id=date_tmp;//用于调试
						if(date_tmp!=0x71){ //低通滤波器分别为12.5Hz，以及125Hz
								date=1; 
						}
						
						
						

						
						//2. 设置加速度的量程和测量速度，，，78页
						date_tmp = IIC_Read_One_Byte(ICM42688_ADDRESS(sensor_type), ICM42688_ACCEL_CONFIG0);  //查询
						date_tmp |= (AFS_4G << 5);   						//量程 ±4g，默认16g，改为4克
						date_tmp |= (0x0F);     								//输出ODR速率 500HZ
						IIC_Write_One_Byte(ICM42688_ADDRESS(sensor_type), ICM42688_ACCEL_CONFIG0, date_tmp);
						//是否设置成功
						date_tmp=IIC_Read_One_Byte(ICM42688_ADDRESS(sensor_type), ICM42688_ACCEL_CONFIG0); //查询
						//product_id=date_tmp;//用于调试
						if(date_tmp!=0x4F){ //+/-4g，500hz
								date=2;
						}
						
						
						
						
						//3. 设置角速度的量程和测量速度，，，77页
						date_tmp=IIC_Read_One_Byte(ICM42688_ADDRESS(sensor_type), ICM42688_GYRO_CONFIG0);  //查询
						date_tmp |= (GFS_2000DPS << 5);   				//量程 +-2000deg/s，就是默认值
						date_tmp |= (0x0F);     									//输出ODR速率 500HZ
						IIC_Write_One_Byte(ICM42688_ADDRESS(sensor_type), ICM42688_GYRO_CONFIG0, date_tmp);
						
						//是否设置成功
						date_tmp=IIC_Read_One_Byte(ICM42688_ADDRESS(sensor_type), ICM42688_GYRO_CONFIG0); //查询
						//product_id=date_tmp;//用于调试
						if(date_tmp!=0x0F){ //+-2000deg/s，500hz
								date=3;
						}




						//4. 设置工作模式 76页
						date_tmp=IIC_Read_One_Byte(ICM42688_ADDRESS(sensor_type), ICM42688_PWR_MGMT0);  //查询
						date_tmp &= ~(1 << 5);								//使能温度测量, 0表示温度使能，也是默认值
						date_tmp |= ((3) << 2);								//设置GYRO_MODE  0:关闭 1:待机 2:预留 3:低噪声
						date_tmp |= (3);										//设置ACCEL_MODE 0:关闭 1:关闭 2:低功耗 3:低噪声
						IIC_Write_One_Byte(ICM42688_ADDRESS(sensor_type), ICM42688_PWR_MGMT0, date_tmp);
						
						//是否设置成功
						date_tmp=IIC_Read_One_Byte(ICM42688_ADDRESS(sensor_type), ICM42688_PWR_MGMT0); //查询
						//product_id=date_tmp;//用于调试
						if(date_tmp!=0x0F){ //温度+A低噪+G低噪
								date=4;
						}
						
						
						
						
						//5. 重启没有数据时，显示上次测量值，防止显示为 0xFFFF(-32768),,,,,FIFO_HOLD_LAST_DATA_EN=1,,,,74页 
						//我们解锁飞控时，remote.c 会调用一下 重设i2c 以及 陀螺仪重启函数
						//如果没有这项设置，在我们解锁飞控时，陀螺仪重启阶段，读出的数据为 0xFFFF，计算出来的姿态角会有异常。
						date_tmp=IIC_Read_One_Byte(ICM42688_ADDRESS(sensor_type), ICM42688_INTF_CONFIG0);  //查询
						date_tmp |= (0x80);										//最高位置1
						IIC_Write_One_Byte(ICM42688_ADDRESS(sensor_type), ICM42688_INTF_CONFIG0, date_tmp);
						
						//是否设置成功
						date_tmp=IIC_Read_One_Byte(ICM42688_ADDRESS(sensor_type), ICM42688_INTF_CONFIG0); //查询
						//product_id=date_tmp;//用于调试
						if( (date_tmp&0x80)!=0x80 ){ //最高位是否为1
								date=5;
						}
						
						
							
						delay_ms(5);			//操作完PWR—MGMT0寄存器后 200us内不能有任何读写寄存器的操作
			
						
						//product_id=date;//用于调试, 记录哪步出错
				}
				while(date != SUCCESS); //设置成功才往下走
				
	}
	
	
	
	//陀螺仪型号不符合，停住，不继续往下
	else{
				do
				{
						hLED_L();		//前左灯 亮，
						delay_ms(10);
					
						hLED_H();		//前左灯 灭，
						delay_ms(100);
					
					
				}
				while(1); //设置成功才往下走
	
	}
	
			
					
	//陀螺仪通讯ok，2个蓝灯亮
	fLED_L();	 	//前灯亮，右
	hLED_L();		//前灯亮，左

	
	//date = IIC_Read_One_Byte(MPU6050_ADDRESS, 0x75);
	
	//芯片确认失败，只亮左边红灯。
	//if( date!= MPU6050_PRODUCT_ID ) //读取芯片型号，确认芯片是否正确识别
	if( sensor_type==0 ) 	
	//if(date!= 0x98) //最新的MPU6050产品id为0x68, 很早期的MPU6050为0x98, 也能用，但效果差很远，而且已停产很多年，有些商家会重新丝印冒充最新的，价格会比正常的低一些。
	{
				//这里会被 后面的无线模块灯状态覆盖。
				
				//fLED_H(); //右上角蓝灯熄灭，表示陀螺仪失败（左上角蓝灯是亮的），蓝灯交替闪烁，而不是同步闪烁
				return FAILED;
	}	

	
	//MpuGetOffset(); //开机取消MPU6050校准，长按遥控器key2可以随时校准（非解锁模式，先把油门打到最低）
	FLASH_read(MpuOffset,7);//从mcu的FLASH中读取MPU6050的水平静止标定校准值
			
	
	MPU_Err = 0;
	

	
/*
	//测试
	while(1){
			delay_ms(2); //模拟每隔2ms读取一次数据
			MpuGetData();

		//测试陀螺仪可靠性，加速度012，角速度345
		if(  		MPU6050.gyroX>30 || MPU6050.gyroX<-30
				 || MPU6050.gyroY>30 || MPU6050.gyroY<-30
				 || MPU6050.gyroZ>30 || MPU6050.gyroZ<-30
		){
				//正常情况，在静止状态，不应该进入这里，
				//如果静止状态，这个计数在增加，说明通讯被干扰了，或者陀螺仪被干扰了。
				product_id++; //
		
		}
		
		if(MPU6050.accZ<1000){
				product_id=100;
		
		}

	}

*/

	
	return SUCCESS;

	
}


//校准+保存，
//长按遥控器key2触发（非解锁模式，先把油门打到最低）
void MpuGetOffset_save(void) //用户主动校准才会保存到flash
{
	
	MpuGetOffset(); //这个开机有时也要执行，不要包含保存flash动作（影响flash寿命）
	
	MpuOffset[6]=1;//已经校准，0~5为6个校准数据，最后一位为新的mcu是否已经校准mpu6050并保存
	
	FLASH_write(MpuOffset,7);//将数据写到FLASH中，一共有6个int16数据

	
}





/****************************************************************************************
*@brief    
*@brief   
*@param[in]
*****************************************************************************************/

#define  Acc_Read_6050() IIC_read_Bytes(MPU6050_ADDRESS, 0X3B, buffer, 6 )
#define  Gyro_Read_6050() IIC_read_Bytes(MPU6050_ADDRESS, 0x43, &buffer[6], 6 )

#define	Acc_Read_ICM42688(T)	IIC_read_Bytes(ICM42688_ADDRESS(T), 0x1F, buffer, 6 ) 	//前6个字节放3轴加速度
#define	Gyro_Read_ICM42688(T)	IIC_read_Bytes(ICM42688_ADDRESS(T), 0x25, &buffer[6], 6 );	//后6个字节放3轴角速度

u8 MPU_empty_cnt;//连续次数
u8 MPU_empty_cnt_all;//总次数

//2ms调用一次，pMpu数组内存地址与MPU6050开始地址一样
//读取周期2ms，解算周期6ms，也就是利用陀螺仪最新数据来解算。
void MpuGetData(void) //读取陀螺仪数据加滤波
{
	  uint8_t i;
    uint8_t buffer[12];
		memset(buffer, 0, 12);  //16字节初始化为0
	
		if( sensor_type == MPU6050_L ){
					Acc_Read_6050();
					Gyro_Read_6050();
		}

		else if( sensor_type == ICM42688_L || sensor_type == ICM42688_H){	
					Acc_Read_ICM42688(sensor_type);
					Gyro_Read_ICM42688(sensor_type);
		}
		

		

		//8位数据合并为16位数据
		for(i=0;i<6;i++)  //3个加速度，3个角速度
		{
			//原始值
			pMpu[i] = (((int16_t)buffer[i<<1] << 8) | buffer[(i<<1)+1]);	
		}
	
		
		//XY对调（仅仅针对ICM42688在PCB背面的情况）	//x(0,3) y(1,4) 
		if( sensor_type == ICM42688_H ){	
					int16_t tmp=0;
					tmp=pMpu[0]; pMpu[0]=pMpu[1]; pMpu[1]=tmp; //xy加速度对调
					tmp=pMpu[3]; pMpu[3]=pMpu[4]; pMpu[4]=tmp; //xy角速度对调
		}
		
	
		//如果3个轴重力加速度都为零或者都为-1(0xFFFF)，都说明数据异常
		//更改闪灯状态，右上蓝灯熄灭
		//fLED_H(); //2，蓝，右上，熄灭
		//当陀螺仪失效，数据：加速度 -233， -138， 2762, 角速度 29，7，0，
		//当时的offset 数据：及速度  234， 139，-2763，角速度 -29，-7，1
		//当陀螺仪失效，数据：加速度 -234， -130， 2762, 角速度 24，6，0，
		//当时的offset 数据：及速度  234， 139，-2763，角速度 -29，-7，1
		
		
		
		if(MPU_Err==0 && SysTick_count>1000 ){//刚开机，或者陀螺仪异常时，不判断数据有效性
				if(ABS(pMpu[0])<=1 && ABS(pMpu[1])<=1 && ABS(pMpu[2])<=1){//因为存在加速度重力分量，不管飞机姿态如何，不可能3个轴加速度同时很小。
				//if(ABS(pMpu[0])<=1 && ABS(pMpu[1])<=1 && ABS(pMpu[2])<=1 && SysTick_count>1000 ){//因为存在加速度重力分量，不管飞机姿态如何，不可能3个轴加速度同时很小。

							MPU_empty_cnt_all++; //异常数据总次数，方便调试
							MPU_empty_cnt++; //连续异常数据的次数，如果时bmi088陀螺仪(数据频率设为400Hz)，但我们每2ms读取一次，不是每次都能读取到，但只要连续次数不超过1，就不算异常。
							//fLED_H(); //2，蓝，右上，熄灭 ， 右上蓝灯与其它灯不同，表示陀螺仪数据异常。
							//有时i2c信号受干扰，比如人手碰触到了主板或线，也会偶尔暂时异常；飞行中一般不会有这么大的干扰；icm42688比mpu6050更容易受人手碰触干扰；可能是更省电导致信号驱动能力弱了
							if(MPU_empty_cnt>0){//2 ， 右上蓝灯灭亮，左上蓝灯亮，表示陀螺仪在正常读取阶段数据异常。（初始化阶段是左上蓝灯闪）
											//确保两个蓝灯状态不同，表示陀螺仪数据异常。
											fLED_H(); //右上
											hLED_L(); //左上
							} 
							//遇到错误数据，应该要废弃本次数据。连续废弃次数达到一定程序再通过LED显示出来。连续废弃次数2以下是很正常的，因为采样率500Hz，每2ms读取一次不一定正好有数据
							//以下是个别老版本MPU6050模块的问题（可能是模块供应商用了二手ic或者焊接温度过高导致，更换新的ic就解决了）
							//有时掉电很久第一次开机时，陀螺仪能识别到，但是数据全是0或者F，
							//这时需要指示灯显示异常，飞控板再重新开机即可。

				}
				else{
							if(MPU_empty_cnt>0){ //刚从异常中恢复，此时左右蓝灯不同步，
								
									//闪灯状态恢复，遥控解锁后或者按一下LOCK，灯都能恢复。
									MPU_empty_cnt=0; //连续次数
							}
							
				
				}
				
		}
		
	
	
	
		for(i=0;i<6;i++)  //3个加速度，3个角速度
		{
			
			//原始值,已经移到上一步，要判断一下原始值是否有效
			//pMpu[i] = (((int16_t)buffer[i<<1] << 8) | buffer[(i<<1)+1]);	

			
			//MPU 贴片位置，从正面改到背面
			//0,3->x,,,1,4->y,,,,2,5->Z
			//if(i==2||i==5 || i==0 || i==3) pMpu[i]= -pMpu[i];
			//if(i==5) pMpu[i]= -pMpu[i];
			
			//y(1,4) z(2,5)转换方向，要放在补偿之前。
			//if(i==2||i==5 || i==1 || i==4) pMpu[i]= -pMpu[i];
			//if(i==2) pMpu[i] = -pMpu[i]; //转换方向，要放在补偿之前。
			
			
			//方向转换，MPU6050在PCB背面
			if( sensor_type == MPU6050_L ){   //背面 
						//z(2,5)转换方向，要放在补偿之前。
						if(i==2||i==5 ) pMpu[i]= -pMpu[i];
						
						//y(1,4) 转换方向，要放在补偿之前。
						if(i==1||i==4 ) pMpu[i]= -pMpu[i];
						
						//x(0,3) 转换方向，要放在补偿之前。
						//if(i==0||i==3 ) pMpu[i]= -pMpu[i];
			}

			//方向转换，ICM42688在PCB正面
			else if( sensor_type == ICM42688_L){	 //正面
						//z(2,5)转换方向，要放在补偿之前。
						//if(i==2||i==5 ) pMpu[i]= -pMpu[i];
						
						//y(1,4) 转换方向，要放在补偿之前。
						if(i==1||i==4 ) pMpu[i]= -pMpu[i];
						
						//x(0,3) 转换方向，要放在补偿之前。
						if(i==0||i==3 ) pMpu[i]= -pMpu[i];
						
			}
			
			//方向转换，ICM42688在PCB背面面
			else if( sensor_type == ICM42688_H ){	 //背面
						//z(2,5)转换方向，要放在补偿之前。
						if(i==2||i==5 ) pMpu[i]= -pMpu[i];
						
						//y(1,4) 转换方向，要放在补偿之前。
						if(i==1||i==4 ) pMpu[i]= -pMpu[i];
						
						//x(0,3) 转换方向，要放在补偿之前。
						if(i==0||i==3 ) pMpu[i]= -pMpu[i];
			}
				
			

			
			
			//陀螺仪静止校准的补偿
			pMpu[i] = pMpu[i] - MpuOffset[i];		//补偿
			
			

			
			//当前为偏航角速度，再做2次修正补偿（静止零漂和飞行中零漂）
			if(i==5){
					MPU6050.gyroZ_raw=pMpu[i]; //保留旧值
					//进一步修正z偏航角速度，消除微小漂移，要对结果四舍五入；直接赋值是向下取整
					pMpu[i] = round( (float)pMpu[i] +  MPU6050.gyroZ_offset ); //静止状态全局偏差 +/-10以内，这个值自动修正（在imu.c）
					pMpu[i] = round( (float)pMpu[i] +  MPU6050.gyroZ_offset1 ); //飞行中测量偏差 ，用户设定YAW漂移修正+/-128以内
			}

			
			
			
			if(i < 3) //=0 1 2，加速度 滤波
			{
				{
					static struct _1_ekf_filter ekf[3] = {{0.02,0,0,0,0.001,0.543},{0.02,0,0,0,0.001,0.543},{0.02,0,0,0,0.001,0.543}};	
					kalman_1(&ekf[i],(float)pMpu[i]);  //一维卡尔曼
					pMpu[i] = (int16_t)ekf[i].out;
				}
			}
				
			
			if(i > 2) //=3 4 5, 角速度 滤波
			{
				uint8_t k=i-3; //k:0,1,2
				const float factor = 0.15f;  //滤波因素			
				static float tBuff[3];//记录上次角速度值

				//pMpu[i] = tBuff[k] = tBuff[k] * (1 - factor) + pMpu[i] * factor;

				tBuff[k] = tBuff[k]+ ( (float)pMpu[i] - tBuff[k] ) * factor; //滤波
				pMpu[i] = round(tBuff[k]);
				
			}
			
			
			//当前为偏航角速度，再做2次修正补偿（静止零漂和飞行中零漂）
			if(i==5){ //给imu.c 做角度积分，6ms一次
						MPU6050.gyroZ_sum += pMpu[i];
						MPU6050.gyroZ_sum_cnt++;
				
			}
			
			
		
	}
		
	

	
	
		
			//在地球坐标下，垂直加速度
			//Aux_Rc.debug1 = NormAccz;
			//在地球坐标下，水平y方向加速度（姿态角roll导致）
			//Aux_Rc.debug2 = pMpu->accZ * sin(Angle.roll*PI/180.0f) - pMpu->accY*cos(Angle.roll*PI/180.0f);
      
			//以下为xyz重力方向，与xyz加速度方向相反。举例：当无人机左侧朝下，或者z轴朝下时向右加速运动，陀螺仪y轴输出的都是正值
	
			//       重力   x+         机体加速度   x-	       机体姿态角 p- （顺时针为负值）
			//             /                      /                    /
			//      y+ ___/___ y-          y- ___/___ y+        r+ ___/___ r-   
			//           /|                     /|                   /|
			//          / |                    / |                  / |
			//        x-	|                  x+  |                p+  |
			//            z+                     z-                     
			//判断陀螺仪方向是否正确：当某轴重力方向指向地面时，该轴的加速度应该显示位 8192，
			//一定要注意：重力方向与加速度方向相反，重力方向也是惯性力的方向；某轴在超正向加速运动时，在其反方向会产生一个假想力（惯性力）
	
	/*
			float acc_y = MPU6050.accY*cos(Angle.roll*PI/180.0f)-MPU6050.accZ * sin(Angle.roll*PI/180.0f);
	
			MPU6050.speedY += acc_y*0.002;//2ms调用一次
	
			Aux_Rc.debug2=MPU6050.speedY;
			Aux_Rc.debug5=acc_y;	
			*/
	
}

/****************************************************************************************
*@brief   get mpu offset
*@brief   initial and cmd call this
*@param[in]
*****************************************************************************************/
//这里是静态校准，但飞机在飞行过程，z角速度仍然有轻微的误测，导致飞行几分钟之后有一些误差。

void MpuGetOffset(void) //校准
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
	
	
	
	TIM_ITConfig(  //使能或者失能指定的TIM中断
				TIM3, //TIM2
				TIM_IT_Update ,
				DISABLE  //使能
	);	
	
	
	//等待飞机静止下来
	while(k--)//30次静止则判定飞行器处于静止状态
	{
				do
				{
							delay_ms(10);
							MpuGetData();
							for(i=0;i<3;i++) //得到相邻2次，后3个数据（角速度）的差异。
							{
										ErrorGyro[i] = pMpu[i+3] - LastGyro[i];
										LastGyro[i] = pMpu[i+3];	
							}
							
				}while ((ErrorGyro[0]  > MAX_GYRO_QUIET )|| (ErrorGyro[0] < MIN_GYRO_QUIET)
					 		 ||(ErrorGyro[1] > MAX_GYRO_QUIET )|| (ErrorGyro[1] < MIN_GYRO_QUIET)
 					 		 ||(ErrorGyro[2] > MAX_GYRO_QUIET )|| (ErrorGyro[2] < MIN_GYRO_QUIET)
							); //差异很小时，认为已经静止

	}
	
	
	

	/* throw first 100  group data and make 256 group average as offset */	
	for(i=0;i<356;i++)	//水平以及静止标定，前3个加速度，后3个角速度
	{		
				MpuGetData();
				if(100 <= i) //取256组（i从100到356）数据进行平均
				{
							uint8_t k;
							for(k=0;k<6;k++) //前3个加速度，后3个角速度
							{
										buffer[k] += pMpu[k]; //256组数据求和
							}
				}
	}
	

		
	//新增，部分恢复原参数。
	//当mode=1，只做水平标定，校准加速度，xy=0，z=8192，数组的后3个元素
	//当mode=2，只做静止标定，校准角速度，xyz=0，数组的前3个元素
	
	for(i=0;i<6;i++) //取出静止时的数据
	{
				MpuOffset[i] = buffer[i]>>8; // 平均值=总和/256
	}
	
	
	
	
	TIM_ITConfig(  //使能或者失能指定的TIM中断
				TIM3, //TIM2
				TIM_IT_Update,
				ENABLE  //使能
	);
	
	
	
	
	
}

/**************************************END OF FILE*************************************/

