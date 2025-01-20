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


#include <stdlib.h>
#include <string.h>
#include "ALL_DATA.h"
#include "ANO_DT.h"
#include "USART3.h"
#include "imu.h"
#include "spl06.h"
#include "myMath.h"
#include "ANO_Data_Transfer.h"
/******************************************************************/
//--------------------------
static uint8_t RatePID[18];
static uint8_t AnglePID[18];
static uint8_t HighPID[18];
static uint8_t HighPID4[18];
_flag_st flag;
static struct{
	uint8_t PID1 :1; //接受到上位机PID组1
	uint8_t PID2 :1; //接受到上位机PID组2
	uint8_t PID3 :1; //接受到上位机PID组3
	uint8_t PID4 :1; //接受到上位机PID组4
	uint8_t PID5 :1; //接受到上位机PID组5
	uint8_t PID6 :1; //接受到上位机PID组6	
	uint8_t CMD2_READ_PID:1; //接受到上位机读取PID的请求
}ANTO_Recived_flag;
 

/////////////////////////////////////////////////////////////////////////////////////


extern u16 save_pid_en;


/***********************************************************************
 * 
 * @param[in] 
 * @param[out] 
 * @return     
 **********************************************************************/	
//这个函数已废弃，被 ANO_DT_Data_Receive_Anl()替代
void ANO_Recive(int8_t *pt)                   //接收到上位机的数据
{
	switch(pt[2])
	{
		case ANTO_RATE_PID:  //0x10
			ANTO_Recived_flag.PID1 = 1;             //接收到上位机发来的PID数据
			memcpy(RatePID,&pt[4],18);              //先把接收到的数据提出来，防止被下一组PID数据覆盖，这组的PID是给速度环用的
			break;
		case ANTO_ANGLE_PID:                      //这组的PID是给角度环用的
			memcpy(AnglePID,&pt[4],18);
			ANTO_Recived_flag.PID2 = 1;
			break;
		case ANTO_HEIGHT_PID:                     //这组的PID是给高度环用的
			memcpy(HighPID,&pt[4],18);
			ANTO_Recived_flag.PID3 = 1;
			break;
		case ANTO_PID4:
			memcpy(HighPID4,&pt[4],18);
			ANTO_Recived_flag.PID4 = 1;
			break;
		case ANTO_PID5:   
			break;
		case ANTO_PID6:
			break;
		case 0x01:                                //上位机发来的CMD1 包含各种校准

			break;
		case 0x02:                                //上位机发来的CMD2 包含请求读取PID等
			{
			   enum                                  //上位请求飞控类型
				{
					READ_PID = 0X01,                    //读取飞控的PID请求
					READ_MODE = 0x02,                   //读取飞行模式
					READ_ROUTE = 0x21,                  //读取航点信息
					READ_VERSION = 0XA0,                //读取飞控版本
					RETURN_DEFAULT_PID = 0xA1           //恢复默认PID
				 }CMD2;

				switch(*(uint8_t*)&pt[4])             //判断上位机发来CMD的内容
				{
					case READ_PID:                      //上位机请求读取飞控PID数据
						ANTO_Recived_flag.CMD2_READ_PID = 1;
						break;
					case READ_MODE: 
						break;
					case READ_ROUTE: 
						break;					
					case READ_VERSION:  
						break;
					case RETURN_DEFAULT_PID:  
						break;					
					default: 
						break;					
				}
			
			}
			break;
		case ANTO_RCDATA: //0x03   Immediately deal with 
			break;
		default:
			break;			
	}
	return;
}
/***********************************************************************
 * 
 * @param[in] 
 * @param[out] 
 * @return     
 **********************************************************************/
static void ANTO_Send(const enum ANTO_SEND FUNCTION) //发送数据到上位机
{
	uint8_t i;
	uint8_t len=2;
	int16_t Anto[12];
	int8_t *pt = (int8_t*)(Anto);
	PidObject *pidX=0;
	PidObject *pidY=0;
	PidObject *pidZ=0;
  long _temp;

	
	switch(FUNCTION)
	{
		case ANTO_RATE_PID:      //PID1
				 pidX = &pidRateX;
				 pidY = &pidRateY;
				 pidZ = &pidRateZ;
         goto send_pid;		
		case ANTO_ANGLE_PID:      //PID2
				 pidX = &pidRoll;
				 pidY = &pidPitch;
				 pidZ = &pidYaw;
				 goto send_pid;				
		case ANTO_HEIGHT_PID:     //PID3
				 pidX = &pidHeightRate;
				 pidY = &pidHeightHigh;
				 pidZ = &Flow_SpeedPid_x;
				 goto send_pid;							
		case ANTO_PID4:	 				 //PID4
				 pidX = &Flow_PosPid_x;
				 pidY = &Flow_SpeedPid_y;
				 pidZ = &Flow_PosPid_y;
				 goto send_pid;	
		case ANTO_PID5:	         //PID5
    case ANTO_PID6:
send_pid:
			if(pidX!=NULL)
			{
				Anto[2] = (int16_t)(pidX->kp *1000);
				Anto[3] = (int16_t)(pidX->ki *1000);
				Anto[4] = (int16_t)(pidX->kd *1000);
			}
			if(pidY!=NULL)
			{
				Anto[5] = (int16_t)(pidY->kp *1000);
				Anto[6] = (int16_t)(pidY->ki *1000);
				Anto[7] = (int16_t)(pidY->kd *1000);
			}
			if(pidZ!=NULL)
			{
				Anto[8] = (int16_t)(pidZ->kp *1000);
				Anto[9] = (int16_t)(pidZ->ki *1000);
				Anto[10] = (int16_t)(pidZ->kd *1000);
			}
			len = 18;
			break;
		case ANTO_MOTOR:    //send motor

				len = 8;
			break;	
		case ANTO_RCDATA:            //发送遥控器的通道参数
		   	Anto[2] = Remote.thr;
				Anto[3] = Remote.yaw;
				Anto[4] = Remote.roll;
				Anto[5] = Remote.pitch;
				Anto[6] = Remote.AUX1;
				Anto[7] = Remote.AUX2;
				Anto[8] = Remote.AUX3;
				Anto[9] = Remote.AUX4;
				Anto[10] =Remote.AUX5; 
				Anto[11] =Remote.AUX6; 
				len = 20;

			break;
		case ANTO_MPU_MAGIC:     //发送MPU6050和磁力计的数据
			memcpy(&Anto[2],(int8_t*)&MPU6050,sizeof(_st_Mpu));
			memcpy(&Anto[8],(int8_t*)&AK8975,sizeof(_st_Mag));
			len = 18;
			break;
		case ANTO_SENSER2:	
				if(spl_flag==1)  //气压标志位 判断气压芯片是否焊接
		    {
						_temp= (long)FlightData.High.ultra_baro_height;						//气压计模块的高度 
					
					
				}
				else _temp=0;
				*(uint16_t*)&Anto[2] = *((uint16_t*)&_temp+1);
				*(uint16_t*)&Anto[3] = *((uint16_t*)&_temp);
				Anto[4] = (uint32_t)FlightData.High.ultra_height;   //超声波模块高度
				//Anto[4] = 0;
				len = 6;
		    
	
			break;
		case ANTO_STATUS:     //send angle
			
		  	if(spl_flag==1)  //气压标志位 判断气压芯片是否焊接
		    {
						 _temp= ((long)FlightData.High.bara_height);						//气压计模块的高度
				}
				else _temp=0;
			
		
				Anto[2] = (int16_t)(-Angle.roll*100);
				Anto[3] = (int16_t)(Angle.pitch*100);
				Anto[4] = (int16_t)(-Angle.yaw*100);
				*(uint16_t*)&Anto[5] = *((uint16_t*)&_temp+1);
				*(uint16_t*)&Anto[6] = *((uint16_t*)&_temp);  				//高度	 低16位	
				switch(Command.FlightMode)
				{
						case LOCK:
							ALL_flag.slock_flag = 0;  //高位飞行模式
							break;
						case NORMOL:
								if(ALL_flag.unlock == 1)
								{
										
									ALL_flag.slock_flag = 0x0101; 
								}
								else{ALL_flag.slock_flag = 0x0100;}
							break;
						case HEIGHT:
								if(ALL_flag.unlock == 1)
								{
										ALL_flag.slock_flag = 0x0201;
								}
								else{ALL_flag.slock_flag = 0x0200;}
						  break;
						case Flow_POSITION:
								if(ALL_flag.unlock == 1)
								{
										ALL_flag.slock_flag= 0x0301;
								}
								else{ALL_flag.slock_flag= 0x0300;}
							break;
						default:
							ALL_flag.slock_flag = 0;  
							break;
				}	
				Anto[7] =ALL_flag.slock_flag ;	
				len = 12;
			break;
		case ANTO_POWER:

				break;
		default:
			break;			
	}
	
	Anto[0] = 0XAAAA;
	Anto[1] = len | FUNCTION<<8;
	pt[len+4] = (int8_t)(0xAA+0xAA);
	for(i=2;i<len+4;i+=2)    //a swap with b;
	{
		pt[i] ^= pt[i+1];
		pt[i+1] ^= pt[i];
		pt[i] ^= pt[i+1];
		pt[len+4] += pt[i] + pt[i+1];
	}
	
	USART3_SendByte(pt,len+5);
}
/***********************************************************************
 * polling  work.
 * @param[in] 
 * @param[out] 
 * @return     
 **********************************************************************/
//通过飞控板的串口3输出数据到 匿名上位机
//同时也会更新来自上位机的pid参数给飞控，这个pid参数不一定从串口接收而来。也可能是遥控转送
void ANTO_polling(void) //轮询扫描上位机端口, 由 scheduler.c 的 Duty_20ms() 调用
{
	volatile static uint8_t status = 0;
	switch(status)
	{
		case 0:
			
			status = 1;
			break;
		case 1:


			if(*(uint8_t*)&ANTO_Recived_flag != 0) //一旦接收到上位机的数据，则暂停发送数据到上位机，转而去判断上位机要求飞控做什么。
			{
				status = 2;
			}
			else
			{
					ANTO_Send(ANTO_MPU_MAGIC);
		//			delay_ms(1);
					ANTO_Send(ANTO_STATUS);
		//			delay_ms(1);
					ANTO_Send(ANTO_RCDATA);
		//			delay_ms(1);
					ANTO_Send(ANTO_SENSER2);
		//			delay_ms(1);
			}
		 	break;
		case 2:
			if(*(uint8_t*)&ANTO_Recived_flag == 0)//上位机的发过来的数据都被处理了，则返回专心的发送数据到上位机
			{
				status = 1;
			}
	
			if(ANTO_Recived_flag.CMD2_READ_PID) //判断上位机是否请求发发送飞控PID数据到上位机
			{		
					ANTO_Send(ANTO_RATE_PID);
					//delay_ms(1);
					ANTO_Send(ANTO_ANGLE_PID);
					//delay_ms(1);
					ANTO_Send(ANTO_HEIGHT_PID);
					//delay_ms(1);
				  ANTO_Send(ANTO_PID4);
					//delay_ms(1);
					ANTO_Recived_flag.CMD2_READ_PID = 0;
			}
			
			if(*(uint8_t*)&ANTO_Recived_flag & 0x3f) //接收到上位机发来的PID数据
			{
					PidObject *pidX=0;
					PidObject *pidY=0;
					PidObject *pidZ=0;
				  uint8_t *P;
				
					if(ANTO_Recived_flag.PID1)
					{
						 pidX = &pidRateX;
						 pidY = &pidRateY;
						 pidZ = &pidRateZ;
						 P = RatePID;
						 ANTO_Recived_flag.PID1 = 0;
					}
					else if(ANTO_Recived_flag.PID2)
					{
						 pidX = &pidRoll;
						 pidY = &pidPitch;
						 pidZ = &pidYaw;
						 P = AnglePID;	
						 ANTO_Recived_flag.PID2 = 0;                             
					}
					else if(ANTO_Recived_flag.PID3)
					{
						 pidX = &pidHeightRate;
						 pidY = &pidHeightHigh;
						 pidZ = &Flow_SpeedPid_x;
						 P = HighPID;	
						 ANTO_Recived_flag.PID3 = 0;     						
					}
					else if(ANTO_Recived_flag.PID4)
					{
						 pidX = &Flow_PosPid_x;
						 pidY = &Flow_SpeedPid_y;
						 pidZ = &Flow_PosPid_y;
						 P = HighPID4;	
						 ANTO_Recived_flag.PID4 = 0;     						
					}
					else
					{
						break;
					}
					{
							union {
								uint16_t _16;
								uint8_t _u8[2];
							}data;
							
							if(pidX!=NULL)
							{
								data._u8[1] = P[0]; 
								data._u8[0] = P[1];
								pidX->kp =  data._16 /1000.0f;
								data._u8[1] = P[2]; 
								data._u8[0] = P[3];
								pidX->ki =  data._16 /1000.0f;
								data._u8[1] = P[4]; 
								data._u8[0] = P[5];
								pidX->kd =  data._16 /1000.0f;								
							}
							if(pidY!=NULL)
							{
								data._u8[1] = P[6]; 
								data._u8[0] = P[7];
								pidY->kp =  data._16 /1000.0f;
								data._u8[1] = P[8]; 
								data._u8[0] = P[9];
								pidY->ki =  data._16 /1000.0f;
								data._u8[1] = P[10]; 
								data._u8[0] = P[11];
								pidY->kd =  data._16 /1000.0f;		
							}
							if(pidZ!=NULL)
							{
								data._u8[1] = P[12]; 
								data._u8[0] = P[13];
								pidZ->kp =  data._16 /1000.0f;
								data._u8[1] = P[14]; 
								data._u8[0] = P[15];
								pidZ->ki =  data._16 /1000.0f;
								data._u8[1] = P[16]; 
								data._u8[0] = P[17];
								pidZ->kd =  data._16 /1000.0f;		
							}				
					}				
			}
			break;
		default:
			break;
	}

}
////////////////////////////////////////////////////////////////////////
////////////////////////串口3中断接收程序///////////////////////////////
////////////////////////////////////////////////////////////////////////
static int8_t CheckSend[7]={0xAA,0XAA,0xEF,2,0,0,0};
u8 TxBuffer3[256];
u8 TxCounter3=0;
u8 count3=0;

void USART3_IRQHandler(void) //串口3中断接收程序
{ 	
	if((USART3->SR & (1<<7))&&(USART3->CR1 & USART_CR1_TXEIE))
	{
		USART3->DR = TxBuffer3[TxCounter3++]; //写DR清除中断标志 
		if(TxCounter3 == count3)
		{
			USART3->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE中断
		}
	}
	//接收中断
	if(USART3->SR & (1<<5))//if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)    
	{
		u8 com_data = USART3->DR;
 	  ANO_DT_Data_Receive_Prepare(com_data);//上位机数据解析
	}

}

//////////////////校验码返回上位机////////////////////////////////////////////////////
//通过串口3发送校验码回应上位机，表示已收到PID		
//data_to_send以及Send_Check在 ANO_Data_Transfer.c里面定义
//这个仅仅在 上位机修改飞控的pid参数时，会使用到
//同时使用串口3和射频发送校验码给上位机，当 Send_Check=1，会触发射频发送校验码
static void ANO_DT_Send_Check(uint8_t head, uint8_t check_sum) //由 ANO_DT_Data_Receive_Anl() 调用
{
	  u8 sum = 0,i=0;
    data_to_send[0]=CheckSend[0]=0xAA; 
    data_to_send[1]=CheckSend[1]=0xAA;
    data_to_send[2]=CheckSend[2]=0xEF;
    data_to_send[3]=CheckSend[3]=2;
    data_to_send[4]=CheckSend[4]=head;
    data_to_send[5]=CheckSend[5]=check_sum;
    for(i=0;i<6;i++) 
        sum += CheckSend[i];
    data_to_send[6]=CheckSend[6]=sum;
	
    Send_Check = 1; //不仅在这里通过串口发送data_to_send[]数据，
	  //而且在 ANO_Data_Transfer.c 的 ANO_DT_Data_Exchange() 通过射频发送给遥控器，转交给上位机；
	  //串口收到消息后，会把回应信息同时通过串口和射频发送给上位机
	
    USART3_SendByte(CheckSend,7);  //串口发送
}

/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Prepare函数是协议预解析，根据协议的格式，将收到的数据进行一次格式性解析，格式正确的话再进行数据解析
//移植时，此函数应由用户根据自身使用的通信方式自行调用，比如串口每收到一字节数据，则调用此函数一次
//此函数解析出符合格式的数据帧后，会自行调用数据解析函数
void ANO_DT_Data_Receive_Prepare(u8 data) 
{
    static u8 RxBuffer[50];
    static u8 _data_len = 0,_data_cnt = 0;
    static u8 state = 0;

    if(state==0&&data==0xAA)  //包头
    {
        state=1;
        RxBuffer[0]=data;
    }
    else if(state==1&&data==0xAF) //包头
    {
        state=2;
        RxBuffer[1]=data;
    }
    else if(state==2&&data<0XF1) //功能字节
    {
        state=3;
        RxBuffer[2]=data;
    }
    else if(state==3&&data<50)  //长度
    {
        state = 4;
        RxBuffer[3]=data;
        _data_len = data;  //记录长度
        _data_cnt = 0;
    }
    else if(state==4&&_data_len>0)  //开始一包数据内容接收
    {
        _data_len--;
        RxBuffer[4+_data_cnt++]=data;
        if(_data_len==0)
            state = 5;
					
    }
    else if(state==5)
    {
        state = 0;
        RxBuffer[4+_data_cnt]=data;
        ANO_DT_Data_Receive_Anl(RxBuffer,_data_cnt+5);//数据解析处理
    }
    else
        state = 0;
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////
//            以下不仅仅串口3直接与上位机通讯需要在这里解析，通过遥控器usb与上位机传输的数据，也通过这里解析。
///////////////////////////////////////////////////////////////////////////////////////////////////////////


//Data_Receive_Anl()函数是协议数据解析函数，函数参数是符合协议格式的一个数据帧，该函数会首先对协议数据进行校验
//校验通过后对数据进行解析，实现相应功能
//此函数可以不用用户自行调用，由函数 ANO_DT_Data_Receive_Prepare() 或者  NRF_Connect() 自动调用
//f 在 ANO_Data_Transfer.c里面定义
// 这个协议不仅仅用于上位机通讯，遥控与飞控之间也用到这个协议
void ANO_DT_Data_Receive_Anl(u8 *data_buf,u8 num)
{

    u8 sum = 0,i=0;
    for(i=0;i<(num-1);i++)
        sum += *(data_buf+i);
    if(!(sum==*(data_buf+num-1)))       return;     //判断sum
    if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))     return;     //判断帧头

    if(*(data_buf+2)==0X01)     //命令集合1，上位机发来的包含各种校准
    {
				//我们通过遥控器给飞控发送校准命令，油门打最低，长按K2即可
			
        if(*(data_buf+4)==0X01)   //ACC校准
            ;//mpu6050.Acc_CALIBRATE = 1;
        if(*(data_buf+4)==0X02)  //GYRO校准
            ;//mpu6050.Gyro_CALIBRATE = 1;
        if(*(data_buf+4)==0X03)		//光流高度校准
        {
            ;//mpu6050.Acc_CALIBRATE = 1;
            ;//mpu6050.Gyro_CALIBRATE = 1;
        }
        if(*(data_buf+4)==0X04)		//罗盘校准，用来手动测试电机，也可以遥控按 set+“下”进入这个模式
        {
						//重启飞控可以取消动平衡测试，按着H+电机转动，松开停止转动，H-更换电机，油门控制转速
						//通过连接上位机可以实现每个电机转动时查看加速度与角速度的实时变化
						Remote.test_motor=1;
        }
        if(*(data_buf+4)==0X05)  //气压计校准
            ;//mpu6050.Gyro_CALIBRATE = 1;
				
    }
 
    else if(*(data_buf+2)==0X02)    //命令集合2，读取飞机信息
    {

			//上位机请求读取飞控pid参数
			if(*(data_buf+4)==0X01)
        {
						ANTO_Recived_flag.CMD2_READ_PID = 1;   //读取PID 信息
						f.send_pid = 1;
						cnt = 0; //很重要，发送pid和发送其他数据是共用的cnt， 在ANO_DT_Data_Exchange()里面
        }
				
				
        if(*(data_buf+4)==0X02)		//读取飞行模式
        {

        }
				
				
        if(*(data_buf+4)==0XA0)     //读取版本信息
        {
            f.send_version = 1;
        }
				
				//上位机请求恢复飞控默认pid参数
        if(*(data_buf+4)==0XA1)     //恢复 PID 默认参数
        {
	 //       Sort_PID_Flag=2;
						pid_param_Init();
						
						//恢复默认pid后，读取一下pid，让上位机马上看到效果
						ANTO_Recived_flag.CMD2_READ_PID = 1;   //读取PID 信息
						//f 在 ANO_Data_Transfer.c里面定义
						f.send_pid = 1;
						cnt = 0; //这里cnt复位很重要，因为 在ANO_DT_Data_Exchange()里面，发送pid和发送其他数据是共有的cnt，
						//假如此时正在发送其他数据，cnt不为零；如果cnt为6，那么只会发送 cnt7和cnt8的情形。
				}
				
				
    }
		 else if(*(data_buf+2)==0X03)    //解析遥控器数据
		 {	
			 
				flag.NS = 1;//??????(1:????2:WiFi?????3:????)
			  Remote.new_data =1;//来了新数据， Mode_Controler()开始解析
			 
			 
				Remote.thr = (vs16)(*(data_buf+4)<<8)|*(data_buf+5) ; //通道1
			  LIMIT(Remote.thr,1000,2000);
			 
				Remote.yaw = (vs16)(*(data_buf+6)<<8)|*(data_buf+7) ;//通道2
				LIMIT(Remote.yaw,1000,2000);
			 
				Remote.roll = (vs16)(*(data_buf+8)<<8)|*(data_buf+9) ;//通道3
			  LIMIT(Remote.roll,1000,2000);
			 
				Remote.pitch = (vs16)(*(data_buf+10)<<8)|*(data_buf+11) ;//通道4
			  LIMIT(Remote.pitch,1000,2000);//
			 
				//Remote.pitch=2000;Remote.roll=2000;Remote.yaw=2000;


				//测试，是否只有这里会更新 Remote
			  //Remote.raw_data1 = Remote.yaw;
			 
				Remote.AUX1 = (vs16)(*(data_buf+12)<<8)|*(data_buf+13) ;//通道5
			  LIMIT(Remote.AUX1,1000,2000);
				Remote.AUX2 = (vs16)(*(data_buf+14)<<8)|*(data_buf+15) ;//通道6
			  LIMIT(Remote.AUX2,1000,2000);
				Remote.AUX3 = (vs16)(*(data_buf+16)<<8)|*(data_buf+17) ;//通道7
			  LIMIT(Remote.AUX3,1000,2000);
				Remote.AUX4 = (vs16)(*(data_buf+18)<<8)|*(data_buf+19) ;//通道8
			  LIMIT(Remote.AUX4,1000,2000);
				
				//电池电压
				Remote.AUX5 = (vs16)(*(data_buf+20)<<8)|*(data_buf+21) ;//通道9
			  LIMIT(Remote.AUX5,1000,2000);
				
				Remote.AUX6 = (vs16)(*(data_buf+22)<<8)|*(data_buf+23) ;//通道10
				LIMIT(Remote.AUX6,1000,2000);
				
				
				//新增，油门比例，力度，目标偏航角
				Remote.AUX7 =  ((uint16_t)data_buf[24]<<8) | data_buf[25];   //通道11  
				//Remote.AUX7 =  LIMIT(Remote.AUX7,1000,2000);	
				
				//新增，定高起飞 a，屏蔽油门 b， 无头模式 c， 拍照 e， LED亮度 f， 高度加指令 g， 高度减指令 h，
				Remote.AUX8 =  ((uint16_t)data_buf[26]<<8) | data_buf[27];   //通道12  
				//Remote.AUX8 =  LIMIT(Remote.AUX8,1000,2000);	
				
				
				
				
				
	//			key_function(RX_CH[AUX4]);
		 }
		 
		 //当 ANTO_Recived_flag 变化后，在 ANTO_polling() 中，会更新暂时的参数RatePID/AnglePID/HighPID/HighPID4到真正的pid
		 //注意：更改后，要等 scheduler.c 的 Duty_20ms() 调用 ANTO_polling() 后，才会生效
		 
    else if(*(data_buf+2)==0X10)                             //PID1
    {								
			  memcpy(RatePID,&data_buf[4],18);          //先把接收到的数据提出来，防止被下一组PID数据覆盖，，这组的PID是给内环速度用的，包含3个
				ANTO_Recived_flag.PID1 = 1;              //接收到上位机发来的PID数据 变换标志			
        ANO_DT_Send_Check(*(data_buf+2),sum);    //发送校验码回应上位机，表示已收到PID		
    }
    else if(*(data_buf+2)==0X11)                             //PID2
    {
				memcpy(AnglePID,&data_buf[4],18);				 //先把接收到的数据提出来，防止被下一组PID数据覆盖，这组的PID是给外环角度用的，包含3个
			  ANTO_Recived_flag.PID2 = 1;						  //接收到上位机发来的PID数据  变换标志
        ANO_DT_Send_Check(*(data_buf+2),sum);   //发送校验码回应上位机，表示已收到PID	
    }
    else if(*(data_buf+2)==0X12)                             //PID3
    {    
				memcpy(HighPID,&data_buf[4],18);				 //先把接收到的数据提出来，防止被下一组PID数据覆盖，这组的PID是给定高用的2个，以及光流x速度	
				
				memcpy(&HighPID4[6], &data_buf[4+12], 6);	//用x速度更新y速度，12~17字节为x速度，同时也是为y速度
			
				ANTO_Recived_flag.PID3 = 1;						  //接收到上位机发来的PID数据 变换标志
        ANO_DT_Send_Check(*(data_buf+2),sum);   //发送校验码回应上位机，表示已收到PID			       
    }
    else if(*(data_buf+2)==0X13)                             //PID4
    {
			
			//注意，光流y速度，要用上组的最后一个，光流y位置，要用本组第一个
			//*************************************
			// 更改PID3的第3个pid也会更改PID4的第2个pid，属于光流xy速度pid
			// 更改PID4的第1个pid也会更改PID4的第3个pid，属于光流xy位置pid
			// 只需要更改x的速度和位置pid，y的速度和位置会同时变更
			//*************************************
    
			//memcpy(HighPID4,&data_buf[4],18);				 //先把接收到的数据提出来，防止被下一组PID数据覆盖，，这组的PID是给光流x位置，光流y速度，光流y位置，	
			memcpy(HighPID4, &data_buf[4], 6);				//更新光流x位置pid，0~5字节为x位置，
			memcpy(&HighPID4[12], &data_buf[4], 6);	 //用x位置更新光流y位置pid，x位置与位置相同
			//注意上位机的y速度和y位置没有用上
			
			ANTO_Recived_flag.PID4 = 1;						  //接收到上位机发来的PID数据 变换标志
			ANO_DT_Send_Check(*(data_buf+2),sum);		//发送校验码回应上位机，表示已收到PID		

    }
    else if(*(data_buf+2)==0X14)                             //PID5
    {
        ANO_DT_Send_Check(*(data_buf+2),sum); //发送校验码回应上位机，表示已收到PID		
    }
    else if(*(data_buf+2)==0X15)                             //PID6
    {
        ANO_DT_Send_Check(*(data_buf+2),sum); //发送校验码回应上位机，表示已收到PID		
//        Sort_PID_Cnt++;
//        Sort_PID_Flag=1;
    }
		else  if(*(data_buf+2)==0XF1)  //自定义命令/数据
		{

			set_flag = ( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
		
//			if((set_flag&BIT0))	keep_high_mode=1;	else keep_high_mode=0;
//			if(set_flag&BIT1)	Throw_Fly_Mode=1; else Throw_Fly_Mode=0;
//			if(set_flag&BIT2)	No_Head_Mode=1; else No_Head_Mode=0;
//			if(set_flag&BIT3)	turn_head=-1; else turn_head=1;
		
//			if(set_flag&BIT4 && Locat_Err==0 || set_flag&BIT5 && Flow_Err==0)	
//						Fixed_Point_Mode=1;
//			else	Fixed_Point_Mode=0;		  								
//			LED_warn = 2;
		
		}
		else{;}

}



/************************END OF FILE********************/
