#include "flow.h"
#include "myMath.h"							 
#include "spl06.h"


struct _flow_ mini;		//mini光流
float pixel_cpi=1.0f; //像素转为cm，积分前就要做

//串口1回调函数，频率/周期 由光流模块决定
void Flow_Receive1(u8 data) //串口1解析光流模块数据
{
	static u8 RxBuffer[32];
	static u8 _data_cnt = 0;
	static u8 state = 0; 
	u8 sum = 0;
	static u8 fault_cnt;
	
	// 0xFE 0x04 DATA0 DATA1 DATA2 DATA3 DATA4 DATA5 SUM SQUAL 0xAA
	switch(state)
	{
		case 0:
			if(data==0xFE)  //包头
			{
				state=1;
				RxBuffer[_data_cnt++]=data;
			}else state = 0;
		break;
			
		case 1:
			if(data==0x04)
			{
				state=2;
				RxBuffer[_data_cnt++]=data;
			}else state = 0;
		break;
			
		case 2:
			RxBuffer[_data_cnt++]=data;
			if(_data_cnt==11)
			{
				state = 0;
				_data_cnt = 0;
				sum =  (RxBuffer[2] + RxBuffer[3] + RxBuffer[4] + RxBuffer[5]+ RxBuffer[6] + RxBuffer[7]);
				// 0xFE 0x04 DATA0 DATA1 DATA2 DATA3 DATA4 DATA5 SUM SQUAL 0xAA
				if((0xAA == data) && (sum == RxBuffer[8])) 
				{
					Flow_SSI_CNT++;//光流数据频率
					
					//读取原始数据
					mini.flow_x = -((s16)(*(RxBuffer+3)<<8)|*(RxBuffer+2));
					mini.flow_y = ((s16)(*(RxBuffer+5)<<8)|*(RxBuffer+4));
					//mini.flow_y = -((s16)(*(RxBuffer+5)<<8)|*(RxBuffer+4));
					
					// 注意光流与加速度坐标系不一致，但与姿态角坐标系一致。 
					// 定点悬停时，光流内环pid运算输出值会赋值给期望姿态角，实现无人机位置调整。
					// 光流原始坐标  x+      最终光流坐标   x-	
					//             /                      /
					//      y+ ___/___ y-          y+ ___/___ y-
					//           /|                     /|
					//          / |                    / |
					//        x-	|                   x+ |
					//            z-                     z-  
					
					
					
					mini.qual = *(RxBuffer+9);
					mini.flow_High = ((s16)(*(RxBuffer+7)<<8)|*(RxBuffer+6));  //光流传输过来的数据为毫米，int16_t
					//旧光流模块 高度单位厘米，波特率19200，
					//新光流模块，高度单位毫米，波特率115200
					mini.flow_High = mini.flow_High/10.0; // flow_High转为 float，单位：厘米
					
					//式中HIGH为实际高度，单位：米；转移到原始数据积分前
					//	 cpi = ((FlightData.High.bara_height*0.01f) / 11.914f) *2.54f;
					pixel_cpi = ((50*0.01f) / 11.914f) *2.54f; //这里用的50cm处的cpi
					
					//单帧位移 高度融合(pixel_cpi ) 还是 总位移 高度融合（cpi）选择一个就可以了
					//不用的那个保持 默认值 1.0f 即可
					
					
					static float high_tmp=0; //高度滤波，计算光流cpi用高度high_tmp
					high_tmp += (mini.flow_High - high_tmp)*0.1; //
					pixel_cpi = pixel_cpi * ( 1.0f + (high_tmp-50.0f)/50.0f*0.1 ); //换算到真实高度下的cpi
					
					//if(high_tmp<10) pixel_cpi=0; //高度<10cm输出0，屏蔽光流输出
					
					
					//pixel_flow.fix_High = pixel_cpi; //上位机调试用
					
					mini.flow_x_iOUT += mini.flow_x ;       //这个位移单位为像素，没有换算为cm  
					mini.flow_y_iOUT += mini.flow_y ;       //这个位移输出调试用，后面的积分位移会被复位（用户拨动摇杆时）
					
					mini.flow_x = mini.flow_x * pixel_cpi;		//像素转为cm
					mini.flow_y = mini.flow_y * pixel_cpi;		//像素转为cm
					
					
					//积分要在这里（串口函数回调函数）做，避免丢失光流数据。
					//但每次收到的数据都要提前做好角度补偿和高度融合（cpi转化），而不能等积分后再做。
					mini.flow_x_i += mini.flow_x ; //积分出位移,cm单位，要在高度融合后才积分
					mini.flow_y_i += mini.flow_y ; 
					

					
					//判断光流数据是否有效
					if(mini.qual<25)  //0x19
					{
							fault_cnt++;													
							if(fault_cnt>60)	//连续60次异常标定为无效
							{
								fault_cnt = 60;
								mini.ok = 0;
							}
					}
					else 
					{
							fault_cnt=0;
							mini.ok = 1;
					}
				}
			}
		break;
			
		default:
			state = 0;
			_data_cnt = 0;
		break;
		
	}
}


/*  已弃用
void Flow_Receive(u8 data) //串口1解析光流模块数据
{
	static u8 RxBuffer[32];
	static u8 _data_cnt = 0;
	static u8 state = 0; 
	u8 sum = 0;
	static u8 fault_cnt;
	
	
	switch(state)
	{
		case 0:
			if(data==0xFE)  //包头
			{
				state=1;
				RxBuffer[_data_cnt++]=data;
			}else state = 0;
		break;
		case 1:
			if(data==0x04)
			{
				state=2;
				RxBuffer[_data_cnt++]=data;
			}else state = 0;
		break;
		case 2:
			RxBuffer[_data_cnt++]=data;
			if(_data_cnt==9)
			{
				state = 0;
				_data_cnt = 0;
				sum =  (RxBuffer[2] + RxBuffer[3] + RxBuffer[4] + RxBuffer[5]);
				if((0xAA == data) && (sum == RxBuffer[6])) 
				{
					Flow_SSI_CNT++;//光流数据频率
					
					//读取原始数据
					mini.flow_x = ((s16)(*(RxBuffer+3)<<8)|*(RxBuffer+2));
					mini.flow_y = ((s16)(*(RxBuffer+5)<<8)|*(RxBuffer+4));
					mini.qual = *(RxBuffer+7);
					
					mini.flow_x_i += mini.flow_x ;
					mini.flow_y_i += mini.flow_y ;
					
					//判断光流数据是否有效
					if(mini.qual<25)
					{
							fault_cnt++;													
							if(fault_cnt>60)	//连续60次异常标定为无效
							{
								fault_cnt = 60;
								mini.ok = 0;
							}
					}
					else 
					{
							fault_cnt=0;
							mini.ok = 1;
					}
				}
			}
		break;
		default:
			state = 0;
			_data_cnt = 0;
		break;
	}
}
*/

struct _pixel_flow_  pixel_flow;//光流输出数据结构体


//mini光流数据滤波融合
//在control.c里面，清空了原始值flow_x,但未清空角度补偿，导致最终位移还在累加（由于运动末尾飞机姿态角逐渐摆正的原因）
//所以应该清空融合后的数据，而非融合前的数据。


//原始数据，每帧移动量（这个帧指的串口数据帧，而非照片帧），
//->1.1 原始数据							  mini.flow_x    （这是每帧像素移动量，串口回调函数Flow_Receive1())
//->1.2 原始数据积分 		      mini.flow_x_i   1.1积分.这个也来自Flow_Receive1()

//以下在Pixel_Flow_Fix()函数中处理
//->1.3 总位移滤波  			pixel_flow.fix_x_i    1.2滤波      这是这个总位移清零没有，角度补偿值会产生位移。
//->2.1 角度补偿值				pixel_flow.ang_x      
//->2.2 融合补偿值       pixel_flow.out_x_i    1.3+2.1
//->3.1 得到速度         pixel_flow.x					 2.2微分
//->3.2 速度滤波         pixel_flow.fix_x      3.1滤波
//->4.1 得到cm单位的坐标 pixel_flow.loc_x	     2.2换算		 
//->4.2 得到cm单位的速度 pixel_flow.loc_xs      3.2换算
//注意，当control.c 把mini.flow_x_i清零复位，
//其实 pixel_flow.loc_x这个坐标仍然在不断计数

//需要引入 光流停止标识符，比清零mini.flow_x_i更合理。

//在scheduler.c调用时，时间参数为0.006s，这个影响到速度的计算。

void Pixel_Flow_set_zero(float desired_x, float desired_y);

float cpi=1.0f; //这里的cpi永远为1.0，真正的cpi转移到最前面, 改名 pixel_cpi

void Pixel_Flow_Fix(float dT) //10ms调用一次，dT=0.01
{
//	float high_temp;

	
	//如果光流模块异常直接退出
	if(Flow_Err ==1 || !mini.ok)
	{		
		mini.flow_x_i=0;
		mini.flow_y_i=0;
		pixel_flow.fix_x = 0;
		pixel_flow.fix_y = 0;	
		
		pixel_flow.err1_cnt++;
		return;
	}
	
	////////////////////////*积分位移处理*////////////////////////////
	//低通滤波//这里只是滤波，mini.flow_x_i本身就是积分值(总位移)
	pixel_flow.fix_x_i += ((mini.flow_x_i - pixel_flow.fix_x_i) *0.2);     //flow_x_i -> fix_x_i
	pixel_flow.fix_y_i += ((mini.flow_y_i - pixel_flow.fix_y_i) *0.2);		 //flow_y_i -> fix_y_i
	
	
	//传感器倾角参数  用姿态角去补偿积分位移（#define  angle_to_rad  0.0174f  //角度转弧度）
	pixel_flow.ang_x += ( 600.0f*tan(-Angle.pitch*ANG_2_RAD) - pixel_flow.ang_x) *0.2;
	pixel_flow.ang_y += ( 600.0f*tan(-Angle.roll*ANG_2_RAD) - pixel_flow.ang_y) *0.2;

	//对姿态角补偿做高度融合，确保补偿与实际位移单位一致
	//pixel_flow.ang_x = pixel_flow.ang_x * pixel_cpi; //不能直接修改 pixel_flow.ang_x 
	//pixel_flow.ang_y = pixel_flow.ang_y * pixel_cpi; //会影响到滤波
	

	//位移与角度互补融合, 角度补偿放在最后是对的。像原子那样把角度补偿也放到单帧数据里面会导致补偿累加。
	//pixel_flow.fix_x_i作为移动总位移，不含角度补偿
	//pixel_flow.out_x_i作为角度补偿后的，在总位移基础上补偿一次即可，这个是新的变量，不是直接修改原来的位移（为了防止补偿累加）
	pixel_flow.out_x_i = pixel_flow.fix_x_i - pixel_flow.ang_x * pixel_cpi;  				// fix_x_i -> out_x_i
	pixel_flow.out_y_i = pixel_flow.fix_y_i - pixel_flow.ang_y * pixel_cpi;					// fix_y_i -> out_y_i
	

	////////////////////////*微分位移处理*////////////////////////////
	
	//对积分位移进行微分处理，得到速度。
	
	//求微分速度，(本次坐标-上次坐标)/调用周期
	pixel_flow.x = (pixel_flow.out_x_i - pixel_flow.out_x_i_o)/dT;	// out_x_i -> x
	pixel_flow.out_x_i_o = pixel_flow.out_x_i;
	pixel_flow.y = (pixel_flow.out_y_i - pixel_flow.out_y_i_o)/dT;	// out_y_i -> y
	pixel_flow.out_y_i_o = pixel_flow.out_y_i;
	
	
	
	//速度低通滤波
	pixel_flow.fix_x += ( pixel_flow.x - pixel_flow.fix_x ) * 0.1f;  //x -> fix_x
	pixel_flow.fix_y += ( pixel_flow.y - pixel_flow.fix_y ) * 0.1f;  //y -> fix_y
	
	
		///////////////////*光流数据与高度数据融合*//////////////////////////
		//由于高度数据与光流像素拍照瞬间高度实际无法做到同步，
		//所以实际效果是不考虑高度因素反而更稳定。高度变化反而引入了额外的测量位移。
		//这里采用折衷的方法，在理论cpi变化时，使用坐标轴移动的方法，尽量确保坐标不变（新旧cpi相比）

		//式中HIGH为实际高度，单位：米；转移到原始数据积分前
		//	 cpi = ((FlightData.High.bara_height*0.01f) / 11.914f) *2.54f ;
		//cpi = ((50*0.01f) / 11.914f) *2.54f; //这里用的50cm处的cpi
		//pixel_flow.fix_High=cpi; //上位机调试用
		
		
		/*
		static float high_tmp=0;
		high_tmp += (mini.flow_High - high_tmp)*0.1; //
		cpi = cpi / 50.0f * high_tmp; //换算到真实高度的cpi
		static float cpi_offset=0;
		static float cpi_old=0;
		cpi_offset=cpi-cpi_old;
		cpi_old=cpi;

		static float offset_x_i=0;
		static float offset_y_i=0;
		offset_x_i = pixel_flow.out_x_i*cpi_offset; //因为cpi变化导致的突变位移
		offset_y_i = pixel_flow.out_y_i*cpi_offset; //突变位移需要用坐标轴移动的方法归零
		Pixel_Flow_set_zero(offset_x_i, offset_y_i); //移动坐标轴，使得offset归零
		*/
		
		//在最后一步才开始cpi换算，有个前提就是累计位移全是在这个高度下拍摄到的，
		//实际不是这样的，所以在积分最最开始的时候就要换算了
		
		//积分位移值单位转换为：厘米
		pixel_flow.loc_x = pixel_flow.out_x_i * cpi;   //out_x_i -> loc_x
		pixel_flow.loc_y = pixel_flow.out_y_i * cpi;   //out_y_i -> loc_y

		//微分速度值单位转换为：厘米/秒
		pixel_flow.loc_xs = pixel_flow.fix_x * cpi; 	//fix_x -> loc_xs
		pixel_flow.loc_ys = pixel_flow.fix_y * cpi;		//fix_y -> loc_ys

		
		
		
		
	
	//pixel_flow.loc_ang_x = pixel_flow.ang_x * cpi;//复位时使用
	//pixel_flow.loc_ang_y = pixel_flow.ang_y * cpi;//姿态角补偿值
	
	
	
	
	
	
	
	
	
	
	//新增，由于摇杆控制结束时，飞机姿态角必然会变化，导致角度补偿值存在。
	//所仅仅通过mini.flow_x_i设为0，复位并不彻底，飞机光流坐标仍然在变化。
	//mini.flow_x_i与角度补偿需要成对出现，只复位mini.flow_x_i会乱套
	if(pixel_flow.flow_pause){
		
			pixel_flow.err2_cnt++;
			//为了能达到现在的pixel_flow.loc_x，我们要反推其他值。
			pixel_flow.flow_pause=0;			//下次需要暂停时，再设为1.
		
			//期望坐标归零
			Pixel_Flow_set_zero(Flow_PosPid_x.desired, Flow_PosPid_y.desired);

	}
	
	

	
	
	
	///////////////////////////////////////////////
	
	//flow_x(帧) -> flow_x_i(位移) |  ->  fix_x_i(滤波)  ->out_x_i(姿态角融合) -------------------------->loc_x(转为厘米)
	//                                                  |                   |
	//                                                  --->out_x_i_o(old)->|
	//                                                                      |
	//                                                                      |->x(微分速度)->fix_x(滤波)->loc_xs(转为厘米)
	

	//当位移太大，大于+/-1000cm，需要移动坐标轴，否则float尾数精度难以保证到0.1cm
  if(pixel_flow.loc_x>1000.0f) Pixel_Flow_set_zero(1000.0f,0.0f);
  else if(pixel_flow.loc_x<-1000.0f) Pixel_Flow_set_zero(-1000.0f,0.0f);
	
  if(pixel_flow.loc_y>1000.0f) Pixel_Flow_set_zero(0.0f,1000.0f);
  else if(pixel_flow.loc_y<-1000.0f) Pixel_Flow_set_zero(0.0f,-1000.0f);


}


//新增函数
//移动坐标轴，目的是不产生任何抖动的情形下，实现期望坐标归零。 期望坐标归零，有很多好处
void Pixel_Flow_set_zero(float desired_x, float desired_y){
	

			Flow_PosPid_x.desired -=desired_x; //光流外环pid期望坐标归零
			pixel_flow.loc_x -=desired_x; //测量坐标要配合移动（已融合高度已转为cm）
			//光流原始数据以及中间数据也要配合移动，才能避免归零过程的抖动
			pixel_flow.out_x_i_o = pixel_flow.out_x_i -= desired_x/cpi; //角度补偿后的（未融合高度未转为cm）
			pixel_flow.fix_x_i -= desired_x/cpi;//已滤波，但角度补偿前的
			pixel_flow.fix_x_i += pixel_flow.ang_x * pixel_cpi; //反向补偿
			mini.flow_x_i -= desired_x/cpi; //未滤波的
			mini.flow_x_i += pixel_flow.ang_x * pixel_cpi; //反向补偿

			Flow_PosPid_y.desired -=desired_y; //光流外环pid期望坐标归零
			pixel_flow.loc_y -=desired_y;//测量坐标要配合移动
			//光流原始数据以及中间数据也要配合移动，才能避免归零过程的抖动
			pixel_flow.out_y_i_o = pixel_flow.out_y_i -= desired_y/cpi;
			pixel_flow.fix_y_i -= desired_y/cpi;
			pixel_flow.fix_y_i += pixel_flow.ang_y * pixel_cpi; //反向补偿
			mini.flow_y_i -= desired_y/cpi;
			mini.flow_y_i += pixel_flow.ang_y * pixel_cpi; //反向补偿

}






///////////////////////*光流数据与高度数据融合*////////////////////////////////////////////////////
//// high_temp = (wcz_h_fus.out - high_begin);
//	
////	high_temp = 50;//固定值方便调试
//	
//	high_temp = FlightData.High.bara_height; //实时高度值
//	
//	cpi = 11.914f / LIMIT(high_temp*0.01f,0.1f,1.5f) ;
//	
//	pixel_flow.dx = (pixel_flow.out_x_i - pixel_flow.out_x_i_o)/cpi*2.54f; 
//	pixel_flow.out_x_i_o = pixel_flow.out_x_i;
//	pixel_flow.dy = (pixel_flow.out_y_i - pixel_flow.out_y_i_o)/cpi*2.54f; 
//	pixel_flow.out_y_i_o = pixel_flow.out_y_i;


//	////////////////////////*微分数据处理*///////////////////////////////

//	//微分求速度(单位：cm/s)
//	pixel_flow.x = pixel_flow.dx/dT;
//	pixel_flow.y = pixel_flow.dy/dT;
//	
//	//低通滤波
//	pixel_flow.fix_x += ( pixel_flow.x - pixel_flow.fix_x ) * 0.1f;
//	pixel_flow.fix_y += ( pixel_flow.y - pixel_flow.fix_y ) * 0.1f;
//	
//	//累加求位移(不需要乘以周期dt，单位：cm)
//	pixel_flow.x_i += pixel_flow.dx;
//	pixel_flow.y_i += pixel_flow.dy;




