#include "ALL_DATA.h"
#include "spl06.h"
#include "i2c.h"
#include "kalman.h"
#include "filter.h"

//注意1：气压计与陀螺仪一样，对焊接温度非常敏感。经受过高温的气压计或陀螺仪或激光ic虽然功能不影响，不至于会烫坏，
//但经过大量测试发现对性能有所影响，即便同规格同批次器件也存在个体差异，有的影响大有的影响小。
//焊接或贴片温度过高，比如250度或更高，芯片效果大打折扣，气压会非常不稳定。陀螺仪经历过高温度后角速度影响不大，但加速度影响很大
//这两个器件最好不要在嘉立创贴片（温度太高250度左右），可以把这两个器件设计到单独的小模块上面，
//使用精密温控加热平台完成回流焊（锡膏185度，平台200度），目前第三代开源无人机实际都是这样设计生产陀螺仪气压模块，以确保每一台效果。
//还有，气压小孔切记不能进入热风或者水气或者松香气，一旦进入焊接热风或者松香焊油气体，使用时气压就会不稳定，零漂过大（几秒时间就能零漂30cm甚至50cm）。
//在维修时一定要注意用超薄高温胶片把气压计密封好，厚度在0.05mm以下才能服帖，太厚的高温胶片遇到热风会脱开并失去保护作用。
//注意2：气压计高度检测延时较大，比激光传感大很多，定高程序应用上需要注意。另外布局位置最好在螺旋桨风力范围外，
//有条件的可以用不带胶的细软海绵保护气压计小孔，防止气流与灰尘水汽干扰内部传感膜片

#undef SUCCESS
#define SUCCESS 0
#undef FAILED
#define FAILED  1

#define PRESSURE_REG 0X00
#define TEMP_REG 0X03
#define PRS_CFG 0x06
#define TMP_CFG 0x07
#define MEAS_CFG 0x08

#define SPL06_REST_VALUE 0x09
#define SPL06_REST_REG 0x0C
#define SPL06_PRODUCT_ID_ADDR 0x0D
#define SPL06_PRODUCT_ID 0X10

#define uint32 unsigned int
	
u8 ult_err=50,ult_ok;
u8 Locat_SSI,Locat_SSI_CNT,Locat_Err,Locat_Mode;
u8 Flow_SSI,Flow_SSI_CNT,Flow_Err;

static struct spl0601_t spl0601;
static struct spl0601_t *p_spl0601;

void spl0601_write(uint8_t hwadr, uint8_t regadr, uint8_t val);
uint8 spl0601_read(uint8 hwadr, uint8 regadr);
void spl0601_get_calib_param(void);


/*****************************************************************************
 函 数 名  : spl0601_write
 功能描述  : I2C 寄存器写入子函数
 输入参数  : uint8 hwadr   硬件地址
             uint8 regadr  寄存器地址
             uint8 val     值
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2015年11月30日
    作    者   : WL
    修改内容   : 新生成函数

*****************************************************************************/
void spl0601_write(unsigned char hwadr, unsigned char regadr, unsigned char val)
{
//	hwI2C0_Device_Addr = hwadr;
//	bI2C0_TxM_Data[0] = regadr;
//	bI2C0_TxM_Data[1] = val;
//
//	I2C0_Engine(2,0,0);
	IIC_Write_One_Byte(hwadr, regadr, val);
}

/*****************************************************************************
 函 数 名  : spl0601_read
 功能描述  : I2C 寄存器读取子函数
 输入参数  : uint8 hwadr   硬件地址
             uint8 regadr  寄存器地址
 输出参数  :
 返 回 值  : uint8 读出值
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2015年11月30日
    作    者   : WL
    修改内容   : 新生成函数

*****************************************************************************/
uint8 spl0601_read(unsigned char hwadr, unsigned char regadr)
{
	uint8 reg_data;

//	hwI2C0_Device_Addr = hwadr;
//	bI2C0_TxM_Data[0] = regadr; //
//	I2C0_Engine(1,1,1);

//	reg_data = bI2C0_RxM_Data[0];
	IIC_Read_1Byte(hwadr, regadr, &reg_data);
	return reg_data;
}

/*****************************************************************************
 函 数 名  : spl0601_init
 功能描述  : SPL06-01 初始化函数
 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2015年11月30日
    作    者   : WL
    修改内容   : 新生成函数

*****************************************************************************/
uint8 spl0601_init(void)
{
	  uint8 data;
    p_spl0601 = &spl0601; /* read Chip Id */
    p_spl0601->i32rawPressure = 0;
    p_spl0601->i32rawTemperature = 0;
  //  p_spl0601->chip_id = 0x34;
	
  	IIC_Read_1Byte(HW_ADR,SPL06_PRODUCT_ID_ADDR,&data);
	  if(SPL06_PRODUCT_ID != data)
	  {
			return FAILED;
		}
		//i2c 通讯成功
		
		
    spl0601_get_calib_param();  //读取校准参数
		
// sampling rate = 32Hz; Pressure oversample = 8;
// spl0601_rateset(PRESSURE_SENSOR,32, 8);
// sampling rate = 32Hz; Temperature oversample = 8;
// spl0601_rateset(TEMPERATURE_SENSOR,32, 8);
		
    spl0601_rateset(PRESSURE_SENSOR,128, 32);  //设置气压测量参数，频率较高
		
// sampling rate = 1Hz; Temperature oversample = 1;
    spl0601_rateset(TEMPERATURE_SENSOR,32, 8); //设置温度测量参数，频率较低
		
    //Start background measurement
    spl0601_start_continuous(CONTINUOUS_P_AND_T); //开始连续测量
		
    return  SUCCESS;
		//初始化成功
}




/*****************************************************************************
 函 数 名  : spl0601_rateset
 功能描述  :  设置温度传感器的每秒采样次数以及过采样率
 输入参数  : uint8 iSensor     0: Pressure; 1: Temperature
						uint8 u8SmplRate  每秒采样次数(Hz) Maximal = 128
						uint8 u8OverSmpl  过采样率         Maximal = 128
             
             
 输出参数  : 无
 返 回 值  : 无
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2015年11月24日
    作    者   : WL
    修改内容   : 新生成函数

*****************************************************************************/
void spl0601_rateset(uint8 iSensor, uint8 u8SmplRate, uint8 u8OverSmpl)
{
    uint8 reg = 0;
    int32 i32kPkT = 0;
    switch(u8SmplRate)
    {
        case 2:
            reg |= (1<<5);
            break;
        case 4:
            reg |= (2<<5);
            break;
        case 8:
            reg |= (3<<5);
            break;
        case 16:
            reg |= (4<<5);
            break;
        case 32:
            reg |= (5<<5);
            break;
        case 64:
            reg |= (6<<5);
            break;
        case 128:
            reg |= (7<<5);
            break;
        case 1:
        default:
            break;
    }
    switch(u8OverSmpl)
    {
        case 2:
            reg |= 1;
            i32kPkT = 1572864;
            break;
        case 4:
            reg |= 2;
            i32kPkT = 3670016;
            break;
        case 8:
            reg |= 3;
            i32kPkT = 7864320;
            break;
        case 16:
            i32kPkT = 253952;
            reg |= 4;
            break;
        case 32:
            i32kPkT = 516096;
            reg |= 5;
            break;
        case 64:
            i32kPkT = 1040384;
            reg |= 6;
            break;
        case 128:
            i32kPkT = 2088960;
            reg |= 7;
            break;
        case 1:
        default:
            i32kPkT = 524288;
            break;
    }

    if(iSensor == 0)
    {
        p_spl0601->i32kP = i32kPkT;
        spl0601_write(HW_ADR, 0x06, reg);
        if(u8OverSmpl > 8)
        {
            reg = spl0601_read(HW_ADR, 0x09);
            spl0601_write(HW_ADR, 0x09, reg | 0x04);
        }
    }
    if(iSensor == 1)
    {
        p_spl0601->i32kT = i32kPkT;
        spl0601_write(HW_ADR, 0x07, reg|0x80);  //Using mems temperature
        if(u8OverSmpl > 8)
        {
            reg = spl0601_read(HW_ADR, 0x09);
            spl0601_write(HW_ADR, 0x09, reg | 0x08);
        }
    }
}

/*****************************************************************************
 函 数 名  : spl0601_get_calib_param
 功能描述  : 获取校准参数
 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2015年11月30日
    作    者   : WL
    修改内容   : 新生成函数

*****************************************************************************/
void spl0601_get_calib_param(void)
{
    uint32 h;
    uint32 m;
    uint32 l;
		//c0
    h = spl0601_read(HW_ADR, 0x10);
    l = spl0601_read(HW_ADR, 0x11);
    p_spl0601->calib_param.c0 = (int16)h<<4 | l>>4;
    p_spl0601->calib_param.c0 = (p_spl0601->calib_param.c0&0x0800)?(0xF000|p_spl0601->calib_param.c0):p_spl0601->calib_param.c0;
		//c1
    h = spl0601_read(HW_ADR, 0x11);
    l = spl0601_read(HW_ADR, 0x12);
    p_spl0601->calib_param.c1 = (int16)(h&0x0F)<<8 | l;
    p_spl0601->calib_param.c1 = (p_spl0601->calib_param.c1&0x0800)?(0xF000|p_spl0601->calib_param.c1):p_spl0601->calib_param.c1;
	
		//c00
    h = spl0601_read(HW_ADR, 0x13);
    m = spl0601_read(HW_ADR, 0x14);
    l = spl0601_read(HW_ADR, 0x15);
    p_spl0601->calib_param.c00 = (int32)h<<12 | (int32)m<<4 | (int32)l>>4;
    p_spl0601->calib_param.c00 = (p_spl0601->calib_param.c00&0x080000)?(0xFFF00000|p_spl0601->calib_param.c00):p_spl0601->calib_param.c00;
    //c10
		h = spl0601_read(HW_ADR, 0x15);
    m = spl0601_read(HW_ADR, 0x16);
    l = spl0601_read(HW_ADR, 0x17);
    p_spl0601->calib_param.c10 = (int32)h<<16 | (int32)m<<8 | l;
    p_spl0601->calib_param.c10 = (p_spl0601->calib_param.c10&0x080000)?(0xFFF00000|p_spl0601->calib_param.c10):p_spl0601->calib_param.c10;
    
		//c01
		h = spl0601_read(HW_ADR, 0x18);
    l = spl0601_read(HW_ADR, 0x19);
    p_spl0601->calib_param.c01 = (int16)h<<8 | l;
		//c11
    h = spl0601_read(HW_ADR, 0x1A);
    l = spl0601_read(HW_ADR, 0x1B);
    p_spl0601->calib_param.c11 = (int16)h<<8 | l;
		
		//c20
    h = spl0601_read(HW_ADR, 0x1C);
    l = spl0601_read(HW_ADR, 0x1D);
    p_spl0601->calib_param.c20 = (int16)h<<8 | l;
		//c21
    h = spl0601_read(HW_ADR, 0x1E);
    l = spl0601_read(HW_ADR, 0x1F);
    p_spl0601->calib_param.c21 = (int16)h<<8 | l;
		
		//c30
    h = spl0601_read(HW_ADR, 0x20);
    l = spl0601_read(HW_ADR, 0x21);
    p_spl0601->calib_param.c30 = (int16)h<<8 | l;
		
}


/*****************************************************************************
 函 数 名  : spl0601_start_temperature
 功能描述  : 发起一次温度测量
 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2015年11月30日
    作    者   : WL
    修改内容   : 新生成函数

*****************************************************************************/
void spl0601_start_temperature(void)
{
    spl0601_write(HW_ADR, 0x08, 0x02);
}

/*****************************************************************************
 函 数 名  : spl0601_start_pressure
 功能描述  : 发起一次压力值测量
 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2015年11月30日
    作    者   : WL
    修改内容   : 新生成函数

*****************************************************************************/
void spl0601_start_pressure(void)
{
    spl0601_write(HW_ADR, 0x08, 0x01);
}

/*****************************************************************************
 函 数 名  : spl0601_start_continuous
 功能描述  : Select node for the continuously measurement
 输入参数  : uint8 mode  1: pressure; 2: temperature; 3: pressure and temperature
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2015年11月25日
    作    者   : WL
    修改内容   : 新生成函数

*****************************************************************************/
void spl0601_start_continuous(uint8 mode)
{
    spl0601_write(HW_ADR, 0x08, mode+4);
}


/*****************************************************************************
 函 数 名  : spl0601_get_raw_temp
 功能描述  : 获取温度的原始值，并转换成32Bits整数
 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2015年11月30日
    作    者   : WL
    修改内容   : 新生成函数

*****************************************************************************/
void spl0601_get_raw_temp(void)
{
    uint8 h[3] = {0};
    h[0] = spl0601_read(HW_ADR, 0x03);
    h[1] = spl0601_read(HW_ADR, 0x04);
    h[2] = spl0601_read(HW_ADR, 0x05);
    p_spl0601->i32rawTemperature = (int32)h[0]<<16 | (int32)h[1]<<8 | (int32)h[2];
    p_spl0601->i32rawTemperature= (p_spl0601->i32rawTemperature&0x800000) ? (0xFF000000|p_spl0601->i32rawTemperature) : p_spl0601->i32rawTemperature;
}

/*****************************************************************************
 函 数 名  : spl0601_get_raw_pressure
 功能描述  : 获取压力原始值，并转换成32bits整数
 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2015年11月30日
    作    者   : WL
    修改内容   : 新生成函数

*****************************************************************************/
void spl0601_get_raw_pressure(void)
{
    uint8 h[3];
    h[0] = spl0601_read(HW_ADR, 0x00);
    h[1] = spl0601_read(HW_ADR, 0x01);
    h[2] = spl0601_read(HW_ADR, 0x02);
    p_spl0601->i32rawPressure = (int32)h[0]<<16 | (int32)h[1]<<8 | (int32)h[2];
    p_spl0601->i32rawPressure= (p_spl0601->i32rawPressure&0x800000) ? (0xFF000000|p_spl0601->i32rawPressure) : p_spl0601->i32rawPressure;
}


/*****************************************************************************
 函 数 名  : spl0601_get_temperature
 功能描述  : 在获取原始值的基础上，返回浮点校准后的温度值
 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2015年11月30日
    作    者   : WL
    修改内容   : 新生成函数

*****************************************************************************/
float spl0601_get_temperature(void)
{
    float fTCompensate;
    float fTsc;

    fTsc = p_spl0601->i32rawTemperature / (float)p_spl0601->i32kT;
    fTCompensate =  p_spl0601->calib_param.c0 * 0.5 + p_spl0601->calib_param.c1 * fTsc;
    return fTCompensate;
}

/*****************************************************************************
 函 数 名  : spl0601_get_pressure
 功能描述  : 在获取原始值的基础上，返回浮点校准后的压力值
 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2015年11月30日
    作    者   : WL
    修改内容   : 新生成函数

*****************************************************************************/
//已废弃，改用spl0601_get_pressure1()
float spl0601_get_pressure(int32_t P,int32_t T)
{
    float fTsc, fPsc;
    float qua2, qua3;
    float fPCompensate;

    fTsc = T / (float)p_spl0601->i32kT;
    fPsc = P / (float)p_spl0601->i32kP;
    qua2 = p_spl0601->calib_param.c10 + fPsc * (p_spl0601->calib_param.c20 + fPsc* p_spl0601->calib_param.c30);
    qua3 = fTsc * fPsc * (p_spl0601->calib_param.c11 + fPsc * p_spl0601->calib_param.c21);
		//qua3 = 0.9f *fTsc * fPsc * (p_spl0601->calib_param.c11 + fPsc * p_spl0601->calib_param.c21);
	
    fPCompensate = p_spl0601->calib_param.c00 + fPsc * qua2 + fTsc * p_spl0601->calib_param.c01 + qua3;
		//fPCompensate = p_spl0601->calib_param.c00 + fPsc * qua2 + 0.9f *fTsc  * p_spl0601->calib_param.c01 + qua3;
    return fPCompensate;
}






	/**************************************************************
 *  data process
 * @param[in] 
 * @param[out] NONE
 * @return   NONE  
 ***************************************************************/

//static const uint8_t D_AVERAGE_NUM = 20;  //原始数据防脉冲干扰的均值滤波   D1  D2
//static int32_t average[2][D_AVERAGE_NUM]={0};

void spl06_001_HeightHighProcess(void)  
{

		static struct _1_ekf_filter ekf[3] = {{0.02,0,0,0,0.001,0.543},{0.02,0,0,0,0.001,0.543},{0.02,0,0,0,0.001,0.543}};	
			
		float f32rawTemperature;	
		float f32rawPressure;	
		float presure;	
			
		kalman_1(&ekf[0],p_spl0601->i32rawPressure);  //一维卡尔曼
		f32rawPressure = 	ekf[0].out;//原始气压数据滤波
			
		kalman_1(&ekf[1],p_spl0601->i32rawTemperature);  //一维卡尔曼		
		f32rawTemperature = 	ekf[1].out;	//原始温度数据滤波
			
		presure = spl0601_get_pressure(f32rawPressure, f32rawTemperature);	
		FlightData.High.bara_height =  (float)((102000.0f	- presure) * 7.8740f); 	//获取高度数据，单位cm
			
		kalman_1(&ekf[2],FlightData.High.bara_height);  //一维卡尔曼
		FlightData.High.bara_height = 	ekf[2].out;		//解算后的高度值滤波	
}	





//已废弃；读取 温度，和气压，
//看 Drv_Spl0601_Read()
void user_spl0601_get(void)
{
	
		spl0601_get_raw_temp();
///		temperature = spl0601_get_temperature();
	
		spl0601_get_raw_pressure();
//		pressure = spl0601_get_pressure();
		
}	
///////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
float spl0601_get_pressure1 ( void )
{
    float fTsc, fPsc;
    float qua2, qua3;
    float fPCompensate;

    fTsc = p_spl0601->i32rawTemperature / ( float ) p_spl0601->i32kT;
    fPsc = p_spl0601->i32rawPressure / ( float ) p_spl0601->i32kP;
    qua2 = p_spl0601->calib_param.c10 + fPsc * ( p_spl0601->calib_param.c20 + fPsc * p_spl0601->calib_param.c30 );
    qua3 = fTsc * fPsc * ( p_spl0601->calib_param.c11 + fPsc * p_spl0601->calib_param.c21 );
    //qua3 = 0.9f *fTsc * fPsc * (p_spl0601->calib_param.c11 + fPsc * p_spl0601->calib_param.c21);

    fPCompensate = p_spl0601->calib_param.c00 + fPsc * qua2 + fTsc * p_spl0601->calib_param.c01 + qua3;
    //fPCompensate = p_spl0601->calib_param.c00 + fPsc * qua2 + 0.9f *fTsc  * p_spl0601->calib_param.c01 + qua3;
    return fPCompensate;
}




float alt_3, height;//海拔相对气压，海拔高度
float temperature; //当前温度
float baro_pressure; //当前气压

//读取 温度，气压，计算海拔高度
float Drv_Spl0601_Read ( void )
{
		//新增
		//spl0601_get_calib_param();  //读取校准参数
	
    spl0601_get_raw_temp();//原始温度传感值   i32rawTemperature
    temperature = spl0601_get_temperature();  //转换为温度  需要c0,c1

    spl0601_get_raw_pressure(); //原始压力传感值   i32rawPressure
    baro_pressure = spl0601_get_pressure1();  //转换为气压， 99537, 需要温度压力原始值，以及7个校准参数

    alt_3 = ( 101000 - baro_pressure ) / 1000.0f; //低于海拔压力值 1.46 千帕
    height = 0.82f * alt_3 * alt_3 * alt_3 + 0.09f * ( 101000 - baro_pressure ) * 100.0f ;

    return height;
}


/*
基础单位：cm
*/
s32 baro_height,baro_height_old,baro_Offset; 
s32 baro_speed_o,baro_speed;

#define safe_div(numerator,denominator,safe_value) ( (denominator == 0) ? (safe_value) : ((numerator)/(denominator)) )
#define LPF_1_(hz,t,in,out) ((out) += ( 1 / ( 1 + 1 / ( (hz) *6.28f *(t) ) ) ) *( (in) - (out) ))

#define MONUM 10
float speed_av_arr[MONUM];
float speed_av;
u16 speed_av_cnt;

float speed_delta;
float speed_delta_lpf;
u8 baro_start=1;


//测量温度与高度, 10ms调用一次
void Height_Get(float dT)
	{
		if(spl_flag==0)  //气压标志位 判断气压芯片是否焊接
		{
			return;
		}
		
		//气压高度清零
		if( baro_start ) //首次执行, 以及每次解锁时
		{
			baro_height = 0;
			baro_height_old = 0;
			baro_Offset = Drv_Spl0601_Read(); //读取初始海拔高度（厘米），开机时的气压
			High_Data_Reset();
			baro_start = 0;
		}
		
		//读取气压计高度
		else
		{	
			baro_height = Drv_Spl0601_Read() - baro_Offset; //当前气压高度厘米（当前海拔高度-初始海拔高度）
			baro_speed_o = safe_div(baro_height - baro_height_old,dT,0);
			baro_height_old = baro_height;
		}

		//计算速度 
		Moving_Average(speed_av_arr,MONUM ,&speed_av_cnt,baro_speed_o,&speed_av);
		speed_delta = LIMIT(speed_av - baro_speed,-2000*dT,2000*dT);
		LPF_1_(0.5f,dT,speed_delta,speed_delta_lpf);
		baro_speed += speed_delta *LIMIT((ABS(speed_delta_lpf)/(2000*dT)),0,1);
		
}


typedef struct
{
	float in_est_d;   //Estimator
	float in_obs;    //Observation，输入值
	
	float fix_kp;
	float e_limit;

	float e;

	float out;			//输出值
}_fix_inte_filter_st;


_fix_inte_filter_st wcz_h_fus,wcz_hs_fus;
_fix_inte_filter_st wcz_x_fus,wcz_xs_fus;
_fix_inte_filter_st wcz_y_fus,wcz_ys_fus;

s32 wcz_ref_h,wcz_ref_hs;
s32 wcz_ref_x,wcz_ref_xs;
s32 wcz_ref_y,wcz_ref_ys;

u8 high_calc_en = 0;

//滤波，dT=0.01
void fix_inte_filter(float dT,_fix_inte_filter_st *data)
{
	data->out += (data->in_est_d + data->e ) *dT;
	
	data->e = data->fix_kp *(data->in_obs - data->out);

	if(data->e_limit>0)
	{		
		data->e = LIMIT(data->e,-data->e_limit,data->e_limit);
	}
}


//高度融合， dT_ms=10，，，dT_ms*1e-3f=0.01
void High_Data_Calc(u8 dT_ms)
{
	static u16 cnt;
	
	if(spl_flag==0)  //气压标志位 判断气压芯片是否焊接
	{
		return;
	}
	
	if(high_calc_en)
	{
		wcz_ref_h = baro_height; //原始气压高度
		wcz_ref_hs  = baro_speed; //原始气压速度
		
		//速度融合
		wcz_hs_fus.fix_kp = 0.8f;
		wcz_hs_fus.in_est_d = Angle.yaw/10;//偏航角
		wcz_hs_fus.in_obs = wcz_ref_hs; //原始气压速度
		wcz_hs_fus.e_limit = 200;
		fix_inte_filter(dT_ms*1e-3f, &wcz_hs_fus); 
		
		//高度位置融合
		wcz_h_fus.fix_kp = 0.8f;
		wcz_h_fus.in_est_d = wcz_hs_fus.out;//滤波速度
		wcz_h_fus.in_obs = wcz_ref_h;//原始气压高度
		wcz_h_fus.e_limit = 100;
		fix_inte_filter(dT_ms*1e-3f, &wcz_h_fus);
		
		
		FlightData.High.bara_height = wcz_h_fus.out;    //融合高度位置值
		FlightData.High.ultra_height= wcz_hs_fus.out;   //融合高度速度值
		FlightData.High.ultra_baro_height=baro_height;  //原始气压高度值（相对于开机时高度）
		
	} 
	else
	{
		cnt++;
		
		//等待加速度计数据稳定
		if(cnt>200)	high_calc_en = 1;
		else				baro_start   = 1;
	}
}

 void High_Data_Reset(void)
{
	wcz_hs_fus.out = 0;
	wcz_hs_fus.e = 0;

	wcz_h_fus.out = 0;
	wcz_h_fus.e = 0;	
}

