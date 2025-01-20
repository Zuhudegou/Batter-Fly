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


#include "sys.h"
#include "ADC.h"

//adc用来测量电池电压
//管脚 B0

/***************ADC GPIO??******************/
#define RCC_GPIO_ADC	RCC_APB2Periph_GPIOB
#define GPIO_ADC			GPIOB
#define GPIO_Pin_ADC	GPIO_Pin_0
#define ADC_Channel   ADC_Channel_8
/*********************************************/

//#define ADC1_DR_Address    ((u32)0x40012400+0x4c)

#define ADC3_DR_Address    ((u32)0x40013C4C)
#define ADC1_DR_Address    ((uint32_t)0x4001244C)

__IO uint16_t ADC_ConvertedValue[2];


/*
 * 函数名：ADC1_GPIO_Config
 * 描述  ：使能ADC1和DMA1的时钟，初始化PC.01
 * 输入  : 无
 * 输出  ：无
 * 调用  ：内部调用
 */
static void ADC1_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/* Enable DMA clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	/* Enable ADC1 and GPIOC clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_GPIO_ADC, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_ADC ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIO_ADC, &GPIO_InitStructure); //PB0 = ADC_Channel_8
}


/* 函数名：ADC1_Mode_Config
 * 描述  ：配置ADC1的工作模式为MDA模式
 * 输入  : 无
 * 输出  ：无
 * 调用  ：内部调用
 */
static void ADC1_Mode_Config(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	
	/* DMA channel1 configuration */
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;	 //ADC地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_ConvertedValue;//内存地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 2;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址固定
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存地址固定
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	//半字
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;		//循环传输
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	
	/* Enable DMA channel1 */
	DMA_Cmd(DMA1_Channel1, ENABLE);
	
	/* ADC1 configuration */
	ADC_DeInit(ADC1);//新增
	ADC_TempSensorVrefintCmd(ENABLE);//新增，使能内部参照电压
	
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//独立ADC模式
	ADC_InitStructure.ADC_ScanConvMode = ENABLE ; 	 //禁止扫描模式，扫描模式用于多通道采集
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	//开启连续转换模式，即不停地进行ADC转换
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//不使用外部触发转换
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; 	//采集数据右对齐
	//ADC_InitStructure.ADC_NbrOfChannel = 1;	 	//要转换的通道数目
	ADC_InitStructure.ADC_NbrOfChannel = 2;	 	//要转换的通道数目
	ADC_Init(ADC1, &ADC_InitStructure);
	
	/*配置ADC时钟，为PCLK2的6分频，即12MHz,ADC频率最高不能超过14MHz*/
	RCC_ADCCLKConfig(RCC_PCLK2_Div6); 
	/*配置ADC1的通道11为55.	5个采样周期，序列为1 */ 
	//ADC_RegularChannelConfig(ADC1, ADC_Channel, 1, ADC_SampleTime_55Cycles5); 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_55Cycles5); 	//电池电压 ADC_ConvertedValue[0]，PB0 = ADC_Channel_8
	ADC_RegularChannelConfig(ADC1, ADC_Channel_17, 2, ADC_SampleTime_55Cycles5); 	//内部参照电压 ADC_ConvertedValue[1]
	
	// 通道0 ~通道7  = PA0~PA7
	// 通道8 ~通道9  = PB0~PB1
	// 通道10~通道15 = PC0~PC5
	// 通道16 = 内部温度传感
	// 通道17 = 内部参考电压

	/* Enable ADC1 DMA */
	ADC_DMACmd(ADC1, ENABLE);
	
	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);
	
	/*复位校准寄存器 */   
	ADC_ResetCalibration(ADC1);
	/*等待校准寄存器复位完成 */
	while(ADC_GetResetCalibrationStatus(ADC1));
	
	/* ADC校准 */
	ADC_StartCalibration(ADC1);
	/* 等待校准完成*/
	while(ADC_GetCalibrationStatus(ADC1));
	
	/* 由于没有采用外部触发，所以使用软件触发ADC转换 */ 
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

/*
 * 函数名：ADC1_Init
 * 描述  ：无
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
 */
void ADC1_Init(void)
{
	ADC1_GPIO_Config();
	ADC1_Mode_Config();
}

s16 voltage = 4000;//单位 1mv
#define power0 3700
#define power1 3750

void Voltage_Check()//20HZ
{
	static u16 cnt0,cnt1;
	
	//voltage += 0.2f *(2 *(3300 *ADC_ConvertedValue[0]/4096) - voltage);

	voltage += 0.2f *( 2.0f*ADC_ConvertedValue[0]/ADC_ConvertedValue[1]*1.2f*1000 - voltage);
	//0.2:滤波系数，2.0:测量分压1/2，1.2:参考电压1.2v，1000:代表1v显示1000
	
	if(ALL_flag.unlock)//飞行过程中不判断低压
	{
		return;
	}
	else//不飞行的时候的低压判断
	{
		if(voltage < power0 && voltage >3400)//低压
		{
			cnt0++;
			cnt1=0;
			if(cnt0>100)
			{
				cnt0 = 100;
//				if(LED_warn==0)
//				{
//					flag.low_power=1;
//					LED_warn = 1;
//				}
			}
		}
		else if(voltage > power1)//正常
		{
			cnt1++;
			cnt0=0;
			if(cnt1>100)
			{
				cnt1 = 100;
//				if(LED_warn==1)
//				{
//					flag.low_power=0;
//					LED_warn = 0;
//				}
			}
		}
		else
		{
			cnt0=0;
			cnt1=0;
		}
	}
}