
/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   ADC
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火 F103-霸道 STM32 开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */ 
#include "./bsp/bsp_adc.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"

ADC_HandleTypeDef hadcx;
DMA_HandleTypeDef hdma_adcx;

void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
   /* 外设时钟使能 */
    ADCx_RCC_CLK_ENABLE();
    
    /* DMA时钟使能 */
    DMAx_RCC_CLK_ENABLE();

    /* DMA外设初始化配置 */  
    hdma_adcx.Instance = ADC_DMAx_CHANNELn;
    hdma_adcx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adcx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adcx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adcx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adcx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adcx.Init.Mode = DMA_CIRCULAR;
    hdma_adcx.Init.Priority = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&hdma_adcx);
    /* 连接DMA */
    __HAL_LINKDMA(hadc,DMA_Handle,hdma_adcx);
}

void ADCx_Init(void)
{
  ADC_ChannelConfTypeDef sConfig;

  // ADC功能配置
  hadcx.Instance = ADCx;
  hadcx.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadcx.Init.ContinuousConvMode = ENABLE;
  hadcx.Init.DiscontinuousConvMode = DISABLE;
  hadcx.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadcx.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadcx.Init.NbrOfConversion = 1;
  HAL_ADC_Init(&hadcx);

  // 配置采样通道
  sConfig.Channel = ADC_CHANNEL;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  HAL_ADC_ConfigChannel(&hadcx, &sConfig);
}



#define V25  0x6EE                  //对于12位的ADC，3.3V的ADC值为0xfff,温度为25度时对应的电压值为1.43V即0x6EE
#define AVG_SLOPE 0x05              //斜率 每摄氏度4.3mV 对应每摄氏度0x05
__IO uint16_t Current_Temperature;  // 用于保存转换计算后的电压值  
uint16_t ADC_ConvertedValue;        // AD转换结果值
void ADC_Test(void)
{
    uint8_t print_flag = 0;
    if(print_flag == 0)
    {
        printf("\r\n 这是一个内部温度传感器实验 \r\n");
        printf( "\r\n Print current Temperature  \r\n");	
        HAL_ADCEx_Calibration_Start(&hadcx);
        HAL_ADC_Start_DMA(&hadcx,(uint32_t *)&ADC_ConvertedValue,sizeof(ADC_ConvertedValue)); 
	}
    /* 无限循环 */
    while (1)
    {
        delay_ms(5);

        Current_Temperature = (V25-ADC_ConvertedValue)/AVG_SLOPE+25;	
        printf("The IC current temp = %3d ℃\n",Current_Temperature);
    }
}

