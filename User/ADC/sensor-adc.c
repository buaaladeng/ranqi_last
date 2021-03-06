/**
  ******************************************************************************
  * @file    SingleChannelContinuous/adc
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    03/09/2010
  * @brief   Main program body
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */ 

#include "sensor-adc.h"
  
/* Private functions ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
__IO uint16_t ADC1ConvertedValue = 0;

/**
  * @brief  ADCconfig
  * @param  None
  * @retval None
  */
void ADCconfig(void)                       //change by gao 
{
  /* System clocks configuration ---------------------------------------------*/
  //RCC_Configuration();

	ADC_InitTypeDef  ADC_InitStructure;
  DMA_InitTypeDef  DMA_InitStructure;
	
  /* GPIO configuration ------------------------------------------------------*/
  GPIO_Configuration();

#ifndef   USE_DMA_Transfer 
  /* NVIC configuration ------------------------------------------------------*/
  NVIC_Configuration();

#else 
  /* DMA1 channel1 configuration ---------------------------------------------*/
	//printf("\r\nEnter ADC Config!\r\n");                //进入ADC初始化设置
  DMA_DeInit(DMA1_Channel1);                           //不同的引脚，端口号不一样        PA1=ADC1
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADC1ConvertedValue;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 1;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);  
  
  /* Enable DMA1 channel1 */
  DMA_Cmd(DMA1_Channel1, ENABLE);
#endif
  
  /* ADC1 configuration ------------------------------------------------------*/
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

   /*??ADC??,?PCLK2?8??,?9MHz*/
	 RCC_ADCCLKConfig(RCC_PCLK2_Div8);
  //RCC_ADCCLKConfig(RCC_PCLK2_Div8);                           //测量温度时需要

  /* ADC1 regular channels configuration */ 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_55Cycles5);    //   55->239 通道也有变
  
	//ADC_TempSensorVrefintCmd(ENABLE);                    //使能温度测量
	
#ifdef   USE_DMA_Transfer   
  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE);
#else
    /* Enable ADC1 EOC interupt */
  ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
#endif  
  
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);
  
  /* Enable ADC1 reset calibaration register */   
  ADC_ResetCalibration(ADC1);
  
  /* Check the end of ADC1 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC1));

  /* Start ADC1 calibaration */
  ADC_StartCalibration(ADC1);
  
  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADC1));
     
  /* Start ADC1 Software Conversion */ 
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  
 //printf("\r\nEnd ADC Config!\r\n");                //结束ADC初始化设置
}


/**
  * @brief  Configures the different GPIO ports.
  * @param  None
  * @retval None
  */
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable DMA1 clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  
/* Enable peripheral clocks --------------------------------------------------*/
  /* Enable ADC1 and GPIOC clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE);                //实际电路板用PA1  测试使用PB0
	
  /* Configure PA.01 (ADC Channel14) as analog input -------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/**
  * @brief  Configures Vector Table base location.
  * @param  None
  * @retval None
  */
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure and enable ADC interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
* Function Name  : float ReadSensorData(uint16_t data)
* Description    : 将ADC采集的数据进行转换为FLOAT
* Input          : ADC采集的数据
* Output         : ADC采集的小数形式
* Return         : None
*******************************************************************************/
float ReadSensorData(uint16_t data)
{
	float ADC_Data;
	float Result_Data;
	float float_temp;
	float int_temp;
	int Result_IntData;
#if DEBUG_TEST
	printf("\r\nThe Original ADC data is %d \r\n",data);          //读取ADC的原始值
#endif
	ADC_Data = (float)data/4096*3.3;                          //电压的浮点形式
	Result_Data = ADC_Data * 40;                              //利用2.5V进行转换
  if(Result_Data<0)
	Result_Data=0;                                          //进行零漂处理
	else if(Result_Data>100.0)
	Result_Data=100.0;                                      //进行上限处理
/***************************************************************************/	
#if DEALDATA > 0	
  Result_IntData=(int)(Result_Data);
	float_temp=Result_Data-Result_IntData;
	float_temp*=10;
	int_temp=(int)(float_temp);
	if(int_temp<5)
	Result_Data=(float)Result_IntData;
	else
	Result_Data=(float)(Result_IntData+1);
#endif
/***************************************************************************/	
#if DEBUG_TEST	
	printf("\r\nThe Sensor Data is %0.2f\r\n",Result_Data);
#endif
	return(Result_Data);
}
/*******************************************************************************
* Function Name  : 平均数法Filter
* Description    : 对传感器采集的数据进行滤波处理
* Input          : 采集的数据
* Output         : 滤波后的数据
* Return         : None
*******************************************************************************/
float Filter(float Original[],int num)
{
	float max=0;                      //最大的数
	float min=0;											//最小的数
	float sum=0;                      //总和
	float averge=0;                   //平均数输出
	int i;
	
	
	//判断输入参数

	if( num>10 )
		printf("\r\nTHE INPUT NUMBER IS MORE THAN 10,THIS FUNCTION WILL NOT WORK!\r\n");
	else if(num < 3)
		printf("\r\nTHE INPUT NUMBER IS LESS THAN 3,THIS FUNCTION WILL NOT WORK!\r\n");
	else
	{
		max=Original[0];
		min=Original[0];                       //进行初始化
		for(i=0;i<num;i++)
		{ 
			if(max<Original[i])
		  max=Original[i];
			if(min>Original[i])
			min=Original[i];
		}                                   //求出数组的最大项和最小项
#if DEBUG_TEST		
		printf("\r\n THE MAX is %f, The MIN is %f",max,min);
#endif
	 if(max-min>8.0)                       //进行数据筛选
	 {
		 for(i=1;i<num-1;i++)
		 sum=sum+Original[i];
		 averge=sum /(num-2);                //去掉最大值和最小值 
		 
   }
	 else
	 {
		 for(i=0;i<num;i++)
		 sum=sum+Original[i];
		 averge=sum /num;                //所有数据的平均数
	 }
#if DEBUG_TEST
	printf("\r\nTHE averge is %f\r\n",averge);
#endif	 

	}
	return averge;
}



#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
//void assert_failed(uint8_t* file, uint32_t line)
//{ 
//  /* User can add his own implementation to report the file name and line number,
//     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

//  /* Infinite loop */
//  while (1)
//  {
//  }
//}
#endif

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
