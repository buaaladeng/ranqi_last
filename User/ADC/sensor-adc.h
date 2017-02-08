/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32f10x_adc.h"
#include "stdio.h"
/** @addtogroup SingleChannelContinuous
  * @{
  */ 


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define USE_DMA_Transfer
#define ADC1_DR_Address    ((uint32_t)0x4001244C)
#define DEALDATA   1   //������������  0��ȡ��

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


//ErrorStatus HSEStartUpStatus;
    
/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);

void ADCconfig(void);                                //����ADC
float ReadSensorData(uint16_t data);                 //��ADC�ɼ�����������ת��ΪС��
float Filter(float Original[],int num);              //�Բɼ������ݽ����˲�

