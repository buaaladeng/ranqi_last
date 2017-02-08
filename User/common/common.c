/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   �����жϽ��ղ���
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ�� iSO-MINI STM32 ������ 
  * ��̳    :http://www.chuxue123.com
  * �Ա�    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */ 
#include "stm32f10x.h"
#include "misc.h"
#include "stm32f10x_it.h"
#include "bsp_usart.h"
#include "bsp_TiMbase.h" 
#include "bsp_SysTick.h"
#include "string.h"
#include "bsp_rtc.h"
#include "bsp_date.h"
#include "WatchDog.h"
#include "common.h"
#include "gprs.h"
#include "SensorInit.h"

extern struct Sensor_Set  DeviceConfig;     //Һλ��������Ϣ�ṹ��
extern u32 Sensor_PowderOn;
/*******************************************************************************
* Function Name  : XX
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void gotoSleep(uint16_t SendCount)
{
	u32       SleepTime;
	u32       TimCount_Current =0;            //����ʱ��
	TimCount_Current = RTC_GetCounter();         //��ȡ��ǰʱ��
	SleepTime=TimCount_Current -Sensor_PowderOn;         //���㹤��ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
	PWR_BackupAccessCmd(ENABLE);
	PWR_WakeUpPinCmd(ENABLE); //ʹ��WAKE-UP�ܽ�
  
	if(SendCount<1) 
	{
		SendCount =60;                         //��ֹ������������豸���ڴ�������״̬���޷����ѣ�Ŀǰ���������ʱ��Ϊ24h
	}
	//Sms_Consult();                          //���Ķ��ţ��������ò����������д�����
	//RTC_SetAlarm(RTC_GetCounter()+10);   //�ɼ�ʱ�䵽��ʼ����
	RTC_SetAlarm(RTC_GetCounter()+((DeviceConfig.SendCount ) * 60 - SleepTime) );      //??,????
	RTC_WaitForLastTask();


//	GPIO_SetBits(GPIOC,GPIO_Pin_5);          //433ģ��SET�ܽ�����
//	Delay_ms(100);
   GPIO_ResetBits(GPIOB,GPIO_Pin_0);
//	Delay_ms(100);
	
	printf("\r\n�ɼ�����:%d-----�ϱ�����:%d\r\n",DeviceConfig.CollectPeriod,DeviceConfig.SendCount);    //����ʹ��
	printf("�豸����˯��״̬!\r\n%dmin ����!!",SendCount);	        //Ӧ�ÿ������������˻�

//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR , ENABLE);
//  PWR_WakeUpPinCmd(ENABLE); //ʹ��WAKE-UP�ܽ�
	PWR_EnterSTANDBYMode();	
	
	
}

