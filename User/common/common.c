/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   串口中断接收测试
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火 iSO-MINI STM32 开发板 
  * 论坛    :http://www.chuxue123.com
  * 淘宝    :http://firestm32.taobao.com
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

extern struct Sensor_Set  DeviceConfig;     //液位计配置信息结构体
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
	u32       TimCount_Current =0;            //现在时间
	TimCount_Current = RTC_GetCounter();         //获取当前时间
	SleepTime=TimCount_Current -Sensor_PowderOn;         //计算工作时间
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
	PWR_BackupAccessCmd(ENABLE);
	PWR_WakeUpPinCmd(ENABLE); //使能WAKE-UP管脚
  
	if(SendCount<1) 
	{
		SendCount =60;                         //防止程序出错，导致设备长期处于休眠状态而无法唤醒，目前设置最长休眠时间为24h
	}
	//Sms_Consult();                          //查阅短信，更新配置参数，后续有待完善
	//RTC_SetAlarm(RTC_GetCounter()+10);   //采集时间到开始唤醒
	RTC_SetAlarm(RTC_GetCounter()+((DeviceConfig.SendCount ) * 60 - SleepTime) );      //??,????
	RTC_WaitForLastTask();


//	GPIO_SetBits(GPIOC,GPIO_Pin_5);          //433模块SET管脚拉高
//	Delay_ms(100);
   GPIO_ResetBits(GPIOB,GPIO_Pin_0);
//	Delay_ms(100);
	
	printf("\r\n采集周期:%d-----上报周期:%d\r\n",DeviceConfig.CollectPeriod,DeviceConfig.SendCount);    //测试使用
	printf("设备进入睡眠状态!\r\n%dmin 后唤醒!!",SendCount);	        //应该考虑两个参数乘积

//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR , ENABLE);
//  PWR_WakeUpPinCmd(ENABLE); //使能WAKE-UP管脚
	PWR_EnterSTANDBYMode();	
	
	
}

