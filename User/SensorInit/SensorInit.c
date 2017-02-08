
#include "SensorInit.h"
#include "test.h"
#include "433_Wiminet.h"
#include "bsp_SysTick.h"


void Device_Init()
{
	u8   DeviceID[6] = SENSORID ;                    //初始化设备ID号
  u16  NodeAddr =0x0000;                           //提取设备ID号最后面两个字节作为节点地址
	u16  NodeID = 0;
  NodeAddr=DeviceID[4]*256 +DeviceID[5];
	printf("\r\n***************设备恢复出厂设置中...****************\r\n");
	write_collect_parameter_to_flash();          //测试函数   进行设备采集参数的初始化设置
  //Set_Time();																 //测试函数   进行设备时间的设置
  //Set_DS2780();
	SX1287_Init(NodeAddr,NETWORKID);                     //进行433模块初始化设置
	printf("\r\n*************** 恢复出厂设置完毕 ****************\r\n");
	Delay_ms(2000);   
}




