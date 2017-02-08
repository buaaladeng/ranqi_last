
#include "SensorInit.h"
#include "test.h"
#include "433_Wiminet.h"
#include "bsp_SysTick.h"


void Device_Init()
{
	u8   DeviceID[6] = SENSORID ;                    //��ʼ���豸ID��
  u16  NodeAddr =0x0000;                           //��ȡ�豸ID������������ֽ���Ϊ�ڵ��ַ
	u16  NodeID = 0;
  NodeAddr=DeviceID[4]*256 +DeviceID[5];
	printf("\r\n***************�豸�ָ�����������...****************\r\n");
	write_collect_parameter_to_flash();          //���Ժ���   �����豸�ɼ������ĳ�ʼ������
  //Set_Time();																 //���Ժ���   �����豸ʱ�������
  //Set_DS2780();
	SX1287_Init(NodeAddr,NETWORKID);                     //����433ģ���ʼ������
	printf("\r\n*************** �ָ������������ ****************\r\n");
	Delay_ms(2000);   
}




