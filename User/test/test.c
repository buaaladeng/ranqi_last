

/***************************������Ϊ��ʼ���豸********************************/
#include "test.h"
#include "time.h"
#include "SensorInit.h"
#include  "DS2780.h"
extern struct Sensor_Set DeviceConfig;

uint8_t SetTime[6]=SETTIME;
//(1)���òɼ������Ϣ���ֱ�Ϊ�ɼ����ڡ��ϴ����ڡ��ش���������Ũ����ֵ����Ũ����ֵ
void write_collect_parameter_to_flash(void)      //���øú����ɽ���ȼ�������ն˲ɼ�����   ����3G�汾
{  
	
	uint8_t collect_period[2]={00,60};     //�ɼ�����
  uint8_t send_out[2]={00,60};           //�ϴ�����
  uint8_t retry_num = 3;                //�ش�����
	uint8_t first_collect[2]={00,00};     //��һ�βɼ�ʱ��
	uint8_t collect_num = 0;              //�Ѳɼ�����
 
	
	union Hex_Float low_alarm;
	union Hex_Float high_alarm;
   low_alarm.Data_Float =25.0;
   high_alarm.Data_Float =50.0;
	printf("\r\n���вɼ�����������...\r\n");
  //PowerON_Flash(); 
	DataWrite_To_Flash(0,2,0,collect_period,2);  
	DataWrite_To_Flash(0,3,0,send_out,2);
  DataWrite_To_Flash(0,7,0,low_alarm.Data_Hex ,4);
	DataWrite_To_Flash(0,8,0,first_collect,2);
	DataWrite_To_Flash(0,9,0,&collect_num,1);
	DataWrite_To_Flash(0,10,0,&(retry_num),1);
	DataWrite_To_Flash(0,11,0,high_alarm.Data_Hex ,4);
	
}

//(2)����3Gģ����Ϣ, ������·���롢IP�˿ںš�PORT�˿ں�
void write_3G_parameter_to_flash(void) 
{
	unsigned char    phone_num[16]={"861064617178004"};             //�����绰����
	unsigned char    ip[16]={"119.254.103.80"};                     //�������IP��ַ
	unsigned char    port[6]={"2017"};                              //�������˿ں�
 
////////////////////������д��FLASH��////////////////////////////
  DataWrite_To_Flash(0,4,0,phone_num,strlen(phone_num));
	DataWrite_To_Flash(0,5,0,ip,strlen(ip));
	DataWrite_To_Flash(0,6,0,port,strlen(port));
}

//(3)����433ģ����Ϣ
void write_433_parameter_to_flash(void) 
{
	
	
}
//(4)����ʱ����Ϣ
void Set_Time(void)
{
//	time_t rawtime;
//  struct tm * timeinfo;
//  //rawtime= time((time_t*)NULL);
//  timeinfo = localtime (&rawtime);
//	DeviceConfig.Time_Year = timeinfo->tm_year;
//	DeviceConfig.Time_Mon  = timeinfo->tm_mon;
//	DeviceConfig.Time_Mday = timeinfo->tm_mday;
//	DeviceConfig.Time_Hour = timeinfo->tm_hour;
//	DeviceConfig.Time_Min  = timeinfo->tm_min;
//	DeviceConfig.Time_Sec  = timeinfo->tm_sec;
	
	DeviceConfig.Time_Year = SetTime[0];
	DeviceConfig.Time_Mon  = SetTime[1];
	DeviceConfig.Time_Mday = SetTime[2];
	DeviceConfig.Time_Hour = SetTime[3];
	DeviceConfig.Time_Min  = SetTime[4];
	DeviceConfig.Time_Sec  = SetTime[5];
	
	
	
	Time_Auto_Regulate(&DeviceConfig);
}

void Set_DS2780(void)
{
	Set_register_ds2780();    //�����Կ��ؼ����³�ʼ��
	set_ACR(1000);           //�����Կ��ؼ����³�ʼ��
	DS2780_CapacityInit();    //���������д�������
}




