#ifndef _GPRS_H_
#define _GPRS_H_
#include "stm32f10x.h"
#include "AiderProtocol.h"


#define DEBUG_WORK 1   //��ӡ���ܣ�Ӧ����������ʾ


///////////////////////////////////////////////////////////////////////////////
//�����ṹ��
struct SMS_Config_RegPara 
{
	
	char     CurrentPhoneNum[16];      //��ǰͨ���ֻ����룬�ַ�����ʽ
	char     AlarmPhoneNum[16];   //��������Һλ�������룬�ַ�����ʽ
	char     ServerIP[16];     //�������÷�����IP���ַ�����ʽ
	char     ServerPort[6];    //�������÷������˿ںţ��ַ�����ʽ
  
	union   Hex_Float LowAlarmLevel;
	union   Hex_Float HighAlarmLevel;
		
	
	uint8_t  CollectPeriod_Byte[2];    //���ݲɼ������ʹ������洢���Է���Flash��д
	uint8_t  SendCount_Byte[2];        //һ���ϴ�Һλ���ݴ�����ʹ������洢���Է���Flash��д  �ϴ�����
	uint8_t  CollectStartTime_Byte[2];     //��ʼ�ɼ�ʱ��
	u8       CollectNum;                  //�Ѿ���ɵĲɼ�����
	uint8_t  RetryNum;                    //�ش�����
	uint8_t  WorkNum;                     //��������
	
	uint8_t  SensorDataInquireFlag;    //���Ų�ѯ��ǰ���������ݱ�־����
	uint8_t  DeviceSetInquireFlag;     //���Ų�ѯ��ǰ ������������Ϣ��־����
	
};

//��������

void  mput(char *str);
void  mput_mix(char *str,int length);
char* Find_String(char *Source, char *Object);
char* Find_SpecialString(char* Source, char* Object, short Length_Source, short Length_Object);

void  USART_DataBlock_Send(USART_TypeDef *USART_PORT,char *SendUartBuf,u16 SendLength);   //�����򴮿ڷ�������

void  ConfigData_Init(struct Sensor_Set* Para);

void  ParameterConfig_SX1278(char* pRecevCommand);  //����433ģ��
void  ParameterInquire_SX1278(u8 InquireCommand);    //��ѯ433������Ϣ
u16 Receive_Data_Analysis(u8* pDeviceID, u16 sNodeAddress);                    //����433���ڽ��յ�����

void Treaty_Data_Analysis(u8* pTreatyBuff, u16* pFlag, u8* pDeviceID, u16 NodeAddress);	
uint16_t char_to_int(char* pSetPara);               //��������·������ã���Ҫ���ַ�ת��Ϊ����
float    char_to_float(char* pSetPara);               //��������·������ã���Ҫ���ַ�ת��ΪС��
             
void mput_mix_sx1278(char *str,int length);           //����433ģ�鷢�����ݣ����ݷ�������Ժ��������SET���ţ�����ģ�龭�����ղ�������
#endif

