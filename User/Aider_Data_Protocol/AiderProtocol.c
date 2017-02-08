#include "stm32f10x.h"
#include "gprs.h"
#include "bsp_SysTick.h"
#include "modbus.h"
#include "string.h"
#include "AiderProtocol.h"
#include "433_Wiminet.h"
#include "bsp_rtc.h"
#include "SPI_Flash.h"
#include "API-Platform.h"

//#include  "bsp_date.h"
////////////////////////////////////////////////////////////////////////////////////////////////////////////
/***********************************���������붨��***************************************************/ 
extern  struct Sensor_Set  DeviceConfig;          //�豸������Ϣ�ṹ��
extern  struct SMS_Config_RegPara   ConfigData;     //������·����ò�����HEX��ʽ��������ϵͳ��������ģʽ֮ǰд��FLASH
struct   DataFrame         StartupSend;
struct   SpecialDataFrame  TrapSend;
struct   DataFrame         GetRespSend;
extern struct   SenserData  ReportUp[MAX_COLLECTNUM];
extern u8     DataCollectCount;                   //���ݲɼ�������
u8     MessageSendSeq=0;                       //����ͨ�ŷ�ʽ��־λ
extern struct    rtc_time        systmtime;        //ϵͳʱ��
extern uint8_t   Usart3_send_buff[300];
extern uint8_t   DMA_UART3_RECEV_FLAG ;      //USART3 DMA���ձ�־����
extern unsigned char   Usart2_send_buff[SENDBUFF_SIZE];     //433����������
extern unsigned int Get_Rand_Num(void);                                 //��ȡ�������

/***********************************��������***************************************************/ 
extern u16  DMA_UART3_RecevDetect(u8* pDeviceID, u16 sNodeAddress);


/*******************************************************************************
* Function Name  : UploadFlash
* Description    : ����FLASHд��
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint8_t  UploadFlash(char* pSetPara, uint8_t  InstructCode)
{
  uint8_t   j=0;
  uint8_t   Counter=0;

	switch(InstructCode)
	{
		
		case 2:          //�洢�ɼ�����
		{   
			 for(j=0;j<strlen(pSetPara);j++)
  		 //printf("\r\nCollect Period Before Write Into Flash:-%x-\r\n",pSetPara[j]);
			 Delay_ms(500); 
			 DataWrite_To_Flash(0,2,0,(uint8_t*)pSetPara,2);      //���ɼ�����д��Flash
			 DeviceConfig.MessageSetFlag =1;                    //��ʾ����ɲ����޸�	
       printf("\r\n��ɲɼ����ڴ洢\r\n");			          //����ʹ��						
			 return 1;
		}
		case 3:          //�洢�ϴ�����
		{  
//			 for(j=0;j<strlen(pSetPara);j++)
//			 printf("\r\nSend Period Before Write Into Flash:-%x-\r\n",pSetPara[j]);
			 Delay_ms(500); 
			 DataWrite_To_Flash(0,3,0,(uint8_t*)pSetPara,2);      //���ϴ�����д��Flash
			 DeviceConfig.MessageSetFlag =1;                      //��ʾ����ɲ����޸�	
       //printf("\r\n send period: %s\r\n",pSetPara);			          //����ʹ��		
       printf("\r\n����ϴ����ڴ洢\r\n");			          //����ʹ��					
			 return 1;
		}
		
		case 4:            //�洢��������
		{
			  //�ֻ�������б�������������Ҫ���ӡ�
			 DataWrite_To_Flash(0,4,0,(uint8_t*)pSetPara,strlen(pSetPara));   //����������д��Flash 
       DeviceConfig.MessageSetFlag =1;                                  //��ʾ��ǰҺλ�ǲ���ͨ�������޸�		 
       printf("\r\n Alarm Phone Number:%s\r\n",pSetPara);			          //����ʹ��
			 return 1;
		}
		
		case 5:              //�洢������IP
		{

			 for(j=0,Counter=0;j<strlen(pSetPara);j++)                       //���ú����Ѿ����˷��������,strlen(pSetPara)<=15
			 {
          if((pSetPara[j]>='0')&&(pSetPara[j]<='9'))
					{
//             CharTemp[j] =pSetPara[j];
						 ;
          }
				  else if(pSetPara[j]=='.')                                    //�ָ���ͳ��
					{
						 Counter++;
          }
					else
					{
              break;
          }
       }
			 if(Counter==3)
			 {
          DataWrite_To_Flash(0,5,0, (uint8_t*)pSetPara,strlen(pSetPara));   //��������IPд��Flash 
       }
			 else
			 {
          printf("\r\nInput Server IP ERROR!!\r\n");
       }
			 DeviceConfig.MessageSetFlag =1;     //��ʾ��ǰҺλ�ǲ���ͨ�������޸�		 
			  printf("\r\n srever ip: %s\r\n",pSetPara);			          //����ʹ��
			 return 1;
			 
		}
		
		case 6:           //�洢�������˿ں�
		{
			 DataWrite_To_Flash(0,6,0,(uint8_t*)pSetPara,strlen(pSetPara));      //���������˿ں�д��Flash
			 DeviceConfig.MessageSetFlag =1;     //��ʾ��ǰҺλ�ǲ���ͨ�������޸�	
        printf("\r\n server port: %s\r\n",pSetPara);			          //����ʹ��			
			 return 1;
			
		}
		case 7:            //�洢��Ũ�ȱ�����ֵ
		{
			 
		   DataWrite_To_Flash(0,7,0,(uint8_t*)pSetPara , 4);   //��������ֵ��Hex��ʽ��д��Flash 
			 DeviceConfig.MessageSetFlag =1;                       //��ʾ����ɲ����޸�	
			 printf("\r\n Low Alarm Threshold: %s\r\n",pSetPara);			          //����ʹ��
			 return 1;
		}
		
	
		case 8:          //�洢��һ���ϱ�ʱ��
		{  
			 for(j=0;j<strlen(pSetPara);j++)
			 printf("\r\n---------%x----------\r\n",pSetPara[j]);
		   DataWrite_To_Flash(0,8,0,(uint8_t*)pSetPara,2);      //����һ���ϱ�ʱ��д��Flash
			 DeviceConfig.MessageSetFlag =1;                     //��ʾ����ɲ����޸�	
       printf("\r\n start collect time: %s\r\n",pSetPara);			          //����ʹ��						
			 return 1;
		}
		
		case 9:          //�洢һ�βɼ�������
		{   
			 for(j=0;j<strlen(pSetPara);j++)
			 printf("\r\n---------%x----------\r\n",pSetPara[j]);
		   DataWrite_To_Flash(0,9,0,(uint8_t*)pSetPara,2);      //��һ�βɼ�����д��Flash
			 DeviceConfig.MessageSetFlag =1;                     //��ʾ����ɲ����޸�	
       printf("\r\n collect number: %s\r\n",pSetPara);			          //����ʹ��						
			 return 1;
		}
		case 10:          //�洢�ش�����
		{
			 Delay_ms(500); 
		   DataWrite_To_Flash(0,10,0,(uint8_t*)pSetPara,1);      //���ش�����д��Flash
			 DeviceConfig.MessageSetFlag =1;     //��ʾ��ǰҺλ�ǲ���ͨ�������޸�		 
			 //printf("\r\nBefore Write Into Flash:-%x-\r\n",pSetPara[0]);			          //����ʹ��	
       printf("\r\n����ش������洢\r\n");			          //����ʹ��					
			 return 1;
		}
		
			case 11:            //�洢��Ũ�ȱ�����ֵ
		{
			 
		   DataWrite_To_Flash(0,11,0,(uint8_t*)pSetPara , 4);   //��������ֵ��Hex��ʽ��д��Flash 
			 DeviceConfig.MessageSetFlag =1;     //��ʾ��ǰҺλ�ǲ���ͨ�������޸�		 
			 printf("\r\n High Alarm Threshold: %s\r\n",pSetPara);			          //����ʹ��
			 return 1;
		}
		
		default:
		{
			 printf("\r\nInstruct Code ERROR !!\r\n");
		   return 0;
		}
  }

}

/*******************************************************************************
* Function Name  : WakeupResponse
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void WakeupResponse(u8* pSendBuff, u8* pDeviceID, u16 NodeAddress)
{
			
	struct   DataFrame  WakeupSend;
	struct   DataFrame* pDataFrame =&WakeupSend;
	char*    pChar =NULL;
  u8       SendCounter =1;     //���ʹ���������
	u16      RecevFlag =0;       //���������ݽ��ձ�־����
	u16      CrcData=0;
	u8       i=0,j=0;
	u8       ValidLength =0;     //��Ч���ݳ��ȣ�ָ1��Tag�ռ���ʵ�ʴ洢��Ч���ݵĳ��ȡ�
	const u8 PreLength   =16;    //֡ǰ׺���ȣ�ָ��֡ǰ���뵽OID���У�����Tag���У������ݳ��ȣ�����֡ǰ���룬��������OID���У�����Tag���У�
	const u8 CoreLength  =12;    //�ؼ���Ϣ���ȣ�ָ���ݾ��ɲ��ֳ����ֶ�ָʾ����ֵ��ȥ��OID���У�����Tag���У���ʣ�����ݵĳ���
	
	pChar =(char*)pDataFrame;
	memset(pChar,0x00,sizeof(struct DataFrame));   //��ʼ���ṹ��
	WakeupSend.Preamble =0xA3;
	WakeupSend.Version  =0x20;
	for(i=0;i<6;i++)
	{
    WakeupSend.DeviceID[i] =pDeviceID[i];
  }
	WakeupSend.RouteFlag =0x01;
	WakeupSend.NodeAddr  =ntohs( NodeAddress);               //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
	WakeupSend.PDU_Type  =(0x0B<<8)+(1<<7)+4;
	WakeupSend.PDU_Type  =ntohs(WakeupSend.PDU_Type);        //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
	WakeupSend.Seq       =1;
	
	WakeupSend.TagList[0].OID_Command =ntohl(DEVICE_WAKEUP); //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
	WakeupSend.TagList[0].Width =1;
	WakeupSend.TagList[0].Value[0]=1;
	WakeupSend.Tag_Count =1;
	WakeupSend.Length =CoreLength + WakeupSend.TagList[0].Width +6;  
	WakeupSend.Length =ntohs(WakeupSend.Length );
	WakeupSend.CrcCode=0xffff;                                        //CRC�ֶθ���ֵ

	memcpy(pSendBuff,pChar,PreLength);
	for(i=0,j=PreLength;i<WakeupSend.Tag_Count;i++)
	{
    
		pChar = (char*)&(WakeupSend.TagList[i].OID_Command);   
		ValidLength =WakeupSend.TagList[i].Width+6;            //����1��Tagʵ��ռ�õ��ֽڿռ�
		if((j+ValidLength) >=(SENDBUFF_SIZE-3))                //��ָֹ�����
		{
       break;
    }
		WakeupSend.TagList[i].Width =ntohs(WakeupSend.TagList[i].Width); //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
	  memcpy((pSendBuff+j),pChar,ValidLength); 
		j = j+ValidLength;
		
  }
	pSendBuff[j++] = WakeupSend.CrcCode &0xff;
	pSendBuff[j++] = WakeupSend.CrcCode>>8;
	
  CrcData = CRC16(pSendBuff, j );   // Update the CRC value
	WakeupSend.CrcCode =CrcData;
	pSendBuff[j-2] = CrcData&0xff;   //CRC���ֽ���ǰ
	pSendBuff[j-1] = CrcData>>8;     //CRC���ֽ��ں�

	if(DeviceConfig.RetryNum>0)
	{
     SendCounter =DeviceConfig.RetryNum;
  }
	else
	{
     SendCounter =1;
  }
	for(i=0;i< SendCounter;i++)
	{
     RecevFlag = SendMessage(pSendBuff, j, pDeviceID, NodeAddress);
//		 Delay_ms(2000); 
		if(RecevFlag != 0)
			break;
		//	 Delay_ms(2000); 
  }
	printf("\r\n----Length:%d----\r\n",j);   //����ʹ��
	for(i=0;i<j;i++)
	{
    printf("%x ",pSendBuff[i]);   //����ʹ��
  }
}

/**************************************************************************
* Function Name  : SendDataToServ()
* Description    : ͨ��Э���ʵ��
* Input          : �������ͣ��������飬�豸ID
* Output         : None
* Return         : None
**************************************************************************/
void SendDataToServ(CommunicateType CommType,struct TagStruct TagList[],uint8_t TagNum,u8* pSendBuff, u8* pDeviceID)
{
  DeviceType DevType 				= DEVICE;        //��ʼ���豸���
	ReportDataType	RDataType =	REPORTDATA;    //��ʼ���ϱ�����
  SendType   SType; 	                       //ͨ�ŷ�ʽ����
	struct   DataFrame  SendData;             //�������ݸ�ʽ
	struct   DataFrame* pDataFrame =&SendData;  //��������ָ��
	struct   TagStruct* pTag=TagList;           //����TAGָ��
	
	char*    pChar =NULL;
  u16      RecevFlag =0;           //���������ݽ��ձ�־����
	uint16_t NodeAddress;            //�豸��ַ
	uint8_t  SendCounter=1;          //���ʹ���
	u16      CrcData=0;
	u8       i=0,j=0;
	uint8_t  RevTagNum = 0;           //TAG������
	unsigned int rand_num;        //����ӳٱ��� ��Χ��0~1000
	uint16_t CollectTime;         //�ɼ�ʱ��
	uint8_t  readdata[7];         //��ȡFLASH������

	u8       ValidLength =0;     //��Ч���ݳ��ȣ�Tag�ռ���ʵ�ʴ洢��Ч���ݵĳ��ȡ�
	const u8 PreLength   =16;    //֡ǰ׺���ȣ�ָ��֡ǰ���뵽OID���У�����Tag���У������ݳ��ȣ�����֡ǰ���룬��������OID���У�����Tag���У�
	const u8 CoreLength  =12;    //�ؼ���Ϣ���ȣ�ָ���ݾ��ɲ��ֳ����ֶ�ָʾ����ֵ��ȥ��OID���У�����Tag���У���ʣ�����ݵĳ���
	const u8 TagPreLength=6;     //TAG��OID����Ϊ4+TAG���ݳ���2
	uint8_t  TagLength=0;        //TAG���ܳ���
	uint8_t  Offset=0;           //ָ��ƫ����
	u8       TotalLength;        //��ȥCRC�������ܳ���
	NodeAddress = pDeviceID[4]*256 +pDeviceID[5];
	pChar =(char*)pDataFrame;
	
	memset(pChar,0x00,sizeof(struct DataFrame));   //��ʼ���ṹ��
	SendData.Preamble =0xA3;
	SendData.Version  = SOFTVERSION;
	for(i=0;i<6;i++)
	{
   SendData.DeviceID[i] = pDeviceID[i];
  }
	
	SendData.NodeAddr  =ntohs( NodeAddress);               //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
	SendData.PDU_Type  =(CommType<<8)+(1<<7)+DevType;
	SendData.PDU_Type  =ntohs(SendData.PDU_Type);        //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
	if(MessageSendSeq==1)
		{
			SType   = TYPE_SMS;                          //���ŷ���
			MessageSendSeq=0;                            //���ݴ���ָ�Ϊ433
		}
		else 
		{SType    = TYPE_433; }                      //ֱ�ӷ�����������
	SendData.RouteFlag =SType;                     //���ݷ��ͷ�ʽ
	SendData.Seq =1;	
	if(pTag!=NULL)
		{
				RevTagNum=TagNum;                     //���ָ�벻�գ���֤�������ݴ������������TAG������
				for(i=0;i<RevTagNum;i++)
				SendData.TagList[i] = TagList[i];
		} 
	else
		{
	  switch(CommType)
	 {
				
		case GETRESPONSE:
		{
   
			break;
		}
		
		case TRAPREQUEST:
		{ 
			
				SendData.TagList[0].OID_Command = ntohl(DEVICE_QTY);   //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
				SendData.TagList[0].Width =1;                         //
				SendData.TagList[0].Value[0] =DeviceConfig.BatteryCapacity;  //�����ϴ�ʱ�ĵ��ʣ�����		  
	
				SendData.TagList[1].OID_Command= ntohl(SYSTERM_DATA);    //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
				SendData.TagList[1].Width =3;                            //
				SendData.TagList[1].Value[0] =systmtime.tm_year-2000;       //ϵͳ���ڣ���
				SendData.TagList[1].Value[1] =systmtime.tm_mon ;            //ϵͳ���ڣ���
				SendData.TagList[1].Value[2] =systmtime.tm_mday ;           //ϵͳ���ڣ���
			
			  
/***********************************��ͬ���豸��Ҫ�����޸��ϱ�TAG*********************************************/	
			
				for(i=0; i< DataCollectCount ;i++)    //��ÿһ���ϱ�����Tag��ֵ
				{
	///////////////////////////////////////��FLASH�ڵ����ݶ�ȡ���������ϱ�////////////////////////////////
				DataRead_From_Flash(1,i+1,0, readdata ,sizeof(readdata));
				CollectTime = readdata[0] *256 + readdata[1];

				SendData.TagList[2+i].OID_Command = (DeviceConfig.CollectPeriod<<11)+CollectTime  +((0xC0 + RDataType )<<24);     //ҵ������5������Э���޸�
				SendData.TagList[2+i].OID_Command = ntohl(SendData.TagList[2+i].OID_Command); //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
				SendData.TagList[2+i].Width =4;
				for(j=0;j<SendData.TagList[2+i].Width;j++)	
					{
						SendData.TagList[2+i].Value[j]=readdata[2+j];                             //�ɼ���������Ũ������
					}
				}
/*****************************************************************************************************/	
				RevTagNum =2+DataCollectCount;                     //TAG���������ڵ���+����+�ϱ�����
			  
		 	  break;
				
		}			
		case ONLINEREQUEST:
		{
			
			break;
		}
			
		case WAKEUPRESPONSE:
		{
	  SendData.TagList[0].OID_Command =ntohl(DEVICE_WAKEUP); //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
	  SendData.TagList[0].Width =1;
	  SendData.TagList[0].Value[0]=1;
	  RevTagNum =1;
	  		
			break;
		}
			
		case STARTUPREQUEST:
		{
			
			SendData.TagList[0].OID_Command =ntohl(DEVICE_STATE); //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
			SendData.TagList[0].Width =1;
			SendData.TagList[0].Value[0]=1;
			RevTagNum =1;

			break;
		}
			
		default:
		{
			RevTagNum =0;
		}

	}	
	}
	 
  		for(i=0; i<RevTagNum;i++)
			{
			TagLength = SendData.TagList[i].Width +TagPreLength;
			ValidLength += TagLength;
		  
			}
			SendData.Length =CoreLength + ValidLength;  
			SendData.Length =ntohs(SendData.Length );
			memcpy(pSendBuff,pChar,PreLength );                             //���г�TAG�����ݸ���
			
			SendData.CrcCode =0xffff;                                        //CRC�ֶθ���ֵ
			TotalLength = PreLength + ValidLength;
			
			Offset=	PreLength;		
			for(i=0; i<RevTagNum;i++)
			{
			
		  pChar = (char*)&(SendData.TagList [i].OID_Command); 
      TagLength = SendData.TagList[i].Width +TagPreLength;				//����TAG�ĸ��� 
			SendData.TagList[i].Width=ntohs(SendData.TagList[i].Width);
     	memcpy((pSendBuff+Offset),pChar,TagLength );
			
			Offset+=TagLength;
			
			}
			
			CrcData = CRC16(pSendBuff, TotalLength);   // Update the CRC value
	    SendData.CrcCode =CrcData;
	    pSendBuff[TotalLength+1] = CrcData&0xff;   //CRC���ֽ���ǰ
	    pSendBuff[TotalLength+2] = CrcData>>8;     //CRC���ֽ��ں�
			
		
	if(DeviceConfig.RetryNum>0)
	{
     SendCounter =DeviceConfig.RetryNum;
  }
	else
	{
     SendCounter =1;
  }
	for(i=0;i<SendCounter;i++)
	{
     RecevFlag =SendMessage(pSendBuff, TotalLength+2, pDeviceID, NodeAddress);
		 if(RecevFlag != 0)     //�ɹ�����
		 {

				printf("\r\n���ݷ��ͳɹ�!\r\n"); //����ʹ��

			  break;     
     }
		  rand_num=Get_Rand_Num();          //��ȡ����ӳ�ʱ��
#if DEBUG_TEST 
		 	printf("\r\nthe rand_num is %d\r\n",rand_num);  
#endif
		   Delay_ms(5000+2*rand_num);                           //�ӳ��������
	}
	
	
}


/*******************************************************************************
* Function Name  : DeviceStartupRequest
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

void DeviceStartupRequest(u8* pSendBuff, u8* pDeviceID, u16 NodeAddress)
{
	DeviceType DevType = BIRMM_RTU100;		
//	struct   DataFrame  StartupSend;
	struct   DataFrame* pDataFrame =&StartupSend;
	char*    pChar =NULL;
	unsigned int rand_num;
	u16      RecevFlag =0;       //���������ݽ��ձ�־����
	u16      CrcData=0;
	u8       i=0,j=0;
	u8       SendCounter =1;     //���ʹ���������
	u8       ValidLength =0;     //��Ч���ݳ��ȣ�ָ1��Tag�ռ���ʵ�ʴ洢��Ч���ݵĳ��ȡ�
	const u8 PreLength   =16;    //֡ǰ׺���ȣ�ָ��֡ǰ���뵽OID���У�����Tag���У������ݳ��ȣ�����֡ǰ���룬��������OID���У�����Tag���У�
	const u8 CoreLength  =12;    //�ؼ���Ϣ���ȣ�ָ���ݾ��ɲ��ֳ����ֶ�ָʾ����ֵ��ȥ��OID���У�����Tag���У���ʣ�����ݵĳ���
	
	pChar =(char*)pDataFrame;
	memset(pChar,0x00,sizeof(struct DataFrame));   //��ʼ���ṹ��
	StartupSend.Preamble =0xA3;
	StartupSend.Version  =0x20;
	for(i=0;i<6;i++)
	{
    StartupSend.DeviceID[i] =pDeviceID[i];
  }
	StartupSend.RouteFlag =0x01;
	StartupSend.NodeAddr  =ntohs( NodeAddress);              //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
	StartupSend.PDU_Type  =(13<<8)+(1<<7)+DevType;
	StartupSend.PDU_Type  =ntohs(StartupSend.PDU_Type);      //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
	StartupSend.Seq       =1;
	StartupSend.TagList[0].OID_Command =ntohl(DEVICE_STATE); //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
	StartupSend.TagList[0].Width =1;
	StartupSend.TagList[0].Value[0]=1;
	StartupSend.Tag_Count =1;
	StartupSend.Length =CoreLength + StartupSend.TagList[0].Width +6;  
	StartupSend.Length =ntohs(StartupSend.Length );
	StartupSend.CrcCode=0xffff;                                        //CRC�ֶθ���ֵ

	memcpy(pSendBuff,pChar,PreLength);
	for(i=0,j=PreLength;i<StartupSend.Tag_Count;i++)
	{
    
		pChar = (char*)&(StartupSend.TagList[i].OID_Command);   
		ValidLength =StartupSend.TagList[i].Width+6;           //����1��Tagʵ��ռ�õ��ֽڿռ�
		if((j+ValidLength) >=(SENDBUFF_SIZE-3))                //��ָֹ�����
		{
       break;
    }
		StartupSend.TagList[i].Width =ntohs(StartupSend.TagList[i].Width); //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
	  memcpy((pSendBuff+j),pChar,ValidLength); 
		j = j+ValidLength;
		
  }
	pSendBuff[j++] = StartupSend.CrcCode &0xff;
	pSendBuff[j++] = StartupSend.CrcCode>>8;
	
  CrcData = CRC16(pSendBuff, j );   // Update the CRC value
	StartupSend.CrcCode =CrcData;
	pSendBuff[j-2] = CrcData&0xff;   //CRC���ֽ���ǰ
	pSendBuff[j-1] = CrcData>>8;     //CRC���ֽ��ں�

	if(DeviceConfig.RetryNum>0)
	{
     SendCounter =DeviceConfig.RetryNum;
  }
	else
	{
     SendCounter =1;
  }
	for(i=0;i<SendCounter;i++)
	{
     RecevFlag =SendMessage(pSendBuff, j, pDeviceID, NodeAddress);
		 if(RecevFlag !=0x00)     //�ɹ����յ�TrapResponse
		 {
       
				printf("\r\n�������ݳɹ�!\r\n"); //����ʹ��
				
			  break;     
     }
		   rand_num=Get_Rand_Num();
		 //printf("the rand_num is %d",rand_num);
		   Delay_ms(2000+2*rand_num);                        //�������
		 //Delay_ms(2000+2*Get_Rand_Num());                                 //��ȡ�������); 
  }

}

/*******************************************************************************
* Function Name  : TrapRequest
* Description    : �����ϴ�����
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void  TrapRequest(u8* pSendBuff, u8* pDeviceID, u16 NodeAddress, struct SenserData* pObtainData)
{
		
//	struct   SpecialDataFrame  TrapSend;
	struct   SpecialDataFrame* pDataFrame  =&TrapSend;
	char*    pChar =NULL;
	u16      RecevFlag =0;       //���������ݽ��ձ�־����
	uint8_t  readdata[7];        //��ȡFLASH������
	unsigned int rand_num;       //0~1000֮����������
	u16      CrcData=0;
	u8       SendCounter =1;     //���ʹ���������
	u8       i=0,j=0;
	u8       Offset=0;           //���ͻ����ַƫ�Ʊ���
	u8       ValidLength =0;     //��Ч���ݳ��ȣ�ָ1��Tag�ռ���ʵ�ʴ洢��Ч���ݵĳ��ȡ�
	u8       LengthKey =0;       //���ݾ��ɲ��ֳ����ֶ�ָʾ����ֵ
	const u8 PreLength   =16;    //֡ǰ׺���ȣ�ָ��֡ǰ���뵽OID���У�����Tag���У������ݳ��ȣ�����֡ǰ���룬��������OID���У�����Tag���У�
	const u8 CoreLength  =12;    //�ؼ���Ϣ���ȣ�ָ���ݾ��ɲ��ֳ����ֶ�ָʾ����ֵ��ȥ��Tag���к�ʣ�����ݵĳ���
	
  printf("\r\nFUNCTION: TrapRequest start! \r\n");                    //

	pChar =(char*)pDataFrame;
	memset(pChar,0x00,sizeof(struct SpecialDataFrame));   //��ʼ���ṹ��
	TrapSend.Preamble =0xA3;
	TrapSend.Version  =0x20;
	for(i=0;i<6;i++)
	{
    TrapSend.DeviceID[i] =pDeviceID[i];
  }
	TrapSend.RouteFlag =0x01;
	TrapSend.NodeAddr  =ntohs( NodeAddress);              //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
	TrapSend.PDU_Type  =(4<<8)+(1<<7)+4;
	TrapSend.PDU_Type  =ntohs(TrapSend.PDU_Type);         //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
	TrapSend.Seq       =1;
	LengthKey = CoreLength;
	
	TrapSend.BattEnergy.OID_Command = ntohl(DEVICE_QTY);   //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
	TrapSend.BattEnergy.Width =1;                         //
	TrapSend.BattEnergy.Value[0] =DeviceConfig.BatteryCapacity;  //�����ϴ�ʱ�ĵ��ʣ�����
	LengthKey = LengthKey+6+TrapSend.BattEnergy.Width;
	
	TrapSend.SysTime.OID_Command= ntohl(SYSTERM_DATA);    //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
	TrapSend.SysTime.Width =3;                            //
	TrapSend.SysTime.Value[0] =systmtime.tm_year-2000;       //ϵͳ���ڣ���
	TrapSend.SysTime.Value[1] =systmtime.tm_mon ;            //ϵͳ���ڣ���
	TrapSend.SysTime.Value[2] =systmtime.tm_mday ;           //ϵͳ���ڣ���
  LengthKey = LengthKey+6+TrapSend.SysTime.Width;

	for(i=0,TrapSend.Tag_Count=0,TrapSend.Length =LengthKey ;i< DataCollectCount ;i++)    //��ÿһ������Tag��ֵ
	{
		///////////////////////////////////////��FLASH�ڵ����ݶ�ȡ���������ϱ�////////////////////////////////
		DataRead_From_Flash(1,i+1,0, readdata ,sizeof(readdata));
		pObtainData[i].CollectTime = readdata[0] *256 + readdata[1];
		for(j=0;j<4;j++)
		pObtainData[i].Ch4Data .Data_Hex [j] = readdata[2+j];
		
    TrapSend.TagList[i].OID_Data = (DeviceConfig.CollectPeriod<<11)+pObtainData[i].CollectTime  +(0xC5<<24);     //ҵ������5������Э���޸�
		TrapSend.TagList[i].OID_Data = ntohl(TrapSend.TagList[i].OID_Data); //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
		TrapSend.TagList[i].Width =4;
		TrapSend.TagList[i].PerceptionData.Data_F   =pObtainData[i].Ch4Data .Data_Float  ;  //�ɼ���������Ũ������
    
		TrapSend.Tag_Count++ ;                                             //�Դ���������Tag���м������������ɼ�����Tag
		TrapSend.Length =TrapSend.Length + TrapSend.TagList[i].Width +6;  
  }

	/////////////////////////////////////////////////////////////////////////////////////////////
	TrapSend.Length =ntohs(TrapSend.Length );              //���㳤���ֶ�
	memcpy(pSendBuff,pChar,PreLength);                     //����Tag֮ǰ�����ݵ�����Buff
	Offset=PreLength;                                           //ָ��ƫ�Ƶ�ַ
	
	pChar = (char*)&(TrapSend.BattEnergy.OID_Command);     //��ص���Tag
	ValidLength =TrapSend.BattEnergy.Width+6;                    //����1��Tagʵ��ռ�õ��ֽڿռ� 
	TrapSend.BattEnergy.Width =ntohs(TrapSend.BattEnergy.Width); //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
	memcpy((pSendBuff+Offset),pChar,ValidLength);                     //���Ƶ�ص���Tag���ݵ�����Buff
	Offset = Offset+ValidLength;
	
	pChar = (char*)&(TrapSend.SysTime.OID_Command);        //ϵͳ����Tag
	ValidLength =TrapSend.SysTime.Width+6;                 //����1��Tagʵ��ռ�õ��ֽڿռ� 
	TrapSend.SysTime.Width =ntohs(TrapSend.SysTime.Width); //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
	memcpy((pSendBuff+Offset),pChar,ValidLength);       //����ϵͳʱ��Tag���ݵ�����Buff
	Offset = Offset+ValidLength;
	
	for(i=0;i<TrapSend.Tag_Count;i++)      //
	{
		pChar = (char*)&(TrapSend.TagList[i].OID_Data);   
		ValidLength =TrapSend.TagList[i].Width+6;            //����1��Tagʵ��ռ�õ��ֽڿռ�
		if((Offset+ValidLength) >=(SENDBUFF_SIZE-3))              //��ָֹ�����
		{
       break;
    }
		TrapSend.TagList[i].Width =ntohs(TrapSend.TagList[i].Width); //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
	  memcpy((pSendBuff+Offset),pChar,ValidLength);                //����ÿһ������Tag������Buff
		Offset = Offset+ValidLength;	
  }
	
	TrapSend.CrcCode=0xffff;                               //CRC�ֶθ���ֵ
	pSendBuff[Offset++] = TrapSend.CrcCode &0xff;
	pSendBuff[Offset++] = TrapSend.CrcCode>>8;
	
  CrcData = CRC16(pSendBuff, Offset );   // Update the CRC value
	TrapSend.CrcCode =CrcData;
	pSendBuff[Offset-2] = CrcData&0xff;   //CRC���ֽ���ǰ
	pSendBuff[Offset-1] = CrcData>>8;     //CRC���ֽ��ں�

	if(DeviceConfig.RetryNum>0)
	{
     SendCounter =DeviceConfig.RetryNum;
  }
	else
	{
     SendCounter =1;
  }
	for(i=0; i<SendCounter; i++)
	{
     RecevFlag =SendMessage(pSendBuff, Offset, pDeviceID, NodeAddress);
//		 printf("\r\nReceive TrapResponse success:%4x!\r\n", RecevFlag); //����ʹ��
		 if(RecevFlag ==0x0584)     //�ɹ����յ�TrapResponse
		 {
        #if DEBUG_TEST	 
				printf("\r\nReceive TrapResponse success!\r\n"); //����ʹ��
				#endif
			  break;     
     }
		 rand_num=Get_Rand_Num();
		 printf("the rand_num is %d",rand_num);
		 Delay_ms(2000+2*rand_num);                        //�������
  }
//	printf("\r\n----Length:%d----\r\n",Offset);   //����ʹ��
//	for(i=0;i<Offset;i++)
//	{
//    printf("-%x-",pSendBuff[i]);   //����ʹ��
//  }
}

/*******************************************************************************
* Function Name  : AlarmTrap
* Description    : ���������ϴ�����
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void  AlarmTrap(u8* pSendBuff, u8* pDeviceID, u16 NodeAddress, struct SenserData* pObtainData)
{
		
//	struct   SpecialDataFrame  TrapSend;
	struct   SpecialDataFrame* pDataFrame  =&TrapSend;
	char*    pChar =NULL;
	u16      RecevFlag =0;       //���������ݽ��ձ�־����
	uint8_t  readdata[7];        //��ȡFLASH������
	unsigned int rand_num;       //0~1000֮����������
	u16      CrcData=0;
	u8       SendCounter =1;     //���ʹ���������
	u8       i=0,j=0;
	u8       Offset=0;           //���ͻ����ַƫ�Ʊ���
	u8       ValidLength =0;     //��Ч���ݳ��ȣ�ָ1��Tag�ռ���ʵ�ʴ洢��Ч���ݵĳ��ȡ�
	u8       LengthKey =0;       //���ݾ��ɲ��ֳ����ֶ�ָʾ����ֵ
	const u8 PreLength   =16;    //֡ǰ׺���ȣ�ָ��֡ǰ���뵽OID���У�����Tag���У������ݳ��ȣ�����֡ǰ���룬��������OID���У�����Tag���У�
	const u8 CoreLength  =12;    //�ؼ���Ϣ���ȣ�ָ���ݾ��ɲ��ֳ����ֶ�ָʾ����ֵ��ȥ��Tag���к�ʣ�����ݵĳ���
	
  printf("\r\nFUNCTION: AlarmTrap start! \r\n");                    //

	pChar =(char*)pDataFrame;
	memset(pChar,0x00,sizeof(struct SpecialDataFrame));   //��ʼ���ṹ��
	TrapSend.Preamble =0xA3;
	TrapSend.Version  =0x20;
	for(i=0;i<6;i++)
	{
    TrapSend.DeviceID[i] =pDeviceID[i];
  }
	TrapSend.RouteFlag =0x01;
	TrapSend.NodeAddr  =ntohs( NodeAddress);              //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
	TrapSend.PDU_Type  =(4<<8)+(1<<7)+4;
	TrapSend.PDU_Type  =ntohs(TrapSend.PDU_Type);         //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
	TrapSend.Seq       =1;
	LengthKey = CoreLength;
	
	TrapSend.BattEnergy.OID_Command = ntohl(DEVICE_QTY);   //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
	TrapSend.BattEnergy.Width =1;                         //
	TrapSend.BattEnergy.Value[0] =DeviceConfig.BatteryCapacity;  //�����ϴ�ʱ�ĵ��ʣ�����
	LengthKey = LengthKey+6+TrapSend.BattEnergy.Width;
	
	TrapSend.SysTime.OID_Command= ntohl(SYSTERM_DATA);    //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
	TrapSend.SysTime.Width =3;                            //
	TrapSend.SysTime.Value[0] =systmtime.tm_year-2000;       //ϵͳ���ڣ���
	TrapSend.SysTime.Value[1] =systmtime.tm_mon ;            //ϵͳ���ڣ���
	TrapSend.SysTime.Value[2] =systmtime.tm_mday ;           //ϵͳ���ڣ���
  LengthKey = LengthKey+6+TrapSend.SysTime.Width;

	for(i=0,TrapSend.Tag_Count=0,TrapSend.Length =LengthKey ;i< 1 ;i++)    //��ÿһ������Tag��ֵ
	{
		///////////////////////////////////////��FLASH�ڵ����ݶ�ȡ���������ϱ�////////////////////////////////
		DataRead_From_Flash(1,DataCollectCount,0, readdata ,sizeof(readdata));
		pObtainData[i].CollectTime = readdata[0] *256 + readdata[1];
		for(j=0;j<4;j++)
		pObtainData[i].Ch4Data .Data_Hex [j] = readdata[2+j];
		
    TrapSend.TagList[i].OID_Data = (DeviceConfig.CollectPeriod<<11)+pObtainData[i].CollectTime  +(0xC5<<24);     //ҵ������5������Э���޸�
		TrapSend.TagList[i].OID_Data = ntohl(TrapSend.TagList[i].OID_Data); //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
		TrapSend.TagList[i].Width =4;
		TrapSend.TagList[i].PerceptionData.Data_F   =pObtainData[i].Ch4Data .Data_Float  ;  //�ɼ���������Ũ������
    
		TrapSend.Tag_Count++ ;                                             //�Դ���������Tag���м������������ɼ�����Tag
		TrapSend.Length =TrapSend.Length + TrapSend.TagList[i].Width +6;  
  }

	/////////////////////////////////////////////////////////////////////////////////////////////
	TrapSend.Length =ntohs(TrapSend.Length );              //���㳤���ֶ�
	memcpy(pSendBuff,pChar,PreLength);                     //����Tag֮ǰ�����ݵ�����Buff
	Offset=PreLength;                                           //ָ��ƫ�Ƶ�ַ
	
	pChar = (char*)&(TrapSend.BattEnergy.OID_Command);     //��ص���Tag
	ValidLength =TrapSend.BattEnergy.Width+6;                    //����1��Tagʵ��ռ�õ��ֽڿռ� 
	TrapSend.BattEnergy.Width =ntohs(TrapSend.BattEnergy.Width); //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
	memcpy((pSendBuff+Offset),pChar,ValidLength);                     //���Ƶ�ص���Tag���ݵ�����Buff
	Offset = Offset+ValidLength;
	
	pChar = (char*)&(TrapSend.SysTime.OID_Command);        //ϵͳ����Tag
	ValidLength =TrapSend.SysTime.Width+6;                 //����1��Tagʵ��ռ�õ��ֽڿռ� 
	TrapSend.SysTime.Width =ntohs(TrapSend.SysTime.Width); //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
	memcpy((pSendBuff+Offset),pChar,ValidLength);       //����ϵͳʱ��Tag���ݵ�����Buff
	Offset = Offset+ValidLength;
	
	for(i=0;i<TrapSend.Tag_Count;i++)      //
	{
		pChar = (char*)&(TrapSend.TagList[i].OID_Data);   
		ValidLength =TrapSend.TagList[i].Width+6;            //����1��Tagʵ��ռ�õ��ֽڿռ�
		if((Offset+ValidLength) >=(SENDBUFF_SIZE-3))              //��ָֹ�����
		{
       break;
    }
		TrapSend.TagList[i].Width =ntohs(TrapSend.TagList[i].Width); //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
	  memcpy((pSendBuff+Offset),pChar,ValidLength);                //����ÿһ������Tag������Buff
		Offset = Offset+ValidLength;	
  }
	
	TrapSend.CrcCode=0xffff;                               //CRC�ֶθ���ֵ
	pSendBuff[Offset++] = TrapSend.CrcCode &0xff;
	pSendBuff[Offset++] = TrapSend.CrcCode>>8;
	
  CrcData = CRC16(pSendBuff, Offset );   // Update the CRC value
	TrapSend.CrcCode =CrcData;
	pSendBuff[Offset-2] = CrcData&0xff;   //CRC���ֽ���ǰ
	pSendBuff[Offset-1] = CrcData>>8;     //CRC���ֽ��ں�

	if(DeviceConfig.RetryNum>0)
	{
     SendCounter =DeviceConfig.RetryNum;
  }
	else
	{
     SendCounter =1;
  }
	for(i=0; i<SendCounter; i++)
	{
     RecevFlag =SendMessage(pSendBuff, Offset, pDeviceID, NodeAddress);
//		 printf("\r\nReceive TrapResponse success:%4x!\r\n", RecevFlag); //����ʹ��
		 if(RecevFlag ==0x0584)     //�ɹ����յ�TrapResponse
		 {
        #if DEBUG_TEST	 
				printf("\r\nReceive TrapResponse success!\r\n"); //����ʹ��
				#endif
			  break;     
     }
		 rand_num=Get_Rand_Num();
		 printf("the rand_num is %d",rand_num);
		 Delay_ms(2000+2*rand_num);                        //�������
  }
//	printf("\r\n----Length:%d----\r\n",Offset);   //����ʹ��
//	for(i=0;i<Offset;i++)
//	{
//    printf("-%x-",pSendBuff[i]);   //����ʹ��
//  }
}

/*******************************************************************************
* Function Name  : GetResponse
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void GetResponse(struct CommandFrame RequestPara, u8* pSendBuff, u8* pDeviceID, u16 NodeAddress)
{
  struct   rtc_time   SystTime;           //RTCʱ�����ýṹ��
//	struct   DataFrame  GetRespSend;
	struct   DataFrame* pDataFrame = & GetRespSend;
	u16      RecevFlag =0;       //���������ݽ��ձ�־����
	char*    pChar =NULL;

	u8       SendCounter =1;     //���ʹ���������
	u16      CrcData=0;
	u8       i=0,j=0;
	u8       k=0;
	u8       ValidLength =0;     //��Ч���ݳ��ȣ�ָ1��Tag�ռ���ʵ�ʴ洢��Ч���ݵĳ��ȡ�
	const u8 PreLength   =16;    //֡ǰ׺���ȣ�ָ��֡ǰ���뵽OID���У�����Tag���У������ݳ��ȣ�����֡ǰ���룬��������OID���У�����Tag���У�
	const u8 CoreLength  =12;    //�ؼ���Ϣ���ȣ�ָ���ݾ��ɲ��ֳ����ֶ�ָʾ����ֵ��ȥ��OID���У�����Tag���У���ʣ�����ݵĳ���
	
	pChar =(char*)pDataFrame;
	memset(pChar,0x00,sizeof(struct DataFrame));   //��ʼ���ṹ��
	GetRespSend.Preamble =0xA3;
	GetRespSend.Version  =0x20;
	for(i=0;i<6;i++)
	{
    GetRespSend.DeviceID[i] =pDeviceID[i];
  }
	GetRespSend.RouteFlag =0x01;
	GetRespSend.NodeAddr  =ntohs( NodeAddress);              //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
	GetRespSend.PDU_Type  =(2<<8)+(1<<7)+4;
	GetRespSend.PDU_Type  =ntohs(GetRespSend.PDU_Type);      //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
	GetRespSend.Seq       =1;
	GetRespSend.Length    =CoreLength ;                       //
	for(i=0,GetRespSend.Tag_Count=0; i<RequestPara.OID_Count; i++)
	{
		 switch(RequestPara.OID_List[i])
	   {
        case DEF_NR:            //�ش�����
				{
            GetRespSend.TagList[i].OID_Command =DEF_NR;
					  GetRespSend.TagList[i].Width =1;
					  GetRespSend.TagList[i].Value[0] =DeviceConfig.RetryNum;
					  GetRespSend.Tag_Count++;
					  GetRespSend.Length =GetRespSend.Length  + GetRespSend.TagList[i].Width +6; 
					  break;
        }	
				case SYSTERM_TIME:     //ϵͳʱ��
				{
      
						Time_Display(RTC_GetCounter(),&SystTime);     //��ȡϵͳ��ǰʱ��			
						DeviceConfig.Time_Sec  =SystTime.tm_sec;
						DeviceConfig.Time_Min  =SystTime.tm_min;
						DeviceConfig.Time_Hour =SystTime.tm_hour;
						DeviceConfig.Time_Mday =SystTime.tm_mday;		
						DeviceConfig.Time_Mon  =SystTime.tm_mon;
						DeviceConfig.Time_Year =SystTime.tm_year-2000; //���ϴ����ȥ��������
					
					  GetRespSend.TagList[i].OID_Command =SYSTERM_TIME;
					  GetRespSend.TagList[i].Width =6;
					  GetRespSend.TagList[i].Value[0] =DeviceConfig.Time_Year;   //ϵͳ���ڣ���
					  GetRespSend.TagList[i].Value[1] =DeviceConfig.Time_Mon;    //ϵͳ���ڣ���
					  GetRespSend.TagList[i].Value[2] =DeviceConfig.Time_Mday;   //ϵͳ���ڣ���
					  GetRespSend.TagList[i].Value[3] =DeviceConfig.Time_Hour;   //ϵͳʱ�䣬Сʱ
					  GetRespSend.TagList[i].Value[4] =DeviceConfig.Time_Min;    //ϵͳʱ�䣬��
					  GetRespSend.TagList[i].Value[5] =DeviceConfig.Time_Sec;    //ϵͳʱ�䣬��
					  GetRespSend.Tag_Count++;
					  GetRespSend.Length =GetRespSend.Length  + GetRespSend.TagList[i].Width +6; 
					  break;
        }	
        case CLT1_ITRL1:       //һʱ���ɼ����
				{
            GetRespSend.TagList[i].OID_Command =CLT1_ITRL1;
					  GetRespSend.TagList[i].Width =2;
					  GetRespSend.TagList[i].Value[0] =DeviceConfig.CollectPeriod >>8;   //���ݲɼ���������ֽ���ǰ
					  GetRespSend.TagList[i].Value[1] =DeviceConfig.CollectPeriod &0xff; //���ݲɼ���������ֽ��ں�
					  GetRespSend.Tag_Count++;
					  GetRespSend.Length =GetRespSend.Length  + GetRespSend.TagList[i].Width +6; 
					  break;
        }	
				
//				
				case UPLOAD_CYCLE:     //�����ϱ�����
				{
            GetRespSend.TagList[i].OID_Command =UPLOAD_CYCLE;
					  GetRespSend.TagList[i].Width =2;
					
					  GetRespSend.TagList[i].Value[0] =DeviceConfig.SendCount  >>8;   //���ݲɼ���������ֽ���ǰ
					  GetRespSend.TagList[i].Value[1] =DeviceConfig.SendCount  &0xff; //���ݲɼ���������ֽ��ں�
					  GetRespSend.Tag_Count++;
					  GetRespSend.Length =GetRespSend.Length  + GetRespSend.TagList[i].Width +6; 
					  break;
        }	

			

////////////////////////////////////////////////////////////////////////////////////////////////////////////				
        default:
				{
					 #if DEBUG_TEST	
           printf("\r\nWarning!!Tag OID not recognition!\r\n"); //����ʹ��
					 #endif
					 break;
        }
		}
  }
	
	GetRespSend.Length =ntohs(GetRespSend.Length );                //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
	GetRespSend.CrcCode=0xffff;                                    //CRC�ֶθ���ֵ
	
	memcpy(pSendBuff,pChar,PreLength);
	
	for(i=0,j=PreLength;i<GetRespSend.Tag_Count;i++)
	{
    GetRespSend.TagList[i].OID_Command =ntohl(GetRespSend.TagList[i].OID_Command);  //��OID���е���Ϊ���ֽ���ǰ�����ֽ��ں�  //�д�����
		pChar = (char*)&(GetRespSend.TagList[i].OID_Command);   
		ValidLength =GetRespSend.TagList[i].Width+6;           //����1��Tagʵ��ռ�õ��ֽڿռ�
		if((j+ValidLength) >=(SENDBUFF_SIZE-3))                //��ָֹ�����
		{
       break;
    }
		GetRespSend.TagList[i].Width =ntohs(GetRespSend.TagList[i].Width); //����Ϊ�����򣬼����ֽ���ǰ�����ֽ��ں�
	  memcpy((pSendBuff+j),pChar,ValidLength); 
		j = j+ValidLength;
		
  }
	pSendBuff[j++] = GetRespSend.CrcCode &0xff;
	pSendBuff[j++] = GetRespSend.CrcCode>>8;
	
  CrcData = CRC16(pSendBuff, j );   // Update the CRC value
	GetRespSend.CrcCode =CrcData;
	pSendBuff[j-2] = CrcData&0xff;   //CRC���ֽ���ǰ
	pSendBuff[j-1] = CrcData>>8;     //CRC���ֽ��ں�
  
	if(DeviceConfig.RetryNum>0)
	{
     SendCounter =DeviceConfig.RetryNum;
  }
	else
	{
     SendCounter =1;
  }
	for(i=0;i<SendCounter;i++)
	{
     RecevFlag =SendMessage(pSendBuff, j, pDeviceID, NodeAddress);
		 printf("\r\nReceive GetResponse success:%4x!\r\n", RecevFlag); //����ʹ��
		 if(RecevFlag ==0x0384)     //�ɹ����յ�GetResponse
		 {
        #if DEBUG_TEST	 
				printf("\r\nReceive GetResponse success!\r\n"); //����ʹ��
				#endif
			  break;     
     }
//		 Delay_ms(3000); 
  }
  
//	printf("\r\n----Length:%d----\r\n",j);   //����ʹ��
//	for(i=0;i<j;i++)
//	{
//    printf(",0x%x",pSendBuff[i]);   //����ʹ��
//  }

}
/*******************************************************************************
* Function Name  : ��ԺЭ�����ݷ���
* Description    : ���ڽ��շ��������ã�
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Treaty_Data_Analysis(u8* pTreatyBuff, u16* pFlag, u8* pDeviceID, u16 NodeAddress)	
{
	u8   DataLen =0;          //Tag���л���OID���еĳ���
	u16  PduType =0;
  
	u8   i=0,j=0,k=0;
	u8*  pChar =NULL;
	u32* pOid  =NULL;
	struct CommandFrame ParaRequest;
	struct TagStruct RecTagList[MAX];           //���յ���TAG
	uint8_t RecTagNum;                          //���յ�TAG������
	struct TagStruct SendTagList[MAX];           //���͵�TAG
	uint8_t SendTagNum;                          //����TAG������
	u32       TimCount_Current =0;
  u8       readdata[7];
	u16      CollectTime;
	u8       NetWorkID=0;
  PduType =pTreatyBuff[13]*256 +pTreatyBuff[14];   //�����������PDU����
/*************************�ж�������Դ******************************************/
  if(pTreatyBuff[10]==2) 
	{MessageSendSeq=1; }                 //�������Զ���

	switch(PduType)
	{
/******************************************����������********************************************/	     
        case ((GETREQUEST<<8)+(1<<7)+DEVICE):
				{
						printf("\r\n��������ѯ���ݵ�GETREQUEST����...\r\n");    //��⵽���������͵���������

						*pFlag =PduType;
						DataLen =pTreatyBuff[2]*256 +pTreatyBuff[3]-12; //���յ�OID���ܳ���
					  //ParaRequest.OID_Count =DataLen/4;               //���յ�OID������
					  if(DataLen >6)   //�޶�OID��������������20��
						{
							  pOid = (u32*)(pTreatyBuff+16);    //�������·��ĵ�һ��OID
							  i=0;
							  while( DataLen >6 )
								{                         //����TAG������
								  pChar = (u8*)pOid;
                   switch(ntohl(*pOid))
									 {
/**********************************************************************************************************/											
											case SYSTERM_TIME:     //ϵͳʱ��
											{
																										
													RecTagList[i].OID_Command=ntohl(SYSTERM_TIME);
												  RecTagList[i].Width=6;                                  // //����TAG��� WIDTH
													TimCount_Current = RTC_GetCounter();         //��ȡ��ǰʱ��
													Time_Display(TimCount_Current,&systmtime); 
													RecTagList[i].Value[5] =systmtime.tm_sec;
													RecTagList[i].Value[4] =systmtime.tm_min;
													RecTagList[i].Value[3] =systmtime.tm_hour;
													RecTagList[i].Value[2] =systmtime.tm_mday;		
													RecTagList[i].Value[1] =systmtime.tm_mon;
													RecTagList[i].Value[0] =systmtime.tm_year-2000; //���ϴ����ȥ��������	
													pOid =(u32*)(pChar+12);                     //ָ�����1��Tag
												  DataLen = DataLen-12; 
												  i++;
													break;			
                          																			
											}	
											
/**********************************************************************************************************/
											case CLT1_ITRL1:       //һʱ���ɼ����
											{ 
												
													RecTagList[i].OID_Command=ntohl(CLT1_ITRL1);
												  RecTagList[i].Width=2;                                  // //����TAG��� WIDTH
													for(k=0;k<RecTagList[i].Width;k++)
													RecTagList[i].Value[k]=ConfigData.CollectPeriod_Byte[k];                      // ����TAG��� VALUE

													pOid =(u32*)(pChar+8);                     //ָ�����1��Tag
												  DataLen = DataLen-8; 
													i++;
													break;
											}	
/**********************************************************************************************************/		
											case UPLOAD_CYCLE:     //�����ϱ�����
											{
												
													RecTagList[i].OID_Command=ntohl(UPLOAD_CYCLE);
													RecTagList[i].Width=2;                                  // //����TAG��� WIDTH
													for(k=0;k<RecTagList[i].Width;k++)
													RecTagList[i].Value[k]=ConfigData.SendCount_Byte[k];                      // ����TAG��� VALUE
												
													pOid =(u32*)(pChar+8);                     //ָ�����1��Tag
												  DataLen = DataLen-8; 
													i++;
													break;
											}	
/**********************************************************************************************************/									
											case DEF_NR:            //�ش�����
											{
													
													RecTagList[i].OID_Command=ntohl(DEF_NR);
													RecTagList[i].Width=1;                                  // //����TAG��� WIDTH
													for(k=0;k<RecTagList[i].Width;k++)
													RecTagList[i].Value[k]=ConfigData.RetryNum;                      // ����TAG��� VALUE
																									
													pOid =(u32*)(pChar+7);                     //ָ�����1��Tag
												  DataLen = DataLen-7; 
													i++;
													break;
											}	
/**********************************************************************************************************/									
											case DATA_REQUEST:            //��ѯ����
											{
													DataRead_From_Flash(1,DataCollectCount,0, readdata ,sizeof(readdata));
													CollectTime = readdata[0] *256 + readdata[1];
													RecTagList[i].OID_Command = ntohl((DeviceConfig.CollectPeriod<<11)+CollectTime  +(0xC5<<24));     //ҵ������5������Э���޸�
												  RecTagList[i].Width=4;                                  // //����TAG��� WIDTH
													for(k=0;k<RecTagList[i].Width;k++)
													RecTagList[i].Value[k]=readdata[2+k];                      // ����TAG��� VALUE											
													pOid =(u32*)(pChar+7);                     //ָ�����1��Tag
												  DataLen = DataLen-7; 
													i++;
													break;
											}	
											
                     case DATA_CONCENTOR_NETID:     //433��������
											{
													//NetWorkID  =pChar[6];
													//SetNodeNetworkID(NetWorkID);
/**********************************************************************************************************/													
                          RecTagList[i].OID_Command=ntohl(DATA_CONCENTOR_NETID);          //����TAG��� OID
													RecTagList[i].Width=1;                                  // //����TAG��� WIDTH
													for(k=0;k<RecTagList[i].Width;k++)
													RecTagList[i].Value[k]=GetNetworkID();                      // ����TAG��� VALUE
/**********************************************************************************************************/													
												  pOid =(u32*)(pChar+7);                           //ָ�����1��Tag
												  DataLen = DataLen-7;
											  	i++;        //�Ϸ�OID������
													break;
											}												
										
											default:
											{
												 #if DEBUG_TEST	
												 printf("\r\nWarning!!Tag OID not recognition!\r\n"); //����ʹ��
												 #endif
												 pOid =(u32*)(pChar+1);  //ָ�����һ���ֽڣ���ѯ�������޺Ϸ�OID
												 DataLen = DataLen-1;    //ָ�����һ���ֽڣ���ѯ�������޺Ϸ�OID
												 break;
											}
/**********************************************************************************************************/									 
								 }
									
								}

							//printf("\r\n--i:%d--j:%d--%4x----.\r\n",i,j, ParaRequest.OID_List[i]);    //��ʾOID
			
						RecTagNum=i;		
            }
					  else
						{
#if DEBUG_TEST	 
							  printf("\r\nReceive Command OID not correct.\r\n");    //????
#endif
            }

						DMA_UART3_RECEV_FLAG =0;     //����ձ�־����
						SendDataToServ(GETRESPONSE,RecTagList,RecTagNum,Usart3_send_buff,pDeviceID);    //������Ӧ
					  //GetResponse( ParaRequest, Usart3_send_buff, pDeviceID,  NodeAddress);
					  break;
        }	
/******************************************�����豸��������********************************************/	
        case ((SETREQUEST<<8)+(1<<7)+DEVICE):                  //���շ������·�������
				{
           	 
						printf("\r\n�������·���������SETREQUEST...\r\n");    //����ʹ��
						
						*pFlag =PduType;
					  DataLen =pTreatyBuff[2]*256 +pTreatyBuff[3]-12;    //���յ���Tag���е��ܳ���
					  if(DataLen >6)           //���ٴ���һ���Ϸ������ò���
						{
                pOid = (u32*)(pTreatyBuff+16);    //�������·��ĵ�һ��OID
							  i=0;
							  while( DataLen >6 )
								{
									 //printf("\r\n--Cycle--%4x----.\r\n",ntohl(*pOid));         //����ʹ��
									 pChar = (u8*)pOid;
                   switch(ntohl(*pOid))
									 {
											
											case SYSTERM_TIME:     //ϵͳʱ��
											{
													DeviceConfig.Time_Year =*(pChar+6);
												  DeviceConfig.Time_Mon  =*(pChar+7);
												  DeviceConfig.Time_Mday =*(pChar+8);
												  DeviceConfig.Time_Hour =*(pChar+9);
												  DeviceConfig.Time_Min  =*(pChar+10);
												  DeviceConfig.Time_Sec  =*(pChar+11);
												  printf("\r\n���յ�OIDΪ��%4x\r\n",SYSTERM_TIME);
												  printf("\r\n-��-��-��-ʱ-��-�룺-%d--%d--%d--%d--%d--%d--.\r\n",DeviceConfig.Time_Year,DeviceConfig.Time_Mon,DeviceConfig.Time_Mday,
												                 DeviceConfig.Time_Hour,DeviceConfig.Time_Min, DeviceConfig.Time_Sec  );         //����ʹ��
												  if((DeviceConfig.Time_Mon<=12)&&(DeviceConfig.Time_Mday<=31)&&(DeviceConfig.Time_Hour<=23)&&(DeviceConfig.Time_Min<=60)&&(DeviceConfig.Time_Sec<=60)) //�����Ϸ����ж�
													{
                            Time_Auto_Regulate(&DeviceConfig);             //ͨ���������·���������RTCʱ��У׼��
                          }
												  ParaRequest.OID_List[i] =SYSTERM_TIME; 
/**********************************************************************************************************/														
                          RecTagList[i].OID_Command=ntohl(SYSTERM_TIME);          //����TAG��� OID
													RecTagList[i].Width=6;                                  // //����TAG��� WIDTH
													for(k=0;k<6;k++)
													RecTagList[i].Value[k]=pChar[6+k];                      // ����TAG��� VALUE
/**********************************************************************************************************/	                          													
												  pOid =(u32*)(pChar+12);                          //ָ�����1��Tag
												  DataLen = DataLen-12;
												 	i++;        //�Ϸ�OID������
													break;							
											}	
											case CLT1_ITRL1:       //һʱ���ɼ����
											{
													DeviceConfig.CollectPeriod =pChar[6]*256+pChar[7];
												   
												  ConfigData.CollectPeriod_Byte[0]= pChar[6];
												  ConfigData.CollectPeriod_Byte[1]= pChar[7];                           											 
												  printf("\r\n���յ�OIDΪ��%4x\r\n",CLT1_ITRL1);
												  printf("\r\n---�ɼ�����:-%d---.\r\n", DeviceConfig.CollectPeriod );         //����ʹ��
												  if(0<DeviceConfig.CollectPeriod<=60)              //�����Ϸ����ж�
													{
														 Delay_ms(100); 
												  UploadFlash((char *)ConfigData.CollectPeriod_Byte,2);  //��������Flash
														 Delay_ms(100); 
                          }
												  ParaRequest.OID_List[i] =CLT1_ITRL1;   
/**********************************************************************************************************/													
                          RecTagList[i].OID_Command=ntohl(CLT1_ITRL1);          //����TAG��� OID
													RecTagList[i].Width=2;                                  // //����TAG��� WIDTH
													for(k=0;k<2;k++)
													RecTagList[i].Value[k]=pChar[6+k];                      // ����TAG��� VALUE
/**********************************************************************************************************/	
													
												  pOid =(u32*)(pChar+8);                         //ָ�����1��Tag
												  DataLen = DataLen-8;
												 	i++;        //�Ϸ�OID������
													break;
											}	

											
//									
											case UPLOAD_CYCLE:     //�����ϱ�����
											{
													DeviceConfig.SendCount  =pChar[6]*256+pChar[7];
														   //for(k=0;k<1;k++)
												  ConfigData.SendCount_Byte [0]= *(pChar+6);
												  ConfigData.SendCount_Byte [1]= *(pChar+7);
													printf("\r\n���յ�OIDΪ��%4x\r\n",UPLOAD_CYCLE);
											  	printf("\r\n-�ϱ�����:-%d---.\r\n", DeviceConfig.SendCount  ); //����ʹ��
												  if(0<DeviceConfig.SendCount <=1440)              //�����Ϸ����ж�
													{
														 Delay_ms(100); 
														 //DataWrite_To_Flash(0,3,0,(uint8_t*)ConfigData.SendCount_Byte ,2);      //���ɼ�����д��Flash
                             UploadFlash((char*)ConfigData.SendCount_Byte, 3);  //��������Flash
														 Delay_ms(100); 
                          }
													
												  ParaRequest.OID_List[i] =UPLOAD_CYCLE;   
/**********************************************************************************************************/													
                          RecTagList[i].OID_Command=ntohl(UPLOAD_CYCLE);          //����TAG��� OID
													RecTagList[i].Width=2;                                  // //����TAG��� WIDTH
													for(k=0;k<2;k++)
													RecTagList[i].Value[k]=pChar[6+k];                      // ����TAG��� VALUE
/**********************************************************************************************************/													
												  pOid =(u32*)(pChar+8);                           //ָ�����1��Tag
												  DataLen = DataLen-8;
											  	i++;        //�Ϸ�OID������
													break;
											}	
									

											case DEF_NR:            //�ش�����
											{
													DeviceConfig.RetryNum =*(pChar+6);
													printf("\r\n���յ�OIDΪ��%4x\r\n",DEF_NR);
												  printf("\r\n-Retry Num-%x---.\r\n", DeviceConfig.RetryNum);   //����ʹ��
												  if(1<= DeviceConfig.RetryNum <10)                         //�����Ϸ����ж�
													{
														 Delay_ms(100); 
                             UploadFlash((char*)&(DeviceConfig.RetryNum), 10);  //��������Flash
														 Delay_ms(100); 
                          }
												
												  ParaRequest.OID_List[i] =DEF_NR;    
/**********************************************************************************************************/													
                          RecTagList[i].OID_Command=ntohl(DEF_NR);          //����TAG��� OID
													RecTagList[i].Width=1;                                  // //����TAG��� WIDTH
													for(k=0;k<1;k++)
													RecTagList[i].Value[k]=pChar[6+k];                      // ����TAG��� VALUE
/**********************************************************************************************************/														
												  pOid =(u32*)(pChar+7);                     //ָ�����1��Tag
												  DataLen = DataLen-7; 
 	                        i++;               //�Ϸ�OID������												
													break;
											}	
											
											
											case DATA_CONCENTOR_NETID:     //433��������
											{
													NetWorkID  =pChar[6];
													SetNodeNetworkID(NetWorkID);
												  printf("\r\n���յ�OIDΪ��%4x\r\n",DATA_CONCENTOR_NETID);
												  printf("\r\n-NetWork ID-%x---.\r\n", NetWorkID);   //����ʹ��
/**********************************************************************************************************/													
                          RecTagList[i].OID_Command=ntohl(DATA_CONCENTOR_NETID);          //����TAG��� OID
													RecTagList[i].Width=1;                                  // //����TAG��� WIDTH
												  //NetWorkID=GetNetworkID();                                //��ȡ�����
												  if(NetWorkID!=GetNetworkID())                                 //�������ò��ɹ��Ļ�
													NetWorkID=0;	
													for(k=0;k<RecTagList[i].Width;k++)
													RecTagList[i].Value[k]=NetWorkID;                      // ����TAG��� VALUE
/**********************************************************************************************************/													
												  pOid =(u32*)(pChar+7);                           //ָ�����1��Tag
												  DataLen = DataLen-7;
											  	i++;        //�Ϸ�OID������
													break;
											}	
									
											default:
											{
												 #if DEBUG_TEST	
												 printf("\r\nWarning!!Tag OID not recognition!\r\n"); //����ʹ��
												 #endif
												 pOid =(u32*)(pChar+1);  //ָ�����һ���ֽڣ���ѯ�������޺Ϸ�OID
												 DataLen = DataLen-1;    //ָ�����һ���ֽڣ���ѯ�������޺Ϸ�OID
												 break;
											}
									 }
                }
						}
						if((i>0)&&(i<=20))
						{
               ParaRequest.OID_Count =i;           //��OID���н��м���	
							 RecTagNum=i;                        //TAG������
            }
            else
						{
               ParaRequest.OID_Count =7;           //��OID������ֵ����20ʱ,��������ֵ���ó�Ĭ��ֵ��Ĭ��7��
            }	
		
            DMA_UART3_RECEV_FLAG =0;     //���ձ�־������λ	
						printf("\r\n������Ϣ���³ɹ�!\r\n");                    //����ʹ��
						printf("\r\n���з������·����õĻ�ӦGETRESPONS\r\n");
            SendDataToServ(GETRESPONSE,RecTagList,RecTagNum,Usart3_send_buff,pDeviceID);    //������Ӧ						
					  //GetResponse( ParaRequest, Usart3_send_buff, pDeviceID,  NodeAddress);        //���ô���3����ͨ��
					  break;
        }	
				
/******************************************�豸�����ϱ���Ϣ��Ӧ*********************************************/	
				case ((TRAPRESPONSE<<8)+(1<<7)+DEVICE):                  //�յ��ϱ����ݵ���Ӧ
				{
 
						printf("\r\n���շ����������ϱ��ظ�.\r\n");    //����ʹ��

					  /////////////////////////
					  //������������������֤������һ���ϴ�һ֡���ݵ������ֻ��ͨ��PDUType������֤�����յ�Ӧ��֡�������������
						*pFlag =PduType;
					  break;
        }	
/******************************************�豸���������Ӧ*********************************************/	
				case ((ONLINERESPONSE<<8)+(1<<7)+DEVICE):                      //�豸���������Ӧ
				{
 
						printf("\r\nReceive Online Response Command from Server.\r\n");    //����ʹ��

					
						*pFlag =PduType;
					  break;
        }	
/******************************************�豸�����ϱ���Ϣ��Ӧ*********************************************/				
				case ((STARTUPRESPONSE<<8)+(1<<7)+DEVICE):                       //�յ��豸�����ϱ���Ϣ��Ӧ
				{
            #if DEBUG_TEST	 
						printf("\r\n���շ�����������Ӧ�ظ�.\r\n");    //����ʹ��
						#endif
						*pFlag =PduType;
					  break;
        }	
/******************************************�����������豸����*********************************************/				
				case ((WAKEUPREQUEST<<8)+(1<<7)+DEVICE):
				{
            #if DEBUG_TEST	 
						printf("\r\n���շ�������������.\r\n");    //����ʹ��
						#endif
					
						*pFlag =PduType;
					
						if(DMA_UART3_RECEV_FLAG==1)                      //��ѯ���ݽ������
						{
							DMA_UART3_RecevDetect(pDeviceID, NodeAddress);     //��ѯ�Ƿ������ݴ���
						}
						printf("\r\n���ͷ�����������ӦWAKEUPRESPONSE\r\n");
					  SendDataToServ(WAKEUPRESPONSE,NULL,0,Usart3_send_buff, pDeviceID);
					  break;
					
						
					  //break;
        }	

        default:
				{
					 #if DEBUG_TEST	
           printf("\r\nWarning!!PDU Type not recognition!\r\n"); //����ʹ��
					 #endif
					 break;
        }
   }      
}

