
// File Name: gprs.c
#include "string.h"
#include "gprs.h"
#include "bsp_SysTick.h"
#include "bsp_usart.h"
#include "AiderProtocol.h"
#include "bsp_rtc.h"
#include "433_Wiminet.h"
#include "common.h"
#include "math.h"
#include "SPI_Flash.h"
#include "DS2780.h"
#include "API-Platform.h"
#include "test.h"
#include "SensorInit.h"


/***********************************���������붨��***************************************************/ 
struct    DataFrame ClientSetRequest;
char      Usart3_recev_buff[300]={'\0'};     //USART3���ջ���
uint16_t  Usart3_recev_count=0;              //USART3���ռ�����



extern unsigned char Usart3_send_buff[300];
extern struct    SMS_Config_RegPara   ConfigData;     //������·����ò�����HEX��ʽ��������ϵͳ��������ģʽ֮ǰд��FLASH

extern struct    Sensor_Set  DeviceConfig;   //Һλ��������Ϣ�ṹ��
extern uint8_t   DataCollectCount;           //���ݲɼ�������
extern uint8_t   LiquidDataSend_Flag;
extern unsigned char   Usart2_send_buff[SENDBUFF_SIZE];    //433����������
extern uint8_t   DMA_USART3_RecevBuff[RECEIVEBUFF_SIZE];
extern uint8_t   DMA_UART3_RECEV_FLAG ;      //USART3 DMA���ձ�־����

extern vu8       Uart4_rev_comflag;          //RS485���ڽ�����ɱ�־����
extern  u8       Uart4_rev_buff[100];
extern  vu8      Uart4_rev_count;            //RS485���ڽ��ռ�����

/***********************************��������***************************************************/ 
extern unsigned char  DMA_UART3_RecevDetect(unsigned char RecevFlag,u8* pDeviceID, u16 sNodeAddress);     //USART3�������ݼ�������ݽ���

/*******************************************************************************
* Function Name  : USART_DataBlock_Send
* Description    : �����򴮿ڷ�������
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART_DataBlock_Send(USART_TypeDef* USART_PORT,char* SendUartBuf,u16 SendLength)    //�����򴮿ڷ�������
{
    u16 i;
        
    for(i=0;i<SendLength;i++)
    {
        USART_SendData(USART_PORT, *(SendUartBuf+i));
        while (USART_GetFlagStatus(USART_PORT, USART_FLAG_TC) == RESET);
    } 
}

void mput_mix(char *str,int length)
{
	int i;                                       //����ʹ��
#if DEBUG_TEST
	printf("length:%d\r\n",length);             //����ʹ��
#endif

	GPIO_ResetBits(GPIOC,GPIO_Pin_5);                  //433ģ��SET�ܽ����ͣ��л������ٷ���ģʽ
	Delay_ms(500);
  GPIO_ResetBits(GPIOB,GPIO_Pin_0);                  //433ģ��EN�ܽ����ͣ��л������ٷ���ģʽ
	Delay_ms(500);
	//USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
	USART_DataBlock_Send(USART2,str,length);
//	USART_DataBlock_Send(USART2,"\r\n",2);
	//USART_DataBlock_Send(USART1,str,length);
	//USART_DataBlock_Send(USART1,"\r\n",2);
	
	Delay_ms(800);                                     
	//GPIO_SetBits(GPIOB,GPIO_Pin_0);                    //433ģ��SET�ܽ����ߣ��л�������ģʽ
	//USART_ClearFlag(USART2,USART_FLAG_TC);
	//USART_ITConfig(USART2, USART_IT_IDLE, ENABLE); 
}

void mput_mix_sx1278(char *str,int length)           //����433ģ�鷢�����ݣ����ݷ�������Ժ��������SET���ţ�����ģ�龭�����ղ�������
{
	//printf("length:%d\r\n",length);                    //����ʹ��
	
  GPIO_ResetBits(GPIOB,GPIO_Pin_0);                  //433ģ��EN�ܽ����ͣ��л������ٷ���ģʽ
	Delay_ms(500);
	GPIO_ResetBits(GPIOC,GPIO_Pin_5);                  //433ģ��SET�ܽ����ͣ��л������ٷ���ģʽ
	Delay_ms(500);
	USART_DataBlock_Send(USART2,str,length);
//	USART_DataBlock_Send(USART2,"\r\n",2);
//	USART_DataBlock_Send(USART1,str,length);
//	USART_DataBlock_Send(USART1,"\r\n",2);
	Delay_ms(2000);
  GPIO_ResetBits(GPIOC,GPIO_Pin_5);                  //433ģ��SET�ܽ����ߣ��л�������ģʽ
	Delay_ms(100);
  GPIO_ResetBits(GPIOB,GPIO_Pin_0);                //433ģ��EN�ܽ����ͣ��л�������ģʽ
	Delay_ms(100);
 USART_ClearFlag(USART2,USART_FLAG_TC);
}


void mput(char* str)
{
	printf("length:%d\r\n",strlen(str));     //����ʹ��
	GPIO_ResetBits(GPIOC,GPIO_Pin_5);                  //433ģ���SET�������ͣ��л�������ģʽ
	Delay_ms(500);
  GPIO_ResetBits(GPIOB,GPIO_Pin_0);                  //433ģ���EN�������ͣ��л�������ģʽ
	Delay_ms(500);
//	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);   //��USART3��������ǰ���ȴ�USART3���տ����жϣ����ڼ�����ݽ������
	USART_DataBlock_Send(USART2,str,strlen(str));
	USART_DataBlock_Send(USART2,"\r\n",2);
	USART_DataBlock_Send(USART1,str,strlen(str));
	USART_DataBlock_Send(USART1,"\r\n",2);
	GPIO_SetBits(GPIOB,GPIO_Pin_0);                    //433ģ���EN�������ߣ� �л�������ģʽ
	GPIO_SetBits(GPIOC,GPIO_Pin_5);                    //433ģ���SET�������ߣ��л�������ģʽ
}

/*******************************************************************************
* Function Name  : char* Find_String(char* Source, char* Object)
* Description    : ��Ŀ���ַ����з���һ��ָ�����ַ���
* Input          : 
* Output         : 
* Return         : ����ҵ����򷵻�Ŀ���ַ�����Դ�ַ����е��׵�ַ
*******************************************************************************/
char* Find_String(char* Source, char* Object)
{
	char*   Ptemp1 = Source;
	char*   Ptemp2 = Object;
	short   Length_Source =0;
	short   Length_Object =0;
	short   i=0,j=0;
	short   count=0;
	
	
	Length_Source = strlen(Source);
	Length_Object = strlen(Object);

	if(Length_Source < Length_Object)
	{
     return NULL;
  }
  
	else if(Length_Source == Length_Object)
	{
		 if((Length_Source==0)&&(Length_Object==0))
		 {
			  return NULL;
     }  
		 else  
		 { 
			  for(i=0;i<Length_Source;i++)
		    {
				   if(Ptemp1[i] != Ptemp2[i])
					 return NULL;
        }
				return Ptemp1;
     }	  
  }
	else 
	{
		 if(Length_Object == 0)
		 {
			  return NULL;
     }  
		 else  
		 {  count = Length_Source - Length_Object + 1;
			  for(i=0;i<count;i++)
		    {  for(j=0;j<Length_Object;j++)
					 {
               if(Ptemp1[i+j] != Ptemp2[j])
						   break;
           }
					 if(j==Length_Object)
					 return  &Ptemp1[i]; 
				 
        }
				return NULL;
     }	  
  }
}
/***********�������ܣ����ض������з���һ��ָ��������****************/
/***********����ҵ����򷵻�Ŀ��������Դ�����е��׵�ַ**************/
char* Find_SpecialString(char* Source, char* Object, short Length_Source, short Length_Object)  
{
	char*   Ptemp1 = Source;
	char*   Ptemp2 = Object;
	short   i=0,j=0;
	short   count=0;
	
	if((Length_Source < 0)||(Length_Object < 0))
	{
     return NULL;
  }
  if(Length_Source < Length_Object)
	{
     return NULL;
  }
  
	else if(Length_Source == Length_Object)
	{
		 if((Length_Source==0)&&(Length_Object==0))
		 {
			  return NULL;
     }  
		 else  
		 { 
			  for(i=0;i<Length_Source;i++)
		    {
				   if(Ptemp1[i] != Ptemp2[i])
					 return NULL;
        }
				return Ptemp1;
     }	  
  }
	else 
	{
		 if(Length_Object == 0)
		 {
			  return NULL;
     }  
		 else  
		 {  
			  count = Length_Source - Length_Object + 1;
			  for(i=0;i<count;i++)
		    {  for(j=0;j<Length_Object;j++)
					 {
               if(Ptemp1[i+j] != Ptemp2[j])
						   break;
           }
					 if(j==Length_Object)
					 {
							return  &Ptemp1[i]; 
					 }
				 
        }
				return NULL;
     }	  
  }
}


/**************************************�ַ�ת��Ϊ������******************************************/
/**************************************���ڼ��㱨����ֵ******************************************/
float  char_to_float(char* pSetPara)
{
	char CharTemp[10]={0x00};
	int i,j;
	int IntegerPart=0;
	int DecimalPart=0;
	int Counter=0;
	float number=0.0;
	for(j=0;j<strlen(pSetPara);j++)                       //���ú����Ѿ����˷��������,strlen(pSetPara)<=5
			 {
          if((pSetPara[j]>='0')&&(pSetPara[j]<='9'))
					{
             CharTemp[j] =pSetPara[j];
          }
				  else
					{
						 break;
          }
       }
			 if(strlen(CharTemp)>2)
			 {
          printf("\r\nInput ERROR!!\r\n");
          return 0;
       } 
			
			 for(i=strlen(CharTemp),j=0; i>=1; i--,j++)
			 {
				 
         IntegerPart =IntegerPart + (CharTemp[i-1]-'0')*(int)(pow(10,j));   //���㱨����ֵ��������
       
				 
       }
			 
			 Counter =strlen(CharTemp)+1; //����С����
			 memset(CharTemp,0x00,sizeof(CharTemp));
			 for(i=0,j=Counter;j<strlen(pSetPara);j++,i++)                     
			 {
          if((pSetPara[j]>='0')&&(pSetPara[j]<='9'))
					{
             CharTemp[i] =pSetPara[j];
          }
				  else
					{
						 break;
          }
       }
			 if(strlen(CharTemp)>2)
			 {
          printf("\r\nInput Alarm Threshold ERROR!!\r\n");
          return 0;
       }
       for(i=strlen(CharTemp),j=0; i>=1; i--,j++)
			 {
         if(strlen(CharTemp)==1)
				 {
           DecimalPart =(CharTemp[i-1]-'0')*10;
					 break;
         }
				 else
				 {
           DecimalPart =DecimalPart + (CharTemp[i-1]-'0')*(int)(pow(10,j));  //���㱨����ֵС������
         }
       }
	
	     number=DecimalPart*0.01 + IntegerPart;
			 return(number);
}


/**************************************�ַ�ת��Ϊ����******************************************/
uint16_t char_to_int(char* pSetPara)
{
	char CharTemp[10]={0x00};
	int i,j;
  int length;
	int IntegerPart=0;


	for(j=0;j<strlen(pSetPara);j++)                       //���ú����Ѿ����˷��������,strlen(pSetPara)<=5
			 {
          if((pSetPara[j]>='0')&&(pSetPara[j]<='9'))
					{
             CharTemp[j] =pSetPara[j];
          }
				  else
					{
						 break;
          }
       }
			 
			 if(strlen(CharTemp)>5)
			 {
          printf("\r\nInput  ERROR!!\r\n");
          return 0;
       } 
			 
			 
			 length=strlen(CharTemp);
	  
			 for(i=0;i<length;i++)
			 printf("----the data is %c -----",CharTemp[i]);
			 
       
			 
			 IntegerPart = IntegerPart + (CharTemp[length-1]-'0');
				
			 if(length>1)			 
			 for(i=1; i<strlen(CharTemp); i++)
			 {
				
				IntegerPart =IntegerPart + (CharTemp[length-1-i]-'0')*(int)(pow(10,i));  //
					
       }

	return (IntegerPart);
}


/*******************************************************************************
* Function Name  : Receive_DataAnalysis
* Description    : �������ݵĵ�һ�����
* Input          : None
* Output         : None      
* Return         : None
*******************************************************************************/
u16 Receive_Data_Analysis(u8* pDeviceID, u16 sNodeAddress)	
{
	 u16               Recev_Flag2 =0;                   //����������ȷ�Ա�־����
	 char*             pRecevBuff =NULL; 

   char   RecevFromCollector[2]={0xA3,SOFTVERSION};       //���շ��������͵����ݱ�־����
   char   RecevFromConfig   [4]={0xA3,0xA3,0xA3,0xA3};       //���ղ������������ݱ�־����
  
   
	 u8     PayloadLen =0;
   u16    CrcVerify =0x0000;
   u16    CrcRecev  =0x0000;

   //RecevFromCollector[2] = sNodeAddress>>8;             //�ڵ��ַ���ֽ�
   //RecevFromCollector[3]  = sNodeAddress&0xff;           //�ڵ��ַ���ֽ�
#if DEBUG_TEST
   printf("\r\n�������ݽ���!!\r\n");          //����ʹ��
#endif

 ////////////////////////////////////����Ƿ�Ϊ��������433ģ������//////////////////////////////////////////////////////
   pRecevBuff = Find_SpecialString(Usart3_recev_buff, RecevFromConfig, sizeof(Usart3_recev_buff), sizeof(RecevFromConfig));  //����Ƿ��յ�����������Ӧ��
	 if((pRecevBuff != NULL)&&(pRecevBuff< Usart3_recev_buff+(sizeof(Usart3_recev_buff)-1-9)))  //��ָֹ��Խ��          
   {
		  printf("\r\nReceive 433 Config Command from Srvies!\r\n"); //����ʹ��
	 	  if(pRecevBuff[4] ==0x00)
			{
         ParameterConfig_SX1278(pRecevBuff+5);
      }
			else if(pRecevBuff[4] ==0x01)
			{
         ParameterInquire_SX1278(pRecevBuff[5]);
      }
			else
			{
					#if DEBUG_TEST	 
					printf("\r\nReceive 433 Config Command not correct!\r\n"); //����ʹ��
					#endif
      }
			pRecevBuff =NULL;
			return 0;
	 }
/////////////////////////////////////����Ƿ�Ϊ��λ���·��ɼ�����/////////////////////////////////////////////////
   pRecevBuff = Find_SpecialString(Usart3_recev_buff,RecevFromCollector,sizeof(Usart3_recev_buff),sizeof(RecevFromCollector));  //��������յ���վ�ظ�
	 if((pRecevBuff != NULL)&&(pRecevBuff< Usart3_recev_buff+(sizeof(Usart3_recev_buff)-1-9)))  //��ָֹ��Խ��          
	 {	
#if DEBUG_TEST	
      printf("\r\n������λ�����ݽ���!!\r\n");          //����ʹ�� 		
#endif		 
			
			Treaty_Data_Analysis(((u8*)pRecevBuff), &Recev_Flag2, pDeviceID, sNodeAddress);  //������������
			return (Recev_Flag2);
	 
 }
#if DEBUG_TEST	
	 printf("\r\n�������ݽ������!!\r\n");                 //����ʹ��
#endif
	 return 0;
}

/*******************************************************************************
* Function Name  : XX
* Description    : 433Զ�����ò����������
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void  ParameterConfig_SX1278(char* pRecevCommand)
{
   u8   RecevCommandArry[5]={0x00};
	
	 u8   UseFlag   =0;    //���ò�����;��־������Ҳ����ָʾ���ò�����ʱЧ�ԣ�����Ϊ0ʱ������Ч��������Ϊ���ݲ���
	 u8   Opcode    =0;    //�������ֶΣ�����ָʾ����Ĳ�������
	 u8   DataLenth =0;    //��������
	 u16  TempLong  =0;
	 u8   TempShort =0;
	 u8   ConfigReply[12] ={0xAA,0xAA,0xAF,0xFF,0xAF,0xEE,0x00,0xAA,0xAA,0xAA,0xAA,0xAA}; //������0xAA�ֶ�Ϊ�����ֶΣ�����ֱ��ǣ��������ң���
     
                       //�ڵ��ַ���ֽڣ��ڵ��ַ���ֽڣ�������;�������룬���ò������ȣ����ò�����1�ֽڣ����ò�����2�ֽڣ����ܲ����ڣ�
   memcpy(RecevCommandArry, pRecevCommand, sizeof(RecevCommandArry));
   UseFlag   =RecevCommandArry[0];
	 Opcode    =RecevCommandArry[1];
	 DataLenth =RecevCommandArry[2];
   TempLong  =GetNodeID (); //��ȡ�ڵ�ID
   ConfigReply[0] =TempLong>>8;
   ConfigReply[1] =TempLong &0xFF;
	 ConfigReply[7] =UseFlag ;
   ConfigReply[8] =Opcode;
   Delay_ms(3000);
	 if(UseFlag ==0)
	 {
     switch(Opcode )
		 {
        case 0x09:                    //����433ģ�鴮�ڲ����ʲ����������޸�!!!
				{
           if(DataLenth ==2)
					 {
//						 SetNodeSerialPort(RecevCommandArry[3], RecevCommandArry[4]);     //����433ģ�鴮�ڲ����ʺ�У������
//						 PowerOFF_433();                                           //����433ģ�飬ʹ������Ч        
//						 Delay_ms(5000);
//						 PowerON_433();
//						 TempLong =GetNodeSerialPortConfig();                      //��ȡ433ģ�鴮�����ò���
//						 ConfigReply[9] =2;
//						 ConfigReply[10] =TempLong >>8;
//					   ConfigReply[11] =TempLong &0xFF;
//						 mput_mix_sx1278((char*)ConfigReply, 12);                  //��������Ӧ��֡
//						 //��������Flash���д�����
						 #if DEBUG_TEST	 
						 printf("\r\nSerial Port Parameters Config not allowable !!\r\n"); //����ʹ��
						 #endif
           }
					 else
					 {
             #if DEBUG_TEST	 
						 printf("\r\nReceive 433 Config Command not correct!\r\n"); //����ʹ��
						 #endif
           }
					 break;
        }
				case 0x0B:                                                    //����433ģ���ز�Ƶ��
				{
           if(DataLenth ==2)
					 {
						 TempLong =(RecevCommandArry[3]<<8) +RecevCommandArry[4];      //�������Ƶ�ʲ���
					
						 ConfigReply[9] =2;
						 ConfigReply[10] =TempLong>>8;
					   ConfigReply[11] =TempLong &0xFF;
						 mput_mix((char*)ConfigReply, 12);                //��������Ӧ��֡,�ȷ���Ӧ���ٸ������ݣ������޷�ͨ�ųɹ�
						 Delay_ms(2000);
						 mput_mix((char*)ConfigReply, 12);                //��������Ӧ��֡,�ȷ���Ӧ���ٸ������ݣ������޷�ͨ�ųɹ�,�෢һ��Ϊ�˽���״η������ݶ�ʧBUG
						 Delay_ms(1000);
						 SetNodeCentralFrequency(TempLong);                      //����433ģ���ز�����Ƶ��
						 TempLong =GetNodeCentralFrequency();                    //��ȡ433ģ���ز�Ƶ�ʲ���
	           #if DEBUG_TEST	 
					   printf("\r\nCentral Frequency :%d MHz\r\n",(TempLong+1));   //����ʹ��
						 #endif
						 //��������Flash���д�����
           }
					 else
					 {
             #if DEBUG_TEST	 
						 printf("\r\nReceive 433 Config Command not correct!\r\n"); //����ʹ��
						 #endif
           }
					 break;
        }
				case 0x0D:                                                    //����433ģ����Ƶ���Ӳ���
				{
           if(DataLenth ==1)
					 {
						 TempShort =RecevCommandArry[3];                           //���������Ƶ���Ӳ���
						 ConfigReply[9] =1;
						 ConfigReply[10] =TempShort ;   
						 mput_mix((char*)ConfigReply, 11);                  //��������Ӧ��֡,�ȷ���Ӧ���ٸ������ݣ������޷�ͨ�ųɹ�
						 Delay_ms(2000);	
						 mput_mix((char*)ConfigReply, 11);                  //��������Ӧ��֡,�ȷ���Ӧ���ٸ������ݣ������޷�ͨ�ųɹ�
						 Delay_ms(1000);							 
						 SetNodeFrequencyExpandFactor(TempShort);                  //����433ģ����Ƶ����
						 TempShort  =GetNodeFrequencyExpandFactor();               //��ȡ433ģ����Ƶ���Ӳ���
			
						 #if DEBUG_TEST	 
					   printf("\r\nNode Frequency Expand Factor(Index Code):%d\r\n",TempShort); //����ʹ��
						 #endif
						//��������Flash���д�����
           }
					 else
					 {
             #if DEBUG_TEST	 
						 printf("\r\nReceive 433 Config Command not correct!\r\n"); //����ʹ��
						 #endif
           }
					 break;
        }
				case 0x0F:                                                        //����433ģ����Ƶ�������
				{
           if(DataLenth ==1)
					 {
						 TempShort =RecevCommandArry[3];                              //���������Ƶ�������
						 ConfigReply[9] =1;
						 ConfigReply[10] =TempShort ;   
						 mput_mix((char*)ConfigReply, 11);                     //��������Ӧ��֡,�ȷ���Ӧ���ٸ������ݣ������޷�ͨ�ųɹ�
						 Delay_ms(2000);			
						 mput_mix((char*)ConfigReply, 11);                     //��������Ӧ��֡,�ȷ���Ӧ���ٸ������ݣ������޷�ͨ�ųɹ�
						 Delay_ms(1000);
						 SetNodeFrequencyExpandBandwidth(TempShort);                  //����433ģ����Ƶ����
						 TempShort  =GetNodeFrequencyExpandBandwidth();               //��ȡ433ģ����Ƶ�������
	           #if DEBUG_TEST	 
					   printf("\r\nNode Frequency Expand Bandwidth(Index Code):%d\r\n",TempShort); //����ʹ��
						 #endif
						 //��������Flash���д�����
           }
					 else
					 {
             #if DEBUG_TEST	 
						 printf("\r\nReceive 433 Config Command not correct!\r\n"); //����ʹ��
						 #endif
           }
					 break;
        }
				case 0x11:                                                     //����433ģ�鹤��ģʽ�����������޸�!!!
				{
           if(DataLenth ==1)
					 {
//						 TempShort =RecevCommandArry[3];                           //������ù���ģʽ����
//						 SetNodeWorkMode(TempShort);                               //����433ģ�鹤��ģʽ
//						 TempShort  =GetNodeWorkMode();                            //��ȡ433ģ�鹤��ģʽ����
//						 ConfigReply[9] =1;
//						 ConfigReply[10] =TempShort ;   
//						 mput_mix_sx1278((char*)ConfigReply, 11);                  //��������Ӧ��֡
//					 //��������Flash���д�����
						 #if DEBUG_TEST	 
						 printf("\r\nNode Work Mode Config not allowable !!\r\n"); //����ʹ��
						 #endif
           }
					 else
					 {
             #if DEBUG_TEST	 
						 printf("\r\nReceive 433 Config Command not correct!\r\n"); //����ʹ��
						 #endif
           }
					 break;
        }
				case 0x13:                                                     //����433ģ��ͻ�ID
				{
					 if(DataLenth ==2)
					 {
						 TempLong =(RecevCommandArry[3]<<8) +RecevCommandArry[4];  //������ÿͻ�ID����
						 ConfigReply[0] =TempLong>>8;                              //���¸��½ڵ�ID�ֶ�
             ConfigReply[1] =TempLong &0xFF;                           //���¸��½ڵ�ID�ֶ�
						 ConfigReply[9] =2;
						 ConfigReply[10] =TempLong>>8;
					   ConfigReply[11] =TempLong &0xFF;
						 mput_mix((char*)ConfigReply, 12);                  //��������Ӧ��֡
						 Delay_ms(2000);
						 mput_mix((char*)ConfigReply, 12);                  //��������Ӧ��֡
						 Delay_ms(1000);
	
						 SetNodeID(TempLong);                                      //����433ģ��ͻ�ID
						 TempLong =GetNodeID();                                    //��ȡ433ģ��ͻ�ID����				 
						 #if DEBUG_TEST	 
					   printf("\r\n433 Module Node ID is:%x\r\n",TempLong); //����ʹ��
						 #endif
						 	//��������Flash���д�����
           }
					 else
					 {
             #if DEBUG_TEST	 
						 printf("\r\nReceive 433 Config Command not correct!\r\n"); //����ʹ��
						 #endif
           }
					 break;
        }
				case 0x15:                                                        //����433ģ������ID
				{
           if(DataLenth ==1)
					 {
						 TempShort =RecevCommandArry[3];                              //�����������ID����
						 ConfigReply[9] =1;
						 ConfigReply[10] =TempShort ;   
						 mput_mix((char*)ConfigReply, 11);                     //��������Ӧ��֡,�ȷ���Ӧ���ٸ������ݣ������޷�ͨ�ųɹ�
						 Delay_ms(2000);	
						 mput_mix((char*)ConfigReply, 11);                     //��������Ӧ��֡,�ȷ���Ӧ���ٸ������ݣ������޷�ͨ�ųɹ�
						 Delay_ms(1000);	
						 
						 SetNodeNetworkID(TempShort);                              //����433ģ������ID
						 TempShort  = GetNetworkID();                              //��ȡ433ģ������ID����
						 #if DEBUG_TEST	 
					   printf("\r\n433 Module Network ID is:%x\r\n",TempShort);  //����ʹ��
						 #endif
						 //��������Flash���д�����
           }
					 else
					 {
             #if DEBUG_TEST	 
						 printf("\r\nReceive 433 Config Command not correct!\r\n"); //����ʹ��
						 #endif
           }
					 break;
        }	
				case 0x17:                                                     //����433ģ�鷢�书�ʵȼ�
				{
           if(DataLenth ==1)
					 {
						 TempShort =RecevCommandArry[3];                           //������÷��书�ʵȼ�����
						 ConfigReply[9] =1;
						 ConfigReply[10] =TempShort ;   
						 mput_mix((char*)ConfigReply, 11);                 //��������Ӧ��֡,�ȷ���Ӧ���ٸ������ݣ������޷�ͨ�ųɹ�
						 Delay_ms(2000);
						 mput_mix((char*)ConfigReply, 11);                 //��������Ӧ��֡,�ȷ���Ӧ���ٸ������ݣ������޷�ͨ�ųɹ�
						 Delay_ms(1000);							 
						 
						 SetNodeSendPowerGrade(TempShort);                         //����433ģ�鷢�书�ʵȼ�
						 TempShort =GetNodeSendPowerGrade();                       //��ȡ433ģ�鷢�书�ʵȼ�����
						 #if DEBUG_TEST	 
					   printf("\r\n433 Module Send Power Grade is:%x\r\n",TempShort);  //����ʹ��
						 #endif				 
						 //��������Flash���д�����
           }
					 else
					 {
             #if DEBUG_TEST	 
						 printf("\r\nReceive 433 Config Command not correct!\r\n"); //����ʹ��
						 #endif
           }
					 break;
        }	
				case 0x19:                                                     //����433ģ���������
				{
           if(DataLenth ==1)
					 {
						 TempShort =RecevCommandArry[3];                           //������ú������ڲ���
						 ConfigReply[9] =1;
						 ConfigReply[10] =TempShort ;   
						 mput_mix((char*)ConfigReply, 11);                 //��������Ӧ��֡,�ȷ���Ӧ���ٸ������ݣ������޷�ͨ�ųɹ�
						 Delay_ms(2000);	
						 mput_mix((char*)ConfigReply, 11);                 //��������Ӧ��֡,�ȷ���Ӧ���ٸ������ݣ������޷�ͨ�ųɹ�
						 Delay_ms(1000);	
						 
						 SetNodeBreathPeriod(TempShort);                           //����433ģ���������
						 TempShort =GetNodeBreathPeriod();                         //��ȡ433ģ��������ڲ���
             #if DEBUG_TEST	 
					   printf("\r\nNode Breath Period is:%x\r\n",TempShort);  //����ʹ��
						 #endif	
						 //��������Flash���д�����
           }
					 else
					 {
             #if DEBUG_TEST	 
						 printf("\r\nReceive 433 Config Command not correct!\r\n"); //����ʹ��
						 #endif
           }
					 break;
        }	
				case 0x1B:                                                     //����433ģ�����ʱ��
				{
           if(DataLenth ==1)
					 {
						 TempShort =RecevCommandArry[3];                           //������ú���ʱ�����
						 ConfigReply[9] =1;
						 ConfigReply[10] =TempShort ;   
						 mput_mix((char*)ConfigReply, 11);                 //��������Ӧ��֡,�ȷ���Ӧ���ٸ������ݣ������޷�ͨ�ųɹ�
						 Delay_ms(2000);	
						 mput_mix((char*)ConfigReply, 11);                 //��������Ӧ��֡,�ȷ���Ӧ���ٸ������ݣ������޷�ͨ�ųɹ�
						 Delay_ms(1000);	
						 
						 SetNodeBreathTime(TempShort);                             //����433ģ�����ʱ��
						 TempShort =GetNodeBreathTime();                           //��ȡ433ģ�����ʱ�����
						 #if DEBUG_TEST	 
					   printf("\r\nNode Wake Time is:%x\r\n",TempShort);         //����ʹ��
						 #endif
						 //��������Flash���д�����
           }
					 else
					 {
             #if DEBUG_TEST	 
						 printf("\r\nReceive 433 Config Command not correct!\r\n"); //����ʹ��
						 #endif
           }
					 break;
        }	
				default:
				{
					 #if DEBUG_TEST	
           printf("\r\nReceive 433 Config Command not correct!\r\n"); //����ʹ��
					 #endif
					 break;
        }
     }
   }
	 else if(UseFlag ==1)
	 {
		 //��������Flash������������ģ�����,������
	 }
	 else
	 {
     //�������� ,������
   }
}
/*******************************************************************************
* Function Name  : XX
* Description    : 433Զ�̲�ѯ�����������
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void  ParameterInquire_SX1278(u8 InquireCommand)
{

   ;//������

}

/*******************************************************************************
* Function Name  : ConfigData_Init
* Description    : ��ʼ��һЩ���ò���
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ConfigData_Init(struct Sensor_Set* Para)
{
  char     CharArry[16] ={0x00};
  uint8_t  length =0;
  uint16_t Temp=0;
	float    fTemp=0;
	int i;
#ifdef TEST_DEBUG
  printf("\r\n����������Ϣ��\r\n");  //����ʹ��
#endif
  BKP_TamperPinCmd(DISABLE);                   //

/******************************************************************************************/
//(2)�ɼ�����
	DataRead_From_Flash(0,2,0, ConfigData.CollectPeriod_Byte,2); //��Flash�ж�ȡҺλ�Ʋɼ����
  Temp =ConfigData.CollectPeriod_Byte[0]*256 + ConfigData.CollectPeriod_Byte[1];
  //printf("------------%d------------",Temp);
  if((Temp>0)&&(Temp<=1440))                    //������1~60���ӽ������ݲɼ����
	{
		Para->CollectPeriod =  Temp;
	}
	else
	{
    Para->CollectPeriod =60;           //ÿ��15���Ӳɼ�һ�����ݡ�
  }
		printf("\r\n**************�ɼ�����-->> %d ***********\r\n",Para->CollectPeriod);       //�������
	
//(3)�ϱ�����	
  DataRead_From_Flash(0,3,0, ConfigData.SendCount_Byte ,2);             //��Flash�ж�ȡ�ϴ�����
  Temp =ConfigData.SendCount_Byte[0]*256 + ConfigData.SendCount_Byte[1];
	
	//printf("------------%d------------",Temp);
  if((Temp>0)&&(Temp<=1440))  
	{
		Para->SendCount =  Temp;
	}
	else
	{    
    Para->SendCount =60;              //1Сʱ����һ������
  }	
	
  printf("\r\n**************�ϱ�����-->> %d *********** \r\n",Para->SendCount );   //�������


//(7)��Ũ�ȱ�����ֵ
	DataRead_From_Flash(0,7,0, ConfigData.LowAlarmLevel .Data_Hex ,4);   //��Flash�ж�ȡԤ�豨����ֵ

	fTemp = ConfigData.LowAlarmLevel .Data_Float ;
	
  //printf("\r\n---read low level ---%f----\r\n",fTemp );                       //��ӡFLASH��ȡ��ԭʼ����
   	
                               

		 if( (0.0<fTemp ) && ( fTemp <25.0) )
     Para->LowAlarmLevel  .Data_Float  =ConfigData.LowAlarmLevel .Data_Float  ;      

	else                                                     
	{ 
		 Para->LowAlarmLevel .Data_Float  =25.0;                        //��ȡ������Чʱ����������ֵ��Ϊ25.0����������ֵΪ0ʱ�����ᴥ�������¼�
  }
	
	printf("\r\n**************�ͱ���Ũ��-->> %0.2f ************\r\n", Para->LowAlarmLevel  .Data_Float );   //�������
	
// ��8����ʼ�ɼ�ʱ��
	DataRead_From_Flash(0,8,0,ConfigData.CollectStartTime_Byte ,2); //��Flash�ж�ȡ��һ�βɼ�ʱ��

	//Temp =  char_to_int(CharArry);
	
  Temp =ConfigData.CollectStartTime_Byte [0]*256 + ConfigData.CollectStartTime_Byte [1];
	
	//printf("\r\n---read CollectStartTime ---%d----\r\n",Temp );                       //��ӡFLASH��ȡ��ԭʼ����
  if((Temp>=0)&&(Temp<=1440))  
	{
		Para->CollectStartTime  =  Temp;
	}
	else
	{
    Para->CollectStartTime  =0;           //��һ�βɼ�ʱ��Ϊ0����
  }

	printf("\r\n**************��ʼ�ɼ�ʱ��-->> %d ***********\r\n", Para->CollectStartTime );   //�������
	
	
//(9)�Ѳɼ�����
	DataRead_From_Flash(0,9,0,    &(ConfigData.CollectNum)   ,1); //��Flash�ж�ȡ��ǰ�ɼ�����
	
//	printf("\r\n---read retry number ---%s----\r\n",(u8*)ConfigData.CollectNum_Byte );                       //��ӡFLASH��ȡ��ԭʼ����
  //Temp =ConfigData.CollectNum_Byte [0]*256 + ConfigData.CollectNum_Byte [1];
	Temp = ConfigData.CollectNum;
	//printf("\r\n---read CollectNum ---%d----\r\n",Temp );                       //��ӡFLASH��ȡ��ԭʼ����
//  if((0 <=ConfigData.CollectNum )&&(ConfigData.CollectNum <= MAX_COLLECTNUM))  
//	{
//		if(ConfigData.CollectNum <=( Para->SendCount /Para->CollectPeriod ))
//		{ Para->CollectNum  =  ConfigData.CollectNum; }
//		else
//		{ Para->CollectNum =( Para->SendCount /Para->CollectPeriod );}
//	}
//	else
//	{
//    Para->CollectNum  = ( Para->SendCount /Para->CollectPeriod ) ;           //��ǰ�ɼ��Ĵ����������� 
//  }
	
		printf("\r\n**************�Ѳɼ�����-->> %d ***********\r\n", Temp );   //�������
//(10)�ش�����
	DataRead_From_Flash(0,10,0, &(ConfigData.RetryNum ) ,1); //��Flash�ж�ȡ�ش�����
  
	//printf("\r\n---read retry number ---%d----\r\n",ConfigData.RetryNum );                       //��ӡFLASH��ȡ��ԭʼ����
	
	Temp =ConfigData.RetryNum ;
  if((Temp>0)&&(Temp<=10))  
	{
		Para->RetryNum  =  Temp;
	}
	else
	{
    Para->RetryNum  =3;           //Ĭ��Ϊ3
  }
	printf("\r\n**************�ش�����-->> %d ***********\r\n", Para->RetryNum );   //�������
	

//(11)��Ũ�ȱ�����ֵ
	DataRead_From_Flash(0,11,0, ConfigData.HighAlarmLevel .Data_Hex  ,4);    //��Flash�ж�ȡ��Ũ�ȱ���
  
	//printf("\r\n---read high alarm level ---%f----\r\n",ConfigData.HighAlarmLevel .Data_Float );                       //��ӡFLASH��ȡ��ԭʼ����
	
	fTemp =ConfigData.HighAlarmLevel .Data_Float  ;
  if((fTemp>0.0) && (fTemp < 50.0) && (fTemp > ConfigData.LowAlarmLevel .Data_Float ))  
	{
		Para->HighAlarmLevel .Data_Float   =  fTemp;
	}
	else
	{
    	Para->HighAlarmLevel .Data_Float  =  50.0;           //������Ũ����ֵĬ��Ϊ50%
  }
	
	printf("\r\n**************�߱���Ũ��-->> %0.2f ***********\r\n", 	Para->HighAlarmLevel .Data_Float );   //�������
//(12)�豸�ѹ�������
	DataRead_From_Flash(0,12,0, &(ConfigData.WorkNum ),1);    //��Flash�ж�ȡ�ѹ�������

	Temp=ConfigData.WorkNum ;
  Para->WorkNum  = Temp; 
	printf("\r\n**************��������-->> %d ***********\r\n", 	Para->WorkNum);   //�������
//(13)��ص���
	printf("\r\n**************��ص���-->> %d%% ***********\r\n", 	Para->BatteryCapacity=DS2780_Test());   //�������
//	Para->Time_Sec  =0x00;
//	Para->Time_Min  =0x00;
//	Para->Time_Hour =0x00;
//	Para->Time_Mday =0x00;
//	Para->Time_Mon  =0x00;
//	Para->Time_Year =0x00;
//  Para->BatteryCapacity =0x64;    //��ص������ݶ�Ϊ100%
	Para->MessageSetFlag  =0;       
}



/**********************************************END******************************************/










