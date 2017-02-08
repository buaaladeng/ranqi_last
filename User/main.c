/**
  ******************************************************************************
  * @file    main.c
  * @author  casic 203
  * @version V1.0
  * @date    2015-07-27
  * @brief   
  * @attention
  *
  ******************************************************************************
  */ 
#include "stm32f10x.h"
#include "misc.h"
#include "stdlib.h"
#include "time.h"
#include "stm32f10x_it.h"
#include "bsp_usart.h"
#include "bsp_TiMbase.h" 
#include "modbus.h"
#include "bsp_SysTick.h"
#include "string.h"
#include "gprs.h"
#include "bsp_rtc.h"
#include "bsp_date.h"
#include "WatchDog.h"
#include "AiderProtocol.h"
#include "SPI_Flash.h"
#include "common.h"
#include "DS2780.h"
#include "433_Wiminet.h"
#include  "test.h"
#include  "sensor-adc.h"
#include  "SensorInit.h"

#define COLLECT_NUM 10                  //������һ�ι����ɼ���������
#define SNESOR_WAIT 180                  //������Ԥ��ʱ��

/***********************************���������붨��***************************************************/ 
struct    rtc_time             systmtime;                  //RTCʱ�����ýṹ��
struct    Sensor_Set           DeviceConfig ={0x00};       //������Ϣ�ṹ��
struct    SMS_Config_RegPara   ConfigData ={0x00};       //��FLASH�����Ľṹ�壬������ϵͳ��������ģʽ֮ǰд��FLASH

uint16_t  WWDOG_Feed =0x1FFF;                  //���ڿ��Ź���λ����Ϊ��XX*1.8s = 7.6min
char      PowerOffReset =0;                    //����������־λ

u8        Send_Request_flag=0;                    //������������λ�����ܵ�����
uint8_t   LiquidDataSend_Flag =0;                 //���ݷ�����ɱ�־λ
uint8_t   SensorCollectNumber=0;                 //һ���ϱ���Ҫ�������ݲɼ�������         �ɼ�����=�ϱ�����/�ɼ�����
uint8_t   DataCollectCount ;                 //���ݲɼ�������   ÿ�γ�ʼ����Ҫ��ȡ��ֵ���ɼ������ݺ�д��FLASH

struct    SenserData   PerceptionData;             //����������   ��u16�ɼ�ʱ��  float����������    u8�ɼ�������
struct    SenserData   ReportUp[MAX_COLLECTNUM];   //�ϴ�������������

char      Usart1_recev_buff[300] ={'\0'};      //USART1���ջ���
uint16_t  Usart1_recev_count =0;               //USART1���ͼ�����

unsigned char   Usart2_send_buff[SENDBUFF_SIZE]={'\0'};          //433ģ�鷢������ 
unsigned char   Usart2_recev_buff[RECEIVEBUFF_SIZE]={'\0'};      //433ģ���������

unsigned char   Usart3_send_buff[300]={'\0'};    //3Gģ�鷢������

extern  char      Usart3_recev_buff[1000];       //3Gģ����433ģ�鹲�ý���������
extern  uint16_t  Usart3_recev_count;            //3Gģ����433ģ�����������

        uint8_t   DMA_UART3_RECEV_FLAG =0;                //USART3 DMA���ձ�־����
extern  uint8_t   DMA_USART3_RecevBuff[RECEIVEBUFF_SIZE];    //3Gģ����433ģ�鹲�ý���������
extern  struct    DMA_USART3_RecevConfig   DMA_USART3_RecevIndicator;    //DMAָʾ

extern __IO uint16_t ADC1ConvertedValue;                    //ADC�ɼ�������
u32 Sensor_PowderOn;
u32 Sensor_PowderTime;

/***********************************��������***************************************************/  
u16   DMA_UART3_RecevDetect(u8* pDeviceID, u16 sNodeAddress);     //USART3�������ݼ�������ݽ���
int   DMA_UART3_RecevDataGet(void);                               //��ȡ����3��DMA����
void  PeripheralInit( void);                                      //��ʼ����Χ�豸
void  Display(void);                                              //������Ϣ��ӡ
unsigned int Get_Rand_Num(void);                                 //��ȡ�������
void Client(u8* pDeviceID);

/***********************��ȡ�������***********************************************/
unsigned int Get_Rand_Num(void)
{    
	    unsigned int rand_num;
      srand((unsigned)RTC_GetCounter());                                     //��������
		  rand_num = rand()%1000;                                          //��ȡ0~1000�����������
	    return(rand_num);
}
/*******************************************************************************
* Function Name  : int  DMA_UART3_RecevDataGet(void)
* Description    : ����433��������ݣ�Usart3_recev_buff[],
* Input          : None
* Output         : None
* Return         : �ɼ������ݵĳ���
*******************************************************************************/
int  DMA_UART3_RecevDataGet(void)
{
   int i=0,j=0;
	 u16 DMA_RecevLength =0;
	
	 memset(Usart3_recev_buff, 0x00, sizeof(Usart3_recev_buff));
	 DMA_USART3_RecevIndicator.CurrentDataStartNum = DMA_USART3_RecevIndicator.NextDataStartNum ;
	  
	 i = RECEIVEBUFF_SIZE - DMA_GetCurrDataCounter(DMA1_Channel6);
	 if(DMA_USART3_RecevIndicator.DMA_RecevCount <i)
	 {
     DMA_RecevLength =i -DMA_USART3_RecevIndicator.DMA_RecevCount;
   }
	 else
	 {
     DMA_RecevLength = RECEIVEBUFF_SIZE -DMA_USART3_RecevIndicator.DMA_RecevCount + i;
   }
   DMA_USART3_RecevIndicator.DMA_RecevCount = i;
	
	 if((DMA_USART3_RecevIndicator.CurrentDataStartNum + DMA_RecevLength-1) < RECEIVEBUFF_SIZE)
	 {
     DMA_USART3_RecevIndicator.CurrentDataEndNum =DMA_USART3_RecevIndicator.CurrentDataStartNum +DMA_RecevLength-1;     
   }
	 else
	 {
     DMA_USART3_RecevIndicator.CurrentDataEndNum =(DMA_USART3_RecevIndicator.CurrentDataStartNum +DMA_RecevLength-1) -RECEIVEBUFF_SIZE;  
   }
#if DEBUG_TEST 	 
	 printf("\r\nDMA UART2 Recev Data Start Num:%d----End Num: %d\r\n",DMA_USART3_RecevIndicator.CurrentDataStartNum,DMA_USART3_RecevIndicator.CurrentDataEndNum);    //������ʼλ������ֹλ��
#endif
	 if(DMA_USART3_RecevIndicator.CurrentDataEndNum ==(RECEIVEBUFF_SIZE-1))
	 {
	   DMA_USART3_RecevIndicator.NextDataStartNum = 0;
   }
	 else
	 {
		 DMA_USART3_RecevIndicator.NextDataStartNum = DMA_USART3_RecevIndicator.CurrentDataEndNum + 1;
   }	
   /*************************************Data Copy*********************************************************/
   if(DMA_RecevLength !=0)
	 {
     j =DMA_USART3_RecevIndicator.CurrentDataStartNum;
		 if(DMA_USART3_RecevIndicator.CurrentDataEndNum >DMA_USART3_RecevIndicator.CurrentDataStartNum)
		 {
			 for(i=0; i<DMA_RecevLength; i++,j++)
			 {
					Usart3_recev_buff[i] =DMA_USART3_RecevBuff[j];	
			 }
		 }
		 else
		 {
			 for(i=0; i<DMA_RecevLength; i++)
			 {
					if( j<(RECEIVEBUFF_SIZE-1) )
					{
						 Usart3_recev_buff[i] =DMA_USART3_RecevBuff[j];
						 j++;				
					}
					else if( j==(RECEIVEBUFF_SIZE-1) )
					{
						 Usart3_recev_buff[i] =DMA_USART3_RecevBuff[j];
						 j =0;				 
					}
			  } 
      }
    }
	  return DMA_RecevLength;
}
/*******************************************************************************
* Function Name  : u16  DMA_UART3_RecevDetect(u8* pDeviceID, u16 sNodeAddress)
* Description    : ���DMA���յ�����
* Input          : �ڵ��ID�� ��ַ
* Output         : ����״̬
* Return         : �������
*******************************************************************************/
u16  DMA_UART3_RecevDetect(u8* pDeviceID, u16 sNodeAddress)
{
	int DataLength =0;
	int i=0;
	u16 StateFlag =0;                         //���ݷ��������־                        
	
  if(DMA_UART3_RECEV_FLAG==1)
  {

		 DataLength = DMA_UART3_RecevDataGet();             //��ȡDMA����
		 if(DataLength>0)
		 {
#if DEBUG_TEST 
				printf("\r\nDataLength:%d\r\n", DataLength);             //�����ݳ��Ƚ��������ʾ
#endif
			  for(i=0;i<DataLength;i++)
			  {
             printf(" %.2x ",Usart3_recev_buff[i]);               //�����յ����ݽ�����ʾ

        }

        StateFlag = Receive_Data_Analysis(pDeviceID, sNodeAddress);	      //����433�������ݷ�����������������
        			
		 }
		 else
		 {
#if DEBUG_TEST 
        printf("\r\nNo data\r\n");
#endif
     }
		 DMA_Cmd(DMA1_Channel6, DISABLE);           //�ر�DMA
		 memset(DMA_USART3_RecevBuff,0x00,RECEIVEBUFF_SIZE);   //������3�Ľ����������
     DMA_UART3_RECEV_FLAG =0;
		 DMA_Cmd(DMA1_Channel6, ENABLE);            //����DMA
		 USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);   // ���д���2����
  } 
	return StateFlag;
}





/*******************************************************************************
* Function Name  : void SenserDataCollect(struct SenserData* pGetData, u8* pDevID, u16 NodeAddr)
* Description    : �������ɼ�����,�����з���
* Input          : ������������ʽ���ڵ�ID���ڵ��ַ
* Output         : None
* Return         : None
*******************************************************************************/
void SenserDataCollect(struct SenserData* pGetData, u8* pDevID, u16 NodeAddr)
{
      

	    uint8_t   sensordata[7]={0x00};                        //������������д��FLASH,����������ɼ�ʱ�䣬���������ݣ��ɼ�������
   		u8        i=0;                                         //����ʹ��
			float sensor_collect[COLLECT_NUM];
			u32 WorkTime;
		  struct TagStruct AlarmTAG;

      printf("\r\n��������ʼ�ɼ�����\r\n");       //������ʾ


/////////////////////////////////���ж�βɼ����ݣ��ɼ���Ϻ󣬽��д�������洢

			Sensor_PowderTime=RTC_GetCounter();
			WorkTime=Sensor_PowderTime-Sensor_PowderOn;

			printf("\r\n����������180S�ϵ�Ԥ��\r\n");

			if(WorkTime < SNESOR_WAIT)
			{
				Delay_ms((SNESOR_WAIT-WorkTime)*1000);                      //���д�����Ԥ��
			}

			for(i=0;i<COLLECT_NUM;i++)
			{
				sensor_collect[i]=ReadSensorData(ADC1ConvertedValue);                     //ÿ���1����вɼ�

				printf("\r\nNO:%d ����������: %0.2f",i,sensor_collect[i]);            //���ɼ������ݽ��д�ӡ

				Delay_ms(500);
			}
    	pGetData->Ch4Data.Data_Float  = Filter(sensor_collect,COLLECT_NUM);                                                 //
	    pGetData->CollectTime =(DeviceConfig.Time_Hour)*60 +(DeviceConfig.Time_Min);      //���ݲɼ�ʱ��
    	pGetData->DataCount =  DataCollectCount+1;                                        //��ǰ�ɼ��ĸ���
			Delay_ms(500);   
/*********************************���ɼ������ݸ���д��FLASH***************************************************/
      DataCollectCount = pGetData->DataCount;
			ConfigData.CollectNum = DataCollectCount;
			DataWrite_To_Flash(0,9,0,&ConfigData.CollectNum,1);                           //���Ѳɼ������ݸ���д��FLASH
/*********************************�������Ĵ���д��FLASH***************************************************/			
			ConfigData.WorkNum  += 1;
			DataWrite_To_Flash(0,12,0,&ConfigData.WorkNum,1);                             //���ѹ����Ĵ���д��FLASH			
			
/*********************************���ɼ��Ĵ���������д��FLASH***************************************************/
			sensordata[0]= pGetData->CollectTime >> 8;
			sensordata[1]= pGetData->CollectTime & 0xff;                                 //�ɼ�ʱ��
      for(i=0;i<4;i++)
      sensordata[2+i] =	pGetData->Ch4Data.Data_Hex[i];                             //�ɼ��Ĵ���������
      sensordata[6] = pGetData->DataCount;			                                   //�ɼ�������
      DataWrite_To_Flash(1,pGetData->DataCount,0,(uint8_t*)sensordata,sizeof(sensordata));       //���ݴ�Sector1��ʼд��
			
			printf("\r\n�ɼ����ݴ���FLASH�� NO.%d ����!\r\n",pGetData->DataCount);
      PowerOFF_Sensor();

//////////////�����������Ũ�ȴﵽ�˱�����ֵ����������������ȵ��ϱ�ʱ��ĵ���///////////////////////		
				if((pGetData->Ch4Data.Data_Float >=DeviceConfig.HighAlarmLevel .Data_Float) ||(pGetData->Ch4Data.Data_Float>=DeviceConfig.LowAlarmLevel .Data_Float)  )
				{
					
					
					AlarmTAG.OID_Command=(DeviceConfig.CollectPeriod<<11)+pGetData->CollectTime  +((0xC0 + REPORTDATA )<<24);
					AlarmTAG.OID_Command=ntohl(AlarmTAG.OID_Command);
					AlarmTAG.Width=4;
					for(i=0;i<AlarmTAG.Width;i++)
					AlarmTAG.Value[i]=pGetData->Ch4Data.Data_Hex[i];                                 //��ɱ�����TAG
					printf("\r\n���������б�����Ϣ�ϴ�\r\n");
					SendDataToServ(TRAPREQUEST,&AlarmTAG,1,Usart3_send_buff,pDevID);                //433ģ�������ϴ�����
		      //AlarmTrap(Usart3_send_buff, pDevID, NodeAddr, &PerceptionData);             //3Gģ�������ϴ�����
		      Send_Request_flag = DMA_UART3_RecevDetect(pDevID, NodeAddr);       //ÿ��5�뷢��һ�Σ�������3�Σ��յ�����ʱ
					if(Send_Request_flag == 1)
		      {
		      Send_Request_flag = 0;	
		    
		      }

				  LiquidDataSend_Flag=1;                 //���ݳɹ����͵���������־����
			 }		
///////////////////////////////////////////////////////////////////////////////////////////////////////			
			 if(DataCollectCount  < SensorCollectNumber)
			 {    
			      gotoSleep(DeviceConfig.CollectPeriod  );               //����˯��״̬���ȴ���һ�βɼ�����
        }                                                         //���δ�ﵽ�ϱ�ʱ�䣬�Բɼ�ʱ��˯�� 
			 else
				if( DataCollectCount  == SensorCollectNumber )      //�ɼ���ϣ����������ϱ� ;���ɼ���������ʱ�����ж��Ƿ���ֵ�ϱ���һ��
				{
		        Delay_ms(100); 
				    DeviceConfig.BatteryCapacity = DS2780_Test(); //����ص���
				
				  printf("\r\n���������вɼ���Ϣ�ϴ�\r\n");
		      SendDataToServ(TRAPREQUEST,NULL,0,Usart3_send_buff,pDevID);                //433ģ�������ϴ�����
					
		      Send_Request_flag = DMA_UART3_RecevDetect(pDevID, NodeAddr);                //ÿ��5�뷢��һ�Σ�������5�Σ��յ�����ʱ�Ͽ�TCP����
		      if(Send_Request_flag == 1)
		       {
		      Send_Request_flag = 0;	
		   		 }
 
				LiquidDataSend_Flag=1;                 //Һλ���ݳɹ����͵���������־����
	      
         }
}




/*******************************************************************************
* Function Name  : int main(void)
* Description    : ������
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int main( void )
{
	u8   i=0;
	u8   DeviceID[6] = SENSORID ;                                 //��ʼ���豸ID��
  u16  NodeAddr =0x0000;                                        //��ȡ�豸ID������������ֽ���Ϊ�ڵ��ַ
  NodeAddr=DeviceID[4]*256 +DeviceID[5];
  PeripheralInit();                            //Ӳ����ʼ��
/***********************************�Բɼ��������д���,һ����ഫ15����*****************************************************/
	if(DeviceConfig.SendCount < DeviceConfig.CollectPeriod)
	{DeviceConfig.CollectPeriod = DeviceConfig.SendCount ;}
	
	SensorCollectNumber = DeviceConfig.SendCount / DeviceConfig.CollectPeriod ;
	if(SensorCollectNumber>15)
	{ SensorCollectNumber=15;	}

   DataRead_From_Flash(0,9,0, &(DeviceConfig.CollectNum ) ,1);            //��ȡ�Ѳɼ��������� 
   DataCollectCount = DeviceConfig.CollectNum ;
   if(DataCollectCount > SensorCollectNumber)                     //����Ѳɼ�������������Ҫ�ɼ������������Ѳɼ���������
	 {
		 DataCollectCount=0;
	 }
#if DEBUG_TEST 
	printf("\r\n The  Collect Number is %d \r\n",DeviceConfig.CollectNum );
#endif	
	//Delay_ms(2000);
	//printf("\r\n�豸�ϱ�������Ϣ...\r\n");
	//Client(DeviceID);
	
  //DeviceStartupRequest(Usart3_send_buff, DeviceID, NodeAddr);      //�����ϱ���Ϣ
	Delay_ms(500);  
  
	while(1)
  {
 
		  WWDOG_Feed =0x1FFF;                            //���ڿ��Ź�ι��,��ʱ4��20�룬�ڶ��ֽ�Լ1��仯һ�Σ���0x09AF�䵽0x099FԼ��ʱ1��
 	    for(i=0;i<5;i++)
		{
			 Delay_ms(2000);                                //����Ƿ������ݴ������
			 if(DMA_UART3_RECEV_FLAG==1)                   //��ѯ���ݽ������
			 {
				 DMA_UART3_RecevDetect(DeviceID, NodeAddr);   
				 break;
			 }
    }	
			if(DataCollectCount < SensorCollectNumber)   //���Ѳɼ�������С�ڹ��ɼ�����
			{
#if DEBUG_TEST 				
				printf("\r\nThe Sensor Has DataCollectCount is %d \r\n",DataCollectCount);  
#endif				
				SenserDataCollect(&PerceptionData, DeviceID, NodeAddr);        //��ȡ����������
      }
			else                                                             
			{	
				if(DataCollectCount == SensorCollectNumber)         //������ݲɼ��󣬿�ʼ�ϴ�����
				DataCollectCount=0;                                   //�����ݽ������㣬���²ɼ�   
		  }
			
			if(LiquidDataSend_Flag ==1)            
			{ 
				gotoSleep(DeviceConfig.CollectPeriod );
			}
	}
	
}

/*******************************************************************************
* Function Name  : void PeripheralInit( void )
* Description    : ��ʼ���˿ڼ�����
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void PeripheralInit( void )
{

	WWDG_Init(0x7F,0X5F,WWDG_Prescaler_8); 	//���ȿ������ڿ��Ź���������ֵΪ7f,���ڼĴ���Ϊ5f,��Ƶ��Ϊ8	
	USART1_Config();      /* USART1 ����ģʽΪ 9600 8-N-1��  �жϽ��� */    //���ڴ��ڵ�������
	USART2_Config();      /* USART2 ����ģʽΪ 9600 8-N-1���жϽ��� */    //����433ͨ�ŷ�ʽ
	USART4_Config();      /* UART4  ����ģʽΪ 38400 8-N-1��  �жϽ��� */    //���ڴ�����ͨ��
  UART_NVIC_Configuration();
	USART2_DMA_Config();
//USART3_DMA_Config();
//	TIM3_Configuration();       /* ��ʱ��TIM3�������� */	
//	TIM3_NVIC_Configuration();  /* ���ö�ʱ��TIM3���ж����ȼ� */
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, DISABLE);     //��ʱ�رն�ʱ��TIM3
	
	RTC_NVIC_Config();                 /* ����RTC���ж����ȼ� */
	RTC_CheckAndConfig(&systmtime);
  SysTick_Init();
	Power_SX1278_Init();               //433ģ��IO������
	ADCconfig();                       //����ADC��ʼ��
	PowerON_433();                    //��GPRSģ���Դ
	PowerON_Sensor();       					//�򿪴�����
	Sensor_PowderOn=RTC_GetCounter();   //��¼������������ʼʱ��
	Delay_ms(500);
  Display();                          //���г�����ʾ
	
	
/**********************************���в�������*******************************/
#if CONFIG>0
  Device_Init();                            //�����豸��ʼ������ҪΪ�ɼ���Ϣ��433������Ϣ
#endif
   
  if (PowerOffReset ==1)     
  {
    printf("\r\n�������������³�ʼ�����ؼ�\r\n");                 //����ʹ��
    Set_DS2780();            //���п��ؼƳ�ʼ��
  	PowerOffReset =0;       //����������־������λ
  }

	ConfigData_Init(&DeviceConfig);
	
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
	GPIO_SetBits(GPIOC,GPIO_Pin_5);                  //433ģ��SET�ܽ����ߣ��л�������ģʽ
	Delay_ms(100);
  GPIO_ResetBits(GPIOB,GPIO_Pin_0);                //433ģ��EN�ܽ����ͣ��л�������ģʽ
	Delay_ms(100);
	USART_GetFlagStatus(USART2,USART_FLAG_TC);       //����Ӳ����λ֮�󣬷������ֽ�֮ǰ���ȶ�һ��USART_SR,��ֹ���ݷ���ʱ���ֽڱ����� 
}

/*******************************************************************************
* Function Name  : void Display( void )
* Description    : ��ӡ������Ϣ
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Display(void)
{
	u32       TimCount_Current =0;
	u8   DeviceID[6] = SENSORID;
	TimCount_Current = RTC_GetCounter();
	
	printf("\r\n********* �����������������޹�˾ *********");
	printf("\r\n************* ȼ�����ܼ���ն� ***********");
	printf("\r\n************* BIRMM-RTU100 ***********");
	printf("\r\n************** Ӳ������: 433�汾 **********");
	printf("\r\n************** �豸ID��: %x %x %x %x %x %x**********",DeviceID[0],DeviceID[1],DeviceID[2],DeviceID[3],DeviceID[4],DeviceID[5]);
	printf("\r\n************** ��ǰӲ���汾��: %x ***********",HARDVERSION);
	printf("\r\n************** ��ǰ����汾��: %x **************\r\n\r\n",SOFTVERSION);
	
	Time_Display(TimCount_Current,&systmtime); 
	DeviceConfig.Time_Sec  =systmtime.tm_sec;
	DeviceConfig.Time_Min  =systmtime.tm_min;
	DeviceConfig.Time_Hour =systmtime.tm_hour;
	DeviceConfig.Time_Mday =systmtime.tm_mday;		
	DeviceConfig.Time_Mon  =systmtime.tm_mon;
	DeviceConfig.Time_Year =systmtime.tm_year-2000; //���ϴ����ȥ��������				
		    

printf("**************��ǰʱ��:%0.4d-%0.2d-%0.2d %0.2d:%0.2d:%0.2d***********\r\n",systmtime.tm_year,systmtime.tm_mon,     //����ʹ��
				systmtime.tm_mday,systmtime.tm_hour,systmtime.tm_min,systmtime.tm_sec);     //����ʹ��
  Delay_ms(3000);
}

/*******************************************************************************
* Function Name  : void Client( void )
* Description    : ��ӡ������Ϣ
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Client(u8* pDeviceID)
{
	  struct TagStruct TagList;
	  uint8_t TagNum;
		TagList.OID_Command =ntohl(DEVICE_STATE); //??????,??????,?????
		TagList.Width =1;
		TagList.Value[0]=1;
		TagNum =1;
	  SendDataToServ(CLIENTREQUEST,&TagList,TagNum,Usart3_send_buff,pDeviceID);
}


#ifdef  DEBUG
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(u8* file, u32 line)
{ 
  /* User can add his own implementation to report the file name and line number */
 
  printf("\n\r Wrong parameter value detected on\r\n");
  printf("       file  %s\r\n", file);
  printf("       line  %d\r\n", line);
    
  /* Infinite loop */
  /* while (1)
  {
  } */
}
#endif
/*********************************************END OF FILE**********************/
