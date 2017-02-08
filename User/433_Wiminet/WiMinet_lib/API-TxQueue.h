// #############################################################################
// *****************************************************************************
//                  Copyright (c) 2007-2009, WiMi-net Corp.
//      THIS IS AN UNPUBLISHED WORK CONTAINING CONFIDENTIAL AND PROPRIETARY
//               INFORMATION WHICH IS THE PROPERTY OF WIMI-NET CORP.
//
//    ANY DISCLOSURE, USE, OR REPRODUCTION, WITHOUT WRITTEN AUTHORIZATION FROM
//                   WIMI-NET CORP., IS STRICTLY PROHIBITED.
// *****************************************************************************
// #############################################################################
//
// File:    api-txqueue.h
// Author:  Mickle.ding
// Created: 11/2/2011
//
// Description:  Define the class api-txqueue
// -----------------------------------------------------------------------------


// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#ifndef _API_TX_QUEUE_INC_

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#define _API_TX_QUEUE_INC_

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include <Windows.h>

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "API-Message.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "API-Queue16.h"


// -----------------------------------------------------------------------------
// DESCRIPTION: 
// -----------------------------------------------------------------------------
#define QOS_NOQOS_MESSAGE                                      0X00

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#define QOS_HIQOS_MESSAGE                                      0X01

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#define QOS_SHORT_MESSAGE                                      0X02






// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#define API_TX_STATUS_ONMARCH                                  0X00

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#define API_TX_STATUS_SUCCESS                                  0X01

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#define API_TX_STATUS_FAILURE                                  0X02




// -----------------------------------------------------------------------------
// DESCRIPTION: ���͹��̣���ʼ��ͨѶ���̣������������ϵ�����ڵ�Ȳ���
// -----------------------------------------------------------------------------
#define API_TX_STATUS_INIT                                     0X01

// -----------------------------------------------------------------------------
// DESCRIPTION: ���͹��̣���ʼ��ͨѶ���̣������������ϵ�����ڵ�Ȳ���
// -----------------------------------------------------------------------------
#define API_TX_STATUS_OPEN                                     0X02

// -----------------------------------------------------------------------------
// DESCRIPTION: ���͹��̣�����ͨѶȫ����������֪ͨ���շ��ļ��Ĵ�С��CRC32
// -----------------------------------------------------------------------------
#define API_TX_STATUS_SINF                                     0X03

// -----------------------------------------------------------------------------
// DESCRIPTION: ���͹��̣��������ݱ���
// -----------------------------------------------------------------------------
#define API_TX_STATUS_SEND                                     0X04

// -----------------------------------------------------------------------------
// DESCRIPTION: ���͹��̣��ȴ��������
// -----------------------------------------------------------------------------
#define API_TX_STATUS_WAIT                                     0X05

// -----------------------------------------------------------------------------
// DESCRIPTION: ���͹��̣���ӡͨѶ���棬������Щ�ڵ㴫��ɹ�����Щ�ڵ㴫��ʧ��
// -----------------------------------------------------------------------------
#define API_TX_STATUS_REPT                                     0X06

// -----------------------------------------------------------------------------
// DESCRIPTION: ���͹��̣�ͨѶ���
// -----------------------------------------------------------------------------
#define API_TX_STATUS_OVER                                     0X07

// -----------------------------------------------------------------------------
// DESCRIPTION: ���͹��̣�ͨѶ����
// -----------------------------------------------------------------------------
#define API_TX_STATUS_IDLE                                     0X08



// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
typedef struct _TX_TASK_INFO_
{   
   // --------------------------------------------------------------------------
   // DESCRIPTION:
   // --------------------------------------------------------------------------
   char                                                        m_pBuffer[1024*1024];
   
   // --------------------------------------------------------------------------
   // DESCRIPTION:
   // --------------------------------------------------------------------------
   unsigned long                                               m_dwBufferSize;
   
   // --------------------------------------------------------------------------
   // DESCRIPTION:
   // --------------------------------------------------------------------------
   unsigned long                                               m_dwPacketSize;

   // --------------------------------------------------------------------------
   // DESCRIPTION:
   // --------------------------------------------------------------------------
   unsigned long                                               m_dwOffsetSize;
   
   // --------------------------------------------------------------------------
   // DESCRIPTION:
   // --------------------------------------------------------------------------
   unsigned long                                               m_dwCRC32;
   
   // --------------------------------------------------------------------------
   // DESCRIPTION:
   // --------------------------------------------------------------------------
   unsigned short                                              m_iNode;

   // --------------------------------------------------------------------------
   // DESCRIPTION:
   // --------------------------------------------------------------------------
   unsigned char                                               m_iQoS;
   
   // --------------------------------------------------------------------------
   // DESCRIPTION:
   // --------------------------------------------------------------------------
   unsigned long                                               m_dwTaskID;

   // --------------------------------------------------------------------------
   // DESCRIPTION:
   // --------------------------------------------------------------------------
   unsigned char                                               m_iStatus; 

   // --------------------------------------------------------------------------
   // DESCRIPTION:
   // --------------------------------------------------------------------------
   ULARGE_INTEGER                                              m_qwInitTimer;   
   
   // --------------------------------------------------------------------------
   // DESCRIPTION:
   // --------------------------------------------------------------------------
   ULARGE_INTEGER                                              m_qwOpenTimer;

   // --------------------------------------------------------------------------
   // DESCRIPTION:
   // --------------------------------------------------------------------------
   ULARGE_INTEGER                                              m_qwSendTimer;
   
   // --------------------------------------------------------------------------
   // DESCRIPTION:
   // --------------------------------------------------------------------------
   ULARGE_INTEGER                                              m_qwThisTimer;
   
   // --------------------------------------------------------------------------
   // DESCRIPTION:
   // --------------------------------------------------------------------------
   ULARGE_INTEGER                                              m_qwTimerSpan;   
   
} TxTaskInfo;



// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
typedef struct _API_TX_QUEUE_STATE_MACHINE_
{
   // --------------------------------------------------------------------------
   // DESCRIPTION:
   // --------------------------------------------------------------------------
   unsigned char                                               m_iMainStatus;
   
   // --------------------------------------------------------------------------
   // DESCRIPTION:
   // --------------------------------------------------------------------------
   ULARGE_INTEGER                                              m_qwOpenTimer;
   
   // --------------------------------------------------------------------------
   // DESCRIPTION:
   // --------------------------------------------------------------------------
   ULARGE_INTEGER                                              m_qwThisTimer;
   
   // --------------------------------------------------------------------------
   // DESCRIPTION:
   // --------------------------------------------------------------------------
   ULARGE_INTEGER                                              m_qwTimerSpan;
   
   // --------------------------------------------------------------------------
   // DESCRIPTION:
   // --------------------------------------------------------------------------
   unsigned char                                               m_iDescriptor;

   // --------------------------------------------------------------------------
   // DESCRIPTION:
   // --------------------------------------------------------------------------
   unsigned char                                               m_pBuffer[0XFF]; 
   
   // --------------------------------------------------------------------------
   // DESCRIPTION:
   // --------------------------------------------------------------------------
   NodeMsg *                                                   m_pMsg;   
   
   // --------------------------------------------------------------------------
   // DESCRIPTION:
   // --------------------------------------------------------------------------
   DWORD                                                       m_dwCRC32;
   
   // --------------------------------------------------------------------------
   // DESCRIPTION:
   // --------------------------------------------------------------------------
   DWORD                                                       m_dwSize;

   // --------------------------------------------------------------------------
   // DESCRIPTION:
   // --------------------------------------------------------------------------
   unsigned short                                              m_iThisNodeID;
      
   
} API_TxQueue_StateMachine;


// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
void API_SetupTxTask();

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
void API_CloseTxTask();

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
void API_SetupTxQueueStateMachine();

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
void API_CloseTxQueueStateMachine();

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
unsigned char API_GetTxPrepareStatus();

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
unsigned long GetOpenTimerSpan();

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
unsigned long GetSendTimerSpan();

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
void API_TxStateHandler_Send();

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
void API_SwitchTxStatus( unsigned char iStatus );

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
unsigned char API_TxStateHandler_CheckEnd();

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
void API_TxStateHandler_Init();

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
void API_TxStateHandler_Open();

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
void API_TxStateHandler_SInf();

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
void API_TxStateHandler_Wait();

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
void API_TxStateHandler_Rept();

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
void API_TxStateHandler_Over();

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
void API_TxStateHandler_Idle();

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
DWORD CommitTransmitTask( short iNode, char * pBuffer, DWORD dwSize, char iQoS );

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
void API_TxQueueHandler();

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#endif