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
// File:    hal-rxqueue.c
// Author:  Mickle.ding
// Created: 11/2/2011
//
// Description:  Define the class hal-rxqueue
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "API-WiMinet.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "API-IOShell.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "API-IOQueue.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "HAL-RxQueue.h"

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void HAL_SetupRxQueueStateMachine()
{
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void HAL_CloseRxQueueStateMachine()
{
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char ReadASCIIMessage( char * pBuffer, unsigned char iSize )
{
   // Reset the buffer contents
   memset( pBuffer, 0X00, iSize );

   // Get the string message
   return API_GetStringQueue( pBuffer, iSize );
}
