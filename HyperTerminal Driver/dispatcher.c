/*****************************************************************************
*
*  dispatcher.c  - CC3000 Host Driver Implementation.
*  Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the   
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*****************************************************************************/

//*****************************************************************************
//
//! \addtogroup dispatcher_api
//! @{
//
//*****************************************************************************
#include "dispatcher.h"

#include <stdint.h>
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/pin_map.h"
#include "utils/uartstdio.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"


///////////////////////////////////////////////////////////////////////////////////////////////////////////                     
//__no_init is used to prevent varible's initialize.                                                    ///
//for every IDE, exist different syntax:          1.   __CCS__ for CCS v5                               ///
//                                                2.  __IAR_SYSTEMS_ICC__ for IAR Embedded Workbench    ///
// *CCS does not initialize variables - therefore, __no_init is not needed.                             ///
///////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef __CCS__
unsigned char g_ucUARTBuffer[UART_IF_BUFFER];
#elif __IAR_SYSTEMS_ICC__
__no_init unsigned char g_ucUARTBuffer[UART_IF_BUFFER];

#endif
volatile unsigned char uart_have_cmd = 0;
volatile unsigned long g_ulRxBuffCount =0;

unsigned long g_uluDMAErrCount =0;




//*****************************************************************************
//
//! UARTIntHandler
//!
//!  \param  buffer
//!
//!  \return none
//!
//!  \brief  Handles RX and TX interrupts
//
//*****************************************************************************
void UARTIntHandler(void)
{
        if (UARTIntStatus(UART0_BASE, TRUE))
	{	// We are reading char by char until the first 'enter'
		
                g_ucUARTBuffer[g_ulRxBuffCount] = UARTCharGet(UART0_BASE);              
                g_ulRxBuffCount++;
        	 if (g_ucUARTBuffer[g_ulRxBuffCount-1] == 0x0D)
                {
                    g_ulRxBuffCount = 0;
                    uart_have_cmd = 1;
                }
        /*Check for buffer overrun*/
              
        if (g_ulRxBuffCount >= UART_IF_BUFFER)
            g_ulRxBuffCount =0;
         
	}

}

//*****************************************************************************
//
//! DispatcherUartSendPacket
//!
//!  \param  inBuff    pointer to the UART input buffer
//!  \param  usLength  buffer length
//!
//!  \return none
//!
//!  \brief  The function sends to UART a buffer of given length 
//
//*****************************************************************************
void DispatcherUartSendPacket(unsigned char *inBuff, unsigned short usLength)
{

	unsigned long ulIndex=0;
	for (ulIndex =0 ; ulIndex < usLength; ulIndex++)
	   MAP_UARTCharPut(UART0_BASE, *inBuff++);	
	
}


//*****************************************************************************
//
//! Cofigure the UART
//!
//!  @param  none
//!
//!  @return none
//!
//!  @brief  Cofigure the UART
//
//*****************************************************************************
void
DispatcherUARTConfigure(unsigned long ucSysClock)
{    
	g_ulRxBuffCount = 0;

    //
    // Enable corresponding peripherals modules
    // The UART Init shall be in test code
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    //
    // Set GPIO A0 and A1 as UART pins
    //
    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1,
                    GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    GPIOPinConfigure(0x00000001); //GPIO_PA0_U0RX
    GPIOPinConfigure(0x00000401); //GPIO_PA1_U0TX
    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    //
    // Configure the UART for 115,200, 8-N-1 operation
    //
    MAP_UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
   
    MAP_UARTIntDisable(UART0_BASE, 0xFFFFFFFF);
    
    
    
    // Set both the TX and RX trigger thresholds to 4.  This will be used by
    // the uDMA controller to signal when more data should be transferred.  The
    // uDMA TX and RX channels will be configured so that it can transfer 2
    // bytes in a burst when the UART is ready to transfer more data.
    //
    UARTFIFOLevelSet(UART0_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8);
    //
    // Enable the UART interrupt but first clear all the pending interrupts
    //
    
    
   
     
    MAP_UARTIntClear(UART0_BASE, 0xFFFFFFFF);

    MAP_UARTIntEnable(UART0_BASE, UART_INT_RT|UART_INT_RX);
    
    MAP_IntEnable(INT_UART0);

	
}


//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
