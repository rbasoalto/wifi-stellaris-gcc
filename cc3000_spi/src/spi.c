
/*****************************************************************************
*
*  spi.c - CC3000 Host Driver Implementation.
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
//! \addtogroup link_buff_api
//! @{
//
//*****************************************************************************


#include "hci.h"
#include "spi.h"

#include "inc/hw_types.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/udma.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"
#include "driverlib/ssi.h"
#include "inc/hw_ints.h"
#include "inc/hw_nvic.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ssi.h"
#include "inc/hw_uart.h"

#define SPI_BASE                    SSI2_BASE
#define SPI_CLK_PIN                 GPIO_PIN_4
#define SPI_RX_PIN                  GPIO_PIN_6
#define SPI_TX_PIN                  GPIO_PIN_7

#define SYSCTL_PERIPH_SPI           SYSCTL_PERIPH_SSI2
#define SYSCTL_PERIPH_SPI_BASE      SYSCTL_PERIPH_GPIOB

#define SPI_PORT                    GPIO_PORTB_BASE
#define SPI_CLK_MUX_SEL             0x00011002 //GPIO_PB4_SSI2CLK
#define SPI_RX_MUX_SEL              0x00011802 //GPIO_PB6_SSI2RX
#define SPI_TX_MUX_SEL              0x00011C02 //GPIO_PB7_SSI2TX

#define SPI_UDMA_RX_CHANNEL          UDMA_CH12_SSI2RX
#define SPI_UDMA_TX_CHANNEL          UDMA_CH13_SSI2TX

#define FWT_DELAY               4000
#define DMA_WINDOW_SIZE         1024

#define SPI_WINDOW_SIZE         DMA_WINDOW_SIZE
#define DMA_CHANNEL_CONTROL_STRUCTURE_SIZE  (512)
#define READ                    3
#define WRITE                   1

#define HI(value)               (((value) & 0xFF00) >> 8)
#define LO(value)               ((value) & 0x00FF)

#define ASSERT_CS()          (MAP_GPIOPinWrite(SPI_CS_PORT, SPI_CS_PIN, 0))

#define DEASSERT_CS()        (MAP_GPIOPinWrite(SPI_CS_PORT, SPI_CS_PIN, 0xFF))

#define HEADERS_SIZE_EVNT       (SPI_HEADER_SIZE + 5)

#define SPI_HEADER_SIZE         (5)

#define     eSPI_STATE_POWERUP               (0)
#define     eSPI_STATE_INITIALIZED           (1)
#define     eSPI_STATE_IDLE                  (2)
#define     eSPI_STATE_WRITE_IRQ             (3)
#define     eSPI_STATE_WRITE_FIRST_PORTION   (4)
#define     eSPI_STATE_WRITE_EOT             (5)
#define     eSPI_STATE_READ_IRQ              (6)
#define     eSPI_STATE_READ_FIRST_PORTION    (7)
#define     eSPI_STATE_READ_EOT              (8)



typedef struct
{
    unsigned long ulPioPortAddress;
    unsigned long ulPioSpiPort;
    unsigned long ulPioSpiCs;
    unsigned long ulPortInt;
    unsigned long ulPioSlEnable;

    unsigned long uluDmaPort;
    unsigned long uluDmaRxChannel;
    unsigned long uluDmaTxChannel;

    unsigned long ulSsiPort;
    unsigned long ulSsiPortAddress;
    unsigned long ulSsiTx;
    unsigned long ulSsiRx;
    unsigned long ulSsiClck;
    unsigned long ulSsiPortInt;
}tSpiHwConfiguration;

typedef struct
{
    gcSpiHandleRx  SPIRxHandler;

    unsigned short usTxPacketLength;
    unsigned short usRxPacketLength;
    volatile unsigned long  ulSpiState;
    unsigned char *pTxPacket;
    unsigned char *pRxPacket;
    tSpiHwConfiguration sHwSettings;
}tSpiInformation;


tSpiInformation sSpiInformation;


// The magic number that resides at the end of the TX/RX buffer (1 byte after
// the allocated size) for the purpose of detection of the overrun. The location
// of the memory where the magic number  resides shall never be written.
// In case it is written - the overrun occurred and either receive function or
// send function will stuck forever.

#define CC3000_BUFFER_MAGIC_NUMBER (0xDE)

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//#pragma is used for determine the memory location for a specific variable.                            ///
//__no_init is used to prevent the buffer initialization in order to prevent hardware WDT expiration    ///
// before entering to 'main()'.                                                                         ///
//for every IDE, different syntax exists :          1.   __CCS__ for CCS v5                             ///
//                                                  2.  __IAR_SYSTEMS_ICC__ for IAR Embedded Workbench  ///
// *CCS does not initialize variables - therefore, __no_init is not needed.                             ///
///////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef __CCS__
unsigned char wlan_rx_buffer[CC3000_RX_BUFFER_SIZE];
unsigned char wlan_tx_buffer[CC3000_TX_BUFFER_SIZE];
unsigned char chBuffer[CC3000_RX_BUFFER_SIZE];
#pragma DATA_ALIGN(ucDMAChannelControlStructure, 1024);
static unsigned char ucDMAChannelControlStructure[DMA_CHANNEL_CONTROL_STRUCTURE_SIZE];
#elif __IAR_SYSTEMS_ICC__
__no_init unsigned char wlan_rx_buffer[CC3000_RX_BUFFER_SIZE];
__no_init unsigned char wlan_tx_buffer[CC3000_TX_BUFFER_SIZE];
__no_init unsigned char chBuffer[CC3000_RX_BUFFER_SIZE];
#pragma data_alignment=1024
__no_init static unsigned char ucDMAChannelControlStructure[DMA_CHANNEL_CONTROL_STRUCTURE_SIZE];
#endif


// Static buffer for 5 bytes of SPI HEADER
unsigned char tSpiReadHeader[] = {READ, 0, 0, 0, 0};

void SpiWriteDataSynchronous(const unsigned char *data, unsigned short size);
void SpiWriteAsync(const unsigned char *data, unsigned short size);
void SpiReadData(unsigned char *data, unsigned short size);
void SpiDisableInterrupts(void);
void SpiResumeSpi(void);


//*****************************************************************************
//
//!  SpiSysDelay
//!
//!  @param  ulTicks ticks
//!
//!  @return none
//!
//!  @brief  This function delay for a few sys ticks.
//!
//
//*****************************************************************************

void
SpiSysDelay(unsigned long ulTicks)
{
    SysCtlDelay(ulTicks);
}

//*****************************************************************************
//
//!  SpiConfigureHwMapping
//!
//!  @param  none
//!
//!  @return none
//!
//!  @brief  This function get the reason for the GPIO interrupt
//!          and clear corresponding interrupt flag
//
//*****************************************************************************
void
SpiConfigureHwMapping(void)
{
    sSpiInformation.sHwSettings.ulPioPortAddress = SYSCTL_PERIPH_SPI_PORT;
    sSpiInformation.sHwSettings.ulPioSpiCs   = SPI_CS_PIN;
    sSpiInformation.sHwSettings.ulPioSpiPort = SPI_PORT;
    sSpiInformation.sHwSettings.uluDmaPort   = SYSCTL_PERIPH_UDMA;
    sSpiInformation.sHwSettings.ulPortInt    = INT_GPIO_SPI;
    sSpiInformation.sHwSettings.ulSsiPortAddress = SYSCTL_PERIPH_SPI;
    sSpiInformation.sHwSettings.ulSsiTx = SPI_TX_PIN;
    sSpiInformation.sHwSettings.ulSsiRx = SPI_RX_PIN;
    sSpiInformation.sHwSettings.ulSsiClck = SPI_CLK_PIN;
    sSpiInformation.sHwSettings.ulSsiPort = SPI_BASE;
    sSpiInformation.sHwSettings.ulSsiPortInt = INT_SPI;
    sSpiInformation.sHwSettings.uluDmaRxChannel = SPI_UDMA_RX_CHANNEL;
    sSpiInformation.sHwSettings.uluDmaTxChannel = SPI_UDMA_TX_CHANNEL;
}

//*****************************************************************************
//
//!  SpiCleanGPIOISR
//!
//!  @param  none
//!
//!  @return none
//!
//!  @brief  This function get the reason for the GPIO interrupt
//!          and clear corresponding interrupt flag
//
//*****************************************************************************
void
SpiCleanGPIOISR(void)
{
    unsigned long    ulStatus;

    // Get the reason for the interrupt
    ulStatus = MAP_GPIOPinIntStatus(SPI_GPIO_IRQ_BASE, true);

    // Clear the asserted interrupts
    MAP_GPIOPinIntClear(SPI_GPIO_IRQ_BASE, ulStatus);
}

//*****************************************************************************
//
//!  SSIConfigure
//!
//!  @param  ulSSIFreq
//!  @param  bForceGpioConfiguration
//!  @param  ulSysClck
//!
//!  @return none
//!
//!  @brief  Configure the SSI
//
//*****************************************************************************
void
SSIConfigure(unsigned long ulSSIFreq, unsigned long bForceGpioConfiguration, unsigned long ulSysClck)
{

    MAP_SysCtlPeripheralEnable(sSpiInformation.sHwSettings.ulPioPortAddress);
    MAP_SysCtlPeripheralEnable(sSpiInformation.sHwSettings.ulSsiPortAddress);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_SPI_BASE);
    GPIOPinConfigure(SPI_CLK_MUX_SEL);
    GPIOPinConfigure(SPI_RX_MUX_SEL);
    GPIOPinConfigure(SPI_TX_MUX_SEL);

    // Configure the appropriate pins to be SSI instead of GPIO
    MAP_GPIOPinTypeSSI(sSpiInformation.sHwSettings.ulPioSpiPort, sSpiInformation.sHwSettings.ulSsiTx | sSpiInformation.sHwSettings.ulSsiRx | sSpiInformation.sHwSettings.ulSsiClck);

    MAP_GPIOPadConfigSet(sSpiInformation.sHwSettings.ulPioSpiPort, sSpiInformation.sHwSettings.ulSsiClck, GPIO_STRENGTH_8MA,
                                             GPIO_PIN_TYPE_STD_WPD);

    // Configure and enable the SSI port for master mode
    MAP_SysCtlPeripheralReset(sSpiInformation.sHwSettings.ulSsiPortAddress);

    // Ensure that the SSE bit in the SSICR1 register is clear before making any configuration changes
    MAP_SSIDisable(sSpiInformation.sHwSettings.ulSsiPort);

    // 16MHz SSI with bit 8 bits data (DSS); Polarity '0' Phase '1'
    MAP_SSIConfigSetExpClk(sSpiInformation.sHwSettings.ulSsiPort, ulSysClck, SSI_FRF_MOTO_MODE_1,
                                                 SSI_MODE_MASTER, ulSSIFreq, 8);

    // Enable EOT mode for the SSIRIS EOT bit
    HWREG(sSpiInformation.sHwSettings.ulSsiPort + SSI_O_CR1) |= SSI_CR1_EOT;

    // Enable the SSI by setting the SSE
    MAP_SSIEnable(sSpiInformation.sHwSettings.ulSsiPort);

    // Enable DMA mode for both RX and TX
    MAP_SSIDMAEnable(sSpiInformation.sHwSettings.ulSsiPort, SSI_DMA_TX);

    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);

    // Enable the uDMA controller.
    MAP_uDMAEnable();

    //Configure the DMA channels for the selected SPI Module
    MAP_uDMAChannelAssign( sSpiInformation.sHwSettings.uluDmaRxChannel);
    MAP_uDMAChannelAssign( sSpiInformation.sHwSettings.uluDmaTxChannel);

    // Point at the control table to use for channel control structures
    MAP_uDMAControlBaseSet(ucDMAChannelControlStructure);

    // Put the attributes in a known state for the uDMA SSIRX channel.  These
    // should already be disabled by default.
    MAP_uDMAChannelAttributeDisable(sSpiInformation.sHwSettings.uluDmaRxChannel,
                                                                    UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST |
                                    UDMA_ATTR_HIGH_PRIORITY |
                                                                            UDMA_ATTR_REQMASK);

    // Configure the control parameters for the primary control structure for
    // the SSI RX channel. The transfer data size is 8 bits, the source
    // address does not increment since it will be reading from a register.
    // The destination address increment is byte 8-bit bytes. The arbitration
    // size is set to 1 to match the RX FIFO trigger threshold. The uDMA
    // controller will use a 4 byte burst transfer if possible.
    MAP_uDMAChannelControlSet(sSpiInformation.sHwSettings.uluDmaRxChannel | UDMA_PRI_SELECT,
                                                        UDMA_SIZE_8 | UDMA_SRC_INC_NONE |
                              UDMA_DST_INC_8 | UDMA_ARB_1);

    // Put the attributes in a known state for the uDMA SSITX channel.  These
    // should already be disabled by default.
    MAP_uDMAChannelAttributeDisable(sSpiInformation.sHwSettings.uluDmaTxChannel,
                                                                    UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST |
                                    UDMA_ATTR_HIGH_PRIORITY |
                                                                            UDMA_ATTR_REQMASK);

    // Configure the control parameters for the SSI TX.  The uDMA SSI TX
    // channel is used to transfer a block of data from a buffer to the SSI.
    // The data size is 8 bits.  The source address increment is 8-bit bytes
    // since the data is coming from a buffer.  The destination increment is
    // none since the data is to be written to the SSI data register.  The
    // arbitration size is set to 8, which matches the SSI TX FIFO trigger
    // threshold.
    MAP_uDMAChannelControlSet(sSpiInformation.sHwSettings.uluDmaTxChannel | UDMA_PRI_SELECT,
                                                        UDMA_SIZE_8 | UDMA_SRC_INC_8 |
                              UDMA_DST_INC_NONE | UDMA_ARB_4);

    // Now both the uDMA SSI TX and RX channels are primed to start a
    // transfer.  As soon as the channels are enabled, the peripheral will
    // issue a transfer request and the data transfers will begin.
    //
    // The uDMA TX/RX channel must be disabled.
    MAP_uDMAChannelDisable(sSpiInformation.sHwSettings.uluDmaRxChannel);
    MAP_uDMAChannelDisable(sSpiInformation.sHwSettings.uluDmaTxChannel);

    // Set the USEBURST attribute for the uDMA UART TX channel.  This will
    // force the controller to always use a burst when transferring data from
    // the TX buffer to the UART.  This is somewhat more efficient bus usage
    // than the default which allows single or burst transfers.
    //
    //uDMAChannelAttributeEnable(sSpiInformation.sHwSettings.uluDmaTxChannel, UDMA_ATTR_USEBURST);

    // Enable the SSI interrupt
    MAP_IntEnable(sSpiInformation.sHwSettings.ulSsiPortInt);
}


//*****************************************************************************
//
//!  SpiClose
//!
//!  @param  none
//!
//!  @return none
//!
//!  @brief  Close the SPI interface
//
//*****************************************************************************
void
SpiClose(void)
{
    if (sSpiInformation.pRxPacket)
    {
        sSpiInformation.pRxPacket = 0;
    }

    //  Disable Interrupt in GPIOA module...
    tSLInformation.WlanInterruptDisable();

    // Disable interrupt for SPI IRQ and SSI module
    MAP_IntDisable(sSpiInformation.sHwSettings.ulPortInt);
    MAP_IntDisable(sSpiInformation.sHwSettings.ulSsiPortInt);
}


//*****************************************************************************
//
//!  SpiOpen
//!
//!  @param  none
//!
//!  @return none
//!
//!  @brief  Open the SPI interface
//
//*****************************************************************************
void
SpiOpen(gcSpiHandleRx pfRxHandler)
{
    sSpiInformation.ulSpiState = eSPI_STATE_POWERUP;

    sSpiInformation.SPIRxHandler = pfRxHandler;
    sSpiInformation.usTxPacketLength = 0;
    sSpiInformation.pTxPacket = NULL;
    sSpiInformation.pRxPacket = wlan_rx_buffer;
    sSpiInformation.usRxPacketLength = 0;
    wlan_rx_buffer[CC3000_RX_BUFFER_SIZE - 1] = CC3000_BUFFER_MAGIC_NUMBER;
    wlan_tx_buffer[CC3000_TX_BUFFER_SIZE - 1] = CC3000_BUFFER_MAGIC_NUMBER;

    // Enable interrupt on the GPIOB ping of WLAN IRQ
    tSLInformation.WlanInterruptEnable();

    // Enable interrupts in NVIC
    MAP_IntEnable(sSpiInformation.sHwSettings.ulPortInt);
    MAP_IntEnable(sSpiInformation.sHwSettings.ulSsiPortInt);
}

//*****************************************************************************
//
//!  init_spi
//!
//!  @param  buffer
//!
//!  @return none
//!
//!  @brief  initializes an SPI interface
//
//*****************************************************************************

int init_spi(void)
{
    // Update the configuration in the SPI driver
    SpiConfigureHwMapping();

    // Configure SPI with CC3000
    SSIConfigure(1000000, 1, 50000000);
    return(ESUCCESS);
}

//*****************************************************************************
//
//! SpiCheckDMAStatus
//!
//! @param  ch
//!
//! @return none
//!
//! @brief  This function get the current state of the SSI DMA channels
//
//*****************************************************************************
unsigned long
SpiCheckDMAStatus(long ch)
{
    long chanel;

    //
    // Check the DMA control table to see if the basic transfer is
    // complete.  The basic transfer uses receive buffer g_ucInBuffer,
    // and the primary control structure.
    //
    chanel = (ch == sSpiInformation.sHwSettings.uluDmaTxChannel) ? (sSpiInformation.sHwSettings.uluDmaTxChannel|UDMA_PRI_SELECT)
                                        : (sSpiInformation.sHwSettings.uluDmaRxChannel|UDMA_PRI_SELECT);

    return(MAP_uDMAChannelModeGet(chanel));
}

//*****************************************************************************
//
//!  SpiIsDmaStop
//!
//!  @param  ch
//!
//!  @return 1 if DMA is stopped, 0 otherwise
//!
//!  @brief  Check DMA status
//
//*****************************************************************************
long
SpiIsDmaStop(long ch)
{
    long mode;
    long enable;

    mode = SpiCheckDMAStatus(ch);
    enable = MAP_uDMAChannelIsEnabled(ch);

    if((mode == UDMA_MODE_STOP) && (!enable))
    {
        return(1);
    }

    return(0);
}

//*****************************************************************************
//
//! SpiFlushRxFifo
//!
//!  \param  none
//!
//!  \return none
//!
//!  \brief  flush RX
//
//*****************************************************************************
void
SpiFlushRxFifo(void)
{
    unsigned long ulIdx;
    while(MAP_SSIDataGetNonBlocking(sSpiInformation.sHwSettings.ulSsiPort, &ulIdx))
    //Clear the RX overrun flag
        SSIIntClear(sSpiInformation.sHwSettings.ulSsiPort, SSI_RIS_RORRIS);
}

//*****************************************************************************
//
//!  SpiFirstWrite
//!
//!  @param  ucBuf buffer to write
//!  @param  ucBuf buffer's length
//!
//!  @return none
//!
//!  @brief  Spi first write operation
//
//*****************************************************************************
long
SpiFirstWrite(unsigned char *ucBuf, unsigned short usLength)
{
    // workaround for first transaction
    ASSERT_CS();

    SpiSysDelay(FWT_DELAY/4);

    // SPI writes first 4 bytes of data
    SpiWriteDataSynchronous(ucBuf, 4);
    while(SSIBusy(SPI_BASE));
    SpiSysDelay(FWT_DELAY/4);

    SpiWriteDataSynchronous(ucBuf + 4, usLength - 4);

    // From this point on - operate in a regular way
    sSpiInformation.ulSpiState = eSPI_STATE_IDLE;
    while(SSIBusy(SPI_BASE));
    DEASSERT_CS();

    return(0);
}



//*****************************************************************************
//
//! SpiWrite
//!
//!  @param  pUserBuffer buffer to write
//!  @param  pUserBuffer buffer's length
//!
//!  @return none
//!
//!  @brief  Spi write operation
//
//*****************************************************************************
long
SpiWrite(unsigned char *pUserBuffer, unsigned short usLength)
{
    unsigned char ucPad = 0;

    // Figure out the total length of the packet in order to figure out if there is padding or not
    if(!(usLength & 0x0001))
    {
        ucPad++;
    }

    pUserBuffer[0] = WRITE;
    pUserBuffer[1] = HI(usLength + ucPad);
    pUserBuffer[2] = LO(usLength + ucPad);
    pUserBuffer[3] = 0;
    pUserBuffer[4] = 0;

    usLength += (SPI_HEADER_SIZE + ucPad);

    // The magic number that resides at the end of the TX/RX buffer (1 byte after the allocated size)
    // for the purpose of detection of the overrun. If the magic number is overwritten - buffer overrun
    // occurred - and we will stuck here forever!
    if (wlan_tx_buffer[CC3000_TX_BUFFER_SIZE - 1] != CC3000_BUFFER_MAGIC_NUMBER)
    {
        while (1)
            ;
    }

    if (sSpiInformation.ulSpiState == eSPI_STATE_POWERUP)
    {
        while (sSpiInformation.ulSpiState != eSPI_STATE_INITIALIZED);
    }

    if (sSpiInformation.ulSpiState == eSPI_STATE_INITIALIZED)
    {

        // This is time for first TX/RX transactions over SPI: the IRQ is down - so need to send read buffer size command
        SpiFirstWrite(pUserBuffer, usLength);

        // Due to the fact that we are currently implementing a blocking situation
        // here we will wait till end of transaction
    }
    else
    {

        // We need to prevent here race that can occur in case 2 back to back packets are sent to the
        // device, so the state will move to IDLE and once again to not IDLE due to IRQ

        while (sSpiInformation.ulSpiState != eSPI_STATE_IDLE)
        {
            ;
        }

        sSpiInformation.ulSpiState = eSPI_STATE_WRITE_IRQ;
        sSpiInformation.pTxPacket = pUserBuffer;
        sSpiInformation.usTxPacketLength = usLength;

        // Assert the CS line and wait till SSI IRQ line is active and then initialize write operation
        ASSERT_CS();
    }

    // Due to the fact that we are currently implementing a blocking situation
    // here we will wait till end of transaction
    while (eSPI_STATE_IDLE != sSpiInformation.ulSpiState)
    {
        ;
    }
    return(0);
}


//*****************************************************************************
//
//! SpiDisableSSIDMAChannels
//!
//!  @param  none
//!
//!  @return none
//!
//!  @brief  This function disables the SSI DMA RX/TX Channels
//
//*****************************************************************************

void
SpiDisableSSIDMAChannels()
{
    MAP_uDMAChannelDisable(sSpiInformation.sHwSettings.uluDmaTxChannel);
    MAP_uDMAChannelDisable(sSpiInformation.sHwSettings.uluDmaRxChannel);
}

//*****************************************************************************
//
//! SpiEnableSSIDMAChannels
//!
//!  @param  none
//!
//!  @return none
//!
//!  @brief  This function enables the SSI DMA Channels
//
//*****************************************************************************

void
SpiEnableSSIDMAChannels()
{
    MAP_uDMAChannelEnable(sSpiInformation.sHwSettings.uluDmaTxChannel);
    MAP_uDMAChannelEnable(sSpiInformation.sHwSettings.uluDmaRxChannel);
}

//*****************************************************************************
//
//! This function enter point for write flow
//!
//!  @param  data  buffer to read to
//!  @param  size  buffer's size
//!
//!  @return none
//!
//!  @brief  Spi read operation
//
//*****************************************************************************
void
SpiReadData(unsigned char *data, unsigned short size)
{

    // The uDMA TX/RX channel must be disabled.
    SpiDisableSSIDMAChannels();

    // Start another DMA transfer to SSI TX.
    MAP_uDMAChannelTransferSet((sSpiInformation.sHwSettings.uluDmaTxChannel|UDMA_PRI_SELECT),
                                                         UDMA_MODE_BASIC,
                                                         (void *)&tSpiReadHeader,
                                                         (void *)(sSpiInformation.sHwSettings.ulSsiPort + SSI_O_DR),
                                                         size );

    // Start another DMA transfer to SSI RX.
    MAP_uDMAChannelTransferSet(sSpiInformation.sHwSettings.uluDmaRxChannel|UDMA_PRI_SELECT,
                                                         UDMA_MODE_BASIC,
                                                         (void *)(sSpiInformation.sHwSettings.ulSsiPort + SSI_O_DR),
                                                         (void *)data,
                                                         size);

    SpiFlushRxFifo();
    MAP_SSIDMAEnable(sSpiInformation.sHwSettings.ulSsiPort, SSI_DMA_TX | SSI_DMA_RX);

    // The uDMA RX channel must be re-enabled.
    // The uDMA TX channel is not required - then it will not be enabled at all
    // (for second part of RX it is required, since the host issues write command)
    MAP_uDMAChannelEnable(sSpiInformation.sHwSettings.uluDmaTxChannel);
    MAP_uDMAChannelEnable(sSpiInformation.sHwSettings.uluDmaRxChannel);

}
//*****************************************************************************
//
//! SpiWriteAsync
//!
//!  @param  data  buffer to write
//!  @param  size  buffer's size
//!
//!  @return none
//!
//!  @brief  Spi write operation
//
//*****************************************************************************
void
SpiWriteAsync(const unsigned char *data, unsigned short size)
{

    // LMS uDMA is capable to transfer data in one shot only of size up to 1024
    // so in case of bigger transfer size will need to perform 2 transfers
    // (scatter-gather uDMA  mode also can be used but it will not be very useful
    // since CPU anyway is blocked on SPI write operation and can not do anything
    // till the end.
    if (size<=SPI_WINDOW_SIZE)
    {

       sSpiInformation.ulSpiState = eSPI_STATE_WRITE_EOT;

        // The uDMA TX/RX channel must be disabled.
        SpiDisableSSIDMAChannels();

        // Start another DMA transfer to SSI TX.
        MAP_uDMAChannelTransferSet((sSpiInformation.sHwSettings.uluDmaTxChannel|UDMA_PRI_SELECT),UDMA_MODE_BASIC,(void *)data,
        (void *)(sSpiInformation.sHwSettings.ulSsiPort + SSI_O_DR), size );
        // Flush out buffer
        SpiFlushRxFifo();

        MAP_SSIDMAEnable(sSpiInformation.sHwSettings.ulSsiPort, SSI_DMA_TX );

        // Send the Data
        MAP_uDMAChannelEnable(sSpiInformation.sHwSettings.uluDmaTxChannel);
    }

    else
    {

        // Send the data over SPI and wait for complete interrupt to transfer the rest
        sSpiInformation.ulSpiState = eSPI_STATE_WRITE_FIRST_PORTION;


        // The uDMA TX/RX channel must be disabled.
        SpiDisableSSIDMAChannels();

        // Start another DMA transfer to SSI TX.
        MAP_uDMAChannelTransferSet(sSpiInformation.sHwSettings.uluDmaTxChannel|UDMA_PRI_SELECT, UDMA_MODE_BASIC, (void *)data,
                               (void *)(sSpiInformation.sHwSettings.ulSsiPort + SSI_O_DR),SPI_WINDOW_SIZE);
      //
      // The uDMA TX channel must be re-enabled.
      MAP_uDMAChannelEnable(sSpiInformation.sHwSettings.uluDmaTxChannel);
      MAP_uDMAChannelEnable(sSpiInformation.sHwSettings.uluDmaRxChannel);
    }

}

//*****************************************************************************
//
//! SpiNewWriteAsync
//!
//!  @param  data  buffer to write
//!  @param  size  buffer's size
//!
//!  @return none
//!
//!  @brief
//
//*****************************************************************************
void
SpiNewWriteAsync(unsigned char *data, unsigned short size)
{

}
//*****************************************************************************
//
//! SpiWriteDataSynchronous
//!
//!  @param  data  buffer to write
//!  @param  size  buffer's size
//!
//!  @return none
//!
//!  @brief  Spi write operation
//
//*****************************************************************************
void
SpiWriteDataSynchronous(const unsigned char *data, unsigned short size)
{
    // we are in the SYNC write to SPI - we do not want to receive ANY SSI related
    // interrupts
    MAP_IntDisable(sSpiInformation.sHwSettings.ulSsiPortInt);
    SpiWriteAsync(data, size);

    // The last one to finish operation is RX channel since
    // TX is finished before the last bit is on the bus XXX was RX
    while(SSIBusy(SPI_BASE));

    // Clear The SSI & DMA Interrupt and enable back SSI
    IntPendClear(INT_SPI);
    IntPendClear(INT_UDMA);

    // Enable back SSI ISR
    MAP_IntEnable(sSpiInformation.sHwSettings.ulSsiPortInt);
}


//*****************************************************************************
//
//! SpiReadHeader
//!
//!  @param  none
//!
//!  @return none
//!
//!  @brief  SPI read operation. first we read minimal 5 SPI header bytes and 5
//!          Event Data bytes
//
//*****************************************************************************
void
SpiReadHeader(void)
{
    sSpiInformation.ulSpiState = eSPI_STATE_READ_IRQ;

    SpiReadData(sSpiInformation.pRxPacket, 10);
}


//*****************************************************************************
//
//! SpiReadDataCont
//!
//!  \param  None
//!
//!  \return None
//!
//!  \brief  This function processes received SPI Header and in accordance with
//!          it - continues reading the packet
//
//*****************************************************************************
long
SpiReadDataCont(void)
{

    long data_to_recv;
    unsigned char *evnt_buff, type;

    //determine what type of packet we have
    evnt_buff =  sSpiInformation.pRxPacket;
    data_to_recv = 0;
    STREAM_TO_UINT8((char *)(evnt_buff + SPI_HEADER_SIZE), HCI_PACKET_TYPE_OFFSET, type);

    switch(type)
    {
    case HCI_TYPE_DATA:
        {
            STREAM_TO_UINT16((char *)(evnt_buff + SPI_HEADER_SIZE), HCI_DATA_LENGTH_OFFSET, data_to_recv);

            if (data_to_recv >= SPI_WINDOW_SIZE)
            {
                data_to_recv = eSPI_STATE_READ_FIRST_PORTION;
                SpiReadData(evnt_buff + 10, SPI_WINDOW_SIZE);
                sSpiInformation.ulSpiState = eSPI_STATE_READ_FIRST_PORTION;
            }
            else
            {
                // We need to read the rest of data..
                if (!((HEADERS_SIZE_EVNT + data_to_recv) & 1))
                {
                    data_to_recv++;
                }

                if (data_to_recv)
                {
                    SpiReadData(evnt_buff + 10, data_to_recv);
                }

                sSpiInformation.ulSpiState = eSPI_STATE_READ_EOT;
            }
            break;
        }
    case HCI_TYPE_EVNT:
        {

            // Calculate the rest length of the data
            STREAM_TO_UINT8((char *)(evnt_buff + SPI_HEADER_SIZE), HCI_EVENT_LENGTH_OFFSET, data_to_recv);
            data_to_recv -= 1;

            // Add padding byte if needed
            if ((HEADERS_SIZE_EVNT + data_to_recv) & 1)
            {

                data_to_recv++;
            }

            if (data_to_recv)
            {
                SpiReadData(evnt_buff + 10, data_to_recv);
            }

            sSpiInformation.ulSpiState = eSPI_STATE_READ_EOT;
            break;
        }
    }

    return (data_to_recv);
}


//*****************************************************************************
//
//! SpiDisableInterrupts
//!
//!  @param  none
//!
//!  @return none
//!
//!  @brief  Disable SPI interrupts
//
//*****************************************************************************

void
SpiDisableInterrupts(void)
{
    MAP_IntDisable(sSpiInformation.sHwSettings.ulSsiPortInt);
    MAP_IntDisable(sSpiInformation.sHwSettings.ulPortInt);
}


//*****************************************************************************
//
//! SpiResumeSpi
//!
//!  @param  none
//!
//!  @return none
//!
//!  @brief  Spi resume operation
//
//*****************************************************************************

void
SpiResumeSpi(void)
{
    MAP_IntEnable(sSpiInformation.sHwSettings.ulSsiPortInt);
    MAP_IntEnable(sSpiInformation.sHwSettings.ulPortInt);
}


//*****************************************************************************
//
//! SpiTriggerRxProcessing
//!
//!  @param  none
//!
//!  @return none
//!
//!  @brief  Processing operation for SPI RX
//
//*****************************************************************************
void
SpiTriggerRxProcessing(void)
{

    // Trigger Rx processing
    SpiDisableInterrupts();
    while(SSIBusy(SPI_BASE));
    DEASSERT_CS();

    // The magic number that resides at the end of the TX/RX buffer (1 byte after the allocated size)
    // for the purpose of detection of the overrun. If the magic number is overwritten - buffer overrun
    // occurred - and we will stuck here forever!
    if (sSpiInformation.pRxPacket[CC3000_RX_BUFFER_SIZE - 1] != CC3000_BUFFER_MAGIC_NUMBER)
    {
        while (1)
            ;
    }

    MAP_SSIDMADisable(sSpiInformation.sHwSettings.ulSsiPort, SSI_DMA_RX);
    sSpiInformation.ulSpiState = eSPI_STATE_IDLE;
    sSpiInformation.SPIRxHandler(sSpiInformation.pRxPacket + SPI_HEADER_SIZE);
}

//*****************************************************************************
//
//! SSIContReadOperationw
//!
//!  @param  none
//!
//!  @return none
//!
//!  @brief  Spi read operation
//
//*****************************************************************************

void
SSIContReadOperation(void)
{

    // The header was read - continue with  the payload read
    if (!SpiReadDataCont())
    {
        // All the data was read - finalize handling by switching to the task
        //  and calling from task Event Handler
        SpiTriggerRxProcessing();
    }
}

//*****************************************************************************
//
//!  IntSpiGPIOHandler
//!
//!  @param  none
//!
//!  @return none
//!
//!  @brief  GPIO A interrupt handler. When the external SSI WLAN device is
//!          ready to interact with Host CPU it generates an interrupt signal.
//!          After that Host CPU has registered this interrupt request
//!          it set the corresponding /CS in active state.
//
//*****************************************************************************
void
IntSpiGPIOHandler(void)
{
    SpiCleanGPIOISR();
    if(!MAP_GPIOPinRead(SPI_GPIO_IRQ_BASE, SPI_IRQ_PIN))
    {
        if (sSpiInformation.ulSpiState == eSPI_STATE_POWERUP)
        {
            // This means IRQ line was low call a callback of HCI Layer to
            // inform on event
            sSpiInformation.ulSpiState = eSPI_STATE_INITIALIZED;
        }
        else if (sSpiInformation.ulSpiState == eSPI_STATE_IDLE)
        {
            sSpiInformation.ulSpiState = eSPI_STATE_READ_IRQ;

            // IRQ line goes down - we are start reception
            ASSERT_CS();

            // Wait for TX/RX Compete which will come as DMA interrupt
            SpiReadHeader();
        }
        else if (sSpiInformation.ulSpiState == eSPI_STATE_WRITE_IRQ)
        {
            SpiWriteAsync(sSpiInformation.pTxPacket, sSpiInformation.usTxPacketLength);
        }
    }
}


//*****************************************************************************
//
//! SpiIntHandler
//!
//! @param  none
//!
//! @return none
//!
//! @brief  SSI interrupt handler. Get command/data packet from the external
//!         WLAN device and pass it through the UART to the PC
//
//*****************************************************************************
void
SpiIntHandler(void)
{
    unsigned long ucTxFinished, ucRxFinished;
    unsigned short data_to_recv;
    unsigned char *evnt_buff;
    ucTxFinished = SpiIsDmaStop(sSpiInformation.sHwSettings.uluDmaTxChannel) ;
    ucRxFinished = SpiIsDmaStop(sSpiInformation.sHwSettings.uluDmaRxChannel) ;
    evnt_buff =  sSpiInformation.pRxPacket;
    data_to_recv = 0;
    if (sSpiInformation.ulSpiState == eSPI_STATE_READ_IRQ)
    {

        // If one of DMA's still did not finished its operation - we need to stay
        // and wait till it will finish
        if (ucTxFinished && ucRxFinished)
        {

            // If SSI Int is pending - this can be second DMA - clean it...
            IntPendClear(sSpiInformation.sHwSettings.ulSsiPortInt);
            SSIContReadOperation();
        }
    }
    else if (sSpiInformation.ulSpiState == eSPI_STATE_READ_FIRST_PORTION)
    {
        if (ucRxFinished)
        {
            // If SSI Int is pending - this can be second DMA - clean it...
            IntPendClear(sSpiInformation.sHwSettings.ulSsiPortInt);


            STREAM_TO_UINT16((char *)(evnt_buff + SPI_HEADER_SIZE), HCI_DATA_LENGTH_OFFSET, data_to_recv);

            // We need to read the rest of data..

            data_to_recv -=SPI_WINDOW_SIZE;


            if (!((HEADERS_SIZE_EVNT + data_to_recv) & 1))
            {
                data_to_recv++;
            }

            SpiReadData(sSpiInformation.pRxPacket + 10 + SPI_WINDOW_SIZE, data_to_recv);

            sSpiInformation.ulSpiState = eSPI_STATE_READ_EOT;
        }
    }
    else if (sSpiInformation.ulSpiState == eSPI_STATE_READ_EOT)
    {
        // All the data was read - finalize handling by switching to the task
        // and calling from task Event Handler
        if (ucRxFinished)
        {

            // If SSI Int is pending - this can be second DMA - clean it...
            IntPendClear(sSpiInformation.sHwSettings.ulSsiPortInt);

            SpiTriggerRxProcessing();
        }
    }
    else if (sSpiInformation.ulSpiState == eSPI_STATE_WRITE_EOT)
    {
        if (ucTxFinished)
        {
            while(SSIBusy(SPI_BASE));
            DEASSERT_CS();
            // Since SSI is full duplex and RX data is irrelevant - flush it
            SpiFlushRxFifo();
            sSpiInformation.ulSpiState = eSPI_STATE_IDLE;
        }
    }
    else if (sSpiInformation.ulSpiState == eSPI_STATE_WRITE_FIRST_PORTION)
    {
        if (ucTxFinished)
        {
            // Since SSI is full duplex and RX data is irrelevant - flush it
            SpiFlushRxFifo();

            sSpiInformation.ulSpiState = eSPI_STATE_WRITE_EOT;

            SpiWriteAsync(sSpiInformation.pTxPacket + SPI_WINDOW_SIZE, sSpiInformation.usTxPacketLength - SPI_WINDOW_SIZE);
        }
    }
}



//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
