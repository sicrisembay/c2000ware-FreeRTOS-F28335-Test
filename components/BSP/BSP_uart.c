/*
 * BSP_uart.c
 *
 *  Created on: 15 Dec 2023
 *      Author: Sicris
 */

#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "BSP_uart.h"

#define UART_CLKIN_FREQ_HZ                  (configCPU_CLOCK_HZ / 4U)
#define UART_BAUDRATE                       (115200)
#define UART_BRR                            ((UART_CLKIN_FREQ_HZ / (UART_BAUDRATE * 8)) - 1)
#define UART_TX_STREAM_BUFFER_SIZE_BYTES    (1024)
#define UART_RX_STREAM_BUFFER_SIZE_BYTES    (1024)
#define UART_FIFO_SZ                        (16)

static bool bInit = false;
static StreamBufferHandle_t txStreamBufHandle = NULL;
static StaticStreamBuffer_t txStreamBufStruct;
static uint8_t txStreamBufSto[UART_TX_STREAM_BUFFER_SIZE_BYTES + 1];
static StreamBufferHandle_t rxStreamBufHandle = NULL;
static StaticStreamBuffer_t rxStreamBufStruct;
static uint8_t rxStreamBufSto[UART_RX_STREAM_BUFFER_SIZE_BYTES + 1];

#pragma CODE_SECTION(BSP_UART_tx_isr, "ramfuncs")
interrupt void BSP_UART_tx_isr(void)
{
    uint8_t data[UART_FIFO_SZ];
    size_t count;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint16_t fifoTxCnt = SciaRegs.SCIFFTX.bit.TXFFST;
    uint16_t fifoTxFree = UART_FIFO_SZ - fifoTxCnt;

    /* Acknowledge Interrupt Group */
    PieCtrlRegs.PIEACK.bit.ACK9 = 1;

    count = xStreamBufferReceiveFromISR(txStreamBufHandle,
                                        (void *)data,
                                        fifoTxFree,
                                        &xHigherPriorityTaskWoken);
    if((count == 0) && (fifoTxCnt == 0)) {
        /* Nothing to transmit.  Disable TX FIFO interrupt */
        SciaRegs.SCIFFTX.bit.TXFFIENA = 0;
    } else {
        if(count > 0) {
            for(uint16_t i = 0; i < count; i++) {
                SciaRegs.SCITXBUF = data[i] & 0x00FF;
            }
        }
    }

    /* Clear Interrupt flag */
    SciaRegs.SCIFFTX.bit.TXFFINTCLR = 1;

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

#pragma CODE_SECTION(BSP_UART_rx_isr, "ramfuncs")
interrupt void BSP_UART_rx_isr(void)
{
    uint8_t data[UART_FIFO_SZ];
    size_t nWritten;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint16_t rxCount = SciaRegs.SCIFFRX.bit.RXFFST;

    /* Acknowledge Interrupt Group */
    PieCtrlRegs.PIEACK.bit.ACK9 = 1;

    if(rxCount > 0) {
        for(uint16_t i = 0; i < rxCount; i++) {
            data[i] = SciaRegs.SCIRXBUF.bit.RXDT;
        }
        nWritten = xStreamBufferSendFromISR(rxStreamBufHandle,
                                            (void *)data,
                                            rxCount,
                                            &xHigherPriorityTaskWoken);
        if(nWritten != rxCount) {
            /* Was not able to write all bytes to stream buffer */
            /// TODO: Handle this.
        }
    }

    /* Clear Interrupt Flag */
    SciaRegs.SCIFFRX.bit.RXFFOVRCLR = 1;
    SciaRegs.SCIFFRX.bit.RXFFINTCLR = 1;

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


void BSP_UART_init(void)
{
    if(!bInit) {
        txStreamBufHandle = xStreamBufferCreateStatic(UART_TX_STREAM_BUFFER_SIZE_BYTES,
                                                      1,
                                                      txStreamBufSto,
                                                      &txStreamBufStruct);
        rxStreamBufHandle = xStreamBufferCreateStatic(UART_RX_STREAM_BUFFER_SIZE_BYTES,
                                                      1,
                                                      rxStreamBufSto,
                                                      &rxStreamBufStruct);
        EALLOW;
        SysCtrlRegs.PCLKCR0.bit.SCIAENCLK = 1;
        GpioCtrlRegs.GPAPUD.bit.GPIO29 = 0;
        GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1;
        GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0;
        GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3;
        GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1;
        EDIS;

        /*
         * SCI and FIFO Initialization
         */
        SciaRegs.SCICCR.all = 0x0007;
        SciaRegs.SCICTL1.all = 0x0003;
        SciaRegs.SCICTL2.bit.TXINTENA = 1;
        SciaRegs.SCICTL2.bit.RXBKINTENA = 1;
        SciaRegs.SCIHBAUD = (UART_BRR >> 8) & 0x00FF;
        SciaRegs.SCILBAUD = UART_BRR & 0x00FF;
        SciaRegs.SCIFFTX.all = 0xC000;
        SciaRegs.SCIFFRX.all = 0x0021;
        SciaRegs.SCIFFCT.all = 0x0000;
        /* Release from reset and Enable FIFO */
        SciaRegs.SCICTL1.bit.SWRESET = 1;
        SciaRegs.SCIFFTX.bit.TXFIFOXRESET = 1;
        SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;

        EALLOW;
        PieVectTable.SCITXINTA = BSP_UART_tx_isr;
        PieVectTable.SCIRXINTA = BSP_UART_rx_isr;
        EDIS;
        /* Enable Interrupt */
        PieCtrlRegs.PIEIER9.bit.INTx1 = 1;
        PieCtrlRegs.PIEIER9.bit.INTx2 = 1;
        IER |= M_INT9;

        bInit = true;
    }
}

bool BSP_UART_init_done(void)
{
    return true;
}

static inline uint16_t UART_send(uint8_t * pBuf, uint16_t size, bool bFromISR)
{
    uint16_t j = 0;
    bool fifoFirst = false;
    uint16_t fifoStatus;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if((pBuf == NULL) || (size == 0) || (bInit == false)) {
        return 0;
    }

    /*
     * Disable Tx interrupt to prevent race condition while
     * setting up the buffer
     */
    SciaRegs.SCIFFTX.bit.TXFFIENA = 0;

    fifoStatus = SciaRegs.SCIFFTX.bit.TXFFST;

    if((fifoStatus == 0) && (xStreamBufferIsEmpty(txStreamBufHandle))) {
        /* fill uart FIFO first before the stream buffer */
        fifoFirst = true;
        j = 0;
    }
    for(uint16_t i = 0; i < size; i++) {
        if(fifoFirst && (j < UART_FIFO_SZ)) {
            SciaRegs.SCITXBUF = pBuf[i];
            j++;
        } else {
            if(bFromISR) {
                if(1 != xStreamBufferSendFromISR(txStreamBufHandle, &(pBuf[i]), 1, &xHigherPriorityTaskWoken)) {
                    break;
                }
            } else {
                if(1 != xStreamBufferSend(txStreamBufHandle, &(pBuf[i]), 1, 0)) {
                    break;
                }
            }
        }
    }
    /* Enable Tx interrupt */
    SciaRegs.SCIFFTX.bit.TXFFIENA = 1;

    if(bFromISR) {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    return 0;
}

uint16_t BSP_UART_send(uint8_t * pBuf, uint16_t size)
{
    return UART_send(pBuf, size, false);
}

uint16_t BSP_UART_sendFromISR(uint8_t * pBuf, uint16_t size)
{
    return UART_send(pBuf, size, true);
}

static inline uint16_t UART_receive(uint8_t * pBuf, uint16_t size, bool bFromISR)
{
    size_t count = 0;

    if(bFromISR) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        count = xStreamBufferReceiveFromISR(rxStreamBufHandle,
                                            (void *)pBuf,
                                            size,
                                            &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    } else {
        count = xStreamBufferReceive(rxStreamBufHandle, (void *)pBuf, size, 0);
    }

    return count;
}

uint16_t BSP_UART_receive(uint8_t * pBuf, uint16_t size)
{
    return UART_receive(pBuf, size, false);
}

uint16_t BSP_UART_receiveFromISR(uint8_t * pBuf, uint16_t size)
{
    return UART_receive(pBuf, size, true);
}



