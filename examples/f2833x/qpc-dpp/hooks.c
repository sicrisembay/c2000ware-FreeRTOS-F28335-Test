/*
 * hooks.c
 *
 *  Created on: 20 Dec 2023
 *      Author: Sicris
 */

#include "autoconf.h"
#include "qpc.h"

uint16_t const l_TickHook = 0;

#if CONFIG_QPC_QSPY_ENABLE
#include "BSP_uart.h"
static uint8_t qsTxBuf[CONFIG_QSPY_TX_BUFFER_SIZE];
static uint8_t qsRxBuf[CONFIG_QSPY_RX_BUFFER_SIZE];
#endif /* CONFIG_QPC_QSPY_ENABLE */

void vApplicationTickHook(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    QTIMEEVT_TICK_FROM_ISR(0U, &xHigherPriorityTaskWoken, &l_TickHook);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void vApplicationIdleHook(void)
{
}

void QF_onStartup(void)
{
}

void QF_onCleanup(void)
{
}

void Q_onAssert(char const *module, int loc)
{
    while(1) {
        /// TODO
    }
}

#if CONFIG_QPC_QSPY_ENABLE
#define QSPY_RX_NOTIFY_TIMEOUT_MS   (2)
#define QSPY_RX_NOTIFY_TIMEOUT      (QSPY_RX_NOTIFY_TIMEOUT_MS / portTICK_PERIOD_MS)

static StaticTask_t xQspyWorkerTCB;
static StackType_t xQspyWorkerStackSto[CONFIG_QSPY_WORKER_STACK_SIZE];
static TaskHandle_t xQspyWorkerTaskHandle = NULL;

static void QS_workerTask(void * pvParam)
{
    uint8_t const * pBlock;
    uint16_t txLen;
    uint16_t rxLen;
    uint8_t rxBuf[16];

    while(1) {
        rxLen = BSP_UART_receive(rxBuf, sizeof(rxBuf), QSPY_RX_NOTIFY_TIMEOUT);
        if(rxLen > 0) {
            for(uint16_t i = 0; i < rxLen; i++) {
                QS_RX_PUT((rxBuf[i] & 0x00FF));
            }
            QS_rxParse();
        }
        txLen = BSP_UART_TX_FIFO_SIZE;
        taskENTER_CRITICAL();
        pBlock = QS_getBlock(&txLen);
        taskEXIT_CRITICAL();
        if(txLen > 0) {
            BSP_UART_send(pBlock, txLen);
        }
    }
}
uint8_t QS_onStartup(void const *arg)
{
    (void)arg;  // unused parameter
    QS_initBuf(qsTxBuf, sizeof(qsTxBuf));
    QS_rxInitBuf(qsRxBuf, sizeof(qsRxBuf));
    BSP_UART_init();

    if(NULL == xQspyWorkerTaskHandle) {
        xQspyWorkerTaskHandle = xTaskCreateStatic(
                QS_workerTask,
                "QSpyWorker",
                CONFIG_QSPY_WORKER_STACK_SIZE,
                (void *)0,
                CONFIG_QPSY_WORKER_TASK_PRIORITY,
                xQspyWorkerStackSto,
                &xQspyWorkerTCB);
    }
    return (uint8_t)1; /* return success */
}


void QS_onCleanup(void)
{
    /// TODO
}


QSTimeCtr QS_onGetTime(void)
{
    /* Use freeRTOS tick count */
    return(xTaskGetTickCount());
}


void QS_onFlush(void)
{
    /* QS buffer flushing to HW peripheral */
    uint8_t const * pBlock;
    uint16_t txLen;

    while(1) {
        txLen = BSP_UART_TX_FIFO_SIZE;
        taskENTER_CRITICAL();
        pBlock = QS_getBlock(&txLen);
        taskEXIT_CRITICAL();
        if(txLen == 0) {
            break;
        }
        (void)BSP_UART_send(pBlock, txLen);
        while(!BSP_UART_sendBufferEmpty());
    }

}


void QS_onReset(void)
{
    /* Tickle dog */
    EALLOW;
    SysCtrlRegs.WDKEY = 0x0055;
    SysCtrlRegs.WDKEY = 0x00AA;
    EDIS;

    /* Enable watchdog */
    EALLOW;
    SysCtrlRegs.WDCR = 0x0000;  /* writing value other than b101 to WDCHK will immediately resets the device */
    EDIS;

    /* Should not reach here */
    while(1);
}


void QS_onCommand(uint8_t cmdId,
                  uint32_t param1, uint32_t param2, uint32_t param3)
{
    (void)cmdId;
    (void)param1;
    (void)param2;
    (void)param3;
}

#endif /* CONFIG_QPC_QSPY_ENABLE */

