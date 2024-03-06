/*
 * main.c
 *
 *  Created on: 9 Dec 2023
 *      Author: Sicris
 */

#include "autoconf.h"
#include "string.h"
#include "qpc.h"
#include "dpp.h"
#include "bsp.h"

extern unsigned int RamfuncsLoadStart;
extern unsigned int RamfuncsLoadEnd;
extern unsigned int RamfuncsRunStart;
extern unsigned int EbssStart;
extern unsigned int EbssEnd;

/*
 * small size pool
 */
static QF_MPOOL_EL(QEvt) smallPoolSto[CONFIG_QPC_SMALL_MEMPOOL_ENTRY_COUNT];

/*
 * medium size pool
 */
typedef struct {
    QEvt super;
    uint8_t data[CONFIG_QPC_MEDIUM_MEMPOOL_ENTRY_SIZE];
} medPool;
static QF_MPOOL_EL(medPool) medPoolSto[CONFIG_QPC_MEDIUM_MEMPOOL_ENTRY_COUNT];

/*
 * large size pool
 */
typedef struct {
    QEvt super;
    uint8_t data[CONFIG_QPC_LARGE_MEMPOOL_ENTRY_SIZE];
} largePool;
static QF_MPOOL_EL(largePool) largePoolSto[CONFIG_QPC_LARGE_MEMPOOL_ENTRY_COUNT];

/*
 * Storage for Publish-Subscribe
 */
static QSubscrList subscrSto[MAX_PUB_SIG];

static void configure_core_pll(uint16_t val)
{
    /* Make sure the PLL is not running in limp mode */
    if (SysCtrlRegs.PLLSTS.bit.MCLKSTS != 0)
    {
       /*
        * Missing external clock has been detected
        * Replace this line with a call to an appropriate
        * SystemShutdown(); function.
        */
       asm("        ESTOP0");
    }

    /* Change the PLLCR */
    if (SysCtrlRegs.PLLCR.bit.DIV != val)
    {

       EALLOW;
       /* Before setting PLLCR turn off missing clock detect logic */
       SysCtrlRegs.PLLSTS.bit.MCLKOFF = 1;
       SysCtrlRegs.PLLCR.bit.DIV = val;
       EDIS;

       /*
        * Wait for PLL to lock.
        * During this time the CPU will switch to OSCCLK/2 until
        * the PLL is stable.  Once the PLL is stable the CPU will
        *
        * Wait for the PLL lock bit to be set.
        */
       while(SysCtrlRegs.PLLSTS.bit.PLLLOCKS == 0) {
           /*
            * Note: The watchdog should be fed within
            * the loop via ServiceDog().
            */
       }

       EALLOW;
       SysCtrlRegs.PLLSTS.bit.MCLKOFF = 0;
       EDIS;
     }
}


#pragma CODE_SECTION(InitFlashWaitState, "ramfuncs");
static void InitFlashWaitState(void)
{
    EALLOW;

    //
    // Enable Flash Pipeline mode to improve performance
    // of code executed from Flash.
    //
    FlashRegs.FOPT.bit.ENPIPE = 1;

    //
    //                CAUTION
    // Minimum waitstates required for the flash operating
    // at a given CPU rate must be characterized by TI.
    // Refer to the datasheet for the latest information.
    //

    //
    // Set the Paged Waitstate for the Flash
    //
    FlashRegs.FBANKWAIT.bit.PAGEWAIT = 5;

    //
    // Set the Random Waitstate for the Flash
    //
    FlashRegs.FBANKWAIT.bit.RANDWAIT = 5;

    //
    // Set the Waitstate for the OTP
    //
    FlashRegs.FOTPWAIT.bit.OTPWAIT = 8;
    //
    //                CAUTION
    // ONLY THE DEFAULT VALUE FOR THESE 2 REGISTERS SHOULD BE USED
    //
    FlashRegs.FSTDBYWAIT.bit.STDBYWAIT = 0x01FF;
    FlashRegs.FACTIVEWAIT.bit.ACTIVEWAIT = 0x01FF;

    EDIS;

    //
    // Force a pipeline flush to ensure that the write to
    // the last register configured occurs before returning.
    //
    asm(" RPT #7 || NOP");
}


interrupt void DefualtISR(void)
{
    asm ("      ESTOP0");
    for(;;);
}


void InitPieCtrl(void)
{
    //
    // Disable Interrupts at the CPU level
    //
    DINT;

    //
    // Disable the PIE
    //
    PieCtrlRegs.PIECTRL.bit.ENPIE = 0;

    //
    // Clear all PIEIER registers
    //
    PieCtrlRegs.PIEIER1.all = 0;
    PieCtrlRegs.PIEIER2.all = 0;
    PieCtrlRegs.PIEIER3.all = 0;
    PieCtrlRegs.PIEIER4.all = 0;
    PieCtrlRegs.PIEIER5.all = 0;
    PieCtrlRegs.PIEIER6.all = 0;
    PieCtrlRegs.PIEIER7.all = 0;
    PieCtrlRegs.PIEIER8.all = 0;
    PieCtrlRegs.PIEIER9.all = 0;
    PieCtrlRegs.PIEIER10.all = 0;
    PieCtrlRegs.PIEIER11.all = 0;
    PieCtrlRegs.PIEIER12.all = 0;

    //
    // Clear all PIEIFR registers
    //
    PieCtrlRegs.PIEIFR1.all = 0;
    PieCtrlRegs.PIEIFR2.all = 0;
    PieCtrlRegs.PIEIFR3.all = 0;
    PieCtrlRegs.PIEIFR4.all = 0;
    PieCtrlRegs.PIEIFR5.all = 0;
    PieCtrlRegs.PIEIFR6.all = 0;
    PieCtrlRegs.PIEIFR7.all = 0;
    PieCtrlRegs.PIEIFR8.all = 0;
    PieCtrlRegs.PIEIFR9.all = 0;
    PieCtrlRegs.PIEIFR10.all = 0;
    PieCtrlRegs.PIEIFR11.all = 0;
    PieCtrlRegs.PIEIFR12.all = 0;

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    volatile Uint32 *dest = (void *)&PieVectTable;
    EALLOW;
    for (uint16_t i = 0; i < 128; i++) {
        *dest++ = (Uint32)&DefualtISR;
    }
    EDIS;

    /* Enable the PIE Vector Table */
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;
}


void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleTaskStackBuffer,
                                    uint32_t *pulIdleTaskStackSize )
{
    /*
     * If the buffers to be provided to the Idle task are declared inside this
     * function then they must be declared static - otherwise they will be
     * allocated on the stack and so not exists after this function exits.
     */
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

    /*
     * Pass out a pointer to the StaticTask_t structure in which the Idle
     * task's state will be stored.
     */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task's stack. */
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /*
     * Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
     * Note that, as the array is necessarily of type StackType_t,
     * configMINIMAL_STACK_SIZE is specified in words, not bytes.
     */
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

static QEvt const * tableQueueSto[N_PHILO];
static QEvt const * philoQueueSto[N_PHILO][N_PHILO];
static StackType_t philoStack[N_PHILO][configMINIMAL_STACK_SIZE];
static StackType_t tableStack[configMINIMAL_STACK_SIZE];

void main()
{
    InitPieCtrl();

    /*
     * Zero ebss section
     */
    memset(&EbssStart, 0, &EbssEnd - &EbssStart);

    /*
     * Copy ramfuncs section
     */
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart,
            &RamfuncsLoadEnd - &RamfuncsLoadStart);

    InitFlashWaitState();
    /* Configure Core Frequency */
    configure_core_pll(0xA);

    /* Initialize QF framework */
    QF_init();
    BSP_init();

    /* Initialize Event Pool
     * Note: QF can manage up to three event pools (e.g., small, medium, and large events).
     * An application may call this function up to three times to initialize up to three event
     * pools in QF.  The subsequent calls to QF_poolInit() function must be made with
     * progressively increasing values of the evtSize parameter.
     */
    QF_poolInit(smallPoolSto, sizeof(smallPoolSto), sizeof(smallPoolSto[0]));
    QF_poolInit(medPoolSto, sizeof(medPoolSto), sizeof(medPoolSto[0]));
    QF_poolInit(largePoolSto, sizeof(largePoolSto), sizeof(largePoolSto[0]));
    QF_psInit(subscrSto, Q_DIM(subscrSto));

#ifdef Q_SPY
    produce_sig_dict();
#endif
    for (uint16_t i = 0; i < N_PHILO; i++) {
        Philo_ctor(i);
        QActive_setAttr(AO_Philo[i], TASK_NAME_ATTR, "Philo");
        QACTIVE_START(AO_Philo[i],
                      (QPrioSpec)(i+1),
                      philoQueueSto[i],
                      Q_DIM(philoQueueSto[i]),
                      philoStack[i],
                      configMINIMAL_STACK_SIZE,
                      (QEvt *)0);
    }

    Table_ctor();
    QActive_setAttr(AO_Table, TASK_NAME_ATTR, "Table");
    QACTIVE_START(AO_Table,
                  (QPrioSpec)(N_PHILO + 1),
                  tableQueueSto,
                  Q_DIM(tableQueueSto),
                  tableStack,
                  configMINIMAL_STACK_SIZE,
                  (QEvt *)0);


    QF_run();

    /* Should not reach here */
    while(1);
}
