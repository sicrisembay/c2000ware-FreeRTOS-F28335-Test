/*
 * main.c
 *
 *  Created on: 9 Dec 2023
 *      Author: Sicris
 */

#include "FreeRTOS.h"
#include "task.h"

extern unsigned int RamfuncsLoadStart;
extern unsigned int RamfuncsLoadEnd;
extern unsigned int RamfuncsRunStart;


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


static void blinkyTask(void * pvParam)
{
    TickType_t xLastWakeTime;

    EALLOW;
    SysCtrlRegs.PCLKCR3.bit.GPIOINENCLK = 1;
    GpioCtrlRegs.GPBPUD.bit.GPIO34 = 1;
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;
    GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;
    EDIS;

    xLastWakeTime = xTaskGetTickCount();

    while(1) {
        /* ON LED */
        GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;
        vTaskDelayUntil(&xLastWakeTime, 500);
        /* OFF LED */
        GpioDataRegs.GPBSET.bit.GPIO34 = 1;
        vTaskDelayUntil(&xLastWakeTime, 1500);
    }
}


static TaskHandle_t blinkyTaskHandle = NULL;

void main()
{
    /*
     * Copy ramfuncs section
     */
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart,
            &RamfuncsLoadEnd - &RamfuncsLoadStart);

    /* Configure Core Frequency */
    portDISABLE_INTERRUPTS();
    IER = 0x0000;
    IFR = 0x0000;

    /* Enable the PIE Vector Table */
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;

    InitFlashWaitState();
    configure_core_pll(0xA);


    xTaskCreate(blinkyTask, "blinky", 512, NULL, 1, &blinkyTaskHandle);
    vTaskStartScheduler();

    /* Should not reach here */
    while(1);
}
