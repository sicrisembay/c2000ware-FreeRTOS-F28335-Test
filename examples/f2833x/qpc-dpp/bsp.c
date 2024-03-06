/*
 * bsp.c
 *
 *  Created on: 20 Dec 2023
 *      Author: Sicris
 */

#include "autoconf.h"
#include "qpc.h"
#include "dpp.h"
#include "bsp.h"

Q_DEFINE_THIS_FILE

static uint32_t l_rnd; // random seed

#if CONFIG_QPC_QSPY_ENABLE

enum AppRecords {
    PHILO_STAT = QS_USER,
    PAUSED_STAT,
    COMMAND_STAT
};
#endif /* CONFIG_QPC_QSPY_ENABLE */

void BSP_init(void)
{
    BSP_randomSeed(1234U);
    if(QS_INIT((void*)0) == 0U) {
        Q_ERROR();
    }
    extern uint16_t const l_TickHook;
    QS_OBJ_DICTIONARY(&l_TickHook);
    QS_USR_DICTIONARY(PHILO_STAT);
    QS_USR_DICTIONARY(PAUSED_STAT);
    QS_USR_DICTIONARY(COMMAND_STAT);
}

void BSP_displayPaused(uint8_t paused)
{
    QS_BEGIN_ID(PAUSED_STAT, 0U)
        QS_U8(1, paused);
    QS_END()
}

void BSP_displayPhilStat(uint8_t n, char const *stat)
{
    QS_BEGIN_ID(PHILO_STAT, AO_Philo[n]->prio)
        QS_U8(1, n);
        QS_STR(stat);
    QS_END()
}

void BSP_terminate(int16_t result)
{
    (void)result; // unused
}

void BSP_randomSeed(uint32_t seed)
{
    l_rnd = seed;
}

uint32_t BSP_random(void)
{
    // a very cheap pseudo-random-number generator
    // Some floating point code is to exercise the VFP...
    float volatile x = 3.1415926F;
    x = x + 2.7182818F;

    vTaskSuspendAll(); // lock FreeRTOS scheduler
    // "Super-Duper" Linear Congruential Generator (LCG)
    // LCG(2^32, 3*7*11*13*23, 0, seed)
    //
    l_rnd = l_rnd * (3U*7U*11U*13U*23U);
    xTaskResumeAll(); // unlock the FreeRTOS scheduler

    return l_rnd >> 8;
}
