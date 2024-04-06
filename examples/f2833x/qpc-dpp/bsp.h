/*
 * bsp.h
 *
 *  Created on: 20 Dec 2023
 *      Author: Sicris
 */

#ifndef BSP_H_
#define BSP_H_

#define BSP_TICKS_PER_SEC    configTICK_RATE_HZ

void BSP_init(void);
void BSP_displayPaused(uint8_t paused);
void BSP_displayPhilStat(uint8_t n, char const *stat);
void BSP_terminate(int16_t result);

void BSP_randomSeed(uint32_t seed);   // random seed
uint32_t BSP_random(void);            // pseudo-random generator

#endif /* BSP_H_ */
