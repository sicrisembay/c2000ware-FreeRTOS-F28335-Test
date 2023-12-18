/*
 * BSP_uart.h
 *
 *  Created on: 15 Dec 2023
 *      Author: Sicris
 */

#ifndef COMPONENTS_BSP_BSP_UART_H_
#define COMPONENTS_BSP_BSP_UART_H_

#include "stdbool.h"
#include "portable.h"

void BSP_UART_init(void);
bool BSP_UART_init_done(void);
uint16_t BSP_UART_send(uint8_t * pBuf, uint16_t size);
uint16_t BSP_UART_sendFromISR(uint8_t * pBuf, uint16_t size);
uint16_t BSP_UART_receive(uint8_t * pBuf, uint16_t size, TickType_t xTicksToWait);
uint16_t BSP_UART_receiveFromISR(uint8_t * pBuf, uint16_t size);

#endif /* COMPONENTS_BSP_BSP_UART_H_ */
