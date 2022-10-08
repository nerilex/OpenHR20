/*
 * uart_config.h
 *
 *  Created on: 10.02.2022
 *      Author: bg
 */

#ifndef SRC_UART_CONFIG_H_
#define SRC_UART_CONFIG_H_

#include <avr/io.h>
#include "avr_uart_defs.h"

#ifndef UART_NI
#define UART_NI 0
#endif

#if UART_NI == 0
#define UART0_I 1
#else
#define UART0_NI 1
#endif

#ifndef UART0_BAUD_RATE
#define UART0_BAUD_RATE MODBUS_BAUD_RATE
#endif
#define UART0_PARATY UART_PARATY_NONE
#define UART0_STOPBITS UART_STOPBITS_1
#define UART0_DATABITS UART_DATABITS_8
#define UART0_RXBUFFER_SIZE 32
#define UART0_TXBUFFER_SIZE 128
#define UART0_SWFLOWCTRL 0
#define UART0_THRESH_LOW 0
#define UART0_THRESH_HIGH 10
#define UART0_HOOK 1
#define UART0_HOOK_PEEK 1

#define UART0_TX_INDICATOR 1
#define UART0_TX_INDICATOR_PIN 2
#define UART0_TX_INDICATOR_PORT PORTE
#define UART0_TX_INDICATOR_DDR  DDRE
#define UART0_TX_INDICATOR_ACTIVE_HIGH 1


#endif /* SRC_UART_CONFIG_H_ */
