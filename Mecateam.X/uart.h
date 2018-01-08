/* 
 * File:   uart.h
 * Author: Mérili
 *
 * Created on 1 de Janeiro de 2011, 01:39
 */

#ifndef UART_H
#define	UART_H

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

#endif	/* UART_H */

extern volatile unsigned char uartBufferRx[127];
extern volatile unsigned char uartConter;

extern volatile unsigned char uartBufferZigbeeRx[2][127];
extern volatile unsigned char uartZigbeeConter;
extern volatile unsigned char uartPilhaZigbee;

extern unsigned char uart1ConterRx;
extern unsigned char uart1ConterTx;


extern unsigned char uartBufferTx[127];
extern unsigned char uartBufferTxSize;

#define Interface_USB 0
#define Interface_Zigbee 1



