//uart2.h
#ifndef _UART2_H
#define _UART2_H

#define USART2_BAUDRATE  38400UL
#define MMU_F_CPU       16000000UL
#define BAUD_PRESCALE (((MMU_F_CPU / (USART2_BAUDRATE * 16UL))) - 1)
#define BLK                0x2D    // Blank data filler "-"

#include <inttypes.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "mmu.h"
#include "Arduino.h"

extern volatile unsigned char rxData1, rxData2, rxData3, rxCSUM1, rxCSUM2;
extern volatile bool startRxFlag, confirmedPayload, txNAKNext, txACKNext,
       txRESEND, pendingACK;
extern unsigned long startTXTimeout;

extern unsigned char lastTxPayload[3];

extern void uart2_txPayload(unsigned char*);
extern void uart2_txACK(bool ACK = true);

extern void uart2_init(void);

#endif //_UART2_H
