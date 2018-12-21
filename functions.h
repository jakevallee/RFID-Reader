/*
 * Author: Jacob Vallee
 * Partner: Ben Rancourt
 * Capstone: General Purpose RFID reader
 * 
 * This file contains the function prototypes for the RFID reader code.
 *
 */


#ifndef FUNCTIONS_H
#define FUNCTIONS_H
	
/* Defines */
#define F_CPU		10000000UL			//Define the CPU speed
#define FOSC 		10000000
#define BAUD 		9600
#define MYUBRR		((FOSC/16/BAUD)-1)	//Equation from Datasheet


/* UART Buffer Defines; Code from atmel app notes for UART transmission used for Debugging */
#define UART_RX_BUFFER_SIZE 8     /* 2,4,8,16,32,64,128 or 256 bytes */
#define UART_TX_BUFFER_SIZE 8
#define UART_RX_BUFFER_MASK (UART_RX_BUFFER_SIZE - 1)

#if (UART_RX_BUFFER_SIZE & UART_RX_BUFFER_MASK)
	#error RX buffer size is not a power of 2
#endif

#define UART_TX_BUFFER_MASK (UART_TX_BUFFER_SIZE - 1)
#if (UART_TX_BUFFER_SIZE & UART_TX_BUFFER_MASK)
	#error TX buffer size is not a power of 2
#endif

/* Prototypes */
void USART_Transmit( unsigned char data );
void sendUSART(unsigned char data);
void initUSART(void);
void initComparator(void);
void initPWM(void);
void changeSER(char c);
void writeSER(char c[]);
void initSpeaker(void);
void stopSpeaker(void);
void failSpeaker(void);
void printString(char* string);
void redLED(void);
void greenLED(void);
void ledOFF(void);

/* Atmel Prototypes */
void TransmitByte(unsigned char data);
void InitUART(unsigned char ubrr_val);	
	
	
#endif