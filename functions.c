/*
 * Author: Jacob Vallee
 * Partner: Ben Rancourt
 * Capstone: General Purpose RFID reader
 * 
 * This file contains the function definitions for the RFID reader code.
 *
 */
 
/* Defines */
#define F_CPU		10000000UL			//Define the CPU speed
#define FOSC 		10000000
#define BAUD 		9600
#define MYUBRR		((FOSC/16/BAUD)-1)	//Equation from Datasheet
 
/* Includes */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "lcd.h"
#include "functions.h"


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

/* Atmel Static Variables */
static volatile unsigned char UART_RxHead;
static volatile unsigned char UART_RxTail;
static unsigned char UART_TxBuf[UART_TX_BUFFER_SIZE];
static volatile unsigned char UART_TxHead;
static volatile unsigned char UART_TxTail;


/* Turns off both LEDs */
void ledOFF(void){
	PORTB &= (0 << PB0);
	PORTB &= (0 << PB1);
}

/* Turns on the green LED */
void greenLED(void){
	PORTB |= (1 << PB0);
}

/* Turns on the red LED */
void redLED(void){
	PORTB |= (1 << PB1);	
}

/* Used for sending a string to laptop via UART (debugging) */
void printString(char* string){
	
	while(*string != 0x00){			//Becasue the string has a null terminating character just look for that
		TransmitByte(*string);
		string++;					//and increment through the pointer here. 
	}
	
}

/* This function enables the 125kHz PWM carrier signal */
void initPWM(void){
	
	//////////////////////////
	//	Setup PWM on PD3,	//
	//	50% Duty Cycle,  	//
	//	125kHz,				//
	//////////////////////////
	
	TCCR2A |= (1<<COM2A1);	//Set Non-inverting mode
	TCCR2A |= (1<<COM2B1);	//Set Non-inverting mode
	TCCR2A |= (1<<WGM21);	//Fast PWM with top 0xFF
	TCCR2A |= (1<<WGM20);	//'' 
	
	
	TCCR2B |= (1<<WGM22);
	TCCR2B |= (1<<CS20);
	OCR2A = 79;				//output compare, 50% duty cycle. 
	OCR2B = 39;	
	
}

/* This function enables the PWM used for a tone for a successful tag read */
void initSpeaker(void) {
	
	
	//////////////////////////
	//	Setup PWM on PB2,	//
	//						//
	//////////////////////////
	
	TCCR1A |= (1<<COM1A1);	//Set Non-inverting mode
	TCCR1A |= (1<<COM1B1);	//Set Non-inverting mode
	
	TCCR1A |= (1<<WGM11);	//Fast PWM with top 0x03FF
	TCCR1A |= (1<<WGM10);	//'' 
	TCCR1B |= (1<<WGM12);	//''
	
	TCCR1B |= (1<<CS11);	//Sets prescalar (clk/8) and starts PWM
	
	OCR1B = 100;			//Used to set Duty Cycle
	
}

/* This code enables a PWM for the speaker at a lower frequency to give a slightly lower tone */
void failSpeaker(void){
	
	//////////////////////////
	//	Setup PWM on PB2,	//
	//	play a tone for 	//
	//	unsuccessful read	//
	//////////////////////////
	
	TCCR1A |= (1<<COM1A1);			//Set Non-inverting mode
	TCCR1A |= (1<<COM1B1);			//Set Non-inverting mode
	
	TCCR1A |= (1<<WGM11);			//Fast PWM with top 0x03FF
	TCCR1A |= (1<<WGM10);			//'' 
	TCCR1B |= (1<<WGM12);			//''
	
	TCCR1B |= (1<<CS11)|(1<<CS10);	//Sets prescalar (clk/64) and starts PWM
	
	//ICR1 = 1500;
	OCR1B = 100;					//Used to set Duty Cycle	
	
}

/* Used to disable PWM that drives the speaker */
void stopSpeaker(void){
	TCCR1B &= (0<<CS21)|(0<<CS11)|(0<<CS10);	//disables PWM
}

///////////////////////////////////////////////////////////////
/* 
 * Atmel Functions Below
 * Code from Atmel application notes for the atmega328
 * Used for UART while debugging, not used in final code 
 */

ISR(USART_UDRE_vect){
	unsigned char tmptail;

	/* Check if all data is transmitted */
	if (UART_TxHead != UART_TxTail) {
		/* Calculate buffer index */
		tmptail = ( UART_TxTail + 1 ) & UART_TX_BUFFER_MASK;
		/* Store new index */
		UART_TxTail = tmptail;      
		/* Start transmission */
		UDR0 = UART_TxBuf[tmptail];
	} else {
		/* Disable UDRE interrupt */
		UCSR0B &= ~(1<<UDRIE0);        
	}
}

void InitUART(unsigned char ubrr_val)
{
	unsigned char x;

	/* Set the baud rate */
	UBRR0H = (unsigned char)(ubrr_val>>8);
	UBRR0L = (unsigned char)ubrr_val;
	/* Enable UART receiver and transmitter */
	UCSR0B = ((1<<RXEN0) | (1<<TXEN0) | (1<<RXCIE0));

	/* Flush receive buffer */
	x = 0; 			    

	UART_RxTail = x;
	UART_RxHead = x;
	UART_TxTail = x;
	UART_TxHead = x;
}

void TransmitByte(unsigned char data){
	unsigned char tmphead;
	
	/* Calculate buffer index */
	tmphead = (UART_TxHead + 1) & UART_TX_BUFFER_MASK;
	/* Wait for free space in buffer */
	while (tmphead == UART_TxTail);
	/* Store data in buffer */
	UART_TxBuf[tmphead] = data;
	/* Store new index */
	UART_TxHead = tmphead;
	/* Enable UDRE interrupt */
	UCSR0B |= (1<<UDRIE0);
}