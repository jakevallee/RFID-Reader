/*
	RFID Code
*/

#define F_CPU		10000000UL			//Define the CPU speed
#define FOSC 		10000000
#define BAUD 		9600
#define MYUBRR		((FOSC/16/BAUD)-1)	//Equation from Datasheet
#define SEND_SIZE	100					//Size of serial transmission 

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "lcd.h"

/////////////////////////////////////////////////////////////////////////////
// More Atmel Defines ///////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

/* UART Buffer Defines */
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


/////////////////////////////////////////////////////////////////////////////

char sendBUFFER[SEND_SIZE];
uint8_t wrINDEX;
uint8_t	rdINDEX;


/* Atmel Static Variables */
static volatile unsigned char UART_RxHead;
static volatile unsigned char UART_RxTail;
static unsigned char UART_TxBuf[UART_TX_BUFFER_SIZE];
static volatile unsigned char UART_TxHead;
static volatile unsigned char UART_TxTail;



/* My Functions */
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

/* Atmel Functions */
void TransmitByte(unsigned char data);
void InitUART(unsigned char ubrr_val);

int main(void) {

	// char rawDat[1500];
	// char manchDat[350];
	// char binDataC[149];
	// char printTag[10];
	// char curVal;
	// char lastVal;
	// int reset = 0;
	// int orig = 0;
	// int i = 0;
	// int j = 0;
	// int start = 0;
	// int sameNum = 0;
	// int state = 0;
	// int parityCheck = 0;
	// int parityBit = 0;
	// int error = 0;
	// long long finalTag = 0;
	
	
	/* Set Port C pin PC5 as input to monitor the output of the comparator.*/
	DDRC &= ~(1 << PC5);			
	
	/* Set Port B pin PB0 to output for Blue LED */
	DDRB |=  (1 << PB0);				
	DDRB |=  (1 << PB1);
	
	/* Set Port D pin PD3 to output for carrier signal -- OC2B */
	DDRD |=  (1 << PD3);				
	
	/* Set Port D pin PB1 to output for speaker -- OC1B */
	DDRB |=  (1 << PB2);                   
	
	/* Enable Global Interrupt */
	//SREG |= 0x80;
	sei();

	/* Setup USART for communication with raspberry pi */
	//initUSART();
	InitUART(MYUBRR);
	
	/* Setup PWM */
	initPWM();
	
	/* Setup Speaker */
	//initSpeaker();
	
	/* LCD initialization */
	lcd_init(LCD_DISP_ON);
	lcd_puts("RFID Tag");
	
	while(1){
		PORTB ^= (1 << PB1);
		_delay_ms(1000);
		PORTB ^= (1 << PB0);
		// _delay_ms(5000);		
	}
	
	
	return(0);
}
void ledOFF(void){
	PORTB &= (0 << PB0);
	
}

void redLED(void){
	PORTB |= (1 << PB0);
}

void greenLED(void){
	PORTB |= (1 << PB1);	
}

/*
ISR(USART_TX_vect){
	
	if(rdINDEX != wrINDEX){
		UDR0 = sendBUFFER[rdINDEX];
		rdINDEX++;
		
		if(rdINDEX >= SEND_SIZE){
			rdINDEX = 0;
		}
	}
}
*/
void printString(char* string){
	
	while(*string != 0x00){			//Becasue the string has a null terminating character just look for that
		TransmitByte(*string);
		string++;					//and increment through the pointer here. 
	}
	
}

void changeSER(char c){
	
	sendBUFFER[wrINDEX] = c;
	wrINDEX++;
	
	if(wrINDEX >= SEND_SIZE){
		wrINDEX = 0;
	}
	
}

void writeSER(char c[]){
	uint8_t i = 0;
	
	
	for(i = 0; i < strlen(c); i++){
		changeSER(c[i]);
	}
	
	if(UCSR0A & (1<<UDRE0)){
		UDR0 = 0;
	}
	
}

/*
void USART_Transmit( unsigned char data ){	//Function from datasheet
 // Wait for empty transmit buffer 
 while ( !( UCSR0A & (1<<UDRE0)) );
 
 // Put data into buffer, sends the data 
 UDR0 = data;
}
*/

/*
void initUSART(void){
	
	UBRR0H = (unsigned char)(MYUBRR>>8);
	UBRR0L =  (unsigned char)MYUBRR;
	
	UCSR0B = (1<<TXEN0)|(1<<TXCIE0);
	UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
	
}
*/

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
	
	//PORTB |= (1 << PB0);
	//PORTB |= (1 << PB1);	
}

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
	
	//ICR1 = 1500;
	OCR1B = 100;			//Used to set Duty Cycle
	
}

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
	OCR1B = 100;			//Used to set Duty Cycle	
	
}

void stopSpeaker(void){
	
	TCCR1B &= (0<<CS21)|(0<<CS11)|(0<<CS10);	//disables PWM
	
}
///////////////////////////////////////////////////////////////
/* Atmel Functions Below */

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
