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
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "lcd.h"


/////////////////////////////////////////////////////////////////////////////

char sendBUFFER[SEND_SIZE];
uint8_t wrINDEX;
uint8_t	rdINDEX;


/* Atmel Static Variables */
static unsigned char UART_RxBuf[UART_RX_BUFFER_SIZE];
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

/* Atmel Functions */
void TransmitByte(unsigned char data);
void InitUART(unsigned char ubrr_val);

int main(void) {

	char rawDat[1500];
	char manchDat[350];
	char binDataC[149];
	char tag[64];
	int binDataI[149];
	char curVal;
	char lastVal;
	char first;
	char second;
	int orig;
	int i = 0;
	int j = 0;
	int start = 0;
	int sameNum = 0;
	int state = 0;
	
	
	
	/* Set Port C pin PC5 as input to monitor the output of the comparator.*/
	DDRC &= ~(1 << PC5);			
	
	/* Set Port B pin PB0 to output for Blue LED */
	DDRB |=  (1 << PB0);				
	
	/* Set Port D pin PD3 to output for carrier signal -- OC2B */
	DDRD |=  (1 << PD3);				
	
	/* Set Port D pin PB1 to output for speaker -- OC1B */
	DDRB |=  (1 << PB2);                   
	
	/* Enable Global Interrupt */
	//SREG |= 0x80;
	sei();

	/* Setup USART for communication with raspberry pi */
	initUSART();
	
	/* Setup PWM */
	initPWM();
	
	/* Setup Speaker */
	//initSpeaker();
	
	/* LCD initialization */
	lcd_init(LCD_DISP_ON);
	lcd_puts("RFID Tag Here");
	
	
	
	//while(1) {	//Loop forever
		//while(1){
		//	_delay_ms(1000);
		//	writeSER("PLeaseWork");
		//}
		
		for(i = 0; i < 1500; i++){	//used to fill buffer with data from comparator
			if(PINC & (1<<PC5)){
				rawDat[i] = '1';
			} else {
				rawDat[i] = '0';
			}
			_delay_us(50);
			
		}
		
		_delay_ms(2000);
		
		i = 0;
		orig = (rawDat[0] == '1');
		for(i=0;i<1500;i++){			//First need to find the first point that the state switches.
			state = (rawDat[i] == '1');	//It does this to avoid the couple of bits that will likely be
			if(orig != state){			//in the beginning. Once the two 'start' is defined the rest
				start = i;				//of the buffer can be processed.
				break;
			}
		}
		
		//First print block goes here
		//writeSER("Collected Data\n\n\r");
		//_delay_ms(1000);
		
		
		for(i=0;i<1500;i++){			//Because the USART transmission can only happen so fast
			if(rawDat[i] == '1'){		//This for loop must be used in order to print the values 
				writeSER("1");
			} else if(rawDat[i] == '0'){
				writeSER("0");
			} else {
				writeSER("2");
			}
			_delay_ms(50);
		}
		
		
		j = 0;
		sameNum = 0;		
		
		lastVal = rawDat[start];
		for(i=start;i<(1500-start);i++){
			curVal = rawDat[i];
			if(curVal == lastVal){
				sameNum++;
			} else {	//If the last and current val are different then process the last group
				if(sameNum >= 3 && sameNum <=7){ //Check for how many times the bit repeated. this represents either a single or double bit.
					manchDat[j] = lastVal;
				} else if(sameNum >= 8 && sameNum <= 11){
					manchDat[j] = lastVal;
					j++;
					manchDat[j] = lastVal;
				} else {
					//last bit sequence was either too long or too short.
					//this means probably bad data... recollect.
					break;
				}
				j++;
				sameNum = 1;
			}
			lastVal = curVal;
		}
		
		
		//Second print block goes here
		
		writeSER("\n\n\n\r");
		for(i=0;i<350;i++){				
			if(manchDat[i] == '1'){			
				writeSER("1");
			} else if(manchDat[i] == '0'){
				writeSER("0");
			} else {
				writeSER("2");
			}
			_delay_ms(50);
		}
		
		
		_delay_ms(1000);
		//Next piece of code needs to find the first double bit to use as a reference
		//this is because the state cannot stay the same through an entire cycle. there must be a transition.
		
		first = manchDat[0];
		for(i=1;i<149;i++){
			second = manchDat[i];
			if(first == second){
				start = i;
				break;
			} else {
				second = first;
			}
		}
		
		//The code below converts the manchester code into binary code
		j = 0;
		for(i=start;i<149;i+=2){
			first = manchDat[i];
			second = manchDat[i+1];
			if((first == '1') && (second == '0')){
				binDataC[j] = '1';
				//binDataI[j] = 1;
			} else if((first == '0') && (second == '1')){
				binDataC[j] = '0';
				//binDataI[j] = 0; 
			} else {
				binDataC[j] = '2';
				//binDataI[j] = 2;
			}
			j++;
		}
		
		_delay_ms(1000);
		//writeSER("\n\n\n\r");
		//This block prints the binary code
		/*
		writeSER("Please");
		writeSER("Work");
		_delay_ms(1000);
		
		for(i=0;i<100;i++){				
			if(binDataC[i] == '1'){			
				writeSER("1");
			} else if(binDataC[i] == '0'){
				writeSER("0");
			} else {
				writeSER("2");
			}
			_delay_ms(200);
		}		
		*/
		
		
		//PORTB ^= (1 << PB0);				//toggle the LED
		
	//}
	
	return(0);
}

ISR(USART_TX_vect){
	
	if(rdINDEX != wrINDEX){
		UDR0 = sendBUFFER[rdINDEX];
		rdINDEX++;
		
		if(rdINDEX >= SEND_SIZE){
			rdINDEX = 0;
		}
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

void USART_Transmit( unsigned char data ){	//Function from datasheet
 /* Wait for empty transmit buffer */
 while ( !( UCSR0A & (1<<UDRE0)) );
 
 /* Put data into buffer, sends the data */
 UDR0 = data;
}

void initUSART(void){
	
	UBRR0H = (unsigned char)(MYUBRR>>8);
	UBRR0L =  (unsigned char)MYUBRR;
	
	UCSR0B = (1<<TXEN0)|(1<<TXCIE0);
	UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
	
}

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
		UDR = UART_TxBuf[tmptail];
	} else {
		/* Disable UDRE interrupt */
		UCSRB &= ~(1<<UDRIE);        
	}
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
	UCSRB |= (1<<UDRIE);
}
