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

	char rawDat[1500];
	char manchDat[350];
	char binDataC[149];
	char printTag[10];
	char test[10] = "\n\n\n\r";
	char curVal;
	char lastVal;
	//int reset = 0;
	int orig = 0;
	int i = 0;
	int j = 0;
	int start = 0;
	int sameNum = 0;
	int state = 0;
	int parityCheck = 0;
	int parityBit = 0;
	int error = 0;
	long long finalTag = 0;
	
	
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
	_delay_ms(100);

	
	while(1) {	//Loop forever
		orig = 0;
		//reset = 0;
		i = 0;
		j = 0;
		start = 0;
		sameNum = 0;
		state = 0;
		parityCheck = 0;
		parityBit = 0;
		error = 0;
		finalTag = 0;		
		// initSpeaker();
		// _delay_ms(100);
		// stopSpeaker();
		

		
		for(i = 0; i < 1500; i++){	//used to fill buffer with data from comparator
			if(PINC & (1<<PC5)){
				rawDat[i] = '1';
			} else {
				rawDat[i] = '0';
			}
			_delay_us(50);
			
		}
		
		i = 0;
		orig = (rawDat[0] == '1');
		for(i=0;i<1500;i++){			//First need to find the first point that the state switches.
			state = (rawDat[i] == '1');	//It does this to avoid the couple of bits that will likely be
			if(orig != state){			//in the beginning. Once the two 'start' is defined the rest
				start = i;				//of the buffer can be processed.
				break;
			}
		}
		
		// First print block goes here
		
		
		// for(i=0;i<1500;i++){			//Updated UART; above code is outdated and will be moved later
			// TransmitByte(rawDat[i]);
			// _delay_ms(20);
		// }
		
		
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
					// last bit sequence was either too long or too short.
					// this means probably bad data... recollect.
					break;
				}
				j++;
				sameNum = 1;
			}
			lastVal = curVal;
		}
		
		// Second print block goes here
		// for(i=0;i<10;i++){
			// TransmitByte(test[i]);
		// }
		// for(i=0;i<350;i++){				
			// TransmitByte(manchDat[i]);
			// _delay_ms(10);
		// }
		
		// Next piece of code needs to find the first double bit to use as a reference
		// this is because the state cannot stay the same through an entire cycle. there must be a transition.
		
		for(i=0;i<350;i++){
			
			if(manchDat[i] == manchDat[i+1]){
				start = i+1;
				break;
			} else {}
		}
		
		// The code below converts the manchester code into binary code
		j = 0;
		for(i=start;i<350;i+=2){
			
			if(manchDat[i] == '0' && manchDat[i+1] != '0'){
				binDataC[j] = '1';
				j++;
			} else if(manchDat[i] != '0' && manchDat[i+1] == '0'){
				binDataC[j] = '0';
				j++;
			} else {
				break;
			}
			
		}
		
		// This block prints the binary code

		// for(i=0;i<10;i++){
			// TransmitByte(test[i]);
		// }
		// for(i=0;i<149;i++){				
			// TransmitByte(binDataC[i]);
			// _delay_ms(10);
		// }		
		
		// Next piece of code will find the 9 bit start sequence 
		// and will store the following 55 bits into the "TAG" buffer.
		sameNum = 0; //reset sameNum variable
		start = 0;
		for(i=0;i<147;i++){
			if(sameNum == 8){
				start = i+1;
				break;
			} else if ((binDataC[i] == '1') && (binDataC[i+1] == '1')){
				sameNum++;
			} else {
				sameNum = 0;
			}

		}
		
		// if((sameNum == 0) || (start <= 7)){
			// reset = 1;		
		// }
		
		// if((reset = 1)){
			// continue;
		// }
		

		/* //Used this code to confirm the value of start
		for(i=0;i<100;i++){
			if(i != start){
				PORTB |= (1 << PB0);
				_delay_ms(500);
				PORTB &= (0 << PB0);
				_delay_ms(500);
			} else {
				break;
			}
		}
		*/
		
		// Now the code will store the 55 bit Tag into the tag buffer
		j=0;
		for(i=start;i<(start+55);i++){
			if(binDataC[i] == '1'){
				binDataC[j] = '1';
				j++;
			} else {
				binDataC[j] = '0';
				j++;
			}
		}
		
		//This block prints the 55bit Tag
		// for(i=0;i<10;i++){
			// TransmitByte(test[i]);
		// }		
		// for(i=0;i<55;i++){				
			// TransmitByte(binDataC[i]);
			// _delay_ms(10);
		// }		
		
		// redLED();
		// _delay_ms(500);
		// ledOFF();
		// _delay_ms(500);

		
		// The next block processes the tag data and checks for a valid tag using the parity bits
		j=0;
		error = 0;
		for(i=0;i<50;i+=5){
			parityCheck = 0;
			parityBit = 0;
			if(binDataC[i] == '1'){
				parityCheck++;
			}
			if(binDataC[i+1] == '1') {
				parityCheck++;
			}  
			if(binDataC[i+2] == '1') {
				parityCheck++;
			}  
			if(binDataC[i+3] == '1') {
				parityCheck++;
			} 
			
			if(binDataC[i+4] == '1'){
				parityBit = 1;
			} else {
				parityBit = 0;
			}
			
			j++;
			if(parityCheck%2 == 0 && parityBit == 1){
				error = 1;
				break;
			} else if((parityCheck == 1 || parityCheck == 3) && parityBit == 0){
				error = 1;
				break;
			}
		}
		
		// redLED();
		// _delay_ms(1000);
		// ledOFF();
		// _delay_ms(1000);
		
		if(error == 0){					//Will tidy this up with a loop later.
			binDataC[0] = binDataC[10];
			binDataC[1] = binDataC[11];
			binDataC[2] = binDataC[12];
			binDataC[3] = binDataC[13];
			binDataC[4] = binDataC[15];
			binDataC[5] = binDataC[16];
			binDataC[6] = binDataC[17];
			binDataC[7] = binDataC[18];
			binDataC[8] = binDataC[20];
			binDataC[9] = binDataC[21];
			binDataC[10] = binDataC[22];
			binDataC[11] = binDataC[23];
			binDataC[12] = binDataC[25];
			binDataC[13] = binDataC[26];
			binDataC[14] = binDataC[27];
			binDataC[15] = binDataC[28];
			binDataC[16] = binDataC[30];
			binDataC[17] = binDataC[31];
			binDataC[18] = binDataC[32];
			binDataC[19] = binDataC[33];
			binDataC[20] = binDataC[35];
			binDataC[21] = binDataC[36];
			binDataC[22] = binDataC[37];
			binDataC[23] = binDataC[38];
			binDataC[24] = binDataC[40];
			binDataC[25] = binDataC[41];
			binDataC[26] = binDataC[42];
			binDataC[27] = binDataC[43];
			binDataC[28] = binDataC[45];
			binDataC[29] = binDataC[46];
			binDataC[30] = binDataC[47];
			binDataC[31] = binDataC[48];
			binDataC[32] = '\0';
			
			// If made it to this point there is a valid tag.
			
			// Now just the 8H section of the tag is stored in binDataC[0-31]
			// This data can now be converted from binary to decimal and displayed. 
			//sscanf(binDataC, "%lld", &binTag);

			//Prints Final Binary Tag
			// for(i=0;i<32;i++){				
				// TransmitByte(binDataC[i]);
				// _delay_ms(10);
			// }
			
			// PORTB |= (1 << PB1);
			// _delay_ms(1000);
			// PORTB &= (0 << PB1);			
			
			i=0;
			while(binDataC[i] != '\0'){
			
				if(binDataC[i] == '1'){
					finalTag |= 1;
				}
				i++;
				if(binDataC[i] != '\0'){
					finalTag <<= 1;
				}
			}			
			
			if(finalTag == 0){
				goto end;
			}
			
			ltoa(finalTag, printTag, 10);
			//snprintf(printTag, "%lld", finalTag);
			//sprintf(printTag, "%lli", finalTag);
			
			// printTag[10] = '\0';
			
			// for(i=0;i<32;i++){				
				// TransmitByte(binDataC[i]);
				// _delay_ms(10);
			// }		
			
			//printTag[14] = '\0';
			lcd_command(LCD_HOME);
			initSpeaker();
			lcd_puts(printTag);
			_delay_ms(100);
			stopSpeaker();
			_delay_ms(1500);
			lcd_command(LCD_CLR);			
			lcd_command(LCD_HOME);
			lcd_puts("\n");
			lcd_puts(printTag);
			for(i=0;i<33;i++){
				binDataC[i] = '0';
			}
			
		} else {

		}
		end:;
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
