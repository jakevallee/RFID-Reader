/*
 * Author: Jacob Vallee
 * Partner: Ben Rancourt
 * Capstone: General Purpose RFID reader
 * 
 * This file contains the main function code for the RFID reader.
 *
 */

#define F_CPU		10000000UL			//Define the CPU speed
#define FOSC 		10000000
#define BAUD 		9600
#define MYUBRR		((FOSC/16/BAUD)-1)	//Equation from Datasheet

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

/////////////////////////////////////////////////////////////////////////////
///////////////////// Atmel Defines /////////////////////////////////////////
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


int main(void) {

	char rawDat[1500];
	char manchDat[350];
	char binDataC[149];
	char printTag[10];
	char curVal;
	char lastVal;
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
	//char test[10] = "\n\n\n\r";
	//int reset = 0;
	
	
	/* Set Port C pin PC5 as input to monitor the output of the comparator.*/
	DDRC &= ~(1 << PC5);			
	
	/* Set Port B pin PB0 and PB1 to output for LEDs */
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
	
	/* Setup PWM for carrier signal */
	initPWM();
	
	/* LCD initialization */
	lcd_init(LCD_DISP_ON);
	lcd_puts("RFID Tag:\n");
	
	_delay_ms(100);
	
	while(1) {	//Loop forever
		orig = 0;
		i = 0;
		j = 0;
		start = 0;
		sameNum = 0;
		state = 0;
		parityCheck = 0;
		parityBit = 0;
		error = 0;
		finalTag = 0;		
		
		
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
		// for(i=0;i<1500;i++){		
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
		// this is because the state cannot stay the same through an entire cycle. There must be a transition.
		
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
		
		if(error == 0){					//This code reuses the binDataC variable and stores only the 32 bit tag followed by a NULL character.
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

			//Prints Final Binary Tag
			// for(i=0;i<32;i++){				
				// TransmitByte(binDataC[i]);
				// _delay_ms(10);
			// }
			
			
			//The code below converts the binary char data into a decimal value
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
			
			//Dont print a tag of all 0s
			if(finalTag == 0){
				continue;
			}
			
			//This converts the decimal value into a char array that can be printed
			ltoa(finalTag, printTag, 10);
			
			// for(i=0;i<32;i++){				
				// TransmitByte(binDataC[i]);
				// _delay_ms(10);
			// }		
			
			initSpeaker();
			redLED();
			lcd_command(LCD_HOME);
			lcd_puts("RFID Tag:\n");			
			lcd_puts(printTag);
			_delay_ms(100);
			stopSpeaker();
			ledOFF();
			_delay_ms(1500);

			
			
			//Clear all of the variables.
			//This needs to be done in order to prevent 
			//the same tag from continuously being read.
			for(i=0;i<1500;i++){
				rawDat[i] = '0';
			}
			for(i=0;i<350;i++){
				manchDat[i] = '0';
			}
			for(i=0;i<149;i++){
				binDataC[i] = '0';
			}
			for(i=0;i<10;i++){
				printTag[i] = '0';
			}
			
		} else {

		}
		
	} 
	return(0);
}
