/*
	RFID Code
*/

#define F_CPU		10000000UL			//Define the CPU speed
#define FOSC 		10000000
#define BAUD 		9600
#define MYUBRR		((FOSC/16/BAUD)-1)
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

void sendUSART(unsigned char data);
void initUSART(void);
void initComparator(void);
void initPWM(void);
void changeSER(char c);
void writeSER(char c[]);
void initSpeaker(void);
void stopSpeaker(void);
void failSpeaker(void);

int main(void) {

	char rawDat[1500];
	char manchDat[350];
	char curVal;
	char lastVal;
	int orig;
	int i = 0;
	int j = 0;
	int start = 0;
	int sameNum = 0;
	int state = 0;
	int Ostate = 0;

	
	
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
	
	/* Setup Comparator */
	//initComparator();
	
	/* Setup Speaker */
	//initSpeaker();
	
	/* LCD initialization */
	lcd_init(LCD_DISP_ON);
	lcd_puts("RFID Tag Here");
	
	
	
	//while(1) {	//Loop forever

		
		for(i = 0; i < 1500; i++){	//used to fill buffer with data from comparator
			if(PINC & (1<<PC5)){
				rawDat[i] = '1';
			} else {
				rawDat[i] = '0';
			}
			_delay_us(50);
			
		}
		
		writeSER("Collected Data\n\r");
		
		/*
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
		*/
		
		i = 0;

		
		orig = (rawDat[0] == '1');
		for(i=0;i<1500;i++){			//First need to find the first point that the state switches.
			state = (rawDat[i] == '1');	//It does this to avoid the couple of bits that will likely be
			if(orig != state){			//in the beginning. Once the two 'start' is defined the rest
				start = i;				//of the buffer can be processed.
				break;
			}
		}
		
		writeSER("Found starting point\n\r");
		
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

void initUSART(void){
	
	UBRR0H = (MYUBRR>>8);
	UBRR0L =  MYUBRR;
	
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

void initComparator(void) {
	
    // ACSR Info from Datasheet:
    //
    // When this bit is written logic one, the power to the Analog
    // Comparator is switched off. This bit can be set at any time to turn
    // off the Analog Comparator. This will reduce power consumption in
    // Active and Idle mode. When changing the ACD bit, the Analog
    // Comparator Interrupt must be disabled by clearing the ACIE bit in
    // ACSR. Otherwise an interrupt can occur when the bit is changed.
    ACSR &= (0 << ACD);
    // When this bit is set, a fixed bandgap reference voltage replaces the
    // positive input to the Analog Comparator. When this bit is cleared,
    // AIN0 is applied to the positive input of the Analog Comparator. When
    // the bandgap referance is used as input to the Analog Comparator, it
    // will take a certain time for the voltage to stabilize. If not
    // stabilized, the first conversion may give a wrong value.
    ACSR &= (0 << ACBG);
    // When the ACIE bit is written logic one and the I-bit in the Status
    // Register is set, the Analog Comparator interrupt is activated.
    // When written logic zero, the interrupt is disabled.
    ACSR |= (1 << ACIE);
    // When written logic one, this bit enables the input capture function
    // in Timer/Counter1 to be triggered by the Analog Comparator. The
    // comparator output is in this case directly connected to the input
    // capture front-end logic, making the comparator utilize the noise
    // canceler and edge select features of the Timer/Counter1 Input
    // Capture interrupt. When written logic zero, no connection between
    // the Analog Comparator and the input capture function exists. To
    // make the comparator trigger the Timer/Counter1 Input Capture
    // interrupt, the ICIE1 bit in the Timer Interrupt Mask Register
    // (TIMSK1) must be set.
    ACSR &= (0 << ACIC);
    // These bits determine which comparator events that trigger the Analog
    // Comparator interrupt.
    // ACIS1 ACIS0 Mode
    // 0 0 Toggle
    // 0 1 Reserved
    // 1 0 Falling edge
    // 1 1 Rising edge
    ACSR |= (1 << ACIS1);
    ACSR |= (1 << ACIS0);
	
	//
    // DIDR1 settings from the Datasheet: 
    //
    // When this bit is written logic one, the digital input buffer on the
    // AIN1/0 pin is disabled. The corresponding PIN Register bit will
    // always read as zero when this bit is set. When an analog signal is
    // applied to the AIN1/0 pin and the digital input from this pin is not
    // needed, this bit should be written logic one to reduce power
    // consumption in the digital input buffer.
    DIDR1 |= (1 << AIN1D);
    DIDR1 |= (1 << AIN0D); 
	
}