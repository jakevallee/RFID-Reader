/*
	RFID Code
*/

#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "lcd.h"

#define F_CPU	10000000U			//Define the CPU speed

/////////////////////////////////////////////////////////////////////////////

//extern int rawDat[1000];

void initComparator(void);
void initPWM(void);

int main(void) {
	
	char rawDat[1000];
	int i = 0;
	int curVal = 0;
	int bitlen = 0;
	int state = 0;
	
	DDRC &= ~(1 << PC5);			//Set Port C pin PC5 as input to monitor the output of the comparator.
	DDRB |= (1 << PB0);				//Set Port B pin PB0 to output for Blue LED
	DDRD |= (1 << PD3);				//Set Port D pin PD3 to output(PWM, Timer/Counter 2)
	SREG |= 0x80;					//Global interrupt enable in Status Register

	
	//Setup PWM
	initPWM;
	
	//Setup Comparator
	//initComparator();
	
	//LCD init
	lcd_init(LCD_DISP_ON);
	lcd_puts("1234567890ABCDEF\nGHIJKLMOPQRSTUVW");
	

	
	while(1) {						//Loop forever
	
		for(i = 0; i < 1000; i++){
			curVal = (PINC & (1<<PC5));
			if(
			_delay_us(25);
		}
	
	
		PORTB ^= (1 << PB0);		//toggle the LED
		_delay_ms(1000);				//Wait
	}
	return(0);
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