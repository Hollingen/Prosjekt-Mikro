/*
 * Prosjekt_Master_C.c
 *
 * Created: 29.04.2021 13:54:08
 * Author : eirik
 */ 

//Includes

#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 16000000UL

#include <util/delay.h>
#include <avr/power.h>
#include <avr/sfr_defs.h>
#include "USART.h"
#include "i2c.h"


//Defines
#define SLAVE_ADDRESS_W			0b00000010
#define SLAVE_ADDRESS_R			0b00000011

volatile uint16_t seconds;
uint16_t lightData;

//Funksjoner

void init_timer(){
	// TCCR1A = 0; CTC normal operation
	TCCR1B = (1<<WGM12)|(1<<CS12)|(1<<CS10);        // CTC, TOP = OCR1A, TOV1 flag at MAX, freq scaling:/1024
	TIMSK1 = (1<<OCIE1A);                            // Interrupt enable when OC1RA compare match
	OCR1A = 15625;                                    // 16Mhz/1024 = 152625, a.k.a 1 second
}

void init_Buttons(){
	
	PORTD = 0b11111111; //Aktiverer pullup resistor på PD2 og PD3. Aktiverer også pullup for å unngå flytende inputs
	DDRD =  0b00000000; //Setter alle til inputs
	PORTC = 0b00000000; //
	DDRC = 0b00000000; //Setter alle til inputs
}

int main(void)
{
	PORTB |= (1 << PORTB0);
	//Inits//
	init_timer(); // Initialize timer 1sec
	sei(); // enables global interrupts
	initI2C();
	init_Buttons();
	//Variabler//
	initUSART();
	uint8_t moistureData, lowLightData, highLightData;
	//uint16_t lightData;
	
	while (1){
		
// 		loop_until_bit_is_clear(PINB, PINB0);
// 		printString("\r\nKnapp\r\n");
// 		i2cStart();
// 		//printString("\r\ni2cStart\r\n");
// 		i2cSend(SLAVE_ADDRESS_R);
// 		printString("\r\nSENDT\r\n");
// 		moistureData = i2cReadAck();
// 		printString("\r\nMoisturedata\r\n");
// 		printByte(moistureData);
// 		lightData = i2cReadNoAck();
// 		printString("\r\nlight\r\n");
// 		printByte(lightData);
// 		
// 		float resistorVoltage = (lightData / 255.0) * 5.0; //spenningen er 5V og verdi mellom 0-1024
// 		float ldrVoltage = 5.0 - resistorVoltage;
// 		float ldrResistance = (ldrVoltage/resistorVoltage) * 47000.0; //referanse resistanse brukt er 10k
// 		float ldrLux = 12518931.0 * pow(ldrResistance,  -1.405); //tallene er skalarer for å få resistansen om til lux.
// 		printString("\r\n");
// 		printWord(ldrLux);
// 		
// 		i2cStop();
// 		_delay_ms(1000);
		if (seconds >= 3600){
			seconds = 0;
			
			i2cStart();
			i2cSend(SLAVE_ADDRESS_R);
			moistureData = i2cReadAck();
			highLightData = i2cReadAck();
			lowLightData = i2cReadNoAck();
			i2cStop();
			
			
			printString("\r\nMeasurement done! New values are ready!\r\n");
			printString("\r\nPress the button to display moisture and lumen\r\n");
			
		}
		if(!(PIND & PIND2)){
			_delay_ms(100);
			if(!(PIND & PIND2)){
				printString("\r\nMoisture: ");
				printByte(moistureData);
				printString("%");
				printString("\r\nLux: ");
				printWord(lightData);
			}
			
		}
	}
}

void combineWord(uint8_t lowByte, uint8_t highByte){
	lightData = (highByte << 8)|lowByte;
}

ISR(TIMER1_COMPA_vect){
	seconds ++;
}
