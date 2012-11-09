/* Copyright 2012 Miguel Moreto
 *
 * This file is part of Moreto Nixie Clock firmware.
 *
 * Moreto Nixie Clock firmware is free software: you can redistribute 
 * it and/or modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 * 
 * Moreto Nixie Clock firmware is distributed in the hope that it will 
 * be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License 
 * along with Moreto Nixie Clock firmware. If not, see http://www.gnu.org/licenses/.
 */

/*
 * NixieClockV1.c
 *
 * Main program file for Moreto Nixie Clock
 * https://code.google.com/p/moreto-nixie-clock/
 *
 * Created: 08/02/2012 20:22:37
 *  Author: Miguel Moreto
 *
 * Software PWM adapted from Smart LED Prototypes (http://todbot.com/blog/2007/03/25/smart-led-prototypes/).
 * DHT22 sensor code adapted from http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&p=974797
 */ 


#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include "i2cmaster.h"
#include "clock.h"
#include "RGBled.h"
#include "DHT22.h"
#include "NRF24L01/radio.h"


#define DS1307		0xD0 	// I2C address of DS1307 11010000. Last zero is the R/W bit. 0x70
#define PCF8574_1	0x70	// I2C address of PCF8574 I/O expander 1. 01000000 0x70
#define PCF8574_2	0x72	// I2C address of PCF8574 I/O expander 2. 01000010 0x72
#define PCF8574_3	0x74	// I2C address of PCF8574 I/O expander 3. 01000100 0x74
#define XTAL		8000000L    // Crystal frequency in Hz
#define TIMER_FREQ	10			// timer1 frequency in Hz
#define MODE_CHANGE_SECONDS	5
#define SLEEP_SECONDS 300	// How many seconds it takes to sleep without PIR interrupt

// DS1307 control register squarewave out definitions
#define CR	0x07				// Control register address.
#define SQW_OUT_1 0x80			// SQW/OUT = 1
#define SQW_OUT_0 0x00			// SQW/OUT = 0 (default)
#define SQW_OUT_1Hz 0x10		// SQW/OUT = 1Hz wave
#define SQW_OUT_4096Hz 0x11		// SQW/OUT = 4.096kHz wave
#define SQW_OUT_8192Hz 0x12		// SQW/OUT = 8.192kHz wave
#define SQW_OUT_32768Hz 0x13	// SQW/OUT = 32.768kHz wave
// DS1307 addresses
#define ADDR_SECS 0x00
#define ADDR_MINS 0x01
#define ADDR_HOURS 0x02
#define ADDR_DAYS 0x04
#define ADDR_MONTHS 0x05
#define ADDR_YEAR 0x06

// Pin and masks definitions:
#define KEY_MASK 0x0F	// Mask used to read keys connected in a PCF8574 I/O expander. 0x0F, only four keys.
#define RGB_LED_MIN 0x00	// Pin number of the Common cathode RGB led conected at PCF8574_3
#define RGB_LED_HOUR 0x01
#define RGB_LED_SYMB 0x02
#define NIXIE_DEGREE 0x04
#define NIXIE_PERCENT 0x05
#define NIXIE_m 0x06
#define NIXIE_M 0x07

#define NEON1 PB6
#define NEON2 PB7
#define SW1 PD5
#define SW2 PD6
#define SW3 PD7
#define POWER_ON_PIN PD4

// Software PWM definitions:
#define RED PC0
#define GREEN PC1
#define BLUE PC2
#define CHMAX 3 // maximum number of PWM channels
#define PWMDEFAULT 0x00 // default PWM value at start up for all channels
#define PWM_PORT_MASK  (1 << RED)|(1 << GREEN)|(1 << BLUE) // PWM pin Mask
#define LED_PORT PORTC
#define LED_DDR DDRC

// NRF24L01 definitions:
volatile uint8_t rxflag = 0; 
uint8_t station_addr[5] = { 0xE4, 0xE4, 0xE4, 0xE4, 0xE4 }; // Receiver address
uint8_t trans_addr[5] = { 0x98, 0x76, 0x54, 0x32, 0x10 };	// Transmitter address
RADIO_RX_STATUS rx_status;
radiopacket_t packet;


// Some macros that make the code more readable
#define output_low(port,pin) port &= ~(1<<pin)
#define output_high(port,pin) port |= (1<<pin)
#define set_input(portdir,pin) portdir &= ~(1<<pin)
#define set_output(portdir,pin) portdir |= (1<<pin)
#define toggle(port,pin) port ^= (1<<pin)

void delay_ms(uint16_t ms);

#define FALSE       0
#define TRUE        1

signed int temperature = 100;
unsigned int humidity = 0;

// Global variables:
volatile unsigned char int0_flag = 0;
volatile unsigned char int1_flag = 0;
volatile unsigned char key_pressed = 0;
volatile unsigned char timer1A_flag = 0;
volatile unsigned char change_mode_flag = 0;
volatile unsigned char leds_on = (1<<RGB_LED_HOUR)|(1<<RGB_LED_MIN)|(1<<RGB_LED_SYMB);	// RGB and symbolic nixies status (value to be written in PCF8574_3).

// Seconds counter to enter in sleep mode.
volatile uint16_t sleep_seconds_counter = 0;

unsigned char compare[CHMAX];
volatile unsigned char compbuff[CHMAX];


// External interrupt 0 ISR (fired by the RTC clock square wave output):
ISR(INT0_vect) 
{ 
	int0_flag = 1;	// Set the flag (the processing is done in main function).
}

// External interrupt 1 ISR (fired by PIR module, wakeup from sleep):
// *** ISR(INT1_vect) 
// *** { 
// *** 	sleep_seconds_counter = 0;
// *** }

// External interrupt Pin Change 2 (PCINT 16 to 23, push-buttons).
ISR(PCINT2_vect)
{
	unsigned char keys;
	keys = (~PIND) & ((1<<SW1)|(1<<SW2)|(1<<SW3));
	if (keys > 0)
	{
		//output_high(PORTB,NEON2);
		key_pressed = keys;
	}
	else
	{
		//output_low(PORTB,NEON2);
		key_pressed = 0;
	}
	sleep_seconds_counter = 0;	// Reset time to sleep counter.
	
}


ISR(TIMER0_OVF_vect) // Update RGB Led pins
{
	static unsigned char softcount=0xFF;
	
	//PORTB = pinlevelB;		// update outputs
  
	if(++softcount == 0)	// increment modulo 256 counter and update the compare values only when counter = 0.
							// One period has passed.
	{         
		compare[0] = compbuff[0];
		compare[1] = compbuff[1];
		compare[2] = compbuff[2];
		LED_PORT |= PWM_PORT_MASK;	// set all rgb led pins high
	}
	// clear port pin on compare match (executed on next interrupt)
	if(compare[0] == softcount)
	{
		LED_PORT &= ~(1 << RED);	// Red LED turn off.
	}
	if(compare[1] == softcount)
	{
		LED_PORT &= ~(1 << GREEN);	// Green LED turn off.
	}		
	if(compare[2] == softcount)
	{
		LED_PORT &= ~(1 << BLUE);	// Blue LED turn off.
	}
}

ISR (TIMER1_COMPA_vect)
{
	timer1A_flag = 1;
}

// Function to write one byte in a PCF8574 given the 8bit address and value:
void PCF8574_write(uint8_t addr, uint8_t val)
{
	i2c_start_wait(addr+I2C_WRITE);
	i2c_write(val);
	i2c_stop();		// release bus.	
}

// Function to read one byte in a PCF8574 given the 8bit address and value:
uint8_t PCF8574_read(uint8_t addr)
{
	uint8_t val;
	i2c_start_wait(addr+I2C_READ);
	val = i2c_readNak();
	i2c_stop();
	return ~val;	// Return the inverse of read (PCF8574 has internal pullups).
}

void readtime_DS1307(my_time_t *my_time)
{
        // Reads the clock registers at the DS1307 RTC and store the digits
        // at the my_time_t structure.
        uint8_t hours;
        
        i2c_start_wait(DS1307+I2C_WRITE);       // set device address and write mode
        i2c_write(0x00);                                        // write address = 0
        i2c_rep_start(DS1307+I2C_READ); // set device address and read mode
        my_time->sec = i2c_readAck();
        my_time->min = i2c_readAck();
        hours = i2c_readNak();
        i2c_stop();

        my_time->hour = calculate_hour_DS1307(hours);
}

void read_date_DS1307(my_time_t *my_time)
{
        // Reads the date registers at the DS1307 RTC and store the digits
        // at the my_time_t structure.
        
        i2c_start_wait(DS1307+I2C_WRITE);       // set device address and write mode
        i2c_write(0x03);                                        // write address = 3
        i2c_rep_start(DS1307+I2C_READ); // set device address and read mode
        my_time->weekday = i2c_readAck();
        my_time->day = i2c_readAck();
        my_time->month = i2c_readAck();
        my_time->year = i2c_readNak();
        i2c_stop();
}

void DS1307_write_byte(uint8_t addr, uint8_t byte)
{
        i2c_start_wait(DS1307+I2C_WRITE);       // set device address and write mode
        i2c_write(addr);                                        // write address
        i2c_write(byte);
        i2c_stop();
}

// Convert Decimal to Binary Coded Decimal (BCD) in 16 bits
//uint8_t dec2bcd16(uint16_t num)
//{
	//uint16_t result = 0;
	//result += (num/1000) << 12;
	//result += (num%1000)/100 << 8;
	//result 
  //return ((num/10 * 16) + (num % 10));
//}

// Convert Decimal to Binary Coded Decimal (BCD)
uint8_t dec2bcd(uint8_t num)
{
  return ((num/10 * 16) + (num % 10));
}
// Convert Binary Coded Decimal (BCD) to Decimal
uint8_t bcd2dec(uint8_t num)
{
  return ((num/16 * 10) + (num % 16));
}

uint8_t inc_BCD_2digits(uint8_t BCD_value, uint8_t top_value)
{
	uint8_t value;
	value = bcd2dec(BCD_value);
	value++;
	if (value >= top_value)
	{
		return 0;
	}
	return dec2bcd(value);
}

/* Function to increment or decrement a field in my_time_t structure.
 * 
 * addr: DS1307 address of the parameter being incremented or decremented
 */
void inc_dec_time_DS1307(my_time_t *my_time, uint8_t addr)
{
	uint8_t value_to_write = 0;
	switch (addr)
	{
		case ADDR_SECS:
			break;
		case ADDR_MINS:
			value_to_write = inc_BCD_2digits(my_time->min,60);
			break;
		case ADDR_HOURS:
			value_to_write = inc_BCD_2digits(my_time->hour,24); // Set in 24h mode only.
			break;
		case ADDR_DAYS:
			value_to_write = inc_BCD_2digits(my_time->day,32);
			break;
		case ADDR_MONTHS:
			value_to_write = inc_BCD_2digits(my_time->month,12);
			break;
		case ADDR_YEAR:
			value_to_write = inc_BCD_2digits(my_time->year,100);
			break;
		default:
			return;
			break;
	}
	DS1307_write_byte(addr,value_to_write);
}

// Update the number shown by nixies accordingly with the selected mode.
void update_nixies(my_time_t *my_time, display_mode current_mode)
{
	switch (current_mode)
	{
		case MIN_SEC:
			toggle(PORTB,NEON1);
			PCF8574_write(PCF8574_1,my_time->min);
			PCF8574_write(PCF8574_2,my_time->sec);
			PCF8574_write(PCF8574_3,leds_on);
			break;
		case HOUR_MIN:
			toggle(PORTB,NEON1);
			PCF8574_write(PCF8574_1,my_time->hour);
			PCF8574_write(PCF8574_2,my_time->min);
			PCF8574_write(PCF8574_3,leds_on);
			break;
		case MONTH_DAY:        
			output_high(PORTB,NEON1);
			PCF8574_write(PCF8574_1,my_time->day);
			PCF8574_write(PCF8574_2,my_time->month);
			PCF8574_write(PCF8574_3,leds_on);
			break;
		case YEAR:
			output_low(PORTB,NEON1);
			PCF8574_write(PCF8574_1,0x20);
			PCF8574_write(PCF8574_2,my_time->year);
			PCF8574_write(PCF8574_3,leds_on);
			break;
		case TEMP:
			output_high(PORTB,NEON1);
			PCF8574_write(PCF8574_1,dec2bcd(my_time->temp_digit));
			PCF8574_write(PCF8574_2,dec2bcd(my_time->temp_decimal));
            PCF8574_write(PCF8574_3,leds_on|(1<<RGB_LED_SYMB)|(1<<NIXIE_DEGREE));
			break;
		case HUMID:
			output_high(PORTB,NEON1);
			PCF8574_write(PCF8574_1,dec2bcd(my_time->humid_digit));
			PCF8574_write(PCF8574_2,dec2bcd(my_time->humid_decimal));
			PCF8574_write(PCF8574_3,leds_on|(1<<RGB_LED_SYMB)|(1<<NIXIE_PERCENT));
			break;
		default:
			//output_low(PORTB, LED_SYMB);
			/* Your code here */
			break;
	}
}



int main(void)
{
	//PCF8574_write(PCF8574_3,0x00);	// Turn off the symbolic nixie.
	// Variables:
	my_time_t clock1;
	DHT22_DATA_t sensor_data;
	
	int8_t temp_int;
	uint8_t temp_dec;
	uint8_t hum_int;
	uint8_t hum_dec;
	
	display_mode current_mode = TEMP;
	set_digit current_adjust = OFF;
	uint8_t adjust_addr = 0;	// DS1307 setting values address variable.
	color led_color;
    uint8_t hue = 0;
	hset(hue,&led_color);

	clock1.temp_decimal = 0;
	clock1.temp_digit = 0;
	clock1.humid_decimal = 0;
	clock1.humid_digit = 0;

	uint8_t mode_change_seconds_counter = 0;
	display_mode modes_cycle[3] = {TEMP, HUMID, HOUR_MIN};
	uint8_t mode_cycle_index = 0;
	

	// Pin setup:
	DDRB |= (1<<NEON1)|(1<<NEON2);	// Output pins of PortB
	DDRC |= (1<<RED)|(1<<GREEN)|(1<<BLUE);	// Output pins of PortC
	DDRD |= (1<<POWER_ON_PIN);
	
	output_high(PORTD,POWER_ON_PIN);	// Turn on power control pin
	unsigned char i, pwm;
	pwm = PWMDEFAULT;
	
	// initialize all pwm channels
	for(i=0 ; i<CHMAX ; i++)
	{
		compare[i] = pwm;           // set default PWM values
		compbuff[i] = pwm;          // set default PWM values
	}
	
	
	
	// Timer 0 setup (used for software PWM):
	TCCR0B = (1 << CS00);         // no prescaller (count to 0xFF). PWM freq = (Clock/256)/256. With 8MHz, PWM freq. is 122Hz.
	TIMSK0 = (1 << TOIE0);         // enable overflow interrupt

	// Timer 1 setup (temp use, update RGB values):
	TCCR1B = (1<<CS11) | (1<<CS10) | (1<<WGM12);	// use CLK/64 prescale value, clear timer/counter on compareA match
	OCR1A = ((XTAL/64/TIMER_FREQ) - 1 );	// preset timer1 high/low byte 
	TIMSK1 |= (1<<OCIE1A);				// enable Output Compare 1 overflow interrupt

	//External interrupt setup:	
	DDRD &= ~((1<<PD2)|(1<<PD3)|(1<<SW1)|(1<<SW2)|(1<<SW3)); // Setting pins as inputs.
// ***	EICRA |= ((1<<ISC01)|(1<<ISC10)|(1<<ISC11));	// Falling edge interrupt for INT0 and rising in INT1.
	EICRA = ((1<<ISC01)|(1<<ISC11));	// Falling edge interrupt for INT0 and rising in INT1.
	EIMSK |= ((1<<INT0)|(1<<INT1));		// Enabling INT0 and INT1
	PCICR |= (1<<PCIE2);		// Enable PCINT2 interrupt.
	PCMSK2 |= (1<<PCINT21)|(1<<PCINT22)|(1<<PCINT23);	// Enable PCINT 21 to 23 (keys)

	i2c_init();		// initialize I2C library
	
	// Configuring DS1307:
	DS1307_write_byte(CR,SQW_OUT_1Hz);

	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	
	output_low(PORTB,NEON2);

 	// initialize the radio, including the SPI module
	Radio_Init();
 
	// configure the receive settings for radio pipe 0
	Radio_Configure_Rx(RADIO_PIPE_0, station_addr, ENABLE);
 
	// configure radio transceiver settings.
	Radio_Configure(RADIO_1MBPS, RADIO_HIGHEST_POWER);
 
	Radio_Set_Tx_Addr(trans_addr);  // or use the address manually informed by trans_addr.
	
	sei();

    while(1)
    {

	if (rxflag)
	{
		rx_status = Radio_Receive(&packet); // Copy received packet to memory and store the result in rx_status.
		if (rx_status == RADIO_RX_SUCCESS || rx_status == RADIO_RX_MORE_PACKETS) // Check if a packet is available.
		{
			output_high(PORTB,NEON2); // Turn on the led.
        
			if (packet.type != TXVALUES)
			{
				//snprintf(output, sizeof(output), "Error: wrong packet type: %d. Should be %d\n\r", packet.type, TXVALUES);
				//Serial.print(output);
			}            

			// Print out the message, along with the message ID and sender address.
//			snprintf(output, sizeof(output), "DirUD: %d DirLR: %d EsqUD: %d EsqLR %d \n\r",
//				packet.payload.txvalues.ud_dir,
//				packet.payload.txvalues.lr_dir,
//				packet.payload.txvalues.ud_esq,
//				packet.payload.txvalues.lr_esq);
//			Serial.print(output); 
			//CalculaVelocidades(packet.payload.txvalues.ud_dir, packet.payload.txvalues.lr_dir);
			//atualizaMotores();
			//atualizaServos(packet.payload.txvalues.ud_esq, packet.payload.txvalues.lr_esq);
			//processaAcao(packet.payload.txvalues.action);
			
			// Resetando a tarefa de verificacao de falha de transmissao:
			//RxCheck.reset();

			// Reply to the sender by sending an ACK packet, reusing the packet data structure.
			packet.type = ACK;
			// Se the ack message id:
			packet.payload.ack.messageid = 44;
			packet.payload.ack.veloc_dir = 10;
			packet.payload.ack.veloc_esq = 100;

			if (Radio_Transmit(&packet, RADIO_WAIT_FOR_TX) == RADIO_TX_MAX_RT)
			{
				// If the max retries was reached, the packet was not acknowledged.
				// This usually occurs if the receiver was not configured correctly or
				// if the sender didn't copy its address into the radio packet properly.
//				snprintf(output, sizeof(output), "Could not reply to sender.\n\r");
//				Serial.print(output);
			}
			else
			{
			// the transmission was completed successfully
			//snprintf(output, sizeof(output), "Replied to sender.\n\r");
			//Serial.print(output);
			output_low(PORTB,NEON2);// turn off the led.
			}
		}
		rxflag = 0;  // clear the flag.
	}
		
		if (int0_flag == 1)	// One second has passed.
		{
			// Check if it is to sleep (according to AVR LibC reference):
			cli();
			if (sleep_seconds_counter == SLEEP_SECONDS)
			{
				// It is time to sleep!
				//EIMSK &= ~((1<<INT0));	// Disable INT0 (square wave from RTC).
				PCICR &= ~(1<<PCIE2);	// Disable PCINT2 interrupt.
				
				output_high(PORTB,NEON2);
				output_low(PORTD,POWER_ON_PIN);

				sleep_enable();
				//sleep_bod_disable();
				sei();
				sleep_cpu();
				sleep_disable();
				output_high(PORTD,POWER_ON_PIN);
				output_low(PORTB,NEON2);
				//EIMSK |= ((1<<INT0));	// Enable INT0 (square wave from RTC).
				PCICR |= (1<<PCIE2);	// Enable PCINT2 interrupt.
			}
			sei();
			// Check if it is to cycle the mode (at every multiple of MODE_CHANGE_SECONDS)
			// while in no adjust mode.
			if ((mode_change_seconds_counter == MODE_CHANGE_SECONDS) & (current_adjust == OFF)) 
			{
				if (mode_cycle_index < 3)
				{
					current_mode = modes_cycle[mode_cycle_index];
					mode_cycle_index++;
				}
				else
				{
					mode_cycle_index = 0;
				}
				mode_change_seconds_counter = 0;	// Reset counter
			}
			readtime_DS1307(&clock1);

			DHT22_ERROR_t errorCode = readDHT22(&sensor_data);
			switch(errorCode)
			{
				case DHT_ERROR_NONE:
					clock1.humid_digit = sensor_data.humidity_integral;
					clock1.humid_decimal = sensor_data.humidity_decimal;
					clock1.temp_digit = abs(sensor_data.temperature_integral);
					clock1.temp_decimal = sensor_data.temperature_decimal;
					break;
				default:
					output_high(PORTB,NEON2);
					break;
			}	
			update_nixies(&clock1,current_mode);
			mode_change_seconds_counter++;
// ***			sleep_seconds_counter++;	
			
			int0_flag = 0;
		}

		if (key_pressed > 0) // Some key was pressed, read it.
		{
			switch (key_pressed)
			{
				case (1<<SW1):	// Key 1, view mode.
					mode_change_seconds_counter = 0; // Reset seconds counter.
                    if (current_mode < HUMID)
                    {
						current_mode++;
                    }
                    else
                    {
						current_mode = MIN_SEC;
                    }
					break;
				case (1<<SW2): // Key 2, se adjust parameter
					current_adjust++;
					if (current_adjust > DIGIT_HOUR)
					{
						current_adjust = OFF;
						leds_on = (1<<RGB_LED_HOUR)|(1<<RGB_LED_MIN)|(1<<RGB_LED_SYMB);	// If not adjusting, turn on RGB LEDs
					}
					if (current_adjust != OFF)
					{
						switch (current_mode)
						{
						case MIN_SEC:   // Seconds setting
							leds_on = (1<<RGB_LED_MIN);	// Only RGB led from Seconds digists will be on.
							adjust_addr = ADDR_SECS;
							current_adjust++;       // Skip next digit setting.
							break;
						case HOUR_MIN:
							if (current_adjust == DIGIT_MIN)        // Minutes setting.
							{
                                leds_on = (1<<RGB_LED_MIN);	// Only RGB led from Minutes digits will be on.
								adjust_addr = ADDR_MINS;
							}
							else
							{
                                leds_on = (1<<RGB_LED_HOUR);
								PCF8574_write(PCF8574_3,(1<<RGB_LED_HOUR)); // Only RGB led from Hours digits will be on.
								adjust_addr = ADDR_HOURS;
							}                                               
							break;
						case MONTH_DAY:
							if (current_adjust == DIGIT_MIN)	// Adjust the months.
							{
                                leds_on = (1<<RGB_LED_MIN);	// Only RGB led from Months/minutes digits will be on.
								adjust_addr = ADDR_MONTHS;
							}
							else	// Adjust the days.
							{
                                leds_on = (1<<RGB_LED_HOUR);	// Only RGB led from Days/hours digits will be on.
								adjust_addr = ADDR_DAYS;
							}                                               
							break;
						case YEAR:
							// Switch on both leds RGB led with red color
                            leds_on = (1<<RGB_LED_HOUR)|(1<<RGB_LED_MIN);	// Both leds will be on.
							adjust_addr = ADDR_YEAR;
							current_adjust++;       // Skip next digit setting.
							break;
                        case TEMP:
							leds_on = (1<<RGB_LED_HOUR)|(1<<RGB_LED_MIN);
                            current_adjust++;       // Skip next digit setting.
                            break;
                        case HUMID:
							leds_on = (1<<RGB_LED_HOUR)|(1<<RGB_LED_MIN);
                            current_adjust++;       // Skip next digit setting.
                            break;
						default:
							// Do nothing, for now.
							break;                                                          
						}
					}
					break;
				case (1<<SW3):	// Increment switch.
					if (current_adjust != OFF)
					{
						inc_dec_time_DS1307(&clock1, adjust_addr);
					}
					break;
				default:
					/* Your code here */
					break;
			}
			readtime_DS1307(&clock1);
			read_date_DS1307(&clock1);
			update_nixies(&clock1,current_mode);	// Update values and RGB led status of nixie displays.
			key_pressed = 0;
		}
		
		if (timer1A_flag == 1) // Update RGB leds.
		{
			if (current_adjust != OFF)	// When in adjust mode, se RGB led color to red.
			{
                hset(125,&led_color);
			}
            else	// otherwise, continue incrementing hue.
            {
                if (hue > 252)
                {
                    hue = 0;
                }
                hset(hue,&led_color);
                hue++;
            }
			compbuff[0] = led_color.r;
			compbuff[1] = led_color.g;
			compbuff[2] = led_color.b;
			
			timer1A_flag = 0;
		}			
    }
}

void delay_ms(uint16_t ms) {
  while (ms) {
    _delay_ms(1);
    ms--;
  }
}

void radio_rxhandler(uint8_t pipe_number)
{
	rxflag = 1;
	// This function is called when the radio receives a packet.
	// It is called in the radio's ISR, so it must be kept short.
	// The function may be left empty if the application doesn't need to respond immediately to the interrupt.
}
