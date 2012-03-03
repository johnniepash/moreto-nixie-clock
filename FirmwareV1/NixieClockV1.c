/*
 * NixieClockV1.c
 *
 * Created: 08/02/2012 20:22:37
 *  Author: moreto
 *
 *
 * Software PWM adapted from Smart LED Prototypes (http://todbot.com/blog/2007/03/25/smart-led-prototypes/).
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "i2cmaster.h"
#include "clock.h"
#include "RGBled.h"

#define DS1307		0xD0 	// I2C address of DS1307 11010000. Last zero is the R/W bit. 0x70
#define PCF8574_1	0x70	// I2C address of PCF8574 I/O expander 1. 01000000 0x70
#define PCF8574_2	0x72	// I2C address of PCF8574 I/O expander 2. 01000010 0x72
#define PCF8574_3	0x74	// I2C address of PCF8574 I/O expander 3. 01000100 0x74
#define XTAL		8000000L    // Crystal frequency in Hz
#define TIMER_FREQ	50			// timer1 frequency in Hz

#define NIXIE_DEGREE 0x1F
#define NIXIE_PERCENT 0x2F
#define NIXIE_m 0x4F
#define NIXIE_M 0x4F

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
#define LED PD6
#define KEY_MASK 0x0F	// Mask used to read keys connected in a PCF8574 I/O expander. 0x0F, only four keys.
#define LED_HOUR PB3
#define LED_MIN PB4
#define LED_SYMB PB5
#define RED PB0
#define GREEN PB1
#define BLUE PB2
#define PORTB_MASK  (1 << RED)|(1 << GREEN)|(1 << BLUE) // PWM pin Mask
#define LED_PORT PORTB
#define LED_DDR DDRB

// Software PWM definitions:
#define CHMAX 3 // maximum number of PWM channels
#define PWMDEFAULT 0x00 // default PWM value at start up for all channels

// Some macros that make the code more readable
#define output_low(port,pin) port &= ~(1<<pin)
#define output_high(port,pin) port |= (1<<pin)
#define set_input(portdir,pin) portdir &= ~(1<<pin)
#define set_output(portdir,pin) portdir |= (1<<pin)
//#define SET(port,pin) port |= (1<<pin)
#define toggle(port,pin) port ^= (1<<pin)

void delay_ms(uint16_t ms);

// Global variables:
volatile char int0_flag = 0;
volatile char int1_flag = 0;
volatile char timer1A_flag = 0;

unsigned char compare[CHMAX];
volatile unsigned char compbuff[CHMAX];

int r_val = 0x00;
int g_val = 0x55;
int b_val = 0xAA;
float dim = 1;

// External interrupt 1 ISR (fired by the DS1307 square wave output):
ISR(SIG_INTERRUPT1) 
{ 
	int1_flag = 1;	// Set the flag (the processing is done in main function).
}

// External interrupt 0 ISR (fired by the PCF8574 with MC14490 key debouncer):
ISR(SIG_INTERRUPT0) 
{ 
	int0_flag = 1;	// Set the flag (the processing is done in main function).
}

ISR (SIG_TIMER0_OVF)
{
	//static unsigned char pinlevelB=PORTB_MASK;	// Initially all pwm pins are 1.
	static unsigned char softcount=0xFF;
	
	//PORTB = pinlevelB;		// update outputs
  
	if(++softcount == 0)	// increment modulo 256 counter and update the compare values only when counter = 0.
							// One period has passed.
	{         
		compare[0] = compbuff[0];
		compare[1] = compbuff[1];
		compare[2] = compbuff[2];
		//pinlevelB = PORTB_MASK;     // set all port pins high
		PORTB |= PORTB_MASK;
	}
	// clear port pin on compare match (executed on next interrupt)
	if(compare[0] == softcount)
	{
		//pinlevelB &= ~(1 << RED);	// Red LED turn off.
		PORTB &= ~(1 << RED);	// Red LED turn off.
	}
	if(compare[1] == softcount)
	{
		//pinlevelB &= ~(1 << GREEN);	// Green LED turn off.
		PORTB &= ~(1 << GREEN);	// Green LED turn off.
	}		
	if(compare[2] == softcount)
	{
		//pinlevelB &= ~(1 << BLUE);	// Blue LED turn off.
		PORTB &= ~(1 << BLUE);	// Blue LED turn off.
	}
}

ISR (SIG_OUTPUT_COMPARE1A)
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
 	uint8_t	hours;
	
	i2c_start_wait(DS1307+I2C_WRITE);	// set device address and write mode
	i2c_write(0x00);					// write address = 0
	i2c_rep_start(DS1307+I2C_READ);	// set device address and read mode
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
	
	i2c_start_wait(DS1307+I2C_WRITE);	// set device address and write mode
	i2c_write(0x03);					// write address = 3
	i2c_rep_start(DS1307+I2C_READ);	// set device address and read mode
	my_time->weekday = i2c_readAck();
	my_time->day = i2c_readAck();
	my_time->month = i2c_readAck();
	my_time->year = i2c_readNak();
	i2c_stop();
}

// Update the number shown by nixies accordingly with the selected mode.
void update_nixies(my_time_t *my_time, display_mode current_mode)
{
	switch (current_mode)
	{
		case MIN_SEC:
			toggle(PORTD,LED);
			output_low(PORTB, LED_SYMB);
			PCF8574_write(PCF8574_1,my_time->min);
			PCF8574_write(PCF8574_2,my_time->sec);
			break;
		case HOUR_MIN:
			toggle(PORTD,LED);
			output_low(PORTB, LED_SYMB);
			PCF8574_write(PCF8574_1,my_time->hour);
			PCF8574_write(PCF8574_2,my_time->min);
			break;
		case MONTH_DAY:
			output_high(PORTD,LED);
			output_low(PORTB, LED_SYMB);
			PCF8574_write(PCF8574_1,my_time->day);
			PCF8574_write(PCF8574_2,my_time->month);
			break;
		case YEAR:
			output_low(PORTD,LED);
			output_low(PORTB,LED_SYMB);
			PCF8574_write(PCF8574_1,0x20);
			PCF8574_write(PCF8574_2,my_time->year);
			break;
		case TEMP:
			output_high(PORTB, LED_SYMB);
			output_high(PORTD,LED);
			PCF8574_write(PCF8574_3,NIXIE_DEGREE);
			PCF8574_write(PCF8574_1,0x00);	// Temp variable
			PCF8574_write(PCF8574_2,0x00);	// Temp variable
			break;
		case HUMID:
			output_high(PORTB, LED_SYMB);
			output_high(PORTD,LED);
			PCF8574_write(PCF8574_3,NIXIE_PERCENT);
			PCF8574_write(PCF8574_1,0x00);	// Temp variable
			PCF8574_write(PCF8574_2,0x00);	// Temp variable
			break;
		default:
			output_low(PORTB, LED_SYMB);
			/* Your code here */
			break;
	}
				
}


int main(void)
{
	PCF8574_write(PCF8574_3,0x0F);	// Turn off the symbolic nixie.
	// Variables:
	uint8_t a = 0;
	uint8_t key_pressed = 0;
	uint8_t PCF_data = 0x0F;
	my_time_t clock1;
	display_mode current_mode = HOUR_MIN;
	set_parameter current_adjust = OFF;
	uint8_t adjust_addr = 0;	// DS1307 adjust address variable.
	color led_color;
    uint8_t hue = 0;
    uint8_t bright = 255;
	hset(hue,&led_color);
		
	// Pin setup:
	set_output(DDRD, LED);
	set_output(DDRB, LED_HOUR);
	set_output(DDRB, LED_MIN);
	set_output(DDRB, LED_SYMB);
	set_output(LED_DDR,RED);	// PWM
	set_output(LED_DDR,GREEN);	// PWM
	set_output(LED_DDR,BLUE);	// PWM

	output_low(PORTB, LED_SYMB);
	output_high(PORTB, LED_MIN);
	output_high(PORTB, LED_HOUR);

	unsigned char i, pwm;
	pwm = PWMDEFAULT;
	
	// initialize all pwm channels
	for(i=0 ; i<CHMAX ; i++)
	{
		compare[i] = pwm;           // set default PWM values
		compbuff[i] = pwm;          // set default PWM values
	}
	
	// Timer 0 setup (used for software PWM):
	TCCR0A = (1<< WGM00);
	TCCR0B = (1 << CS00);         // no prescaller (count to 0xFF). PWM freq = (Clock/256)/256. With 8MHz, PWM freq. is 122Hz.
	TIMSK = (1 << TOIE0);         // enable overflow interrupt

	TCCR1B = (1<<CS11) | (1<<WGM12);	// use CLK/8 prescale value, clear timer/counter on compareA match
	OCR1A = ((XTAL/8/TIMER_FREQ) - 1 );	// preset timer1 high/low byte 
	TIMSK  |= (1<<OCIE1A);				// enable Output Compare 1 overflow interrupt

	//External interrupt setup:	
	set_input(DDRD, PD2);
	set_input(DDRD, PD3);
	MCUCR |= ((1<<ISC01)|(1<<ISC11));	// Falling edge interrupt for INT0 and INT1
	GIMSK |= ((1<<INT0)|(1<<INT1));		// Enabling INT0 and INT1

	i2c_init();		// initialize I2C library

	// Configuring DS1307:
	i2c_start_wait(DS1307+I2C_WRITE);
	i2c_write(CR);	// write control register address
	i2c_write(SQW_OUT_1Hz);	// write control register value
	i2c_stop();		// release bus.
	
	sei();
		
    while(1)
    {
		

		if (int1_flag == 1)	// One second has passed.
		{
			readtime_DS1307(&clock1);
			update_nixies(&clock1,current_mode);

			int1_flag = 0;
			
		}
		if (int0_flag == 1) // Some key was pressed, read it.
		{
			if (current_mode == TEMP)
			{
				PCF8574_write(PCF8574_3,NIXIE_DEGREE);
			}
			else if (current_mode == HUMID)
			{
				PCF8574_write(PCF8574_3,NIXIE_PERCENT);
			}
			else
			{
				PCF8574_write(PCF8574_3,0x0F);
			}
			key_pressed = KEY_MASK & PCF8574_read(PCF8574_3);
			
			switch (key_pressed)
			{
				case 1:	// Key 1, view mode.
					if (current_mode < HUMID)
					{
						current_mode++;
					}
					else
					{
						current_mode = MIN_SEC;
					}
					break;
				case 2: // Key 2, adjust
					current_adjust++;
					switch (current_adjust)
					{
						case SECS:
							current_mode = MIN_SEC;
							// Switch on seconds RGB led with red color
							output_low(PORTB, LED_HOUR);
							output_high(PORTB, LED_MIN);
							adjust_addr = ADDR_SECS;
							break;
						case MINS:
							current_mode = HOUR_MIN;
							// Switch on minutes RGB led with red color
							output_low(PORTB, LED_HOUR);
							output_high(PORTB, LED_MIN);
							adjust_addr = ADDR_MINS;
							break;
						case HOURS:
							current_mode = HOUR_MIN;
							// Switch on hours RGB led with red color
							output_high(PORTB, LED_HOUR);
							output_low(PORTB, LED_MIN);							
							adjust_addr = ADDR_HOURS;
							break;
						case DAYS:
							current_mode = MONTH_DAY;
							// Switch on days RGB led with red color
							output_low(PORTB, LED_HOUR);
							output_high(PORTB, LED_MIN);
							adjust_addr = ADDR_DAYS;
							break;
						case MONTHS:
							current_mode = MONTH_DAY;
							// Switch on month RGB led with red color
							output_high(PORTB, LED_HOUR);
							output_low(PORTB, LED_MIN);		
							adjust_addr = ADDR_MONTHS;
							break;
						case YEARS:
							current_mode = YEAR;
							// Switch on both leds RGB led with red color
							output_high(PORTB, LED_HOUR);
							output_high(PORTB, LED_MIN);		
							adjust_addr = ADDR_YEAR;
							break;		
						default:
							current_adjust = OFF;
							current_mode = HOUR_MIN;
							output_high(PORTB, LED_HOUR);
							output_high(PORTB, LED_MIN);
							break;	
							// Switch on hours and minutes RGB led.							
					}
					break;					
				default:
					/* Your code here */
					break;
			}
			readtime_DS1307(&clock1);
			update_nixies(&clock1,current_mode);
			int0_flag = 0;
		}
		
		if (timer1A_flag == 1) // Update RGB leds.
		{
			if (current_adjust != OFF)
			{
				hue = 43;
			}
			
			if (hue > 252)
			{
				hue = 0;
			}
			hset(hue,&led_color);
			compbuff[0] = led_color.r;
			compbuff[1] = led_color.g;
			compbuff[2] = led_color.b;
			hue++;
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