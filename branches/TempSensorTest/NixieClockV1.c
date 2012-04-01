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

#include "OWIIntFunctions.h"
#include "OWIInterruptDriven.h"
#include ".\common_files\OWIcrc.h"

#define DS1307		0xD0 	// I2C address of DS1307 11010000. Last zero is the R/W bit. 0x70
#define PCF8574_1	0x70	// I2C address of PCF8574 I/O expander 1. 01000000 0x70
#define PCF8574_2	0x72	// I2C address of PCF8574 I/O expander 2. 01000010 0x72
#define PCF8574_3	0x74	// I2C address of PCF8574 I/O expander 3. 01000100 0x74
#define XTAL		8000000L    // Crystal frequency in Hz
#define TIMER_FREQ	10			// timer1 frequency in Hz
#define MODE_CHANGE_SECONDS	3

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
//#define LED PD6
#define KEY_MASK 0x0F	// Mask used to read keys connected in a PCF8574 I/O expander. 0x0F, only four keys.
#define RGB_LED_MIN 0x00	// Pin number of the Common cathode RGB led conected at PCF8574_3
#define RGB_LED_HOUR 0x01
#define RGB_LED_SYMB 0x02
#define NIXIE_DEGREE 0x04
#define NIXIE_PERCENT 0x05
#define NIXIE_m 0x06
#define NIXIE_M 0x07

#define RED PB0
#define GREEN PB1
#define BLUE PB2
#define NEON1 PB6
#define NEON2 PB7
#define SW1 PD5
#define SW2 PD6
#define SW3 PD7

// Software PWM definitions:
#define CHMAX 3 // maximum number of PWM channels
#define PWMDEFAULT 0x00 // default PWM value at start up for all channels
#define PORTB_MASK  (1 << RED)|(1 << GREEN)|(1 << BLUE) // PWM pin Mask
#define LED_PORT PORTB
#define LED_DDR DDRB


// Some macros that make the code more readable
#define output_low(port,pin) port &= ~(1<<pin)
#define output_high(port,pin) port |= (1<<pin)
#define set_input(portdir,pin) portdir &= ~(1<<pin)
#define set_output(portdir,pin) portdir |= (1<<pin)
//#define SET(port,pin) port |= (1<<pin)
#define toggle(port,pin) port ^= (1<<pin)

void delay_ms(uint16_t ms);

// Dallas one wire lib definitions:
void OWI_StateMachine();

// Defines used only in code example.
#define OWI_STATE_IDLE                  0
#define OWI_STATE_DETECT_PRESENCE1      1
#define OWI_STATE_WAIT_FOR_CONVERSION1  2
#define OWI_STATE_WAIT_FOR_CONVERSION2  3
#define OWI_STATE_DETECT_PRESENCE2      4
#define OWI_STATE_READ_SCRATCHPAD       5
#define OWI_STATE_CHECK_CRC             6

#define FALSE       0
#define TRUE        1

#define DS1820_START_CONVERSION         0x44    //!< DS1820 start conversion command
#define DS1820_READ_SCRATCHPAD          0xbe    //!< DS1820 Read scratchpad command

extern OWIflags OWIStatus;
extern unsigned char *OWIDataBuffer;

signed int temperature = 100;
const uint8_t temp_decimal[] = {00,06,12,19,25,31,37,44,50,56,62,69,75,81,87,94,00};

// Global variables:
volatile unsigned char int0_flag = 0;
volatile unsigned char int1_flag = 0;
volatile unsigned char key_pressed = 0;
volatile unsigned char timer1A_flag = 0;
volatile unsigned char change_mode_flag = 0;
volatile unsigned char leds_on = (1<<RGB_LED_HOUR)|(1<<RGB_LED_MIN)|(1<<RGB_LED_SYMB);	// RGB and symbolic nixies status (value to be written in PCF8574_3).

unsigned char compare[CHMAX];
volatile unsigned char compbuff[CHMAX];


// External interrupt 0 ISR (fired by the PCF8574 with MC14490 key debouncer):
ISR(INT0_vect) 
{ 
	int0_flag = 1;	// Set the flag (the processing is done in main function).
}

// External interrupt 0 ISR (fired by the PCF8574 with MC14490 key debouncer):
ISR(INT1_vect) 
{ 
	int1_flag = 1;	// Set the flag (the processing is done in main function).
}

// External interrupt Pin Change 2 (PCINT 16 to 23).
ISR(PCINT2_vect)
{
	unsigned char keys;
	keys = (~PIND) & ((1<<SW1)|(1<<SW2)|(1<<SW3));
	if (keys > 0)
	{
		key_pressed = keys;
	}
	else
	{
		key_pressed = 0;
	}
	
}


ISR(TIMER0_OVF_vect)
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
	//uint8_t upperval, lowerval;
	// Convert val to bcd:
	//upperval = (uint8_t)(val/100);
	//lowerval = (uint8_t)(val % 100);
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
			PCF8574_write(PCF8574_1,0x00);  // Temp variable
			PCF8574_write(PCF8574_2,0x00);  // Temp variable
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
	PCF8574_write(PCF8574_3,0x00);	// Turn off the symbolic nixie.
	// Variables:
	//uint16_t a = 0;
	my_time_t clock1;
	display_mode current_mode = TEMP;
	set_digit current_adjust = OFF;
	uint8_t adjust_addr = 0;	// DS1307 adjust address variable.
	color led_color;
    uint8_t hue = 0;
	hset(hue,&led_color);
		
	// Pin setup:
	DDRB |= (1<<RED)|(1<<GREEN)|(1<<BLUE)|(1<<NEON1)|(1<<NEON2);	// Output pins of PortB

	unsigned char i, pwm;
	pwm = PWMDEFAULT;
	
	// initialize all pwm channels
	for(i=0 ; i<CHMAX ; i++)
	{
		compare[i] = pwm;           // set default PWM values
		compbuff[i] = pwm;          // set default PWM values
	}
	
	
	
	// Timer 0 setup (used for software PWM):
	//TCCR0A = 0x00;//(1<< WGM00);
	TCCR0B = (1 << CS00);         // no prescaller (count to 0xFF). PWM freq = (Clock/256)/256. With 8MHz, PWM freq. is 122Hz.
	TIMSK0 = (1 << TOIE0);         // enable overflow interrupt
	// Timer 1 setup (temp use):
	TCCR1B = (1<<CS11) | (1<<CS10) | (1<<WGM12);	// use CLK/64 prescale value, clear timer/counter on compareA match
	OCR1A = ((XTAL/64/TIMER_FREQ) - 1 );	// preset timer1 high/low byte 
	TIMSK1 |= (1<<OCIE1A);				// enable Output Compare 1 overflow interrupt

	//External interrupt setup:	
//	set_input(DDRD, PD2);	// INT0
//	set_input(DDRD, PD3);	// INT1
	DDRD &= ~((1<<PD2)|(1<<PD3)|(1<<SW1)|(1<<SW2)|(1<<SW3)); // Setting pins as inputs.
	EICRA |= ((1<<ISC01)|(1<<ISC11));	// Falling edge interrupt for INT0 and INT1
	EIMSK |= ((1<<INT0)|(1<<INT1));		// Enabling INT0 and INT1
	PCICR |= (1<<PCIE2);		// Enable PCINT2 interrupt.
	PCMSK2 |= (1<<PCINT21)|(1<<PCINT22)|(1<<PCINT23);	// Enable PCINT 21 to 23 (keys)

	i2c_init();		// initialize I2C library
	
	// Configuring DS1307:
	DS1307_write_byte(CR,SQW_OUT_1Hz);
	//i2c_start_wait(DS1307+I2C_WRITE);
	//i2c_write(CR);	// write control register address
	//i2c_write(SQW_OUT_1Hz);	// write control register value
	//i2c_stop();		// release bus.
	
	OWI_Init();
	
	sei();

	//update_nixies(a);
    while(1)
    {

		// If the 1-Wire(R) bus is not busy, run the state machine.
		if (!OWIStatus.busy)
		{
			OWI_StateMachine();
		}
		
		
		if (int0_flag == 1)	// One second has passed.
		{
			readtime_DS1307(&clock1);
//			toggle(PORTB,NEON1);
			clock1.temp_digit = (uint8_t)(temperature >> 4);
			clock1.temp_decimal = temp_decimal[temperature & 0x000F];
			update_nixies(&clock1,current_mode);
			int0_flag = 0;
			
		}

		if (key_pressed > 0) // Some key was pressed, read it.
		{
			switch (key_pressed)
			{
				case (1<<SW1):	// Key 1, view mode.
					toggle(PORTB,NEON2);
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
					toggle(PORTB,NEON2);
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
                            adjust_addr = 0x10;	// TEMPORARY STUFF
                            break;
                        case HUMID:
							leds_on = (1<<RGB_LED_HOUR)|(1<<RGB_LED_MIN);
                            current_adjust++;       // Skip next digit setting.
                            adjust_addr = 0x10;	// TEMPORARY STUFF
                            break;
						default:
							// Do nothing, for now.
							break;                                                          
						}
					}
					break;
				case (1<<SW3):	// Increment switch.
					toggle(PORTB,NEON2);
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

/*! \brief  The state machine that controls communication on the 1-Wire bus
 *  
 *  This function is called from main every time the 1-Wire driver is not
 *  busy. The state machine will read the temperature from a DS1820 temperature
 *  sensor, crc check it, and put it in the global variable temperature if 
 *  everything is OK.
 */
void OWI_StateMachine()
{
    static unsigned char state = OWI_STATE_IDLE;
    static unsigned char buf[9];
    unsigned char i;
    unsigned char crc;
    
    // If an error has occurred since last time, clear all flags and
    // return to idle state.
    if (OWIStatus.error)
    {
        state = OWI_STATE_IDLE;
        OWIStatus.allFlags = FALSE;
    }
    
    switch (state)
    {
        case OWI_STATE_IDLE:
        {
            // Send reset signal and update state.
            OWI_DetectPresence();
            state = OWI_STATE_DETECT_PRESENCE1;
            break;
        }

        case OWI_STATE_DETECT_PRESENCE1:
        {
            // If no presence was detected, go back to idle state.
            if(OWIStatus.presenceDetected == FALSE)
            {
                state = OWI_STATE_IDLE;
            }
            // If presence was detected, send Skip ROM and Start conversion
            // signals. 
            else
            {
                buf[0] = OWI_ROM_SKIP;
                buf[1] = DS1820_START_CONVERSION;
                OWI_TransmitData(buf, 16);
                state = OWI_STATE_WAIT_FOR_CONVERSION1;
            }
            break;
        }

        case OWI_STATE_WAIT_FOR_CONVERSION1:
        {
            // Receive one byte of data to check for completion of the 
            // temperature conversion.
            OWI_ReceiveData(buf, 8);
            state = OWI_STATE_WAIT_FOR_CONVERSION2;
            break;
        }
    
        case OWI_STATE_WAIT_FOR_CONVERSION2:
        {
            // If everything received was zero. Jump to the last state
            // to receive a new byte.
            if (buf[0] == 0x00)
            {
                state = OWI_STATE_WAIT_FOR_CONVERSION1;
            }
            // If there was at least 1 one received, continue with a new
            // reset.
            else
            {
                OWI_DetectPresence();
                state = OWI_STATE_DETECT_PRESENCE2;
            }
            break;
        }
    
        case OWI_STATE_DETECT_PRESENCE2:
        {
            // If no presence was detected, go back to idle state.
            if(OWIStatus.presenceDetected == FALSE)
            {
                state = OWI_STATE_IDLE;
            }
            // If presence was detected, send Skip ROM and Read scratchpad
            // signals. 
            else
            {
                buf[0] = OWI_ROM_SKIP;
                buf[1] = DS1820_READ_SCRATCHPAD;
                OWI_TransmitData(buf, 16);
                state = OWI_STATE_READ_SCRATCHPAD;   
            }
            break;
        }
    
        case OWI_STATE_READ_SCRATCHPAD:
        {
            // Read the 9 bytes of scratchpad data.
            OWI_ReceiveData(buf, 9 * 8);
            state = OWI_STATE_CHECK_CRC;
            break;
        }
    
        case OWI_STATE_CHECK_CRC:
        {
            // Compare the computed crc with the crc read from the 
            // scratchpad. 
            crc = 0;
            for(i = 0; i < 8; i++)
            {
                crc =  OWI_ComputeCRC8(buf[i], crc);
            }
            // If they match, update the temperature variable.
            if (crc == buf[8])
            {
                temperature = buf[0] | (buf[1] << 8);                
                state = OWI_STATE_IDLE;
            }
            // If they don't match, go back to the second Reset to 
            // read the scratchpad again.
            else
            {
                OWI_DetectPresence();
                state = OWI_STATE_DETECT_PRESENCE2;
            }
            break;
        }
    }
}