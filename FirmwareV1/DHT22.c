/* DHT22.c
 *
 * DHT22 sensor AVR library
 *
 * Code by funkytransistor published at AVR Freaks forum:
 * http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&p=974797
 *
 * Modified by Miguel Moreto in order to return four values:
 *   Integer part or temperature
 *   Decimal part of temperature
 *   Integer part of humidity
 *   Decimal part of humidity
 * 
 * Miguel Moreto, Brazil, 2012.
 */
#include "DHT22.h"

DHT22_ERROR_t readDHT22(int8_t *temp_integral, uint8_t *temp_decimal, uint8_t *hum_integral,uint8_t *hum_decimal)
{

	uint8_t retryCount = 0;
	uint8_t bitTimes[DHT22_DATA_BIT_COUNT];
	int rawHumidity = 0;
	int rawTemperature = 0;
	uint8_t checkSum = 0, csPart1, csPart2, csPart3, csPart4;
	int i;

	// Pin needs to start HIGH, wait until it is HIGH with a timeout
	retryCount = 0;
	cli();
	DDRC &= ~(1 << ( PC3 )); 							//DIRECT_MODE_INPUT(reg, bitmask);
	sei();
	do
	{
		if(retryCount > 125) return DHT_BUS_HUNG;
		retryCount++;
		_delay_us(2);
	} while( !( PINC & ( 1 << PC3 ) ) );				//!DIRECT_READ(reg, bitmask)

	// Send the activate pulse
	cli();
	PORTC &= ~(1 << ( PC3 )); 							//DIRECT_WRITE_LOW(reg, bitmask);
	DDRC |= 1 << ( PC3 );								//DIRECT_MODE_OUTPUT(reg, bitmask); // Output Low
	sei();
	_delay_ms(20); 										// spec is 1 to 10ms
	cli();
	DDRC &= ~(1 << ( PC3 ));;							// Switch back to input so pin can float
	sei();

	// Find the start of the ACK signal
	retryCount = 0;
	do
	{
		if (retryCount > 25) 							//(Spec is 20 to 40 us, 25*2 == 50 us)
		{
			return DHT_ERROR_NOT_PRESENT;
		}
		retryCount++;
		_delay_us(2);
	} while( PINC & ( 1 << PC3 ) );


	// Find the transition of the ACK signal
	retryCount = 0;
	do
	{
		if (retryCount > 50) 							//(Spec is 80 us, 50*2 == 100 us)
		{
			return DHT_ERROR_ACK_TOO_LONG;
		}
		retryCount++;
		_delay_us(2);
	} while( !(PINC & ( 1 << PC3 )) );

	// Find the end of the ACK signal
	retryCount = 0;
	do
	{
		if (retryCount > 50) 							//(Spec is 80 us, 50*2 == 100 us)
		{
			return DHT_ERROR_ACK_TOO_LONG;
		}
		retryCount++;
		_delay_us(2);
	} while( PINC & ( 1 << PC3 ) );

	// Read the 40 bit data stream
	for(i = 0; i < DHT22_DATA_BIT_COUNT; i++)
	{
		// Find the start of the sync pulse
		retryCount = 0;
		do
		{
			if (retryCount > 35) 						//(Spec is 50 us, 35*2 == 70 us)
			{
				return DHT_ERROR_SYNC_TIMEOUT;
			}
			retryCount++;
			_delay_us(2);
		} while( !(PINC & ( 1 << PC3 )) );

		// Measure the width of the data pulse
		retryCount = 0;
		do
		{
			if (retryCount > 50) 						//(Spec is 80 us, 50*2 == 100 us)
			{
				return DHT_ERROR_DATA_TIMEOUT;
			}
			retryCount++;
			_delay_us(2);
		} while( PINC & ( 1 << PC3 ) );
		bitTimes[i] = retryCount;
	}

	// translate bitTimes
	// 26~28us == logical 0
	// 70us	   == logical 1
	// here treshold is 40us
	for(i = 0; i < 16; i++)
	{
		if( bitTimes[i] > 20 ) rawHumidity |= (1 << (15 - i));
	}

	for(i = 0; i < 16; i++)
	{
		if( bitTimes[i + 16] > 20 ) rawTemperature |= (1 << (15 - i));
	}

	for(i = 0; i < 8; i++)
	{
		if(bitTimes[i + 32] > 20) checkSum |= (1 << (7 - i));
	}

	// calculate checksum
	csPart1 = rawHumidity >> 8;
	csPart2 = rawHumidity & 0xFF;
	csPart3 = rawTemperature >> 8;
	csPart4 = rawTemperature & 0xFF;

	if( checkSum == ( (csPart1 + csPart2 + csPart3 + csPart4) & 0xFF ) )
	{
		// raw data to sensor values
		*hum_integral = (uint8_t)(rawHumidity / 10);
		*hum_decimal = (uint8_t)(rawHumidity % 10);

		if(rawTemperature & 0x8000)	// Check if temperature is below zero, non standard way of encoding negative numbers!
		{
			rawTemperature &= 0x7FFF; // Remove signal bit
			*temp_integral = (int8_t)(rawTemperature / 10.0) * -1;
			*temp_decimal = (uint8_t)(rawTemperature % 10);
		} else
		{
			*temp_integral = (int8_t)(rawTemperature / 10.0);
			*temp_decimal = (uint8_t)(rawTemperature % 10);			
		}

		return DHT_ERROR_NONE;
	}

	return DHT_ERROR_CHECKSUM;
}




