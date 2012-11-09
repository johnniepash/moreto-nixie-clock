/* DHT22.h
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
#ifndef _DHT22_H_
#define _DHT22_H_

//#include <stdlib.h>
#include <avr/interrupt.h>
#include <util/delay.h>


#define DHT22_DATA_BIT_COUNT 40

/* Configure port and pin */
#define DHT22_PIN PC3
#define DHT22_DDR DDRC
#define DHT22_PORT_OUT PORTC
#define DHT22_PORT_IN PINC

typedef enum
{
  DHT_ERROR_NONE = 0,
  DHT_BUS_HUNG,
  DHT_ERROR_NOT_PRESENT,
  DHT_ERROR_ACK_TOO_LONG,
  DHT_ERROR_SYNC_TIMEOUT,
  DHT_ERROR_DATA_TIMEOUT,
  DHT_ERROR_CHECKSUM,
} DHT22_ERROR_t;

typedef struct  
{
	int8_t temperature_integral;
	uint8_t temperature_decimal;
	uint8_t humidity_integral; 
	uint8_t humidity_decimal; 
} DHT22_DATA_t;


//DHT22_ERROR_t readDHT22(int8_t *temp_integral, uint8_t *temp_decimal, uint8_t *hum_integral,uint8_t *hum_decimal);
DHT22_ERROR_t readDHT22(DHT22_DATA_t* data);


#endif
