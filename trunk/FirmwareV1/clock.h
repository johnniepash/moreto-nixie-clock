/*
 * clock.h
 *
 * Time manipulation library definitions
 *
 * Created: 05/01/2012 16:16:25
 *  Author: Miguel
 */ 

#include <inttypes.h>

#ifndef CLOCK_H_
#define CLOCK_H_


#endif /* CLOCK_H_ */

// Enum that holds the possible display modes:
typedef enum _mode 
{
	MIN_SEC,
	HOUR_MIN,
	MONTH_DAY,
	YEAR,
	TEMP,
	HUMID,
} display_mode;

// Enum that holds the current digits that are being set.
typedef enum _set
{
	OFF,
	DIGIT_MIN,
	DIGIT_HOUR,
} set_digit;

typedef struct
{
	uint8_t sec;	// LS nibble has seconds and MS nibble has tens of seconds.
	uint8_t min;	// same for minutes
	uint8_t hour;	// same for hours
	uint8_t weekday;	// week day 1 to 7
	uint8_t day;	// 
	uint8_t month;
	uint8_t year;	// 0 to 99, BCD sapareted by nibble.
	uint16_t temperature;
	uint16_t humidity; 
} my_time_t;

uint8_t calculate_hour_DS1307(uint8_t hours); // Calculate the digits from the hour, min and seconds, using the format of DS1307 RTC.
