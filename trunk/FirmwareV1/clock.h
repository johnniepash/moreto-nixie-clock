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
 * clock.h
 *
 * Time manipulation routines and data structures
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
	SLEEP_TIME,
	CHANGE_MODE_TIME,
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
	uint8_t year;	// 0 to 99, BCD separated by nibble.
	uint8_t temp_digit;
	uint8_t temp_decimal;
	uint8_t humid_digit; 
	uint8_t humid_decimal;
	uint8_t seconds_sleep_x4;
	uint8_t seconds_change_mode;
} my_time_t;

typedef struct  
{
	uint8_t seconds_sleep_x5;
	uint8_t seconds_change_mode;
} my_config_t;

uint8_t calculate_hour_DS1307(uint8_t hours); // Calculate the digits from the hour, min and seconds, using the format of DS1307 RTC.
