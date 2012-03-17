/*
 * clock.c
 *
 * Time manipulation routines
 *
 * 
 *
 * Created: 05/01/2012 16:39:36
 *  Author: Miguel
 */ 
#include "clock.h"

// Process the hours byte from DS1307. Need to check if it is 12 or 24h mode:
uint8_t calculate_hour_DS1307(uint8_t hours)
{
	uint8_t ampmflag;
	ampmflag = (hours & 0x40) >> 6;
	if (ampmflag == 1) // 12 hours mode.
	{
		return (hours & 0x1F);
	}
	else
	{
		return (hours & 0x3F);
	}
}