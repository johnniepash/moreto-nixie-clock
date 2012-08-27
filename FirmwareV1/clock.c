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
 * clock.c
 *
 * Time manipulation routines and data structures
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