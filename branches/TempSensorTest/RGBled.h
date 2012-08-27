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
 * RGBled.h
 *
 * Created: 12/02/2012 21:52:34
 *  Author: moreto
 */ 

#include <inttypes.h>

#ifndef RGBLED_H_
#define RGBLED_H_



#endif /* RGBLED_H_ */


typedef struct _color
{ 
    uint8_t r; uint8_t g; uint8_t b;
} color;

void h_to_rgb(uint8_t h, color *c);
void hset(uint8_t h, color *c);
void hvset(uint8_t h, uint8_t v, color *c);