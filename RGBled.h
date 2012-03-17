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