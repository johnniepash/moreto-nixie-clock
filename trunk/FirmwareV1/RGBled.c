/*
 * RGBled.c
 *
 * RGB led color routines.
 *
 * Adapted from Smart LED Prototypes (http://todbot.com/blog/2007/03/25/smart-led-prototypes/).
 *
 * Created: 12/02/2012 21:52:59
 *  Author: moreto
 */ 

#include "RGBled.h"

/*
 * Given a variable hue 'h', that ranges from 0-252,
 * set RGB color value appropriately.
 * Assumes maximum Saturation & maximum Value (brightness)
 * Performs purely integer math, no floating point.
 */
void h_to_rgb(uint8_t h, color *c) 
{
    uint8_t hd = h / 42;   // 42 == 252/6,  252 == H_MAX
    uint8_t hi = hd % 6;   // gives 0-5
    uint8_t f = h % 42; 
    uint8_t fs = f * 6;
    switch( hi ) {
        case 0:
            c->r = 252;     c->g = fs;      c->b = 0;
           break;
        case 1:
            c->r = 252-fs;  c->g = 252;     c->b = 0;
            break;
        case 2:
            c->r = 0;       c->g = 252;     c->b = fs;
            break;
        case 3:
            c->r = 0;       c->g = 252-fs;  c->b = 252;
            break;
        case 4:
            c->r = fs;      c->g = 0;       c->b = 252;
            break;
        case 5:
            c->r = 252;     c->g = 0;       c->b = 252-fs;
            break;
    }
}

/* 
 * Given a hue 0-252, set the LEDs at maximum brightness for that hue
 */
void hset(uint8_t h, color *c)
{
    h_to_rgb(h,c);
}

/* 
 * Given a hue 0-252 and a brightness 0-255, set LEDs appropriately
 */
void hvset(uint8_t h, uint8_t v, color *c) 
{
    h_to_rgb(h,c);
    c->r = ((c->r * v) / 255);
    c->g = ((c->g * v) / 255);
    c->b = ((c->b * v) / 255);
}