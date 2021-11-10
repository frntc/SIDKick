/*
   _________.___________   ____  __.__        __     
  /   _____/|   \______ \ |    |/ _|__| ____ |  | __ 
  \_____  \ |   ||    |  \|      < |  |/ ___\|  |/ / 
  /        \|   ||    `   \    |  \|  \  \___|    <  
 /_______  /|___/_______  /____|__ \__|\___  >__|_ \ 
         \/             \/        \/       \/     \/ 
        
 LED_Helpers.h

 SIDKick - SID-replacement with SID, Sound Expander and MIDI Emulation based on Teensy 4.1
           (using reSID by Dag Lem and FMOPL by Jarek Burczynski, Tatsuyuki Satoh, Marco van den Heuvel, and Acho A. Tang)
 Copyright (c) 2019-2021 Carsten Dachsbacher <frenetic@dachsbacher.de>

 Logo created with http://patorjk.com/software/taag/
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

extern uint32_t NUM_LEDS;

extern uint32_t ledWriteRGB[32];

extern uint8_t ledBits2Write;
extern uint8_t ledColors2Write;
extern int32_t ledWriteStatus;
extern uint8_t ledColorsBufOfs;

int32_t nLEDs = 0;
int32_t nLEDsConfigTool = 0;

static uint8_t ledVisMode = 0;
static float last_tL = 0.0f;
static float last_tR = 0.0f;
static uint32_t vuSumL = 0;
static uint32_t vuSumR = 0;
static float vuAvg = 0.0f;
static uint32_t vuMeter = 0;
static uint32_t vu_nValues = 0;
static uint32_t nSamplesCollected = 64*8;
static float scaleSamples = 1.0f;
static float scaleSamplesConfigTool = 1.0f;
static int32_t nLEDsL = 0;
static int32_t nLEDsR = 0;
static uint32_t colorCycle = 0;
static uint32_t ledColorCycleSpeed = 0;
uint8_t collectStatistics = 1;

static uint32_t col0 = 0;
static uint32_t col1 = 0;
static uint32_t hsv0 = 0;
static uint32_t hsv1 = 0;
static uint32_t col0_cycle[ 128 ], col1_cycle[ 128 ];

static uint32_t delayLED = 0;
    
#define H_( hsv ) (((uint8_t*)&hsv)[2])
#define S_( hsv ) (((uint8_t*)&hsv)[1])
#define V_( hsv ) (((uint8_t*)&hsv)[0])

// unusual format of WS2818
#define G_( rgb ) (((uint8_t*)&rgb)[2])
#define R_( rgb ) (((uint8_t*)&rgb)[1])
#define B_( rgb ) (((uint8_t*)&rgb)[0])

#define LRG( r, g ) (((r)<<16)|g)
//#define LR_( rg ) (((rg)>>16)&255)
//#define LG_( rg ) ((rg)&255)

#define RGB( r, g, b ) (((g)<<16)|((r)<<8)|b)
#define HSV( h, s, v ) (((h)<<16)|((s)<<8)|v)

__attribute__( ( always_inline ) ) inline uint32_t SCALE( const uint32_t a, const int32_t t )
{
  int32_t c; uint8_t *p = (uint8_t *)&c;
  p[ 1 ] = (R_(a) * t) >> 8;
  p[ 2 ] = (G_(a) * t) >> 8;
  p[ 0 ] = (B_(a) * t) >> 8;
  return c;
}

__attribute__( ( always_inline ) ) inline uint32_t LERP( uint32_t a, uint32_t b, int32_t t )
{
  int32_t it = 255 - t;
  int32_t c; uint8_t *p = (uint8_t *)&c;
  p[ 1 ] = (R_(a) * t + R_(b) * it) >> 8;
  p[ 2 ] = (G_(a) * t + G_(b) * it) >> 8;
  p[ 0 ] = (B_(a) * t + B_(b) * it) >> 8;
  return c;
}

__attribute__( ( always_inline ) ) inline uint32_t ADD_CLAMP( const uint32_t a, const uint32_t b )
{
  uint32_t result;
  asm volatile ( "uqadd8 %0, %1, %2" : "=r" (result) : "r" (a), "r" (b) );
  return(result);
}

__attribute__( ( always_inline ) ) inline uint32_t HSV2RGB(uint32_t hsv)
{
    int32_t region, remainder, p, q, t;

    if (S_(hsv) == 0)
        return RGB( V_( hsv ), V_( hsv ), V_( hsv ) );

    region = H_( hsv ) / 43;
    remainder = ( H_( hsv ) - (region * 43)) * 6; 

    p = (V_( hsv ) * (255 - S_( hsv ))) / 256;
    q = (V_( hsv ) * (255*256 - ((S_( hsv ) * remainder)))) / 65536;
    t = (V_( hsv ) * (255*256 - ((S_( hsv ) * (255 - remainder))))) / 65536;

    switch (region)
    {
        case 0:
            return RGB( V_( hsv ), t, p );
        case 1:
            return RGB( q, V_( hsv ), p );
        case 2:
            return RGB( p, V_( hsv ), t );
        case 3:
            return RGB( p, q, V_( hsv ) );
        case 4:
            return RGB( t, p, V_( hsv ) );
        default:
            return RGB( V_( hsv ), p, q );
    }
}

__attribute__( ( always_inline ) ) inline uint32_t RGB2HSV( uint32_t rgb )
{
    int32_t rgbMin, rgbMax, h, s, v;

    rgbMax = rgbMin = R_(rgb);
    if ( G_(rgb) > rgbMax ) rgbMax = G_(rgb);
    if ( G_(rgb) < rgbMin ) rgbMin = G_(rgb);
    if ( B_(rgb) > rgbMax ) rgbMax = B_(rgb);
    if ( B_(rgb) < rgbMin ) rgbMin = B_(rgb);
    v = rgbMax;

    if ( v == 0 )
      return 0;

    s = 255 * ((int32_t)(rgbMax - rgbMin)) / v;

    int32_t diff = rgbMax - rgbMin;

    if ( s == 0 )
      return HSV( 0, 0, v );

    if (rgbMax == R_(rgb))
      h = 0 + 43 * ((int32_t)G_(rgb) - (int32_t)B_(rgb)) / diff; else 
      if (rgbMax == G_(rgb))
        h = 85 + 43 * ((int32_t)B_(rgb) - (int32_t)R_(rgb)) / diff; else
        h = 171 + 43 * ((int32_t)R_(rgb) - (int32_t)G_(rgb)) / diff;

    h &= 255;
    return HSV( h, s, v );
}        

//
// https://github.com/ekmett/approximate/blob/master/cbits/fast.c
//
__attribute__( ( always_inline ) ) inline float powf_fast(float a, float b) {
  union { float d; int x; } u = { a };
  u.x = (int)(b * (u.x - 1064866805) + 1064866805);
  return u.d;
}

__attribute__( ( always_inline ) ) inline float powf_fast_lb(float a, float b) {
  union { float d; int x; } u = { a };
  u.x = (int)(b * (u.x - 1065353217) + 1064631197);
  return u.d;
}

__attribute__( ( always_inline ) ) inline float powf_fast_ub(float a, float b) {
  union { float d; int x; } u = { a };
  u.x = (int)(b * (u.x - 1064631197) + 1065353217);
  return u.d;
}
