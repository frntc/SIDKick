//  ---------------------------------------------------------------------------
//  This file is part of reSID, a MOS6581 SID emulator engine.
//  Copyright (C) 2010  Dag Lem <resid@nimrod.no>
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//  ---------------------------------------------------------------------------

// please note: there are some modifications in this file to be used with SIDKick, and also some 
//              alternative solutions to the classic LUTs used in original reSID
//

#define RESID_FILTER_CC

#ifdef _M_ARM
#undef _ARM_WINAPI_PARTITION_DESKTOP_SDK_AVAILABLE
#define _ARM_WINAPI_PARTITION_DESKTOP_SDK_AVAILABLE 1
#endif

#include "filter.h"
#include "dac.h"
#include "spline.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern void *allocPool( int size );

RESID_NAMESPACE_START

#define v1 fstate[ 0 ]
#define v2 fstate[ 1 ]
#define v3 fstate[ 2 ]
#define ve fstate[ 3 ]
#define Vlp fstate[ 4 ]
#define Vbp fstate[ 5 ]
#define Vhp fstate[ 6 ]

#define Vbp_x fstate[ 7 ]
#define Vbp_vc fstate[ 8 ]
#define Vlp_x fstate[ 9 ]
#define Vlp_vc fstate[ 10 ]

#include  <avr/pgmspace.h>

#pragma pack(push,1)
#include "gainmixer.h"
#pragma pack(pop)

FITCURVELUT fgainmixer[ 2 ][ 16 ][ 8 ];
signed short *globCoeff;

#ifdef PRECALC_TABLES
#ifdef HIGH_PRECISION_TABLES
#include "resid_precalc_4_4.h"
#else
//#include "resid_precalc_4_4_16.h"
//#include "resid_precalc_64_64.h"
#include "resid_precalc_128_64.h"
#endif
#endif

// This is the SID 6581 op-amp voltage transfer function, measured on
// CAP1B/CAP1A on a chip marked MOS 6581R4AR 0687 14.
// All measured chips have op-amps with output voltages (and thus input
// voltages) within the range of 0.81V - 10.31V.

static double_point opamp_voltage_6581[] = {
  {  0.81, 10.31 },  // Approximate start of actual range
  {  0.81, 10.31 },  // Repeated point
  {  2.40, 10.31 },
  {  2.60, 10.30 },
  {  2.70, 10.29 },
  {  2.80, 10.26 },
  {  2.90, 10.17 },
  {  3.00, 10.04 },
  {  3.10,  9.83 },
  {  3.20,  9.58 },
  {  3.30,  9.32 },
  {  3.50,  8.69 },
  {  3.70,  8.00 },
  {  4.00,  6.89 },
  {  4.40,  5.21 },
  {  4.54,  4.54 },  // Working point (vi = vo)
  {  4.60,  4.19 },
  {  4.80,  3.00 },
  {  4.90,  2.30 },  // Change of curvature
  {  4.95,  2.03 },
  {  5.00,  1.88 },
  {  5.05,  1.77 },
  {  5.10,  1.69 },
  {  5.20,  1.58 },
  {  5.40,  1.44 },
  {  5.60,  1.33 },
  {  5.80,  1.26 },
  {  6.00,  1.21 },
  {  6.40,  1.12 },
  {  7.00,  1.02 },
  {  7.50,  0.97 },
  {  8.50,  0.89 },
  { 10.00,  0.81 },
  { 10.31,  0.81 },  // Approximate end of actual range
  { 10.31,  0.81 }   // Repeated end point
};

// This is the SID 8580 op-amp voltage transfer function, measured on
// CAP1B/CAP1A on a chip marked CSG 8580R5 1690 25.
static double_point opamp_voltage_8580[] = {
  {  1.30,  8.91 },  // Approximate start of actual range
  {  1.30,  8.91 },  // Repeated end point
  {  4.76,  8.91 },
  {  4.77,  8.90 },
  {  4.78,  8.88 },
  {  4.785, 8.86 },
  {  4.79,  8.80 },
  {  4.795, 8.60 },
  {  4.80,  8.25 },
  {  4.805, 7.50 },
  {  4.81,  6.10 },
  {  4.815, 4.05 },  // Change of curvature
  {  4.82,  2.27 },
  {  4.825, 1.65 },
  {  4.83,  1.55 },
  {  4.84,  1.47 },
  {  4.85,  1.43 },
  {  4.87,  1.37 },
  {  4.90,  1.34 },
  {  5.00,  1.30 },
  {  5.10,  1.30 },
  {  8.91,  1.30 },  // Approximate end of actual range
  {  8.91,  1.30 }   // Repeated end point
};


/*
 * R1 = 15.3*Ri
 * R2 =  7.3*Ri
 * R3 =  4.7*Ri
 * Rf =  1.4*Ri
 * R4 =  1.4*Ri
 * R8 =  2.0*Ri
 * RC =  2.8*Ri
 *
 * res  feedback  input
 * ---  --------  -----
 *  0   Rf        Ri
 *  1   Rf|R1     Ri
 *  2   Rf|R2     Ri
 *  3   Rf|R3     Ri
 *  4   Rf        R4
 *  5   Rf|R1     R4
 *  6   Rf|R2     R4
 *  7   Rf|R3     R4
 *  8   Rf        R8
 *  9   Rf|R1     R8
 *  A   Rf|R2     R8
 *  B   Rf|R3     R8
 *  C   Rf        RC
 *  D   Rf|R1     RC
 *  E   Rf|R2     RC
 *  F   Rf|R3     RC
 */
#ifndef PRECALC_TABLES
static int resGain[16] =
{
  (int)((1<<7)*(1.4/1.0)),                     //      Rf/Ri   1.4
  (int)((1<<7)*(((1.4*15.3)/(1.4+15.3))/1.0)), // (Rf|R1)/Ri   1.28263
  (int)((1<<7)*(((1.4*7.3)/(1.4+7.3))/1.0)),   // (Rf|R2)/Ri   1.17471
  (int)((1<<7)*(((1.4*4.7)/(1.4+4.7))/1.0)),   // (Rf|R3)/Ri   1.07869
  (int)((1<<7)*(1.4/1.4)),                     //      Rf/R4   1.0
  (int)((1<<7)*(((1.4*15.3)/(1.4+15.3))/1.4)), // (Rf|R1)/R4   0.916168
  (int)((1<<7)*(((1.4*7.3)/(1.4+7.3))/1.4)),   // (Rf|R2)/R4   0.83908
  (int)((1<<7)*(((1.4*4.7)/(1.4+4.7))/1.4)),   // (Rf|R3)/R4   0.770492
  (int)((1<<7)*(1.4/2.0)),                     //      Rf/R8   0.7
  (int)((1<<7)*(((1.4*15.3)/(1.4+15.3))/2.0)), // (Rf|R1)/R8   0.641317
  (int)((1<<7)*(((1.4*7.3)/(1.4+7.3))/2.0)),   // (Rf|R2)/R8   0.587356
  (int)((1<<7)*(((1.4*4.7)/(1.4+4.7))/2.0)),   // (Rf|R3)/R8   0.539344
  (int)((1<<7)*(1.4/2.8)),                     //      Rf/RC   0.5
  (int)((1<<7)*(((1.4*15.3)/(1.4+15.3))/2.8)), // (Rf|R1)/RC   0.458084
  (int)((1<<7)*(((1.4*7.3)/(1.4+7.3))/2.8)),   // (Rf|R2)/RC   0.41954
  (int)((1<<7)*(((1.4*4.7)/(1.4+4.7))/2.8)),   // (Rf|R3)/RC   0.385246
};
#endif

typedef struct {
  // Op-amp transfer function.
  double_point* opamp_voltage;
  int opamp_voltage_size;
  // Voice output characteristics.
  FLOAT voice_voltage_range;
  FLOAT voice_DC_voltage;
  // Capacitor value.
  FLOAT C;
  // Transistor parameters.
  FLOAT Vdd;
  FLOAT Vth;        // Threshold voltage
  FLOAT Ut;         // Thermal voltage: Ut = k*T/q = 8.61734315e-5*T ~ 26mV
  FLOAT k;          // Gate coupling coefficient: K = Cox/(Cox+Cdep) ~ 0.7
  FLOAT uCox;       // u*Cox
  FLOAT WL_vcr;     // W/L for VCR
  FLOAT WL_snake;   // W/L for "snake"
  // DAC parameters.
  FLOAT dac_zero;
  FLOAT dac_scale;
  FLOAT dac_2R_div_R;
  bool dac_term;
} model_filter_init_t;

static model_filter_init_t model_filter_init[2] = {
  {
    opamp_voltage_6581,
    sizeof(opamp_voltage_6581)/sizeof(*opamp_voltage_6581),
    // The dynamic analog range of one voice is approximately 1.5V,
    1.5,
    // riding at a DC level of approximately 5.0V.
    5.0,
    // Capacitor value.
    470e-12,
    // Transistor parameters.
    12.18,
    1.31,
    26.0e-3,
    1.0,
    20e-6,
    9.0/1.0,
    1.0/115,
    // DAC parameters.
    6.65,
    2.63,
    2.20,
    false
  },
  {
    opamp_voltage_8580,
    sizeof(opamp_voltage_8580)/sizeof(*opamp_voltage_8580),
    // FIXME: Measure for the 8580.
    0.3,
    // The 4.75V voltage for the virtual ground is generated by a PolySi resistor divider
    4.75,
    // Capacitor value.
    22e-9,
    // Transistor parameters.
    9.09,
    0.80,
    26.0e-3,
    1.0,
    10e-6,
    // FIXME: 6581 only
    0,
    0,
    0,
    0,
    2.00,
    true
  }
};

#ifndef PRECALC_TABLES
 unsigned short  Filter::resonance[16][(1 << 16)/SUB_SAMPLE_PRE];
 unsigned short  Filter::vcr_kVg[(1 << 16)/SUB_SAMPLE_PRE];
 unsigned short  Filter::vcr_n_Ids_term[(1 << 16)/SUB_SAMPLE_PRE];
#endif
int Filter::n_snake;
int Filter::n_param;

//#if defined(__amiga__) && defined(__mc68000__)
//#undef HAS_LOG1P
//#endif

//#ifndef HAS_LOG1P
/*static FLOAT log1p(FLOAT x)
{
    return log(1 + x) - (((1 + x) - 1) - x) / (1 + x);
}*/
//#endif

Filter::model_filter_t Filter::model_filter[2];

// ----------------------------------------------------------------------------
// Destructor.
// ----------------------------------------------------------------------------
Filter::~Filter()
{
}

#if SUB_SAMPLE_PRE == 4
unsigned short *opamp_rev0;
unsigned short *opamp_rev1;
unsigned short *vcr_kVg_precalc;
unsigned short *vcr_n_Ids_term_precalc;
//unsigned short *summer0;
//unsigned short *summer1;
#endif

// ----------------------------------------------------------------------------
// Constructor.
// ----------------------------------------------------------------------------
Filter::Filter()
{
  //static bool class_init = 0;
  mixerInput = gainInput = 0;

  //if (!class_init) 
  {
    #if SUB_SAMPLE_PRE == 4
  static int doOnlyOnce2 = 1;

  if ( doOnlyOnce2 )
  {
    doOnlyOnce2 = 0;
    opamp_rev0 = new unsigned short[ 16384 ];
    memcpy( opamp_rev0, opamp_rev0_slow, sizeof( unsigned short ) * 16384 );
    opamp_rev1 = new unsigned short[ 16384 ];
    memcpy( opamp_rev1, opamp_rev1_slow, sizeof( unsigned short ) * 16384 );
    vcr_kVg_precalc = new unsigned short[ 16384 ];
    memcpy( vcr_kVg_precalc, vcr_kVg_precalc_slow, sizeof( unsigned short ) * 16384 );
    vcr_n_Ids_term_precalc = new unsigned short[ 16384 ];
    memcpy( vcr_n_Ids_term_precalc, vcr_n_Ids_term_precalc_slow, sizeof( unsigned short ) * 16384 );
  }
/*    gain0 = new unsigned short[ ( 16 * 16384 ) / 4 ];
    gain1 = new unsigned short[ ( 16 * 16384 ) / 4 ];

    for ( int v = 0; v < 16; v++ )
    {
      for ( int i = 0; i < 16384 / 4; i++ )
      {
        gain0[ v * (16384 / 4) + i ] = gain0_slow[ v ][ i * 4 ];
        gain1[ v * (16384 / 4) + i ] = gain1_slow[ v ][ i * 4 ];
      }
    }*/

    /*summer0 = new unsigned short[ 327680 / 4 ];
    summer1 = new unsigned short[ 327680 / 4 ];
    if ( summer0 != NULL && summer1 != NULL )
    {
      for ( int i = 0; i < 327680 / 4; i++ )
      {
        summer0[ i ] = ( (int)summer0_slow[ i * 4 ] + (int)summer0_slow[ i * 4 + 1 ] + (int)summer0_slow[ i * 4 + 2 ] + (int)summer0_slow[ i * 4 + 3 ] ) / 4;
        summer1[ i ] = ( (int)summer1_slow[ i * 4 ] + (int)summer1_slow[ i * 4 + 1 ] + (int)summer1_slow[ i * 4 + 2 ] + (int)summer1_slow[ i * 4 + 3 ] ) / 4;
      }
    } else
    {
      summer0 = summer0_slow;
      summer1 = summer1_slow;
    }*/
    #endif

    FLOAT tmp_n_param[2];

    // Temporary tables for op-amp transfer function.
#ifndef PRECALC_TABLES
    unsigned int voltages[1 << 16];
    opamp_t opamp[(1 << 16)/SUB_SAMPLE_PRE];
#else
    //const unsigned int *voltages; 
#endif

    for (int m = 0; m < 2; m++) 
	{
      model_filter_init_t& fi = model_filter_init[m];
      model_filter_t& mf = model_filter[m];

      // Convert op-amp voltage transfer to 16 bit values.
      FLOAT vmin = fi.opamp_voltage[0][0];
      FLOAT opamp_max = fi.opamp_voltage[0][1];
      FLOAT kVddt = fi.k*(fi.Vdd - fi.Vth);
      FLOAT vmax = kVddt < opamp_max ? opamp_max : kVddt;
      FLOAT denorm = vmax - vmin;
      FLOAT norm = 1.0f/denorm;

      // Scaling and translation constants.
      FLOAT N16 = norm*((1u << 16) - 1);
      FLOAT N30 = norm*((1u << 30) - 1);
      //FLOAT N31 = norm*((1u << 31) - 1);
      mf.vo_N16 = N16;

      // The "zero" output level of the voices.
      // The digital range of one voice is 20 bits; create a scaling term
      // for multiplication which fits in 11 bits.
      FLOAT N14 = norm*(1u << 14);
      mf.voice_scale_s14 = (int)(N14*fi.voice_voltage_range);
      mf.voice_DC = (int)(N16*(fi.voice_DC_voltage - vmin));

      // Vdd - Vth, normalized so that translated values can be subtracted:
      // k*Vddt - x = (k*Vddt - t) - (x - t)
      mf.kVddt = (int)(N16*(kVddt - vmin) + 0.5);

      tmp_n_param[m] = denorm*(1 << 13)*(fi.uCox/(2.0f*fi.k)*1.0e-6f/fi.C);

#ifndef PRECALC_TABLES

      // Create lookup table mapping op-amp voltage across output and input
      // to input voltage: vo - vx -> vx
      // FIXME: No variable length arrays in ISO C++, hardcoding to max 50
      // points.
      // double_point scaled_voltage[fi.opamp_voltage_size];

      double_point scaled_voltage[50];

      for (int i = 0; i < fi.opamp_voltage_size; i++) {
        // The target output range is 16 bits, in order to fit in an unsigned
        // short.
        //
        // The y axis is temporarily scaled to 31 bits for maximum accuracy in
        // the calculated derivative.
        //
        // Values are normalized using
        //
        //   x_n = m*2^N*(x - xmin)
        //
        // and are translated back later (for fixed point math) using
        //
        //   m*2^N*x = x_n - m*2^N*xmin
        //
        scaled_voltage[fi.opamp_voltage_size - 1 - i][0] = int((N16*(fi.opamp_voltage[i][1] - fi.opamp_voltage[i][0]) + (1 << 16))/2 + 0.5);
        scaled_voltage[fi.opamp_voltage_size - 1 - i][1] = N31*(fi.opamp_voltage[i][0] - vmin);
      }

      // Clamp x to 16 bits (rounding may cause overflow).
      if (scaled_voltage[fi.opamp_voltage_size - 1][0] >= (1 << 16)) {
        // The last point is repeated.
        scaled_voltage[fi.opamp_voltage_size - 1][0] =
            scaled_voltage[fi.opamp_voltage_size - 2][0] = (1 << 16) - 1;
      }
      interpolate(scaled_voltage, scaled_voltage + fi.opamp_voltage_size - 1,
                    PointPlotter<unsigned int>(voltages), 1.0);
#else
      if ( m == 0 )
      {
        mf.ak = 1825; mf.bk = 63711;
        //voltages = voltages0; 
      } else
      {
        mf.ak = 1; mf.bk = 63535;
        //voltages = voltages1; 
      }
#endif

#ifndef PRECALC_TABLES
      // Store both fn and dfn in the same table.
      mf.ak = (int)scaled_voltage[0][0];
      mf.bk = (int)scaled_voltage[fi.opamp_voltage_size - 1][0];

      int j;
      for (j = 0; j < mf.ak; j++) {
        opamp[j/SUB_SAMPLE_PRE].vx = 0;
        opamp[j/SUB_SAMPLE_PRE].dvx = 0;
      }
      unsigned int f = voltages[j];
      for (; j <= mf.bk; j++) {
        unsigned int fp = f;
        f = voltages[j];  // Scaled by m*2^31
        // m*2^31*dy/1 = (m*2^31*dy)/(m*2^16*dx) = 2^15*dy/dx
        int df = f - fp;  // Scaled by 2^15

        // 16 bits unsigned: m*2^16*(fn - xmin)
        opamp[j/SUB_SAMPLE_PRE].vx = f > (0xffff << 15) ? 0xffff : f >> 15;
        // 16 bits (15 bits + sign bit): 2^11*dfn
        opamp[j/SUB_SAMPLE_PRE].dvx = df >> (15 - 11);
      }
      for (; j < (1 << 16); j++) {
        opamp[j/SUB_SAMPLE_PRE].vx = 0;
        opamp[j/SUB_SAMPLE_PRE].dvx = 0;
      }

      // We don't have the differential for the first point so just assume
      // it's the same as the second point's
      opamp[mf.ak/SUB_SAMPLE_PRE].dvx = opamp[(mf.ak)/SUB_SAMPLE_PRE+1].dvx;
#endif


      // Create lookup tables for gains / summers.

      // 4 bit "resistor" ladders in the bandpass resonance gain and the audio
      // output gain necessitate 16 gain tables.
      // From die photographs of the bandpass and volume "resistor" ladders
      // it follows that gain ~ vol/8 and 1/Q ~ ~res/8 (assuming ideal
      // op-amps and ideal "resistors").
      for (int n8 = 0; n8 < 16; n8++) {
#ifndef PRECALC_TABLES
        int n = n8 << 4;  // Scaled by 2^7
        int x = mf.ak;
        for (int vi = 0; vi < (1 << 16); vi+=SUB_SAMPLE_GAIN) {
          mf.gain[n8][vi/SUB_SAMPLE_GAIN] = solve_gain(opamp, n, vi, x, mf);
        }
#else
        if ( m == 0 )
          mf.gain[n8] = &gain0[n8][0]; else
          mf.gain[n8] = &gain1[n8][0];
        /*if ( m == 0 )
          mf.gain[n8] = &gain0[n8*(16384/4)]; else
          mf.gain[n8] = &gain1[n8*(16384/4)];*/
#endif
      }

      //mf.summer = new unsigned short[ 1310720 ];
      //mf.mixer = new unsigned short[ 1835009 ];

      // The filter summer operates at n ~ 1, and has 5 fundamentally different
      // input configurations (2 - 6 input "resistors").
      //
      // Note that all "on" transistors are modeled as one. This is not
      // entirely accurate, since the input for each transistor is different,
      // and transistors are not linear components. However modeling all
      // transistors separately would be extremely costly.
      int offset = 0;
      int size;
/*      for (int k = 0; k < 5; k++) {
        int idiv = 2 + k;        // 2 - 6 input "resistors".
        int n_idiv = idiv << 7;  // n*idiv, scaled by 2^7
        size = idiv << 16;
        int x = mf.ak;
        for (int vi = 0; vi < size; vi+=1+0*SUB_SAMPLE_PRE) {
          mf.summer[(offset + vi)/SUB_SAMPLE_PRE] =
            solve_gain(opamp, n_idiv, vi/idiv, x, mf);
        }
        offset += size;
      }*/
      for (int k = 0; k < 5; k++) {
#ifndef PRECALC_TABLES
        int idiv = 2 + k;        // 2 - 6 input "resistors".
        int n_idiv = idiv << 7;  // n*idiv, scaled by 2^7
        size = idiv << 16;
        int x = mf.ak;
        for (int vi = 0; vi < size; vi++) {
          mf.summer[(offset + vi)] =
            solve_gain(opamp, n_idiv, vi/idiv, x, mf);
        }
#else
        if ( m == 0 )
          mf.summer = &summer0[0]; else
          mf.summer = &summer1[0];
#endif

        offset += size;
      }

      // The audio mixer operates at n ~ 8/6, and has 8 fundamentally different
      // input configurations (0 - 7 input "resistors").
      //
      // All "on", transistors are modeled as one - see comments above for
      // the filter summer.
      offset = 0;
      size = 1;  // Only one lookup element for 0 input "resistors".
      for (int l = 0; l < 8; l++) {
#ifndef PRECALC_TABLES
        int idiv = l;                 // 0 - 7 input "resistors".
        int n_idiv = (idiv << 7)*8/6; // n*idiv, scaled by 2^7
        if (idiv == 0) {
          // Avoid division by zero; the result will be correct since
          // n_idiv = 0.
          idiv = 1;
        }
        int x = mf.ak;
        for (int vi = 0; vi < size; vi+=SUB_SAMPLE_PRE) 
        //if ( (offset + vi) < 1835009 )
        {
          mf.mixer[(offset + vi)/SUB_SAMPLE_PRE] = 
            solve_gain(opamp, n_idiv, vi/idiv, x, mf);
        }
#else
        /*if ( m == 0 )
          mf.mixer = &mixer0[0]; else
          mf.mixer = &mixer1[0];*/
#endif

        offset += size;
        size = (l + 1) << 16;
      }

#ifndef PRECALC_TABLES
      for (int vi = 0; vi < 1835009/SUB_SAMPLE_PRE; vi++) 
      {
          mf.mixer[vi] = mf.mixer[vi*SUB_SAMPLE_PRE];
      }
#endif

      // Create lookup table mapping capacitor voltage to op-amp input voltage:
      // vc -> vx
#ifndef PRECALC_TABLES
      for (int m = 0; m < (1 << 16); m+=SUB_SAMPLE_PRE) {
        mf.opamp_rev[(m)/SUB_SAMPLE_PRE] = opamp[m/SUB_SAMPLE_PRE].vx;
      }
#else      
        if ( m == 0 )
          mf.opamp_rev = &opamp_rev0[0]; else
          mf.opamp_rev = &opamp_rev1[0];
#endif

      mf.vc_max = (int)(N30*(fi.opamp_voltage[0][1] - fi.opamp_voltage[0][0]));
      mf.vc_min = (int)(N30*(fi.opamp_voltage[fi.opamp_voltage_size - 1][1] - fi.opamp_voltage[fi.opamp_voltage_size - 1][0]));
    }

    // Free temporary table.
    //delete[] voltages;

#ifndef PRECALC_TABLES
    unsigned int dac_bits = 11;
#endif

    {
      // 8580 only
      for (int n8 = 0; n8 < 16; n8++) {
#ifndef PRECALC_TABLES
        int x = model_filter[1].ak;
        for (int vi = 0; vi < (1 << 16); vi+=SUB_SAMPLE_PRE) {
          resonance[n8][vi/SUB_SAMPLE_PRE] = solve_gain(opamp, resGain[n8], vi, x, model_filter[1]);
        }
#else
        resonance[n8] = &resonance_precalc[n8][0];
#endif
      }

      // scaled 5 bits
      n_param = (int)(tmp_n_param[1] * 32 + 0.5);

      model_filter_init_t& fi = model_filter_init[1];
      model_filter_t& f = model_filter[1];

      FLOAT Vgt = fi.k * ((4.75 * 1.6) - fi.Vth);
      kVgt = (int)(f.vo_N16 * (Vgt - fi.opamp_voltage[0][0]) + 0.5);

      // DAC table.
      // W/L ratio for frequency DAC, bits are proportional.
      // scaled 5 bits
#ifndef PRECALC_TABLES
      unsigned short dacWL = 1; // 0.03125 * 32 FIXME actual value is ~= 0.003075
      f.f0_dac[0] = dacWL;
      for (int n = 1; n < (1 << dac_bits); n++) {
        // Calculate W/L ratio for parallel NMOS resistances
        unsigned short wl = 0;
        for (unsigned int i = 0; i < dac_bits; i++) {
          unsigned int bitmask = 1 << i;
          if (n & bitmask) {
            wl += dacWL * (bitmask<<1);
          }
        }
        f.f0_dac[n] = wl;
      }
  #else
      f.f0_dac = &f0_dac1[0];
  #endif





    }

    // Free temporary table.
    //delete[] opamp;

    {
      // 6581 only
      model_filter_t& f = model_filter[0];
      // DAC table.
#ifndef PRECALC_TABLES
      model_filter_init_t& fi = model_filter_init[0];

      FLOAT N16 = f.vo_N16;
      FLOAT vmin = fi.opamp_voltage[0][0];

      Vw_bias = 0;

      // Normalized snake current factor, 1 cycle at 1MHz.
      // Fit in 5 bits.
      n_snake = (int)(fi.WL_snake * tmp_n_param[0] + 0.5);

    build_dac_table(f.f0_dac, dac_bits, fi.dac_2R_div_R, fi.dac_term);
      for (int n = 0; n < (1 << dac_bits); n++) {
        f.f0_dac[n] = (unsigned short)(N16*(fi.dac_zero + f.f0_dac[n]*fi.dac_scale/(1 << dac_bits) - vmin) + 0.5);
      }
#else
      f.f0_dac = &f0_dac0[0];
#endif

      // VCR table.

#ifndef PRECALC_TABLES
      FLOAT k = fi.k;
      FLOAT kVddt = N16*(k*(fi.Vdd - fi.Vth));
      vmin *= N16;

      for (int i = 0; i < (1 << 16); i+=SUB_SAMPLE_PRE) {
        // The table index is right-shifted 16 times in order to fit in
        // 16 bits; the argument to sqrt is thus multiplied by (1 << 16).
        //
        // The returned value must be corrected for translation. Vg always
        // takes part in a subtraction as follows:
        //
        //   k*Vg - Vx = (k*Vg - t) - (Vx - t)
        //
        // I.e. k*Vg - t must be returned.
        FLOAT Vg = kVddt - sqrt((FLOAT)i*(1 << 16));
        vcr_kVg[i/SUB_SAMPLE_PRE] = (unsigned short)(k*Vg - vmin + 0.5);
      }
#else      
      vcr_kVg = &vcr_kVg_precalc[0];
#endif

      /*
        EKV model:

        Ids = Is*(if - ir)
        Is = 2*u*Cox*Ut^2/k*W/L
        if = ln^2(1 + e^((k*(Vg - Vt) - Vs)/(2*Ut))
        ir = ln^2(1 + e^((k*(Vg - Vt) - Vd)/(2*Ut))
      */

#ifndef PRECALC_TABLES
      FLOAT kVt = fi.k*fi.Vth;
      FLOAT Ut = fi.Ut;
      FLOAT Is = 2*fi.uCox*Ut*Ut/fi.k*fi.WL_vcr;
      // Normalized current factor for 1 cycle at 1MHz.
      FLOAT N15 = N16/2;
      FLOAT n_Is = N15*1.0e-6/fi.C*Is;

      // kVg_Vx = k*Vg - Vx
      // I.e. if k != 1.0, Vg must be scaled accordingly.

      for (int kVg_Vx = 0; kVg_Vx < (1 << 16); kVg_Vx+=SUB_SAMPLE_PRE) {
        FLOAT log_term = log1p(exp((kVg_Vx/N16 - kVt)/(2*Ut)));
        // Scaled by m*2^15
        vcr_n_Ids_term[kVg_Vx/SUB_SAMPLE_PRE] = (unsigned short)(n_Is*log_term*log_term);
      }
#else
      vcr_n_Ids_term = &vcr_n_Ids_term_precalc[0];
  #endif

    }

    //
    // convert globCoeff indices into pointers!
    //
    static int doOnlyOnce = 1;

    if ( doOnlyOnce )
    {
      doOnlyOnce = 0;

      globCoeff = (short int *)allocPool( sizeof( signed short ) * 21957 );
      memcpy( globCoeff, globCoeff_slow, sizeof( signed short ) * 21957 );

      for ( int m = 0; m < 2; m++ )
          for ( int vol = 0; vol < 16; vol ++ )
          {
            memcpy( &fgainmixer[ m ][ vol ][ 0 ], &fgainmixer_slow[ m ][ vol ][ 0 ], sizeof( FITCURVELUT ) * 8 );
            for ( int j = 0; j < 8; j++ )
              fgainmixer[ m ][ vol ][ j ].coeffsIdx = (unsigned long)&globCoeff[ fgainmixer[ m ][ vol ][ j ].coeffsIdx ];
          }
    }
    //class_init = true;
  }


#if 0

  unsigned int totalMemory = 0;


	printf( "const unsigned short resonance_precalc[ 16 ][ %d ] PROGMEM = {\n", 65536/SUB_SAMPLE_PRE );
	int br = 0;
	for ( int j = 0; j < 16; j++ )
	{
		printf( "{ " );
		for ( int i = 0; i < 65536/SUB_SAMPLE_PRE; i += 1 )
		{
			totalMemory += 2;
			printf( "%d", resonance[ j ][ i ] );
			if ( i != ( 65536/SUB_SAMPLE_PRE - 1 ) )
				printf( ", " );
			if ( ++br == 24 ) {
				printf( "\n" );
				br = 0;
			}

		}
		if ( j < 15 )
			printf( "},\n " ); else
			printf( "}\n " );
	}
	printf( "};\n\n" );

		printf( "const unsigned short vcr_kVg_precalc[ %d ] PROGMEM = {\n", 65536/SUB_SAMPLE_PRE );
		br = 0;
		for ( int i = 0; i < 65536/SUB_SAMPLE_PRE; i += 1 )
		{
			totalMemory += 2;
			printf( "%d", vcr_kVg[ i ] );
			if ( i != ( 65536/SUB_SAMPLE_PRE - 1 ) )
				printf( ", " );
			if ( ++br == 24 ) {
				printf( "\n" );
				br = 0;
			}

		}
		printf( "};\n\n" );		
		
		printf( "const unsigned short vcr_n_Ids_term_precalc[ %d ] PROGMEM = {\n", 65536/SUB_SAMPLE_PRE );
		br = 0;
		for ( int i = 0; i < 65536/SUB_SAMPLE_PRE; i += 1 )
		{
			totalMemory += 2;
			printf( "%d", vcr_n_Ids_term[ i ] );
			if ( i != ( 65536/SUB_SAMPLE_PRE - 1 ) )
				printf( ", " );
			if ( ++br == 24 ) {
				printf( "\n" );
				br = 0;
			}

		}
		printf( "};\n\n" );

	for ( int m = 0; m < 2; m++ )
	{
		model_filter_init_t& fi = model_filter_init[ m ];
		model_filter_t& mf = model_filter[ m ];

 
		int br = 0;

		printf( "const unsigned short opamp_rev%d[ %d ] PROGMEM = {\n", m, 65536/SUB_SAMPLE_PRE );
		for ( int i = 0; i < 65536/SUB_SAMPLE_PRE; i += 1 )
		{
			totalMemory += 2;
			printf( "%d", mf.opamp_rev[ i ] );
			if ( i != ( 65536/SUB_SAMPLE_PRE - 1 ) )
				printf( ", " );
			if ( ++br == 24 ) {
				printf( "\n" );
				br = 0;
			}

		}
		printf( "};\n\n" );

      //mf.summer = new unsigned short[ 1310720 ];
      //mf.mixer = new unsigned short[ 1835009 ];


		printf( "const unsigned short summer%d[ %d ] PROGMEM = {\n", m, 1310720/SUB_SAMPLE_PRE );
		br = 0;
		for ( int i = 0; i < 1310720/SUB_SAMPLE_PRE; i += 1 )
		{
			totalMemory += 2;
			printf( "%d", mf.summer[ i ] );
			if ( i != ( 1310720/SUB_SAMPLE_PRE - 1 ) )
				printf( ", " );
			if ( ++br == 24 ) {
				printf( "\n" );
				br = 0;
			}

		}
		printf( "};\n\n" );


		printf( "const unsigned short gain%d[ 16 ][ %d ] PROGMEM = {\n", m, 65536/SUB_SAMPLE_GAIN );
		br = 0;
		for ( int j = 0; j < 16; j++ )
		{
			printf( "{ " );
			for ( int i = 0; i < 65536/SUB_SAMPLE_GAIN; i += 1 )
			{
				totalMemory += 2;
				printf( "%d", mf.gain[ j ][ i ] );
				if ( i != ( 65536/SUB_SAMPLE_GAIN - 1 ) )
					printf( ", " );
				if ( ++br == 24 ) {
					printf( "\n" );
					br = 0;
				}

			}
			if ( j < 15 )
				printf( "},\n " ); else
				printf( "}\n " );
		}
		printf( "};\n\n" );



		printf( "const unsigned short mixer%d[ %d ] PROGMEM = {\n", m, 1835009/SUB_SAMPLE_PRE );
		br = 0;
		for ( int i = 0; i < 1835009/SUB_SAMPLE_PRE; i += 1 )
		{
			totalMemory += 2;
			printf( "%d", mf.mixer[ i ] );
			if ( i != ( 1835009/SUB_SAMPLE_PRE - 1 ) )
				printf( ", " );
			if ( ++br == 24 ) {
				printf( "\n" );
				br = 0;
			}

		}
		printf( "};\n\n" );


		printf( "const unsigned short f0_dac%d[ 2048 ] PROGMEM = {\n", m );
		br = 0;
		for ( int i = 0; i < 2048; i += 1 )
		{
			totalMemory += 2;
			printf( "%d", mf.f0_dac[ i ] );
			if ( i != ( 2048 - 1 ) )
				printf( ", " );
			if ( ++br == 24 ) {
				printf( "\n" );
				br = 0;
			}

		}
		printf( "};\n\n" );


		printf( "// total memory: %d bytes\n", totalMemory );
	}
#endif


  enable_filter(true);
  set_chip_model(MOS6581);
  set_voice_mask(0x07);
  input(0);
  reset();
}


// ----------------------------------------------------------------------------
// Enable filter.
// ----------------------------------------------------------------------------
void Filter::enable_filter(bool enable)
{
  enabled = enable;
  set_sum_mix();
}


// ----------------------------------------------------------------------------
// Adjust the DAC bias parameter of the filter.
// This gives user variable control of the exact CF -> center frequency
// mapping used by the filter.
// ----------------------------------------------------------------------------
void Filter::adjust_filter_bias(FLOAT dac_bias)
{
  Vw_bias = int(dac_bias*model_filter[0].vo_N16);
  set_w0();

  // Gate voltage is controlled by the switched capacitor voltage divider
  // Ua = Ue * v = 4.75v  1<v<2
  FLOAT Vg = 4.75 * (dac_bias*6./100. + 1.6);
  FLOAT Vgt = model_filter_init[1].k * (Vg - model_filter_init[1].Vth);
  FLOAT vmin = model_filter_init[1].opamp_voltage[0][0];

  // Vg - Vth, normalized so that translated values can be subtracted:
  // k*Vgt - x = (k*Vgt - t) - (x - t)
  kVgt = (int)(model_filter[1].vo_N16 * (Vgt - vmin) + 0.5);
}

// ----------------------------------------------------------------------------
// Set chip model.
// ----------------------------------------------------------------------------
void Filter::set_chip_model(chip_model model)
{
  sid_model = model;
  /* We initialize the state variables again just to make sure that
   * the earlier model didn't leave behind some foreign, unrecoverable
   * state. Hopefully set_chip_model() only occurs simultaneously with
   * reset(). */
  Vhp = 0;
  Vbp = Vbp_x = Vbp_vc = 0;
  Vlp = Vlp_x = Vlp_vc = 0;

  prefetchMixer();
}


// ----------------------------------------------------------------------------
// Mask for voices routed into the filter / audio output stage.
// Used to physically connect/disconnect EXT IN, and for test purposes
// (voice muting).
// ----------------------------------------------------------------------------
void Filter::set_voice_mask(reg4 mask)
{
  voice_mask = 0xf0 | (mask & 0x0f);
  set_sum_mix();
}


// ----------------------------------------------------------------------------
// SID reset.
// ----------------------------------------------------------------------------
void Filter::reset()
{
  fc = 0;
  res = 0;
  filt = 0;
  mode = 0;
  vol = 0;

  Vhp = 0;
  Vbp = Vbp_x = Vbp_vc = 0;
  Vlp = Vlp_x = Vlp_vc = 0;

  set_w0();
  set_Q();
  set_sum_mix();
}


// ----------------------------------------------------------------------------
// Register functions.
// ----------------------------------------------------------------------------
void Filter::writeFC_LO(reg8 fc_lo)
{
  fc = (fc & 0x7f8) | (fc_lo & 0x007);
  set_w0();
}

void Filter::writeFC_HI(reg8 fc_hi)
{
  fc = ((fc_hi << 3) & 0x7f8) | (fc & 0x007);
  set_w0();
}

void Filter::writeRES_FILT(reg8 res_filt)
{
  res = (res_filt >> 4) & 0x0f;
  set_Q();

  filt = res_filt & 0x0f;
  set_sum_mix();
}

void Filter::writeMODE_VOL(reg8 mode_vol)
{
  mode = mode_vol & 0xf0;
  set_sum_mix();

  vol = mode_vol & 0x0f;
  prefetchMixer();

}

// Set filter cutoff frequency.
void Filter::set_w0()
{
  {
    // MOS 6581
    model_filter_t& f = model_filter[0];
    int Vw = Vw_bias + f.f0_dac[fc];
    Vddt_Vw_2 = unsigned(f.kVddt - Vw)*unsigned(f.kVddt - Vw) >> 1;
  }

  {
    // MOS 8580 cutoff: 0 - 12.5kHz.
    model_filter_t& f = model_filter[1];
    n_dac = (n_param * f.f0_dac[fc]) >> 10;
  }
}

/*
Set filter resonance.

In the MOS 6581, 1/Q is controlled linearly by res. From die photographs
of the resonance "resistor" ladder it follows that 1/Q ~ ~res/8
(assuming an ideal op-amp and ideal "resistors"). This implies that Q
ranges from 0.533 (res = 0) to 8 (res = E). For res = F, Q is actually
theoretically unlimited, which is quite unheard of in a filter
circuit.

To obtain Q ~ 1/sqrt(2) = 0.707 for maximally flat frequency response,
res should be set to 4: Q = 8/~4 = 8/11 = 0.7272 (again assuming an ideal
op-amp and ideal "resistors").

Q as low as 0.707 is not achievable because of low gain op-amps; res = 0
should yield the flattest possible frequency response at Q ~ 0.8 - 1.0
in the op-amp's pseudo-linear range (high amplitude signals will be
clipped). As resonance is increased, the filter must be clocked more
often to keep it stable.

In the MOS 8580, the resonance "resistor" ladder above the bp feedback
op-amp is split in two parts; one ladder for the op-amp input and one
ladder for the op-amp feedback.

input:         feedback:

               Rf
Ri R4 RC R8    R3
               R2
               R1


The "resistors" are switched in as follows by bits in register $17:

feedback:
R1: bit4&!bit5
R2: !bit4&bit5
R3: bit4&bit5
Rf: always on

input:
R4: bit6&!bit7
R8: !bit6&bit7
RC: bit6&bit7
Ri: !(R4|R8|RC) = !(bit6|bit7) = !bit6&!bit7


The relative "resistor" values are approximately (using channel length):

R1 = 15.3*Ri
R2 =  7.3*Ri
R3 =  4.7*Ri
Rf =  1.4*Ri
R4 =  1.4*Ri
R8 =  2.0*Ri
RC =  2.8*Ri


Approximate values for 1/Q can now be found as follows (assuming an
ideal op-amp):

res  feedback  input  -gain (1/Q)
---  --------  -----  ----------
 0   Rf        Ri     Rf/Ri      = 1/(Ri*(1/Rf))      = 1/0.71
 1   Rf|R1     Ri     (Rf|R1)/Ri = 1/(Ri*(1/Rf+1/R1)) = 1/0.78
 2   Rf|R2     Ri     (Rf|R2)/Ri = 1/(Ri*(1/Rf+1/R2)) = 1/0.85
 3   Rf|R3     Ri     (Rf|R3)/Ri = 1/(Ri*(1/Rf+1/R3)) = 1/0.92
 4   Rf        R4     Rf/R4      = 1/(R4*(1/Rf))      = 1/1.00
 5   Rf|R1     R4     (Rf|R1)/R4 = 1/(R4*(1/Rf+1/R1)) = 1/1.10
 6   Rf|R2     R4     (Rf|R2)/R4 = 1/(R4*(1/Rf+1/R2)) = 1/1.20
 7   Rf|R3     R4     (Rf|R3)/R4 = 1/(R4*(1/Rf+1/R3)) = 1/1.30
 8   Rf        R8     Rf/R8      = 1/(R8*(1/Rf))      = 1/1.43
 9   Rf|R1     R8     (Rf|R1)/R8 = 1/(R8*(1/Rf+1/R1)) = 1/1.56
 A   Rf|R2     R8     (Rf|R2)/R8 = 1/(R8*(1/Rf+1/R2)) = 1/1.70
 B   Rf|R3     R8     (Rf|R3)/R8 = 1/(R8*(1/Rf+1/R3)) = 1/1.86
 C   Rf        RC     Rf/RC      = 1/(RC*(1/Rf))      = 1/2.00
 D   Rf|R1     RC     (Rf|R1)/RC = 1/(RC*(1/Rf+1/R1)) = 1/2.18
 E   Rf|R2     RC     (Rf|R2)/RC = 1/(RC*(1/Rf+1/R2)) = 1/2.38
 F   Rf|R3     RC     (Rf|R3)/RC = 1/(RC*(1/Rf+1/R3)) = 1/2.60


These data indicate that the following function for 1/Q has been
modeled in the MOS 8580:

  1/Q = 2^(1/2)*2^(-x/8) = 2^(1/2 - x/8) = 2^((4 - x)/8)

*/
void Filter::set_Q()
{
  // Cutoff for MOS 6581.
  // The coefficient 8 is dispensed of later by right-shifting 3 times
  // (2 ^ 3 = 8).
  _8_div_Q = ~res & 0x0f;
}

void Filter::prefetchMixer()
{
  int curve = 0;
    int m = mix;
    for ( int i = 0; i < 7; i++ )
    {
        if ( m & 1 ) curve ++;
        m >>= 1;
    }

  crvPrefetch = &fgainmixer[sid_model][ vol ][ curve ];
  coeffsPrefetch = (signed short*)fgainmixer[ sid_model ][ vol ][ curve ].coeffsIdx;

}

// Set input routing bits.
void Filter::set_sum_mix()
{
  // NB! voice3off (mode bit 7) only affects voice 3 if it is routed directly
  // to the mixer.
  sum = (enabled ? filt : 0x00) & voice_mask;
  mix =
    (enabled ? (mode & 0x70) | ((~(filt | (mode & 0x80) >> 5)) & 0x0f) : 0x0f)
    & voice_mask;

    prefetchMixer();
}

#undef v1
#undef v2
#undef v3
#undef ve
#undef Vlp
#undef Vbp
#undef Vhp

#undef Vbp_x
#undef Vbp_vc
#undef Vlp_x
#undef Vlp_vc


RESID_NAMESPACE_STOP

