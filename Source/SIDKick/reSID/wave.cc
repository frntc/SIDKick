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

#define RESID_WAVE_CC

#include "wave.h"
#include "dac.h"

RESID_NAMESPACE_START

#include  <avr/pgmspace.h>

#define USE_PRECOMPUTED_TABLES

#ifdef USE_PRECOMPUTED_TABLES

#include "model_wave.h" 
#include "model_dac.h" 

unsigned short model_dac[2][1 << 12] = {
  {0},
  {0},
};

#else

// Waveform lookup tables.
unsigned short model_wave[2][8][1 << 12] = {
  {
    {0},
    {0},
    {0},
#include "wave6581__ST.h"
    {0},
#include "wave6581_P_T.h"
#include "wave6581_PS_.h"
#include "wave6581_PST.h"
  },
  {
    {0},
    {0},
    {0},
#include "wave8580__ST.h"
    {0},
#include "wave8580_P_T.h"
#include "wave8580_PS_.h"
#include "wave8580_PST.h"
  }
};


// DAC lookup tables.
unsigned short model_dac[2][1 << 12] = {
  {0},
  {0},
};

unsigned short model_dac0[ 1 << 6 ];
unsigned short model_dac1[ 1 << 6 ];
unsigned char model_dac0_8[ 1 << 6 ];
unsigned char model_dac1_8[ 1 << 6 ];

unsigned char model_wave8[ 2 ][ 4 ][ 1 << 12 ];

#endif

// ----------------------------------------------------------------------------
// Constructor.
// ----------------------------------------------------------------------------
WaveformGenerator::WaveformGenerator()
{
  static bool class_init;

  if (!class_init) {
  #ifndef USE_PRECOMPUTED_TABLES
      // Calculate tables for normal waveforms.
    accumulator = 0;
    for (int i = 0; i < (1 << 12); i++) {
      reg24 msb = accumulator & 0x800000;

      if ( i == 0 )
          i = i;

      // Noise mask, triangle, sawtooth, pulse mask.
      // The triangle calculation is made branch-free, just for the hell of it.
      /*model_wave[0][0][i] = model_wave[1][0][i] = 0xfff;
      model_wave[0][1][i] = model_wave[1][1][i] = ((accumulator ^ -!!msb) >> 11) & 0xffe;
      model_wave[0][2][i] = model_wave[1][2][i] = accumulator >> 12;
      model_wave[0][4][i] = model_wave[1][4][i] = 0xfff;*/

      for ( int k = 0; k < 2; k++ )
      {
          model_wave8[ k ][ 0 ][ i ] = model_wave[ k ][ 3 ][ i ] >> 4;
          model_wave8[ k ][ 1 ][ i ] = model_wave[ k ][ 5 ][ i ] >> 4;
          model_wave8[ k ][ 2 ][ i ] = model_wave[ k ][ 6 ][ i ] >> 4;
          model_wave8[ k ][ 3 ][ i ] = model_wave[ k ][ 7 ][ i ] >> 4;
      }

      /*for ( int k = 0; k < 8; k++ )
      {
          model_wave8[ 0 ][ k ][ i ] = model_wave[ 0 ][ k ][ i ] >> 4;
          model_wave8[ 1 ][ k ][ i ] = model_wave[ 1 ][ k ][ i ] >> 4;
      }*/

      accumulator += 0x1000;
    }

    // Build DAC lookup tables for 12-bit DACs.

    // MOS 8580: 2R/R ~ 2.00, correct termination.
    build_dac_table_fac( 12, 2.00, true, model_dac0, model_dac1 );

    // MOS 6581: 2R/R ~ 2.20, missing termination resistor.
    build_dac_table_fac( 12, 2.50, false, model_dac0, model_dac1 );

    for ( int i = 0; i < 1 << 6; i++ )
    {
        model_dac0_8[ i ] = model_dac0[ i ];
        int v = model_dac1[ i ] - i * 64 + 128 + 16;
        if ( v < 0 ) v = 0; 
        if ( v > 255 ) v = 255;
        model_dac1_8[ i ] = v;
    }

    {
        FILE *f = fopen( "model_wave.h", "wt" );
        fprintf( f, "const unsigned char model_wave8[ 2 ][ 4 ][ 4096 ] DMAMEM = {\n" );
        for ( int k = 0; k < 2; k++ )
        {
            fprintf( f, "{ " );
            for ( int j = 0; j < 4; j++ )
            {
                fprintf( f, "{ " );
                for ( int i = 0; i < 1 << 12; i++ )
                {
                    if ( ( i & 31 ) == 31 )
                        fprintf( f, "\n  " );
                    fprintf( f, "%3d, ", (int)model_wave8[ k ][ j ][ i ] );
                }
                fprintf( f, "},\n" );
            }
            fprintf( f, "},\n" );
        }
        fprintf( f, "};\n\n" );
        fclose( f );
        exit( 1 );
    }
    {
        FILE *f = fopen( "model_dac.h", "wt" );
        fprintf( f, "const unsigned char model_dac0_8[ 64 ] DMAMEM = {\n" );
        for ( int i = 0; i < 1 << 6; i++ )
            fprintf( f, "%3d, ", (int)model_dac0_8[ i ] );
        fprintf( f, "};\n\n" );
        fprintf( f, "const unsigned char model_dac1_8[ 64 ] DMAMEM = {\n" );
        for ( int i = 0; i < 1 << 6; i++ )
            fprintf( f, "%3d, ", (int)model_dac1_8[ i ] );
        fprintf( f, "};\n\n" );
        fclose( f );
        exit( 1 );
    }
       /*FILE *f = fopen( "model_dac.data", "wt" );
    for ( int i = 0; i < 1 << 6; i++ )
        fprintf( f, "%d %d\n", i, model_dac0[ i ] );
    int minV = 11111, maxV = -111111;
    for ( int i = 0; i < 1 << 6; i++ )
    {
        int v = model_dac1[ i ] - i * 64 + 128 + 16;
        if ( v < minV ) minV = v;
        if ( v > maxV ) maxV = v;
        if ( v >= 0 && v < 256 ) v = 0;
        if ( v > 255 ) v = v - 255;
        fprintf( f, "%d %d\n", i + ( 1 << 6 ), v );
    }
    fprintf( f, "min=%d   max=%d\n", minV, maxV );
    fclose( f );
    exit( 1 );*/


/*    FILE *f = fopen( "model_dac.data", "wt" );
    for ( int i = 0; i < 1 << 12; i++ )
        fprintf( f, "%d %1.8f\n", i, (float)model_dac[ 0 ][ i ] );
    for ( int i = 0; i < 1 << 12; i++ )
        fprintf( f, "%d %1.8f\n", i + (1<<12), (float)model_dac[ 1 ][ i ] );
    fclose( f );
    exit( 1 );*/

#endif
    // Build DAC lookup tables for 12-bit DACs.
    // MOS 6581: 2R/R ~ 2.20, missing termination resistor.
    build_dac_table(model_dac[0], 12, 2.20, false);
    // MOS 8580: 2R/R ~ 2.00, correct termination.
    build_dac_table(model_dac[1], 12, 2.00, true);

    class_init = true;
  }

  sync_source = this;

  sid_model = MOS6581;

  // Accumulator's even bits are high on powerup
  accumulator = 0x555555;

  tri_saw_pipeline = 0x555;

  reset();
}


// ----------------------------------------------------------------------------
// Set sync source.
// ----------------------------------------------------------------------------
void WaveformGenerator::set_sync_source(WaveformGenerator* source)
{
  sync_source = source;
  source->sync_dest = this;
}


// ----------------------------------------------------------------------------
// Set chip model.
// ----------------------------------------------------------------------------
void WaveformGenerator::set_chip_model(chip_model model)
{
  sid_model = model;
 // wave = model_wave[ model ][ waveform & 0x7 ];
  //wave8 = model_wave8[ model ][ waveform & 0x7 ];
  unsigned char w7 = waveform & 0x7;
  if ( w7 >= 5 ) 
      wave8 = model_wave8[ model ][ w7 - 4 ]; else
      wave8 = model_wave8[ model ][ 0 ]; 
}


// ----------------------------------------------------------------------------
// Register functions.
// ----------------------------------------------------------------------------
void WaveformGenerator::writeFREQ_LO(reg8 freq_lo)
{
  freq = (freq & 0xff00) | (freq_lo & 0x00ff);
}

void WaveformGenerator::writeFREQ_HI(reg8 freq_hi)
{
  freq = ((freq_hi << 8) & 0xff00) | (freq & 0x00ff);
}

void WaveformGenerator::writePW_LO(reg8 pw_lo)
{
  pw = (pw & 0xf00) | (pw_lo & 0x0ff);
  // Push next pulse level into pulse level pipeline.
  pulse_output = (accumulator >> 12) >= pw ? 0xfff : 0x000;
}

void WaveformGenerator::writePW_HI(reg8 pw_hi)
{
  pw = ((pw_hi << 8) & 0xf00) | (pw & 0x0ff);
  // Push next pulse level into pulse level pipeline.
  pulse_output = (accumulator >> 12) >= pw ? 0xfff : 0x000;
}

bool do_pre_writeback(reg8 waveform_prev, reg8 waveform, bool is6581)
{
    // no writeback without combined waveforms
    if (likely(waveform_prev <= 0x8))
        return false;
    // This need more investigation
    if (waveform == 8)
        return false;
    // What's happening here?
    if (is6581 &&
            ((((waveform_prev & 0x3) == 0x1) && ((waveform & 0x3) == 0x2))
            || (((waveform_prev & 0x3) == 0x2) && ((waveform & 0x3) == 0x1))))
        return false;
    // ok do the writeback
    return true;
}

void WaveformGenerator::writeCONTROL_REG(reg8 control)
{
  reg8 waveform_prev = waveform;
  reg8 test_prev = test;
  waveform = (control >> 4) & 0x0f;
  test = control & 0x08;
  ring_mod = control & 0x04;
  sync = control & 0x02;

  // Set up waveform table.
  //wave = model_wave[ sid_model ][ waveform & 0x7 ];
  //wave8 = model_wave8[ sid_model ][ waveform & 0x7 ];
  unsigned char w7 = waveform & 0x7;
  if ( w7 >= 5 )
      wave8 = model_wave8[ sid_model ][ w7 - 4 ]; else
      wave8 = model_wave8[ sid_model ][ 0 ];

  // Substitution of accumulator MSB when sawtooth = 0, ring_mod = 1.
  ring_msb_mask = ((~control >> 5) & (control >> 2) & 0x1) << 23;

  // no_noise and no_pulse are used in set_waveform_output() as bitmasks to
  // only let the noise or pulse influence the output when the noise or pulse
  // waveforms are selected.
  no_noise = waveform & 0x8 ? 0x000 : 0xfff;
  no_noise_or_noise_output = no_noise | noise_output;
  no_pulse = waveform & 0x4 ? 0x000 : 0xfff;

  // Test bit rising.
  // The accumulator is cleared, while the the shift register is prepared for
  // shifting by interconnecting the register bits. The internal SRAM cells
  // start to slowly rise up towards one. The SRAM cells reach one within
  // approximately $8000 cycles, yielding a shift register value of
  // 0x7fffff.
  if (!test_prev && test) {
    // Reset accumulator.
    accumulator = 0;

    // Flush shift pipeline.
    shift_pipeline = 0;

    // Set reset time for shift register.
    shift_register_reset = 0x8000;

    // The test bit sets pulse high.
    pulse_output = 0xfff;
  }
  else if (test_prev && !test) {
    // When the test bit is falling, the second phase of the shift is
    // completed by enabling SRAM write.

    // During first phase of the shift the bits are interconnected
    // and the output of each bit is latched into the following.
    // The output may overwrite the latched value.
    if (do_pre_writeback(waveform_prev, waveform, sid_model == MOS6581)) {
        write_shift_register();
    }

    // bit0 = (bit22 | test) ^ bit17 = 1 ^ bit17 = ~bit17
    reg24 bit0 = (~shift_register >> 17) & 0x1;
    shift_register = ((shift_register << 1) | bit0) & 0x7fffff;

    // Set new noise waveform output.
    set_noise_output();
  }

  if (waveform) {
    // Set new waveform output.
    set_waveform_output();
  }
  else if (waveform_prev) {
    // Change to floating DAC input.
    // Reset fading time for floating DAC input.
    //
    // We have two SOAS/C samplings showing that floating DAC
    // keeps its state for at least 0x14000 cycles.
    //
    // This can't be found via sampling OSC3, it seems that
    // the actual analog output must be sampled and timed.
    floating_output_ttl = 0x14000;
  }

  // The gate bit is handled by the EnvelopeGenerator.
}

reg8 WaveformGenerator::readOSC()
{
  return osc3 >> 4;
}

// ----------------------------------------------------------------------------
// SID reset.
// ----------------------------------------------------------------------------
void WaveformGenerator::reset()
{
  // accumulator is not changed on reset
  freq = 0;
  pw = 0;

  msb_rising = false;

  waveform = 0;
  test = 0;
  ring_mod = 0;
  sync = 0;

  //wave = model_wave[sid_model][0];
  wave8 = model_wave8[ sid_model ][ 0 ];

  ring_msb_mask = 0;
  no_noise = 0xfff;
  no_pulse = 0xfff;
  pulse_output = 0xfff;

  reset_shift_register();
  shift_pipeline = 0;

  waveform_output = 0;
  osc3 = 0;
  floating_output_ttl = 0;
}

RESID_NAMESPACE_STOP
