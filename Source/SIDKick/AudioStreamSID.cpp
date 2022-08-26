/*
   _________.___________   ____  __.__        __     
  /   _____/|   \______ \ |    |/ _|__| ____ |  | __ 
  \_____  \ |   ||    |  \|      < |  |/ ___\|  |/ / 
  /        \|   ||    `   \    |  \|  \  \___|    <  
 /_______  /|___/_______  /____|__ \__|\___  >__|_ \ 
         \/             \/        \/       \/     \/ 
        
 AudioStreamSID.cpp

 SIDKick - SID-replacement with SID, Sound Expander and MIDI Emulation based on Teensy 4.1
           (using reSID by Dag Lem and FMOPL by Jarek Burczynski, Tatsuyuki Satoh, Marco van den Heuvel, and Acho A. Tang)
 Copyright (c) 2019-2022 Carsten Dachsbacher <frenetic@dachsbacher.de>

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

#include "globals.h"

#include "AudioStreamSID.h"
#include <math.h>
#include <AudioStream.h>
#include "samvoice.h"

#include "imxrt.h"
#include "utility/imxrt_hw.h"

#define SAMPLERATE AUDIO_SAMPLE_RATE_EXACT

// overwritten in SIDKick.ino according to auto detection
unsigned int CLOCKFREQ = 985248;
unsigned int CLOCKFREQ_NOMINAL = 985248;

bool useSID16 = true,
     emulateSID2 = true, 
     emulateFM = false, 
     readRegistersFM = false, 
     activeSID2 = false, 
     activeFM = false, 
     activeSID2Prev = false, 
     activeFMPrev = false, 
     registerRead = true, 
     emulateMIDI = false, 
     midiInputEnabled = false,
     driveLEDs = false;
     
uint32_t SID1_MODEL = 0;
uint32_t SID2_MODEL = 0;
uint32_t SID2_ADDR = 0xffffffff;

int32_t cfgVolSID1_Left, cfgVolSID1_Right;
int32_t cfgVolSID2_Left, cfgVolSID2_Right;
int32_t cfgVolOPL_Left, cfgVolOPL_Right;

int32_t actVolSID1_Left, actVolSID1_Right;
int32_t actVolSID2_Left, actVolSID2_Right;
int32_t actVolOPL_Left, actVolOPL_Right;

#define max( a, b ) ((a)>(b)?(a):(b))
#define min( a, b ) ((a)<(b)?(a):(b))

extern uint32_t nSIDFMDelay;
extern uint32_t nMIDIDelay;
uint8_t samActive = 0;

#ifdef FANCY_LED
// look into this file for all the variables...
#include "LED_Helpers.h"
#endif

uint32_t samCurPos, samEndPos1, samStartPos2, samEndPos2;
uint8_t samSpeaking = 0;

void speakSAM( uint8_t digit, bool onlyDigit )
{
  samCurPos    = 0;
  samEndPos1   = ( 18 * 1024 - 1 ) << 1;
  samStartPos2 = samEndPos1 + ( ( digit * 12 * 1024 ) << 1 );
  samEndPos2   = samStartPos2 + ( ( 12 * 1024 ) << 1 );
  if ( onlyDigit )
    samCurPos = samStartPos2;
  samSpeaking  = 1;
}

uint8_t firstBufferAfterReset = 0;
uint8_t recalibrateFrequency = 0;

void AudioStreamSID::init()
{
  sid16_1 = new SID16();
  sid16_2 = new SID16();

#ifndef NO_RESID10
  sid_1 = new RESID_NAMESPACE::SID();
  sid_2 = new RESID_NAMESPACE::SID();
#endif
  
  #ifdef EMULATE_OPL2
  pOPL = ym3812_init( 3579545, SAMPLERATE );
  ym3812_reset_chip( pOPL );
  fmFakeOutput = 0;
  hack_OPL_Sample_Value = 0;
  hack_OPL_Sample_Enabled = 0;
  #endif
  
  sid16_1->set_chip_model( MOS6581 );
  sid16_2->set_chip_model( MOS6581 );
  sid16_1->reset();
  sid16_2->reset();
  sid16_1->set_sampling_parameters( CLOCKFREQ, SAMPLE_INTERPOLATE, AUDIO_SAMPLE_RATE_EXACT );
  sid16_2->set_sampling_parameters( CLOCKFREQ, SAMPLE_INTERPOLATE, AUDIO_SAMPLE_RATE_EXACT );
  
#ifndef NO_RESID10
  sid_1->set_chip_model( RESID_NAMESPACE::MOS8580 );
  sid_1->set_voice_mask( 0x07 );
  sid_1->input( 0 );

  sid_2->set_chip_model( RESID_NAMESPACE::MOS8580 );
  sid_2->set_voice_mask( 0x07 );
  sid_2->input( 0 );

  sid_1->reset();
  sid_2->reset();
  
  int SID_passband = 90+9;
  int SID_gain = 97;
  int SID_filterbias = +500;

  sid_1->adjust_filter_bias( SID_filterbias / 1000.0f );
  sid_2->adjust_filter_bias( SID_filterbias / 1000.0f );

  sid_1->set_sampling_parameters( CLOCKFREQ, RESID_NAMESPACE::SAMPLE_FAST, AUDIO_SAMPLE_RATE_EXACT, AUDIO_SAMPLE_RATE_EXACT * SID_passband / 200.0f, SID_gain / 100.0f );
  sid_2->set_sampling_parameters( CLOCKFREQ, RESID_NAMESPACE::SAMPLE_FAST, AUDIO_SAMPLE_RATE_EXACT, AUDIO_SAMPLE_RATE_EXACT * SID_passband / 200.0f, SID_gain / 100.0f );
#endif

  activeSID2 = activeSID2Prev = false;
  activeFM   = activeFMPrev   = false;
}

void AudioStreamSID::updateMixer( bool playSID2, bool playFM )
{
  actVolSID1_Left  = cfgVolSID1_Left;
  actVolSID1_Right = cfgVolSID1_Right;
  actVolSID2_Left  = cfgVolSID2_Left;
  actVolSID2_Right = cfgVolSID2_Right;
  actVolOPL_Left   = cfgVolOPL_Left;
  actVolOPL_Right  = cfgVolOPL_Right;

  if ( !playSID2 )
    actVolSID2_Left = actVolSID2_Right = 0;
  if ( !playFM )
    actVolOPL_Left = actVolOPL_Right = 0;

#ifdef DEBUG_OUTPUT
  Serial.print( "SID #2: " );
  if ( playSID2 )
    Serial.println( "active" ); else
    Serial.println( "off" ); 

  Serial.print( "FM: " );
  if ( playFM )
    Serial.println( "active" ); else
    Serial.println( "off" ); 
#endif

  // old:
  //int maxVolFactor = max( actVolSID1_Left, max( actVolSID1_Right, max( actVolSID2_Left, max( actVolSID2_Right, max( actVolOPL_Left, actVolOPL_Right ) ) ) ) );
  int maxVolFactor = max( actVolSID1_Left + actVolSID2_Left + actVolOPL_Left, actVolSID1_Right + actVolSID2_Right + actVolOPL_Right );

  const int globalVolume = 255; // old: 128
  actVolSID1_Left  = actVolSID1_Left  * globalVolume / maxVolFactor;
  actVolSID1_Right = actVolSID1_Right * globalVolume / maxVolFactor;
  actVolSID2_Left  = actVolSID2_Left  * globalVolume / maxVolFactor;
  actVolSID2_Right = actVolSID2_Right * globalVolume / maxVolFactor;
  actVolOPL_Left   = actVolOPL_Left   * globalVolume / maxVolFactor;
  actVolOPL_Right  = actVolOPL_Right  * globalVolume / maxVolFactor;
}

static int muteTest = 0;

void AudioStreamSID::updateConfiguration( uint8_t *cfg, uint8_t *globalCfg )
{
  switch ( cfg[ 10 ] ) // sid 2 address settings
  {
    case 0: // $d400
      SID2_ADDR = 1 << 31;
      break;
    case 1: // $d420
      SID2_ADDR = CORE_PIN34_BITMASK; // A5
      break;
    case 2: // $d500
      SID2_ADDR = CORE_PIN37_BITMASK; // A8
      break;
    default:
    case 3: // $d420 and $d500
      SID2_ADDR = CORE_PIN34_BITMASK | CORE_PIN37_BITMASK; // A5 | A8
      break;
    case 4: // $de00
      SID2_ADDR = CORE_PIN11_BITMASK; // IO1
      break;
    case 5: // $df00
      SID2_ADDR = CORE_PIN13_BITMASK; // IO2
      break;
  }
  
  registerRead = cfg[ 2 ] > 0 ? true : false;  
  #ifdef NO_RESID10
  useSID16     = true;  
  #else
  useSID16     = cfg[ 19 ] == 0 ? true : false;  
  #endif
  emulateFM    = cfg[ 16 ] > 0 ? true : false;
  readRegistersFM = cfg[ 16 ] == 1 ? true : false;
  emulateSID2  = cfg[  8 ] < 3 ? true : false;  

  if ( !emulateSID2 )
    SID2_ADDR = 0;

  if ( cfg[0] == 0 )
  {
    SID1_MODEL = 6581;
    sid16_1->set_chip_model( MOS6581 );
#ifndef NO_RESID10
    sid_1->set_chip_model( RESID_NAMESPACE::MOS6581 );
    sid_1->set_voice_mask( 0x07 );
    sid_1->input( 0 );
#endif
  } else
  {
    SID1_MODEL = 8580;
    sid16_1->set_chip_model( MOS8580 );
#ifndef NO_RESID10
    sid_1->set_chip_model( RESID_NAMESPACE::MOS8580 );
    if ( cfg[0] == 1 ) // no digiboost
    {
      sid_1->set_voice_mask( 0x07 );
      sid_1->input( 0 );
    } else
    {
      sid_1->set_voice_mask( 0x0f );
      sid_1->input( -(1<<cfg[1]) );
    }
#endif
  }

  if ( cfg[8] == 0 )
  {
    SID2_MODEL = 6581;
    sid16_2->set_chip_model( MOS6581 );
#ifndef NO_RESID10
    sid_2->set_chip_model( RESID_NAMESPACE::MOS6581 );
    sid_2->set_voice_mask( 0x07 );
    sid_2->input( 0 );
#endif
  } else
  if ( cfg[8] != 3 )
  {
      SID2_MODEL = 8580;
      sid16_2->set_chip_model( MOS8580 );
#ifndef NO_RESID10
      sid_2->set_chip_model( RESID_NAMESPACE::MOS8580 );
      if ( cfg[8] == 1 ) // no digiboost
      {
        sid_2->set_voice_mask( 0x07 );
        sid_2->input( 0 );
      } else
      {
        sid_2->set_voice_mask( 0x0f );
        sid_2->input( -(1<<cfg[9]) );
      }
#endif
  } else
    SID2_MODEL = 0;

  cfgVolSID1_Left  = (int)cfg[ 3] * (int)( 14 - cfg[ 4] );
  cfgVolSID1_Right = (int)cfg[ 3] * (int)( cfg[ 4] );
  cfgVolSID2_Left  = (int)cfg[11] * (int)( 14 - cfg[12] );
  cfgVolSID2_Right = (int)cfg[11] * (int)( cfg[12] );
  cfgVolOPL_Left   = (int)cfg[17] * (int)( 14 - cfg[18] );
  cfgVolOPL_Right  = (int)cfg[17] * (int)( cfg[18] );

  activeSID2 = activeSID2Prev = false;
  activeFM = activeFMPrev = false;
  updateMixer( activeSID2, activeFM );

#ifndef NO_RESID10
  {
    int SID_passband = cfg[ 5 ];
    int SID_gain = 90 + cfg[ 6 ];
    int SID_filterbias = ( (int)cfg[ 7 ] - 50 ) * 100;
  
    sid_1->adjust_filter_bias(SID_filterbias / 1000.0f);
    sid_1->set_sampling_parameters( CLOCKFREQ, RESID_NAMESPACE::SAMPLE_FAST, AUDIO_SAMPLE_RATE_EXACT, AUDIO_SAMPLE_RATE_EXACT * SID_passband / 200.0f, SID_gain / 100.0f );

    SID_passband = cfg[ 13 ];
    SID_gain = 90 + cfg[ 14 ];
    SID_filterbias = ( (int)cfg[ 15 ] - 50 ) * 100;
  
    sid_2->adjust_filter_bias(SID_filterbias / 1000.0f);
    sid_2->set_sampling_parameters( CLOCKFREQ, RESID_NAMESPACE::SAMPLE_FAST, AUDIO_SAMPLE_RATE_EXACT, AUDIO_SAMPLE_RATE_EXACT * SID_passband / 200.0f, SID_gain / 100.0f );
  }

#endif

#ifdef FANCY_LED
  ledWriteStatus = -1;
  last_tL = 0.0f;
  last_tR = 0.0f;
  vuSumL = 0;
  vuSumR = 0;
  vuAvg = 0.0f;
  vuMeter = 0;
  vu_nValues = 0;
  nLEDs = 0;
  nLEDsL = 0;
  nLEDsR = 0;

  ledVisMode = (int)cfg[ 20 ];
  driveLEDs = (ledVisMode > 0) ? true : false;

  ledColorsBufOfs = 0;
  ledColors2Write = 0;
  ledBits2Write = 0;

  NUM_LEDS = cfg[21] + 1;

  nSamplesCollected = 3 + (uint32_t)cfg[ 22 ];
  nSamplesCollected *= (uint32_t)64;
  scaleSamples = (float)cfg[ 23 ] / 64.0f;

  if ( ledVisMode == 5 || ledVisMode == 6 )
    scaleSamples *= 4.0f;
  if ( ledVisMode == 3 || ledVisMode == 4 )
    scaleSamples *= 2.0f;
  if ( ledVisMode == 2 )
    scaleSamples *= 2.0f;

  scaleSamples *= 2.480903f; // from powf approximation

  scaleSamplesConfigTool = scaleSamples * 1.2f;
  scaleSamplesConfigTool = 3.0f / scaleSamplesConfigTool; 

  colorCycle = 0;
  ledColorCycleSpeed = cfg[24];

  // color #0 25-27
  hsv0  = (cfg[25]*255/89) << 16;
  hsv0 |= cfg[26] << 11;
  hsv0 |= cfg[27] << 2;

  hsv1  = (cfg[28]*255/89) << 16;
  hsv1 |= cfg[29] << 11;
  hsv1 |= cfg[30] << 2;
  
  col0 = HSV2RGB( hsv0 );
  col1 = HSV2RGB( hsv1 );

   for ( int i = 0; i < 128; i++ )
   {
      col0_cycle[ i ] = HSV2RGB( ( hsv0 & 0x00ffff ) | ( (( H_( hsv0 ) + (i * 2) ) & 255) << 16 ) );
      col1_cycle[ i ] = HSV2RGB( ( hsv1 & 0x00ffff ) | ( (( H_( hsv1 ) + (i * 2) ) & 255) << 16 ) );
   }

   hsv0 = RGB2HSV( col0 );
   hsv1 = RGB2HSV( col1 );
 #endif

  //
  // MIDI
  //
  #ifdef SUPPORT_MIDI
  extern uint8_t MIDI_START_ADDR;
  extern uint8_t MIDI_END_ADDR;
  emulateMIDI = true;
  switch ( cfg[ 31 ] )
  {
    default:
    case 0:
      emulateMIDI = false;
      MIDI_START_ADDR = 0xff;
      MIDI_END_ADDR = 0x00;
      break;
    case 1: // Datel
      MIDI_START_ADDR = 0x04;
      MIDI_END_ADDR = 0x07;
      break;
    case 2: // Sequential
    case 4: // Namesoft
      MIDI_START_ADDR = 0x00;
      MIDI_END_ADDR = 0x03;
      break;
    case 3: // Datel & Sequential
      MIDI_START_ADDR = 0x00;
      MIDI_END_ADDR = 0x07;
      break;
  }
  #endif

  midiInputEnabled = cfg[ 32 ] > 0;

  nSIDFMDelay = nMIDIDelay = 0;
  extern uint8_t runningOnPAL;

  if ( cfg[ 34 ] == 0 )
    nSIDFMDelay = cfg[ 33 ] * (runningOnPAL ? 985 : 1023); else
    nMIDIDelay  = cfg[ 33 ] * (runningOnPAL ? 985 : 1023);
    
  samActive = cfg[ 35 ];

  muteTest = 768;
}


void AudioStreamSID::begin()
{
  playing = true;
}

void AudioStreamSID::stop()
{
  __disable_irq();
  playing = false;
  __enable_irq();
}

void AudioStreamSID::continuePlaying()
{
  //__disable_irq();
  playing = true;
  //__enable_irq();
}

extern uint32_t c64CycleCount;
extern uint32_t nCyclesEmulated;

#define RING_SIZE (4096)
extern uint32_t ringBufGPIO[ RING_SIZE ];
extern uint32_t ringTime[ RING_SIZE ];
extern uint16_t ringWrite;

uint16_t ringRead = 0;
unsigned long long samplesElapsed = 0;
uint32_t samplesTotal = 0;

__attribute__( ( always_inline ) ) inline uint32_t __SMLAWB(const uint32_t Rd, const uint32_t Rm, const uint32_t Rn, const uint32_t Ra)
{
  uint32_t result;
  asm volatile ( "SMLAWB %0, %1, %2, %3" : "=r" (result) : "r" (Rn), "r" (Rm), "r" (Ra) );
  return(result);
}

__attribute__( ( always_inline ) ) inline uint32_t __SSAT(const uint32_t op)
{
  uint32_t result;
  asm volatile ( "SSAT %0, #16, %1" : "=r" (result) : "r" (op) );
  return(result);
}

__attribute__( ( always_inline ) ) inline uint32_t __UQADD8(const uint32_t op1, const uint32_t op2)
{
  uint32_t result;
  asm volatile ( "uqadd8 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}

__attribute__( ( always_inline ) ) inline uint32_t __UXTB16(const uint32_t op1)
{
  uint32_t result;

  asm volatile ( "uxtb16 %0, %1" : "=r" (result) : "r" (op1));
  return(result);
}

static uint32_t totalCyclesPerBuffer = 0;
static uint32_t nTotalCycleCounter = 0;

FASTRUN void AudioStreamSID::update()
{
  if ( firstBufferAfterReset > 0 )
  {
    firstBufferAfterReset --;
    fillBlock();
    return;
  }
  
  audio_block_t *block;
  audio_block_t *block2;

  if ( !playing ) return;

  block = allocate();
  block2 = allocate();
  if ( block == NULL || block2 == NULL ) return;

  uint32_t nSamplesComputed = 0;
  int16_t *output = (int16_t*)block->data;
  int16_t *output2 = (int16_t*)block2->data;

  samplesTotal += AUDIO_BLOCK_SAMPLES;


  if ( activeSID2 != activeSID2Prev || activeFM != activeFMPrev )
  {
    activeSID2Prev = activeSID2;
    activeFMPrev   = activeFM;
    updateMixer( activeSID2, activeFM );
  }

  #ifdef DYNAMIC_ADJUSTMENT_MIXER

  static long lastDiff = 0;

  static int cnt = 0;
  #define BUFFER_OBSERVE 256
  if ( ++ cnt > BUFFER_OBSERVE )
  {
    cnt = 0;

    static int targetCycleDiff = 4000;
    if ( recalibrateFrequency )
    {
      targetCycleDiff = lastDiff = (int)c64CycleCount - (int)nCyclesEmulated;
      recalibrateFrequency = 0;
    }
  
    int diff = (int)c64CycleCount - (int)nCyclesEmulated;
    static int nObservations = 0;
  
    static int checkNextTime = 10;
    if ( ++nObservations >= checkNextTime )
    {
      float dC = (float)lastDiff - (float)diff;
      dC += ( (float)targetCycleDiff - (float)diff ) * 0.5f;
      if ( dC > 1000.0f ) dC = 1000.0f;
      if ( dC < -1000.0f ) dC = -1000.0f;
      uint32_t S = nObservations * BUFFER_OBSERVE * 128;
      float F = AUDIO_SAMPLE_RATE_EXACT / (float)S * ( (float)S / 44100.0f * (float)CLOCKFREQ - dC);
      CLOCKFREQ = (int) ( F + 0.5f );
      lastDiff = diff;
      nObservations = 0;
    }
  }
  #else
  #ifdef FIRMWARE_C128
    int32_t cyclesToEmulatePerRun = 16;
    if ( activeSID2 )
      cyclesToEmulatePerRun += 3;
    if ( activeFM )
      cyclesToEmulatePerRun += 3;
    if ( activeSID2 || activeFM )
      cyclesToEmulatePerRun = 22;
  #else
    int32_t cyclesToEmulatePerRun = 16;
    if ( activeSID2 || activeFM )
      cyclesToEmulatePerRun = 22;
  #endif
  #endif

  uint32_t nCyclesPerBuffer = 0;
  
  while ( nSamplesComputed < AUDIO_BLOCK_SAMPLES )
  {
         //
        //
       // sample generation with clock rate adapting to "measured" clock
      //  (in fact it can only be called measurement assuming that the 44.1kHz playback is accurate)
     //
    //
    #ifdef DYNAMIC_ADJUSTMENT_MIXER

    samplesElapsed = ( ( unsigned long long )nCyclesEmulated * ( unsigned long long )AUDIO_SAMPLE_RATE_EXACT ) / ( unsigned long long )CLOCKFREQ;

    unsigned long long tmp;
    long long _cycleNextSampleReady = nCyclesEmulated + 20; // unless the C64/C128 is severely underclocked (or if we change the sample rate) then we always need to compute at least 22 cycles
    do
    {
      _cycleNextSampleReady ++;
      tmp = ( _cycleNextSampleReady * (unsigned long long)AUDIO_SAMPLE_RATE_EXACT ) / (unsigned long long)CLOCKFREQ;
    } while ( tmp == samplesElapsed);

    long long cycleNextSampleReady = _cycleNextSampleReady;
    long long cyclesToNextSample = (long long)cycleNextSampleReady - (long long)nCyclesEmulated;

    uint32_t cyclesToNextWrite;
    
    // check if next SID-command comes earlier
    // this is crucial to play tunes with elaborate sample techniques correctly
  #ifdef FIRMWARE_C128
    if ( ringRead != ringWrite && !activeSID2 && SID2_MODEL == 6581 )
      cyclesToNextWrite = ringTime[ ringRead ] - nCyclesEmulated; else
      cyclesToNextWrite = 0xffffffff;
  #else
    if ( ringRead != ringWrite )
      cyclesToNextWrite = ringTime[ ringRead ] - nCyclesEmulated; else
      cyclesToNextWrite = 0xffffffff;
  #endif


    do { // do SID emulation until time passed to create an additional sample (i.e. there may be several loops until a sample value is created)

      uint32_t cyclesToEmulate = cyclesToNextSample;
      
      if ( cyclesToEmulate > cyclesToNextWrite )
        cyclesToEmulate = cyclesToNextWrite;

      if ( useSID16 )
      {
        sid16_1->clock( cyclesToEmulate );
        if ( emulateSID2 && activeSID2 )
        {
          sid16_2->clock( cyclesToEmulate );
        }
      } else
      {
        sid_1->clock( cyclesToEmulate );
        if ( emulateSID2 && activeSID2 )
        {
          sid_2->clock( cyclesToEmulate );
        }
      }

      nCyclesPerBuffer += cyclesToEmulate;
      nCyclesEmulated += cyclesToEmulate;
      cyclesToNextSample -= cyclesToEmulate;

      if ( ringRead != ringWrite && nCyclesEmulated >= ringTime[ ringRead ] )
      {
      quicklyGetAnotherRegisterWrite:
        register uint8_t A, D;D = ringBufGPIO[ ringRead ] & 255;A = (ringBufGPIO[ ringRead ] >> 8);

        if ( ringBufGPIO[ ringRead ] == 0xffffffff ) // reset everything
        { reset(); } else
        if ( ringBufGPIO[ ringRead ] & (1<<28) ) // fm command?
        {
          #ifdef EMULATE_OPL2
          activeFM = true; ym3812_write( pOPL, ( A >> 4 ) & 1, D );
          if ( pOPL->address == 1 )
          {
            if ( D == 4 ) // enable digi hack
              hack_OPL_Sample_Enabled = 1;  else
              hack_OPL_Sample_Enabled = 0;
          }
          if ( hack_OPL_Sample_Enabled && pOPL->address == 0xa0 ) // enable digi hack
            hack_OPL_Sample_Value = D; else
            hack_OPL_Sample_Value = 0;
          #endif
        } else
        {
          if ( ringBufGPIO[ ringRead ] & (1<<21) ) // pseudo-stereo command?
          {
            if ( useSID16 )
            {
              sid16_1->write( A & 31, D );
              {
                sid16_2->write( A & 31, D ); 
                activeSID2 = true;
              }
            } else
            {
              #ifndef NO_RESID10
              sid_1->write( A & 31, D );
              {
                sid_2->write( A & 31, D ); 
                activeSID2 = true;
              }
              #endif
            }
          } else
          {
            if ( useSID16 )
            {
              if ( ringBufGPIO[ ringRead ] & (1<<20) )
              {
                sid16_2->write( A & 31, D ); 
                activeSID2 = true;
              } else
                sid16_1->write( A & 31, D );
            } else
            {
              #ifndef NO_RESID10
              if ( ringBufGPIO[ ringRead ] & (1<<20) )
              {
                sid_2->write( A & 31, D ); 
                activeSID2 = true;
              } else
                sid_1->write( A & 31, D );
              #endif
            }
          }
        }
        
        ringRead++;
        ringRead &= ( RING_SIZE - 1 );

        if ( ringRead != ringWrite )
        {
          int32_t t = ringTime[ ringRead ] - nCyclesEmulated;
          if ( t <= 0 )
            goto quicklyGetAnotherRegisterWrite;
          cyclesToNextWrite = t; 
        } else
          cyclesToNextWrite = 0xffffffff;

      }

    } while ( nCyclesEmulated < cycleNextSampleReady );

    //Serial.print( "  afterwards: " );
    //Serial.println( (uint32_t)nCyclesEmulated );

    samplesElapsed ++;
    //    samplesElapsed = ( ( unsigned long long )nCyclesEmulated * ( unsigned long long )AUDIO_SAMPLE_RATE_EXACT ) / ( unsigned long long )CLOCKFREQ;

    extern uint8_t outRegisters[ 32 ];
    extern uint8_t outRegisters_2[ 32 ];
    if ( useSID16 )
    {
      outRegisters[ 0x1b ] = sid16_1->read( 0x1b );
      outRegisters[ 0x1c ] = sid16_1->read( 0x1c );
      if ( activeSID2 )
      {
        outRegisters_2[ 0x1b ] = sid16_2->read( 0x1b );
        outRegisters_2[ 0x1c ] = sid16_2->read( 0x1c );
      }
    } else
    {
      #ifndef NO_RESID10
      outRegisters[ 0x1b ] = sid_1->read( 0x1b );
      outRegisters[ 0x1c ] = sid_1->read( 0x1c );
      if ( activeSID2 )
      {
        outRegisters_2[ 0x1b ] = sid_2->read( 0x1b );
        outRegisters_2[ 0x1c ] = sid_2->read( 0x1c );
      }
      #endif
    }

    int valOPL = 0;
    #ifdef EMULATE_OPL2
    if ( activeFM )
    {
      // todo: bulk updates only for C128?
      ym3812_update_one( pOPL, &valOPL, 1 ); 

      if ( hack_OPL_Sample_Enabled )
        valOPL = (uint16_t)hack_OPL_Sample_Value << 5;

      // TODO asynchronous read back is an issue...
      //fmOutRegister = encodeGPIO( ym3812_read( pOPL, 0 ) ); 
    }
    #endif



#else 
         //
        //
       // sample generation with fixed clock (adjusting the SIDKick CPU cycle counter if necessary)
      //  
     //
    //


    unsigned long long samplesElapsedBefore = samplesElapsed;

    long long cycleNextSampleReady = ( ( unsigned long long )(samplesElapsedBefore+1) * ( unsigned long long )CLOCKFREQ ) / ( unsigned long long )AUDIO_SAMPLE_RATE_EXACT;
    int32_t cyclesToNextSample = cycleNextSampleReady - nCyclesEmulated;

    uint32_t cyclesToNextWrite;
    
    // check if next SID-command comes earlier
    // this is crucial to play tunes with elaborate sample techniques correctly
    if ( ringRead != ringWrite )
      cyclesToNextWrite = ringTime[ ringRead ] - nCyclesEmulated; else
      cyclesToNextWrite = 0xffffffff;

#ifdef DEBUG_OUTPUT
    uint32_t nCyclesEmulatedTab[ 64 ];
    uint32_t idx = 0;
#endif

    do { // do SID emulation until time passed to create an additional sample (i.e. there may be several loops until a sample value is created)

      uint32_t cyclesToEmulate = cyclesToEmulatePerRun; 

      // this really eats time, only do this if Teensy is overclocked or SID2 is non-active
      if ( !activeSID2 && !activeFM )
      {
        // cyclesToEmulate might be reduced because of ... 
        // a) next sample is due earlier, ...
        // this should super-slightly increase sound quality, however, requires overclocking already when 2 SIDs + FM are emulated
        if ( (int32_t)cyclesToEmulate > cyclesToNextSample )
          cyclesToEmulate = cyclesToNextSample;
      }
      
      // b) next SID command is earlier ...
//      if ( !(activeSID2 && activeFM) )
      if ( !(activeSID2 || activeFM) )
      if ( cyclesToEmulate > cyclesToNextWrite )
        cyclesToEmulate = cyclesToNextWrite;

      if ( cyclesToEmulate <= 0 )
        cyclesToEmulate = 1;

#ifdef DEBUG_OUTPUT
      nCyclesEmulatedTab[ idx ++ ] = cyclesToEmulate;
#endif

      if ( useSID16 )
      {
        sid16_1->clock( cyclesToEmulate );
        if ( emulateSID2 && activeSID2 )
          sid16_2->clock( cyclesToEmulate );
      } else
      {
#ifndef NO_RESID10
        sid_1->clock( cyclesToEmulate );
        if ( emulateSID2 && activeSID2 )
          sid_2->clock( cyclesToEmulate );
#endif
      }

      nCyclesEmulated += cyclesToEmulate;
      cyclesToNextSample -= cyclesToEmulate;

      if ( ringRead != ringWrite && nCyclesEmulated >= ringTime[ ringRead ] )
      {
      quicklyGetAnotherRegisterWrite:
        register uint8_t A, D;
        D = ringBufGPIO[ ringRead ] & 255;
        A = (ringBufGPIO[ ringRead ] >> 8);

        if ( ringBufGPIO[ ringRead ] == 0xffffffff ) // reset everything
        {
          reset();
        } else
        if ( ringBufGPIO[ ringRead ] & (1<<28) ) // fm command?
        {
          #ifdef EMULATE_OPL2
          activeFM = true;
          ym3812_write( pOPL, ( A >> 4 ) & 1, D );
          if ( pOPL->address == 1 )
          {
            if ( D == 4 ) // enable digi hack
              hack_OPL_Sample_Enabled = 1;  else
              hack_OPL_Sample_Enabled = 0;
          }
          if ( hack_OPL_Sample_Enabled && pOPL->address == 0xa0 ) // enable digi hack
            hack_OPL_Sample_Value = D; else
            hack_OPL_Sample_Value = 0;
          #endif
        } else
        {
          if ( ringBufGPIO[ ringRead ] & (1<<21) ) // pseudo-stereo command?
          {
            if ( useSID16 )
            {
              sid16_1->write( A & 31, D );
              {
                sid16_2->write( A & 31, D ); 
                activeSID2 = true;
              }
            } else
            {
              #ifndef NO_RESID10
              sid_1->write( A & 31, D );
              {
                sid_2->write( A & 31, D ); 
                activeSID2 = true;
              }
              #endif
            }
          } else
          {
            if ( useSID16 )
            {
              if ( ringBufGPIO[ ringRead ] & (1<<20) )
              {
                sid16_2->write( A & 31, D ); 
                activeSID2 = true;
              } else
                sid16_1->write( A & 31, D );
            } else
            {
              #ifndef NO_RESID10
              if ( ringBufGPIO[ ringRead ] & (1<<20) )
              {
                sid_2->write( A & 31, D ); 
                activeSID2 = true;
              } else
                sid_1->write( A & 31, D );
              #endif
            }
          }
        }
        
        ringRead++;
        ringRead &= ( RING_SIZE - 1 );

        if ( ringRead != ringWrite )
        {
          int32_t t = ringTime[ ringRead ] - nCyclesEmulated;
          if ( t <= 0 )
            goto quicklyGetAnotherRegisterWrite;
          cyclesToNextWrite = t; 
        } else
          cyclesToNextWrite = 0xffffffff;

      }
     
      samplesElapsed = ( ( unsigned long long )nCyclesEmulated * ( unsigned long long )AUDIO_SAMPLE_RATE_EXACT ) / ( unsigned long long )CLOCKFREQ;
    } while ( samplesElapsed == samplesElapsedBefore );

    extern uint8_t outRegisters[ 32 ];
    extern uint8_t outRegisters_2[ 32 ];
    if ( useSID16 )
    {
      outRegisters[ 0x1b ] = sid16_1->read( 0x1b );
      outRegisters[ 0x1c ] = sid16_1->read( 0x1c );
      if ( activeSID2 )
      {
        outRegisters_2[ 0x1b ] = sid16_2->read( 0x1b );
        outRegisters_2[ 0x1c ] = sid16_2->read( 0x1c );
      }
    } else
    {
      #ifndef NO_RESID10
      outRegisters[ 0x1b ] = sid_1->read( 0x1b );
      outRegisters[ 0x1c ] = sid_1->read( 0x1c );
      if ( activeSID2 )
      {
        outRegisters_2[ 0x1b ] = sid_2->read( 0x1b );
        outRegisters_2[ 0x1c ] = sid_2->read( 0x1c );
      }
      #endif
    }

    int valOPL = 0;

    #ifdef EMULATE_OPL2
    if ( activeFM )
    {
      ym3812_update_one( pOPL, &valOPL, 1 ); 

      if ( hack_OPL_Sample_Enabled )
        valOPL = (uint16_t)hack_OPL_Sample_Value << 5;

      // TODO asynchronous read back is an issue...
      //fmOutRegister = encodeGPIO( ym3812_read( pOPL, 0 ) ); 
    }
    #endif
#endif


    signed int sid1, sid2 = 0;

    int32_t left, right;

    if ( muteTest > 0 )
    {
      muteTest --;
      left = right = 0;
    } else
    {
      if ( useSID16 )
      {
        sid1 = sid16_1->output();
        if ( emulateSID2 && activeSID2 )
          sid2 = sid16_2->output();
      } else
      {
  #ifndef NO_RESID10
        sid1 = sid_1->output();
        if ( emulateSID2 && activeSID2 )
          sid2 = sid_2->output();
  #endif
      }

      //left  = ( sid1 * cfgVolSID1_Left  + sid2 * cfgVolSID2_Left  + valOPL * cfgVolOPL_Left ) >> 8;
      //right = ( sid1 * cfgVolSID1_Right + sid2 * cfgVolSID2_Right + valOPL * cfgVolOPL_Right ) >> 8;
  
      asm volatile ( "SMULBB %0, %1, %2" : "=r" (left) : "r" (sid1), "r" (actVolSID1_Left) );
      asm volatile ( "SMLABB %0, %1, %2, %3" : "=r" (left) : "r" (sid2), "r" (actVolSID2_Left), "r" (left) );
      asm volatile ( "SMLABB %0, %1, %2, %3" : "=r" (left) : "r" (valOPL), "r" (actVolOPL_Left), "r" (left) );
      left >>= 8;
  
      asm volatile ( "SMULBB %0, %1, %2" : "=r" (right) : "r" (sid1), "r" (actVolSID1_Right) );
      asm volatile ( "SMLABB %0, %1, %2, %3" : "=r" (right) : "r" (sid2), "r" (actVolSID2_Right), "r" (right) );
      asm volatile ( "SMLABB %0, %1, %2, %3" : "=r" (right) : "r" (valOPL), "r" (actVolOPL_Right), "r" (right) );
      right >>= 8;
      
  
      if ( samSpeaking )
      {
        int32_t sam = ( (int32_t)(*(int8_t*)&samvoice_raw[ samCurPos >> 1 ]) * samActive ) << 2;
  
        if ( ++ samCurPos == samEndPos1 )
          samCurPos = samStartPos2;
        if ( samCurPos >= samEndPos2 )
          samSpeaking = 0;
  
        left  += sam;
        right += sam;
      }
  
      //right = max( -32767, min( 32767, right ) );
      //left  = max( -32767, min( 32767, left ) );
      right = __SSAT( right );
      left  = __SSAT( left );

    }

#ifdef FANCY_LED

    if ( driveLEDs && collectStatistics && (( ++vu_nValues & 31 )==0) )
    {
      register uint32_t t;
      asm volatile ( "SMULBB %0, %1, %1" : "=r" (t) : "r" (left) );
      vuSumL += t >> 16;
      asm volatile ( "SMULBB %0, %1, %1" : "=r" (t) : "r" (right) );
      vuSumR += t >> 16;
  
      float scale = scaleSamples;
  
      if ( vu_nValues >= nSamplesCollected )
      {
        float tmp = ( (float)vuSumL/16384.0f / (float)nSamplesCollected * 32.0f );
        float vu_VolumeL = powf_fast( tmp, 0.247518f ) * scale;
  
        tmp = ( (float)vuSumR/16384.0f / (float)nSamplesCollected * 32.0f );
        float vu_VolumeR = powf_fast( tmp, 0.247518f ) * scale;

        {
          const float movAvg = 0.7f;
          static float ledValueAvgL = 0.0f;
          ledValueAvgL = ledValueAvgL * movAvg + vu_VolumeL * ( 1.0f - movAvg );
          static float ledValueAvgR = 0.0f;
          ledValueAvgR = ledValueAvgR * movAvg + vu_VolumeR * ( 1.0f - movAvg );
  
          nLEDsL = ledValueAvgL * 256.0f; 
          nLEDsR = ledValueAvgR * 256.0f; 
  
          nLEDs = (nLEDsL + nLEDsR) >> 1;
          nLEDsConfigTool = nLEDs * scaleSamplesConfigTool; 
        }
  
        vuSumL = 0;
        vuSumR = 0;
        vu_nValues = 0;
        collectStatistics = 0;
      }
    }
    delayLED ++;
#endif

    *( output++ ) = left;
    *( output2++ ) = right;

    nSamplesComputed++;
  }



totalCyclesPerBuffer += nCyclesPerBuffer;
nTotalCycleCounter ++;



  /*if ( nCyclesEmulated > cycleCount )
  {
    nCyclesEmulated = cycleCount;
  }*/
 
  transmit( block, 0 );
  transmit( block2, 1 );
  release( block );
  release( block2 );

#ifdef FANCY_LED

  static uint32_t lastVisUpdate = 0;
  static uint8_t updatePhase = 0;
  #define N_PHASES 32
  #define LEDS_UPD_PER_RUN 8

  if ( driveLEDs )
  {
    if ( delayLED > nSamplesCollected )
    {
      colorCycle += ledColorCycleSpeed * 4;
      delayLED = 0;
    }
    
    if ( ledWriteStatus == -1 && ( ( samplesTotal - lastVisUpdate ) > nSamplesCollected ) )// && ledColors2Write == 0 && ledBits2Write == 0 )
    {
      if ( updatePhase == 0 )
      {
        lastVisUpdate = samplesTotal;

        if ( ledColorCycleSpeed )
        {
          col0 = col0_cycle[ ( colorCycle >> 11 ) & 127 ];
          col1 = col1_cycle[ ( colorCycle >> 11 ) & 127 ];
        }
      }

      uint8_t startIdx = updatePhase * LEDS_UPD_PER_RUN;
      uint8_t endIdx = ( updatePhase + 1 ) * LEDS_UPD_PER_RUN;
      if ( endIdx > NUM_LEDS ) endIdx = NUM_LEDS;

      uint32_t i;
      switch ( ledVisMode )
      {
      default:
      case 0: // off
        for ( i = startIdx; i < endIdx; i++ )
          ledWriteRGB[ i ] = 0;
        break;
      case 1: // constant
        for ( i = startIdx; i < endIdx; i++ )
          ledWriteRGB[ i ] = col0;
        break;
      case 5: // mono peakmeter
        for ( i = startIdx; i < endIdx; i++ )
        {
          int v = ( (int)nLEDs * (int)NUM_LEDS / 8 - i * 32 ) * 2;
          if ( v < 0 ) v = 0; if ( v > 63 ) v = 63;
          int r = v * i / (int)( NUM_LEDS - 1 );

          ledWriteRGB[ i ] = SCALE( col0, ( v - r ) ) + SCALE( col1, r );
        }
        break;
      case 6: // stereo peakmeter
        for ( i = startIdx; i < endIdx; i++ )
        {
          int r, v;
          if ( i >= NUM_LEDS / 2 )
          {
            v = ( (int)nLEDsR * (int)NUM_LEDS / 16 - ( i - NUM_LEDS / 2 ) * 32 ) * 2;
            if ( v < 0 ) v = 0; if ( v > 63 ) v = 63;
            r = v * ( i - NUM_LEDS / 2 ) / (int)( NUM_LEDS / 2 - 1 );
          } else
          {
            v = ( (int)nLEDsL * (int)NUM_LEDS / 16 - i * 32 ) * 2;
            if ( v < 0 ) v = 0; if ( v > 63 ) v = 63;
            r = v * i / (int)( NUM_LEDS / 2 - 1 );
          }

      
          if ( i >= NUM_LEDS / 2 )
            ledWriteRGB[ i ] = SCALE( col0, ( v - r ) ) + SCALE( col1, r ); else
            ledWriteRGB[ NUM_LEDS / 2 - 1 - i ] = SCALE( col0, ( v - r ) ) + SCALE( col1, r );
        }
        ledColorsBufOfs = 0;
        break;
      case 3: // scroll left and right
      case 4:
        if ( startIdx == 0 )
        {
          int v = nLEDs * 4;
          if ( v > 255 ) v = 255;
          v = ( v * v ) >> 8;

          uint32_t color = LERP( col1, col0, ( v * v ) >> 8 );

          color = SCALE( color, v );

          if ( ledVisMode == 4 )
          {
            ledColorsBufOfs = ( ledColorsBufOfs + ( NUM_LEDS - 1 ) ) % NUM_LEDS;
            ledWriteRGB[ ledColorsBufOfs ] = color;
          } else
          {
            ledWriteRGB[ ledColorsBufOfs ] = color;
            ledColorsBufOfs = ( ledColorsBufOfs + 1 ) % NUM_LEDS;
          }
        }
        break;
      case 2: // pulse
        for ( i = startIdx; i < endIdx; i++ )
        {
          int t = (int)nLEDs * 2;
          if ( i >= NUM_LEDS / 2 )
            t *= (int)NUM_LEDS - i + 1; else
            t *= i + 2;
          t /= (int)( NUM_LEDS );

          int tt = t * 4;
          if ( tt > 255 ) tt = 255;
          tt = ( tt * tt ) >> 8;

          uint32_t c1 = SCALE( col0, tt );
          if ( nLEDs > 32 )
          {
            t = ( t * ( nLEDs - 32 ) ) / 16;
            if ( t > 255 ) t = 255;
            t = ( t * t ) >> 8;
            uint32_t c2 = SCALE( col1, t );
            c1 = ADD_CLAMP( c1, c2 );
          }

          ledWriteRGB[ i ] = c1;
        }

        break;
      }

      if ( endIdx >= NUM_LEDS - 1 )
      {
        if ( ledVisMode != 3 && ledVisMode != 4 )
          ledColorsBufOfs = 0;
        ledColors2Write = NUM_LEDS;
        ledWriteStatus = 0;
        updatePhase = 0;
        collectStatistics = 1;
      } else
        updatePhase++;
    }
  }
#endif     
  
}

void AudioStreamSID::reset()
{
  firstBufferAfterReset = 1;
  recalibrateFrequency = 1;
  muteTest = 768;

  #ifdef FANCY_LED
  driveLEDs = (ledVisMode > 0) ? true : false;
  #endif

  activeSID2 = activeSID2Prev = false;
  activeFM   = activeFMPrev   = false;
  updateMixer( false, false );

  for ( int i = 0; i <= 24; i++ )
  {
#ifndef NO_RESID10
    sid_1->write( i, 0 );
    sid_2->write( i, 0 );
#endif
    sid16_1->write( i, 0 );
    sid16_2->write( i, 0 );
  }
  
  #ifdef EMULATE_OPL2
  ym3812_reset_chip( pOPL );
  fmFakeOutput = 0;
  #endif
}

void AudioStreamSID::fillBlock()
{
  audio_block_t *block;
  audio_block_t *block2;

  block = allocate();
  block2 = allocate();

  if ( block == NULL || block2 == NULL ) return;

  memset( block->data, 0, sizeof( int16_t ) * AUDIO_BLOCK_SAMPLES );
  memset( block2->data, 0, sizeof( int16_t ) * AUDIO_BLOCK_SAMPLES );
 
  transmit( block, 0 );
  transmit( block2, 1 );
  release( block );
  release( block2 );
}
