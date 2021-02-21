/*
   _________.___________   ____  __.__        __     
  /   _____/|   \______ \ |    |/ _|__| ____ |  | __ 
  \_____  \ |   ||    |  \|      < |  |/ ___\|  |/ / 
  /        \|   ||    `   \    |  \|  \  \___|    <  
 /_______  /|___/_______  /____|__ \__|\___  >__|_ \ 
         \/             \/        \/       \/     \/ 
        
 AudioStreamSID.cpp

 SIDKick - SID-replacement with SID and Sound Expander Emulation based on Teensy 4.1
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

#include "AudioStreamSID.h"
#include <math.h>
#include <AudioStream.h>
#include "samvoice.h"

#define SAMPLERATE AUDIO_SAMPLE_RATE_EXACT
unsigned int CLOCKFREQ = 985248;

bool useSID16 = true;
bool emulateSID2 = true, emulateFM = false, readRegistersFM = false, activeSID2 = false, activeFM = false, registerRead = true;

uint32_t SID1_MODEL = 6581;
uint32_t SID2_MODEL = 6581;
uint32_t SID2_ADDR = 0xffffffff;

int32_t cfgVolSID1_Left, cfgVolSID1_Right;
int32_t cfgVolSID2_Left, cfgVolSID2_Right;
int32_t cfgVolOPL_Left, cfgVolOPL_Right;

#define max( a, b ) ((a)>(b)?(a):(b))
#define min( a, b ) ((a)<(b)?(a):(b))

uint32_t samCurPos, samEndPos1, samStartPos2, samEndPos2;
uint8_t samSpeaking = 0;

void speakSAM( uint8_t digit )
{
  samCurPos    = 0;
  samEndPos1   = ( 18 * 1024 - 1 ) << 1;
  samStartPos2 = samEndPos1 + ( ( digit * 12 * 1024 ) << 1 );
  samEndPos2   = samStartPos2 + ( ( 12 * 1024 ) << 1 );
  samSpeaking  = 1;
}

void AudioStreamSID::init()
{
  sid16_1 = new SID16();
  sid16_2 = new SID16();

  sid_1 = new RESID_NAMESPACE::SID();
  sid_2 = new RESID_NAMESPACE::SID();
  
  #ifdef EMULATE_OPL2
  pOPL = ym3812_init( 3579545, SAMPLERATE );
  ym3812_reset_chip( pOPL );
  fmFakeOutput = 0;
  #endif
	
	//this->reset();

  sid16_1->set_chip_model( MOS6581 );
  sid16_2->set_chip_model( MOS6581 );
  sid16_1->reset();
  sid16_2->reset();
  sid16_1->set_sampling_parameters( CLOCKFREQ, SAMPLE_INTERPOLATE, AUDIO_SAMPLE_RATE_EXACT );
  sid16_2->set_sampling_parameters( CLOCKFREQ, SAMPLE_INTERPOLATE, AUDIO_SAMPLE_RATE_EXACT );
  
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

  sid_1->adjust_filter_bias(SID_filterbias / 1000.0f);
  sid_2->adjust_filter_bias(SID_filterbias / 1000.0f);

  sid_1->set_sampling_parameters( CLOCKFREQ, RESID_NAMESPACE::SAMPLE_FAST, AUDIO_SAMPLE_RATE_EXACT, AUDIO_SAMPLE_RATE_EXACT * SID_passband / 200.0f, SID_gain / 100.0f );
  sid_2->set_sampling_parameters( CLOCKFREQ, RESID_NAMESPACE::SAMPLE_FAST, AUDIO_SAMPLE_RATE_EXACT, AUDIO_SAMPLE_RATE_EXACT * SID_passband / 200.0f, SID_gain / 100.0f );

  activeSID2 = false;
  activeFM   = false;
}

void AudioStreamSID::updateConfiguration( uint8_t *cfg )
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
  useSID16     = cfg[ 19 ] == 0 ? true : false;  
  emulateFM    = cfg[ 16 ] > 0 ? true : false;
  readRegistersFM = cfg[ 16 ] == 1 ? true : false;
  emulateSID2  = cfg[  8 ] < 3 ? true : false;  

  if ( cfg[0] == 0 )
  {
    SID1_MODEL = 6581;
    sid16_1->set_chip_model( MOS6581 );
    sid_1->set_chip_model( RESID_NAMESPACE::MOS6581 );
  } else
  {
    SID1_MODEL = 8580;
    sid16_1->set_chip_model( MOS8580 );
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
  }

  if ( cfg[8] == 0 )
  {
    SID2_MODEL = 6581;
    sid16_2->set_chip_model( MOS6581 );
    sid_2->set_chip_model( RESID_NAMESPACE::MOS6581 );
  } else
  if ( cfg[8] != 3 )
  {
      SID2_MODEL = 8580;
      sid16_2->set_chip_model( MOS8580 );
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
  } else
    SID2_MODEL = 0;

  cfgVolSID1_Left  = (int)cfg[ 3] * (int)( 14 - cfg[ 4] );
  cfgVolSID1_Right = (int)cfg[ 3] * (int)( cfg[ 4] );
  cfgVolSID2_Left  = (int)cfg[11] * (int)( 14 - cfg[12] );
  cfgVolSID2_Right = (int)cfg[11] * (int)( cfg[12] );
  cfgVolOPL_Left   = (int)cfg[17] * (int)( 14 - cfg[18] );
  cfgVolOPL_Right  = (int)cfg[17] * (int)( cfg[18] );

  if ( !emulateSID2 )
    cfgVolSID2_Left = cfgVolSID2_Right = 0;
  if ( !emulateFM )
    cfgVolOPL_Left = cfgVolOPL_Right = 0;

  int maxVolFactor = max( cfgVolSID1_Left, max( cfgVolSID1_Right, max( cfgVolSID2_Left, max( cfgVolSID2_Right, max( cfgVolOPL_Left, cfgVolOPL_Right ) ) ) ) );

  cfgVolSID1_Left  = cfgVolSID1_Left  * 256 / maxVolFactor;
  cfgVolSID1_Right = cfgVolSID1_Right * 256 / maxVolFactor;
  cfgVolSID2_Left  = cfgVolSID2_Left  * 256 / maxVolFactor;
  cfgVolSID2_Right = cfgVolSID2_Right * 256 / maxVolFactor;
  cfgVolOPL_Left   = cfgVolOPL_Left   * 256 / maxVolFactor;
  cfgVolOPL_Right  = cfgVolOPL_Right  * 256 / maxVolFactor;

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
 
}


void AudioStreamSID::begin()
{
  playing = true;
}

void AudioStreamSID::stop()
{
	//__disable_irq();
	playing = false;
	//__enable_irq();
}

void AudioStreamSID::continuePlaying()
{
  //__disable_irq();
  playing = true;
  //__enable_irq();
}

extern uint32_t c64CycleCount;
extern uint32_t nCyclesEmulated;

#define RING_SIZE (1024*4)
extern uint32_t ringBufGPIO[ RING_SIZE ];
extern uint32_t ringTime[ RING_SIZE ];
extern uint32_t ringWrite;

uint32_t ringRead = 0;
unsigned long long samplesElapsed = 0;

uint32_t samplesTotal = 0;

void AudioStreamSID::update()
{
  audio_block_t *block;
  audio_block_t *block2;

	if ( !playing ) return;

  block = allocate();
  block2 = allocate();
	if ( block == NULL ) return;

	uint32_t nSamplesComputed = 0;
  int16_t *output = (int16_t*)block->data;
  int16_t *output2 = (int16_t*)block2->data;

	uint32_t cycleCount = c64CycleCount;

  samplesTotal += AUDIO_BLOCK_SAMPLES;

	while ( nSamplesComputed < AUDIO_BLOCK_SAMPLES )
	{

		unsigned long long samplesElapsedBefore = samplesElapsed;

    long long cycleNextSampleReady = ( ( unsigned long long )(samplesElapsedBefore+1) * ( unsigned long long )CLOCKFREQ ) / ( unsigned long long )AUDIO_SAMPLE_RATE_EXACT;
    int32_t cyclesToNextSample = cycleNextSampleReady - nCyclesEmulated;

		do { // do SID emulation until time passed to create an additional sample (i.e. there may be several loops until a sample value is created)

      int32_t cyclesToEmulate = 16; 

      // this really eats time, only do this if Teensy is overclocked or SID2 is non-active
      if ( !activeSID2 )
      {
        // cyclesToEmulate might be reduced because of ... 
        // a) next sample is due earlier, ...
        // this should super-slightly increase sound quality, however, requires overclocking already when 2 SIDs + FM are emulated
        if ( cyclesToEmulate > cyclesToNextSample )
          cyclesToEmulate = cyclesToNextSample;
        if ( cyclesToEmulate == 0 )
          cyclesToEmulate = 1;
      }
      
      // ... and b) next SID-command comes earlier
      // this is crucial to play LMAN's tunes
      if ( ringRead != ringWrite )
      {
          int  cyclesToNextWrite = ringTime[ ringRead ] - nCyclesEmulated;

          if ( cyclesToEmulate > cyclesToNextWrite )
            cyclesToEmulate = cyclesToNextWrite;
      }

      if ( cyclesToEmulate == 0 )
        cyclesToEmulate = 1;

      extern uint8_t outRegisters[ 32 ];
      extern uint8_t outRegisters_2[ 32 ];
      if ( useSID16 )
      {
        sid16_1->clock( cyclesToEmulate );
        if ( emulateSID2 && activeSID2 )
        {
          sid16_2->clock( cyclesToEmulate );
          outRegisters_2[ 0x1b ] = 0;//sid16_2->read( 0x1b );
          outRegisters_2[ 0x1c ] = 0;//sid16_2->read( 0x1c );
        }
        outRegisters[ 0x1b ] = sid16_1->read( 0x1b );
        outRegisters[ 0x1c ] = sid16_1->read( 0x1c );
      } else
      {
        sid_1->clock( cyclesToEmulate );
        if ( emulateSID2 && activeSID2 )
        {
          sid_2->clock( cyclesToEmulate );
          outRegisters_2[ 0x1b ] = 0;//sid_2->read( 0x1b );
          outRegisters_2[ 0x1c ] = 0;//sid_2->read( 0x1c );
        }
        outRegisters[ 0x1b ] = sid_1->read( 0x1b );
        outRegisters[ 0x1c ] = sid_1->read( 0x1c );
      }

      nCyclesEmulated += cyclesToEmulate;
      cyclesToNextSample -= cyclesToEmulate;

			unsigned int readUpTo = ringWrite;

			if ( ringRead != readUpTo && nCyclesEmulated >= ringTime[ ringRead ] )
			{
				unsigned char A, D;
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
          if ( ( ( A & ( 1 << 4 ) ) == 0 ) )
            ym3812_write( pOPL, 0, D ); else
            ym3812_write( pOPL, 1, D );
          #endif
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
            if ( ringBufGPIO[ ringRead ] & (1<<20) )
            {
              sid_2->write( A & 31, D ); 
              activeSID2 = true;
            } else
              sid_1->write( A & 31, D );
          }
        }
        
				ringRead++;
				ringRead &= ( RING_SIZE - 1 );
			}

			samplesElapsed = ( ( unsigned long long )nCyclesEmulated * ( unsigned long long )AUDIO_SAMPLE_RATE_EXACT ) / ( unsigned long long )CLOCKFREQ;

		} while ( samplesElapsed == samplesElapsedBefore );

   //output:

    int valOPL = 0;
    #ifdef EMULATE_OPL2
    if ( activeFM )
    {
      ym3812_update_one( pOPL, &valOPL, 1 );
      // TODO asynchronous read back is an issue, needs to be fixed
      //fmOutRegister = encodeGPIO( ym3812_read( pOPL, 0 ) ); 
    }
    #endif

    signed int sid1, sid2 = 0;
    if ( useSID16 )
    {
      sid1 = sid16_1->output();
      if ( emulateSID2 && activeSID2 )
        sid2 = sid16_2->output();
    } else
    {
      sid1 = sid_1->output();
      if ( emulateSID2 && activeSID2 )
        sid2 = sid_2->output();
    }

    int32_t left, right;
    
    left  = ( sid1 * cfgVolSID1_Left  + sid2 * cfgVolSID2_Left  + valOPL * cfgVolOPL_Left ) >> 8;
    right = ( sid1 * cfgVolSID1_Right + sid2 * cfgVolSID2_Right + valOPL * cfgVolOPL_Right ) >> 8;

    if ( samSpeaking )
    {
      int32_t sam = (int32_t)(*(int8_t*)&samvoice_raw[ samCurPos >> 1 ]) << 7;

      if ( ++ samCurPos == samEndPos1 )
        samCurPos = samStartPos2;
      if ( samCurPos >= samEndPos2 )
        samSpeaking = 0;

      left  += sam;
      right += sam;
    }

    right = max( -32767, min( 32767, right ) );
    left  = max( -32767, min( 32767, left ) );
 
    *( output++ ) = left;
    *( output2++ ) = right;

		nSamplesComputed++;
	}

  if ( nCyclesEmulated > cycleCount )
  	nCyclesEmulated = cycleCount;
 
  transmit( block,0 );
  transmit( block2,1 );
  release( block );
  release( block2 );
}

void AudioStreamSID::reset()
{
  activeSID2 = false;
  activeFM   = false;

  for ( int i = 0; i <= 24; i++ )
  {
    sid_1->write( i, 0 );
    sid_2->write( i, 0 );
    sid16_1->write( i, 0 );
    sid16_2->write( i, 0 );
  }
  
  #ifdef EMULATE_OPL2
  ym3812_reset_chip( pOPL );
  fmFakeOutput = 0;
  #endif
}
