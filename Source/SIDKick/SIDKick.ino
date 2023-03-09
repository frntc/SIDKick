/*
   _________.___________   ____  __.__        __
  /   _____/|   \______ \ |    |/ _|__| ____ |  | __
  \_____  \ |   ||    |  \|      < |  |/ ___\|  |/ /
  /        \|   ||    `   \    |  \|  \  \___|    <
 /_______  /|___/_______  /____|__ \__|\___  >__|_ \
         \/             \/        \/       \/     \/


  SIDKick.ino

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

/*
   pinout for extenion header (X1 to X5)
   MIDI IN/OUT = X1/X2 (GPIO 28/29)
   MIDI IRQ    = X3    (GPIO 30)
   LED         = X4    (GPIO 31)
   BUTTON      = X5    (GPIO 33)
*/

#include "globals.h"

#include <EEPROM.h>
#include <Audio.h>
#include "AudioStreamSID.h"
#include "imxrt.h"
#include "utility/imxrt_hw.h"

#include "launch.h"     // trampoline code for launching PRGs
#include "prgslots.h"   // memory for PRG-launcher


#define globalRestPoolSize (120*1024)
static uint32_t globalRestPoolOfs = 0;
static unsigned char globalRestMemPool[ globalRestPoolSize ];

void *allocPool( int size )
{
  void *p = (void*)&globalRestMemPool[ globalRestPoolOfs ];
  globalRestPoolOfs += size;
  return p;
}

uint32_t prgSize = 0;
const uint8_t *prgCode = NULL;

#include "config.h"     // built-in config tool

#ifdef FIRMWARE_C128
#if audioDevice == 1
static const char VERSION_STR_Flash[20] = {0x53, 0x49, 0x44, 0x4b, 0x09, 0x03, 0x0b, '0', '.', '2', 0x44, 0x41, 0x43, '1', '2', '8' };
#else
static const char VERSION_STR_Flash[20] = {0x53, 0x49, 0x44, 0x4b, 0x09, 0x03, 0x0b, '0', '.', '2', 0x4d, 0x51, 0x53, '1', '2', '8' };
#endif
#else
#if audioDevice == 1
static const char VERSION_STR_Flash[20] = {0x53, 0x49, 0x44, 0x4b, 0x09, 0x03, 0x0b, '0', '.', '2', '/', 0x44, 0x41, 0x43, '6', '4' };
#else
static const char VERSION_STR_Flash[20] = {0x53, 0x49, 0x44, 0x4b, 0x09, 0x03, 0x0b, '0', '.', '2', '/', 0x4d, 0x51, 0x53, '6', '4' };
#endif
#endif

// signature + version 0.21
#define VERSION_STR_EXT_SIZE  16
static const unsigned char VERSION_STR_ext[VERSION_STR_EXT_SIZE] = { 
  0x53, 0x49, 0x44, 0x4b, 0x09, 0x03, 0x0b, 0x00,   // signature + extension version 0
  0, 21,                                            // firmware version with stepping
#ifdef SID_DAC_MODE_SUPPORT                         // support DAC modes? which?
  1, 
#else
  0, 
#endif
  0, 0, 0, 0, 0 };

// new +8 Bytes extended version information
char VERSION_STR[20 + VERSION_STR_EXT_SIZE];

const uint8_t portPinsC[ 5 ] PROGMEM = {  0, 24, 25, 26, 27 };                // A0..4 (GPIOs on port 6)
const uint8_t portPinsD[ 8 ] PROGMEM = { 23, 22, 16, 17, 15, 14, 18, 19 };    // D0..7 (GPIOs on port 6)
const uint8_t portPinsA[ 7 ] PROGMEM = { 34, 35, 36, 37, 11, 13, 32 };        // A5..8, IO1, IO2, RESET (GPIOs on port 7)

#define S_A5   CORE_PIN34_BITMASK
#define S_A6   CORE_PIN35_BITMASK
#define S_A7   CORE_PIN36_BITMASK
#define S_A8   CORE_PIN37_BITMASK

#define DUAL_SID_MASK      (S_A5 | S_A8 | CORE_PIN11_BITMASK | CORE_PIN13_BITMASK )
#define DUAL_SID_ADDR_MASK (S_A5 | S_A8)
#define DUAL_SID_FLIPMASK  (CORE_PIN11_BITMASK | CORE_PIN13_BITMASK)

const uint8_t pinPHI2  = 1,   // GPIO on port 6
              pinRW    = 2,   // GPIO on port 9
              pinCS    = 3,   // GPIO on port 9

              potX     = 4,   // GPIO on ...
              potY     = 5,   // ... port 9
              pin245OE = 6;   // GPIO on port 6

const uint8_t pinButtonProfile = 33;   // GPIO on port 9

uint8_t outRegisters[ 32 ];
uint8_t outRegisters_2[ 32 ];

AudioStreamSID    *playSID;
AudioOutputMQS    *mqs;
AudioOutputPT8211 *audioPT8211;

uint32_t debounceCycle = 0;
uint32_t c64CycleCount, c64CycleCountResetReleased;
uint32_t nCyclesEmulated;

#ifdef SID_WRITE_ONLY_MODE_SUPPORT
#define SID_ONLY_OFF      0
#define SID_ONLY_ENABLE   1
#define SID_ONLY_DISABLE  2
#define SID_ONLY_ACTIVE   3
uint8_t  sidOnlyMode = SID_ONLY_OFF;
#endif

#ifdef SID_DAC_MODE_SUPPORT
uint8_t sidDACMode = SID_DAC_OFF;
#endif

uint8_t  addrLine = 0;
uint32_t fmFakeOutput = 0;
uint32_t fmAutoDetectStep = 0;
uint32_t sidFakeOutput = 0;
uint32_t sidAutoDetectStep = 0;
uint32_t sidAutoDetectStep_2 = 0;
uint8_t  sidAutoDetectRegs[ 32 ];
uint8_t  sidAutoDetectRegs_2[ 32 ];

uint32_t nSIDFMDelay = 0;
uint32_t nMIDIDelay = 0;

const uint8_t midiPinOut = 29;
const uint8_t midiPinIn  = 28;
const uint8_t midiIRQPin = 30;

#define MIDI_IRQ_PIN_LOW    { CORE_PIN30_PORTCLEAR = CORE_PIN30_BITMASK; }
#define MIDI_IRQ_PIN_HIGH   { CORE_PIN30_PORTSET = CORE_PIN30_BITMASK;   }

#define MIDI_OUT_PIN_LOW    { CORE_PIN29_PORTCLEAR = CORE_PIN29_BITMASK; }
#define MIDI_OUT_PIN_HIGH   { CORE_PIN29_PORTSET = CORE_PIN29_BITMASK;   }

#ifdef SUPPORT_MIDI

uint8_t MIDI_START_ADDR   = 0x04;
uint8_t MIDI_END_ADDR     = 0x07;

bool activeMIDI_In  = false,
     activeMIDI_Out = false;

uint32_t midiDelayCyclesFP16 = 2066198;
uint32_t midiWrite = 0;
int32_t  midiWriteBit = 65536;

// MIDI-interface register offsets
const uint8_t MIDI_CONTROL_REG  = 0x00;
const uint8_t MIDI_STATUS_REG   = 0x02;
const uint8_t MIDI_TRANSMIT_REG = 0x01;
const uint8_t MIDI_RECEIVE_REG  = 0x03;

uint8_t MIDIControlReg = 0, MIDIStatusReg = 0;
uint8_t MIDINextReceivedByte = 0;

uint8_t MIDIFetchReceivedByte = 1;
int16_t midiSerialWrite = -1;

uint8_t midiAutoDetectStep = 0;

static uint32_t clock2midi_Counter   = 0;
static uint32_t midiIn_DetectedStartBitThreshold = 28;

#define MIDI_RING_SIZE (64*16)
uint32_t MIDI_ringBuf[ MIDI_RING_SIZE ];
uint32_t MIDI_ringTime[ MIDI_RING_SIZE ];
uint16_t MIDI_ringWrite;
uint16_t MIDI_ringRead;

uint32_t MIDI_ringBufIn[ MIDI_RING_SIZE ];
uint16_t MIDI_ringWriteIn;
uint16_t MIDI_ringReadIn;

static uint8_t releasedIRQ = 1;
static uint8_t IRQconditionTX = 0, IRQconditionRX = 0;
static uint8_t doNotTriggerIRQAgainThisCycle = 0;

#endif

#ifdef FANCY_LED
#define MAX_NUM_LEDS 32
const uint8_t ledPin = 31;
#ifdef FIRMWARE_C128
const int LED_HIGH_DURATION = 210; // ISR takes long enough that this explicit delay is unnecessary, except for SID-write-only-mode
#else
const int LED_HIGH_DURATION = 175; // ISR takes long enough that this explicit delay is unnecessary, except for SID-write-only-mode
#endif
uint32_t NUM_LEDS = MAX_NUM_LEDS;
uint32_t ledWriteRGB[MAX_NUM_LEDS] = { 0 };
uint8_t  ledCurRGB = 0;
uint8_t  ledBits2Write = 0;
uint8_t  ledColors2Write = 0;
int32_t  ledWriteStatus = 0;
uint8_t  ledColorsBufOfs = 0;
uint8_t  ledPinStatus = 0;
extern uint32_t nLEDsConfigTool;
#else
uint32_t nLEDsConfigTool = 0;
#endif

#define RING_SIZE (4096)
uint32_t ringBufGPIO[ RING_SIZE ];
uint32_t ringTime[ RING_SIZE ];
uint16_t ringWrite;
extern uint16_t ringRead;
extern unsigned long long samplesElapsed;
extern uint32_t samplesTotal;

volatile uint32_t dummy;

int doReset    = 0;
int disableISR = 0;
uint8_t runningOnPAL = 1;

#define CACHE_PRELOAD( ptr ) { asm volatile ("pld\t[%0]" :: "r" (ptr)); }
#define CACHE_PRELOADW( ptr ) { asm volatile ("pldw\t[%0]" :: "r" (ptr)); }
#define CACHE_PRELOADI( ptr ) { asm volatile ("pli\t[%0]" :: "r" (ptr)); }

extern bool registerRead;
static uint32_t stateReadDirectoryMode = 0;
static uint32_t stateGoingTowardsTransferMode = 0;
static uint32_t stateInTransferMode = 0;
static uint32_t stateInConfigMode = 0;
static uint32_t stateInVisualizationMode = 0;
static uint32_t stateConfigRegisterAccess = 0;
static uint32_t stateWriteConfigToEEPROM = 0;
static uint8_t configUpdatedTemporarily = 0;

static const uint8_t  *transferData = NULL;
static uint8_t        *transferDataEnd = NULL;
static uint32_t       transferAddress,
       jumpAddress,
       launcherAddress,
       transferStage;    // 0 = copy launcher, 1 = copy PRG, 2 = done

#define CONFIG_MODE_CYCLES 25000
#define TRANSFER_MODE_CYCLES 256
#define MAX_SETTINGS 64
uint8_t stateConfig[ MAX_SETTINGS * 11 ];
uint8_t stateConfigPrevious[ MAX_SETTINGS * 11 ];

const unsigned char rangeSettings[ MAX_SETTINGS ] PROGMEM  =
{ 4, 16, 2, 16, 15, 91, 11, 101, 4, 16, 6, 16, 15, 91, 11, 101, 3, 16, 15, 2,
  7, 32, 32, 32, 254, 90, 32, 32, 90, 32, 32, 5, 254, 2, 127, 127, 254, 254,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

void readConfigFromEEPROM()
{
  for (int i = 0; i < MAX_SETTINGS * 11; i++)
    stateConfig[ i ] = EEPROM.read(i);

  bool allOK = true;

  for ( int i = 0; i < MAX_SETTINGS * 10; i++ )
  {
    if ( stateConfig[ i ] >= rangeSettings[ i % MAX_SETTINGS ] )
    {
      stateConfig[ i ] = 0;
      allOK = false;
    }
  }

  if ( stateConfig[ MAX_SETTINGS * 10 ] > 9 )
  {
    stateConfig[ MAX_SETTINGS * 10 ] = 0;
    allOK = false;
  }

  if ( !allOK )
    for (int i = 0; i < MAX_SETTINGS * 11; i++)
      EEPROM.write( i, stateConfig[ i ] );
}

void writeConfigToEEPROM()
{
  for (int i = 0; i < MAX_SETTINGS * 11; i++)
    EEPROM.write( i, stateConfig[ i ] );
}

#define CPU_RESET_CYCLECOUNTER    { ARM_DEMCR |= ARM_DEMCR_TRCENA; ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA; ARM_DWT_CYCCNT = 0; }

void setD07_Write();

//static uint32_t nTotalTeensyCycles = 0;

void setup()
{
  CORE_PIN34_CONFIG = CONFIG_PULLUP;
  CORE_PIN35_CONFIG = CONFIG_PULLUP;
  CORE_PIN36_CONFIG = CONFIG_PULLUP;
  CORE_PIN37_CONFIG = CONFIG_PULLUP;
  CORE_PIN11_CONFIG = CONFIG_PULLUP;
  CORE_PIN13_CONFIG = CONFIG_PULLUP;
  CORE_PIN32_CONFIG = CONFIG_PULLUP;

  for ( int i = 0; i < 5; i++ )
    pinMode( portPinsC[ i ], INPUT );

  for ( int i = 0; i < 7; i++ )
    pinMode( portPinsA[ i ], INPUT );

  pinMode( pinPHI2, INPUT );
  pinMode( pinRW, INPUT );
  pinMode( pinCS, INPUT );

  pinMode( pin245OE, OUTPUT );
  CORE_PIN6_PORTSET = CORE_PIN6_BITMASK;

  pinMode( pinButtonProfile, INPUT_PULLUP );

  pinMode( potX, OUTPUT );
  pinMode( potY, OUTPUT );

  setD07_Write();

  delay( 5 );

  disableISR = 1;

  memset( outRegisters, 0, 32 );
  memset( outRegisters_2, 0, 32 );

  memcpy( prgDirectory, prgDirectory_Flash, 36 * 24 + 1 );

#ifdef DEBUG_OUTPUT
  Serial.begin( 9600 );
  Serial.println( "SIDKick here... ");
#endif

  memcpy( VERSION_STR, VERSION_STR_Flash, 20 );
  memcpy( VERSION_STR + 20, VERSION_STR_ext, VERSION_STR_EXT_SIZE );

  cfgPRGCode = new unsigned char[ cfgPRGCode_size ];
  memcpy( cfgPRGCode, cfgPRGCode_Flash, cfgPRGCode_size );

  runningOnPAL = 1;

  c64CycleCount = c64CycleCountResetReleased = 0;
  attachInterrupt( digitalPinToInterrupt( pinPHI2 ), isrDetermineClockFrequency, RISING );
  while ( c64CycleCount < 1000 ) {
    delay( 1 );
  }
  detachInterrupt( digitalPinToInterrupt( pinPHI2 ) );

  c64CycleCount = c64CycleCountResetReleased = 0;
  attachInterrupt( digitalPinToInterrupt( pinPHI2 ), isrSID, RISING );

  readConfigFromEEPROM();
  uint8_t activeProfile = stateConfig[ MAX_SETTINGS * 10 ];
  if ( activeProfile >= 10 )
    activeProfile = 0;

  if ( !runningOnPAL ) // NTSC?
  {
    CLOCKFREQ = CLOCKFREQ_NOMINAL = 1022727;
#ifdef SUPPORT_MIDI
    midiDelayCyclesFP16 = 2144814;
    midiIn_DetectedStartBitThreshold = 29;
#endif
#ifdef DEBUG_OUTPUT
    Serial.println( "running on NTSC" );
#endif
  } else
  {
    CLOCKFREQ = CLOCKFREQ_NOMINAL = 985248;
#ifdef SUPPORT_MIDI
    midiDelayCyclesFP16 = 2066202; // CLOCKFREQ * 65536 / 31250
    midiIn_DetectedStartBitThreshold = 28;
#endif
#ifdef DEBUG_OUTPUT
    Serial.println( "running on PAL" );
#endif
  }

  c64CycleCount =
  c64CycleCountResetReleased = 
  nCyclesEmulated =
  fmFakeOutput =
  fmAutoDetectStep =
  sidFakeOutput =
  sidAutoDetectStep =
  sidAutoDetectStep_2 =
  ringRead = ringWrite =
  samplesElapsed =
  samplesTotal = 0;

#ifdef FANCY_LED
  pinMode( ledPin, OUTPUT );
  //digitalWrite( ledPin, LOW );
  CORE_PIN31_PORTCLEAR = CORE_PIN31_BITMASK;
  ledPinStatus = 0;

  ledColorsBufOfs = 0;
  ledColors2Write = 0;
  ledWriteStatus = 0;
#endif

  // to make sure we don't have undefined behaviour
  pinMode( midiPinIn, INPUT );
  pinMode( midiPinOut, OUTPUT );
  pinMode( midiIRQPin, OUTPUT );
  MIDI_IRQ_PIN_LOW

#ifdef SUPPORT_MIDI
  MIDI_ringRead = MIDI_ringWrite = 0;
  MIDI_ringReadIn = MIDI_ringWriteIn = 0;
#endif

  dummy = prgDirectory[ 0 ];
  dummy = prgRepository[ 0 ];
  memset( globalRestMemPool, 0, globalRestPoolSize );

  for ( uint32_t i = 0; i < cfgPRGCode_size; i++ )
    dummy = cfgPRGCode[ i ];

  playSID = new AudioStreamSID;

  playSID->init();

  playSID->updateConfiguration( &stateConfig[ activeProfile * MAX_SETTINGS ], &stateConfig[ 10 * MAX_SETTINGS ] );
  memcpy( stateConfigPrevious, stateConfig, MAX_SETTINGS * 11 );

#ifdef DEBUG_OUTPUT
//dummyptr = new unsigned char[ SIZE ];
//unsigned char globalRestMemPool[ 100000 ];

const int SIZE = 60000;
//memset( globalRestMemPool, 0, SIZE );
  Serial.println( "malloc successful");

#endif

  playSID->begin();

#if ( audioDevice == 0 )
  mqs = new AudioOutputMQS();
  AudioMemory( 512 );
  new AudioConnection(*playSID, 0, *mqs, 0);
  new AudioConnection(*playSID, 1, *mqs, 1);
#else
  audioPT8211 = new AudioOutputPT8211();
  AudioMemory( 512 );
  new AudioConnection(*playSID, 0, *audioPT8211, 0);
  new AudioConnection(*playSID, 1, *audioPT8211, 1);
#endif

  disableISR = 0;
  doReset = 1;
}

void resetEmulation()
{
  disableISR = 1;

  stateInConfigMode =
  stateInTransferMode = 
  stateInVisualizationMode = 
  stateConfigRegisterAccess =
  stateReadDirectoryMode =
  debounceCycle =
  ringRead =
  ringWrite =
  nCyclesEmulated =
  samplesElapsed =
  samplesTotal = 0;
  c64CycleCountResetReleased = 
  c64CycleCount = 10;

#ifdef SID_WRITE_ONLY_MODE_SUPPORT
  if ( sidOnlyMode == SID_ONLY_DISABLE || sidOnlyMode == SID_ONLY_ACTIVE )
  {
    detachInterrupt( digitalPinToInterrupt( pinPHI2 ) );
    attachInterrupt( digitalPinToInterrupt( pinPHI2 ), isrSID, RISING );
    Serial.println( "[init] normal mode");
  }
  sidOnlyMode = SID_ONLY_OFF;
#endif
#ifdef SID_DAC_MODE_SUPPORT
  sidDACMode = SID_DAC_OFF;
#endif

  extern uint8_t firstBufferAfterReset;
  firstBufferAfterReset = 0;

  extern bool activeSID2, activeFM, activeSID2Prev, activeFMPrev;

  activeSID2 = activeSID2Prev = false;
  activeFM   = activeFMPrev   = false;

#ifdef SUPPORT_MIDI
  activeMIDI_In = activeMIDI_Out = false;
#endif

  stateInTransferMode = 0;
  transferData = (uint8_t *)&launchCode[ 2 ];
  transferDataEnd = (uint8_t *)&launchCode[ launchSize ];

  /*  if ( configUpdatedTemporarily )
    {
      configUpdatedTemporarily = 0;
      readConfigFromEEPROM();

      uint8_t activeProfile = stateConfig[ MAX_SETTINGS * 10 ];
      playSID->updateConfiguration( &stateConfig[ activeProfile * MAX_SETTINGS ] );
      memcpy( stateConfigPrevious, stateConfig, MAX_SETTINGS * 11 );
    }*/

  // send special command to reset sound chips
  ringBufGPIO[ ringWrite ] = 0xffffffff;
  ringTime[ ringWrite ] = c64CycleCount;
  ringWrite++;
  ringWrite &= ( RING_SIZE - 1 );
  c64CycleCount ++;

#ifdef SUPPORT_MIDI

  MIDIControlReg = MIDIStatusReg = 0;
  MIDI_ringWrite = MIDI_ringRead = 0;
  MIDI_ringWriteIn = MIDI_ringReadIn = 0;


#endif

  fmFakeOutput =
  fmAutoDetectStep = 0;

  sidFakeOutput =
  sidAutoDetectStep =
  sidAutoDetectStep_2 = 0;

  for ( int i = 0; i < 20; i++ )
  {
    sidAutoDetectRegs[ i ] = 0;
    sidAutoDetectRegs_2[ i ] = 0;
  }

  disableISR = 0;
}


uint32_t accCycleCounter = 0;
uint32_t accCycleCounterMin = 1000000;
uint32_t accCycleCounterMax = 0;
uint32_t accCycleN = 0;

void loop()
{
  if ( doReset == 1 )
  {
    doReset = 2;
#ifdef DEBUG_OUTPUT
    Serial.println( "reset emulation" );
#endif
    resetEmulation();
#ifdef SUPPORT_MIDI
    MIDIControlReg = 3;
    MIDIStatusReg &= 127;
    releasedIRQ = 1;
    MIDI_IRQ_PIN_LOW
#endif
  }

  if ( stateWriteConfigToEEPROM )
  {
    disableISR = 1;
    playSID->stop();

    if ( stateWriteConfigToEEPROM == 1 )
    {
      writeConfigToEEPROM();
      doReset = 1;
      configUpdatedTemporarily = 0;
    } else if ( stateWriteConfigToEEPROM == 3 )
    {
      for (int i = MAX_SETTINGS * 10; i < MAX_SETTINGS * 11; i++)
        EEPROM.write( i, stateConfig[ i ] );
      doReset = 1;
      configUpdatedTemporarily = 0;
    }

    uint8_t activeProfile = stateConfig[ MAX_SETTINGS * 10 ];
    if ( activeProfile >= 10 )
      activeProfile = 0;

    // if profile or SID #1/#2 or FM or reSID version changed => do emulation reset
    playSID->updateConfiguration( &stateConfig[ activeProfile * MAX_SETTINGS ], &stateConfig[ 10 * MAX_SETTINGS ] );
    memcpy( stateConfigPrevious, stateConfig, MAX_SETTINGS * 11 );

    playSID->reset();

    doReset = 2;

    resetEmulation();

    #ifdef SUPPORT_MIDI
    MIDIControlReg = 3;
    MIDIStatusReg &= 127;
    releasedIRQ = 1;
    MIDI_IRQ_PIN_LOW
    #endif

    playSID->begin();
    stateWriteConfigToEEPROM = 0;
    disableISR = 0;
  }

  if ( stateInVisualizationMode )
  {
    extern bool driveLEDs;
    driveLEDs = true;
  }

#ifdef SID_WRITE_ONLY_MODE_SUPPORT
  if ( sidOnlyMode == SID_ONLY_ENABLE )
  {
    detachInterrupt( digitalPinToInterrupt( pinPHI2 ) );
    attachInterrupt( digitalPinToInterrupt( pinPHI2 ), isrSIDOnly, RISING );
    sidOnlyMode = SID_ONLY_ACTIVE;
    Serial.println( "SID-only mode");
  } else
  if ( sidOnlyMode == SID_ONLY_DISABLE )
  {
    detachInterrupt( digitalPinToInterrupt( pinPHI2 ) );
    attachInterrupt( digitalPinToInterrupt( pinPHI2 ), isrSID, RISING );
    sidOnlyMode = SID_ONLY_OFF;
    Serial.println( "normal mode");
  } 
#endif
  
#if ( audioDevice == 0 )
  mqs->poll_update();
#else
  audioPT8211->poll_update();
#endif
}

void setD07_Read()
{
  CORE_PIN19_DDRREG &= ~CORE_PIN19_BITMASK & ~CORE_PIN18_BITMASK & ~CORE_PIN14_BITMASK & ~CORE_PIN15_BITMASK & ~CORE_PIN17_BITMASK & ~CORE_PIN16_BITMASK & ~CORE_PIN22_BITMASK & ~CORE_PIN23_BITMASK;
}

void setD07_Write()
{
  CORE_PIN19_DDRREG |= CORE_PIN19_BITMASK | CORE_PIN18_BITMASK | CORE_PIN14_BITMASK | CORE_PIN15_BITMASK | CORE_PIN17_BITMASK | CORE_PIN16_BITMASK | CORE_PIN22_BITMASK | CORE_PIN23_BITMASK;
}

#define D_FLAGS (CORE_PIN19_BITMASK|CORE_PIN18_BITMASK|CORE_PIN14_BITMASK|CORE_PIN15_BITMASK|CORE_PIN17_BITMASK|CORE_PIN16_BITMASK|CORE_PIN22_BITMASK|CORE_PIN23_BITMASK)

__attribute__( ( always_inline ) ) inline void writeDataPins(uint8_t val)
{
  // enable LVC245
  CORE_PIN6_PORTCLEAR = CORE_PIN6_BITMASK;

  register uint32_t s =
    ( ( val & 128 ) >> 7 ) << CORE_PIN19_BIT |
    ( ( val &  64 ) >> 6 ) << CORE_PIN18_BIT |
    ( ( val &  32 ) >> 5 ) << CORE_PIN14_BIT |
    ( ( val &  16 ) >> 4 ) << CORE_PIN15_BIT |
    ( ( val &   8 ) >> 3 ) << CORE_PIN17_BIT |
    ( ( val &   4 ) >> 2 ) << CORE_PIN16_BIT |
    ( ( val &   2 ) >> 1 ) << CORE_PIN22_BIT |
    ( ( val &   1 ) >> 0 ) << CORE_PIN23_BIT;

  uint32_t c = ( ~s ) & D_FLAGS;

  CORE_PIN19_PORTSET = s;
  CORE_PIN19_PORTCLEAR = c;

#ifdef FIRMWARE_C128
  do {} while (ARM_DWT_CYCCNT < ( 180 * TEENSY_CLOCK / 600 ) );
#else
  do {} while (ARM_DWT_CYCCNT < ( 120 * TEENSY_CLOCK / 600 ) );
#endif
  CORE_PIN6_PORTSET = CORE_PIN6_BITMASK;
}

__attribute__( ( always_inline ) ) inline void omitReadDataPins()
{
  CORE_PIN6_PORTSET = CORE_PIN6_BITMASK;
  setD07_Write();
}

__attribute__( ( always_inline ) ) inline uint8_t readDataPins()
{
/*  
  // this setup of the level shifter has been moved into the ISR-handler. Not nice, but saves a few cycles...   
 
  setD07_Read();

  // some delay (maybe use CPU cycle counter instead)
  CORE_PIN6_PORTCLEAR = CORE_PIN6_BITMASK;
  CORE_PIN6_PORTCLEAR = CORE_PIN6_BITMASK;
  CORE_PIN6_PORTCLEAR = CORE_PIN6_BITMASK;
  CORE_PIN6_PORTCLEAR = CORE_PIN6_BITMASK;
  CORE_PIN6_PORTCLEAR = CORE_PIN6_BITMASK;

  if ( TEENSY_CLOCK > 720 )
    CORE_PIN6_PORTCLEAR = CORE_PIN6_BITMASK;
  if ( TEENSY_CLOCK > 600 )
    CORE_PIN6_PORTCLEAR = CORE_PIN6_BITMASK;*/

  #ifdef FIRMWARE_C128
  // this delay does not seem to be necessary
  // const uint32_t waitC128 = 80 * TEENSY_CLOCK / 600;
  // do { } while (ARM_DWT_CYCCNT < waitC128 );
  #endif

  #define IMXRT_GPIO6_DIRECT (*(volatile uint32_t *)0x42000000)
  register uint32_t data = IMXRT_GPIO6_DIRECT;

  // disable reading/writing D0...D7
  CORE_PIN6_PORTSET = CORE_PIN6_BITMASK;

  setD07_Write();

  register uint8_t D =
    ( ( data & CORE_PIN19_BITMASK ) >> ( CORE_PIN19_BIT - 7 ) ) |
    ( ( data & CORE_PIN18_BITMASK ) >> ( CORE_PIN18_BIT - 6 ) ) |
    ( ( data & CORE_PIN14_BITMASK ) >> ( CORE_PIN14_BIT - 5 ) ) |
    ( ( data & CORE_PIN15_BITMASK ) >> ( CORE_PIN15_BIT - 4 ) ) |
    ( ( data & CORE_PIN17_BITMASK ) >> ( CORE_PIN17_BIT - 3 ) ) |
    ( ( data & CORE_PIN16_BITMASK ) >> ( CORE_PIN16_BIT - 2 ) ) |
    ( ( data & CORE_PIN22_BITMASK ) >> ( CORE_PIN22_BIT - 1 ) ) |
    ( ( data & CORE_PIN23_BITMASK ) >> ( CORE_PIN23_BIT - 0 ) );

  return D;
}



volatile uint8_t forceRead __attribute__ ((aligned (64)));


FASTRUN void isrDetermineClockFrequency()
{
  static uint32_t cyclesSinceLastCallAcc = 0, cyclesSinceLastCall = 816;
  static uint32_t accCyclesSinceLastCall = 0;

  cyclesSinceLastCallAcc += ARM_DWT_CYCCNT;

  CPU_RESET_CYCLECOUNTER;

  if ( ++ accCyclesSinceLastCall >= 64 )
  {
    cyclesSinceLastCall = cyclesSinceLastCallAcc / 64 + 16;
    accCyclesSinceLastCall = 0;
    cyclesSinceLastCallAcc = 0;

    // cyculesSinceLastCall: ~828 cycles for PAL, approx. 797 or 798 for NTSC for TEENSY_CLOCK 816
    if ( cyclesSinceLastCall > ( 813 * TEENSY_CLOCK ) / 816 ) // PAL system
      runningOnPAL = 1; else
      runningOnPAL = 0;
  }
  c64CycleCount ++;
}

#ifdef SID_WRITE_ONLY_MODE_SUPPORT
FASTRUN void isrSIDOnly()
{
  if ( ARM_DWT_CYCCNT > 100000 )
    sidOnlyMode = SID_ONLY_DISABLE;
  
  // for accurate timing
  CPU_RESET_CYCLECOUNTER;

  c64CycleCount ++;

  if ( disableISR )
    return;

  register uint32_t data_7, data_9;
  register uint32_t signals, sid2;
  register uint32_t data, A = 0;

  #ifdef C128_D500_SUPPORT
  uint8_t sid2_d500 = 0;
  #endif

  // C128 needs a delay
  #ifdef FIRMWARE_C128
  // shorter delays result in invalid RW line
  do { } while (ARM_DWT_CYCCNT < 20 * TEENSY_CLOCK / 600 );
  #endif

  // GPIO Bank #9: Phi2, CS, ...
  #define IMXRT_GPIO9_DIRECT (*(volatile uint32_t *)0x4200C000)
  data_9 = IMXRT_GPIO9_DIRECT;

  #ifdef FIRMWARE_C128
  if ( !( data_9 & CORE_PIN2_BITMASK ) ) // CPU writes to bus
  {
    do { } while (ARM_DWT_CYCCNT < 60 * TEENSY_CLOCK / 600 );
    data_9 = IMXRT_GPIO9_DIRECT;
  }
  #endif

  // read  A5..8, IO1, IO2, RESET
  // this must not happen earlier, at least on a C128 (the above delay 15 or 20 + read data_9 is sufficient)
  #define IMXRT_GPIO7_DIRECT (*(volatile uint32_t *)0x42004000)
  data_7 = IMXRT_GPIO7_DIRECT;

  #ifdef C128_D500_SUPPORT
  register uint32_t temp_7;
  temp_7 = data_7;
  data_7 &= ~CORE_PIN37_BITMASK;
  #endif
  
  //
  // is (any) SID selected?
  //
  extern uint32_t SID2_ADDR;
  signals = ( ( data_7 ^ DUAL_SID_FLIPMASK ) & DUAL_SID_FLIPMASK ) | ( data_7 & DUAL_SID_ADDR_MASK );
  sid2 = (signals & SID2_ADDR);

  #ifdef C128_D500_SUPPORT
  if ( ( SID2_ADDR & CORE_PIN37_BITMASK ) &&  // D500?
       !( temp_7 & CORE_PIN36_BITMASK ) &&    // MMU Pin 47 (C64-Mode = low)
       !( temp_7 & CORE_PIN37_BITMASK ) &&    // A8 (read at U3 pin 14)
       !( data_9 & CORE_PIN2_BITMASK ) &&
       c64CycleCount > 500000 )
  {
    sid2_d500 = sid2 = 1;
    extern uint32_t SID2_MODEL;
    if ( SID2_MODEL == 0 )
    {
      sid2 = 0;
      #ifdef C128_D500_SUPPORT
      sid2_d500 = 0;
      #endif
    }
  }
  data_7 = temp_7;
  #endif

  uint8_t tasksToDo = 0;
  if ( !( data_9 & CORE_PIN3_BITMASK ) // Chip Select
    #ifdef C128_D500_SUPPORT
       || sid2_d500
    #endif
       || ( (SID2_ADDR & DUAL_SID_FLIPMASK) && sid2 ) )
    tasksToDo |= 1;

  if ( ( tasksToDo & ( 1 + 4 + 8 ) ) && !( data_9 & CORE_PIN2_BITMASK ) ) 
  {
    setD07_Read();
    CORE_PIN6_PORTCLEAR = CORE_PIN6_BITMASK;
  }

  /*
          ___  __
    |    |__  |  \
    |___ |___ |__/

  */
  #ifdef FANCY_LED

  // 0 = do nothing
  // 1 = writing '0', need to do something before we quit the FIQ-handler
  // 2 = started writing a '1', need to switch low next cycle
  // 3 = started writing the interbit gap (T0L, T1L), can do more work next time
  // 4 + x = wait for x more cycles (reset signal)
  // 5 = wait to send reset signal
  // -1 = do nothing
  extern bool driveLEDs;
  static uint8_t ledUpdate = 0;
  if ( driveLEDs && tasksToDo == 0 && ( (++ledUpdate & 3) == 0 || ledWriteStatus == 1 ) )
  {
    switch (ledWriteStatus)
    {
      case 0:
        {
          if ( ledBits2Write || ledColors2Write )
          {
            static uint32_t currentLEDColor;
            if ( ledBits2Write == 0 && ledColors2Write ) // next led
            {
              currentLEDColor = ledWriteRGB[ ( (NUM_LEDS - ledColors2Write) + ledColorsBufOfs ) % NUM_LEDS ];
              ledColors2Write --;
              ledBits2Write = 24;
            }

            // transmit next bit
            register uint8_t bit = ( currentLEDColor >> 23 ) & 1;
            currentLEDColor <<= 1;
            ledBits2Write --;
            if ( bit && !ledPinStatus )
              CORE_PIN31_PORTSET = CORE_PIN31_BITMASK;
            ledPinStatus = bit;

            ledWriteStatus = bit + 1;
          } else
          {
            // transmit reset, this delay (must be >5) is essential for the update-rate of the LED-animation
            ledWriteStatus = 5 + 200 * 4;
            if ( ledPinStatus )
              CORE_PIN31_PORTCLEAR = CORE_PIN31_BITMASK;
            ledPinStatus = 0;
          }
          break;
        }

      case 1:
        if ( !ledPinStatus )
          CORE_PIN31_PORTSET = CORE_PIN31_BITMASK;
        ledPinStatus = 1;
        ledWriteStatus = 4;
        break;
      
      case 2:
        if ( ledPinStatus )
          CORE_PIN31_PORTCLEAR = CORE_PIN31_BITMASK;
        ledPinStatus = 0;
        ledWriteStatus = 0;
        break;

      case 5:
        ledWriteStatus = -1;

      default:
        if ( ledWriteStatus >= 6 )
          ledWriteStatus --;
        break;
    }
  }
#endif // FANCY_LED

  // read signals:  A0..4 (D0..7 are only on this port, but the level shifter is not set to read here)
  if ( tasksToDo & ~2 )
  {
    #define IMXRT_GPIO6_DIRECT (*(volatile uint32_t *)0x42000000)
    data = IMXRT_GPIO6_DIRECT;

    A = ( ( data & ( 1 << CORE_PIN0_BIT ) ) >> CORE_PIN0_BIT ) |
        ( ( data & ( 3 << CORE_PIN24_BIT ) ) >> ( CORE_PIN24_BIT - 1 ) ) |
        ( ( data & ( 3 << CORE_PIN26_BIT ) ) >> ( CORE_PIN26_BIT - 3 ) );
  }

  if ( ( tasksToDo & 127 ) == 0 )
    goto noSID_FM_MIDI_Commands_2;

  /*
     __     __      __        __                     __               __
    /__` | |  \    |__) |  | /__`    |__|  /\  |\ | |  \ |    | |\ | / _`
    .__/ | |__/    |__) \__/ .__/    |  | /~~\ | \| |__/ |___ | | \| \__>

  */
  if ( tasksToDo & 1 )
  {
    if ( ( data_9 & CORE_PIN2_BITMASK ) ) // RW -> READ
    {
      //
      // CPU reads SID register
      //
    } else
    {
      //
      // CPU writes to SID
      //
      register uint8_t D = readDataPins();

      A &= 0x1f;

      if ( A <= 24 )
      {
        // pseudo stereo?
        if ( SID2_ADDR == (uint32_t)(1 << 31) )
          // yes, flag command for both SIDs
          ringBufGPIO[ ringWrite ] = D | ( A << 8 ) | ( 1 << 21 ); else
          // command is only for one of the SIDs
          ringBufGPIO[ ringWrite ] = D | ( A << 8 ) | ( sid2 ? 1 << 20 : 0 );
        ringTime[ ringWrite ] = c64CycleCount + nSIDFMDelay;
        ringWrite++;
        ringWrite &= ( RING_SIZE - 1 );
      }
    }
  }

noSID_FM_MIDI_Commands_2:
  /*
     __   ___  __   ___ ___
    |__) |__  /__` |__   |
    |  \ |___ .__/ |___  |

  */
  if ( !(data_7 & CORE_PIN32_BITMASK) ) // reset
  {
    sidOnlyMode = SID_ONLY_DISABLE;
    #ifdef SID_DAC_MODE_SUPPORT
    sidDACMode = SID_DAC_OFF;
    #endif
    if ( doReset == 0 ) doReset = 1;
  } else 
  if ( doReset == 2 )
  {
    doReset = 0;
    c64CycleCountResetReleased = c64CycleCount;
  }

  /*
          ___  __
    |    |__  |  \
    |___ |___ |__/

  */
#ifdef FANCY_LED
  if ( ledWriteStatus == 4 )
  {
    if ( ledPinStatus )
    {
      do {} while ( ARM_DWT_CYCCNT < ( LED_HIGH_DURATION * TEENSY_CLOCK / 600 ) );
      CORE_PIN31_PORTCLEAR = CORE_PIN31_BITMASK;
    }
    ledPinStatus = 0;
    ledWriteStatus = 0;
  }
#endif
}
#endif


FASTRUN void isrSID()
{
  // for accurate timing
  CPU_RESET_CYCLECOUNTER;

  c64CycleCount ++;


  if ( disableISR )
    return;

  register uint32_t data_7, data_9;
  register uint32_t signals, sid2;
  register uint32_t data, A = 0;

  #ifdef C128_D500_SUPPORT
  uint8_t sid2_d500 = 0;
  #endif

  // C128 needs a delay
  #ifdef FIRMWARE_C128
  // shorter delays result in invalid RW line
  do { } while (ARM_DWT_CYCCNT < 20 * TEENSY_CLOCK / 600 );
  #endif

  // GPIO Bank #9: Phi2, CS, ...
  #define IMXRT_GPIO9_DIRECT (*(volatile uint32_t *)0x4200C000)
  data_9 = IMXRT_GPIO9_DIRECT;

  #ifdef FIRMWARE_C128
  if ( !( data_9 & CORE_PIN2_BITMASK ) ) // CPU writes to bus
  {
    do { } while (ARM_DWT_CYCCNT < 60 * TEENSY_CLOCK / 600 );
    data_9 = IMXRT_GPIO9_DIRECT;
  }
  #endif

  // read  A5..8, IO1, IO2, RESET
  // this must not happen earlier, at least on a C128 (the above delay 15 or 20 + read data_9 is sufficient)
  #define IMXRT_GPIO7_DIRECT (*(volatile uint32_t *)0x42004000)
  data_7 = IMXRT_GPIO7_DIRECT;

  #ifdef C128_D500_SUPPORT
  register uint32_t temp_7;
  temp_7 = data_7;
  data_7 &= ~CORE_PIN37_BITMASK;
  #endif
  
  //
  // is (any) SID selected?
  //
  extern uint32_t SID2_ADDR;
  extern uint32_t SID2_MODEL;
  signals = ( ( data_7 ^ DUAL_SID_FLIPMASK ) & DUAL_SID_FLIPMASK ) | ( data_7 & DUAL_SID_ADDR_MASK );
  sid2 = (signals & SID2_ADDR);

  #ifdef C128_D500_SUPPORT
  if ( ( SID2_ADDR & CORE_PIN37_BITMASK ) &&  // D500?
       !( temp_7 & CORE_PIN36_BITMASK ) &&    // MMU Pin 47 (C64-Mode = low)
       !( temp_7 & CORE_PIN37_BITMASK ) &&    // A8 (read at U3 pin 14)
       !( data_9 & CORE_PIN2_BITMASK ) &&
       c64CycleCount > 500000 )
  {
    sid2_d500 = sid2 = 1;
    if ( SID2_MODEL == 0 )
    {
      sid2 = 0;
      #ifdef C128_D500_SUPPORT
      sid2_d500 = 0;
      #endif
    }
  }
  data_7 = temp_7;
  #endif

  static int32_t potCycleCounter = 0;
  extern bool emulateFM, readRegistersFM;
  #ifdef SUPPORT_MIDI
  extern bool emulateMIDI;
  #endif


  uint8_t tasksToDo = 0;
  if ( !( data_9 & CORE_PIN3_BITMASK ) // Chip Select
    #ifdef C128_D500_SUPPORT
       || sid2_d500
    #endif
       || ( (SID2_ADDR & DUAL_SID_FLIPMASK) && sid2 ) )
    tasksToDo |= 1;

  if ( !stateInConfigMode && !stateInTransferMode )
  {
    if ( potCycleCounter == 0 || potCycleCounter == 256 )
      tasksToDo |= 2;

    if ( emulateFM && !( data_7 & CORE_PIN13_BITMASK ) ) // IO2
      tasksToDo |= 4;

    #ifdef SUPPORT_MIDI
    if ( emulateMIDI && !( data_7 & CORE_PIN11_BITMASK ) ) // IO1 access
      tasksToDo |= 8;
    #endif

    if ( stateInVisualizationMode )
    {
      #ifdef FIRMWARE_C128
      if ( !( temp_7 & CORE_PIN36_BITMASK ) &&    // MMU Pin 47 (C64-Mode = low)
           !( temp_7 & CORE_PIN37_BITMASK ) &&    // A8 (read at U3 pin 14)
           !( data_9 & CORE_PIN2_BITMASK ) )      // RW
           addrLine |= 8;
      #endif
      
      if ( !( data_9 & CORE_PIN3_BITMASK ) ) // Chip Select
      {
        if ( ( data_7 & CORE_PIN34_BITMASK ) ) addrLine |= 1;
        if ( ( data_7 & CORE_PIN35_BITMASK ) ) addrLine |= 2;
        if ( ( data_7 & CORE_PIN36_BITMASK ) ) addrLine |= 4;
        #ifndef FIRMWARE_C128
        if ( ( data_7 & CORE_PIN37_BITMASK ) ) addrLine |= 8;
        #endif
      }
      if ( !( data_7 & CORE_PIN11_BITMASK ) ) // IO1 access -> set flag 
        addrLine |= 0x10;
      if ( !( data_7 & CORE_PIN13_BITMASK ) ) // IO2 access -> set flag 
        addrLine |= 0x20;
    }
  } else
  {
    tasksToDo = ( tasksToDo & 1 ) | 0x80;
    #ifdef FANCY_LED
    ledWriteStatus = 0;
    #endif
  }

  if ( ( tasksToDo & ( 1 + 4 + 8 ) ) && !( data_9 & CORE_PIN2_BITMASK ) ) 
  {
    setD07_Read();
    CORE_PIN6_PORTCLEAR = CORE_PIN6_BITMASK;
  }

  /*
          ___  __
    |    |__  |  \
    |___ |___ |__/

  */
  #ifdef FANCY_LED

  // 0 = do nothing
  // 1 = writing '0', need to do something before we quit the FIQ-handler
  // 2 = started writing a '1', need to switch low next cycle
  // 3 = started writing the interbit gap (T0L, T1L), can do more work next time
  // 4 + x = wait for x more cycles (reset signal)
  // 5 = wait to send reset signal
  // -1 = do nothing
  extern bool driveLEDs;
  static uint8_t ledUpdate = 0;
  if ( driveLEDs && tasksToDo == 0 && ( (++ledUpdate & 3) == 0 || ledWriteStatus == 1 ) )
  {
    switch (ledWriteStatus)
    {
      case 0:
        {
          if ( ledBits2Write || ledColors2Write )
          {
            static uint32_t currentLEDColor;
            if ( ledBits2Write == 0 && ledColors2Write ) // next led
            {
              currentLEDColor = ledWriteRGB[ ( (NUM_LEDS - ledColors2Write) + ledColorsBufOfs ) % NUM_LEDS ];
              ledColors2Write --;
              ledBits2Write = 24;
            }

            // transmit next bit
            register uint8_t bit = ( currentLEDColor >> 23 ) & 1;
            currentLEDColor <<= 1;
            ledBits2Write --;
            if ( bit && !ledPinStatus )
              CORE_PIN31_PORTSET = CORE_PIN31_BITMASK;
            ledPinStatus = bit;

            ledWriteStatus = bit + 1;
          } else
          {
            // transmit reset, this delay (must be >5) is essential for the update-rate of the LED-animation
            ledWriteStatus = 5 + 200 * 4;
            if ( ledPinStatus )
              CORE_PIN31_PORTCLEAR = CORE_PIN31_BITMASK;
            ledPinStatus = 0;
          }
          break;
        }

      case 1:
        if ( !ledPinStatus )
          CORE_PIN31_PORTSET = CORE_PIN31_BITMASK;
        ledPinStatus = 1;
        ledWriteStatus = 4;
        break;
      
      case 2:
        if ( ledPinStatus )
          CORE_PIN31_PORTCLEAR = CORE_PIN31_BITMASK;
        ledPinStatus = 0;
        ledWriteStatus = 0;
        break;

      case 5:
        ledWriteStatus = -1;

      default:
        if ( ledWriteStatus >= 6 )
          ledWriteStatus --;
        break;
    }
  }
#endif // FANCY_LED

  // read signals:  A0..4 (D0..7 are only on this port, but the level shifter is not set to read here)
  if ( tasksToDo & ~2 )
  {
    #define IMXRT_GPIO6_DIRECT (*(volatile uint32_t *)0x42000000)
    data = IMXRT_GPIO6_DIRECT;

    A = ( ( data & ( 1 << CORE_PIN0_BIT ) ) >> CORE_PIN0_BIT ) |
        ( ( data & ( 3 << CORE_PIN24_BIT ) ) >> ( CORE_PIN24_BIT - 1 ) ) |
        ( ( data & ( 3 << CORE_PIN26_BIT ) ) >> ( CORE_PIN26_BIT - 3 ) );
  }

  static uint8_t busValue = 0;
  static int busValueTTL = 0;

  if ( busValueTTL <= 0 )
  {
    busValue = 0;
  } else
    busValueTTL --;

  if ( ( tasksToDo & 127 ) == 0 )
  {
    goto noSID_FM_MIDI_Commands;
  }

  /*
     __     __      __        __                     __               __
    /__` | |  \    |__) |  | /__`    |__|  /\  |\ | |  \ |    | |\ | / _`
    .__/ | |__/    |__) \__/ .__/    |  | /~~\ | \| |__/ |___ | | \| \__>

  */
  if ( tasksToDo & 1 )
  {
    if ( ( data_9 & CORE_PIN2_BITMASK ) ) // RW -> READ
    {
      //
      // CPU reads SID register
      //
      // this mirrors the 0x1D to 0x1F registers to launch the config tool with SYS54301, SYS54333, ...
      if ( (A & 0x1F) >= 0x1D && (A & 0x1F) <= 0x1F )
      {
        sid2 = 0; A &= 0x1F;
      }

      if ( sid2 == 0 )
      {
        if ( ( stateInTransferMode || stateGoingTowardsTransferMode == 3 ) && A < 10 )
        {
          switch ( A )
          {
            case 0: // start transfer
              writeDataPins( 0x78 ); // execute SEI, previous: NOP 0xea
              transferStage = 0;
              launcherAddress = transferAddress = ( launchCode[ 1 ] << 8 ) + launchCode[ 0 ];
              transferData = (uint8_t *)&launchCode[ 2 ];
              transferDataEnd = (uint8_t *)&launchCode[ launchSize ];
              jumpAddress = 0xD401;

              forceRead = *transferData;
              CACHE_PRELOAD( transferData )
              break;

            case 1:
              writeDataPins( 0xA9 ); // execute LDA
              forceRead = *transferData;
              break;

            case 2:
              writeDataPins( forceRead ); // LDA value
              transferData ++;
              forceRead = *transferData;
              break;

            case 3:
              writeDataPins( 0x8D ); // execute STA
              forceRead = *transferData;
              break;

            case 4:
              writeDataPins( transferAddress & 255 ); // STA lowbyte address
              forceRead = *transferData;
              break;

            case 5:
              writeDataPins( transferAddress >> 8 ); // STA highbyte address
              transferAddress ++;
              break;

            case 6:
              writeDataPins( 0x4C ); // JMP
              if ( transferData >= transferDataEnd && transferStage == 0 )
              {
                transferStage = 1;
                transferAddress = ( prgCode[ 1 ] << 8 ) + prgCode[ 0 ];
                transferData = (uint8_t *)&prgCode[ 2 ];
                transferDataEnd = (uint8_t *)&prgCode[ prgSize ];
                forceRead = *transferData;
              }
              if ( transferData >= transferDataEnd && transferStage == 1 )
              {
                jumpAddress = launcherAddress;
              }
              break;

            case 7:
              writeDataPins( jumpAddress & 255 ); // JMP address low byte
              break;

            case 8:
              writeDataPins( jumpAddress >> 8 ); // JMP address high byte
              if ( stateInTransferMode && jumpAddress == launcherAddress )
                stateInTransferMode = 0;
              break;
          }
          stateInTransferMode = TRANSFER_MODE_CYCLES;
          stateGoingTowardsTransferMode = 0;
          
        } /* if ( stateInTransferMode && A < 9 ) */ else 
        if ( A == 0x1d && stateInVisualizationMode )
        {
          #ifdef FANCY_LED
          writeDataPins( nLEDsConfigTool );
          #else
          writeDataPins( 0 );
          #endif
        } else if ( A == 0x1e && stateInVisualizationMode )
        {
          // firmware features
          register uint8_t F = 0;
          #ifdef EMULATE_OPL2
          F |= 1;
          #endif
          #ifdef SUPPORT_MIDI
          F |= 2;
          #endif
          #ifdef FANCY_LED
          F |= 4;
          #endif
          writeDataPins( F );
        } else if ( A == 0x1c && stateInVisualizationMode )
        {
          // address line flags
          writeDataPins( addrLine );
        } else 

        if ( A == 0x1d && stateInConfigMode == 0 && c64CycleCount > 1000000 )
        {
          writeDataPins( 0x4C ); // JMP
          prgSize = cfgPRGCode_size;
          prgCode = cfgPRGCode;
          stateGoingTowardsTransferMode = 1;
        } else if ( A == 0x1e && stateInConfigMode == 0 && stateGoingTowardsTransferMode == 1 )
        {
          writeDataPins( 0x00 );
          stateGoingTowardsTransferMode = 2;
        } else if ( A == 0x1f && stateInConfigMode == 0 && stateGoingTowardsTransferMode == 2 )
        {
          writeDataPins( 0xD4 );
          stateGoingTowardsTransferMode = 3;
        } else 
        if ( A == 0x1d && stateInConfigMode > 0 )
        {
          if ( stateConfigRegisterAccess < 65536 )
            writeDataPins( stateConfig[ stateConfigRegisterAccess ++ ] ); else 
          if ( stateConfigRegisterAccess < 65536 + 16 + VERSION_STR_EXT_SIZE )
            writeDataPins( VERSION_STR[ stateConfigRegisterAccess - 65536 ] );
          stateInConfigMode = CONFIG_MODE_CYCLES;
        } else if ( A == 0x1c && stateReadDirectoryMode >= 0x1000 )
        {
          if ( stateReadDirectoryMode < 0x1000 + 36 * 24 + 1 )
          {
            writeDataPins( *transferData );
            transferData ++;
            forceRead = *transferData;
            stateReadDirectoryMode ++;
          } else
          {
            stateReadDirectoryMode = 0;
          }
          stateInConfigMode = CONFIG_MODE_CYCLES;
        } else 
        {
          if ( registerRead )
          {
            if ( sidAutoDetectStep == 1 && A == 0x1b )
            {
              sidAutoDetectStep = 0;
              extern uint32_t SID1_MODEL;
              if ( SID1_MODEL == 8580 )
                writeDataPins( 2 ); else
                writeDataPins( 3 );
            } else
            {
              if ( A >= 0x19 && A <= 0x1c )
                writeDataPins( outRegisters[ A & 31 ] ); else
                writeDataPins( busValue );
            }
          }
        }
      } else
      {
        if ( registerRead )
        {
          if ( sidAutoDetectStep_2 == 1 && A == 0x1b )
          {
            sidAutoDetectStep_2 = 0;
            if ( SID2_MODEL == 8580 )
              writeDataPins( 2 ); else
              writeDataPins( 3 );
          } else
          {
            if ( A >= 0x19 && A <= 0x1c )
              writeDataPins( outRegisters_2[ A & 31 ] ); else
              writeDataPins( busValue );
          }
        }
      }
    } else
    {
      //
      // CPU writes to SID
      //
      register uint8_t D = readDataPins();

      if ( sid2 == 0 )
      {
        if ( sidAutoDetectStep == 0 &&
             sidAutoDetectRegs[ 0x12 ] == 0xff &&
             sidAutoDetectRegs[ 0x0e ] == 0xff &&
             sidAutoDetectRegs[ 0x0f ] == 0xff &&
             A == 0x12 && D == 0x20 )
        {
          sidAutoDetectStep = 1;
        }
        sidAutoDetectRegs[ A & 31 ] = D;

        if ( A == 0x1f )
        {
          if ( D == 0xfe )
          {
            stateInVisualizationMode = CONFIG_MODE_CYCLES;
            addrLine = 0;
          } else
          if ( D == 0xff )
          {
            stateInConfigMode = CONFIG_MODE_CYCLES; // SID remains in config mode for 1/40sec
          } 
#ifdef SID_WRITE_ONLY_MODE_SUPPORT
          else if ( D == 0xfd )
          {
            sidOnlyMode = SID_ONLY_ENABLE;
            stateInConfigMode = 
            stateInTransferMode =
            stateInVisualizationMode = 0;
          }
#endif
#ifdef SID_DAC_MODE_SUPPORT
          else if ( D == 0xfc )
          {
            sidDACMode = 1;
            stateInConfigMode = 
            stateInTransferMode =
            stateInVisualizationMode = 0;
          }
#endif
        
          goto noSID_FM_MIDI_Commands;
        } else if ( stateInConfigMode > 0 )
        {
          if ( A == 0x1e )
          {
            if ( D >= 224 )
              stateConfigRegisterAccess = 65536 - 224 + D; else
              stateConfigRegisterAccess = D * 64;
            stateInConfigMode = CONFIG_MODE_CYCLES;
            goto noSID_FM_MIDI_Commands;
          } else if ( A == 0x1d )
          {
            if ( D == 0xff )
            {
              stateWriteConfigToEEPROM = 1;
            } else if ( D == 0xfe )
            {
              configUpdatedTemporarily = 1;
              stateWriteConfigToEEPROM = 2; // update settings, but DO NOT WRITE to EEPROM
            } else if ( D == 0xfd )
            {
              configUpdatedTemporarily = 1;
              stateWriteConfigToEEPROM = 3; // only write current profile to EEPROM
            } else
            {
              stateConfig[ stateConfigRegisterAccess ++ ] = D;
              stateInConfigMode = CONFIG_MODE_CYCLES;
            }
            goto noSID_FM_MIDI_Commands;
          } else if ( A == 0x1c )
          {
            // launch PRG!
            if ( D == 0xff ) // read directory
            {
              stateReadDirectoryMode = 0x1000;
              stateInVisualizationMode =
              stateInConfigMode = 0;
              transferData = &prgDirectory[ 0 ];
              __builtin_prefetch( &prgDirectory[ 0 ] );
              forceRead = *transferData;
            } else
            {
              const uint8_t *dirEntry = &prgDirectory[ D * 24 ];
              uint32_t ofs = ( dirEntry[ 19 ] * 256 + dirEntry[ 20 ] ) * 256 + dirEntry[ 21 ];
              prgSize = dirEntry[ 22 ] * 256 + dirEntry[ 23 ];
              prgCode = &prgRepository[ ofs ];

              launchCode[ 12 + 70 ] = ( 0x0801 + prgSize + 2 ) & 255;
              launchCode[ 14 + 70 ] = ( 0x0801 + prgSize + 2 ) >> 8;

              __builtin_prefetch( &prgCode[ 0 ] );
              __builtin_prefetch( &launchCode[ 0 ] );

              stateInConfigMode = 0;
              stateGoingTowardsTransferMode = 3;
              stateInTransferMode = TRANSFER_MODE_CYCLES;
            }
            goto noSID_FM_MIDI_Commands;
          }
        }
      } else
      {
        // SID #2
        if ( sidAutoDetectStep_2 == 0 &&
             sidAutoDetectRegs_2[ 0x12 ] == 0xff &&
             sidAutoDetectRegs_2[ 0x0e ] == 0xff &&
             sidAutoDetectRegs_2[ 0x0f ] == 0xff &&
             A == 0x12 && D == 0x20 )
        {
          sidAutoDetectStep_2 = 1;
        }
        sidAutoDetectRegs_2[ A & 31 ] = D;
      }

      A &= 0x1f;

      extern uint32_t SID1_MODEL;
      busValue = D;
      if ( SID1_MODEL == 8580 )
        busValueTTL = 0xa2000; else // 8580
        busValueTTL = 0x1d00; // 6581

      if ( A <= 24 )
      {
        // pseudo stereo?
        if ( SID2_ADDR == (uint32_t)(1 << 31) )
          // yes, flag command for both SIDs
          ringBufGPIO[ ringWrite ] = D | ( A << 8 ) | ( 1 << 21 ); else
          // command is only for one of the SIDs
          ringBufGPIO[ ringWrite ] = D | ( A << 8 ) | ( sid2 ? 1 << 20 : 0 );
        ringTime[ ringWrite ] = c64CycleCount + nSIDFMDelay;
        ringWrite++;
        ringWrite &= ( RING_SIZE - 1 );
      }
    }
    goto handleLED_and_quit_FIQ;
  }

  /*
     ___           __   __             __      ___      __             __   ___  __
    |__   |\/|    /__` /  \ |  | |\ | |  \    |__  \_/ |__)  /\  |\ | |  \ |__  |__)
    |     |  |    .__/ \__/ \__/ | \| |__/    |___ / \ |    /~~\ | \| |__/ |___ |  \

  */
  if ( tasksToDo & 4 )
  {
    if ( !( data_9 & CORE_PIN2_BITMASK ) ) // RW -> Write
    {
      register uint8_t D = readDataPins();

      if ( ( data_7 & CORE_PIN34_BITMASK ) ) A |= 1 << 5;
      if ( ( data_7 & CORE_PIN35_BITMASK ) ) A |= 1 << 6;
      //if ( ( data_7 & CORE_PIN36_BITMASK ) ) A |= 1 << 7;
      //if ( ( data_7 & CORE_PIN37_BITMASK ) ) A |= 1 << 8;

      if ( A == 0 && D == 0x04 )
        fmAutoDetectStep = 1;
      if ( A > 0 && D == 0x60 && fmAutoDetectStep == 1 )
        fmAutoDetectStep = 2;
      if ( A == 0 && D == 0x04 && fmAutoDetectStep == 2 )
        fmAutoDetectStep = 3;
      if ( A > 0 && D == 0x80 && fmAutoDetectStep == 3 )
      {
        fmAutoDetectStep = 4;
        fmFakeOutput = 0;
      }

      if ( A >= 0x40 && !( A & 15 ) )
      {
        ringBufGPIO[ ringWrite ] = D | ( A << 8 ) | ( 1 << 28 );
        ringTime[ ringWrite ] = c64CycleCount + nSIDFMDelay;
        ringWrite++;
        ringWrite &= ( RING_SIZE - 1 );
      }
      goto handleLED_and_quit_FIQ;
    } else if ( readRegistersFM ) // RW and IO2 already checked
    {
      // CPU reads from bus
      if ( ( data_7 & CORE_PIN34_BITMASK ) ) A |= 1 << 5;
      if ( ( data_7 & CORE_PIN35_BITMASK ) ) A |= 1 << 6;
      //if ( ( data_7 & CORE_PIN36_BITMASK ) ) A |= 1 << 7;
      //if ( ( data_7 & CORE_PIN37_BITMASK ) ) A |= 1 << 8;

      if ( (A & 0x60) == 0x60 )
      {
        // this is not a real read of the YM3812 status register!
        // only a fake that let's the detection routine be happy
        uint32_t D = fmFakeOutput;
        fmFakeOutput = 0xc0 - fmFakeOutput;
        writeDataPins( D );
        goto handleLED_and_quit_FIQ;
      } else if ( A >= 0x08 && A <= 0x0f )
      {
        // reading of the external Sound Expander Keyboard => we don't have it, return 0xFF
        writeDataPins( 0xff );
        goto handleLED_and_quit_FIQ;
      }
    }
  }



  /*
             __        __        __                     __               __
     |\/| | |  \ |    |__) |  | /__`    |__|  /\  |\ | |  \ |    | |\ | / _`
     |  | | |__/ |    |__) \__/ .__/    |  | /~~\ | \| |__/ |___ | | \| \__>

  */
#ifdef SUPPORT_MIDI

  if ( tasksToDo & 8 )
  {
    if ( !( data_9 & CORE_PIN2_BITMASK ) ) // RW -> Write
    {
      register uint8_t D = readDataPins(); 
      // write to IO1
      if ( A >= MIDI_START_ADDR && A <= MIDI_END_ADDR )
      {
        switch ( (A & 3) )
        {
          default:
            break;
          case MIDI_CONTROL_REG:
            MIDIControlReg = D;
            if ( D == 3 )
            {
              MIDIStatusReg = 0;
              activeMIDI_In = true;
            }

            if ( /*midiAutoDetectStep == 0 &&*/ D == 3 )
              midiAutoDetectStep = 1; else 
              if ( midiAutoDetectStep == 1 && D >= 0x15 )
                midiAutoDetectStep = 2; else
                midiAutoDetectStep = 0;

            if ( ( !( D & 0x80 ) ) // disable ACIA IRQ
                 || ( !releasedIRQ && IRQconditionTX )
               )
            {
              MIDI_IRQ_PIN_LOW
              releasedIRQ = 1;
              doNotTriggerIRQAgainThisCycle = 1;
            }
            break;
          case MIDI_TRANSMIT_REG:
            //D = readDataPins();
            //MIDIStatusReg &= 127;
            MIDIStatusReg |= 128;
            MIDI_ringBuf[ MIDI_ringWrite ] = ( D << 1 ) | ( 255 << 9 );
            MIDI_ringTime[ MIDI_ringWrite++ ] = c64CycleCount + nMIDIDelay;
            MIDI_ringWrite &= MIDI_RING_SIZE - 1;
            midiSerialWrite = -1;
            activeMIDI_Out = true;

            if ( !releasedIRQ && IRQconditionTX ) // disable IRQ when it was active because of TX
            {
              MIDI_IRQ_PIN_LOW
              releasedIRQ = 1;
              doNotTriggerIRQAgainThisCycle = 1;
            }
            break;
        }
      }
    } else
    {
      // reading IO1 range
      extern bool midiInputEnabled;
      if ( midiInputEnabled && A >= MIDI_START_ADDR && A <= MIDI_END_ADDR )
      {
        register uint8_t D = 0;
        // read from IO1
        if ( (A & 3) == MIDI_STATUS_REG )
        {
          if ( midiAutoDetectStep == 1 )
          {
            D = 0;
          } else
          {
            D = MIDIStatusReg;
            D |= (1 << 1);// | ( 1 << 7 ); // also turns on bit 7
            if ( !MIDIFetchReceivedByte )
              D |= (1 << 0);// | ( 1 << 7 ); // also turns on bit 7
          }
          writeDataPins( D );
        } else if ( (A & 3) == MIDI_RECEIVE_REG )
        {
          activeMIDI_In = true;

          if ( !MIDIFetchReceivedByte )
          {
            D = MIDINextReceivedByte;
            MIDIFetchReceivedByte = 1;
            writeDataPins( D );
            if ( !releasedIRQ && IRQconditionRX ) // disable IRQ when it was active because of RX
            {
              MIDI_IRQ_PIN_LOW
              releasedIRQ = 1;
              doNotTriggerIRQAgainThisCycle = 1;
              IRQconditionRX = 0;
            }
          }
        }
      } else
        writeDataPins( 0xff );
    }
  }

#endif


noSID_FM_MIDI_Commands:
  /*
     __   __  ___
    |__) /  \  |     \_/   \ /
    |    \__/  |     / \ .  |
                         '
  */
  static uint8_t newPotCounterX = 0;
  static uint8_t newPotCounterY = 0;

  if ( potCycleCounter == 0 )
  {
    //pinMode( potX, OUTPUT ); pinMode( potY, OUTPUT );
    CORE_PIN4_DDRREG |= CORE_PIN4_BITMASK | CORE_PIN5_BITMASK;
    CORE_PIN4_PORTCLEAR = CORE_PIN4_BITMASK | CORE_PIN5_BITMASK;
  } else if ( potCycleCounter == 256 )
  {
    //pinMode( potX, INPUT ); pinMode( potY, INPUT );
    CORE_PIN4_DDRREG &= ~CORE_PIN4_BITMASK & ~CORE_PIN5_BITMASK;
    newPotCounterX = newPotCounterY = 1;
  } else if ( potCycleCounter >= 256 )
  {
    if ( newPotCounterX && (data_9 & CORE_PIN4_BITMASK) )
    {
      outRegisters[ 25 ] = potCycleCounter - 256;
      newPotCounterX = 0;
    }
    if ( newPotCounterY && (data_9 & CORE_PIN5_BITMASK) )
    {
      outRegisters[ 26 ] = potCycleCounter - 256;
      newPotCounterY = 0;
    }
  }

  potCycleCounter ++;
  potCycleCounter &= 511;

  if ( stateInConfigMode > 0 )
    stateInConfigMode --;

  if ( stateInTransferMode > 0 )
    stateInTransferMode --;

  if ( stateInVisualizationMode )
    stateInVisualizationMode --;

handleLED_and_quit_FIQ:;

  /*
     __   ___  __   ___ ___
    |__) |__  /__` |__   |
    |  \ |___ .__/ |___  |

  */
  /*static uint8_t subsample = 0;
  subsample ++;
  subsample &= 63;*/
  
  if ( /*!subsample && */!(data_7 & CORE_PIN32_BITMASK) ) // reset
  {
#ifdef SID_WRITE_ONLY_MODE_SUPPORT
    if ( sidOnlyMode != SID_ONLY_OFF )
      sidOnlyMode = SID_ONLY_DISABLE;
#endif
    if ( doReset == 0 )
    {
      doReset = 1;
    }
  } else 
  if ( doReset == 2 )
  {
    doReset = 0;
    c64CycleCountResetReleased = c64CycleCount;
  }

  /*
           __
    |\/| | |  \ |
    |  | | |__/ |

  */
#ifdef SUPPORT_MIDI

  if ( emulateMIDI )
  {
    if ( !stateInConfigMode && !stateInTransferMode )
    if ( activeMIDI_In && releasedIRQ && MIDIFetchReceivedByte && MIDI_ringReadIn != MIDI_ringWriteIn && doNotTriggerIRQAgainThisCycle == 0 )
    {
      //MIDINextReceivedByteTime = MIDI_ringTimeIn[ MIDI_ringReadIn ];
      MIDINextReceivedByte = MIDI_ringBufIn[ MIDI_ringReadIn++ ];
      MIDI_ringReadIn &= MIDI_RING_SIZE - 1;
  
      MIDIFetchReceivedByte = 0;
      if ( MIDIControlReg & 0x80 ) // receiver interupt enable?
      {
        MIDI_IRQ_PIN_HIGH
        releasedIRQ = 0;
        IRQconditionRX = 1;
      }
    }
  
    if ( releasedIRQ && doNotTriggerIRQAgainThisCycle == 0 )
    {
      if ( ((MIDIControlReg >> 5) & 3 ) == 1 ) // CR5*~CR6
      {
        MIDI_IRQ_PIN_HIGH
        releasedIRQ = 0;
        IRQconditionTX = 1;
      }
    }
  
    if ( !releasedIRQ )
    {
      MIDIStatusReg |= 128;
    } else
    {
      if ( IRQconditionRX == 0 && ((MIDIControlReg >> 5) & 3 ) != 1 )
        MIDIStatusReg &= 127;
    }
  
    if ( ((MIDIControlReg >> 5) & 3 ) != 1 && ( MIDIControlReg & 0x80 ) == 0 )
      MIDIStatusReg &= 127;
  
  
    doNotTriggerIRQAgainThisCycle = 0;
  
    clock2midi_Counter += 65536;
  
    static uint8_t lastBit = 2;
    
    if ( activeMIDI_Out && clock2midi_Counter >= midiDelayCyclesFP16 )
    {
      clock2midi_Counter -= midiDelayCyclesFP16;
  
      if ( midiWriteBit <= 1024 / 2 )
      {
        if ( ( midiWrite & 1 ) != lastBit )
        {
          if ( midiWrite & 1 )
            CORE_PIN29_PORTSET = CORE_PIN29_BITMASK; else
            CORE_PIN29_PORTCLEAR = CORE_PIN29_BITMASK;
            lastBit = midiWrite & 1;
        }
        midiWrite >>= 1;
        midiWriteBit <<= 1;
      } else
      {
        CORE_PIN29_PORTSET = CORE_PIN29_BITMASK;
  
        if ( MIDI_ringRead != MIDI_ringWrite && nCyclesEmulated >= MIDI_ringTime[ MIDI_ringRead ] )
        {
          lastBit = 2;
          midiWrite = MIDI_ringBuf[ MIDI_ringRead ];
          MIDI_ringRead ++;
          MIDI_ringRead &= MIDI_RING_SIZE - 1;
          midiWriteBit = 1;
        }
      }
    }
  
    static uint8_t midiIn_Status = 1; // 1 = idle
    static uint32_t midiIn_Value = 0;
    static uint32_t midiIn_CurBit = 0;
    static uint32_t midiIn_DetectedStartBit = 0;
  
    static int32_t midiIn_Clock = 0;
  
    extern bool midiInputEnabled;
    if ( midiInputEnabled && !stateInConfigMode && !stateInTransferMode && activeMIDI_In )
    {
      #define IMXRT_GPIO8_DIRECT (*(volatile uint32_t *)0x42008000)
      register uint32_t data_8 = IMXRT_GPIO8_DIRECT;
  
      if ( midiIn_Status == 2 ) // receive 8 bits + stop bit
      {
        midiIn_Clock += 65536;
        if ( midiIn_Clock >= (int32_t)midiDelayCyclesFP16 )
        {
          uint8_t l = (data_8 >> CORE_PIN28_BIT) & 1;
          midiIn_Clock -= midiDelayCyclesFP16;
          if ( midiIn_CurBit < 8 ) // 9-th bit is the stop bit
          {
            midiIn_Value |= l << midiIn_CurBit;
          } else
          {
            if ( l == 1 )
            {
              midiIn_Status = 1;
              MIDI_ringBufIn[ MIDI_ringWriteIn++ ] = midiIn_Value;
              MIDI_ringWriteIn &= MIDI_RING_SIZE - 1;
            }
          }
          midiIn_CurBit ++;
        }
      } else
      {
  
        if ( ( data_8 & ( 1 << CORE_PIN28_BIT ) ) == 0 )
        {
          midiIn_DetectedStartBit ++;
  
          if ( midiIn_DetectedStartBit > midiIn_DetectedStartBitThreshold )
          {
            midiIn_DetectedStartBit = 0;
            midiIn_Value = 0;
            midiIn_CurBit = 0;
            midiIn_Clock = 0;
            midiIn_Status = 2; // 2 = receive
          }
        } else
          midiIn_DetectedStartBit = 0;
      }
    }
  }
#endif

  /*
     __       ___ ___  __
    |__) |  |  |   |  /  \ |\ |
    |__) \__/  |   |  \__/ | \|

  */
  if ( /*!subsample && */!stateInConfigMode && !stateInTransferMode )
  {
    static uint32_t buttonPressedCycle = 0, nCyclesPressed = 0, nCyclesReleased = 0, lastSAMMessage = 0;
    static uint8_t buttonState = 0;
    if ( !( data_9 & CORE_PIN33_BITMASK ) && debounceCycle < c64CycleCount ) // Button
    {
      if ( buttonState == 0 )
      {
        buttonPressedCycle = c64CycleCount;
        buttonState = 1;
      }
      nCyclesPressed ++;
      nCyclesReleased = 0;
    } else
    {
      nCyclesReleased ++;
      nCyclesPressed = 0;
    }
  
    if ( buttonState && nCyclesReleased > 1000 )
    {
      debounceCycle = c64CycleCount + 200000;
  
      uint8_t activeProfile = stateConfig[ MAX_SETTINGS * 10 ];
  
      if ( c64CycleCount - buttonPressedCycle > 300000 )
      {
        if ( activeProfile > 0 )
          activeProfile --; else
          activeProfile = 9;
      } else
      {
        activeProfile ++;
        if ( activeProfile >= 10 )
          activeProfile = 0;
      }
  
      stateConfig[ MAX_SETTINGS * 10 ] = activeProfile;
  
      playSID->updateConfiguration( &stateConfig[ activeProfile * MAX_SETTINGS ], &stateConfig[ 10 * MAX_SETTINGS ] );
      memcpy( stateConfigPrevious, stateConfig, MAX_SETTINGS * 11 );
  
      extern uint8_t samActive;
      if ( samActive )
      {
        speakSAM( activeProfile, c64CycleCount - lastSAMMessage < 2000000 ? true : false );
        lastSAMMessage = c64CycleCount;
      }
  
      buttonState = 0;
      nCyclesReleased = 0;
  
    }
  }


  /*
          ___  __
    |    |__  |  \
    |___ |___ |__/

  */
#ifdef FANCY_LED
  if ( ledWriteStatus == 4 )
  {
    if ( ledPinStatus )
    {
      do {} while ( ARM_DWT_CYCCNT < ( LED_HIGH_DURATION * TEENSY_CLOCK / 600 ) );
      CORE_PIN31_PORTCLEAR = CORE_PIN31_BITMASK;
    }
    ledPinStatus = 0;
    ledWriteStatus = 0;
  }
#endif
}
