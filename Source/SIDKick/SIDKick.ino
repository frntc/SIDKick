/*
   _________.___________   ____  __.__        __     
  /   _____/|   \______ \ |    |/ _|__| ____ |  | __ 
  \_____  \ |   ||    |  \|      < |  |/ ___\|  |/ / 
  /        \|   ||    `   \    |  \|  \  \___|    <  
 /_______  /|___/_______  /____|__ \__|\___  >__|_ \ 
         \/             \/        \/       \/     \/ 

  SIDKick.ino

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

#include <EEPROM.h>
#include <Audio.h>

#include "AudioStreamSID.h"
#include "imxrt.h"
#include "utility/imxrt_hw.h"

#include "launch.h"     // trampoline code for launching PRGs
#include "prgslots.h"   // memory for PRG-launcher

uint32_t prgSize = 0;
const uint8_t *prgCode = NULL;

#include "config.h"     // built-in config tool

// 0 = MQS, 1 = DAC
#define audioDevice 1

#define TEENSY_CLOCK 816

//#define FIRMWARE_C128

// C128 does not support $D500 for SID #2
// except for the C64-mode if:
// A8 is connected to U3 pin 14, and A7 to U7 (MMU, MOS 8722) Pin 47
#ifdef FIRMWARE_C128
#define C128_D500_SUPPORT
#endif


#ifdef FIRMWARE_C128
  #if audioDevice == 1
  static const char VERSION_STR[20] = {0x53, 0x49, 0x44, 0x4b, 0x09, 0x03, 0x0b, '0', '.', '1', 0x44, 0x41, 0x43, '1', '2', '8' };
  #else
  static const char VERSION_STR[20] = {0x53, 0x49, 0x44, 0x4b, 0x09, 0x03, 0x0b, '0', '.', '1', 0x4d, 0x51, 0x53, '1', '2', '8' };
  #endif
#else
  #if audioDevice == 1
  static const char VERSION_STR[20] = {0x53, 0x49, 0x44, 0x4b, 0x09, 0x03, 0x0b, '0', '.', '1', '/', 0x44, 0x41, 0x43, '6', '4' };
  #else
  static const char VERSION_STR[20] = {0x53, 0x49, 0x44, 0x4b, 0x09, 0x03, 0x0b, '0', '.', '1', '/', 0x4d, 0x51, 0x53, '6', '4' };
  #endif
#endif

uint8_t portPinsC[ 5 ] = {  0, 24, 25, 26, 27 };                // A0..4 (GPIOs on port 6)
uint8_t portPinsD[ 8 ] = { 23, 22, 16, 17, 15, 14, 18, 19 };    // D0..7 (GPIOs on port 6)
uint8_t portPinsA[ 7 ] = { 34, 35, 36, 37, 11, 13, 32 };        // A5..8, IO1, IO2, RESET (GPIOs on port 7)

#define S_A5   CORE_PIN34_BITMASK
#define S_A6   CORE_PIN35_BITMASK
#define S_A7   CORE_PIN36_BITMASK
#define S_A8   CORE_PIN37_BITMASK

#define DUAL_SID_MASK      (S_A5 | S_A8 | CORE_PIN11_BITMASK | CORE_PIN13_BITMASK )
#define DUAL_SID_ADDR_MASK (S_A5 | S_A8)
#define DUAL_SID_FLIPMASK  (CORE_PIN11_BITMASK | CORE_PIN13_BITMASK)

uint8_t pinPHI2  = 1,   // GPIO on port 6
        pinRW    = 2,   // GPIO on port 9
        pinCS    = 3,   // GPIO on port 9
        potX     = 4,   // GPIO on ...
        potY     = 5,   // ... port 9
        pin245OE = 6;   // GPIO on port 6

uint8_t pinButtonProfile = 29;   // GPIO on port 9

uint8_t outRegisters[ 32 ];
uint8_t outRegisters_2[ 32 ];

AudioStreamSID    *playSID;
AudioOutputMQS    *mqs;
AudioOutputPT8211 *audioPT8211;

uint32_t debounceCycle = 0;
uint32_t c64CycleCount;
uint32_t nCyclesEmulated;

uint32_t fmFakeOutput = 0;
uint32_t fmAutoDetectStep = 0;
uint32_t sidFakeOutput = 0;
uint32_t sidAutoDetectStep = 0;
uint32_t sidAutoDetectStep_2 = 0;
uint8_t  sidAutoDetectRegs[ 32 ];
uint8_t  sidAutoDetectRegs_2[ 32 ];

#define RING_SIZE (1024*4)
uint32_t ringBufGPIO[ RING_SIZE ];
uint32_t ringTime[ RING_SIZE ];
uint32_t ringWrite;
extern uint32_t ringRead;
extern unsigned long long samplesElapsed;
extern uint32_t samplesTotal;

volatile uint32_t dummy;

int doReset    = 0;
int disableISR = 0;

#define CACHE_PRELOAD( ptr ) { asm volatile ("pld\t[%0]" :: "r" (ptr)); }
#define CACHE_PRELOADW( ptr ) { asm volatile ("pldw\t[%0]" :: "r" (ptr)); }
#define CACHE_PRELOADI( ptr ) { asm volatile ("pli\t[%0]" :: "r" (ptr)); }

extern bool registerRead;
static uint32_t stateReadDirectoryMode = 0;
static uint32_t stateInTransferMode = 0;
static uint32_t stateInConfigMode = 0;
static uint32_t stateConfigRegisterAccess = 0;
static uint32_t stateWriteConfigToEEPROM = 0;

static const uint8_t  *transferData = NULL;
static uint8_t        *transferDataEnd = NULL;
static uint32_t       transferAddress, 
                      jumpAddress, 
                      launcherAddress,
                      transferStage;    // 0 = copy launcher, 1 = copy PRG, 2 = done

#define CONFIG_MODE_CYCLES 25000
uint8_t stateConfig[ 20 * 10 + 1 ];

void readConfigFromEEPROM()
{
  for (int i = 0; i < 20 * 10 + 1; i++)
    stateConfig[ i ] = EEPROM.read(i);

  #define MAX_SETTINGS 20
  char rangeSettings[ MAX_SETTINGS ]   = { 4, 16, 2, 16, 15, 91, 11, 101, 4, 16, 6, 16, 15, 91, 11, 101, 3, 16, 15, 2 };

  for ( int i = 0; i < 20 * 10; i++ )
  {
    if ( stateConfig[ i ] >= rangeSettings[ i % 20 ] )
      stateConfig[ i ] = 0;
  }

  if ( stateConfig[ 200 ] > 9 )
    stateConfig[ 200 ] = 0;

  for (int i = 0; i < 20 * 10 + 1; i++)
    EEPROM.write( i, stateConfig[ i ] );
}

void writeConfigToEEPROM()
{
  for (int i = 0; i < 20 * 10 + 1; i++)
    EEPROM.write( i, stateConfig[ i ] );
}

void setup()
{
  dummy = prgDirectory[ 0 ];
  dummy = prgRepository[ 0 ];
  
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
  attachInterrupt( digitalPinToInterrupt( pinPHI2 ), isrSID, RISING );

  playSID = new AudioStreamSID;
  
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

  readConfigFromEEPROM();
  uint8_t activeProfile = stateConfig[ 20 * 10 ];
  if ( activeProfile >= 10 )
    activeProfile = 0;

  c64CycleCount = 
  nCyclesEmulated = 
  fmFakeOutput = 
  fmAutoDetectStep = 
  sidFakeOutput = 
  sidAutoDetectStep = 
  sidAutoDetectStep_2 = 
  ringRead = ringWrite = 
  samplesElapsed = 
  samplesTotal = 0;

  playSID->updateConfiguration( &stateConfig[ activeProfile * 20 ] );

  playSID->begin();

  disableISR = 0;
  doReset = 1;
}

void resetEmulation()
{
  //playSID->stop();

  disableISR = 1;
  debounceCycle = 
  ringRead = 
  ringWrite = 
  nCyclesEmulated = 
  samplesElapsed = 
  samplesTotal = 0;
  c64CycleCount = 10;

  // send special command to reset sound chips
  ringBufGPIO[ ringWrite ] = 0xffffffff;
  ringTime[ ringWrite ] = c64CycleCount;
  ringWrite++;
  ringWrite &= ( RING_SIZE - 1 );
  c64CycleCount ++;

  //playSID->continuePlaying();

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


void loop()
{
  if ( doReset == 1 )
  {
    doReset = 2;
    resetEmulation();
    //speakSAM( 7 );
  }

  if ( stateWriteConfigToEEPROM )
  {
    stateWriteConfigToEEPROM = 0;
    writeConfigToEEPROM();
    
    uint8_t activeProfile = stateConfig[ 20 * 10 ];
    if ( activeProfile >= 10 )
      activeProfile = 0;
      
    playSID->updateConfiguration( &stateConfig[ activeProfile * 20 ] );
  }

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

#define CPU_RESET_CYCLECOUNTER    { ARM_DEMCR |= ARM_DEMCR_TRCENA; ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA; ARM_DWT_CYCCNT = 0; }

#define D_FLAGS (CORE_PIN19_BITMASK|CORE_PIN18_BITMASK|CORE_PIN14_BITMASK|CORE_PIN15_BITMASK|CORE_PIN17_BITMASK|CORE_PIN16_BITMASK|CORE_PIN22_BITMASK|CORE_PIN23_BITMASK)

inline void writeDataPins(uint8_t val)
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

  unsigned long cycles;
  do {
    cycles = ARM_DWT_CYCCNT;
#ifdef FIRMWARE_C128
  } while (cycles < ( 180 * TEENSY_CLOCK / 600 ) );
#else
  } while (cycles < ( 120 * TEENSY_CLOCK / 600 ) );
#endif
  CORE_PIN6_PORTSET = CORE_PIN6_BITMASK;
}

inline uint8_t readDataPins()
{
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
    CORE_PIN6_PORTCLEAR = CORE_PIN6_BITMASK;

  #define IMXRT_GPIO6_DIRECT (*(volatile uint32_t *)0x42000000)
  register uint32_t data = IMXRT_GPIO6_DIRECT;
  register uint32_t data2 = data >> 12;
  asm volatile( "bfi %0, %1, 14, 2" : "+r"( data ) : "r"( data2 ) );

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

volatile uint8_t forceRead;

FASTRUN void isrSID()
{
  // for accurate timing
  CPU_RESET_CYCLECOUNTER;

  // C128 needs at least a delay value of 6, let's go with 10 for now
  // to do: test with various PLAs on C64s
  #ifdef FIRMWARE_C128
  unsigned long cycles;
  do {
    cycles = ARM_DWT_CYCCNT;
  } while (cycles < ( 20 * TEENSY_CLOCK / 600 ) );
  #endif

  static int potCycleCounterAll = 0;

  c64CycleCount ++;
  potCycleCounterAll ++;

  // GPIO Bank #9: Phi2, CS, ...
  #define IMXRT_GPIO9_DIRECT (*(volatile uint32_t *)0x4200C000)
  register uint32_t data_9 = IMXRT_GPIO9_DIRECT;
  register uint32_t data2_9 = data_9 >> 12;
  asm volatile( "bfi %0, %1, 14, 2" : "+r"( data_9 ) : "r"( data2_9 ) );

  // read signals: address, RW, ...
  #define IMXRT_GPIO6_DIRECT (*(volatile uint32_t *)0x42000000)
  register uint32_t data = IMXRT_GPIO6_DIRECT;
  register uint32_t data2 = data >> 12;
  asm volatile( "bfi %0, %1, 14, 2" : "+r"( data ) : "r"( data2 ) );

  // check PHI2 (only if you're testing on breadboards)
  // if ( !( data & CORE_PIN1_BITMASK ) ) return;

  // read more signals
  #define IMXRT_GPIO7_DIRECT (*(volatile uint32_t *)0x42004000)
  register uint32_t data_7 = IMXRT_GPIO7_DIRECT;
  register uint32_t data2_7 = data_7 >> 12;
  asm volatile( "bfi %0, %1, 14, 2" : "+r"( data_7 ) : "r"( data2_7 ) );

  register uint32_t A = ( ( data & ( 1 << CORE_PIN0_BIT ) ) >> CORE_PIN0_BIT ) |
                        ( ( data & ( 3 << CORE_PIN24_BIT ) ) >> ( CORE_PIN24_BIT - 1 ) ) |
                        ( ( data & ( 3 << CORE_PIN26_BIT ) ) >> ( CORE_PIN26_BIT - 3 ) );

  #ifdef C128_D500_SUPPORT
  register uint32_t temp_7 = data_7;
  data_7 &= ~CORE_PIN37_BITMASK;
  #endif

  if ( disableISR )
    return;

  static int busValue = 0;
  static int busValueTTL = 0;

  if ( busValueTTL <= 0 )
  {
    busValue = 0;
  } else
    busValueTTL --;

  //
  // is (any) SID selected?
  //
  extern uint32_t SID2_ADDR;
  register uint32_t signals = ( ( data_7 ^ DUAL_SID_FLIPMASK ) & DUAL_SID_FLIPMASK ) | ( data_7 & DUAL_SID_ADDR_MASK );
  register uint32_t sid2 = (signals & SID2_ADDR);

  #ifdef C128_D500_SUPPORT
  uint8_t sid2_d500 = 0;
  if ( ( SID2_ADDR & CORE_PIN37_BITMASK ) &&  // D500?
       !( temp_7 & CORE_PIN36_BITMASK ) &&    // MMU Pin 47 (C64-Mode = low)
       !( temp_7 & CORE_PIN37_BITMASK ) &&    // A8 (read at U3 pin 14)
       !( data_9 & CORE_PIN2_BITMASK ) ) 
  {
    sid2_d500 = sid2 = 1;
  }
  data_7 = temp_7;
  #endif

  extern uint32_t SID2_MODEL;
  if ( SID2_MODEL == 0 ) sid2 = 0;

  if ( !( data_9 & CORE_PIN3_BITMASK ) // Chip Select
       #ifdef C128_D500_SUPPORT
       || sid2_d500 
       #endif
       || ( (SID2_ADDR & DUAL_SID_FLIPMASK) && sid2 ) )
  {
    if ( !( data_9 & CORE_PIN2_BITMASK ) ) // RW
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
          stateInConfigMode = CONFIG_MODE_CYCLES; // SID remains in config mode for 1/40sec
          goto noSIDCommand;
        } else 
        if ( A == 0x1e && stateInConfigMode > 0 )
        {
          stateConfigRegisterAccess = D;
          stateInConfigMode = CONFIG_MODE_CYCLES;
          goto noSIDCommand;
        } else
        if ( A == 0x1d && stateInConfigMode > 0 )
        {
          if ( D == 0xff )
          {
            stateWriteConfigToEEPROM = 1;
          } else
          {
            stateConfig[ stateConfigRegisterAccess ] = D;
            stateInConfigMode = CONFIG_MODE_CYCLES;
          }
          goto noSIDCommand;
        } else
        if ( A == 0x1c && stateInConfigMode > 0 )
        {
          // launch PRG!
          if ( D == 0xff ) // read directory
          {
            stateReadDirectoryMode = 0x1000;
            transferData = &prgDirectory[ 0 ];
            forceRead = *transferData;
          } else
          {
            const uint8_t *dirEntry = &prgDirectory[ D * 24 ];
            uint32_t ofs = ( dirEntry[ 19 ] * 256 + dirEntry[ 20 ] ) * 256 + dirEntry[ 21 ];
            prgSize = dirEntry[ 22 ] * 256 + dirEntry[ 23 ];
            prgCode = &prgRepository[ ofs ];

            launchCode[ 11 ] = ( 0x0801 + prgSize + 2 ) & 255;
            launchCode[ 13 ] = ( 0x0801 + prgSize + 2 ) >> 8;

            for ( int i = 0; i < 256; i++ )
              forceRead = prgCode[ i ];

            stateInConfigMode = 0;
            stateInTransferMode = 1;
          }
          goto noSIDCommand;
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
        ringBufGPIO[ ringWrite ] = D | ( A << 8 ) | ( sid2 ? 1 << 20 : 0 );
        ringTime[ ringWrite ] = c64CycleCount;
        ringWrite++;
        ringWrite &= ( RING_SIZE - 1 );
  
        // pseudo stereo
        if ( SID2_ADDR == (uint32_t)(1 << 31) )
        {
          ringBufGPIO[ ringWrite ] = D | ( A << 8 ) | ( 1 << 20 );
          ringTime[ ringWrite ] = c64CycleCount;
          ringWrite++;
          ringWrite &= ( RING_SIZE - 1 );
        }
      }
    } else
    {
      //
      // CPU reads SID register
      //
      if ( sid2 == 0 )
      {
        if ( stateInTransferMode && A < 16 )
        {
  
          if ( A == 0 ) // start transfer
          {
            writeDataPins( 0xEA ); // execute NOP
            transferStage = 0;
            launcherAddress = transferAddress = ( launchCode[ 1 ] << 8 ) + launchCode[ 0 ];
            transferData = (uint8_t *)&launchCode[ 2 ];
            transferDataEnd = (uint8_t *)&launchCode[ launchSize ];
            jumpAddress = 0xD401;
            forceRead = *transferData;
          } else if ( A == 1 )
          {
            writeDataPins( 0xA9 ); // execute LDA
            forceRead = *transferData;
          } else if ( A == 2 )
          {
            writeDataPins( forceRead ); // LDA value
            transferData ++;
            forceRead = *transferData;
          } else if ( A == 3 )
          {
            writeDataPins( 0x8D ); // execute STA
            forceRead = *transferData;
          } else if ( A == 4 )
          {
            writeDataPins( transferAddress & 255 ); // STA lowbyte address
            forceRead = *transferData;
          } else if ( A == 5 )
          {
            writeDataPins( transferAddress >> 8 ); // STA highbyte address
            transferAddress ++;
          } else if ( A == 6 )
          {
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
          } else if ( A == 7 )
          {
            writeDataPins( jumpAddress & 255 ); // JMP address low byte
          } else if ( A == 8 )
          {
            writeDataPins( jumpAddress >> 8 ); // JMP address high byte
            if ( stateInTransferMode && jumpAddress == launcherAddress )
              stateInTransferMode = 0;
          }
        } else
        if ( A == 0x1d && stateInConfigMode == 0 )
        {
          prgSize = cfgPRGSize;
          prgCode = cfgPRGCode;
  
          stateInTransferMode = 1;
          writeDataPins( 0x4C ); // JMP
        } else if ( A == 0x1e && stateInConfigMode == 0 )
        {
          writeDataPins( 0x00 );
        } else if ( A == 0x1f && stateInConfigMode == 0 )
        {
          writeDataPins( 0xD4 );
        } else if ( A == 0x1d && stateInConfigMode > 0 )
        {
          if ( stateConfigRegisterAccess < 224 )
            writeDataPins( stateConfig[ stateConfigRegisterAccess ] ); else 
          if ( stateConfigRegisterAccess < 224 + 16 )
            writeDataPins( VERSION_STR[ stateConfigRegisterAccess - 224 ] );
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
            stateReadDirectoryMode = 0;
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
                //writeDataPins( 0 ); else
                writeDataPins( outRegisters_2[ A & 31 ] ); else
                writeDataPins( busValue );
            }
          }
      }
    }
    return;
  }

  extern bool emulateFM, readRegistersFM;
  if ( emulateFM )
  {
    if ( readRegistersFM )
    if ( ( data_9 & CORE_PIN2_BITMASK ) ) // RW
      if ( !( data_7 & CORE_PIN13_BITMASK ) ) // IO2
      {
        // CPU reads from bus
        if ( ( data_7 & CORE_PIN34_BITMASK ) ) A |= 1 << 5;
        if ( ( data_7 & CORE_PIN35_BITMASK ) ) A |= 1 << 6;
        if ( ( data_7 & CORE_PIN36_BITMASK ) ) A |= 1 << 7;
        if ( ( data_7 & CORE_PIN37_BITMASK ) ) A |= 1 << 8;
  
        if ( (A & 0x60) == 0x60 )
        {
          // this is not a real read of the YM3812 status register!
          // only a fake that let's the detection routine be happy
          uint32_t D = fmFakeOutput; 
          fmFakeOutput = 0xc0 - fmFakeOutput;
          writeDataPins( D ); 
          return;
        } else
        if ( A >= 0x08 && A <= 0x0f )
        {
          // reading of the external Sound Expander Keyboard => we don't have it, return 0xFF
          writeDataPins( 0xff ); 
          return;
        }
      }

    if ( !( data_9 & CORE_PIN2_BITMASK ) ) // RW
    {
      if ( !( data_7 & CORE_PIN13_BITMASK ) ) // IO2
      {
        register uint8_t D = readDataPins();

        if ( ( data_7 & CORE_PIN34_BITMASK ) ) A |= 1 << 5;
        if ( ( data_7 & CORE_PIN35_BITMASK ) ) A |= 1 << 6;
        if ( ( data_7 & CORE_PIN36_BITMASK ) ) A |= 1 << 7;
        if ( ( data_7 & CORE_PIN37_BITMASK ) ) A |= 1 << 8;

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
        

        ringBufGPIO[ ringWrite ] = D | ( A << 8 ) | ( 1 << 28 );
        ringTime[ ringWrite ] = c64CycleCount;
        ringWrite++;
        ringWrite &= ( RING_SIZE - 1 );
        return;
      }
    } 
    
  }
noSIDCommand:

  if ( true )
  {
    static int32_t potCycleCounter = 0;
    static int newPotCounterX = 0;
    static int newPotCounterY = 0;

    if ( potCycleCounter == 0 )
    {
      pinMode( potX, OUTPUT ); pinMode( potY, OUTPUT );
      CORE_PIN4_PORTCLEAR = CORE_PIN4_BITMASK | CORE_PIN5_BITMASK;
    } else if ( potCycleCounter == 256 )
    {
      pinMode( potX, INPUT ); pinMode( potY, INPUT );

      newPotCounterX = 1;
      newPotCounterY = 1;
    } else if ( potCycleCounter >= 256 )
    {
      if ( newPotCounterX && (data_9 & CORE_PIN4_BITMASK) )
      {
        outRegisters[ 25 ] = potCycleCounter - 256 - 5;
        newPotCounterX = 0;
      }
      if ( newPotCounterY && (data_9 & CORE_PIN5_BITMASK) )
      {
        outRegisters[ 26 ] = potCycleCounter - 256 - 5;
        newPotCounterY = 0;
      }
    }

    potCycleCounter ++;

    if ( potCycleCounterAll >= 512 )
    {
      if ( newPotCounterX ) outRegisters[ 25 ] = 255;
      if ( newPotCounterY ) outRegisters[ 26 ] = 255;
      potCycleCounter = 0;
      potCycleCounterAll = 0;
    }
  }

  if ( stateInConfigMode > 0 )
    stateInConfigMode --;

  if ( !( data_9 & CORE_PIN29_BITMASK ) && debounceCycle < c64CycleCount ) // Button
  {
    stateConfig[ 200 ] ++;
    stateConfig[ 200 ] %= 10;
    debounceCycle = c64CycleCount + 200000;

    playSID->updateConfiguration( &stateConfig[ stateConfig[ 200 ] * 20 ] );
    speakSAM( stateConfig[ 200 ] );
  }

  if ( !(data_7 & CORE_PIN32_BITMASK) ) // reset
  {
    if ( doReset == 0 )
    {
      //playSID->stop();
      doReset = 1;
      return;
    }
  } else 
  if ( doReset == 2 )
    doReset = 0;
}
