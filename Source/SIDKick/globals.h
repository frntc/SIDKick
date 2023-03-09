/*
   _________.___________   ____  __.__        __     
  /   _____/|   \______ \ |    |/ _|__| ____ |  | __ 
  \_____  \ |   ||    |  \|      < |  |/ ___\|  |/ / 
  /        \|   ||    `   \    |  \|  \  \___|    <  
 /_______  /|___/_______  /____|__ \__|\___  >__|_ \ 
         \/             \/        \/       \/     \/ 

  globals.h

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

#pragma GCC optimize( "Ofast", "unroll-loops", "omit-frame-pointer", "inline", "finite-math-only", "single-precision-constant", "fp-contract=fast", "modulo-sched", "modulo-sched-allow-regmoves", "gcse-sm", "gcse-las" ) 
//#pragma GCC optimize( "Ofast", "unroll-loops", "omit-frame-pointer", "inline", "finite-math-only", "single-precision-constant", "fp-contract=fast" ) 

// choose audio device used for output: 0 = MQS, 1 = DAC
#define audioDevice 1

// Teensy clock speed (keep 816 unless you deactivate MIDI and LED)
#define TEENSY_CLOCK 816

#define FIRMWARE_C128

// C128 does not support $D500 for SID #2
// except for the C64-mode if:
// A8 is connected to U3 pin 14, and A7 to U7 (MMU, MOS 8722) pin 47
#ifdef FIRMWARE_C128
#define C128_D500_SUPPORT
#endif

// define if MIDI device emulation to be included in the firmware
//#define SUPPORT_MIDI

// define if WS2818b-LED support to be included in the firmware
#define FANCY_LED

// define this to use the sample generation routine which adapts to the actual C64/C128 clock 
#define DYNAMIC_ADJUSTMENT_MIXER

// support SID-write only mode
#define SID_WRITE_ONLY_MODE_SUPPORT

// support straight DAC output
#define SID_DAC_MODE_SUPPORT
#define SID_DAC_OFF      0
#define SID_DAC_MONO8    1
#define SID_DAC_STEREO8  2
#define SID_DAC_MONO16   3
#define SID_DAC_STEREO16 4

//#define DEBUG_OUTPUT
