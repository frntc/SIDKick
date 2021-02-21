/*
   _________.___________   ____  __.__        __     
  /   _____/|   \______ \ |    |/ _|__| ____ |  | __ 
  \_____  \ |   ||    |  \|      < |  |/ ___\|  |/ / 
  /        \|   ||    `   \    |  \|  \  \___|    <  
 /_______  /|___/_______  /____|__ \__|\___  >__|_ \ 
         \/             \/        \/       \/     \/ 
        
 AudioStreamSID.h

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

#include "incl_resid.h"

#ifndef AudioStreamSID_h_
#define AudioStreamSID_h_

#include <core_pins.h>
#include <AudioStream.h>

#define EMULATE_OPL2

#ifdef EMULATE_OPL2
#include "fmopl.h"
#endif

extern void speakSAM( uint8_t digit );

class AudioStreamSID : public AudioStream
{
public:
	AudioStreamSID(void) : AudioStream( 0, NULL) { init(); }
  void init();
  void begin();
	void reset();
	void stop();
  void continuePlaying();
  void quickreset();
  void updateConfiguration( uint8_t *cfg );

private:
	volatile bool playing;
	virtual void update();
	SID16 *sid16_1, *sid16_2;
  RESID_NAMESPACE::SID *sid_1, *sid_2;

  #ifdef EMULATE_OPL2
  FM_OPL *pOPL;
  uint32_t fmOutRegister, fmFakeOutput;
  #endif

public:
  RESID_NAMESPACE::SID *getSID() { return sid_1; };
  SID16 *getSID16() { return sid16_1; };
  
};

#endif
