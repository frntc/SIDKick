/*
   _________.___________   ____  __.__        __     
  /   _____/|   \______ \ |    |/ _|__| ____ |  | __ 
  \_____  \ |   ||    |  \|      < |  |/ ___\|  |/ / 
  /        \|   ||    `   \    |  \|  \  \___|    <  
 /_______  /|___/_______  /____|__ \__|\___  >__|_ \ 
         \/             \/        \/       \/     \/ 

  SIDKickEEPROM.ino

  SIDKickEEPROM - resets/initializes the SIDKick-settings stored in the EEPROM
  Copyright (c) 2021 Carsten Dachsbacher <frenetic@dachsbacher.de>

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

const unsigned char defaultSettings[ 64 ] = { 
  2,  // SID #1
  12, // Digiboost
  1,  // Register Read
  15, // volume
  7,  // panning
  90, // passband
  7,  // gain
  60, // filter bias
   3, // sid #2
  12, // digiboost
   0, // address
  15, // vol
   7, // panning
  90, // passband
  7,  // gain
  60, // filter bias
   0, // Sound Expander
  15, // vol
   7, // panning
   1, // reSID version
   2, // LED mode
   7, // #LEDs 
   8, // Window
  15, // Scale
   1, // Color Cycle Speed
   0, // H
  31, // S
  31, // V
  45, // H 
  31, // S
  31, // V
   0, // MIDI
   0, // MIDI input enabled?
   0, // SID+FM Delay
   0, // Delay sign
   7, // SAM
   0, 0, 0, 0, 0, 0, 0, 0,
   0, 0, 0, 0, 0, 0, 0, 0,
   0, 0, 0, 0, 0, 0, 0, 0, 
   0, 0, 0, 0 };

void setup()
{
  for (int i = 0; i < 64 * 10; i++)
    EEPROM.write( i, defaultSettings[ i % 64 ] );

  for (int i = 0; i < 64; i++)
    EEPROM.write( 640 + i, 0 );
}

void loop()
{
}
