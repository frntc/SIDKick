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

uint8_t stateConfig[ 20 * 10 + 1 ];

void setup()
{
  uint8_t defaultSettings[ 20 ] = { 2, 12, 0, 15,  7, 90,  7,  55, 3, 12, 0, 15, 7, 90,  7,  55, 0, 15, 7, 1 };

  // yes I know I could directly call EEPROM.write, but this is copy-paste
  for (int i = 0; i < 20 * 10; i++)
    stateConfig[ i ] = defaultSettings[ i % 20 ];

  stateConfig[ 200 ] = 0;
  
  for (int i = 0; i < 20 * 10 + 1; i++)
    EEPROM.write( i, stateConfig[ i ] );
}


void loop()
{
}
