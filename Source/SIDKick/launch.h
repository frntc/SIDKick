/*
   _________.___________   ____  __.__        __     
  /   _____/|   \______ \ |    |/ _|__| ____ |  | __ 
  \_____  \ |   ||    |  \|      < |  |/ ___\|  |/ / 
  /        \|   ||    `   \    |  \|  \  \___|    <  
 /_______  /|___/_______  /____|__ \__|\___  >__|_ \ 
         \/             \/        \/       \/     \/ 
        
 SIDKick - SID-replacement with SID and Sound Expander Emulation based on Teensy 4.1
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

const long int launchSize = 55;
unsigned char launchCode[55] = {
  // with keyboard buffer and rU:[return]
  0x3C, 0x03, 0xA2, 0x01, 0xA9, 0x08, 0x86, 0x2B, 0x85, 0x2C, 0xA2, 0x01, 0xA9, 0x80, 0x86, 0x2D,
  0x85, 0x2E, 0x86, 0x2F, 0x85, 0x30, 0x86, 0x31, 0x85, 0x32, 0x86, 0xAE, 0x85, 0xAF, 0xA9, 0x52,
  0x8D, 0x77, 0x02, 0xA9, 0xD5, 0x8D, 0x78, 0x02, 0xA9, 0x3A, 0x8D, 0x79, 0x02, 0xA9, 0x0D, 0x8D,
  0x7A, 0x02, 0xA9, 0x04, 0x85, 0xC6, 0x60
};
