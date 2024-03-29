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

#define RESID_FILTER_CC

#ifdef _M_ARM
#undef _ARM_WINAPI_PARTITION_DESKTOP_SDK_AVAILABLE
#define _ARM_WINAPI_PARTITION_DESKTOP_SDK_AVAILABLE 1
#endif

#include "filter.h"
#include "dac.h"
#include "spline.h"
#include <math.h>
//#include "summer.h"
//#include "mixer.h"


namespace reSID
{

const int opamp0[ 256 ][ 2 ] = {
{ 0, 0 } , { 0, 0 } , { 0, 0 } , { 0, 0 } , { 0, 0 } , { 0, 0 } , { 0, 0 } , { 0, 0 } , { 61432, 0 } , { 60913, -14792 } , { 60402, 1528 } , { 59902, 26520 } , { 59412, -20816 } , { 58922, -20480 } , { 58433, -18928 } , { 57944, -18432 } , { 57455, -17824 } , { 56967, -16384 } , { 56479, -16384 } , { 55992, -14472 } , { 55505, -14336 } , { 55019, -12984 } , { 54533, -12288 } , { 54047, -11336 } , 
{ 53562, -10240 } , { 53078, -9528 } , { 52594, -8192 } , { 52110, -7592 } , { 51627, -6144 } , { 51144, -5528 } , { 50662, -4096 } , { 50181, -3360 } , { 49700, -1456 } , { 49221, 1456 } , { 48743, 4616 } , { 48266, 7048 } , { 47791, 9312 } , { 47317, 11360 } , { 46843, 13072 } , { 46370, 14336 } , { 45898, 16256 } , { 45426, 16384 } , { 44955, 17936 } , { 44484, 18432 } , { 44013, 18432 } , { 43541, 17432 } , { 43070, 17256 } , { 42599, 18424 } , 
{ 42129, 21000 } , { 41662, 26096 } , { 41198, -32088 } , { 40739, -22424 } , { 40286, -10768 } , { 39837, -1272 } , { 39391, 4944 } , { 38949, 10648 } , { 38509, 16096 } , { 38071, 20808 } , { 37635, 25192 } , { 37201, 29136 } , { 36769, 32424 } , { 36338, -29896 } , { 35913, -20296 } , { 35492, -9576 } , { 35075, -1752 } , { 34661, 3112 } , { 34247, 4096 } , { 33832, 3448 } , { 33417, 1032 } , { 33004, 6104 } , { 32600, 24400 } , { 32206, -18668 } , 
{ 31820, -4812 } , { 31446, 19940 } , { 31089, -10124 } , { 30747, 21044 } , { 30417, -21104 } , { 30095, -3364 } , { 29779, 8704 } , { 29468, 17536 } , { 29163, 30564 } , { 28867, -16836 } , { 28582, 6548 } , { 28322, -8156 } , { 28086, -24532 } , { 27863, 1092 } , { 27668, -4204 } , { 27496, -26140 } , { 27339, 7664 } , { 27197, -29672 } , { 27064, -9496 } , { 26955, -27856 } , { 26864, 10280 } , { 26782, 29960 } , { 26708, -21140 } , { 26638, -11928 } , 
{ 26571, -6056 } , { 26507, -1272 } , { 26444, 2200 } , { 26382, 4544 } , { 26320, 5120 } , { 26259, 5088 } , { 26197, 3520 } , { 26133, 708 } , { 26067, -3388 } , { 25999, -8780 } , { 25929, -12728 } , { 25858, -14684 } , { 25786, -15940 } , { 25714, -17124 } , { 25641, -18224 } , { 25567, -19200 } , { 25493, -19992 } , { 25419, -20480 } , { 25345, -21472 } , { 25271, -21504 } , { 25196, -21872 } , { 25121, -22528 } , { 25046, -22528 } , { 24971, -22528 } , 
{ 24896, -21716 } , { 24822, -21504 } , { 24747, -21316 } , { 24674, -19752 } , { 24601, -17504 } , { 24529, -16460 } , { 24456, -18940 } , { 24380, -25040 } , { 24299, 30764 } , { 24214, 24264 } , { 24130, 23152 } , { 24044, 21868 } , { 23958, 20056 } , { 23871, 18140 } , { 23783, 15828 } , { 23693, 13280 } , { 23602, 10368 } , { 23510, 7176 } , { 23416, 3676 } , { 23320, 1048 } , { 23224, 32 } , { 23128, 0 } , { 23032, 0 } , { 22936, -248 } , 
{ 22840, -1024 } , { 22743, -1024 } , { 22646, -1412 } , { 22549, -2048 } , { 22452, -2048 } , { 22355, -3036 } , { 22257, -3072 } , { 22160, -3932 } , { 22062, -4096 } , { 21963, -5036 } , { 21864, -5272 } , { 21765, -6144 } , { 21666, -6660 } , { 21567, -7168 } , { 21467, -8148 } , { 21366, -8700 } , { 21266, -9324 } , { 21165, -10240 } , { 21064, -11036 } , { 20962, -11780 } , { 20860, -12572 } , { 20757, -13468 } , { 20654, -15348 } , { 20549, -17472 } , 
{ 20444, -19448 } , { 20337, -21236 } , { 20230, -22992 } , { 20122, -24460 } , { 20013, -25936 } , { 19904, -27128 } , { 19794, -28220 } , { 19684, -29160 } , { 19574, -29844 } , { 19463, -30720 } , { 19351, -30924 } , { 19240, -31744 } , { 19128, -31744 } , { 19017, -31744 } , { 18905, -31744 } , { 18794, -31152 } , { 18682, -32320 } , { 18570, 31980 } , { 18457, 30512 } , { 18343, 29228 } , { 18229, 27944 } , { 18114, 26400 } , { 17998, 25060 } , { 17882, 23616 } , 
{ 17764, 22092 } , { 17646, 20672 } , { 17528, 19044 } , { 17408, 17800 } , { 17288, 16436 } , { 17168, 14944 } , { 17046, 13408 } , { 16924, 11696 } , { 16801, 9744 } , { 16676, 7720 } , { 16551, 5480 } , { 16425, 3160 } , { 16297, 684 } , { 16167, -3726 } , { 16030, -18844 } , { 15887, -31334 } , { 15740, 28168 } , { 15594, 28656 } , { 15451, -31872 } , { 15308, -31440 } , { 15163, 30852 } , { 15014, 23322 } , { 14860, 12336 } , { 14705, 9198 } , 
{ 14544, -1788 } , { 14372, -24260 } , { 14183, 6704 } , { 13969, 20204 } , { 13736, -18382 } , { 13493, 26692 } , { 13235, -4488 } , { 12933, -28762 } , { 12550, 3214 } , { 12129, -11864 } , { 11645, -8260 } , { 11166, 2226 } , { 10680, -12438 } , { 10177, 18484 } , { 9667, 5058 } , { 9157, 4270 } , { 8647, 3404 } , { 8136, 2704 } , { 7625, 2009 } , { 7114, 1328 } , { 6602, 732 } , { 6090, 166 } , { 5578, -366 } , { 5066, -853 } , 
{ 4553, -1283 } , { 4040, -1672 } , { 3527, -2027 } , { 3014, -2324 } , { 2501, -2586 } , { 1987, -2792 } , { 1474, -2968 } , { 960, -3101 } , { 447, -3181 } , { 0, 0 } , { 0, 0 } , { 0, 0 } , { 0, 0 } , { 0, 0 } , { 0, 0 } , { 0, 0 } };

const int opamp1[ 256 ][ 2 ] = {
{ 0, 0 } , { 65026, 0 } , { 64514, 0 } , { 64002, 0 } , { 63490, 0 } , { 62978, 0 } , { 62466, 0 } , { 61954, 0 } , { 61442, 0 } , { 60930, 0 } , { 60418, 0 } , { 59906, 0 } , { 59394, 0 } , { 58882, 0 } , { 58370, 0 } , { 57858, 0 } , { 57346, 0 } , { 56834, 0 } , { 56322, 0 } , { 55810, 0 } , { 55298, 0 } , { 54786, 0 } , { 54274, 0 } , { 53762, 0 } , 
{ 53250, 0 } , { 52738, 0 } , { 52226, 0 } , { 51714, 0 } , { 51202, 0 } , { 50690, 0 } , { 50178, 0 } , { 49666, 0 } , { 49154, 0 } , { 48642, 0 } , { 48130, 0 } , { 47618, 0 } , { 47106, 0 } , { 46594, 0 } , { 46082, 0 } , { 45570, 0 } , { 45058, 0 } , { 44546, 0 } , { 44034, 0 } , { 43522, 0 } , { 43010, 0 } , { 42498, 0 } , { 41986, 0 } , { 41474, 0 } , 
{ 40962, 0 } , { 40450, 0 } , { 39938, 0 } , { 39426, 0 } , { 38914, 0 } , { 38402, 0 } , { 37890, 0 } , { 37378, 0 } , { 36866, 0 } , { 36354, 0 } , { 35842, 0 } , { 35330, 0 } , { 34818, 0 } , { 34306, 0 } , { 33794, 0 } , { 33282, 0 } , { 32770, 0 } , { 32237, 22540 } , { 31765, 17868 } , { 31385, 7000 } , { 31042, 19964 } , { 30769, 30412 } , { 30624, 31136 } , { 30513, -31532 } , 
{ 30440, -18108 } , { 30393, -31476 } , { 30364, 6908 } , { 30353, -22160 } , { 30347, -12984 } , { 30341, -11708 } , { 30336, -10488 } , { 30332, -9328 } , { 30328, -8244 } , { 30324, -7264 } , { 30321, -6424 } , { 30318, -5816 } , { 30316, -5120 } , { 30313, -4620 } , { 30311, -4108 } , { 30309, -4096 } , { 30307, -4096 } , { 30305, -4096 } , { 30303, -4096 } , { 30302, -3140 } , { 30300, -3072 } , { 30299, -3072 } , { 30297, -3072 } , { 30296, -3072 } , 
{ 30294, -3072 } , { 30293, -3072 } , { 30291, -3072 } , { 30290, -3072 } , { 30288, -3072 } , { 30287, -3072 } , { 30285, -2768 } , { 30284, -2048 } , { 30283, -2048 } , { 30282, -2048 } , { 30281, -2048 } , { 30280, -2048 } , { 30279, -2048 } , { 30278, -2048 } , { 30277, -2048 } , { 30276, -2048 } , { 30275, -2076 } , { 30274, -3072 } , { 30272, -3072 } , { 30271, -3072 } , { 30269, -4464 } , { 30267, -3072 } , { 30266, -3072 } , { 30264, -3072 } , 
{ 30263, -2276 } , { 30262, -2048 } , { 30261, -2048 } , { 30260, -2048 } , { 30259, -2048 } , { 30258, -2048 } , { 30257, -2048 } , { 30256, -2048 } , { 30255, -2048 } , { 30254, -2048 } , { 30253, -2048 } , { 30252, -2048 } , { 30251, -2048 } , { 30250, -2048 } , { 30249, -2048 } , { 30248, -2048 } , { 30247, -2048 } , { 30246, -2048 } , { 30245, -2048 } , { 30244, -2048 } , { 30243, -2048 } , { 30242, -2700 } , { 30240, -3072 } , { 30239, -3072 } , 
{ 30237, -3072 } , { 30236, -3072 } , { 30234, -3072 } , { 30233, -3072 } , { 30231, -3072 } , { 30230, -3072 } , { 30227, -6720 } , { 30225, -3072 } , { 30224, -3072 } , { 30222, -3072 } , { 30221, -3072 } , { 30219, -3072 } , { 30218, -3072 } , { 30216, -3072 } , { 30215, -3072 } , { 30213, -3072 } , { 30212, -3072 } , { 30210, -3124 } , { 30208, -4096 } , { 30206, -4096 } , { 30204, -4096 } , { 30202, -4096 } , { 30200, -4096 } , { 30198, -4096 } , 
{ 30196, -4096 } , { 30194, -4096 } , { 30192, -4096 } , { 30190, -4096 } , { 30188, -4740 } , { 30185, -5120 } , { 30182, -6044 } , { 30180, -5120 } , { 30177, -5120 } , { 30174, -6088 } , { 30171, -6144 } , { 30168, -6144 } , { 30165, -7004 } , { 30161, -7168 } , { 30158, -7276 } , { 30154, -8192 } , { 30150, -8192 } , { 30146, -8840 } , { 30141, -9216 } , { 30136, -10436 } , { 30130, -12496 } , { 30123, -14248 } , { 30115, -15844 } , { 30107, -17116 } , 
{ 30098, -18200 } , { 30088, -19372 } , { 30077, -22908 } , { 30063, -29640 } , { 30039, 16564 } , { 29962, -26396 } , { 29695, -23012 } , { 29184, 3072 } , { 28674, 3072 } , { 28163, 2604 } , { 27652, 2048 } , { 27141, 2048 } , { 26630, 2048 } , { 26119, 2048 } , { 25608, 2048 } , { 25097, 2048 } , { 24586, 2048 } , { 24074, 1024 } , { 23563, 1024 } , { 23051, 1024 } , { 22540, 1024 } , { 22028, 1024 } , { 21517, 1024 } , { 21005, 1024 } , 
{ 20494, 1024 } , { 19982, 740 } , { 19470, 0 } , { 18958, 0 } , { 18446, 0 } , { 17934, 0 } , { 17422, 0 } , { 16910, 0 } , { 16398, 0 } , { 15886, -158 } , { 15374, -512 } , { 14862, -512 } , { 14350, -512 } , { 13837, -512 } , { 13325, -512 } , { 12813, -512 } , { 12301, -512 } , { 11788, -994 } , { 11276, -1024 } , { 10763, -1024 } , { 10251, -1024 } , { 9738, -1024 } , { 9226, -1024 } , { 8713, -1024 } , 
{ 8201, -1024 } , { 7688, -1276 } , { 7175, -1280 } , { 6663, -1280 } , { 6150, -1280 } , { 5637, -1280 } , { 5125, -1280 } , { 4612, -1498 } , { 4099, -1536 } , { 3587, -1518 } , { 3074, -1536 } , { 2561, -1536 } , { 2048, -1536 } , { 1536, -1536 } , { 1023, -1536 } , { 510, -1536 } };



// This is the SID 6581 op-amp voltage transfer function, measured on
// CAP1B/CAP1A on a chip marked MOS 6581R4AR 0687 14.
// All measured chips have op-amps with output voltages (and thus input
// voltages) within the range of 0.81V - 10.31V.

static const double_point opamp_voltage_6581[] = {
  {  0.81, 10.31 },  // Approximate start of actual range
  {  0.81, 10.31 },  // Repeated point
  {  2.40, 10.31 },
  {  2.60, 10.30 },
  {  2.70, 10.29 },
  {  2.80, 10.26 },
  {  2.90, 10.17 },
  {  3.00, 10.04 },
  {  3.10,  9.83 },
  {  3.20,  9.58 },
  {  3.30,  9.32 },
  {  3.50,  8.69 },
  {  3.70,  8.00 },
  {  4.00,  6.89 },
  {  4.40,  5.21 },
  {  4.54,  4.54 },  // Working point (vi = vo)
  {  4.60,  4.19 },
  {  4.80,  3.00 },
  {  4.90,  2.30 },  // Change of curvature
  {  4.95,  2.03 },
  {  5.00,  1.88 },
  {  5.05,  1.77 },
  {  5.10,  1.69 },
  {  5.20,  1.58 },
  {  5.40,  1.44 },
  {  5.60,  1.33 },
  {  5.80,  1.26 },
  {  6.00,  1.21 },
  {  6.40,  1.12 },
  {  7.00,  1.02 },
  {  7.50,  0.97 },
  {  8.50,  0.89 },
  { 10.00,  0.81 },
  { 10.31,  0.81 },  // Approximate end of actual range
  { 10.31,  0.81 }   // Repeated end point
};

// This is the SID 8580 op-amp voltage transfer function, measured on
// CAP1B/CAP1A on a chip marked CSG 8580R5 1690 25.
static const double_point opamp_voltage_8580[] = {
  {  1.30,  8.91 },  // Approximate start of actual range
  {  1.30,  8.91 },  // Repeated end point
  {  4.76,  8.91 },
  {  4.77,  8.90 },
  {  4.78,  8.88 },
  {  4.785, 8.86 },
  {  4.79,  8.80 },
  {  4.795, 8.60 },
  {  4.80,  8.25 },
  {  4.805, 7.50 },
  {  4.81,  6.10 },
  {  4.815, 4.05 },  // Change of curvature
  {  4.82,  2.27 },
  {  4.825, 1.65 },
  {  4.83,  1.55 },
  {  4.84,  1.47 },
  {  4.85,  1.43 },
  {  4.87,  1.37 },
  {  4.90,  1.34 },
  {  5.00,  1.30 },
  {  5.10,  1.30 },
  {  8.91,  1.30 },  // Approximate end of actual range
  {  8.91,  1.30 }   // Repeated end point
};


/*
 * R1 = 15.3*Ri
 * R2 =  7.3*Ri
 * R3 =  4.7*Ri
 * Rf =  1.4*Ri
 * R4 =  1.4*Ri
 * R8 =  2.0*Ri
 * RC =  2.8*Ri
 *
 * res  feedback  input
 * ---  --------  -----
 *  0   Rf        Ri
 *  1   Rf|R1     Ri
 *  2   Rf|R2     Ri
 *  3   Rf|R3     Ri
 *  4   Rf        R4
 *  5   Rf|R1     R4
 *  6   Rf|R2     R4
 *  7   Rf|R3     R4
 *  8   Rf        R8
 *  9   Rf|R1     R8
 *  A   Rf|R2     R8
 *  B   Rf|R3     R8
 *  C   Rf        RC
 *  D   Rf|R1     RC
 *  E   Rf|R2     RC
 *  F   Rf|R3     RC
 */
static const int resGain[16] =
{
  (int)((1<<7)*(1.4/1.0)),                     //      Rf/Ri   1.4
  (int)((1<<7)*(((1.4*15.3)/(1.4+15.3))/1.0)), // (Rf|R1)/Ri   1.28263
  (int)((1<<7)*(((1.4*7.3)/(1.4+7.3))/1.0)),   // (Rf|R2)/Ri   1.17471
  (int)((1<<7)*(((1.4*4.7)/(1.4+4.7))/1.0)),   // (Rf|R3)/Ri   1.07869
  (int)((1<<7)*(1.4/1.4)),                     //      Rf/R4   1.0
  (int)((1<<7)*(((1.4*15.3)/(1.4+15.3))/1.4)), // (Rf|R1)/R4   0.916168
  (int)((1<<7)*(((1.4*7.3)/(1.4+7.3))/1.4)),   // (Rf|R2)/R4   0.83908
  (int)((1<<7)*(((1.4*4.7)/(1.4+4.7))/1.4)),   // (Rf|R3)/R4   0.770492
  (int)((1<<7)*(1.4/2.0)),                     //      Rf/R8   0.7
  (int)((1<<7)*(((1.4*15.3)/(1.4+15.3))/2.0)), // (Rf|R1)/R8   0.641317
  (int)((1<<7)*(((1.4*7.3)/(1.4+7.3))/2.0)),   // (Rf|R2)/R8   0.587356
  (int)((1<<7)*(((1.4*4.7)/(1.4+4.7))/2.0)),   // (Rf|R3)/R8   0.539344
  (int)((1<<7)*(1.4/2.8)),                     //      Rf/RC   0.5
  (int)((1<<7)*(((1.4*15.3)/(1.4+15.3))/2.8)), // (Rf|R1)/RC   0.458084
  (int)((1<<7)*(((1.4*7.3)/(1.4+7.3))/2.8)),   // (Rf|R2)/RC   0.41954
  (int)((1<<7)*(((1.4*4.7)/(1.4+4.7))/2.8)),   // (Rf|R3)/RC   0.385246
};

typedef struct {
  // Op-amp transfer function.
  const double_point* opamp_voltage;
  int opamp_voltage_size;
  // Voice output characteristics.
  float voice_voltage_range;
  float voice_DC_voltage;
  // Capacitor value.
  float C;
  // Transistor parameters.
  float Vdd;
  float Vth;        // Threshold voltage
  float Ut;         // Thermal voltage: Ut = k*T/q = 8.61734315e-5*T ~ 26mV
  float k;          // Gate coupling coefficient: K = Cox/(Cox+Cdep) ~ 0.7
  float uCox;       // u*Cox
  float WL_vcr;     // W/L for VCR
  float WL_snake;   // W/L for "snake"
  // DAC parameters.
  float dac_zero;
  float dac_scale;
  float dac_2R_div_R;
  bool dac_term;
} model_filter_init_t;

static model_filter_init_t model_filter_init[2] = {
  {
    opamp_voltage_6581,
    sizeof(opamp_voltage_6581)/sizeof(*opamp_voltage_6581),
    // The dynamic analog range of one voice is approximately 1.5V,
    1.5,
    // riding at a DC level of approximately 5.0V.
    5.0,
    // Capacitor value.
    470e-12,
    // Transistor parameters.
    12.18,
    1.31,
    26.0e-3,
    1.0,
    20e-6,
    9.0/1.0,
    1.0/115,
    // DAC parameters.
    6.65,
    2.63,
    2.20,
    false
  },
  {
    opamp_voltage_8580,
    sizeof(opamp_voltage_8580)/sizeof(*opamp_voltage_8580),
    // FIXME: Measure for the 8580.
    0.3,
    // The 4.75V voltage for the virtual ground is generated by a PolySi resistor divider
    4.75,
    // Capacitor value.
    22e-9,
    // Transistor parameters.
    9.09,
    0.80,
    26.0e-3,
    1.0,
    10e-6,
    // FIXME: 6581 only
    0,
    0,
    0,
    0,
    2.00,
    true
  }
};

unsigned short Filter::resonance[16][(1 << 16)/SUB_SAMPLE_PRE];
unsigned short Filter::vcr_kVg[(1 << 16)/SUB_SAMPLE_PRE];
unsigned short Filter::vcr_n_Ids_term[(1 << 16)/SUB_SAMPLE_PRE];
int Filter::n_snake;
int Filter::n_param;

//#if defined(__amiga__) && defined(__mc68000__)
//#undef HAS_LOG1P
//#endif

//#ifndef HAS_LOG1P
static float log1p_(float x)
{
    return log(1 + x) - (((1 + x) - 1) - x) / (1 + x);
}
//#endif

Filter::model_filter_t Filter::model_filter[2];

// ----------------------------------------------------------------------------
// Destructor.
// ----------------------------------------------------------------------------
Filter::~Filter()
{
}

// ----------------------------------------------------------------------------
// Constructor.
// ----------------------------------------------------------------------------
Filter::Filter()
{
  //static bool class_init = 0;

  //if (!class_init) 
  {
    float tmp_n_param[2];

    // Temporary tables for op-amp transfer function.
    unsigned int voltages[1 << 16];
    opamp_t opamp[(1 << 16)/SUB_SAMPLE_PRE];

    for (int m = 0; m < 2; m++) {
      model_filter_init_t& fi = model_filter_init[m];
      model_filter_t& mf = model_filter[m];

      // Convert op-amp voltage transfer to 16 bit values.
      float vmin = fi.opamp_voltage[0][0];
      float opamp_max = fi.opamp_voltage[0][1];
      float kVddt = fi.k*(fi.Vdd - fi.Vth);
      float vmax = kVddt < opamp_max ? opamp_max : kVddt;
      float denorm = vmax - vmin;
      float norm = 1.0/denorm;

      // Scaling and translation constants.
      float N16 = norm*((1u << 16) - 1);
      float N30 = norm*((1u << 30) - 1);
      float N31 = norm*((1u << 31) - 1);
      mf.vo_N16 = N16;

      // The "zero" output level of the voices.
      // The digital range of one voice is 20 bits; create a scaling term
      // for multiplication which fits in 11 bits.
      float N14 = norm*(1u << 14);
      mf.voice_scale_s14 = (int)(N14*fi.voice_voltage_range);
      mf.voice_DC = (int)(N16*(fi.voice_DC_voltage - vmin));

      // Vdd - Vth, normalized so that translated values can be subtracted:
      // k*Vddt - x = (k*Vddt - t) - (x - t)
      mf.kVddt = (int)(N16*(kVddt - vmin) + 0.5);

      tmp_n_param[m] = denorm*(1 << 13)*(fi.uCox/(2*fi.k)*1.0e-6/fi.C);

      // Create lookup table mapping op-amp voltage across output and input
      // to input voltage: vo - vx -> vx
      // FIXME: No variable length arrays in ISO C++, hardcoding to max 50
      // points.
      // double_point scaled_voltage[fi.opamp_voltage_size];
    #if 1
      double_point scaled_voltage[50];

      for (int i = 0; i < fi.opamp_voltage_size; i++) {
        // The target output range is 16 bits, in order to fit in an unsigned
        // short.
        //
        // The y axis is temporarily scaled to 31 bits for maximum accuracy in
        // the calculated derivative.
        //
        // Values are normalized using
        //
        //   x_n = m*2^N*(x - xmin)
        //
        // and are translated back later (for fixed point math) using
        //
        //   m*2^N*x = x_n - m*2^N*xmin
        //
        scaled_voltage[fi.opamp_voltage_size - 1 - i][0] = int((N16*(fi.opamp_voltage[i][1] - fi.opamp_voltage[i][0]) + (1 << 16))/2 + 0.5);
        scaled_voltage[fi.opamp_voltage_size - 1 - i][1] = N31*(fi.opamp_voltage[i][0] - vmin);
      }

      // Clamp x to 16 bits (rounding may cause overflow).
      if (scaled_voltage[fi.opamp_voltage_size - 1][0] >= (1 << 16)) {
        // The last point is repeated.
        scaled_voltage[fi.opamp_voltage_size - 1][0] =
            scaled_voltage[fi.opamp_voltage_size - 2][0] = (1 << 16) - 1;
      }

      interpolate(scaled_voltage, scaled_voltage + fi.opamp_voltage_size - 1,
                    PointPlotter<unsigned int>(voltages), 1.0);

      // Store both fn and dfn in the same table.
      mf.ak = (int)scaled_voltage[0][0];
      mf.bk = (int)scaled_voltage[fi.opamp_voltage_size - 1][0];
      int j;
      for (j = 0; j < mf.ak; j+=SUB_SAMPLE_PRE) {
        opamp[j/SUB_SAMPLE_PRE].vx = 0;
        opamp[j/SUB_SAMPLE_PRE].dvx = 0;
      }

      unsigned int f = voltages[j];
      for (; j <= mf.bk; j+=SUB_SAMPLE_PRE) {
        unsigned int fp = f;
        f = voltages[j];  // Scaled by m*2^31
        // m*2^31*dy/1 = (m*2^31*dy)/(m*2^16*dx) = 2^15*dy/dx
        int df = f - fp;  // Scaled by 2^15

        // 16 bits unsigned: m*2^16*(fn - xmin)
        opamp[j/SUB_SAMPLE_PRE].vx = f > (0xffff << 15) ? 0xffff : f >> 15;
        // 16 bits (15 bits + sign bit): 2^11*dfn
        opamp[j/SUB_SAMPLE_PRE].dvx = df >> (15 - 11);
      }
      for (; j < (1 << 16); j+=SUB_SAMPLE_PRE) {
        opamp[j/SUB_SAMPLE_PRE].vx = 0;
        opamp[j/SUB_SAMPLE_PRE].dvx = 0;
      }

      // We don't have the differential for the first point so just assume
      // it's the same as the second point's
      // TODO: check
      opamp[mf.ak/SUB_SAMPLE_PRE].dvx = opamp[mf.ak/SUB_SAMPLE_PRE+1].dvx;
    #endif

/*for ( int g = 0; g < 256; g++ )
{
  if ( m == 0 )
  {
    opamp[g].vx = opamp0[ g ][ 0 ]; 
    opamp[g].dvx = opamp0[ g ][ 1 ];
  } else
  {
    opamp[g].vx = opamp1[ g ][ 0 ]; 
    opamp[g].dvx = opamp1[ g ][ 1 ];
  }
}*/


      // Create lookup tables for gains / summers.

      // 4 bit "resistor" ladders in the bandpass resonance gain and the audio
      // output gain necessitate 16 gain tables.
      // From die photographs of the bandpass and volume "resistor" ladders
      // it follows that gain ~ vol/8 and 1/Q ~ ~res/8 (assuming ideal
      // op-amps and ideal "resistors").
      for (int n8 = 0; n8 < 16; n8++) {
        int n = n8 << 4;  // Scaled by 2^7
        int x = mf.ak;
        for (int vi = 0; vi < (1 << 16); vi+=SUB_SAMPLE_PRE) {
          mf.gain[n8][vi/SUB_SAMPLE_PRE] = solve_gain(opamp, n, vi, x, mf);
        }
      }

      //mf.summer = new unsigned short[ 1310720 ];
      //mf.mixer = new unsigned short[ 1835009 ];

      // The filter summer operates at n ~ 1, and has 5 fundamentally different
      // input configurations (2 - 6 input "resistors").
      //
      // Note that all "on" transistors are modeled as one. This is not
      // entirely accurate, since the input for each transistor is different,
      // and transistors are not linear components. However modeling all
      // transistors separately would be extremely costly.
      int offset = 0;
      int size;
      for (int k = 0; k < 5; k++) {
        int idiv = 2 + k;        // 2 - 6 input "resistors".
        int n_idiv = idiv << 7;  // n*idiv, scaled by 2^7
        size = idiv << 16;
        int x = mf.ak;
        for (int vi = 0; vi < size; vi+=SUB_SAMPLE_PRE) {
          mf.summer[(offset + vi)/SUB_SAMPLE_PRE] =
            solve_gain(opamp, n_idiv, vi/idiv, x, mf);
        }
        offset += size;
      }

      // The audio mixer operates at n ~ 8/6, and has 8 fundamentally different
      // input configurations (0 - 7 input "resistors").
      //
      // All "on", transistors are modeled as one - see comments above for
      // the filter summer.
      offset = 0;
      size = 1;  // Only one lookup element for 0 input "resistors".
      for (int l = 0; l < 8; l++) {
        int idiv = l;                 // 0 - 7 input "resistors".
        int n_idiv = (idiv << 7)*8/6; // n*idiv, scaled by 2^7
        if (idiv == 0) {
          // Avoid division by zero; the result will be correct since
          // n_idiv = 0.
          idiv = 1;
        }
        int x = mf.ak;
        for (int vi = 0; vi < size; vi+=SUB_SAMPLE_PRE) 
        //if ( (offset + vi) < 1835009 )
        {
          mf.mixer[(offset + vi)/SUB_SAMPLE_PRE] = 
            solve_gain(opamp, n_idiv, vi/idiv, x, mf);
        }
        offset += size;
        size = (l + 1) << 16;
      }

      // Create lookup table mapping capacitor voltage to op-amp input voltage:
      // vc -> vx
      for (int m = 0; m < (1 << 16); m+=SUB_SAMPLE_PRE) {
        mf.opamp_rev[m/SUB_SAMPLE_PRE] = opamp[m/SUB_SAMPLE_PRE].vx;
      }

      mf.vc_max = (int)(N30*(fi.opamp_voltage[0][1] - fi.opamp_voltage[0][0]));
      mf.vc_min = (int)(N30*(fi.opamp_voltage[fi.opamp_voltage_size - 1][1] - fi.opamp_voltage[fi.opamp_voltage_size - 1][0]));
    }

    // Free temporary table.
    //delete[] voltages;

    unsigned int dac_bits = 11;

    {
      // 8580 only
      for (int n8 = 0; n8 < 16; n8++) {
        int x = model_filter[1].ak;
        for (int vi = 0; vi < (1 << 16); vi+=SUB_SAMPLE_PRE) {
          resonance[n8][vi/SUB_SAMPLE_PRE] = solve_gain(opamp, resGain[n8], vi, x, model_filter[1]);
        }
      }

      // scaled 5 bits
      n_param = (int)(tmp_n_param[1] * 32 + 0.5);

      model_filter_init_t& fi = model_filter_init[1];
      model_filter_t& f = model_filter[1];

      float Vgt = fi.k * ((4.75 * 1.6) - fi.Vth);
      kVgt = (int)(f.vo_N16 * (Vgt - fi.opamp_voltage[0][0]) + 0.5);

      // DAC table.
      // W/L ratio for frequency DAC, bits are proportional.
      // scaled 5 bits
      unsigned short dacWL = 1; // 0.03125 * 32 FIXME actual value is ~= 0.003075
      f.f0_dac[0] = dacWL;
      for (int n = 1; n < (1 << dac_bits); n++) {
        // Calculate W/L ratio for parallel NMOS resistances
        unsigned short wl = 0;
        for (unsigned int i = 0; i < dac_bits; i++) {
          unsigned int bitmask = 1 << i;
          if (n & bitmask) {
            wl += dacWL * (bitmask<<1);
          }
        }
        f.f0_dac[n] = wl;
      }
    }

    // Free temporary table.
    //delete[] opamp;

    {
      // 6581 only
      model_filter_init_t& fi = model_filter_init[0];
      model_filter_t& f = model_filter[0];
      float N16 = f.vo_N16;
      float vmin = fi.opamp_voltage[0][0];

      Vw_bias = 0;

      // Normalized snake current factor, 1 cycle at 1MHz.
      // Fit in 5 bits.
      n_snake = (int)(fi.WL_snake * tmp_n_param[0] + 0.5);

      // DAC table.
      build_dac_table(f.f0_dac, dac_bits, fi.dac_2R_div_R, fi.dac_term);
      for (int n = 0; n < (1 << dac_bits); n++) {
        f.f0_dac[n] = (unsigned short)(N16*(fi.dac_zero + f.f0_dac[n]*fi.dac_scale/(1 << dac_bits) - vmin) + 0.5);
      }

      // VCR table.
      float k = fi.k;
      float kVddt = N16*(k*(fi.Vdd - fi.Vth));
      vmin *= N16;

      for (int i = 0; i < (1 << 16); i+=SUB_SAMPLE_PRE) {
        // The table index is right-shifted 16 times in order to fit in
        // 16 bits; the argument to sqrt is thus multiplied by (1 << 16).
        //
        // The returned value must be corrected for translation. Vg always
        // takes part in a subtraction as follows:
        //
        //   k*Vg - Vx = (k*Vg - t) - (Vx - t)
        //
        // I.e. k*Vg - t must be returned.
        float Vg = kVddt - sqrt((float)i*(1 << 16));
        vcr_kVg[i/SUB_SAMPLE_PRE] = (unsigned short)(k*Vg - vmin + 0.5);
      }

      /*
        EKV model:

        Ids = Is*(if - ir)
        Is = 2*u*Cox*Ut^2/k*W/L
        if = ln^2(1 + e^((k*(Vg - Vt) - Vs)/(2*Ut))
        ir = ln^2(1 + e^((k*(Vg - Vt) - Vd)/(2*Ut))
      */
      float kVt = fi.k*fi.Vth;
      float Ut = fi.Ut;
      float Is = 2*fi.uCox*Ut*Ut/fi.k*fi.WL_vcr;
      // Normalized current factor for 1 cycle at 1MHz.
      float N15 = N16/2;
      float n_Is = N15*1.0e-6/fi.C*Is;

      // kVg_Vx = k*Vg - Vx
      // I.e. if k != 1.0, Vg must be scaled accordingly.
      for (int kVg_Vx = 0; kVg_Vx < (1 << 16); kVg_Vx+=SUB_SAMPLE_PRE) {
        float log_term = log1p_(exp((kVg_Vx/N16 - kVt)/(2*Ut)));
        // Scaled by m*2^15
        vcr_n_Ids_term[kVg_Vx/SUB_SAMPLE_PRE] = (unsigned short)(n_Is*log_term*log_term);
      }
    }

    //class_init = true;
  }

  enable_filter(true);
  set_chip_model(MOS6581);
  set_voice_mask(0x07);
  input(0);
  reset();
}


// ----------------------------------------------------------------------------
// Enable filter.
// ----------------------------------------------------------------------------
void Filter::enable_filter(bool enable)
{
  enabled = enable;
  set_sum_mix();
}


// ----------------------------------------------------------------------------
// Adjust the DAC bias parameter of the filter.
// This gives user variable control of the exact CF -> center frequency
// mapping used by the filter.
// ----------------------------------------------------------------------------
void Filter::adjust_filter_bias(float dac_bias)
{
  Vw_bias = int(dac_bias*model_filter[0].vo_N16);
  set_w0();

  // Gate voltage is controlled by the switched capacitor voltage divider
  // Ua = Ue * v = 4.75v  1<v<2
  float Vg = 4.75 * (dac_bias*6./100. + 1.6);
  float Vgt = model_filter_init[1].k * (Vg - model_filter_init[1].Vth);
  float vmin = model_filter_init[1].opamp_voltage[0][0];

  // Vg - Vth, normalized so that translated values can be subtracted:
  // k*Vgt - x = (k*Vgt - t) - (x - t)
  kVgt = (int)(model_filter[1].vo_N16 * (Vgt - vmin) + 0.5);
}

// ----------------------------------------------------------------------------
// Set chip model.
// ----------------------------------------------------------------------------
void Filter::set_chip_model(chip_model model)
{
  sid_model = model;
  /* We initialize the state variables again just to make sure that
   * the earlier model didn't leave behind some foreign, unrecoverable
   * state. Hopefully set_chip_model() only occurs simultaneously with
   * reset(). */
  Vhp = 0;
  Vbp = Vbp_x = Vbp_vc = 0;
  Vlp = Vlp_x = Vlp_vc = 0;
}


// ----------------------------------------------------------------------------
// Mask for voices routed into the filter / audio output stage.
// Used to physically connect/disconnect EXT IN, and for test purposes
// (voice muting).
// ----------------------------------------------------------------------------
void Filter::set_voice_mask(reg4 mask)
{
  voice_mask = 0xf0 | (mask & 0x0f);
  set_sum_mix();
}


// ----------------------------------------------------------------------------
// SID reset.
// ----------------------------------------------------------------------------
void Filter::reset()
{
  fc = 0;
  res = 0;
  filt = 0;
  mode = 0;
  vol = 0;

  Vhp = 0;
  Vbp = Vbp_x = Vbp_vc = 0;
  Vlp = Vlp_x = Vlp_vc = 0;

  set_w0();
  set_Q();
  set_sum_mix();
}


// ----------------------------------------------------------------------------
// Register functions.
// ----------------------------------------------------------------------------
void Filter::writeFC_LO(reg8 fc_lo)
{
  fc = (fc & 0x7f8) | (fc_lo & 0x007);
  set_w0();
}

void Filter::writeFC_HI(reg8 fc_hi)
{
  fc = ((fc_hi << 3) & 0x7f8) | (fc & 0x007);
  set_w0();
}

void Filter::writeRES_FILT(reg8 res_filt)
{
  res = (res_filt >> 4) & 0x0f;
  set_Q();

  filt = res_filt & 0x0f;
  set_sum_mix();
}

void Filter::writeMODE_VOL(reg8 mode_vol)
{
  mode = mode_vol & 0xf0;
  set_sum_mix();

  vol = mode_vol & 0x0f;
}

// Set filter cutoff frequency.
void Filter::set_w0()
{
  {
    // MOS 6581
    model_filter_t& f = model_filter[0];
    int Vw = Vw_bias + f.f0_dac[fc];
    Vddt_Vw_2 = unsigned(f.kVddt - Vw)*unsigned(f.kVddt - Vw) >> 1;
  }

  {
    // MOS 8580 cutoff: 0 - 12.5kHz.
    model_filter_t& f = model_filter[1];
    n_dac = (n_param * f.f0_dac[fc]) >> 10;
  }
}

/*
Set filter resonance.

In the MOS 6581, 1/Q is controlled linearly by res. From die photographs
of the resonance "resistor" ladder it follows that 1/Q ~ ~res/8
(assuming an ideal op-amp and ideal "resistors"). This implies that Q
ranges from 0.533 (res = 0) to 8 (res = E). For res = F, Q is actually
theoretically unlimited, which is quite unheard of in a filter
circuit.

To obtain Q ~ 1/sqrt(2) = 0.707 for maximally flat frequency response,
res should be set to 4: Q = 8/~4 = 8/11 = 0.7272 (again assuming an ideal
op-amp and ideal "resistors").

Q as low as 0.707 is not achievable because of low gain op-amps; res = 0
should yield the flattest possible frequency response at Q ~ 0.8 - 1.0
in the op-amp's pseudo-linear range (high amplitude signals will be
clipped). As resonance is increased, the filter must be clocked more
often to keep it stable.

In the MOS 8580, the resonance "resistor" ladder above the bp feedback
op-amp is split in two parts; one ladder for the op-amp input and one
ladder for the op-amp feedback.

input:         feedback:

               Rf
Ri R4 RC R8    R3
               R2
               R1


The "resistors" are switched in as follows by bits in register $17:

feedback:
R1: bit4&!bit5
R2: !bit4&bit5
R3: bit4&bit5
Rf: always on

input:
R4: bit6&!bit7
R8: !bit6&bit7
RC: bit6&bit7
Ri: !(R4|R8|RC) = !(bit6|bit7) = !bit6&!bit7


The relative "resistor" values are approximately (using channel length):

R1 = 15.3*Ri
R2 =  7.3*Ri
R3 =  4.7*Ri
Rf =  1.4*Ri
R4 =  1.4*Ri
R8 =  2.0*Ri
RC =  2.8*Ri


Approximate values for 1/Q can now be found as follows (assuming an
ideal op-amp):

res  feedback  input  -gain (1/Q)
---  --------  -----  ----------
 0   Rf        Ri     Rf/Ri      = 1/(Ri*(1/Rf))      = 1/0.71
 1   Rf|R1     Ri     (Rf|R1)/Ri = 1/(Ri*(1/Rf+1/R1)) = 1/0.78
 2   Rf|R2     Ri     (Rf|R2)/Ri = 1/(Ri*(1/Rf+1/R2)) = 1/0.85
 3   Rf|R3     Ri     (Rf|R3)/Ri = 1/(Ri*(1/Rf+1/R3)) = 1/0.92
 4   Rf        R4     Rf/R4      = 1/(R4*(1/Rf))      = 1/1.00
 5   Rf|R1     R4     (Rf|R1)/R4 = 1/(R4*(1/Rf+1/R1)) = 1/1.10
 6   Rf|R2     R4     (Rf|R2)/R4 = 1/(R4*(1/Rf+1/R2)) = 1/1.20
 7   Rf|R3     R4     (Rf|R3)/R4 = 1/(R4*(1/Rf+1/R3)) = 1/1.30
 8   Rf        R8     Rf/R8      = 1/(R8*(1/Rf))      = 1/1.43
 9   Rf|R1     R8     (Rf|R1)/R8 = 1/(R8*(1/Rf+1/R1)) = 1/1.56
 A   Rf|R2     R8     (Rf|R2)/R8 = 1/(R8*(1/Rf+1/R2)) = 1/1.70
 B   Rf|R3     R8     (Rf|R3)/R8 = 1/(R8*(1/Rf+1/R3)) = 1/1.86
 C   Rf        RC     Rf/RC      = 1/(RC*(1/Rf))      = 1/2.00
 D   Rf|R1     RC     (Rf|R1)/RC = 1/(RC*(1/Rf+1/R1)) = 1/2.18
 E   Rf|R2     RC     (Rf|R2)/RC = 1/(RC*(1/Rf+1/R2)) = 1/2.38
 F   Rf|R3     RC     (Rf|R3)/RC = 1/(RC*(1/Rf+1/R3)) = 1/2.60


These data indicate that the following function for 1/Q has been
modeled in the MOS 8580:

  1/Q = 2^(1/2)*2^(-x/8) = 2^(1/2 - x/8) = 2^((4 - x)/8)

*/
void Filter::set_Q()
{
  // Cutoff for MOS 6581.
  // The coefficient 8 is dispensed of later by right-shifting 3 times
  // (2 ^ 3 = 8).
  _8_div_Q = ~res & 0x0f;
}

// Set input routing bits.
void Filter::set_sum_mix()
{
  // NB! voice3off (mode bit 7) only affects voice 3 if it is routed directly
  // to the mixer.
  sum = (enabled ? filt : 0x00) & voice_mask;
  mix =
    (enabled ? (mode & 0x70) | ((~(filt | (mode & 0x80) >> 5)) & 0x0f) : 0x0f)
    & voice_mask;
}

}
