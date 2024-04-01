/*
   CBUS Module Library - RasberryPi Pico SDK port
   Copyright (c) Kevin Kimber 2023

   Based on work by Duncan Greenwood
   Copyright (C) Duncan Greenwood 2017 (duncan_greenwood@hotmail.com)

   This work is licensed under the:
      Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
   To view a copy of this license, visit:
      http://creativecommons.org/licenses/by-nc-sa/4.0/
   or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

   License summary:
    You are free to:
      Share, copy and redistribute the material in any medium or format
      Adapt, remix, transform, and build upon the material

    The licensor cannot revoke these freedoms as long as you follow the license terms.

    Attribution : You must give appropriate credit, provide a link to the license,
                  and indicate if changes were made. You may do so in any reasonable manner,
                  but not in any way that suggests the licensor endorses you or your use.

    NonCommercial : You may not use the material for commercial purposes. **(see note below)

    ShareAlike : If you remix, transform, or build upon the material, you must distribute
                 your contributions under the same license as the original.

    No additional restrictions : You may not apply legal terms or technological measures that
                                 legally restrict others from doing anything the license permits.

   ** For commercial use, please contact the original copyright holder(s) to agree licensing terms

    This software is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE

*/

#pragma once

// Utility macros

/// Extracts the low-order (rightmost) byte of a variable (e.g. a word).
#define lowByte(w) (static_cast<uint8_t>(((w) & 0xffU)))

/// Extracts the high-order (leftmost) byte of a word (or the second lowest byte of a larger data type).
#define highByte(w) (static_cast<uint8_t>(((w) >> 8U)))

/// Reads a bit of a variable, e.g. uint8_t, uint16_t. Note that float & double are not supported. 
/// You can read the bit of variables up to an uint64_t (64 bits / 8 bytes).
#define bitRead(value, bit) (((value) >> (bit)) & 0x01U)

/// Sets (writes a 1 to) a bit of a numeric variable.
#define bitSet(value, bit) ((value) |= (1UL << (bit)))

/// Clears (writes a 0 to) a bit of a numeric variable.
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))

/// Writes to a bit of a variable, e.g. uint8_t, uint16_t, uint32_t. Note that float & double are not supported.
// You can write to a bit of variables up to an uint32_t
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))
