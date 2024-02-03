/*
   MERG CBUS Module Library - RasberryPi Pico SDK port
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

#include "CBUSGridConnect.h"
#include <cstdio>
#include <cctype>
#include <cstdlib>

///
/// Class to encode and decode CBUS Grid Connect messages
///
CBUSGridConnect::CBUSGridConnect()
{
}

///
/// @brief Encode a CANFrame as info a GridConnect string
///
/// @param canMsg CAN Frame to encode
/// @param gcMsg Grid Connect message where result will be stored
/// @return true if the provided CANFrame was successfully encoded
/// @return false if the provided CANFrame could not be encoded into GridConnect
///
bool CBUSGridConnect::encodeGC(const CANFrame &canMsg, gcMessage_t &gcMsg)
{
   hexByteChars_t tmp;
   uint8_t offset = 0;

   gcMsg = {}; // Initialize / null terminate provided buffer

   // Check for validity of the CAN frame
   if ((canMsg.len > 8) ||                                 // Check max length
       ((canMsg.ext == false) && (canMsg.id > 0x7FF)) ||   // Check ID for standard CAN frame
       ((canMsg.ext == true) && (canMsg.id > 0x1FFFFFFF))) // Check ID for extended ID CAN frame
   {
      return false;
   }

   // Mark start of frame
   gcMsg.byte[0] = ':';

   // set starting character & standard or extended CAN identifier
   if (canMsg.ext)
   {
      // mark as extended message
      gcMsg.byte[1] = 'X';

      // extended 29 bit CAN idenfier in bytes 2 to 9

      // chars 2 & 3 are ID bits 21 to 28
      uint8ToHex((canMsg.id >> 21), tmp);
      gcMsg.byte[2] = tmp.upperNibble;
      gcMsg.byte[3] = tmp.lowerNibble;

      //  char 4 -  bits 1 to 3 are ID bits 18 to 20
      uint8ToHex((canMsg.id >> 17) & 0xE, tmp);
      gcMsg.byte[4] = tmp.lowerNibble;

      //  char 5 -  bits 0 to 1 are ID bits 16 & 17
      uint8ToHex((canMsg.id >> 16) & 0x3, tmp);
      gcMsg.byte[5] = tmp.lowerNibble;

      // chars 6 to 9 are ID bits 0 to 15
      uint8ToHex((canMsg.id & 0xff00) >> 8, tmp);
      gcMsg.byte[6] = tmp.upperNibble;
      gcMsg.byte[7] = tmp.lowerNibble;

      uint8ToHex((canMsg.id & 0x00ff), tmp);
      gcMsg.byte[8] = tmp.upperNibble;
      gcMsg.byte[9] = tmp.lowerNibble;

      offset = 10; // ':X' + 8
   }
   else
   {
      // mark as standard message
      gcMsg.byte[1] = 'S';

      // standard 11 bit CAN idenfier in bytes 2 to 5, left shifted 5 to occupy highest bits

      uint16_t canId = (canMsg.id << 5);

      uint8ToHex((canId & 0xff00) >> 8, tmp);
      gcMsg.byte[2] = tmp.upperNibble;
      gcMsg.byte[3] = tmp.lowerNibble;

      uint8ToHex((canId & 0x00ff), tmp);
      gcMsg.byte[4] = tmp.upperNibble;
      gcMsg.byte[5] = tmp.lowerNibble;

      offset = 6; // ':S' + 4
   }

   // set RTR or normal, in byte 6 or 10
   gcMsg.byte[offset++] = canMsg.rtr ? 'R' : 'N';

   // now add hex data from byte 7 or 11, if len > 0
   for (int_fast8_t i = 0; i < canMsg.len; i++)
   {
      // convert data byte to hex
      uint8ToHex(canMsg.data[i], tmp);
      gcMsg.byte[offset++] = tmp.upperNibble;
      gcMsg.byte[offset++] = tmp.lowerNibble;
   }

   // finally add terminator
   gcMsg.byte[offset++] = ';';

   // set length of resulting Grid Connect message
   gcMsg.len = offset;

   return true;
}

///
/// @brief Decode a Grid Connect message into a CANFrame
///
/// @param gcMsg Reference to Grid Connect message to decode
/// @param canMsg Reference to a CANFrame where decoded frame is written
/// @return true if the provided Grid Connect message was successfully decoded
/// @return false if the provided Grid Connect message could not be decoded
///
bool CBUSGridConnect::decodeGC(const gcMessage_t &gcMsg, CANFrame &canMsg)
{
   hexByteChars_t tmp;
   uint8_t offset = 0;

   // Sanity check overal length first
   if (gcMsg.len > GC_MAX_MSG)
   {
      return false;
   }

   // must start with message start indicator
   if (gcMsg.byte[offset++] != ':')
   {
      return false;
   }

   // CAN Identifier, must be either 'X' or 'S'
   if (gcMsg.byte[offset] == 'X')
   {
      // Extended ID CAN Frame
      canMsg.ext = true;

      // now get 29 bit ID - convert from hex, but check they are all hex first
      if (checkHexChars(gcMsg, 2, 8) == false)
      {
         return false;
      }

      // all chars are hex, so build up id from characters 2 to 9
      // chars 2 & 3 are bits 21 to 28
      tmp.upperNibble = gcMsg.byte[2];
      tmp.lowerNibble = gcMsg.byte[3];
      canMsg.id = static_cast<uint32_t>(hexToUint8(tmp)) << 21;

      // chars 4 & 5 -  bits 5 to 7 are bits 18 to 20
      tmp.upperNibble = gcMsg.byte[4];
      tmp.lowerNibble = gcMsg.byte[5];
      canMsg.id += static_cast<uint32_t>(hexToUint8(tmp) & 0xE0) << 13;

      // chars 4 & 5 -  bits 0 to 1 are bits 16 & 17
      tmp.upperNibble = gcMsg.byte[4];
      tmp.lowerNibble = gcMsg.byte[5];
      canMsg.id += static_cast<uint32_t>(hexToUint8(tmp) & 0x3) << 16;

      // chars 6 & 7 are bits 8 to
      tmp.upperNibble = gcMsg.byte[6];
      tmp.lowerNibble = gcMsg.byte[7];
      canMsg.id += static_cast<uint32_t>(hexToUint8(tmp)) << 8;

      // chars 8 & 9 are bits 0 to 7
      tmp.upperNibble = gcMsg.byte[8];
      tmp.lowerNibble = gcMsg.byte[9];
      canMsg.id += hexToUint8(tmp);

      offset = 10; // ':X' + 8
   }
   else if (gcMsg.byte[offset] == 'S')
   {
      // Standard ID CAN Frame
      canMsg.ext = false;

      // now get 11 bit ID - convert from hex, but check they are all hex first
      if (checkHexChars(gcMsg, 2, 4) == false)
      {
         return false;
      }

      // 11 bit identifier needs to be shifted right by 5
      canMsg.id = std::strtol(&gcMsg.byte[2], nullptr, 16) >> 5;

      offset = 6; // ':S' + 4
   }
   else
   {
      // Not marked either Extended or Standard ID frame
      return false;
   }

   // do RTR flag
   if (gcMsg.byte[offset] == 'R')
   {
      canMsg.rtr = true;
   }
   else if (gcMsg.byte[offset] == 'N')
   {
      canMsg.rtr = false;
   }
   else
   {
      // Not marked either RTR or Normal frame
      return false;
   }

   offset++; // set to next character afert RTR flag

   // Convert the CAN frame data

   // first find out how many chars of data there are
   // data length should be gcMsg.len minus current offset
   // as it is now (after RTR flag), minus terminator
   int dataLength = gcMsg.len - offset - 1;

   // must be even number of hex characters, and no more than 16
   if ((dataLength % 2) || (dataLength > 16))
   {
      return false;
   }

   // set byte length of the CANFrame
   canMsg.len = dataLength / 2;

   // now convert hex data into bytes
   for (int_fast8_t i = 0; i < canMsg.len; i++)
   {
      // check they are hex chars first
      if (checkHexChars(gcMsg, offset, 2) == false)
      {
         return false;
      }

      // convert byte
      tmp.upperNibble = gcMsg.byte[offset++];
      tmp.lowerNibble = gcMsg.byte[offset++];
      canMsg.data[i] = hexToUint8(tmp);
   }

   //
   // must have end of message character
   if (gcMsg.byte[gcMsg.len - 1] != ';')
   {
      return false;
   }

   // Grid Connect message parsed successfully
   return true;
}

///
/// @brief Convert a uint8_t value to two hex characters
///
/// @param u8 value to convert
/// @param byteStr converted hex characters
///
void CBUSGridConnect::uint8ToHex(const uint8_t u8, hexByteChars_t &byteStr)
{
   constexpr char HexChars[16]{'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

   byteStr.lowerNibble = HexChars[u8 & 0x0f];
   byteStr.upperNibble = HexChars[(u8 >> 4) & 0x0f];
}

///
/// @brief Convert two hex characters to a uint8_t
///
/// @param hexChar hex characters to conver, upper and lower nibbles
/// @return converted uint8_t value
///
uint8_t CBUSGridConnect::hexToUint8(const hexByteChars_t &hexChar)
{
   uint8_t result;

   // Convert lower nibble
   if (hexChar.lowerNibble < 'A')
   {
      result = hexChar.lowerNibble - '0';
   }
   else
   {
      result = hexChar.lowerNibble - 'A' + 10;
   }

   // Convert upper nibble
   if (hexChar.upperNibble < 'A')
   {
      result += (hexChar.upperNibble - '0') << 4;
   }
   else
   {
      result += (hexChar.upperNibble - 'A' + 10) << 4;
   }

   return result;
}

///
/// @brief Check all characeters in the GC message, in the range specified, are hex
///
/// @param msg Reference to the grid connect message to check
/// @param start byte offset of the first character to check
/// @param count number of characters to check
/// @return true if the characters in range are all hex
/// @return false if one or more characters are not hex
///
bool CBUSGridConnect::checkHexChars(const gcMessage_t &msg, uint8_t start, uint8_t count)
{
   // check number of characters requested
   for (int_fast8_t i = start; i < start + count; i++)
   {
      // must be upper case, so fail if lower case
      if (islower(msg.byte[i]))
      {
         return false;
      }

      // must be hexadecimal character
      if (!isxdigit(msg.byte[i]))
      {
         return false;
      }
   }

   return true;
}
