/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <mrpt/core/common.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/round.h>  // round()
#include <mrpt/system/string_utils.h>

#include <iostream>

using namespace mrpt::system;
using namespace std;

// This code is based on files in the public domain:
//  http://gd.tuwien.ac.at/infosys/mail/vm

const unsigned char alphabet[64 + 1] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

/*---------------------------------------------------------------
          encodeBase64
---------------------------------------------------------------*/
void mrpt::system::encodeBase64(const std::vector<uint8_t>& inputData, std::string& outString)
{
  outString.clear();
  outString.reserve(inputData.size() * static_cast<std::size_t>(mrpt::round(4.0 / 3.0)));

  const auto addChar = [&](const unsigned char c) { outString.push_back(static_cast<char>(c)); };

  int char_count = 0;
  int bits = 0;
  int cols = 0;

  for (unsigned char c : inputData)
  {
    bits += c;
    char_count++;

    if (char_count == 3)
    {
      addChar(alphabet[bits >> 18]);
      addChar(alphabet[(bits >> 12) & 0x3f]);
      addChar(alphabet[(bits >> 6) & 0x3f]);
      addChar(alphabet[bits & 0x3f]);
      cols += 4;
      if (cols == 72)
      {
        addChar('\n');
        cols = 0;
      }
      bits = 0;
      char_count = 0;
    }
    else
    {
      bits <<= 8;
    }
  }

  if (char_count != 0)
  {
    bits <<= 16 - (8 * char_count);
    addChar(alphabet[bits >> 18]);
    addChar(alphabet[(bits >> 12) & 0x3f]);

    if (char_count == 1)
    {
      addChar('=');
      addChar('=');
    }
    else
    {
      addChar(alphabet[(bits >> 6) & 0x3f]);
      addChar('=');
    }
    if (cols > 0)
    {
      addChar('\n');
    }
  }
}

/*---------------------------------------------------------------
          decodeBase64
---------------------------------------------------------------*/
bool mrpt::system::decodeBase64(const std::string& inString, std::vector<uint8_t>& outData)
{
  static bool in_alphabet[256];
  static char decoder[256];

  static bool tablesBuilt = false;

  if (!tablesBuilt)
  {
    tablesBuilt = true;
    for (int i = (sizeof(alphabet)) - 1; i >= 0; i--)
    {
      in_alphabet[alphabet[i]] = true;
      decoder[alphabet[i]] = static_cast<char>(i);
    }
  }

  outData.clear();
  outData.reserve((inString.size() * 3) / 4);

  const auto addByte = [&](const int c) { outData.push_back(static_cast<uint8_t>(c)); };

  int errors = 0;

  int char_count = 0;
  int bits = 0;
  bool finish_flag_found = false;

  for (const auto ch : inString)
  {
    const auto c = static_cast<unsigned char>(ch);

    if (c == '=')
    {
      finish_flag_found = true;
      break;
    }
    if (!in_alphabet[c])
    {
      continue;
    }

    bits += decoder[c];
    char_count++;
    if (char_count == 4)
    {
      addByte(bits >> 16);
      addByte(((bits >> 8) & 0xff));
      addByte((bits & 0xff));
      bits = 0;
      char_count = 0;
    }
    else
    {
      bits <<= 6;
    }
  }

  if (!finish_flag_found)
  {
    if (char_count)
    {
      std::cerr << "[decodeBase64] ERROR: base64 encoding incomplete, atleast "
                << ((4 - char_count) * 6) << "bits truncated." << std::endl;
      errors++;
    }
  }
  else
  { /* c == '=' */
    switch (char_count)
    {
      case 1:
        std::cerr << "[decodeBase64] ERROR: base64 encoding incomplete, at least 2 bits missing"
                  << std::endl;
        errors++;
        break;
      case 2:
        addByte(bits >> 10);
        break;
      case 3:
        addByte(bits >> 16);
        addByte(((bits >> 8) & 0xff));
        break;
      default:
      {
        THROW_EXCEPTION("Unexpected char count in decodeBase64()");
      }
    }
  }

  return errors == 0;
}
