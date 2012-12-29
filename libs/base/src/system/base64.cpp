/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */

#include <mrpt/base.h>  // Precompiled headers

#include <mrpt/system/string_utils.h>
#include <mrpt/utils.h>

using namespace mrpt::system;
using namespace mrpt::utils;
using namespace std;

// This code is based on files in the public domain:
//  http://gd.tuwien.ac.at/infosys/mail/vm

const unsigned char alphabet[64+1] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

/*---------------------------------------------------------------
					encodeBase64
---------------------------------------------------------------*/
void mrpt::system::encodeBase64( const vector_byte &inputData,  std::string &outString )
{
	outString.clear();
	outString.reserve( inputData.size() * mrpt::utils::round(4.0/3.0) );

	int char_count = 0;
	int bits = 0;
	int cols = 0;

	for (size_t i=0;i<inputData.size();i++)
	{
		const uint8_t c = inputData[i];
		bits += c;
		char_count++;

		if (char_count == 3)
		{
			outString.push_back( alphabet[bits >> 18] );
			outString.push_back( alphabet[(bits >> 12) & 0x3f] );
			outString.push_back( alphabet[(bits >> 6) & 0x3f]);
			outString.push_back( alphabet[bits & 0x3f]);
			cols += 4;
			if (cols == 72)
			{
				outString.push_back('\n');
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
		outString.push_back( alphabet[bits >> 18]);
		outString.push_back( alphabet[(bits >> 12) & 0x3f]);

		if (char_count == 1)
		{
			outString.push_back('=');
			outString.push_back('=');
		}
		else
		{
			outString.push_back( alphabet[(bits >> 6) & 0x3f]);
			outString.push_back( '=');
		}
		if (cols > 0)
			outString.push_back('\n');
	}
}

/*---------------------------------------------------------------
					decodeBase64
---------------------------------------------------------------*/
bool mrpt::system::decodeBase64( const std::string &inString, vector_byte &outData )
{
	static bool inalphabet[256];
	static char decoder[256];

	static bool tablesBuilt = false;

	if (!tablesBuilt)
	{
		tablesBuilt = true;
		for (int i = (sizeof(alphabet)) - 1; i >= 0 ; i--)
		{
			inalphabet[alphabet[i]] = 1;
			decoder[alphabet[i]] = i;
		}
	}

	outData.clear();
	outData.reserve( inString.size() * round(3.0/4.0) );

	int errors = 0;

	int char_count = 0;
	int bits = 0;
	bool finish_flag_found = false;

	for (size_t i=0;i<inString.size();i++)
	{
		const unsigned char c = inString[i];

		if (c == '=')
		{
			finish_flag_found = true;
			break;
		}
		if (!inalphabet[c])
			continue;

		bits += decoder[c];
		char_count++;
		if (char_count == 4)
		{
			outData.push_back((bits >> 16));
			outData.push_back(((bits >> 8) & 0xff));
			outData.push_back((bits & 0xff));
			bits = 0;
			char_count = 0;
		}
		else
			bits <<= 6;
	}

	if (!finish_flag_found)
	{
		if (char_count)
		{
			std::cerr << format("[decodeBase64] ERROR: base64 encoding incomplete, at least %d bits truncated", ((4 - char_count) * 6)) << std::endl;
			errors++;
		}
	}
	else
	{ /* c == '=' */
		switch (char_count)
		{
		case 1:
			std::cerr << "[decodeBase64] ERROR: base64 encoding incomplete, at least 2 bits missing" << std::endl;
			errors++;
			break;
		case 2:
			outData.push_back((bits >> 10));
			break;
		case 3:
			outData.push_back((bits >> 16));
			outData.push_back(((bits >> 8) & 0xff));
			break;
		}
	}

	return errors==0;
}

