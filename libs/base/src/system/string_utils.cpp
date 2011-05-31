/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/base.h>  // Precompiled headers

#include <mrpt/system/string_utils.h>
#include <mrpt/synch.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace std;

/*---------------------------------------------------------------
						lowerCase
  ---------------------------------------------------------------*/
string  mrpt::system::lowerCase(const string& str)
{
	string outStr( str );

	transform(
		outStr.begin(), outStr.end(),		// In
		outStr.begin(),			// Out
		(int(*)(int)) tolower );
	return outStr;
}

/*---------------------------------------------------------------
						upperCase
  ---------------------------------------------------------------*/
string  mrpt::system::upperCase(const string& str)
{
	string outStr( str );
	transform(
		outStr.begin(), outStr.end(),		// In
		outStr.begin(),			// Out
		(int(*)(int)) toupper );
	return outStr;
}

/*---------------------------------------------------------------
					encodeUTF8

    Author: Marius Bancila
    http://www.codeguru.com/cpp/misc/misc/multi-lingualsupport/article.php/c10451/
---------------------------------------------------------------*/
#define         MASKBITS                0x3F
#define         MASKBYTE                0x80
#define         MASK2BYTES              0xC0
#define         MASK3BYTES              0xE0
#define         MASK4BYTES              0xF0
#define         MASK5BYTES              0xF8
#define         MASK6BYTES              0xFC

void mrpt::system::encodeUTF8( const vector_word &input, std::string &output )
{
    output = ""; // output.clear();  VC6...
    output.reserve( input.size() );
    for(size_t i=0; i < input.size(); i++)
    {
        // 0xxxxxxx
        if(input[i] < 0x80)
        {
            output += (char)input[i];
        }
        // 110xxxxx 10xxxxxx
        else if(input[i] < 0x800)
        {
            output += (char)(MASK2BYTES | input[i] >> 6);
            output += (char)(MASKBYTE | (input[i] & MASKBITS) );
        }
        // 1110xxxx 10xxxxxx 10xxxxxx
        /*else if(input[i] < 0x10000)
        {
            output.push_back((char)(MASK3BYTES | input[i] >> 12));
            output.push_back((char)(MASKBYTE | input[i] >> 6 & MASKBITS));
            output.push_back((char)(MASKBYTE | input[i] & MASKBITS));
        }*/
    }
}

/*---------------------------------------------------------------
					decodeUTF8

    Author: Marius Bancila
    http://www.codeguru.com/cpp/misc/misc/multi-lingualsupport/article.php/c10451/
---------------------------------------------------------------*/
void mrpt::system::decodeUTF8( const std::string &input, vector_word &output )
{
    output.clear();
    output.reserve( input.size() );
    for(size_t i=0; i < input.size();)
    {
        uint16_t ch;

        // 1110xxxx 10xxxxxx 10xxxxxx
        if((input[i] & MASK3BYTES) == MASK3BYTES)
        {
            ch = ((input[i] & 0x0F) << 12) | (
            (input[i+1] & MASKBITS) << 6)
            | (input[i+2] & MASKBITS);
            i += 3;
        }
        // 110xxxxx 10xxxxxx
        else if((input[i] & MASK2BYTES) == MASK2BYTES)
        {
            ch = ((input[i] & 0x1F) << 6) | (input[i+1] & MASKBITS);
            i += 2;
        }
        // 0xxxxxxx
        else if( uint8_t(input[i]) < MASKBYTE)
        {
            ch = input[i];
            i += 1;
        }
        output.push_back(ch);
    }
}

/** This function implements formatting with the appropriate SI metric unit prefix:
  1e-12->'p', 1e-9->'n', 1e-6->'u', 1e-3->'m',
  1->'',
  1e3->'K', 1e6->'M', 1e9->'G', 1e12->'T'
 */
std::string
mrpt::system::unitsFormat(const double val,int nDecimalDigits, bool middle_space)
{
	char	prefix;
	double	mult;

	if (val>=1e12)
		{mult=1e-12; prefix='T';}
	else if (val>=1e9)
		{mult=1e-9; prefix='G';}
	else if (val>=1e6)
		{mult=1e-6; prefix='M';}
	else if (val>=1e3)
		{mult=1e-3; prefix='K';}
	else if (val>=1)
		{mult=1; prefix=' ';}
	else if (val>=1e-3)
		{mult=1e+3; prefix='m';}
	else if (val>=1e-6)
		{mult=1e+6; prefix='u';}
	else if (val>=1e-9)
		{mult=1e+9; prefix='n';}
	else
		{mult=1e+12; prefix='p';}

	return format(
		middle_space ? "%.*f %c" : "%.*f%c",
		nDecimalDigits,
		val*mult,
		prefix );
}

/*---------------------------------------------------------------
						strtok
---------------------------------------------------------------*/
char *mrpt::system::strtok( char *str, const char *strDelimit, char **context ) MRPT_NO_THROWS
{
#if defined(_MSC_VER) && (_MSC_VER>=1400)
	// Use a secure version in Visual Studio 2005:
	return ::strtok_s(str,strDelimit,context);
#else
	// Use standard version:
	return ::strtok(str,strDelimit);
#endif
}

/*---------------------------------------------------------------
						tokenize
---------------------------------------------------------------*/
void  mrpt::system::tokenize(
	const std::string			&inString,
	const std::string			&inDelimiters,
	std::deque<std::string>	&outTokens ) MRPT_NO_THROWS
{
    static mrpt::synch::CCriticalSection cs;
    mrpt::synch::CCriticalSectionLocker lock(&cs);

	char	*nextTok,*context;

	outTokens.clear();

	char *dupStr = ::strdup(inString.c_str());

	nextTok = strtok (dupStr,inDelimiters.c_str(),&context);
	while (nextTok != NULL)
	{
		outTokens.push_back( std::string(nextTok) );
		nextTok = strtok (NULL,inDelimiters.c_str(),&context);
	};

	free(dupStr);
}

/*---------------------------------------------------------------
						tokenize
---------------------------------------------------------------*/
void  mrpt::system::tokenize(
	const std::string			&inString,
	const std::string			&inDelimiters,
	std::vector<std::string>	&outTokens ) MRPT_NO_THROWS
{
    static mrpt::synch::CCriticalSection cs;
    mrpt::synch::CCriticalSectionLocker lock(&cs);

	char	*nextTok,*context;

	outTokens.clear();
	char *dupStr = ::strdup(inString.c_str());

	nextTok = strtok (dupStr,inDelimiters.c_str(),&context);
	while (nextTok != NULL)
	{
		outTokens.push_back( std::string(nextTok) );
		nextTok = strtok (NULL,inDelimiters.c_str(),&context);
	};

	free(dupStr);
}

/*---------------------------------------------------------------
						trim
---------------------------------------------------------------*/
std::string mrpt::system::trim(const std::string &str)
{
	if (str.empty())
	{
		return std::string();
	}
	else
	{
		size_t s = str.find_first_not_of(" \t");
		size_t e = str.find_last_not_of(" \t");
		if (s==std::string::npos || e==std::string::npos)
				return std::string();
		else	return str.substr( s, e-s+1);
	}
}

/*---------------------------------------------------------------
						rightPad
---------------------------------------------------------------*/
std::string mrpt::system::rightPad(const std::string &str, const size_t total_len, bool truncate_if_larger)
{
	std::string r = str;
	if (r.size()<total_len || truncate_if_larger)
		r.resize(total_len,' ');
	return r;
}

/** Return true if the two strings are equal (case sensitive)  \sa StrCmpI  */
bool mrpt::system::strCmp(const std::string &s1, const std::string &s2)
{
	return !mrpt::system::os::_strcmp(s1.c_str(),s2.c_str());
}

/** Return true if the two strings are equal (case insensitive)  \sa StrCmp */
bool mrpt::system::strCmpI(const std::string &s1, const std::string &s2)
{
	return !mrpt::system::os::_strcmpi(s1.c_str(),s2.c_str());
}

/** Return true if "str" starts with "subStr" (case sensitive)  \sa strStartsI  */
bool mrpt::system::strStarts(const std::string &s1, const std::string &s2)
{
	return !mrpt::system::os::_strncmp(s1.c_str(),s2.c_str(),s2.size()); // if s1 is shorter it's not a problem
}

/** Return true if "str" starts with "subStr" (case insensitive)  \sa strStarts */
bool mrpt::system::strStartsI(const std::string &s1, const std::string &s2)
{
	return !mrpt::system::os::_strnicmp(s1.c_str(),s2.c_str(),s2.size()); // if s1 is shorter it's not a problem
}
