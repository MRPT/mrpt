/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/system/string_utils.h>
#include <mrpt/system/os.h>
#include <cstring>

#ifndef HAVE_STRTOK_R
#   include <mrpt/synch/CCriticalSection.h>
#endif

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
	const double aVal = std::abs(val);

	if (aVal >=1e12)
		{mult=1e-12; prefix='T';}
	else if (aVal >=1e9)
		{mult=1e-9; prefix='G';}
	else if (aVal >=1e6)
		{mult=1e-6; prefix='M';}
	else if (aVal >=1e3)
		{mult=1e-3; prefix='K';}
	else if (aVal >=1)
		{mult=1; prefix=' ';}
	else if (aVal >=1e-3)
		{mult=1e+3; prefix='m';}
	else if (aVal >=1e-6)
		{mult=1e+6; prefix='u';}
	else if (aVal >=1e-9)
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
#ifdef HAVE_STRTOK_R
	// POSIX safe version:
	return ::strtok_r(str,strDelimit,context);
#else
	// Use standard version:
	return ::strtok(str,strDelimit);
#endif
#endif
}

template <class CONTAINER>
void  my_tokenize(
	const std::string  & inString,
	const std::string  & inDelimiters,
	CONTAINER &outTokens,
	bool skipBlankTokens) MRPT_NO_THROWS
{
	outTokens.clear();
	
	const size_t len = inString.size();
	bool prev_was_delim = true;
	std::string cur_token;
	for (size_t pos=0; pos<=len ; pos++) // the "<=" is intentional!!
	{
		char c='\0';
		bool cur_is_delim;
		if (pos==len) 
		{
			// end of string.
			cur_is_delim = true;
		} else 
		{
			// Regular string char:
			c= inString[pos];
			cur_is_delim = (inDelimiters.find(c)!=string::npos);
		}
		if (cur_is_delim) 
		{
			if (prev_was_delim) {
				if (!skipBlankTokens)
					outTokens.push_back( std::string() );
			}
			else {
				outTokens.push_back( cur_token );
			}
			cur_token.clear();
		} else {
			cur_token.push_back(c);
		}
		prev_was_delim = cur_is_delim;
	}
}

void  mrpt::system::tokenize(
	const std::string			&inString,
	const std::string			&inDelimiters,
	std::deque<std::string>	&outTokens,
	bool skipBlankTokens) MRPT_NO_THROWS
{
	my_tokenize(inString,inDelimiters,outTokens,skipBlankTokens);
}

void  mrpt::system::tokenize(
	const std::string			&inString,
	const std::string			&inDelimiters,
	std::vector<std::string>	&outTokens,
	bool skipBlankTokens) MRPT_NO_THROWS
{
	my_tokenize(inString,inDelimiters,outTokens,skipBlankTokens);
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
