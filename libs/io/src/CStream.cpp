/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "io-precomp.h"  // Precompiled headers

#include <mrpt/io/CStream.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/reverse_bytes.h>
#include <iostream>
#include <cstdarg>
#include <vector>
#include <cstring>  // strlen()

//#include "internal_class_registry.h"

using namespace mrpt;
using namespace mrpt::io;
using namespace std;

CStream::~CStream() = default;
/*---------------------------------------------------------------
			Writes an elemental data type to stream.
 ---------------------------------------------------------------*/
int CStream::printf(const char* fmt, ...)
{
	MRPT_START

	if (!fmt) throw std::runtime_error("fmt in CStream::printf cannot be NULL");

	int result = -1, length = 1024;
	vector<char> buffer;
	while (result == -1)
	{
		buffer.resize(length + 10);

		va_list args;  // This must be done WITHIN the loop
		va_start(args, fmt);
#if defined(_MSC_VER)
		result = ::vsnprintf_s(&buffer[0], length, _TRUNCATE, fmt, args);
#else
		result = ::vsnprintf(&buffer[0], length, fmt, args);
#endif
		va_end(args);

		// Truncated?
		if (result >= length) result = -1;
		length *= 2;
	}

	size_t l = strlen(&buffer[0]);
	this->Write(&buffer[0], (int)l);

	return result;

	MRPT_END
}

/*-------------------------------------------------------------
Reads from the stream until a '\n' character is found ('\r' characters are
ignored).
return false on EOF or any other read error.
-------------------------------------------------------------*/
bool CStream::getline(std::string& out_str)
{
	out_str.clear();
	try
	{
		for (;;)
		{
			size_t N = out_str.size();
			out_str.resize(N + 1);
			if (!Read(&out_str[N], 1)) return false;

			// New char read:
			if (out_str[N] == '\r')
			{
				out_str.resize(N);  // Ignore.
			}
			else if (out_str[N] == '\n')
			{
				out_str.resize(N);  // End of line!
				return true;  // Ok.
			}
		}
	}
	catch (...)
	{  // Any read error:
		return false;
	}
}
