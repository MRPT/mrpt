/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/*! \file
	\brief Implementations of Cmtmath class functions.

	For information about objects in this file, see the appropriate header:
	\ref Cmtmath.h

	\section FileCopyright Copyright Notice
	Copyright (C) Xsens Technologies B.V., 2006.  All rights reserved.

	This source code is intended for use only by Xsens Technologies BV and
	those that have explicit written permission to use it from
	Xsens Technologies BV.

	THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
	KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
	IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
	PARTICULAR PURPOSE.

	\section FileSwitches Switches
	\li	\c _XSENS_MATH_RANGE_CHECKS	Check whether indices fall in the valid
			range(s) of the object when set to 1 (in CmtMath.h).

	\section FileChangelog	Changelog
	\par 2006-05-31, v0.0.1
	\li Job Mulder:	Created

	\par 2006-06-08, v0.0.2
	\li Job Mulder:	Moved implementation of List to Cmtlist.hpp
	\li Job Mulder: Removed Matlab class and integrated save functions in math classes

	\par 2006-07-21, v0.1.0
	\li Job Mulder:	Updated file for release 0.1.0
*/

#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif

#include <string.h>
#include <ctype.h>
#include <stdio.h>

#include "xsens_list.hpp"

namespace xsens {

int32_t IntList::deserialize(const char* str)
{
	uint32_t tmp = *((const uint32_t *) str);
	resize(tmp);

	memcpy(m_data,str+4,tmp*4);
	m_count = tmp;
	return (int32_t) (4 + tmp*4);
}

int32_t IntList::serialize(char* buffer) const
{
	if (buffer != NULL)
	{
		*((uint32_t*) buffer) = m_count;
		memcpy(buffer+4,m_data,m_count*4);
	}
	return (int32_t) (4 + m_count*4);
}

void IntList::setIncremental(const uint32_t start, const uint32_t end, const int32_t step)
{
	if (step == 0)
		return;
	int32_t size;
	size = 1 + (((int32_t) end - (int32_t) start) / step);

	if ((uint32_t) size > m_max)
		resize(size);

	uint32_t astep = (uint32_t) step;
	m_count = 0;
	if (step > 0)
		for (uint32_t i = start; i < end; i += astep)
			m_data[m_count++] = i;
	else
		for (uint32_t i = start; i > end; i += astep)
			m_data[m_count++] = i;
}

int32_t IntList::readFromString(const char* str)
{
	long int rd = 0;
	long unsigned sz = 0;
	const char* start = str;

	if (sscanf(str,"%lu:%ln",&sz,&rd) != 1)
		return 0;
	str += rd;
	resize(sz);
	m_count = sz;

	for (uint32_t i = 0; i < m_count; ++i)
	{
		long int  auxInt;
		if (sscanf(str,"%li%ln",&auxInt,&rd) != 1)
			return 0;
		m_data[i] = auxInt;
		str += rd;
	}
	return (int32_t) (str - start);
}

int32_t IntList::writeToString(char* buffer) const
{
	char* buf = buffer;
	char fake[128];
	uint32_t written = 0;
	if (buffer == NULL)
		buf = fake;
	written = sprintf(buf,"%lu:",(long unsigned)m_count);
	if (buf != fake)
		buf += written;
	for (uint32_t i = 0;i < m_count;++i)
	{
		written += sprintf(buf," %lu",static_cast<long unsigned>(m_data[i]));
		if (buf != fake)
			buf = buffer + written;
	}
	return (int32_t) written;
}

int32_t IntList::writeToStringHex(char* buffer) const
{
	char* buf = buffer;
	char fake[128];
	uint32_t written = 0;
	if (buffer == NULL)
		buf = fake;
	written = sprintf(buf,"%lu:",static_cast<long unsigned>(m_count));
	if (buf != fake)
		buf += written;
	for (uint32_t i = 0;i < m_count;++i)
	{
		written += sprintf(buf," 0x%lX",static_cast<long unsigned>(m_data[i]));
		if (buf != fake)
			buf = buffer + written;
	}
	return (int32_t) written;
}

void IntList::addValue(int32_t value)
{
	uint32_t add = *((uint32_t*) &value);
	for (uint32_t i = 0; i < m_count; ++i)
		m_data[i] += add;
}

bool IntList::operator == (const IntList& lst)
{
	if (m_count != lst.m_count)
		return false;
	for (uint32_t i = 0; i < m_count; ++i)
		if (m_data[i] != lst.m_data[i])
			return false;
	return true;
}

} // end of xsens namespace
