
//  Copyright (c) 2003-2019 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

#ifndef XSRANGE_H
#define XSRANGE_H

#include "xstypesconfig.h"

#ifdef __cplusplus
extern "C" {
#endif
#ifndef __cplusplus
#define XSRANGE_INITIALIZER	{ 0, -1 }
#endif

struct XsRange;

XSTYPES_DLL_API int XsRange_count(const struct XsRange* thisPtr);
XSTYPES_DLL_API int XsRange_contains(const struct XsRange* thisPtr, int i);
XSTYPES_DLL_API int XsRange_interval(const struct XsRange* thisPtr);
XSTYPES_DLL_API void XsRange_setRange(struct XsRange* thisPtr, int f, int l);
XSTYPES_DLL_API int XsRange_empty(const struct XsRange* thisPtr);

#ifdef __cplusplus
} // extern "C"
#endif

struct XsRange {
#ifdef __cplusplus
	//! \brief Constructs a range starting at \a f and ending at \a l. Default values are 0 and -1 respectively.
	inline explicit XsRange(int f = 0, int l = -1)
		: m_first(f)
		, m_last(l)
	{}

	//! \brief Constructs a range based upon \a other
	inline XsRange(const XsRange& other)
		: m_first(other.m_first)
		, m_last(other.m_last)
	{}

	//! \brief Assigns the range \a other to this
	inline XsRange& operator = (const XsRange& other)
	{
		m_first = other.m_first;
		m_last = other.m_last;
		return *this;
	}

	//! \brief \copybrief XsRange_count
	inline int count() const
	{
		return XsRange_count(this);
	}

	//! \brief \copybrief XsRange_interval
	inline int interval() const
	{
		return XsRange_interval(this);
	}

	//! \brief \copybrief XsRange_contains
	inline bool contains(int i) const
	{
		return 0 != XsRange_contains(this, i);
	}

	//! \brief Set the range to \a f - \a l
	inline void setRange(int f, int l)
	{
		m_first = f;
		m_last = l;
	}

	//! \brief \copybrief XsRange_empty
	inline bool empty() const
	{
		return 0 != XsRange_empty(this);
	}

	//! \brief Return the \e first value of the range
	inline int first() const
	{
		return m_first;
	}

	//! \brief Return the \e last value of the range
	inline int last() const
	{
		return m_last;
	}

	//! \brief Return true if this is equal to other
	bool operator == (const XsRange& other) const
	{
		return m_first == other.m_first && m_last == other.m_last;
	}

	//! \brief Return true if start of this is less than start of other
	bool operator < (const XsRange& other) const
	{
		return m_first < other.m_first;
	}

private:
#endif

	int m_first;	//!< Storage for the lower end of the range
	int m_last;		//!< Storage for the upper end of the range
};

typedef struct XsRange XsRange;

#if defined(__cplusplus) && !defined(XSENS_NO_STL)
#include <ostream>

template<typename _CharT, typename _Traits>
std::basic_ostream<_CharT, _Traits>& operator<<(std::basic_ostream<_CharT, _Traits>& o, XsRange const& xs)
{
	return (o << "(" << xs.first() << ", " << xs.last() << "]");
}
#endif

#endif
