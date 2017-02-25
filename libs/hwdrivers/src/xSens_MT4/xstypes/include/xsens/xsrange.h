/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSRANGE_H
#define XSRANGE_H

#include "xstypesconfig.h"

#ifdef __cplusplus
extern "C" {
#else
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
	//! \brief Constructs a range starting at \a f and ending at \a l. Default value are 0.
	inline explicit XsRange(int f = 0, int l = 0)
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
private:
#endif

	int m_first;	//!< Storage for the lower end of the range
	int m_last;		//!< Storage for the upper end of the range
};

typedef struct XsRange XsRange;

#endif // file guard
