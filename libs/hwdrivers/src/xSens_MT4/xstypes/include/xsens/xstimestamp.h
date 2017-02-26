/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSTIMESTAMP_H
#define XSTIMESTAMP_H

#include "xstypesconfig.h"
#include "pstdint.h"

#ifdef __cplusplus
extern "C" {
#else
#define XSTIMESTAMP_INITIALIZER	{ 0 }
#endif

struct XsTimeStamp;
struct XsUtcTime;

XSTYPES_DLL_API void XsTimeStamp_setMilliSecondTime(struct XsTimeStamp* thisPtr, int64_t t);
XSTYPES_DLL_API double XsTimeStamp_timeOfDay(const struct XsTimeStamp* thisPtr);
XSTYPES_DLL_API int64_t XsTimeStamp_secondTime(const struct XsTimeStamp* thisPtr);
XSTYPES_DLL_API int32_t XsTimeStamp_milliSecondPart(const struct XsTimeStamp* thisPtr);
XSTYPES_DLL_API int32_t XsTimeStamp_secondPart(const struct XsTimeStamp* thisPtr);
XSTYPES_DLL_API int32_t XsTimeStamp_minutePart(const struct XsTimeStamp* thisPtr);
XSTYPES_DLL_API int32_t XsTimeStamp_hourPart(const struct XsTimeStamp* thisPtr);
XSTYPES_DLL_API int64_t XsTimeStamp_now(struct XsTimeStamp* thisPtr);
XSTYPES_DLL_API int64_t XsTimeStamp_maxValue();
XSTYPES_DLL_API int64_t XsTimeStamp_fromUtcTime(struct XsTimeStamp* thisPtr, const struct XsUtcTime* utc);
XSTYPES_DLL_API void XsTimeStamp_toUtcTime(struct XsTimeStamp* thisPtr, struct XsUtcTime* utc);

#ifdef __cplusplus
} // extern "C"
#endif

/*!	\struct XsTimeStamp
	\brief Class for managing timestamps in a unified way.
*/
struct XsTimeStamp {
#ifdef __cplusplus
	/*! \brief Construct a timestamp with \a t as the time in milliseconds. */
	inline XsTimeStamp(int64_t t = 0) : m_msTime(t) {}

	/*! \brief Construct a timestamp with \a t as the time in milliseconds. */
	inline XsTimeStamp(int t) : m_msTime(t) {}

	/*! \brief Construct a copy of \a other. */
	inline XsTimeStamp(const XsTimeStamp& other) : m_msTime(other.m_msTime) {}

	/*! \brief Construct from \a utc */
	inline XsTimeStamp(const XsUtcTime& utc)
	{
		XsTimeStamp_fromUtcTime(this, &utc);
	}

	/*! \brief Convert this timestamp to UTC time \a utc */
	inline void toUtcTime(XsUtcTime& utc)
	{
		XsTimeStamp_toUtcTime(this, &utc);
	}

	/*! \brief Assign the contents of the \a other timestamp to this timestamp. */
	inline XsTimeStamp& operator = (const XsTimeStamp& other)
	{
		m_msTime = other.m_msTime;
		return *this;
	}

	/*! \brief Get the stored time as milliseconds. */
	inline int64_t msTime(void) const
	{ return m_msTime; }

	/*! \brief Set the stored time to \a t milliseconds. */
	inline void setMsTime(int64_t t)
	{ m_msTime = t; }

	/*! \brief Get the time of day component of the stored timestamp in seconds as a double precision value.
	*/
	inline double timeOfDay() const
	{ return XsTimeStamp_timeOfDay(this); }

	/*! \brief Get the time of day component of the stored timestamp in milliseconds
	*/
	inline int64_t msTimeOfDay() const
	{ return m_msTime % (24*60*60*1000); }

	/*! \brief Return the time as seconds */
	inline double secTime() const
	{ return ((double)m_msTime)*0.001; }

	/*! \brief Set the time as seconds */
	inline void setSecTime(double t)
	{ m_msTime = (int64_t) (t*1000.0); }

	/*! \brief Get the sum of the current and the given \a other timestamp. \param other The value to add to this \returns The added timestamp values */
	inline XsTimeStamp operator + (const XsTimeStamp& other) const
	{ return XsTimeStamp(m_msTime + other.m_msTime); }

	/*! \brief Get the current minus the given \a other timestamp. \param other The value to subtract from this \returns The subtracted timestamp values */
	inline XsTimeStamp operator - (const XsTimeStamp& other) const
	{ return XsTimeStamp(m_msTime - other.m_msTime); }

	/*! \brief Get the result of adding the given \a other timestamp to the current timestamp. */
	inline XsTimeStamp& operator += (const XsTimeStamp& other)
	{ m_msTime += other.m_msTime; return *this; }

	/*! \brief Get the result of subtracting the given \a other timestamp from the current timestamp. */
	inline XsTimeStamp& operator -= (const XsTimeStamp& d)
	{ m_msTime -= d.m_msTime; return *this; }

	/*! \brief Test if the given \a other timestamp is smaller than the current timestamp. */
	inline bool operator <  (const XsTimeStamp& other) const
	{ return m_msTime <  other.m_msTime; }

	/*! \brief Test if the given \a other timestamp is smaller than or equal to the current timestamp. */
	inline bool operator <= (const XsTimeStamp& other) const
	{ return m_msTime <= other.m_msTime; }

	/*! \brief Test if the given \a other timestamp is equal to the current timestamp. */
	inline bool operator == (const XsTimeStamp& other) const
	{ return m_msTime == other.m_msTime; }

	/*! \brief Test if the given \a other timestamp is larger than the current timestamp. */
	inline bool operator >  (const XsTimeStamp& other) const
	{ return m_msTime >  other.m_msTime; }

	/*! \brief Test if the given \a other timestamp is larger than or equal to the current timestamp. */
	inline bool operator >= (const XsTimeStamp& other) const
	{ return m_msTime >= other.m_msTime; }

	/*! \brief Test if the given \a other timestamp is not equal to the current timestamp. */
	inline bool operator != (const XsTimeStamp& other) const
	{ return m_msTime != other.m_msTime; }

	/*! \brief Test if the given \a other is smaller than the current timestamp. */
	inline bool operator <  (int other) const
	{ return m_msTime <  other; }

	/*! \brief Test if the given \a other is smaller than or equal to the current timestamp. */
	inline bool operator <= (int other) const
	{ return m_msTime <= other; }

	/*! \brief Test if the given \a other is equal to the current timestamp. */
	inline bool operator == (int other) const
	{ return m_msTime == other; }

	/*! \brief Test if the given \a other is larger than the current timestamp. */
	inline bool operator >  (int other) const
	{ return m_msTime >  other; }

	/*! \brief Test if the given \a other is larger than or equal to the current timestamp. */
	inline bool operator >= (int other) const
	{ return m_msTime >= other; }

	/*! \brief Test if the given \a other is not equal to the current timestamp. */
	inline bool operator != (int other) const
	{ return m_msTime != other; }

	/*! \brief Returns the number of seconds elapsed since the epoch as stored in the XsTimeStamp */
	inline int64_t secondTime() const
	{ return m_msTime/1000; }

	/*! \brief Returns the millisecond part of the time (in the range 0-999) */
	inline int32_t milliSecondPart() const
	{ return (int32_t) (m_msTime % 1000); }

	/*! \brief Returns the seconds part of the time (in the range 0-59) */
	inline int32_t secondPart() const
	{ return (int32_t) ((m_msTime/(1000))%60); }

	/*! \brief Returns the minutes part of the time (in the range 0-59) */
	inline int32_t minutePart() const
	{ return (int32_t) ((m_msTime/(60*1000))%60); }

	/*! \brief Returns the hours part of the time (in the range 0-23) */
	inline int32_t hourPart() const
	{ return (int32_t) ((m_msTime/(60*60*1000))%24); }

	/*! \brief Returns the current time in ms since the epoch (Jan 1st 1970) */
	inline static XsTimeStamp now()
	{
		XsTimeStamp tmp;
		XsTimeStamp_now(&tmp);
		return tmp;
	}

	/*! \brief Returns the current time in ms since the epoch (Jan 1st 1970) */
	inline static int64_t nowMs()
	{
		XsTimeStamp tmp;
		XsTimeStamp_now(&tmp);
		return tmp.msTime();
	}

	/*! \brief Returns the maximum value of an %XsTimeStamp */
	inline static XsTimeStamp maxValue()
	{
		return XsTimeStamp(int64_t(9223372036854775807LL));	//INT64_MAX
	}

	/*! \brief Increment the timestamp by one ms, prefix */
	XsTimeStamp operator++()
	{ return XsTimeStamp(++m_msTime); }

	/*! \brief Increment the timestamp by one ms, postfix */
	XsTimeStamp operator++(int)
	{ return XsTimeStamp(m_msTime++); }

	/*! \brief Decrement the timestamp by one ms, prefix */
	XsTimeStamp operator--()
	{ return XsTimeStamp(--m_msTime); }

	/*! \brief Decrement the timestamp by one ms, postfix */
	XsTimeStamp operator--(int)
	{ return XsTimeStamp(m_msTime--); }

private:
#endif

	int64_t m_msTime;		//!< The timestamp value
};

typedef struct XsTimeStamp XsTimeStamp;

#endif // file guard
