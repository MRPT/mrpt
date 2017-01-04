/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSCALIBRATEDDATA_H
#define XSCALIBRATEDDATA_H

#include "xstypesconfig.h"
#include "xsvector3.h"

struct XsCalibratedData;

#ifdef __cplusplus
extern "C" {
#else
#define XSCALIBRATEDDATA_INITIALIZER {XSVECTOR3_INITIALIZER, XSVECTOR3_INITIALIZER, XSVECTOR3_INITIALIZER}
#endif

XSTYPES_DLL_API void XsCalibratedData_construct(struct XsCalibratedData* thisPtr, const XsReal* acc, const XsReal* gyr, const XsReal* mag);
XSTYPES_DLL_API void XsCalibratedData_destruct(struct XsCalibratedData* thisPtr);

#ifdef __cplusplus
} // extern "C"
#endif

struct XsCalibratedData
{
	XsVector3 m_acc;	//!< Accelerometer data
	XsVector3 m_gyr;	//!< Gyroscope data
	XsVector3 m_mag;	//!< Magnetometer data

#ifdef __cplusplus
	//! \brief Constructor \sa XsCalibratedData_construct
	inline XsCalibratedData()
	{}

	//! \brief Copy constructor, copies the values from \a other to this
	inline XsCalibratedData(const XsCalibratedData& other)
		: m_acc(other.m_acc)
		, m_gyr(other.m_gyr)
		, m_mag(other.m_mag)
	{
	}

	//! \brief Destructor
	inline ~XsCalibratedData()
	{}

	//! \brief Assignment operator, copies the values from \a other to this
	inline const XsCalibratedData& operator = (const XsCalibratedData& other)
	{
		if (this != &other)
		{
			m_acc = other.m_acc;
			m_gyr = other.m_gyr;
			m_mag = other.m_mag;
		}
		return *this;
	}
#endif
};
typedef struct XsCalibratedData XsCalibratedData;

#endif // file guard
