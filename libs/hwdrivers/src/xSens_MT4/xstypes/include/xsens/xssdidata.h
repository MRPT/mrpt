/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSSDIDATA_H
#define XSSDIDATA_H

#include "xstypesconfig.h"
#include "xsvector3.h"
#include "xsquaternion.h"

struct XsSdiData;

#ifdef __cplusplus
extern "C" {
#else
#define XSSDIDATA_INITIALIZER {XSQUATERNION_INITIALIZER, XSVECTOR3_INITIALIZER}
#endif

XSTYPES_DLL_API void XsSdiData_construct(struct XsSdiData* thisPtr, const XsReal* orientationIncrement, const XsReal* velocityIncrement);
XSTYPES_DLL_API void XsSdiData_destruct(struct XsSdiData* thisPtr);

#ifdef __cplusplus
} // extern "C"
#endif

struct XsSdiData
{
#ifdef __cplusplus
	//! \brief Construct an empty object
	inline XsSdiData()
	{
	}

	//! \brief Construct an initialized object
	inline XsSdiData(const XsQuaternion& dq, const XsVector& dv)
		: m_orientationIncrement(dq)
		, m_velocityIncrement(dv)
	{
	}

	//! \brief Copy constructor
	inline XsSdiData(const XsSdiData& other)
		: m_orientationIncrement(other.m_orientationIncrement)
		, m_velocityIncrement(other.m_velocityIncrement)
	{
	}

	//! \brief Assignment operator
	inline const XsSdiData& operator=(const XsSdiData& other)
	{
		if (this != &other)
		{
			m_orientationIncrement = other.m_orientationIncrement;
			m_velocityIncrement = other.m_velocityIncrement;
		}
		return *this;
	}

	//! \brief Clear the object so it contains unity data
	inline void zero()
	{
		m_orientationIncrement = XsQuaternion::identity();
		m_velocityIncrement.zero();
	}

	//! \brief Returns the contained orientation increment
	inline const XsQuaternion& orientationIncrement() const
	{
		return m_orientationIncrement;
	}

	//! \brief Update the contained orientation increment
	inline void setOrientationIncrement(const XsQuaternion& dq)
	{
		m_orientationIncrement = dq;
	}

	//! \brief Returns the contained velocity increment
	inline const XsVector3& velocityIncrement() const
	{
		return m_velocityIncrement;
	}

	//! \brief Update the contained velocity increment
	inline void setVelocityIncrement(const XsVector& dv)
	{
		m_velocityIncrement = dv;
	}

private:
#endif

	XsQuaternion m_orientationIncrement;	//!< The orientation increment
	XsVector3    m_velocityIncrement;		//!< The velocity increment
};

typedef struct XsSdiData XsSdiData;

#endif	// file guard
