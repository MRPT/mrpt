
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

#ifndef XSSDIDATA_H
#define XSSDIDATA_H

#include "xstypesconfig.h"
#include "xsvector3.h"
#include "xsquaternion.h"

struct XsSdiData;

#ifdef __cplusplus
extern "C" {
#endif
#ifndef __cplusplus
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

	/*! \brief Returns true if all fields of this and \a other are exactly identical */
	inline bool operator == (const XsSdiData& other) const
	{
		return	m_orientationIncrement == other.m_orientationIncrement &&
				m_velocityIncrement == other.m_velocityIncrement;
	}

private:
#endif

	XsQuaternion m_orientationIncrement;	//!< The orientation increment
	XsVector3    m_velocityIncrement;		//!< The velocity increment
};

typedef struct XsSdiData XsSdiData;

#endif	// file guard
