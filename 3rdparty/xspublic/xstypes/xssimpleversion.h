
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

#ifndef XSSIMPLEVERSION_H
#define XSSIMPLEVERSION_H

#include "xstypesconfig.h"

#undef minor
#undef major

typedef struct XsSimpleVersion XsSimpleVersion;

#ifdef __cplusplus
extern "C" {
#endif
#ifndef __cplusplus
#define XSSIMPLEVERSION_INITIALIZER { 0, 0, 0 }
#endif

XSTYPES_DLL_API int XsSimpleVersion_empty(const XsSimpleVersion* thisPtr);
XSTYPES_DLL_API void XsSimpleVersion_swap(XsSimpleVersion* a, XsSimpleVersion* b);
XSTYPES_DLL_API int XsSimpleVersion_compare(XsSimpleVersion const* a, XsSimpleVersion const* b);

#ifdef __cplusplus
} // extern "C"
#endif

struct XsSimpleVersion {
#ifdef __cplusplus
	//! \brief Constructs a simple-version object using the supplied parameters or an empty version object if no parameters are given.
	explicit XsSimpleVersion(int vmaj = 0, int vmin = 0, int vrev = 0)
		: m_major((uint8_t) vmaj)
		, m_minor((uint8_t) vmin)
		, m_revision((uint8_t) vrev)
	{}

	//! \brief Constructs a simple-version object based upon the \a other object
	XsSimpleVersion(const XsSimpleVersion& other)
		: m_major(other.m_major)
		, m_minor(other.m_minor)
		, m_revision(other.m_revision)
	{}

	//! \brief Assign the simple-version from the \a other object
	XsSimpleVersion& operator = (const XsSimpleVersion& other)
	{
		m_major = other.m_major;
		m_minor = other.m_minor;
		m_revision = other.m_revision;
		return *this;
	}

	/*! \brief Test if the \a other simple-version is equal to this. */
	inline bool operator == (const XsSimpleVersion& other) const
	{
		return !XsSimpleVersion_compare(this, &other);
	}

	/*! \brief Test if the \a other simple-version is different to this. */
	inline bool operator != (const XsSimpleVersion& other) const
	{
		if (m_major != other.m_major || m_minor != other.m_minor || m_revision != other.m_revision)
			return true;

		return false;
	}

	/*! \brief Test if the \a other version is lower than this. The comparison involves only the version numbers (major, minor and revision). */
	inline bool operator < (const XsSimpleVersion& other) const
	{
		if (m_major < other.m_major)
			return true;
		else if (m_major > other.m_major)
			return false;
		else
		{
			if (m_minor < other.m_minor)
				return true;
			else if (m_minor > other.m_minor)
				return false;
			else
			{
				if (m_revision < other.m_revision)
					return true;
				else
					return false;
			}
		}
	}

	/*! \brief Test if the \a other version is lower or equal than this. */
	inline bool operator <= (const XsSimpleVersion& other) const
	{
		return (*this == other) || (*this < other);
	}

	/*! \brief Test if the \a other version is higher than this. */
	inline bool operator > (const XsSimpleVersion& other) const
	{
		return !(*this <= other);
	}

	/*! \brief Test if the \a other version is higher or equal than this. */
	inline bool operator >= (const XsSimpleVersion& other) const
	{
		return (*this == other) || (*this > other);
	}

	//! \brief \copybrief XsSimpleVersion_empty
	inline bool empty() const
	{
		return 0 != XsSimpleVersion_empty(this);
	}

	//! \brief Return the \e major part of the version
	inline int major() const { return (int) m_major; }
	//! \brief Return the \e minor part of the version
	inline int minor() const { return (int) m_minor; }
	//! \brief Return the \e revision part of the version
	inline int revision() const { return (int) m_revision; }

private:
#endif
	uint8_t m_major;			//!< The major part of the version number
	uint8_t m_minor;			//!< The minor part of the version number
	uint8_t m_revision;			//!< The revision number of the version
};

#endif
