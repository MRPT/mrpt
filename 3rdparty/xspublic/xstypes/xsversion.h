
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

#ifndef XSVERSION_H
#define XSVERSION_H

#include "xstypesconfig.h"
#include "xssimpleversion.h"
#include "xsstring.h"

#undef minor
#undef major

typedef struct XsVersion XsVersion;

#ifdef __cplusplus
extern "C" {
#endif
#ifndef __cplusplus
#define XSVERSION_INITIALIZER { 0, 0, 0, 0, 0, XsString_INITIALIZER }
#endif

XSTYPES_DLL_API int XsVersion_empty(const XsVersion* thisPtr);
XSTYPES_DLL_API void XsVersion_toString(const XsVersion* thisPtr, XsString* version);
XSTYPES_DLL_API void XsVersion_fromString(XsVersion* thisPtr, const XsString* version);
XSTYPES_DLL_API void XsVersion_fromSimpleVersion(XsVersion* thisPtr, const XsSimpleVersion* simpleVersion);
XSTYPES_DLL_API void XsVersion_toSimpleVersion(const XsVersion* thisPtr, XsSimpleVersion* simpleVersion);
XSTYPES_DLL_API void XsVersion_toSimpleString(const XsVersion* thisPtr, XsString* version);

#ifdef __cplusplus
} // extern "C"
#endif

struct XsVersion {
#ifdef __cplusplus
	//! \brief Constructs a version object using the supplied parameters or an empty version object if no parameters are given.
	explicit XsVersion(int maj, int min, int rev, int build, const XsString& extra)
		: m_major(maj)
		, m_minor(min)
		, m_revision(rev)
		, m_build(build)
		, m_reposVersion(0)
		, m_extra(extra)
	{}

	//! \brief Constructs a version object using the supplied parameters or an empty version object if no parameters are given.
	explicit XsVersion(int maj = 0, int min = 0, int rev = 0, int build = 0, int reposVersion = 0, const XsString& extra = XsString())
		: m_major(maj)
		, m_minor(min)
		, m_revision(rev)
		, m_build(build)
		, m_reposVersion(reposVersion)
		, m_extra(extra)
	{}

	//! \brief Constructs a version object based upon the version contained by \a vString.
	explicit XsVersion(const XsString& vString)
	{
		XsVersion_fromString(this, &vString);
	}

	//! \brief Constructs a version object based upon the version contained by \a simpleVersion.
	explicit XsVersion(const XsSimpleVersion& simpleVersion)
	{
		XsVersion_fromSimpleVersion(this, &simpleVersion);
	}

	//! \brief Constructs a version object based upon the \a other object
	XsVersion(const XsVersion& other)
		: m_major(other.m_major)
		, m_minor(other.m_minor)
		, m_revision(other.m_revision)
		, m_build(other.m_build)
		, m_reposVersion(other.m_reposVersion)
		, m_extra(other.m_extra)
	{}

	//! \brief Assign the version from the \a other object
	XsVersion& operator = (const XsVersion& other)
	{
		m_major = other.m_major;
		m_minor = other.m_minor;
		m_revision = other.m_revision;
		m_build = other.m_build;
		m_reposVersion = other.m_reposVersion;
		m_extra = other.m_extra;
		return *this;
	}

	/*! \brief Test if the \a other version is equal to this. The comparison involves the entire object.*/
	inline bool isEqual (const XsVersion& other) const
	{
		return (*this == other) && (m_build == other.m_build) && (m_extra == other.m_extra) && (m_reposVersion == other.m_reposVersion);
	}

	/*! \brief Test if the \a other version is equal to this. The comparison involves only the version numbers (major, minor and revision). */
	inline bool operator == (const XsVersion& other) const
	{
		if (m_major == other.m_major && m_minor == other.m_minor && m_revision == other.m_revision)
			return true;

		return false;
	}

	/*! \brief Test if the \a other version is NOT equal to this. The comparison involves only the version numbers (major, minor and revision). */
	inline bool operator != (const XsVersion& other) const
	{
		return !(*this == other);
	}

	/*! \brief Test if the \a other version is lower than this. The comparison involves only the version numbers (major, minor and revision). */
	inline bool operator < (const XsVersion& other) const
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

	/*! \brief Test if the \a other version is lower or equal than this. The comparison involves only the version numbers (major, minor and revision). */
	inline bool operator <= (const XsVersion& other) const
	{
		return (*this == other) || (*this < other);
	}

	/*! \brief Test if the \a other version is higher than this. The comparison involves only the version numbers (major, minor and revision). */
	inline bool operator > (const XsVersion& other) const
	{
		return !(*this <= other);
	}

	/*! \brief Test if the \a other version is higher or equal than this. The comparison involves only the version numbers (major, minor and revision). */
	inline bool operator >= (const XsVersion& other) const
	{
		return (*this == other) || (*this > other);
	}

	//! \brief \copybrief XsVersion_empty
	inline bool empty() const
	{
		return 0 != XsVersion_empty(this);
	}

	//! \brief \copybrief XsVersion_toString
	inline XsString toString() const
	{
		XsString tmp;
		XsVersion_toString(this, &tmp);
		return tmp;
	}

	//! \brief \copybrief XsVersion_toSimpleString
	inline XsString toSimpleString() const
	{
		XsString tmp;
		XsVersion_toSimpleString(this, &tmp);
		return tmp;
	}

	//! \brief \copybrief XsVersion_toSimpleVersion
	inline XsSimpleVersion toSimpleVersion() const
	{
		XsSimpleVersion result;
		XsVersion_toSimpleVersion(this, &result);
		return result;
	}

	//! \brief Return the \e major part of the version
	inline int major() const { return m_major; }
	//! \brief Return the \e minor part of the version
	inline int minor() const { return m_minor; }
	//! \brief Return the \e revision part of the version
	inline int revision() const { return m_revision; }
	//! \brief Return the \e build number used for this build
	inline int build() const { return m_build; }
	//! \brief Return the \e source revision used for this build
	inline int reposVersion() const { return m_reposVersion;}
	//! \brief Return the extra part of the version. This may contain custom version details such as 'beta' or 'Mk4' to indicate the readiness and purpose of this version of the object.
	inline const XsString& extra() const { return m_extra; }

	//! \brief Set the \e major part of the version
	inline void setMajor(int major) { m_major = major; }
	//! \brief Set the \e minor part of the version
	inline void setMinor(int minor) { m_minor = minor; }
	//! \brief Set the \e revision part of the version
	inline void setRevision(int revision) { m_revision = revision; }
	//! \brief Set the \e build part of the version
	inline void setBuild(int build) { m_build = build; }
	//! \brief Set the \e reposVersion part of the version
	inline void setReposVersion(int reposVersion) { m_reposVersion = reposVersion; }
	//! \brief Set the \e extra part of the version. This may contain custom version details such as 'beta' or 'Mk4' to indicate the readiness and purpose of this version of the object.
	inline void setExtra(const XsString& extra) { m_extra = extra; }

private:
#endif

	int m_major;			//!< The major part of the version number
	int m_minor;			//!< The minor part of the version number
	int m_revision;			//!< The revision number of the version
	int m_build;			//!< The build number for this build
	int m_reposVersion;		//!< The source revision used for this build
	XsString m_extra;		//!< Storage for some extra information about the version
};

#endif
