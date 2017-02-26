/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSVERSION_H
#define XSVERSION_H

#include "xstypesconfig.h"
#include "xsstring.h"

typedef struct XsVersion XsVersion;

#ifdef __cplusplus
extern "C" {
#else
#define XSVERSION_INITIALIZER { 0, 0, 0, 0, XsString_INITIALIZER }
#endif

XSTYPES_DLL_API int XsVersion_empty(const XsVersion* thisPtr);
XSTYPES_DLL_API void XsVersion_toString(const XsVersion* thisPtr, XsString* version);

#ifdef __cplusplus
} // extern "C"
#endif

struct XsVersion {
#ifdef __cplusplus
	//! \brief Constructs a version object using the supplied parameters or an empty version object if no parameters are given.
	explicit XsVersion(int maj = 0, int min = 0, int rev = 0, int bld = 0, const XsString& extra = XsString())
		: m_major(maj)
		, m_minor(min)
		, m_revision(rev)
		, m_build(bld)
		, m_extra(extra)
	{}

	//! \brief Constructs a version object based upon the \a other object
	XsVersion(const XsVersion& other)
		: m_major(other.m_major)
		, m_minor(other.m_minor)
		, m_revision(other.m_revision)
		, m_build(other.m_build)
		, m_extra(other.m_extra)
	{}

	//! \brief Assign the version from the \a other object
	XsVersion& operator = (const XsVersion& other)
	{
		m_major = other.m_major;
		m_minor = other.m_minor;
		m_revision = other.m_revision;
		m_build = other.m_build;
		m_extra = other.m_extra;
		return *this;
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

	//! \brief Return the \e major part of the version
	inline int major() const { return m_major; }
	//! \brief Return the \e minor part of the version
	inline int minor() const { return m_minor; }
	//! \brief Return the \e revision part of the version
	inline int revision() const { return m_revision; }
	//! \brief Return the \e build \e number part of the version
	inline int build() const { return m_build; }
	//! \brief Return the extra part of the version. This may contain custom version details such as 'beta' or 'Mk4' to indicate the readiness and purpose of this version of the object.
	inline XsString extra() const { return m_extra; }

private:
#endif

	int m_major;			//!< The major part of the version number 
	int m_minor;			//!< The minor part of the version number
	int m_revision;			//!< The revision number of the version	
	int m_build;			//!< The build number of the version
	XsString m_extra;		//!< Storage for some extra information about the version
};

#endif // file guard
