
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

#ifndef XSFILTERPROFILE_H
#define XSFILTERPROFILE_H

#include "xstypesconfig.h"
#include "xsstring.h"
#include "pstdint.h"
#include "xsfilterprofilekind.h"

#ifdef __cplusplus
extern "C" {
#else
#define XSFILTERPROFILE_INITIALIZER {0, 0, {0}, 0, 0, 0}
#endif

struct XsFilterProfile;
XSTYPES_DLL_API void XsFilterProfile_toString(struct XsFilterProfile const* thisPtr, XsString *out);
XSTYPES_DLL_API int XsFilterProfile_empty(struct XsFilterProfile const* thisPtr);
XSTYPES_DLL_API void XsFilterProfile_swap(struct XsFilterProfile* a, struct XsFilterProfile* b);
#ifdef __cplusplus
}
#endif

#define XS_MAX_FILTERPROFILES			254
#define XS_LEN_FILTERPROFILELABEL_TERM	(20+1)
#define XS_LEN_FILTERPROFILEKIND_TERM	(20+1)
#define XS_MAX_FILTERPROFILES_IN_MT		5

struct XsFilterProfile
{
#ifdef __cplusplus
	/*! \brief Construct a filter profile object
		\param type_ the profile type
		\param version_ the profile version
		\param kind_ the kind of profile
		\param label_ the profile name
		\param filterType_ the filter type this profile is for
		\param filterMajor_ the major version of the compatible filter
		\param filterMinor_ the minor version of the compatible filter
	*/
	explicit XsFilterProfile(uint8_t type_ = 0, uint8_t version_ = 0, const char* kind_ = nullptr, const char* label_ = nullptr, char filterType_ = 0, uint8_t filterMajor_ = 0, uint8_t filterMinor_ = 0)
		: m_type(type_)
		, m_version(version_)
		, m_filterType(filterType_)
		, m_filterMajor(filterMajor_)
		, m_filterMinor(filterMinor_)
	{
		setKind(kind_);
		setLabel(label_);
	}

	/*! \brief Copy constructor for a filter profile object
		\param other the filter profile object to construct a copy of
	*/
	XsFilterProfile(const XsFilterProfile& other)
		: m_type(other.m_type)
		, m_version(other.m_version)
		, m_filterType(other.m_filterType)
		, m_filterMajor(other.m_filterMajor)
		, m_filterMinor(other.m_filterMinor)
	{
		setKind(other.m_kind);
		setLabel(other.m_label);
	}

	/*! \brief Destroy the filter profile */
	~XsFilterProfile() {}

	/*! \brief \copybrief XsFilterProfile_empty
		\returns true if the filter profile is empty
		\sa XsFilterProfile_empty
	*/
	inline bool empty()
	{
		return (0 != XsFilterProfile_empty(this));
	}

	/*! \brief \copybrief XsFilterProfile_toString
		\returns a string representation of this filter profile
		\sa XsFilterProfile_toString
	*/
	inline XsString toString()
	{
		XsString out;
		XsFilterProfile_toString(this, &out);
		return out;
	}

	/*! \brief The filter profile type */
	inline uint8_t type() const { return m_type; }

	/*! \brief The filter profile version */
	inline uint8_t version() const { return m_version; }

	/*! \brief The filter profile name */
	inline const char* label() const { return m_label; }

	/*! \brief The filter profile kind */
	inline const char* kind() const { return m_kind; }

	/*! \brief The filter type this filter profile is for */
	inline char filterType() const { return m_filterType; }

	/*! \brief The major version of the compatible filter */
	inline uint8_t filterMajor() const { return m_filterMajor; }

	/*! \brief The minor version of the compatible filter */
	inline uint8_t filterMinor() const { return m_filterMinor; }

	/*! \brief Set the type of the filter profile to \a type_
		\param type_ the new type of the filter profile
	*/
	inline void setType(uint8_t type_)
	{
		m_type = type_;
	}

	/*! \brief Set the version of the filter profile to \a version_
		\param version_ the new label of the filter profile
	*/
	inline void setVersion(uint8_t version_)
	{
		m_version = version_;
	}

	/*! \brief Set the label of the filter profile \a label_
		\param label_ the new label of the filter profile
	*/
	inline void setLabel(const char* label_)
	{
		if (!label_ || label_[0] == 0)
		{
			m_label[0] = 0;
		}
		else
		{
			int i = 0;
			for (; i < 2 * (XS_LEN_FILTERPROFILELABEL_TERM-1); ++i)
			{
				if (label_[i] == '\0' || label_[i] == ' ')
					break;
				m_label[i] = label_[i];
			}
			m_label[i] = 0;
		}
	}

	/*! \brief Set the kind of filter profile \a kind_
	  \param kind_ the new kind of filter profile
	*/
	inline void setKind(const char* kind_)
	{
		if (!kind_ || kind_[0] == 0)
		{
			m_kind[0] = 0;
		}
		else
		{
			int i = 0;
			for (; i < XS_LEN_FILTERPROFILELABEL_TERM-1; ++i)
			{
				if (kind_[i] == '\0' || kind_[i] == ' ')
					break;
				m_kind[i] = kind_[i];
			}
			m_kind[i] = 0;
		}
	}

	/*! \brief Set the filter type of this filter profile to \a filterType_
		\param filterType_ the new filter type
	*/
	inline void setFilterType(char filterType_)
	{
		m_filterType = filterType_;
	}

	/*! \brief Set the filter version of this filter profile to \a major_, \a minor_
		\param major_ the major version number
		\param minor_ the minor version number
	*/
	inline void setFilterVersion(uint8_t major_, uint8_t minor_)
	{
		m_filterMajor = major_;
		m_filterMinor = minor_;
	}

	/*! \brief Swap the contents with \a other
	*/
	inline void swap(XsFilterProfile& other)
	{
		XsFilterProfile_swap(this, &other);
	}

	/*! \brief Return true if the filter profile type and version are identical to those of \a other */
	inline bool operator == (const XsFilterProfile& other) const
	{
		return m_filterMajor == other.m_filterMajor && m_filterMinor == other.m_filterMinor;
	}

protected:
#endif

	uint8_t m_type;								//!< The type of the filter profile. When set to 255 in an operation, the 'current' filter profile is used.
	uint8_t m_version;							//!< The version of the filter profile.
	char m_kind[XS_LEN_FILTERPROFILEKIND_TERM];	//!< The kind of filter profile.
	char m_label[1 + 2 * XS_LEN_FILTERPROFILELABEL_TERM];	//!< The label of the filter profile (Can be 2 names, including separator)
	char m_filterType;							//!< The type of the XKF filter this filter profile is intended for '3': XKF-3, '6': XKF-6. \note The value is a character, so XKF-3 is '3', which is hex 0x33
	uint8_t m_filterMajor;						//!< The major version of the XKF filter this filter profile is intended for
	uint8_t m_filterMinor;						//!< The minor version of the XKF filter this filter profile is intended for
};

typedef struct XsFilterProfile XsFilterProfile;

#endif
