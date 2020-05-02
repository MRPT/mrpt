
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

#ifndef XSUSBHUBINFO_H
#define XSUSBHUBINFO_H

#include "xscontrollerconfig.h"

#ifdef _WIN32
	typedef int XsHubIdentifier;
#else
	typedef const char* XsHubIdentifier;
#endif

struct XsUsbHubInfo;
#ifndef __cplusplus
#define XSUSBHUBINFO_INITIALIZER { 0 }
typedef struct XsUsbHubInfo XsUsbHubInfo;
#else
extern "C" {
#endif

void XDA_DLL_API XsUsbHubInfo_assign(XsUsbHubInfo* thisPtr, XsHubIdentifier hub);
void XDA_DLL_API XsUsbHubInfo_construct(XsUsbHubInfo* thisPtr, XsHubIdentifier hub);
void XDA_DLL_API XsUsbHubInfo_destruct(XsUsbHubInfo* thisPtr);
void XDA_DLL_API XsUsbHubInfo_copy(XsUsbHubInfo* copy, XsUsbHubInfo const* src);
void XDA_DLL_API XsUsbHubInfo_swap(XsUsbHubInfo* thisPtr, XsUsbHubInfo* thatPtr);
int XDA_DLL_API  XsUsbHubInfo_parentPathMatches(const XsUsbHubInfo* thisPtr, const XsUsbHubInfo* other);

#ifdef __cplusplus
} // extern "C"
#endif


/*! \struct XsUsbHubInfo
	\brief A structure that wraps USB hub information
*/
struct XsUsbHubInfo {
#ifdef __cplusplus
	/*! \brief Default constructor

	  \param hubid an optional hub identifier to initialize with
	  \sa XsUsbHubInfo_construct
	 */
	explicit XsUsbHubInfo(XsHubIdentifier hubid = 0)
		: m_hub(0)
	{
		if (hubid)
			XsUsbHubInfo_construct(this, hubid);
	}

	/*! \brief Destructor \sa XsUsbHubInfo_destruct */
	~XsUsbHubInfo()
	{
		XsUsbHubInfo_destruct(this);
	}

	/*! \brief Copy constructor
	  \param other the object to copy
	  \sa XsUsbHubInfo_copy \sa XsUsbHubInfo_construct
	 */
	XsUsbHubInfo(const XsUsbHubInfo &other)
		: m_hub(0)
	{
		if (other.m_hub)
			XsUsbHubInfo_construct(this, other.m_hub);
	}

	/*! \brief Assigns \a other to this XsUsbHubInfo
	   \param other the object to copy
	   \returns a const reference to this info object
	   \sa XsUsbHubInfo_copy
	 */
	const XsUsbHubInfo& operator=(const XsUsbHubInfo &other)
	{
		if (this != &other)
			XsUsbHubInfo_copy(this, &other);
		return *this;
	}

	/*! \brief \copybrief XsUsbHubInfo_parentPathMatches
	 * \param other the object to compare to
	 * \returns true if the two objects share the same immediate parent hub, false otherwise
	 * \sa XsUsbHubInfo_parentPathMatches
	 */
	bool parentPathMatches(const XsUsbHubInfo &other) const
	{
		return 0 != XsUsbHubInfo_parentPathMatches(this, &other);
	}

	/*! \brief Returns true if a valid hub is set
	*/
	bool isValid() const
	{
		return m_hub != 0;
	}

	/*! \brief Return the hub identifier
	*/
	inline XsHubIdentifier hub() const
	{
		return m_hub;
	}

private:
//! \protectedsection
#endif
	XsHubIdentifier m_hub;		//!< The identifier of the USB hub
};
typedef struct XsUsbHubInfo XsUsbHubInfo;

#endif	// file guard
