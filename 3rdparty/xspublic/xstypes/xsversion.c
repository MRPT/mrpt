
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

#include "xsversion.h"
#include "xsstring.h"
#include <stdio.h>

/*! \class XsVersion
	\brief A class to store version information
*/

/*! \addtogroup cinterface C Interface
	@{
*/

/*! \relates XsVersion \brief Test if this is a null-version. */
int XsVersion_empty(const XsVersion* thisPtr)
{
	return thisPtr->m_major == 0 && thisPtr->m_minor == 0 && thisPtr->m_revision == 0;
}

/*! \relates XsVersion \brief Get a string with the version expressed in a readable format. */
void XsVersion_toString(const XsVersion* thisPtr, XsString* version)
{
	char buffer[256];
	int chars;

	if (thisPtr->m_build != 0 && thisPtr->m_reposVersion != 0)
		chars = sprintf(buffer, "%d.%d.%d build %d rev %d", thisPtr->m_major, thisPtr->m_minor, thisPtr->m_revision, thisPtr->m_build, thisPtr->m_reposVersion);
	else
		chars = sprintf(buffer, "%d.%d.%d", thisPtr->m_major, thisPtr->m_minor, thisPtr->m_revision);

	XsString_assign(version, (XsSize)(ptrdiff_t)chars, buffer);
	if (thisPtr->m_extra.m_size != 0)
	{
		const char space = ' ';
		XsArray_insert(version, version->m_size-1, 1, &space);
		XsString_append(version, &thisPtr->m_extra);
	}
}

/*!
 *	\relates XsVersion
 *	\brief Get a string with the version expressed in a readable format.
 */
void XsVersion_toSimpleString(const XsVersion* thisPtr, XsString* version)
{
	char buffer[256];
	int chars = sprintf(buffer, "%d.%d.%d", thisPtr->m_major, thisPtr->m_minor, thisPtr->m_revision);
	XsString_assign(version, (XsSize)(ptrdiff_t)chars, buffer);
}

/*!
 *	\relates XsVersion
 *	\brief Set the version to the values in the string
 */
void XsVersion_fromString(XsVersion* thisPtr, const XsString* version)
{
	int major = 0;
	int minor = 0;
	int revision = 0;
	int build = 0;
	int reposVersion = 0;
	int result;
	size_t count = 0;

	assert(thisPtr);
	thisPtr->m_major = 0;
	thisPtr->m_minor = 0;
	thisPtr->m_revision = 0;
	thisPtr->m_build = 0;
	thisPtr->m_reposVersion = 0;
	XsString_resize(&thisPtr->m_extra, 0);
	if (!version || XsString_empty(version))
		return;

	result = sscanf(version->m_data, "%d.%d.%d build %d rev %d%zn", &major, &minor, &revision, &build, &reposVersion, &count);

	if (result > 0)
	{
		thisPtr->m_major = major;
		thisPtr->m_minor = minor;
		thisPtr->m_revision = revision;
	}

	if (result > 3)
	{
		thisPtr->m_build = build;
		thisPtr->m_reposVersion = reposVersion;
	}

	if ((result == 5) && ((count + 1) < version->m_size))
		XsString_assignCharArray(&thisPtr->m_extra, &version->m_data[count + 1]);
}

/*! \relates XsVersion
 *	\brief Create a XsVersion a XsSimpleVersion, \a simpleVersion.
 */
void XsVersion_fromSimpleVersion(XsVersion* thisPtr, const XsSimpleVersion* simpleVersion)
{
	thisPtr->m_major = simpleVersion->m_major;
	thisPtr->m_minor = simpleVersion->m_minor;
	thisPtr->m_revision = simpleVersion->m_revision;
	thisPtr->m_build = 0;
	thisPtr->m_reposVersion = 0;
	XsString_resize(&thisPtr->m_extra, 0);
}

/*! \relates XsVersion
 *	\brief Create a XsSimpleVersion (\a version) from a XsVersion.
 */
void XsVersion_toSimpleVersion(const XsVersion* thisPtr, XsSimpleVersion* simpleVersion)
{
	simpleVersion->m_major = (uint8_t)(int8_t)thisPtr->m_major;
	simpleVersion->m_minor = (uint8_t)(int8_t)thisPtr->m_minor;
	simpleVersion->m_revision = (uint8_t)(int8_t)thisPtr->m_revision;
}

/*! @} */
