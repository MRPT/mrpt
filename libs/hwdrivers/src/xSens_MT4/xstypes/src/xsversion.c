/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
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
	size_t chars;

	if (XsVersion_empty(thisPtr))
		return;

	if (thisPtr->m_build != 0)
		chars = sprintf(buffer, "%d.%d.%d build %d", thisPtr->m_major, thisPtr->m_minor, thisPtr->m_revision, thisPtr->m_build);
	else
		chars = sprintf(buffer, "%d.%d.%d", thisPtr->m_major, thisPtr->m_minor, thisPtr->m_revision);
	XsString_assign(version, chars, buffer);
	if (thisPtr->m_extra.m_size != 0)
	{
		const char space = ' ';
		XsArray_insert(version, version->m_size-2, 1, &space);	//lint !e64
		XsString_append(version, &thisPtr->m_extra);
	}
}

/*! @} */
