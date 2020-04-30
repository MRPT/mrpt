
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

#include "xsusbhubinfo.h"
#include <string.h>
#include <stdlib.h>

/*! \addtogroup cinterface C Interface
	@{
*/

/*! \typedef XsHubIdentifier
  On Windows: \code
	typedef int XsHubIdentifier;
  \endcode


  On non-windows: \code
	typedef const char* XsHubIdentifier;
  \endcode
*/

/*! \brief Initialize the %XsUsbHubInfo with \a hub

  \param[in] hub the hub identifier to initialize with
*/
void XsUsbHubInfo_construct(XsUsbHubInfo* thisPtr, XsHubIdentifier hub)
{
#ifdef _WIN32
	thisPtr->m_hub = hub;
#else
	if (hub)
		thisPtr->m_hub = strdup(hub);
	else
		thisPtr->m_hub = 0;
#endif
}

/*! \brief Destroy the %XsUsbHubInfo
*/
void XsUsbHubInfo_destruct(XsUsbHubInfo* thisPtr)
{
#ifndef _WIN32
	if (thisPtr->m_hub)
		free((void*) thisPtr->m_hub);
#endif
	thisPtr->m_hub = 0;
}

/*! \brief Assign a new hub identifier to the %XsUsbHubInfo
	\param[in] hub the hub identifier
*/
void XsUsbHubInfo_assign(XsUsbHubInfo* thisPtr, XsHubIdentifier hub)
{
	XsUsbHubInfo_destruct(thisPtr);
	XsUsbHubInfo_construct(thisPtr, hub);
}

/*! \brief Copy the contents of the %XsUsbHubInfo to \a copy
*/
void XsUsbHubInfo_copy(XsUsbHubInfo* copy, XsUsbHubInfo const* src)
{
	XsUsbHubInfo_assign(copy, src->m_hub);
}

/*! \brief Swap the two %XsUsbHubInfo items
*/
void XsUsbHubInfo_swap(XsUsbHubInfo* thisPtr, XsUsbHubInfo* thatPtr)
{
	XsHubIdentifier tmp;
	if (thisPtr == thatPtr)
		return;

	tmp = thisPtr->m_hub;
	thisPtr->m_hub = thatPtr->m_hub;
	thatPtr->m_hub = tmp;
}

/*! \brief Returns true if the two hub info objects share the same device path

  \param[in] left the left hand side to compare
  \param[in] right the right hand side to compare

  \returns non-zero if the two hubs share the same parent
*/
int XsUsbHubInfo_parentPathMatches(const XsUsbHubInfo* left, const XsUsbHubInfo* right)
{
#ifdef _WIN32
	return left->m_hub == right->m_hub && left->m_hub != 0;
#else
	int dotPos, tmp;
	XsHubIdentifier otherLastDot;
	XsHubIdentifier thisLastDot;

	if (!right->m_hub || !left->m_hub)
		return 0;
	if (left == right)
		return 1;

	otherLastDot = strrchr(right->m_hub, '.');
	thisLastDot = strrchr(left->m_hub, '.');

	dotPos = (int) (otherLastDot - right->m_hub);
	tmp = (int) (thisLastDot - left->m_hub);

	if (dotPos != tmp)
		return 0;

	return !strncmp(right->m_hub, left->m_hub, dotPos);
#endif
}

/*! @} */
