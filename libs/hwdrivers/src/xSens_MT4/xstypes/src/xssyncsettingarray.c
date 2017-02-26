/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "xssyncsettingarray.h"
#include "xssyncsetting.h"
#include <memory.h>

/*! \struct XsSyncSettingArray
	\brief A list of XsSyncSetting values
	\sa XsArray
*/

//! \copydoc XsArrayDescriptor::itemCopy \note Specialization for XsSyncSetting
void copySyncSetting(XsSyncSetting* to, XsSyncSetting const* from)
{
	*to = *from;
}

//! \copydoc XsArrayDescriptor::itemCompare \note Specialization for XsSyncSetting
int compareSyncSetting(XsSyncSetting const* a, XsSyncSetting const* b)
{
	return memcmp(a, b, sizeof(XsSyncSetting));
}

//! \brief zero the pointer value
void zeroSyncSetting(XsSyncSetting* a)
{
	memset(a, 0, sizeof(XsSyncSetting));
	a->m_line = XSL_Invalid;
}

//! \brief Descriptor for XsSyncSettingArray
XsArrayDescriptor const g_xsSyncSettingArrayDescriptor = {
	//lint --e{64} ignore exact type mismatches here
	sizeof(XsSyncSetting),
	XSEXPCASTITEMSWAP XsSyncSetting_swap,
	XSEXPCASTITEMMAKE zeroSyncSetting,		// construct
	XSEXPCASTITEMCOPY copySyncSetting,		// copy construct
	0,										// destruct
	XSEXPCASTITEMCOPY copySyncSetting,
	XSEXPCASTITEMCOMP compareSyncSetting
};

/*! \copydoc XsArray_construct
	\note Specialization for XsSyncSettingArray
*/
void XsSyncSettingArray_construct(XsSyncSettingArray* thisPtr, XsSize count, XsSyncSetting const* src)
{
	XsArray_construct(thisPtr, &g_xsSyncSettingArrayDescriptor, count, src);
}
