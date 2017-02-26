/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "xsdeviceid.h"
#include "xsstring.h"
#include "xsdid.h"
#include <stdio.h>

/*! \class XsDeviceId
	\brief Contains an Xsens device ID and provides operations for determining the type of device
*/

/*! \addtogroup cinterface C Interface
	@{
*/

/*!	\relates XsDeviceId
	\brief Returns the value of a broadcast deviceID.
*/
uint32_t XsDeviceId_broadcast(void)
{
	return XS_DID_BROADCAST;
}

/*!	\relates XsDeviceId
	\brief Test if the device ID is a valid id (not 0).
*/
int XsDeviceId_isValid(const XsDeviceId* thisPtr)
{
	return thisPtr->m_deviceId != 0;
}

/*! \relates XsDeviceId
	\brief Test if this device ID represents an Xbus Master. */
int XsDeviceId_isXbusMaster(const XsDeviceId* thisPtr)
{
	return ((thisPtr->m_deviceId & XS_DID_TYPEH_MASK) == XS_DID_TYPEH_XM);
}

/*! \relates XsDeviceId
	\brief Test if this device ID represents a Wireless Master device (Awinda Station, Awinda Dongle, Awinda OEM). */
int XsDeviceId_isWirelessMaster(const XsDeviceId* thisPtr)
{
	return ((thisPtr->m_deviceId & XS_DID_TYPEH_MASK) == XS_DID_TYPEH_AWINDAMASTER);
}

/*! \relates XsDeviceId
	\brief Test if this device ID represents an Awinda Station.
*/
int XsDeviceId_isAwindaStation(const XsDeviceId* thisPtr)
{
	return XS_DID_AWINDA_STATION(thisPtr->m_deviceId);
}

/*! \relates XsDeviceId
	\brief Test if this device ID represents an Awinda Dongle. */
int XsDeviceId_isAwindaDongle(const XsDeviceId* thisPtr)
{
	return XS_DID_AWINDA_DONGLE(thisPtr->m_deviceId);
}

/*! \relates XsDeviceId
	\brief Test if this device ID represents an Awinda OEM board. */
int XsDeviceId_isAwindaOem(const XsDeviceId* thisPtr)
{
	return XS_DID_AWINDA_OEM(thisPtr->m_deviceId);
}

/*! \relates XsDeviceId
	\brief Test if this device ID represents an MTw. */
int XsDeviceId_isMtw(const XsDeviceId* thisPtr)
{
	return XS_DID_WMMT(thisPtr->m_deviceId);
}

/*! \relates XsDeviceId
	\brief Test if this device ID represents an Xbus Master Motion Tracker. */
int XsDeviceId_isXbusMasterMotionTracker(const XsDeviceId* thisPtr)
{
	return ((thisPtr->m_deviceId & XS_DID_TYPE_MASK) == XS_DID_TYPE_MTX_XBUS);
}

/*! \relates XsDeviceId
	\brief Test if this device ID represents an MTi or an MTx. */
int XsDeviceId_isMtix(const XsDeviceId* thisPtr)
{
	return (((thisPtr->m_deviceId & XS_DID_TYPEH_MASK) == XS_DID_TYPEH_MTI_MTX) && !XsDeviceId_isMtw(thisPtr));
}

/*! \relates XsDeviceId
	\brief Test if this device ID represents a legacy MTi-G. */
int XsDeviceId_isLegacyMtig(const XsDeviceId* thisPtr)
{
	return ((thisPtr->m_deviceId & XS_DID_TYPEH_MASK) == XS_DID_TYPEH_MTIG);
}

/*! \relates XsDeviceId
	\brief Test if this device ID represents an MTi-G. */
int XsDeviceId_isMtig(const XsDeviceId* thisPtr)
{
	return (((thisPtr->m_deviceId & XS_DID_TYPEH_MASK) == XS_DID_TYPEH_MTIG) || ((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_700));
}

/*! \relates XsDeviceId
	\brief Test if this device ID represents an MTMk4. */
int XsDeviceId_isMtMk4(const XsDeviceId* thisPtr)
{
	return ( ((thisPtr->m_deviceId & XS_DID_TYPEH_MASK) == XS_DID_TYPEH_MT_X0) ||
			 ((thisPtr->m_deviceId & XS_DID_TYPEH_MASK) == XS_DID_TYPEH_MT_X00) );
}

/*! \relates XsDeviceId
	\brief Test if this device ID represents an MTMk4 10 series. */
int XsDeviceId_isMtMk4_X0(const struct XsDeviceId* thisPtr)
{
	return ((thisPtr->m_deviceId & XS_DID_TYPEH_MASK) == XS_DID_TYPEH_MT_X0); 
}

/*! \relates XsDeviceId
	\brief Test if this device ID represents an MTMk4 10. */
int XsDeviceId_isMtMk4_10(const struct XsDeviceId* thisPtr)
{
	return ((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_10); 
}

/*! \relates XsDeviceId
	\brief Test if this device ID represents an MTMk4 20. */
int XsDeviceId_isMtMk4_20(const struct XsDeviceId* thisPtr)
{
	return ((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_20); 
}

/*! \relates XsDeviceId
	\brief Test if this device ID represents an MTMk4 30. */
int XsDeviceId_isMtMk4_30(const struct XsDeviceId* thisPtr)
{
	return ((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_30); 
}

/*! \relates XsDeviceId
	\brief Test if this device ID represents an MTMk4 100 series. */
int XsDeviceId_isMtMk4_X00(const struct XsDeviceId* thisPtr)
{
	return ((thisPtr->m_deviceId & XS_DID_TYPEH_MASK) == XS_DID_TYPEH_MT_X00); 
}

/*! \relates XsDeviceId
	\brief Test if this device ID represents an MTMk4 100. */
int XsDeviceId_isMtMk4_100(const struct XsDeviceId* thisPtr)
{
	return ((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_100); 
}

/*! \relates XsDeviceId
	\brief Test if this device ID represents an MTMk4 200. */
int XsDeviceId_isMtMk4_200(const struct XsDeviceId* thisPtr)
{
	return ((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_200); 
}

/*! \relates XsDeviceId
	\brief Test if this device ID represents an MTMk4 300. */
int XsDeviceId_isMtMk4_300(const struct XsDeviceId* thisPtr)
{
	return ((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_300); 
}

/*! \relates XsDeviceId
	\brief Test if this device ID represents an MTMk4 700. */
int XsDeviceId_isMtMk4_700(const struct XsDeviceId* thisPtr)
{
	return ((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_700); 
}

/*! \relates XsDeviceId
	\brief Test if this device ID represents an MTw2 */
int XsDeviceId_isMtw2(const struct XsDeviceId* thisPtr)
{
	return XS_DID_MTW2(thisPtr->m_deviceId);
}

/*! \relates XsDeviceId
	\brief Test if this device ID represents an MTx2 */
int XsDeviceId_isMtx2(const struct XsDeviceId* thisPtr)
{	
	return (((thisPtr->m_deviceId & XS_DID_TYPEH_MASK) == XS_DID_TYPEH_MTX2_MTW2) && !XsDeviceId_isMtw2(thisPtr));
}

/*!	\relates XsDeviceId
	\brief Test if the device ID has the broadcast bit set
*/
int XsDeviceId_containsBroadcast(const XsDeviceId* thisPtr)
{
	return ((thisPtr->m_deviceId & XS_DID_BROADCAST) != 0);
}

/*! \relates XsDeviceId
	\brief Test if this device ID \e is \e the broadcast id.
*/
int XsDeviceId_isBroadcast(const XsDeviceId* thisPtr)
{
	return (thisPtr->m_deviceId == XS_DID_BROADCAST);
}

/*! \relates XsDeviceId
	\brief Get a string with a readable representation of this device ID. */
void XsDeviceId_toString(const XsDeviceId* thisPtr, XsString* str)
{
	char device[9];
	sprintf(device, "%08X", thisPtr->m_deviceId);	//lint !e534
	XsString_assign(str, 8, device);
}

/*! \brief Swap the contents of \a a with those of \a b
*/
void XsDeviceId_swap(XsDeviceId* a, XsDeviceId* b)
{
	uint32_t tmp = a->m_deviceId;
	a->m_deviceId = b->m_deviceId;
	b->m_deviceId = tmp;
}

/*! @} */
