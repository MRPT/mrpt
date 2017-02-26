/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSDEVICEID_H
#define XSDEVICEID_H

#include "xstypesconfig.h"
#include "pstdint.h"
#include "xsstring.h"
#ifdef __cplusplus
extern "C" {
#else
#define XSDEVICEID_INITIALIZER	{ 0 }
#endif

struct XsDeviceId;

XSTYPES_DLL_API void XsDeviceId_toString(struct XsDeviceId const* thisPtr, XsString* str);
XSTYPES_DLL_API int XsDeviceId_isValid(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isXbusMaster(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isXbusMasterMotionTracker(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isWirelessMaster(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtw(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtix(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtig(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isLegacyMtig(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isAwindaStation(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isAwindaDongle(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isAwindaOem(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk4(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk4_X0(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk4_10(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk4_20(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk4_30(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk4_X00(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk4_100(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk4_200(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk4_300(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk4_700(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtw2(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtx2(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_containsBroadcast(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isBroadcast(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API uint32_t XsDeviceId_broadcast(void);
XSTYPES_DLL_API void XsDeviceId_swap(struct XsDeviceId* a, struct XsDeviceId* b);

#ifdef __cplusplus
} // extern "C"
#endif

struct XsDeviceId {
#ifdef __cplusplus
	/*! \brief Constructor that creates an XsDeviceId from the supplied \a deviceId */
	inline XsDeviceId(uint32_t deviceId = 0)
		: m_deviceId(deviceId)
	{
	}
	/*! \brief Constructor that creates an XsDeviceId from the supplied XsDeviceId \a other */
	inline XsDeviceId(const XsDeviceId& other)
		: m_deviceId(other.m_deviceId)
	{
	}

	/*! \brief Assign the \a other deviceId to this XsDeviceId */
	inline const XsDeviceId& operator=(const XsDeviceId& other)
	{
		m_deviceId = other.m_deviceId;
		return *this;
	}
	/*! \brief Assign the \a deviceId to this XsDeviceId */
	inline const XsDeviceId& operator=(uint32_t deviceId)
	{
		m_deviceId = deviceId;
		return *this;
	}

	/*! \brief Returns the deviceId as an unsigned integer */
	inline uint32_t toInt() const
	{
		return m_deviceId;
	}

	/*! \brief Returns the deviceId as an XsString */
	inline XsString toString() const
	{
		XsString tmp;
		XsDeviceId_toString(this, &tmp);
		return tmp;
	}

	/*! \brief \copybrief XsDeviceId_isValid */
	inline bool isValid() const
	{
		return 0 != XsDeviceId_isValid(this);
	}
	/*! \brief \copybrief XsDeviceId_isXbusMaster */
	inline bool isXbusMaster() const
	{
		return 0 != XsDeviceId_isXbusMaster(this);
	}
	/*! \brief \copybrief XsDeviceId_isXbusMasterMotionTracker */
	inline bool isXbusMasterMotionTracker() const
	{
		return 0 != XsDeviceId_isXbusMasterMotionTracker(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtw */
	inline bool isMtw() const
	{
		return 0 != XsDeviceId_isMtw(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtix */
	inline bool isMtix() const
	{
		return 0 != XsDeviceId_isMtix(this);
	}
	/*! \brief \copybrief XsDeviceId_isLegacyMtig */
	inline bool isLegacyMtig() const
	{
		return 0 != XsDeviceId_isLegacyMtig(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtig */
	inline bool isMtig() const
	{
		return 0 != XsDeviceId_isMtig(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk4 */
	inline bool isMtMk4() const
	{
		return 0 != XsDeviceId_isMtMk4(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk4_X0 */
	inline bool isMtMk4_X0() const
	{
		return 0 != XsDeviceId_isMtMk4_X0(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk4_10 */
	inline bool isMtMk4_10() const
	{
		return 0 != XsDeviceId_isMtMk4_10(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk4_20 */
	inline bool isMtMk4_20() const
	{
		return 0 != XsDeviceId_isMtMk4_20(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk4_30 */
	inline bool isMtMk4_30() const
	{
		return 0 != XsDeviceId_isMtMk4_30(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk4_X00 */
	inline bool isMtMk4_X00() const
	{
		return 0 != XsDeviceId_isMtMk4_X00(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk4_100 */
	inline bool isMtMk4_100() const
	{
		return 0 != XsDeviceId_isMtMk4_100(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk4_200 */
	inline bool isMtMk4_200() const
	{
		return 0 != XsDeviceId_isMtMk4_200(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk4_300 */
	inline bool isMtMk4_300() const
	{
		return 0 != XsDeviceId_isMtMk4_300(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk4_700 */
	inline bool isMtMk4_700() const
	{
		return 0 != XsDeviceId_isMtMk4_700(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtw2 */
	inline bool isMtw2() const
	{
		return 0 != XsDeviceId_isMtw2(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtx2 */
	inline bool isMtx2() const
	{
		return 0 != XsDeviceId_isMtx2(this);
	}
	/*! \brief \copybrief XsDeviceId_isWirelessMaster */
	inline bool isWirelessMaster() const
	{
		return 0 != XsDeviceId_isWirelessMaster(this);
	}
	/*! \brief \copybrief XsDeviceId_isAwindaStation */
	inline bool isAwindaStation() const
	{
		return 0 != XsDeviceId_isAwindaStation(this);
	}
	/*! \brief \copybrief XsDeviceId_isAwindaDongle */
	inline bool isAwindaDongle() const
	{
		return 0 != XsDeviceId_isAwindaDongle(this);
	}
	/*! \brief \copybrief XsDeviceId_isAwindaOem */
	inline bool isAwindaOem() const
	{
		return 0 != XsDeviceId_isAwindaOem(this);
	}
	/*! \brief \copybrief XsDeviceId_containsBroadcast */
	inline bool containsBroadcast() const
	{
		return 0 != XsDeviceId_containsBroadcast(this);
	}
	/*! \brief \copybrief XsDeviceId_isBroadcast */
	inline bool isBroadcast() const
	{
		return 0 != XsDeviceId_isBroadcast(this);
	}

	/*! \brief Returns true if the \a other deviceId matches this deviceId */
	inline bool operator==(const XsDeviceId& other) const { return m_deviceId == other.m_deviceId; }
	/*! \brief Returns true if the \a other deviceId does not match this deviceId */
	inline bool operator!=(const XsDeviceId& other) const { return m_deviceId != other.m_deviceId; }
	/*! \brief Returns true if this deviceId is less than the \a other deviceId */
	inline bool operator<(const XsDeviceId& other) const { return m_deviceId < other.m_deviceId; }
	/*! \brief Returns true if this deviceId is less or equal to the \a other deviceId */
	inline bool operator<=(const XsDeviceId& other) const { return m_deviceId <= other.m_deviceId; }
	/*! \brief Returns true if this deviceId is larger than the \a other deviceId */
	inline bool operator>(const XsDeviceId& other) const { return m_deviceId > other.m_deviceId; }
	/*! \brief Returns true if this deviceId is larger or equal to the \a other deviceId */
	inline bool operator>=(const XsDeviceId& other) const { return m_deviceId >= other.m_deviceId; }

	/*! \brief Creates and returns a XsDeviceId representing the broadcast deviceId */
	static XsDeviceId broadcast()
	{
		return XsDeviceId(XsDeviceId_broadcast());
	}

private:
#endif
	uint32_t m_deviceId;	//!< The actual device id
};

typedef struct XsDeviceId XsDeviceId;

#endif // file guard
