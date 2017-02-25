/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSPORTINFO_H
#define XSPORTINFO_H

#include "xstypesconfig.h"
#include "pstdint.h"
#include "xsdeviceid.h"
#include "xsbaud.h"
#include "xsstring.h"
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef __cplusplus
	typedef struct XsPortInfo XsPortInfo;
#else
	struct XsPortInfo;
#endif

XSTYPES_DLL_API void XsPortInfo_clear(XsPortInfo* thisPtr);
XSTYPES_DLL_API int XsPortInfo_empty(XsPortInfo const* thisPtr);
XSTYPES_DLL_API int XsPortInfo_portNumber(XsPortInfo const* thisPtr);
XSTYPES_DLL_API int XsPortInfo_isUsb(XsPortInfo const* thisPtr);
XSTYPES_DLL_API int XsPortInfo_usbBus(XsPortInfo const* thisPtr);
XSTYPES_DLL_API int XsPortInfo_usbAddress(XsPortInfo const* thisPtr);
XSTYPES_DLL_API void XsPortInfo_swap(XsPortInfo* a, struct XsPortInfo* b);

#ifdef __cplusplus
}
#endif

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif

struct XsPortInfo {
#ifdef __cplusplus
	/*! \brief Default constructor, creates an empty port info object */
	XsPortInfo()
		: m_deviceId(0)
		, m_baudrate(XBR_Invalid)
	{
		m_portName[0] = '\0';
	}

	/*! \brief Named constructor, initializes the object to the supplied \a portname and optional \a baudRate
		\param portname The name of the port, maximum 255 characters
		\param baudRate The baud rate to configure for the port, for scanning XBR_Invalid may be used to scan all known baud rates
	*/
	explicit XsPortInfo(const XsString& portname, XsBaudRate baudRate = XBR_Invalid)
		: m_deviceId(0)
		, m_baudrate(baudRate)
	{
		if (portname.size() < 255)
			strcpy(m_portName, portname.c_str());
		else
			m_portName[0] = '\0';
	}

#ifndef XSENS_NO_PORT_NUMBERS
	/*! \brief Port number constructor, initializes the port to have COM<portNumber> as its name and the optional \a baudRate for a baud rate
		\param portNr The number of the COM port
		\param baudRate The baud rate to configure for the port, for scanning XBR_Invalid may be used to scan all known baud rates
		\note Numbered COM ports are only available on Windows platforms.
	*/
	explicit XsPortInfo(int portNr, XsBaudRate baudRate = XBR_Invalid)
		: m_deviceId(0)
		, m_baudrate(baudRate)
	{
		sprintf(m_portName, "COM%d", portNr);
	}
#endif

	/*! \brief \copybrief XsPortInfo_clear */
	inline void clear()
	{
		XsPortInfo_clear(this);
	}

	/*! \brief \copybrief XsPortInfo_empty */
	inline bool empty() const
	{
		return XsPortInfo_empty(this) != 0;
	}

	/*! \brief greater than operator, used for sorting the list. */
	inline bool operator > (const XsPortInfo& p) const { return strcmp(m_portName, p.m_portName) > 0; }

	/*! \brief less than operator, used for sorting the list. */
	inline bool operator < (const XsPortInfo& p) const { return strcmp(m_portName, p.m_portName) < 0; }

	/*! \brief equality operator, used for finding items in a list. */
	inline bool operator == (const XsPortInfo& p) const { return strcmp(m_portName, p.m_portName) == 0; }

	/*! \brief equality operator, used for finding items in a list. */
	inline bool operator == (const char *port) const { return strcmp(m_portName, port) == 0; }

	/*! \copydoc XsPortInfo_portNumber */
	inline int portNumber() const
	{
		return XsPortInfo_portNumber(this);
	}

	/*! \brief The port name
	*/
	inline XsString portName() const
	{
		return XsString(m_portName);
	}

	/*! \brief Set the port name
	*/
	inline void setPortName(const XsString& portName_)
	{
		strncpy(m_portName, portName_.c_str(), 256);
	}

	/*! \brief \copybrief XsPortInfo_isUsb */
	inline bool isUsb() const
	{
		return XsPortInfo_isUsb(this) != 0;
	}

	/*! \copydoc XsPortInfo_usbBus */
	inline int usbBus() const
	{
		return XsPortInfo_usbBus(this);
	}

	/*! \copydoc XsPortInfo_usbAddress */
	inline int usbAddress() const
	{
		return XsPortInfo_usbAddress(this);
	}

	/*! \brief The baudrate
	*/
	inline XsBaudRate baudrate() const
	{
		return m_baudrate;
	}

	/*! \brief Set the baudrate
	*/
	inline void setBaudrate(XsBaudRate baudrate_)
	{
		m_baudrate = baudrate_;
	}

	/*! \brief The device ID
	*/
	inline XsDeviceId deviceId() const
	{
		return m_deviceId;
	}

	/*! \brief Set the device ID
	*/
	inline void setDeviceId(XsDeviceId deviceId_)
	{
		m_deviceId = deviceId_;
	}

private:
#endif

	XsDeviceId m_deviceId;	//!< The device Id of main Xsens device detected on the port
	char m_portName[256];	//!< The port name
	XsBaudRate m_baudrate;	//!< The baudrate at which an Xsens device was detected, may be XBR_Invalid for pure USB ports
};

#if defined(_MSC_VER)
#pragma warning(pop)
#endif

#endif // file guard
