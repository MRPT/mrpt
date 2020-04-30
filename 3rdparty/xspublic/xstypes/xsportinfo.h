
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

#ifndef XSPORTINFO_H
#define XSPORTINFO_H

#include "xstypesconfig.h"
#include "pstdint.h"
#include "xsdeviceid.h"
#include "xsbaud.h"
#include "xsstring.h"
#include <stdio.h>

#define XSENS_VENDOR_ID				0x2639
#define FTDI_VENDOR_ID				0x0403 // needed for Xsens USB-serial converters

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
XSTYPES_DLL_API int XsPortInfo_isNetwork(XsPortInfo const* thisPtr);
XSTYPES_DLL_API const char* XsPortInfo_networkServiceName(XsPortInfo const* thisPtr);
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

typedef enum XsPortLinesOptions
{
	XPLO_Invalid	= 0,

	XPLO_RTS_Set	= 1,
	XPLO_RTS_Clear	= (1 << 1),
	XPLO_RTS_Ignore	= (1 << 2),

	XPLO_DTR_Set	= (1 << 3),
	XPLO_DTR_Clear	= (1 << 4),
	XPLO_DTR_Ignore	= (1 << 5),

	XPLO_All_Set	= XPLO_RTS_Set | XPLO_DTR_Set,
	XPLO_All_Clear	= XPLO_RTS_Clear | XPLO_DTR_Clear,
	XPLO_All_Ignore	= XPLO_RTS_Ignore | XPLO_DTR_Ignore,

	XPLO_RtsCtsFlowControl = (1 << 6)
} XsPortLinesOptions;

struct XsPortInfo {
#ifdef __cplusplus
	/*! \brief Default constructor, creates an empty port info object */
	XsPortInfo()
		: m_deviceId(0)
		, m_baudrate(XBR_Invalid)
		, m_linesOptions(XPLO_All_Ignore)
		, m_vid(0)
		, m_pid(0)
	{
		m_portName[0] = '\0';
	}

	/*! \brief Named constructor, initializes the object to the supplied \a portname and optional \a baudRate
		\param portname The name of the port, maximum 255 characters
		\param baudRate The baud rate to configure for the port, for scanning XBR_Invalid may be used to scan all known baud rates
		\param linesOptions The options for the hardware flow control lines
	*/
	explicit XsPortInfo(const XsString& portname, XsBaudRate baudRate = XBR_Invalid, XsPortLinesOptions linesOptions = XPLO_All_Ignore)
		: m_deviceId(0)
		, m_baudrate(baudRate)
		, m_linesOptions(linesOptions)
		, m_vid(0)
		, m_pid(0)
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
		\param linesOptions The options for the hardware flow control lines
		\note Numbered COM ports are only available on Windows platforms.
	*/
	explicit XsPortInfo(int portNr, XsBaudRate baudRate = XBR_Invalid, XsPortLinesOptions linesOptions = XPLO_All_Ignore)
		: m_deviceId(0)
		, m_baudrate(baudRate)
		, m_linesOptions(linesOptions)
		, m_vid(0)
		, m_pid(0)
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

	/*! \brief \copybrief XsPortInfo_isNetwork */
	inline bool isNetwork() const
	{
		return XsPortInfo_isNetwork(this) != 0;
	}

	/*! \brief \copybrief XsPortInfo_networkServiceName */
	inline XsString networkServiceName() const
	{
		return XsString(XsPortInfo_networkServiceName(this));
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

	/*! \brief The options for the hw flow control lines
	*/
	inline XsPortLinesOptions linesOptions() const
	{
		return m_linesOptions;
	}

	/*! \brief Set the options for the hw flow control lines
	*/
	inline void setLinesOptions(XsPortLinesOptions linesOptions)
	{
		m_linesOptions = linesOptions;
	}

	/*! \brief Get the vid and pid of this portinfo
	*/
	inline void getVidPid(uint16_t& vid, uint16_t& pid) const
	{
		vid = m_vid;
		pid = m_pid;
	}

	/*! \brief Set the vid and pid
	*/
	inline void setVidPid(uint16_t vid, uint16_t pid)
	{
		m_vid = vid;
		m_pid = pid;
	}

private:
#endif

	XsDeviceId m_deviceId;				//!< The device Id of main Xsens device detected on the port
	char m_portName[256];				//!< The port name
	XsBaudRate m_baudrate;				//!< The baudrate at which an Xsens device was detected, may be XBR_Invalid for pure USB ports
	XsPortLinesOptions m_linesOptions;	//!< The hardware flow control lines options for the port
	uint16_t m_vid, m_pid;				//!< The USB Vendor Id and Hardware Id of this connection, when available
};

#if defined(_MSC_VER)
#pragma warning(pop)
#endif

#if defined(__cplusplus) && !defined(XSENS_NO_STL)
#include <ostream>
namespace std
{
	template<typename _CharT, typename _Traits>
	basic_ostream<_CharT, _Traits>& operator<<(basic_ostream<_CharT, _Traits>& o, XsPortInfo const& xpi)
	{
		if (xpi.isUsb())
			o << "usb ";
		o << "port " << xpi.portName();
		if (xpi.baudrate() != XBR_Invalid)
			o << " at " << xpi.baudrate() << " bps";
		if (xpi.deviceId() != 0)
			o << " (" << xpi.deviceId() << ")";
		return o;
	}
}

#endif


#endif
