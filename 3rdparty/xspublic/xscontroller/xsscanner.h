
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

#ifndef XSSCANNER_H
#define XSSCANNER_H

#include "xscontrollerconfig.h"
#include <xstypes/xsbaud.h>

//AUTO namespace xstypes {
struct XsPortInfoArray;
//AUTO }

//AUTO namespace xscontroller {
struct XsUsbHubInfo;
//AUTO }

#ifdef __cplusplus
#include <xstypes/xsportinfoarray.h>
#include <xstypes/xsintarray.h>
#include <xstypes/xsstringarray.h>
#include "xsusbhubinfo.h"
#include <xstypes/xsstring.h>
#include <sstream>
extern "C" {
#endif
struct XsPortInfo;

//! \brief Defines the callback type that can be supplied to XsScanner_setScanLogCallback
typedef void (*XsScanLogCallbackFunc)(struct XsString const*);

XDA_DLL_API void XsScanner_scanPorts(struct XsPortInfoArray* ports, XsBaudRate baudrate, int singleScanTimeout, int ignoreNonXsensDevices, int detectRs485);
XDA_DLL_API int XsScanner_scanPort(struct XsPortInfo* port, XsBaudRate baudrate, int singleScanTimeout, int detectRs485);
XDA_DLL_API void XsScanner_enumerateSerialPorts(struct XsPortInfoArray* ports, int ignoreNonXsensDevices);
XDA_DLL_API void XsScanner_filterResponsiveDevices(struct XsPortInfoArray* ports, XsBaudRate baudrate, int singleScanTimeout, int detectRs485);
XDA_DLL_API void XsScanner_enumerateUsbDevices(struct XsPortInfoArray* ports);
XDA_DLL_API void XsScanner_scanUsbHub(struct XsUsbHubInfo* hub, const struct XsPortInfo* port);
XDA_DLL_API void XsScanner_enumerateNetworkDevices(struct XsPortInfoArray* ports);
XDA_DLL_API void XsScanner_abortScan(void);
XDA_DLL_API void XsScanner_setScanLogCallback(XsScanLogCallbackFunc cb);

#ifdef __cplusplus
} // extern "C"

class XsScanner {
public:
	/*!	\brief Scan all ports for Xsens devices.
		\param[in] baudrate The baudrate to scan at. When set to XBR_Invalid, all known baudrates are scanned.
		\param[in] singleScanTimeout The timeout of a scan of a single port at a single baud rate in ms.
		\param[in] ignoreNonXsensDevices When true (the default), only Xsens devices are returned. Otherwise other devices that comply with the Xsens message protocol will also be returned.
		\param[in] detectRs485 Enable more extended scan to detect rs485 devices
		\returns The list of detected ports.
		\sa XsScanner_scanPorts
	*/
	static inline XsPortInfoArray scanPorts(XsBaudRate baudrate = XBR_Invalid, int singleScanTimeout = 100, bool ignoreNonXsensDevices = true, bool detectRs485 = false)
	{
		XsPortInfoArray ports;
		XsScanner_scanPorts(&ports, baudrate, singleScanTimeout, ignoreNonXsensDevices, detectRs485);
		return ports;
	}

	//! \copydoc XsScanner_scanPort
	static inline bool XSNOCOMEXPORT scanPort(XsPortInfo& port, XsBaudRate baudrate = XBR_Invalid, int singleScanTimeout = 100, bool detectRs485 = false)
	{
		return 0 != XsScanner_scanPort(&port, baudrate, singleScanTimeout, detectRs485);
	}

	/*!	\brief Scan a single port for Xsens devices.
		\param[in] portName The name of the port to scan.
		\param[in] baudrate The baudrate to scan at. When set to XBR_Invalid, all known baudrates are scanned.
		\param[in] singleScanTimeout The timeout of a scan at a single baud rate in ms.
		\param[in] detectRs485 Enable more extended scan to detect rs485 devices
		\returns An XsPortInfo structure with the results of the scan.
		\sa XsScanner_scanPort
	*/
	static inline XsPortInfo scanPort(const XsString& portName, XsBaudRate baudrate = XBR_Invalid, int singleScanTimeout = 100, bool detectRs485 = false)
	{
		XsPortInfo pi(portName, baudrate);
		if (scanPort(pi, baudrate, singleScanTimeout, detectRs485))
			return pi;

		return XsPortInfo();
	}

	/*!	\brief Scan a list of Com ports for Xsens devices.
		\param[in] portList The list of port names to scan.
		\param[in] portLinesOptionsList The list of hardware flow control options for the ports to scan.
		\param[in] baudrate The baudrate to scan at. When set to XBR_Invalid, all known baudrates are scanned.
		\param[in] singleScanTimeout The timeout of a scan at a single baud rate in ms.
		\returns An array of XsPortInfo structures with the results of the scan.
		\sa XsScanner_scanCOMPortList
	*/
	static inline XsPortInfoArray scanComPortList(const XsStringArray& portList, const XsIntArray& portLinesOptionsList = XsIntArray(), XsBaudRate baudrate = XBR_Invalid, int singleScanTimeout = 100)
	{
		XsPortInfoArray pInfoArray;

		if (!portLinesOptionsList.empty() && (portLinesOptionsList.size() != portList.size()))
			return pInfoArray;

		for (XsIntArray::size_type idxPort = 0; idxPort < portList.size(); ++idxPort)
		{
			const XsPortLinesOptions portLinesOptions = portLinesOptionsList.empty() ? XPLO_All_Ignore : static_cast<XsPortLinesOptions>(portLinesOptionsList[idxPort]);

			XsPortInfo portInfo(portList[idxPort], baudrate, portLinesOptions);

			if (scanPort(portInfo, baudrate, singleScanTimeout, (portLinesOptions == XPLO_All_Clear))) // XPLO_All_Clear == both RTS/DTR to 0 (RS485).
				pInfoArray.push_back(portInfo);
		}
		return pInfoArray;
	}

	/*!	\brief List all serial ports without scanning
		\param ignoreNonXsensDevices When true (the default), only Xsens ports are returned.
		\returns The list of detected ports.
	*/
	static inline XsPortInfoArray enumerateSerialPorts(bool ignoreNonXsensDevices = true)
	{
		XsPortInfoArray ports;
		XsScanner_enumerateSerialPorts(&ports, ignoreNonXsensDevices);
		return ports;
	}

	/*!	\brief Scan the supplied ports for Xsens devices.
		\details This function does not modify the input list as opposed to XsScanner_filterResponsiveDevices
		\param[in] ports The list of ports to scan.
		\param[in] baudrate The baudrate to scan at. When set to XBR_Invalid, all known baudrates are scanned.
		\param[in] singleScanTimeout The timeout of a scan of a single port at a single baud rate in ms.
		\param[in] detectRs485 Enable more extended scan to detect rs485 devices
		\returns The list of ports that have responsive devices on them.
		\sa XsScanner_filterResponsiveDevices
	*/
	static inline XsPortInfoArray filterResponsiveDevices(const XsPortInfoArray& ports, XsBaudRate baudrate = XBR_Invalid, int singleScanTimeout = 100, bool detectRs485 = false)
	{
		XsPortInfoArray filtered(ports);
		XsScanner_filterResponsiveDevices(&filtered, baudrate, singleScanTimeout, detectRs485);
		return filtered;
	}

	/*!	\brief List all compatible USB ports without scanning.
		\returns The list of detected usb devices.
		\sa XsScanner_enumerateUsbDevices
	*/
	static inline XsPortInfoArray enumerateUsbDevices(void)
	{
		XsPortInfoArray ports;
		XsScanner_enumerateUsbDevices(&ports);
		return ports;
	}

	/*!	\brief Determine the USB hub that \a port is attached to
		\param port The port for which to determine the USB hub.
		\returns The identifier of the hub that \a port is attached to.
		\sa XsScanner_scanUsbHub
	*/
	static inline XsUsbHubInfo scanUsbHub(const XsPortInfo& port)
	{
		XsUsbHubInfo hub;
		XsScanner_scanUsbHub(&hub, &port);
		return hub;
	}

	/*!	\brief List all compatible network devices without scanning.
		\returns The list of detected network services.
		\sa XsScanner_enumerateNetworkDevices
	*/
	static inline XsPortInfoArray enumerateNetworkDevices(void)
	{
		XsPortInfoArray ports;
		XsScanner_enumerateNetworkDevices(&ports);
		return ports;
	}

	/*! \brief Abort the currently running port scan(s)
		\sa XsScanner_abortScan
	*/
	static inline void abortScan(void)
	{
		XsScanner_abortScan();
	}

	/*!	\brief Set a callback function for scan log progress and problem reporting
		\details When set, any scan will use the provided callback function to report progress and failures.
		Normal operation is not affected, so all return values for the scan functions remain valid.
		\param cb The callback function to use. When set to NULL, no callbacks will be generated.
		\sa XsScanner_setScanLogCallback
	*/
	static inline void XSNOCOMEXPORT setScanLogCallback(XsScanLogCallbackFunc cb)
	{
		XsScanner_setScanLogCallback(cb);
	}
};

#endif

#endif
