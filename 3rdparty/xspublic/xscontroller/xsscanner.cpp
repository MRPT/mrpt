
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

#include "xsscanner.h"
#include "scanner.h"
#include "enumerateusbdevices.h"

/*!	\class XsScanner
	\brief Provides static functionality for scanning for Xsens devices.
*/

/*!	\brief Set a callback function for scan log progress and problem reporting
	\details When set, any scan will use the provided callback function to report progress and failures.
	Normal operation is not affected, so all return values for the scan functions remain valid.
	\param cb The callback function to use. When set to NULL, no callbacks will be generated.
*/
void XsScanner_setScanLogCallback(XsScanLogCallbackFunc cb)
{
	Scanner::setScanLogCallback(cb);
}

/*! \copydoc XsScanner_scanPorts */
void XsScanner_scanPorts_int(XsPortInfoArray* ports, XsBaudRate baudrate, int singleScanTimeout, int ignoreNonXsensDevices, int detectRs485)
{
	LOGXSSCAN(__FUNCTION__ << " baudrate " << baudrate << " singleScanTimeout " << singleScanTimeout << " ignoreNonXsensDevices " << ignoreNonXsensDevices << " detectRs485 " << detectRs485);

	assert(ports != nullptr);
	if (!ports)
		return;

	XsPortInfoArray tmp;
	Scanner::Accessor accessor;
	accessor.scanner().xsScanPorts(tmp, baudrate, singleScanTimeout, ignoreNonXsensDevices != 0, detectRs485 != 0);
	if (tmp.size())
		ports->assign(tmp.size(), &tmp[0]);
	else
		ports->clear();
}

extern "C" {

	/*!	\relates XsScanner
		\brief Scan all ports for Xsens devices.
		\param[out] ports The list of detected ports.
		\param[in] baudrate The baudrate to scan at. When set to XBR_Invalid, all known baudrates are scanned.
		\param[in] singleScanTimeout The timeout of a scan of a single port at a single baud rate in ms.
		\param[in] ignoreNonXsensDevices When non-zero (the default), only Xsens devices are returned. Otherwise other devices that comply with the Xsens message protocol will also be returned.
		\param[in] detectRs485 Enable more extended scan to detect rs485 devices
	*/
	void XsScanner_scanPorts(XsPortInfoArray* ports, XsBaudRate baudrate, int singleScanTimeout, int ignoreNonXsensDevices, int detectRs485)
	{
		LOGXSSCAN(__FUNCTION__ << " baudrate " << baudrate << " singleScanTimeout " << singleScanTimeout << " ignoreNonXsensDevices " << ignoreNonXsensDevices << " detectRs485 " << detectRs485);
		XsScanner_scanPorts_int(ports, baudrate, singleScanTimeout, ignoreNonXsensDevices, detectRs485);
	}

	/*!	\relates XsScanner
		\brief Scan a single port for Xsens devices.
		\param[in,out] port The name of the port to scan should be in this parameter, the other contents will be filled by the function.
		\param[in] baudrate The baudrate to scan at. When set to XBR_Invalid, all known baudrates are scanned.
		\param[in] singleScanTimeout The timeout of a scan at a single baud rate in ms.
		\param[in] detectRs485 Enable more extended scan to detect rs485 devices
		\returns true if a device was found, false otherwise
	*/
	int XsScanner_scanPort(XsPortInfo* port, XsBaudRate baudrate, int singleScanTimeout, int detectRs485)
	{
		LOGXSSCAN(__FUNCTION__ << " baudrate " << baudrate << " singleScanTimeout " << singleScanTimeout << " detectRs485 " << detectRs485);
		assert(port != nullptr);
		if (!port)
			return 0;
		Scanner::Accessor accessor;
		return accessor.scanner().xsScanPort(*port, baudrate, singleScanTimeout, detectRs485 != 0) ? 1 : 0;
	}

	/*!	\relates XsScanner
		\brief List all serial ports without scanning
		\param[out] ports The list of detected ports.
		\param[in] ignoreNonXsensDevices When non-zero (the default), only Xsens ports are returned.
	*/
	void XsScanner_enumerateSerialPorts_int(XsPortInfoArray* ports, int ignoreNonXsensDevices)
	{
		LOGXSSCAN(__FUNCTION__ << " ignoreNonXsensDevices " << ignoreNonXsensDevices);

		assert(ports != nullptr);
		if (!ports)
			return;

		XsPortInfoArray tmp;
		Scanner::Accessor accessor;
		accessor.scanner().xsEnumerateSerialPorts(tmp, ignoreNonXsensDevices != 0);
		if (tmp.size())
			ports->assign(tmp.size(), &tmp[0]);
		else
			ports->clear();
	}

	/*! \copydoc XsScanner_enumerateSerialPorts_int */
	void XsScanner_enumerateSerialPorts(XsPortInfoArray* ports, int ignoreNonXsensDevices)
	{
		LOGXSSCAN(__FUNCTION__ << " ignoreNonXsensDevices " << ignoreNonXsensDevices);
		XsScanner_enumerateSerialPorts_int(ports, ignoreNonXsensDevices);
	}

	/*!	\relates XsScanner
		\brief Scan the supplied ports for Xsens devices.
		\param[in,out] ports The list of ports to scan. Unresponsive devices will be removed from the list.
		\param[in] baudrate The baudrate to scan at. When set to XBR_Invalid, all known baudrates are scanned.
		\param[in] singleScanTimeout The timeout of a scan of a single port at a single baud rate in ms.
		\param[in] detectRs485 Enable more extended scan to detect rs485 devices
	*/
	void XsScanner_filterResponsiveDevices(XsPortInfoArray* ports, XsBaudRate baudrate, int singleScanTimeout, int detectRs485)
	{
		LOGXSSCAN(__FUNCTION__ << " baudrate " << baudrate << " singleScanTimeout " << singleScanTimeout << " detectRs485 " << detectRs485);
		assert(ports != nullptr);
		if (!ports)
			return;

		XsPortInfoArray tmp;
		for (XsSize i = 0; i < ports->size(); ++i)
			tmp.push_back(ports->at(i));
		Scanner::Accessor accessor;
		if (accessor.scanner().xsFilterResponsiveDevices(tmp, baudrate, singleScanTimeout, detectRs485 != 0))
		{
			if (tmp.size())
			{
				ports->assign(tmp.size(), &tmp[0]);
				return;
			}
		}
		ports->clear();
	}

	/*!	\relates XsScanner
		\brief List all compatible USB ports without scanning.
		\param[out] ports The list of detected ports.
	*/
	void XsScanner_enumerateUsbDevices(XsPortInfoArray* ports)
	{
		LOGXSSCAN(__FUNCTION__);

		assert(ports != nullptr);
		if (!ports)
			return;

		XsPortInfoArray tmp;
		xsEnumerateUsbDevices(tmp);
		if (tmp.size())
			ports->assign(tmp.size(), &tmp[0]);
		else
			ports->clear();
	}

	/*!	\relates XsScanner
		\brief Determine the USB hub that \a port is attached to
		\param[out] hub The identifier of the hub that \a port is attached to.
		\param[in] port The port for which to determine the USB hub.
	*/
	void XsScanner_scanUsbHub(XsUsbHubInfo* hub, const XsPortInfo* port)
	{
		LOGXSSCAN(__FUNCTION__);

		assert(hub != nullptr && port != nullptr);
		if (!hub || !port)
			return;

		Scanner::Accessor accessor;
		*hub = accessor.scanner().xsScanUsbHub(*port);
	}

	void XsScanner_enumerateNetworkDevices(XsPortInfoArray* ports)
	{
		LOGXSSCAN(__FUNCTION__);
		assert(ports != nullptr);
		if (!ports)
			return;

		XsPortInfoArray tmp;

		Scanner::Accessor accessor;
		accessor.scanner().xsEnumerateNetworkDevices(tmp);
		ports->swap(tmp);
	}

	/*!	\relates XsScanner
		\brief Abort the currently running port scan(s)
	*/
	void XsScanner_abortScan(void)
	{
		LOGXSSCAN(__FUNCTION__);
		XsScannerNamespace::abortPortScan = true;
	}

}
