
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

#include "xscontrollerconfig.h"
#include <xstypes/xsdeviceid.h>
#include <xstypes/xsportinfoarray.h>
#include <xstypes/xstime.h>
#include <xstypes/xsdid.h>
#include "xsscanner.h"
//#include "networkcommunicator.h"
#include "deviceredetector.h"

/*! \class DeviceRedetector
	\brief A class which re-detects a device with a certain device Id
*/

/*! \brief Construct an Device Redetector object */
DeviceRedetector::DeviceRedetector(const XsPortInfo &portInfo)
{
#ifdef XSENS_WINDOWS
	if (portInfo.isUsb())
#else
	(void)portInfo;
	if (1)
#endif
	{
		m_detectFunctions[XS_DID_TYPEH_MT_X_MPU] = &DeviceRedetector::redetectScanPorts;
		m_detectFunctions[XS_DID_TYPEH_MT_X0] = &DeviceRedetector::redetectScanPorts;
		m_detectFunctions[XS_DID_TYPEH_MT_X00] = &DeviceRedetector::redetectScanPorts;
		m_detectFunctions[XS_DID64_BIT] = &DeviceRedetector::redetectScanPorts;
		m_detectFunctions[XsDeviceId("MTi-610", 0, 0, XS_DID64_BIT)] = &DeviceRedetector::redetectScanPorts;
		m_detectFunctions[XsDeviceId("MTi-620", 0, 0, XS_DID64_BIT)] = &DeviceRedetector::redetectScanPorts;
		m_detectFunctions[XsDeviceId("MTi-630", 0, 0, XS_DID64_BIT)] = &DeviceRedetector::redetectScanPorts;
		m_detectFunctions[XsDeviceId("MTi-670", 0, 0, XS_DID64_BIT)] = &DeviceRedetector::redetectScanPorts;
	}
	else
	{
		m_detectFunctions[XS_DID_TYPEH_MT_X_MPU] = &DeviceRedetector::redetectOneComPort;
		m_detectFunctions[XS_DID_TYPEH_MT_X0] = &DeviceRedetector::redetectOneComPort;
		m_detectFunctions[XS_DID_TYPEH_MT_X00] = &DeviceRedetector::redetectOneComPort;
		m_detectFunctions[XS_DID64_BIT] = &DeviceRedetector::redetectOneComPort;
		m_detectFunctions[XsDeviceId("MTi-610", 0, 0, XS_DID64_BIT)] = &DeviceRedetector::redetectOneComPort;
		m_detectFunctions[XsDeviceId("MTi-620", 0, 0, XS_DID64_BIT)] = &DeviceRedetector::redetectOneComPort;
		m_detectFunctions[XsDeviceId("MTi-630", 0, 0, XS_DID64_BIT)] = &DeviceRedetector::redetectOneComPort;
		m_detectFunctions[XsDeviceId("MTi-670", 0, 0, XS_DID64_BIT)] = &DeviceRedetector::redetectOneComPort;
	}
}

/*! \brief Base redetect function which calls the appropriate redetect based on deviceId
	\param deviceId The device ID to look for
	\param portInfo The updated \a portinfo when the device is re-detected.
	\param skipDeviceIdCheck If set to true the it will skip device id check
	\returns True if successful
*/
bool DeviceRedetector::redetect(const XsDeviceId &deviceId, XsPortInfo &portInfo, bool skipDeviceIdCheck)
{
	FunctionPointer currentFunction;
	currentFunction = m_detectFunctions[deviceId.deviceType(false)];

	if (currentFunction)
		return (this->*currentFunction)(deviceId, portInfo, skipDeviceIdCheck);
	else
	{
		JLDEBUGG("No functions defined for device. Defaulting to OneComPort" << deviceId);
		return redetectOneComPort(deviceId, portInfo, skipDeviceIdCheck);
	}
}

/*! \returns True
	\details This function does nothing
	\param deviceId Device ID
	\param portInfo Port info
	\param skipDeviceIdCheck Skip device ID check
*/
bool DeviceRedetector::redetectNoScan(const XsDeviceId &deviceId, XsPortInfo &portInfo, bool skipDeviceIdCheck)
{
	(void)deviceId;
	(void)portInfo;
	(void)skipDeviceIdCheck;
	return true;
}

/*! \brief Scan for devices until the requested device is found
	\param deviceId The device ID to look for
	\param portInfo The updated \a portinfo when the device is re-detected.
	\param skipDeviceIdCheck If set to true then it will skip device ID check
	\details Scans for a certain period all available ports for the device.
	\returns True if successful
*/
bool DeviceRedetector::redetectScanPorts(const XsDeviceId& deviceId, XsPortInfo& portInfo, bool skipDeviceIdCheck)
{
	JLDEBUGG("Redetecting device " << deviceId);

	bool found = false;
	int count = 0;
	while (!found && count < 25)
	{
		XsTime::msleep(100);
		XsPortInfoArray portInfoArray = XsScanner::scanPorts(XBR_Invalid, 100, false, true);

		JLDEBUGG("Attempt " << count << ". Detected " << portInfoArray.size() << " devices.");

		for (uint32_t i = 0; i < portInfoArray.size(); i++)
		{
			portInfo = portInfoArray[i];
			if ((skipDeviceIdCheck && !portInfo.empty()) || portInfo.deviceId() == deviceId)
			{
				portInfo = portInfoArray[i];
				JLDEBUGG("Found device: " << portInfo);
				found = true;
				break;
			}
		}
		XsTime::msleep(1000);
		count++;
	}
	JLDEBUGG("Redetecting device " << deviceId << " . Result: " << found << " after " << count << " tries");
	return found;
}

/*! \brief Redectets and enumerates the serial ports for a given device
	\param deviceId The device ID to look for
	\param portInfo The updated \a portinfo when the device is re-detected.
	\param skipDeviceIdCheck If set to true then it will skip device ID check
	\returns True if successful
*/
bool DeviceRedetector::redetectEnumerateSerialPorts(const XsDeviceId &deviceId, XsPortInfo &portInfo, bool skipDeviceIdCheck)
{
	JLDEBUGG("Redetecting device " << deviceId);

	bool found = false;
	int count = 0;
	while (!found && count < 20)
	{
		XsTime::msleep(100);
		XsPortInfoArray portInfoArray = XsScanner::enumerateSerialPorts();

		JLDEBUGG("Attempt " << count << ". Detected " << portInfoArray.size() << " devices.");

		for (uint32_t i = 0; i < portInfoArray.size(); i++)
		{
			JLDEBUGG(portInfoArray[i]);
			if ((skipDeviceIdCheck && !portInfoArray[i].empty()) || portInfoArray[i].deviceId() == deviceId)
			{
				XsTime::msleep(3000);
				portInfo = portInfoArray[i];
				found = true;
				break;
			}
		}
		count++;
	}
	JLDEBUGG("Redetecting device " << deviceId << " . Result: " << found << " after " << count << " tries");
	return found;
}

/*! \brief Redectes a device on one com port
	\param deviceId The device ID to look for
	\param portInfo The updated \a portinfo when the device is re-detected.
	\param skipDeviceIdCheck If set to true then it will skip device ID check
	\returns True if successful
*/
bool DeviceRedetector::redetectOneComPort(const XsDeviceId &deviceId, XsPortInfo &portInfo, bool skipDeviceIdCheck)
{
	JLDEBUGG("Redetecting device " << deviceId);

	bool found = false;
	int count = 0;

	XsStringArray portList;
	portList.push_back(portInfo.portName());

	XsIntArray portLinesOptions;
	portLinesOptions.push_back((int)portInfo.linesOptions());

	while (!found && count < 20)
	{
		XsTime::msleep(100);

		XsPortInfoArray detectedPorts = XsScanner::scanComPortList(portList, portLinesOptions, XBR_Invalid, 100);

		assert(detectedPorts.size() <= 1); // at most one port found

		JLDEBUGG("Attempt " << count << ".");

		if (!detectedPorts.empty())
		{
			const XsPortInfo portInfoDetected = detectedPorts[0];

			JLDEBUGG(portInfoDetected);

			if ((skipDeviceIdCheck && !portInfoDetected.empty()) || portInfoDetected.deviceId() == deviceId)
			{
				uint16_t vid, pid;
				portInfo.getVidPid(vid, pid);
				portInfo = portInfoDetected;
				portInfo.setVidPid(vid, pid);
				found = true;
				break;
			}
		}
		count++;
	}

	JLDEBUGG("Redetecting device " << deviceId << " . Result: " << found << " after " << count << " tries");
	JLDEBUGG("PortInfo " << portInfo);
	return found;
}

/*! \returns False
	\details This function does nothing
	\param deviceId Device ID
	\param portInfo Port info
	\param skipDeviceIdCheck Skip device ID check
*/
bool DeviceRedetector::redetectEnumerateNetworkDevices(const XsDeviceId &deviceId, XsPortInfo &portInfo, bool skipDeviceIdCheck)
{
	(void)deviceId;
	(void)portInfo;
	(void)skipDeviceIdCheck;
	return false;
}
