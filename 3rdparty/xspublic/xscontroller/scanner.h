
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

#ifndef SCANNER_H
#define SCANNER_H

#include <xstypes/xsresultvalue.h>
#include <xstypes/xsportinfo.h>
#include <xstypes/xsportinfoarray.h>
#include <xstypes/xsbaudrate.h>
#include "xsusbhubinfo.h"
#include "xsscanner.h"

#include <atomic>

struct XsIntArray;

#ifdef _WIN32
#	include <setupapi.h>
#endif

struct Scanner
{
	class Accessor
	{
	public:
		Scanner& scanner() const;
	};

	XsResultValue fetchBasicInfo(XsPortInfo &portInfo, uint32_t singleScanTimeout, bool detectRs485);
	bool xsScanPort(XsPortInfo& portInfo, XsBaudRate baud, uint32_t singleScanTimeout, bool detectRs485);
	virtual bool xsScanPorts(XsPortInfoArray& ports, XsBaudRate baudrate, uint32_t singleScanTimeout, bool ignoreNonXsensDevices, bool detectRs485);
	bool xsFilterResponsiveDevices(XsPortInfoArray& ports, XsBaudRate baudrate, uint32_t singleScanTimeout, bool detectRs485);

#ifdef _WIN32
	std::string getDevicePath(HDEVINFO hDevInfo, SP_DEVINFO_DATA *DeviceInfoData);
	int xsScanGetHubNumber(HDEVINFO hDevInfo, SP_DEVINFO_DATA *deviceInfoData);
	XsPortInfo xsScanPortByHubId(const char* id);
	bool xsScanXsensUsbHubs(XsIntArray& hubs, XsPortInfoArray& ports);
#endif
	bool isXsensUsbDevice(uint16_t vid, uint16_t pid);
	bool xsEnumerateSerialPorts(XsPortInfoArray& ports, bool ignoreNonXsensDevices);
	virtual bool xsEnumerateNetworkDevices(XsPortInfoArray& ports);

	XsUsbHubInfo xsScanUsbHub(const XsPortInfo& portInfo);

	static void setScanLogCallback(XsScanLogCallbackFunc cb);
};

namespace XsScannerNamespace {
	extern volatile std::atomic_bool abortPortScan;
	extern Scanner* gScanner;
	extern XsScanLogCallbackFunc gScanLogCallback;
}

#define LOGXSSCAN(msg)\
	do {\
		JLDEBUGG(msg); \
		if (XsScannerNamespace::gScanLogCallback) \
		{ \
			std::ostringstream os; \
			os << msg; \
			const XsString cbVal(os.str()); \
			XsScannerNamespace::gScanLogCallback(&cbVal); \
		} \
	} while(0)

#endif
