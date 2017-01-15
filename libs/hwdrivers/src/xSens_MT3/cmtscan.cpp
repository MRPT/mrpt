/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/*! \file ScanPorts.cpp

	For information about objects in this file, see the appropriate header:
	\ref ScanPorts.h

	\section FileCopyright Copyright Notice 
	Copyright (C) Xsens Technologies B.V., 2006.  All rights reserved.
	
	This source code is intended for use only by Xsens Technologies BV and
	those that have explicit written permission to use it from
	Xsens Technologies BV.
	
	THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
	KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
	IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
	PARTICULAR PURPOSE.
	
	\section FileChangelog	Changelog
	\par 2006-06-08, v0.0.1
	\li Job Mulder:	Created
	\par 2006-07-21, v0.1.0
	\li Job Mulder:	Updated file for release 0.1.0
*/

#ifdef _WIN32
#	include <windows.h>
#	include <string.h>
#	include <setupapi.h>
#	include <devguid.h>
#	include <regstr.h>
#else
#	include <stdlib.h>
#	include <string.h>
#	include <dirent.h>
#endif

#include "cmt3.h"
#include "cmtscan.h"
#include "xsens_janitors.h"

namespace xsens {

#ifdef _LOG_CMT_SCAN
#	define SCANLOG		CMTLOG
#else
#	define SCANLOG(...)
#endif

bool abortScan = false;

bool cmtScanPort(CmtPortInfo& portInfo, uint32_t baud, uint32_t singleScanTimeout, uint32_t scanTries)
{
	uint32_t baudrate;
	Cmt3 port;
	port.setGotoConfigTries(scanTries?(uint16_t)scanTries:1);
	port.setTimeoutConfig(singleScanTimeout);
	XsensResultValue res;

	if (baud == 0)
		baudrate = CMT_BAUD_RATE_115K2;
	else
		baudrate = baud;

	while(!abortScan)
	{
		// try to connect at current baudrate
#ifdef _WIN32
		if ((res = port.openPort(portInfo.m_portNr,baudrate)) == XRV_OK)
#else
		if ((res = port.openPort(portInfo.m_portName,baudrate)) == XRV_OK)
#endif
		{
			SCANLOG("SP: L3 port-check returns OK\n");
			portInfo.m_baudrate = baudrate;
			portInfo.m_deviceId = port.getMasterId();
			return true;	// this also closes the port
		}
		// failed, determine if we need to scan other baudrates or not
		if (res != XRV_TIMEOUT && res != XRV_TIMEOUTNODATA && res != XRV_CONFIGCHECKFAIL)
		{
			SCANLOG("SP: L3 port-check returned ERROR, aborting\n");
			return false;
		}

		SCANLOG("SP: L3 port-check returned TIMEOUT, check next baudrate or abort\n");
		// not detected, try next baudrate
		if (baud != 0)
			return false;
		switch(baudrate)
		{
		default:
		case CMT_BAUD_RATE_115K2:
			baudrate = CMT_BAUD_RATE_460K8; break;
		case CMT_BAUD_RATE_460K8:
			baudrate = CMT_BAUD_RATE_921K6; break;
		case CMT_BAUD_RATE_921K6:
			baudrate = CMT_BAUD_RATE_230K4; break;
		case CMT_BAUD_RATE_230K4:
			baudrate = CMT_BAUD_RATE_57K6; break;
		case CMT_BAUD_RATE_57K6:
			baudrate = CMT_BAUD_RATE_38K4; break;
		case CMT_BAUD_RATE_38K4:
			baudrate = CMT_BAUD_RATE_19K2; break;
		case CMT_BAUD_RATE_19K2:
			baudrate = CMT_BAUD_RATE_9600; break;
		case CMT_BAUD_RATE_9600:
			return false;	// could not detect Xsens sensor, return false
		}
	}
	return false;
}

bool cmtScanPorts(List<CmtPortInfo>& ports,uint32_t baudrate, uint32_t singleScanTimeout, uint32_t scanTries)
{
	CmtPortInfo current;
	ports.clear();	// clear the list
#ifdef _WIN32
	HDEVINFO hDevInfo;
	SP_DEVINFO_DATA DeviceInfoData;
	DWORD i;

	// Create a HDEVINFO with all present devices.

	// GUID for Ports: 4D36E978-E325-11CE-BFC1-08002BE10318
	GUID portGuid = 
		{0x4D36E978,0xE325,0x11CE,{0xBF,0xC1,0x08,0x00,0x2B,0xE1,0x03,0x18}};
		
	//	"4D36E978-E325-11CE-BFC1-08002BE10318"
	hDevInfo = SetupDiGetClassDevs(&portGuid, 0, 0, DIGCF_PRESENT | DIGCF_PROFILE);

	if (hDevInfo == INVALID_HANDLE_VALUE)
		return false;

	// Enumerate through all devices in Set.
	DeviceInfoData.cbSize = sizeof(SP_DEVINFO_DATA);
	for (i=0;!abortScan && SetupDiEnumDeviceInfo(hDevInfo,i,&DeviceInfoData);++i)
	{
		DWORD DataT;
		char buffer[256];
		bool isBT = false;

		//
		// Call function with null to begin with,
		// then use the returned buffer size
		// to Alloc the buffer. Keep calling until
		// success or an unknown failure.
		//
#if 1
		if (SetupDiGetDeviceRegistryProperty(hDevInfo,
						&DeviceInfoData,
						SPDRP_MFG,
						&DataT,
						(PBYTE)buffer,
						256,
						NULL))
		{
			// on failure, this is not an Xsens Device
			// on success, we need to check if the device is an Xsens Device
			//if (_strnicmp(buffer,"xsens",5))
			//	scan = true;
			//else
			if (!_strnicmp(buffer,"(Standard port types)",strlen("(Standard port types)")))
				continue;
			if (_strnicmp(buffer,"xsens",5))	// if this is NOT an xsens device, treat it as a BT device
			{
				isBT = true;
				if (_strnicmp(buffer,"WIDCOMM",7))	// if this is NOT a WIDCOMM (Ezureo / TDK stack), skip it
					continue;
			}
		}
#endif
		// we found an Xsens Device, add its port nr to the list
		//Get the registry key which stores the ports settings
		HKEY hDeviceKey = SetupDiOpenDevRegKey(hDevInfo, &DeviceInfoData, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_QUERY_VALUE);
		if (hDeviceKey != INVALID_HANDLE_VALUE)
		{
			//Read in the name of the port
			char pszPortName[256];
			DWORD dwSize = 256;
			DWORD dwType = 0;
			if ((RegQueryValueExA(hDeviceKey, "PortName", NULL, &dwType, (LPBYTE) pszPortName, &dwSize) == ERROR_SUCCESS) && (dwType == REG_SZ))
			{
				//If it looks like "COMX" then
				//add it to the array which will be returned
				int32_t nLen = (int32_t) strlen(pszPortName);
				if (nLen > 3)
				{
					if (_strnicmp(pszPortName, "COM", 3))
						continue;
					int32_t nPort = atoi(&pszPortName[3]);
					if (nPort)
					{
						current.m_portNr = (uint8_t) nPort;
						if (isBT)
							current.m_baudrate = CMT_BAUD_RATE_460K8;
						else
							current.m_baudrate = baudrate;
						ports.append(current);
					}
				}
			}
		}
		//Close the key now that we are finished with it
		RegCloseKey(hDeviceKey);
	}

	//  Cleanup

	SetupDiDestroyDeviceInfoList(hDevInfo);

	// Now sort the list by ascending port nr
	ports.sortAscending();

	// Add the standard com ports 1 and 2 unless they are already in the list
	bool	add1 = true,
			add2 = true;
	if ((ports.length() > 0) && (ports[0].m_portNr == 1))
		add1 = false;
	if (ports.length() > 0)
	{
		if (ports[0].m_portNr == 2)
			add2 = false;
		else
			if (ports.length() > 1)
				if (ports[1].m_portNr == 2)
					add2 = false;
	}
	if (add1)
	{
		current.m_portNr = 1;
		current.m_baudrate = baudrate;
		ports.append(current);
	}
	if (add2)
	{
		current.m_portNr = 2;
		current.m_baudrate = baudrate;
		ports.append(current);
	}
#else
	DIR *dir;
	struct dirent *entry;
	
	if ((dir = opendir("/dev/")) == NULL)
		return false;
	
	while ((entry = readdir(dir)))
		if (strncmp("ttyS", entry->d_name, 4) == 0 || strncmp("ttyUSB", entry->d_name, 6) == 0)
		{
			sprintf(current.m_portName, "/dev/%s", entry->d_name);
			current.m_baudrate = baudrate;
			ports.append(current);
		}
	closedir(dir);
	
	ports.sortAscending();
#endif

	// try to connect so we can detect if there really is an MT / XM attached
	unsigned p = 0;
	while (!abortScan && p < ports.length())
	{
		if (cmtScanPort(ports[p],ports[p].m_baudrate,singleScanTimeout,scanTries))
			++p;
		else
			ports.remove(p);
	}

	if (abortScan)
		return abortScan = false;

	// Now sort the final list by ascending port nr
	ports.sortAscending();
	abortScan = false;
	return true;
}

} // end of xsens namespace
