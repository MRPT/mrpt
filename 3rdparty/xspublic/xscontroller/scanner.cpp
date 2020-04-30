
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

#include "scanner.h"

#ifdef _WIN32
#	include <devguid.h>
#	include <initguid.h>
#	include <Usbiodef.h>
#	include <cfgmgr32.h>
#	include <regstr.h>
#	include <regex>
#	include "idfetchhelpers.h"
#else
#	include <stdlib.h>
#	include <string.h>
#	include <dirent.h>
#	include "xslibusb.h"
#	include "udev.h"
#endif
#include <algorithm>

#include "serialportcommunicator.h"
#include "usbcommunicator.h"
#include <iostream>
#include <atomic>
#include <xscommon/xsens_janitors.h>
#include <xstypes/xsbusid.h>
#include <xstypes/xsportinfo.h>
#include <xstypes/xsintarray.h>
#include <xstypes/xsstringarray.h>
#include "enumerateusbdevices.h"
#include <xscommon/journaller.h>

namespace XsScannerNamespace {
	volatile std::atomic_bool abortPortScan{false};
	Scanner* gScanner = nullptr;
	XsScanLogCallbackFunc gScanLogCallback = nullptr;
}
using namespace XsScannerNamespace;

/*!	\class Scanner::Accessor
	\brief An accessor class for scanner
*/

/*! \returns The reference to a scanner object
*/
Scanner& Scanner::Accessor::scanner() const
{
	if (!gScanner)
	{
		gScanner = new Scanner();
	}
	return *gScanner;
}

/*!	\class Scanner
	\brief Provides static functionality for scanning for Xsens devices.
*/

/*!	\brief Set a callback function for scan log progress and problem reporting
	\details When set, any scan will use the provided callback function to report progress and failures.
	Normal operation is not affected, so all return values for the scan functions remain valid.
	\param cb The callback function to use. When set to NULL, no callbacks will be generated.
*/
void Scanner::setScanLogCallback(XsScanLogCallbackFunc cb)
{
	gScanLogCallback = cb;
}

/*!	\brief Fetch basic device information

	\param[in, out] portInfo  The name of the port to fetch from
	\param[in] singleScanTimeout The timeout of a scan of a single port at a single baud rate in ms.
	\param[in] detectRs485 Enable more extended scan to detect rs485 devices

	\returns XRV_OK if successful
*/
XsResultValue Scanner::fetchBasicInfo(XsPortInfo &portInfo, uint32_t singleScanTimeout, bool detectRs485)
{
	auto serial = Communicator::createUniquePtr<SerialPortCommunicator>();
	auto usb = Communicator::createUniquePtr<UsbCommunicator>();

	SerialPortCommunicator *port = (portInfo.isUsb() ? usb.get() : serial.get());

	port->setGotoConfigTimeout(singleScanTimeout);
	LOGXSSCAN("Opening port " << portInfo.portName() << " (" << portInfo.portNumber() << ") @ " << portInfo.baudrate() << " baud, expected device " << portInfo.deviceId());
	if (!port->openPort(portInfo, OPS_OpenPort))
	{
		LOGXSSCAN("Failed to open port because: " << XsResultValue_toString(port->lastResult()));
		return port->lastResult();
	}

	if (!port->openPort(portInfo, OPS_InitStart, detectRs485))
	{
		LOGXSSCAN("Failed to initialize port because: " << XsResultValue_toString(port->lastResult()));
		if (port->lastResult() < XRV_ERROR)
		{
			LOGXSSCAN("Attempting to reset device");
			// we got an xbus protocol error message, attempt to reset the device and try again (once)
			XsMessage snd(XMID_Reset);
			snd.setBusId(XS_BID_MASTER);
			if (!port->writeMessage(snd))
			{
				LOGXSSCAN("Failed to wriite reset to device because: " << XsResultValue_toString(port->lastResult()));
				return port->lastResult();
			}
			LOGXSSCAN("Reopening port after reset");
			port->closePort();
			XsTime::msleep(2000);
			if (!port->openPort(portInfo, OPS_Full, detectRs485))
			{
				LOGXSSCAN("Failed to reopen port after reset because: " << XsResultValue_toString(port->lastResult()));
				return port->lastResult();
			}
		}
		else
			return port->lastResult();
	}

	LOGXSSCAN("Port " << portInfo.portName() << " opened succesfully, device is " << port->masterDeviceId());
	portInfo.setDeviceId(port->masterDeviceId());

	// Enable flow control for Awinda2 stations/dongles which support this:
	if (port->masterDeviceId().isAwinda2())
	{
		XsPortLinesOptions portLinesOptions = portInfo.linesOptions();
		XsVersion fwVersion = port->firmwareRevision();
		XsVersion hwVersion = port->hardwareRevision();

		if (port->masterDeviceId().isAwinda2Station() && fwVersion.major() != 255 && fwVersion >= XsVersion(4, 2, 1))
			portLinesOptions = (XsPortLinesOptions)(portLinesOptions | XPLO_RtsCtsFlowControl);
		else if (port->masterDeviceId().isAwinda2Dongle() && fwVersion.major() != 255 && fwVersion >= XsVersion(4, 3, 2) && hwVersion >= XsVersion(2, 3))
			portLinesOptions = (XsPortLinesOptions)(portLinesOptions | XPLO_RtsCtsFlowControl);
		else
			portLinesOptions = (XsPortLinesOptions)(portLinesOptions & ~XPLO_RtsCtsFlowControl);
		LOGXSSCAN("Setting flow control options for Awinda device to " << JLHEXLOG(portLinesOptions));
		portInfo.setLinesOptions(portLinesOptions);
	}

	return XRV_OK;
}

/*!	\brief Scan a single COM port for connected Xsens devices

	\details The xsScanPort function will scan a single port for connected Xsens devices. If the
	baudrate parameter is 0 (default), it will try to connect at all supported baud
	rates, starting with the most common 115k2, 460k8 and 58k6. If the baudrate parameter
	is non-zero, only the specified baud rate is tried. Any detected devices are returned
	in the portInfo parameter.

	\param[in, out] portInfo The name of the port to scan should be in this parameter, the other contents will be filled by the function
	\param[in] baud The baudrate to scan at. When set to XBR_Invalid, all known baudrates are scanned
	\param[in] singleScanTimeout The timeout of a scan of a single port at a single baud rate in ms
	\param[in] detectRs485 Enable more extended scan to detect rs485 devices

	\returns true if a device was found, false otherwise
*/
bool Scanner::xsScanPort(XsPortInfo& portInfo, XsBaudRate baud, uint32_t singleScanTimeout, bool detectRs485)
{
	LOGXSSCAN("Scanning port " << portInfo.portName() << " at baudrate " << baud << " with timeout " << singleScanTimeout << " detectRs485 " << detectRs485);
	XsResultValue res;
	XsBaudRate baudrate;

	if (baud == 0)
		baudrate = XBR_115k2;
	else
		baudrate = baud;

	while (!abortPortScan)
	{
		portInfo.setBaudrate(baudrate);
		res = fetchBasicInfo(portInfo, singleScanTimeout, detectRs485);
		if (res == XRV_OK)
		{
			LOGXSSCAN("Scan successfully found device " << portInfo.deviceId() << " on port " << portInfo.portName());
			portInfo.setBaudrate(baudrate);
			return true;
		}

		// failed, determine if we need to scan other baudrates or not
		if (res != XRV_TIMEOUT && res != XRV_TIMEOUTNODATA && res != XRV_CONFIGCHECKFAIL)
		{
			LOGXSSCAN("Failed to fetch basic info: " << XsResultValue_toString(res));
			return false;
		}
		LOGXSSCAN("Failed to fetch basic info within timeout");

		// not detected, try next baudrate
		if (baud != 0)
		{
			LOGXSSCAN("Failed to find device");
			return false;
		}
		switch(baudrate)
		{
		default:
		case XBR_115k2:
			baudrate = XBR_2000k; break;
		case XBR_2000k:
			baudrate = XBR_921k6; break;
		case XBR_921k6:
			baudrate = XBR_460k8; break;
		case XBR_460k8:
			baudrate = XBR_230k4; break;
		case XBR_230k4:
			// On some systems a delay of about 100ms seems necessary to successfully perform the scan.
			XsTime::msleep(100);
			baudrate = XBR_57k6; break;
		case XBR_57k6:
			baudrate = XBR_38k4; break;
		case XBR_38k4:
			baudrate = XBR_19k2; break;
		case XBR_19k2:
			baudrate = XBR_9600; break;
		case XBR_9600:
			LOGXSSCAN("No more available baudrates, failed to find device");
			return false;	// could not detect Xsens device, return false
		}
		LOGXSSCAN("Checking next baudrate: " << baudrate);
	}
	LOGXSSCAN("Port scan aborted by external trigger");
	return false;
}

/*!	\brief Scan serial ports for connected Xsens devices.

	\details The xsScanPorts function will scan registered Xsens USB converters and serial COM ports
	for connected Xsens devices. If the baudrate parameter is 0 (default), it will try to
	connect at all supported baud rates, starting with the most common 115k2, 460k8 and
	58k6. If the baudrate parameter is non-zero, only the specified baudrate is tried.
	Any detected devices are returned in the ports list, which is sorted by port nr.

	\param[out] ports The list of detected ports.
	\param[in] baudrate The baudrate to scan at. When set to XBR_Invalid, all known baudrates are scanned.
	\param[in] singleScanTimeout The timeout of a scan of a single port at a single baud rate in ms.
	\param[in] ignoreNonXsensDevices When non-zero (the default), only Xsens devices are returned. Otherwise other devices that comply with the Xsens message protocol will also be returned.
	\param[in] detectRs485 Enable more extended scan to detect rs485 devices

	\returns true if at least one device was found, false otherwise
*/
bool Scanner::xsScanPorts(XsPortInfoArray& ports, XsBaudRate baudrate, uint32_t singleScanTimeout, bool ignoreNonXsensDevices, bool detectRs485)
{
	ports.clear();

	if (!xsEnumerateSerialPorts(ports, ignoreNonXsensDevices))
		return false;

	if (!xsEnumerateUsbDevices(ports))
		return false;

	return xsFilterResponsiveDevices(ports, baudrate, singleScanTimeout, detectRs485);
}

/*!	\brief Filter responsive devices

	\details Serial ports that do not have a responsive Xsens device connected are removed from \a ports.

	\param[in,out] ports The list of ports to filter
	\param[in] baudrate The baud rate used for scanning. If \a baudrate equals XBR_Invalid, all rates are scanned.
	\param[in] singleScanTimeout The maximum time allowed for response
	\param[in] detectRs485 Enable more extended scan to detect rs485 devices

	\returns true if successful
*/
bool Scanner::xsFilterResponsiveDevices(XsPortInfoArray& ports, XsBaudRate baudrate, uint32_t singleScanTimeout, bool detectRs485)
{
	// try to connect so we can detect if there really is an MT / XM attached
	unsigned p = 0;
	while (!abortPortScan && p < ports.size())
	{
		if (ports[p].isNetwork() || xsScanPort(ports[p], baudrate, singleScanTimeout, detectRs485))
			++p;
		else
		{
			LOGXSSCAN("Port : " << ports[p].portName() << " is not responsive, discarding");
			ports.erase(ports.begin() + p);
		}
	}

	if (abortPortScan)
	{
		abortPortScan = false;
		return false;
	}

	// Now sort the final list by ascending port nr
	std::sort(ports.begin(), ports.end());
	abortPortScan = false;
	return true;
}

#ifdef _WIN32
/*! \returns The device path for given windows device
	\param[in] hDevInfo The refernce to a device information
	\param[in] DeviceInfoData The pointer to a device information data
*/
std::string Scanner::getDevicePath(HDEVINFO hDevInfo, SP_DEVINFO_DATA *DeviceInfoData)
{
	char deviceInstanceID[MAX_DEVICE_ID_LEN];
	SetupDiGetDeviceInstanceIdA(hDevInfo, DeviceInfoData,deviceInstanceID, MAX_DEVICE_ID_LEN, NULL);
	return std::string(deviceInstanceID);
}
#endif

/*! \returns true if the vendor/product combination may point to an xsens device
	\param[in] vid The vendor ID
	\param[in] pid the product ID
*/
bool Scanner::isXsensUsbDevice(uint16_t vid, uint16_t pid)
{
	switch (vid)
	{
	// Xsens
	case XSENS_VENDOR_ID:
		// ignore the body pack serial port
		return (pid != 0x0100);

	// FTDI reserved PIDs
	case FTDI_VENDOR_ID:
		return (pid >= 0xd388 && pid <= 0xd38f);

	default:
		return false;
	}
}

/*!	\brief Enumerate the serial ports
	\param[in,out] ports The list of ports to append to
	\param[in] ignoreNonXsensDevices If set to true (default), ignore serial ports that aren't Xsens USB devices
	\returns True if successful
*/
bool Scanner::xsEnumerateSerialPorts(XsPortInfoArray& ports, bool ignoreNonXsensDevices)
{
	LOGXSSCAN("Enumerating USB devices");
	XsPortInfo current;

#ifdef _WIN32
	HDEVINFO hDevInfo;
	SP_DEVINFO_DATA DeviceInfoData;
	DWORD i;

	// Create a HDEVINFO with all present devices.
	hDevInfo = SetupDiGetClassDevs(&GUID_DEVCLASS_PORTS, 0, 0, DIGCF_PRESENT | DIGCF_PROFILE | DIGCF_ALLCLASSES);

	if (hDevInfo == INVALID_HANDLE_VALUE)
	{
		LOGXSSCAN("Failed to get any USB device information, check permissions");
		return false;
	}

	// Enumerate through all devices in Set.
	DeviceInfoData.cbSize = sizeof(SP_DEVINFO_DATA);
	for (i=0;!abortPortScan && SetupDiEnumDeviceInfo(hDevInfo,i,&DeviceInfoData);++i)
	{
		// Get the registry key which stores the ports settings
		HKEY hDeviceKey = SetupDiOpenDevRegKey(hDevInfo, &DeviceInfoData, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_QUERY_VALUE);
		if (hDeviceKey == INVALID_HANDLE_VALUE)
			continue;

		auto devkeycleaner = [](HKEY *key)
		{
			RegCloseKey(*key);
		};
		std::unique_ptr<HKEY, decltype(devkeycleaner)> devkey(&hDeviceKey, devkeycleaner);

		// Read in the name of the port
		char pszPortName[256];
		DWORD dwSize = 256;
		DWORD dwType = 0;
		if ((RegQueryValueExA(hDeviceKey, "PortName", NULL, &dwType, (LPBYTE) pszPortName, &dwSize) != ERROR_SUCCESS) || (dwType != REG_SZ))
			continue;

		// If it looks like "COMX" then
		// add it to the array which will be returned
		if (_strnicmp(pszPortName, "COM", 3))
			continue;
		int32_t nPort = atoi(&pszPortName[3]);
		if (nPort == 0)
			continue;

		std::string devpath = getDevicePath(hDevInfo, &DeviceInfoData);
		uint16_t vid = vidFromString(devpath);
		uint16_t pid = pidFromString(devpath);

		LOGXSSCAN("Found USB device " << devpath);
		if (ignoreNonXsensDevices)
		{
			if (!isXsensUsbDevice(vid, pid))
			{
				LOGXSSCAN("Ignoring non-Xsens device " << devpath);
				continue;
			}
		}

		current.setPortName(pszPortName);
		current.setBaudrate(XBR_Invalid);
		current.setDeviceId(deviceIdFromDevPath(devpath));
		current.setVidPid(vid, pid);
		ports.push_back(current);

		current.clear();
	}

	//  Cleanup

	SetupDiDestroyDeviceInfoList(hDevInfo);

	// Now sort the list by ascending port nr
	std::sort(ports.begin(), ports.end());

#else // !_WIN32
	Udev xsudev;
	struct udev *udev = xsudev.unew();
	if (udev)
	{
		struct udev_enumerate *enumerate = xsudev.enumerate_new(udev);

		xsudev.enumerate_add_match_subsystem(enumerate, "tty");
		xsudev.enumerate_scan_devices(enumerate);

		struct udev_list_entry *devices, *dev_list_entry;
		devices = xsudev.enumerate_get_list_entry(enumerate);
		for (dev_list_entry = devices; dev_list_entry != nullptr; dev_list_entry = xsudev.list_entry_get_next(dev_list_entry))
		{
			const char *path = xsudev.list_entry_get_name(dev_list_entry);
			auto devcleaner = [&](struct udev_device *d)
			{
				xsudev.device_unref(d);
			};
			std::unique_ptr<struct udev_device, decltype(devcleaner)>
				device(xsudev.device_new_from_syspath(udev, path), devcleaner);
			if (!device)
				continue;

			if (!xsudev.device_get_parent(device.get()))
				// this is probably a console, definitely not a physical serial port device
				continue;

			struct udev_device *usbParentDevice = xsudev.device_get_parent_with_subsystem_devtype(device.get(), "usb", "usb_device");
			unsigned int vid = 0, pid = 0;
			if (usbParentDevice)
			{
				const char *vendor = xsudev.device_get_sysattr_value(usbParentDevice, "idVendor");
				const char *product = xsudev.device_get_sysattr_value(usbParentDevice, "idProduct");

				if (vendor && product)
				{
					sscanf(vendor, "%x", &vid);
					sscanf(product, "%x", &pid);
				}
			}

			LOGXSSCAN("Found USB device " << path);

			if (ignoreNonXsensDevices)
			{
				if (!isXsensUsbDevice(vid, pid))
				{
					LOGXSSCAN("Ignoring non-Xsens device " << path);
					continue;
				}
			}

			XsPortInfo portInfo;

			const char *devnode = xsudev.device_get_devnode(device.get());
			if (strlen(devnode) > 255 || strncmp(devnode, "/dev/ttyS", 9) == 0)
				continue;
			portInfo.setPortName(devnode);
			portInfo.setVidPid(vid, pid);

			if (usbParentDevice)
			{
				const char *deviceidstring;
				deviceidstring = xsudev.device_get_sysattr_value(usbParentDevice, "serial");
				if (deviceidstring)
				{
					int deviceId = 0;
					sscanf(deviceidstring, "%08X", &deviceId);
					portInfo.setDeviceId(deviceId);
				}
			}

			ports.push_back(portInfo);
		}
		xsudev.enumerate_unref(enumerate);
		xsudev.unref(udev);
	}
	else
	{
		(void)ignoreNonXsensDevices;
		DIR *dir;
		struct dirent *entry;

		if ((dir = opendir("/dev/")) == NULL)
			return false;

		while ((entry = readdir(dir)))
		{
			if (strncmp("ttyUSB", entry->d_name, 6) == 0)
			{
				char name[261];
				sprintf(name, "/dev/%s", entry->d_name);
				current.setPortName(name);
				ports.push_back(current);
				LOGXSSCAN("Found USB device " << name);
			}
		}
		closedir(dir);
	}
#endif // _WIN32
	return true;
}

/*!	\brief Enaumerates a network device
	\param ports The port info array
	\returns false
*/
bool Scanner::xsEnumerateNetworkDevices(XsPortInfoArray & ports)
{
	(void)ports;
	return false;
}

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

#define ILLEGAL_HUB	(0)
#ifdef _WIN32
#define HUB_SEARCH_STRING ("Hub_#")

/*!	\brief Retrieves the USB Hub number for a device
	\param[in] hDevInfo The refernce to a device information
	\param[in] deviceInfoData The pointer to a device information data
	\returns Non-zero if successful
*/
int Scanner::xsScanGetHubNumber(HDEVINFO hDevInfo, SP_DEVINFO_DATA *deviceInfoData)
{
	DWORD DataT;
	char buffer[256];
	int result = ILLEGAL_HUB;

	if (SetupDiGetDeviceRegistryPropertyA(hDevInfo,
			deviceInfoData,
			SPDRP_LOCATION_INFORMATION,
			&DataT,
			(PBYTE)buffer,
			256,
			NULL))
	{
		LOGXSSCAN("Registry access successful: \"" << buffer << "\"");
		char const * hubString = strstr((char const *)buffer, HUB_SEARCH_STRING);
		if (hubString)
		{
			result = strtol(hubString + strlen(HUB_SEARCH_STRING), 0, 10);
		}
	}
	else
		LOGXSSCAN("Get Hub Number failed with error " << GetLastError());

	return result;
}

/*!	\brief Scans a port by an Xsens hub ID (e.g. Awinda Station)
	\param[in] id The id of a hub
	\returns The found port info
*/
XsPortInfo Scanner::xsScanPortByHubId(const char* id)
{
	assert(id != nullptr);
	if (!id)
		return XsPortInfo();

	LOGXSSCAN("xsScanPortByHubId with id " << id);

	// Get device interface info set handle for all devices attached to system
	HDEVINFO hDevInfo = SetupDiGetClassDevs(
		&GUID_DEVCLASS_PORTS, /* CONST GUID * ClassGuid - USB class GUID */
		NULL, /* PCTSTR Enumerator */
		NULL, /* HWND hwndParent */
		DIGCF_PRESENT | DIGCF_DEVICEINTERFACE /* DWORD Flags */
		);

	if (hDevInfo == INVALID_HANDLE_VALUE)
	{
		LOGXSSCAN("Failed to get any USB device information, check permissions");
		return XsPortInfo();
	}

	// Retrieve a context structure for a device interface of a device
	// information set.

	SP_DEVICE_INTERFACE_DATA devInterfaceData;
	ZeroMemory(&devInterfaceData, sizeof(SP_DEVICE_INTERFACE_DATA));
	devInterfaceData.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);
	XsPortInfo portInfo;
	int port = 0;
	for (DWORD dwIndex = 0; port == 0; ++dwIndex)
	{
		BOOL bRet = SetupDiEnumDeviceInterfaces(
			hDevInfo, /* HDEVINFO DeviceInfoSet */
			NULL, /* PSP_DEVINFO_DATA DeviceInfoData */
			&GUID_DEVINTERFACE_SERENUM_BUS_ENUMERATOR, /* CONST GUID * InterfaceClassGuid */
			dwIndex,
			&devInterfaceData /* PSP_DEVICE_INTERFACE_DATA DeviceInterfaceData */
			);
		if (!bRet)
		{
			if (GetLastError() == ERROR_NO_MORE_ITEMS)
				break;
		}
		else
		{
			char buffer[1024];

			SP_DEVINFO_DATA diData;
			diData.cbSize = sizeof(SP_DEVINFO_DATA);
			SP_DEVICE_INTERFACE_DETAIL_DATA_A& ifdData = *(SP_DEVICE_INTERFACE_DETAIL_DATA_A*) buffer;
			ifdData.cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_A);
			DWORD reqSize;
			if (SetupDiGetDeviceInterfaceDetailA(hDevInfo, &devInterfaceData, &ifdData, 1020, &reqSize, &diData))
			{
				// determine my own id
				// strip A#0000#* from path
				char* end = strrchr(ifdData.DevicePath, '#');
				if (end)
					end[0] = 0;
				end = strrchr(ifdData.DevicePath, '#');
				if (end)
					end[0] = 0;
				char* start = strrchr(ifdData.DevicePath, '+');
				if (!start || !end)
					continue;
				++start;

				// compare IDs
				if (_strnicmp(id, start, strlen(id)) == 0)
				{
					// we found a match, determine port nr
					// Get the registry key which stores the ports settings
					HKEY hDeviceKey = SetupDiOpenDevRegKey(hDevInfo, &diData, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_QUERY_VALUE);
					if (hDeviceKey != INVALID_HANDLE_VALUE)
					{
						// Read in the name of the port
						char pszPortName[256] = "";
						DWORD dwSize = 256;
						DWORD dwType = 0;
						if ((RegQueryValueExA(hDeviceKey, "PortName", NULL, &dwType, (LPBYTE) pszPortName, &dwSize) == ERROR_SUCCESS) && (dwType == REG_SZ))
						{
							// If it looks like "COMX" then
							// add it to the array which will be returned
							if (!_strnicmp(pszPortName, "COM", 3))
								port = atoi(&pszPortName[3]);
							portInfo.setPortName(pszPortName);
							std::string devpath = getDevicePath(hDevInfo, &diData);
							uint16_t vid = vidFromString(devpath);
							uint16_t pid = pidFromString(devpath);
							LOGXSSCAN("Adding " << pszPortName << " with vid " << JLHEXLOG(vid) << " and pid " << JLHEXLOG(pid));
							portInfo.setVidPid(vid, pid);
						}
					}
					// Close the key now that we are finished with it
					RegCloseKey(hDeviceKey);
				}
			}
		}
	}

	SetupDiDestroyDeviceInfoList(hDevInfo);

	return portInfo;
}

/*!	\brief Scans all USB controllers to search for an Xsens hubs (e.g. Awinda Station)
	\param[out] hubs The list of hubs found
	\param[out] ports The list of ports associated with the found hubs
	\returns True if successful
*/
bool Scanner::xsScanXsensUsbHubs(XsIntArray& hubs, XsPortInfoArray& ports)
{
	LOGXSSCAN("xsScanXsensUsbHubs");

	// clear the lists
	hubs.clear();
	ports.clear();

	// Get device interface info set handle for all devices attached to system
	HDEVINFO hDevInfo = SetupDiGetClassDevs(
		&GUID_DEVINTERFACE_USB_DEVICE, /* CONST GUID * ClassGuid - USB class GUID */
		NULL, /* PCTSTR Enumerator */
		NULL, /* HWND hwndParent */
		DIGCF_PRESENT | DIGCF_DEVICEINTERFACE /* DWORD Flags */
		);

	if (hDevInfo == INVALID_HANDLE_VALUE)
	{
		LOGXSSCAN("Failed to get any USB device information, check permissions");
		return false;
	}

	// Retrieve a context structure for a device interface of a device
	// information set.

	SP_DEVICE_INTERFACE_DATA devInterfaceData;
	ZeroMemory(&devInterfaceData, sizeof(SP_DEVICE_INTERFACE_DATA));
	devInterfaceData.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);
	for (DWORD dwIndex = 0; true; ++dwIndex)
	{
		BOOL bRet = SetupDiEnumDeviceInterfaces(
			hDevInfo, /* HDEVINFO DeviceInfoSet */
			NULL, /* PSP_DEVINFO_DATA DeviceInfoData */
			&GUID_DEVINTERFACE_USB_DEVICE, /* CONST GUID * InterfaceClassGuid */
			dwIndex,
			&devInterfaceData /* PSP_DEVICE_INTERFACE_DATA DeviceInterfaceData */
			);
		if (!bRet)
		{
			if (GetLastError() == ERROR_NO_MORE_ITEMS)
				break;
			continue;
		}

		char* tmp = new char[2048];
		char* buffer = tmp;
		char* devPath = tmp+1024;
		SP_DEVINFO_DATA diData;
		diData.cbSize = sizeof(SP_DEVINFO_DATA);

		SP_DEVICE_INTERFACE_DETAIL_DATA_A& ifdData = *(SP_DEVICE_INTERFACE_DETAIL_DATA_A*) devPath;
		ifdData.cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_A);
		DWORD reqSize;
		XsPortInfo port;
		int hubNr = 0;
		while (SetupDiGetDeviceInterfaceDetailA(hDevInfo, &devInterfaceData, &ifdData, 1020, &reqSize, &diData))
		{
			DWORD dataT;
			if (SetupDiGetDeviceRegistryPropertyA(hDevInfo,
						&diData,
						SPDRP_MFG,
						&dataT,
						(PBYTE)buffer,
						256,
						NULL))
			{
				if (_strnicmp(buffer,"xsens",5))	// if this is NOT an xsens device, ignore it
					break;
			}
			else
				break;

			hubNr = xsScanGetHubNumber(hDevInfo, &diData);
			// we found something, determine its port
			if (hubNr)
			{
				char* hubEnd = strrchr(ifdData.DevicePath, '#');
				if (!hubEnd || hubEnd == ifdData.DevicePath)
					break;
				hubEnd[0] = 0;
				char* hubStart = strrchr(ifdData.DevicePath, '#');
				if (!hubStart || hubStart == ifdData.DevicePath)
					break;
				++hubStart;
				port = xsScanPortByHubId(hubStart);
			}
			break;
		}
		if (port.portName().size())
		{
			LOGXSSCAN("xsScanXsensUsbHubs Adding hub " << hubNr << " and port " << port.portName());
			hubs.push_back(hubNr);
			ports.push_back(port);
		}
		delete[] tmp;
	}

	SetupDiDestroyDeviceInfoList(hDevInfo);

	return true;
}
#endif // _WIN32

/*!	\brief Get information about the hub configuration
	\param portInfo Descriptor of the USB port to scan
	\returns An XsUsbHubInfo object that provides platform independent comparison

	\details This function behaves slightly different on different platforms. On Windows
	only hubs that are Xsens devices are taken into account. On platforms supporting udev
	the information is always filled in if \a port exists. This means that
	\code
	if (xsScanUsbHub(port).isValid())
	{
	// device is docked
	}
	\endcode

	is not enough to accurately determine whether a device was docked in a cross-platform
	and future proof application.
	Instead use the	recommended approach to check whether two devices share the same hub:

	\code
	XsUsbHubInfo hubInfo1 = xsScanUsbHub(port1);
	XsUsbHubInfo hubInfo2 = xsScanUsbHub(port2);

	hubInfo1.parentPathMatches(hubInfo2);
	\endcode
*/
XsUsbHubInfo Scanner::xsScanUsbHub(const XsPortInfo& portInfo)
{
#ifdef _WIN32
	int behindXsensHub = ILLEGAL_HUB;
	XsIntArray hubs;
	XsPortInfoArray ports;
	if (!xsScanXsensUsbHubs(hubs, ports))
		return XsUsbHubInfo();

	// check if it's in the ports list before going through all THAT trouble...
	for (size_t i = 0; i < ports.size(); ++i)
		if (_stricmp(ports[i].portName().c_str(), portInfo.portName().c_str()) == 0)
			return XsUsbHubInfo(hubs[i]);

	HDEVINFO hDevInfo;
	SP_DEVINFO_DATA devInfoData;

	// Create a HDEVINFO with all present devices.
	hDevInfo = SetupDiGetClassDevs(&GUID_DEVCLASS_PORTS, 0, 0, DIGCF_PRESENT | DIGCF_PROFILE);

	if (hDevInfo == INVALID_HANDLE_VALUE)
		return XsUsbHubInfo();

	// Enumerate through all devices in Set.
	devInfoData.cbSize = sizeof(SP_DEVINFO_DATA);
	for (DWORD i=0; behindXsensHub == ILLEGAL_HUB && SetupDiEnumDeviceInfo(hDevInfo,i,&devInfoData); ++i)
	{
		DWORD dataT;
		char buffer[256];

		// Call function with null to begin with,
		// then use the returned buffer size
		// to Alloc the buffer. Keep calling until
		// success or an unknown failure.
		//

		if (!SetupDiGetDeviceRegistryPropertyA(hDevInfo,
						&devInfoData,
						SPDRP_MFG,
						&dataT,
						(PBYTE)buffer,
						256,
						NULL))
			continue;

		// Get the registry key which stores the ports settings
		HKEY hDeviceKey = SetupDiOpenDevRegKey(hDevInfo, &devInfoData, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_QUERY_VALUE);
		if (hDeviceKey != INVALID_HANDLE_VALUE)
		{
			// Read in the name of the port
			char pszPortName[256];
			DWORD dwSize = 256;
			DWORD dwType = 0;
			if ((RegQueryValueExA(hDeviceKey, "PortName", NULL, &dwType, (LPBYTE) pszPortName, &dwSize) == ERROR_SUCCESS) && (dwType == REG_SZ))
			{
				// If it looks like "COMX" then
				// add it to the array which will be returned
				int32_t nLen = (int32_t) strlen(pszPortName);
				if (nLen > 3)
				{
					if (_strnicmp(pszPortName, "COM", 3))
						continue;
					//int32_t nPort = atoi(&pszPortName[3]);
					if (_stricmp(pszPortName, portInfo.portName().c_str()) == 0)
					{
						//This is the port we are looking for
						//check if the hub number is in the hubs list
						int hub = xsScanGetHubNumber(hDevInfo, &devInfoData);
						JLDEBUGG("Got hub " << hub << " for port " << std::string(pszPortName));

						if (hub != ILLEGAL_HUB)
						{
							for (XsIntArray::const_iterator hi = hubs.begin(); hi != hubs.end(); ++hi)
							{
								if (*hi == hub)
								{
									behindXsensHub = hub;
									break;
								}
							}
						}
					}
				}
			}
		}
		// Close the key now that we are finished with it
		RegCloseKey(hDeviceKey);
	}

	//  Cleanup

	SetupDiDestroyDeviceInfoList(hDevInfo);
	return XsUsbHubInfo(behindXsensHub);
#else
	XsUsbHubInfo inf;

	Udev xsudev;

	udev *udevInstance = xsudev.unew();
	if (udevInstance == NULL) {
		fprintf(stderr, "Unable to create udev object\n");
		return XsUsbHubInfo();
	}

	udev_enumerate *enumerate = xsudev.enumerate_new(udevInstance);
	xsudev.enumerate_scan_devices(enumerate);

	udev_list_entry *devices = xsudev.enumerate_get_list_entry(enumerate);
	udev_list_entry *dev;
	for (dev = devices; dev != NULL; dev = xsudev.list_entry_get_next(dev))
	{
		const char *path = xsudev.list_entry_get_name(dev);
		udev_device *device = xsudev.device_new_from_syspath(udevInstance, path);
		if (!device)
			return XsUsbHubInfo();

		const char *devnode = xsudev.device_get_devnode(device);
		if (!devnode || strcmp(devnode, portInfo.portName().c_str()) != 0)
			continue;

		udev_device *parent = xsudev.device_get_parent_with_subsystem_devtype(device, "usb", "usb_device");
		if (!parent)
			break;

		const char *devpath = xsudev.device_get_sysattr_value(parent, "devpath");
		if (!devpath)
			break;

		inf = XsUsbHubInfo(devpath);
		break;
	}
	xsudev.enumerate_unref(enumerate);
	xsudev.unref(udevInstance);
	return inf;
#endif
}
