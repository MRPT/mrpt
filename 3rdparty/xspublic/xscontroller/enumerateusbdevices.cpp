
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
#include "enumerateusbdevices.h"

#ifdef _WIN32
#	include <windows.h>
#	include <string.h>
#	include <setupapi.h>
#	include <devguid.h>
#	include <regstr.h>
#	include <regex>
#	include <cfgmgr32.h>
#else
#	include <stdlib.h>
#	include <string.h>
#	include <dirent.h>
#	include "xslibusb.h"
#endif

#ifdef _WIN32
/* return the device path for given windows device */
static std::string getDevicePath(HDEVINFO hDevInfo, SP_DEVINFO_DATA *DeviceInfoData)
{
	char deviceInstanceID[MAX_DEVICE_ID_LEN];
	SetupDiGetDeviceInstanceIdA(hDevInfo, DeviceInfoData,deviceInstanceID, MAX_DEVICE_ID_LEN, NULL);
	return std::string(deviceInstanceID);
}


/* Get the first match for _regex_ within _devpath_ */
static inline std::string searchOne(std::string const& devpath, std::string const& regex)
{
	std::smatch sm;
	std::regex_search(devpath, sm, std::regex(regex));
	if (sm.size() < 2)
		return std::string();
	return sm[1];
}

/* Fetch the vendor ID from the given device path */
static inline uint16_t vidFromDevPath(std::string const& devpath)
{
	try
	{
		auto t1 = searchOne(devpath, "VID_([0-9a-fA-F]{4}).*PID_.*");
		if (t1.empty())	// prevent unnecessary exceptions
			return 0;
		return static_cast<uint16_t>(std::stoi(t1, nullptr, 16));
	}
	catch (std::invalid_argument &)
	{
		return 0;
	}
}

/* Fetch the product ID from the given device path */
static inline uint16_t pidFromDevPath(std::string const& devpath)
{
	try
	{
		auto t1 = searchOne(devpath, "VID_.*PID_([0-9a-fA-F]{4})");
		if (t1.empty())	// prevent unnecessary exceptions
			return 0;
		return static_cast<uint16_t>(std::stoi(t1, nullptr, 16));
	}
	catch (std::invalid_argument &)
	{
		return 0;
	}
}
#endif

/*! \brief Enumerate Xsens USB devices

	If the OS already has drivers running for a device, the device should already have been
	found by xsEnumerateSerialPorts().

	\param[in,out] ports The list of serial ports to append to
	\returns False if an error occured during scanning.
			 True if zero or more device found or no scan could be done	because both WINUSB and LIBUSB were not defined.
*/
bool xsEnumerateUsbDevices(XsPortInfoArray& ports)
{
	XsPortInfo current;
#ifdef USE_WINUSB
	BOOL bResult = FALSE;
	ULONG length;
	ULONG requiredLength=0;

	// {FD51225C-700A-47e5-9999-B2D9031B88ED}
	GUID guid = { 0xfd51225c, 0x700a, 0x47e5, { 0x99, 0x99, 0xb2, 0xd9, 0x3, 0x1b, 0x88, 0xed } };

	HDEVINFO deviceInfo;
	SP_DEVICE_INTERFACE_DATA interfaceData;
	PSP_DEVICE_INTERFACE_DETAIL_DATA_A detailData = NULL;

	deviceInfo = SetupDiGetClassDevs(&guid, NULL, NULL,	DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);

	// Initialize variables.
	interfaceData.cbSize = sizeof(SP_INTERFACE_DEVICE_DATA);
	int port = 0;
	for (DWORD dwIndex = 0; port == 0; ++dwIndex)
	{
		BOOL bRet = SetupDiEnumDeviceInterfaces( deviceInfo, NULL, &guid, dwIndex, &interfaceData);
		if (!bRet)
		{
			if (GetLastError() == ERROR_NO_MORE_ITEMS)
				break;
		}
		else
		{
			if (!SetupDiGetDeviceInterfaceDetail(deviceInfo, &interfaceData, NULL, 0, &requiredLength, NULL))
			{
				if (GetLastError() != ERROR_INSUFFICIENT_BUFFER)
				{
					SetupDiDestroyDeviceInfoList(deviceInfo);
					return false;
				}
			}
			detailData = (PSP_DEVICE_INTERFACE_DETAIL_DATA_A)LocalAlloc(LMEM_FIXED, requiredLength);
			if (NULL == detailData)
			{
				SetupDiDestroyDeviceInfoList(deviceInfo);
				return false;
			}

			detailData->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_A);
			length = requiredLength;
			SP_DEVINFO_DATA DevInfoData;
			DevInfoData.cbSize = sizeof(SP_DEVINFO_DATA);
			bResult = SetupDiGetDeviceInterfaceDetailA(deviceInfo, &interfaceData, detailData, length, &requiredLength, &DevInfoData);

			if (!bResult)
			{
				LocalFree(detailData);
				SetupDiDestroyDeviceInfoList(deviceInfo);
				return false;
			}

			unsigned char serialNumber[256];
			char* ptrEnd, *ptrStart = strchr(detailData->DevicePath, '#');
			if (!ptrStart)
				continue;
			ptrStart = strchr(ptrStart+1, '#');
			if (!ptrStart)
				continue;
			ptrEnd = strchr(ptrStart+1, '#');
			if (!ptrEnd)
				continue;

			strncpy((char*)serialNumber, ptrStart+1, ptrEnd-ptrStart-1);
			serialNumber[ptrEnd-ptrStart-1] = '\0';

			current.setPortName(detailData->DevicePath);

			int id = 0;
			sscanf((const char *)serialNumber, "%X", &id);
			current.setDeviceId((uint32_t) id);

			std::string devpath = getDevicePath(deviceInfo, &DevInfoData);
			uint16_t vid = vidFromDevPath(devpath);
			uint16_t pid = pidFromDevPath(devpath);
			current.setVidPid(vid, pid);

			ports.push_back(current);
		}
	}

	SetupDiDestroyDeviceInfoList(deviceInfo);
	return true;
#elif defined(HAVE_LIBUSB)
	XsLibUsb libUsb;
	libusb_context *context;
	int result = libUsb.init(&context);
	if (result != LIBUSB_SUCCESS)
		return true;

	libusb_device **deviceList;
	ssize_t deviceCount = libUsb.get_device_list(context, &deviceList);
	for (ssize_t i = 0; i < deviceCount; i++)
	{
		libusb_device *device = deviceList[i];
		libusb_device_descriptor desc;
		result = libUsb.get_device_descriptor(device, &desc);
		if (result != LIBUSB_SUCCESS)
			continue;

		if (desc.idVendor != XSENS_VENDOR_ID)
			continue;

		libusb_device_handle *handle;
		result = libUsb.open(device, &handle);
		if (result != LIBUSB_SUCCESS)
			continue;

		unsigned char serialNumber[256];
		result = libUsb.get_string_descriptor_ascii(handle, desc.iSerialNumber, serialNumber, 256);

		libusb_config_descriptor *configDesc;
		result = libUsb.get_active_config_descriptor(device, &configDesc);
		if (result != LIBUSB_SUCCESS)
		{
			libUsb.close(handle);
			continue;
		}

		bool kernelActive = false;
		for (uint8_t ifCount = 0; ifCount < configDesc->bNumInterfaces; ++ifCount) {
			int res = libUsb.kernel_driver_active(handle, ifCount);
			kernelActive |= (res == 1);
		}

		libUsb.free_config_descriptor(configDesc);

		if (!kernelActive)
		{
			char name[256];
			sprintf(name, "USB%03u:%03u", libUsb.get_bus_number(device),
								libUsb.get_device_address(device));
			current.setPortName(name);

			int id = 0;
			sscanf((const char *)serialNumber, "%X", &id);
			current.setDeviceId((uint32_t) id);
			current.setVidPid(desc.idVendor, desc.idProduct);
			ports.push_back(current);
		}
		else
		{
			JLDEBUGG("Kernel driver active on USB" <<
				libUsb.get_bus_number(device) << ":" << libUsb.get_device_address(device) <<
					" device " << serialNumber);

		}
		libUsb.close(handle);
	}
	libUsb.free_device_list(deviceList, 1);
	libUsb.exit(context);
	return true;
#else
	(void)ports;
	return true;
#endif
}
