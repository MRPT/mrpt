/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "xcommunicationconfig.h"
#include "enumerateusbdevices.h"

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
#	include <libusb-1.0/libusb.h>
#	ifdef HAVE_UDEV
#		include <libudev.h>
#	endif
#endif

#define XSENS_VENDOR_ID				0x2639
#define ATMEL_VENDOR_ID				0x03eb // needed for old MTw
#define ATMEL_BORROWED_PRODUCT_ID	0x2307 // needed for old MTw

/*! \brief Enumerate Xsens USB devices

	If the OS already has drivers running for a device, the device should already have been
	found by xsEnumerateSerialPorts().

	\param[in,out] ports The list of serial ports to append to
*/
bool xsEnumerateUsbDevices(XsPortInfoList& ports)
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

			ports.push_back(current);
		}
	}

	return true;
#else
	libusb_context *context;
	int result = libusb_init(&context);
	if (result != LIBUSB_SUCCESS)
		return false;

	libusb_device **deviceList;
	ssize_t deviceCount = libusb_get_device_list(context, &deviceList);
	for (ssize_t i = 0; i < deviceCount; i++)
	{
		libusb_device *device = deviceList[i];
		libusb_device_descriptor desc;
		result = libusb_get_device_descriptor(device, &desc);
		if (result != LIBUSB_SUCCESS)
			continue;

		if (desc.idVendor != XSENS_VENDOR_ID && desc.idVendor != ATMEL_VENDOR_ID)
			continue;

		libusb_device_handle *handle;
		result = libusb_open(device, &handle);
		if (result != LIBUSB_SUCCESS)
		{
			libusb_unref_device(device);
			continue;
		}

		unsigned char serialNumber[256];
		result = libusb_get_string_descriptor_ascii(handle, desc.iSerialNumber, serialNumber, 256);

		if (desc.idVendor == ATMEL_VENDOR_ID && desc.idProduct == ATMEL_BORROWED_PRODUCT_ID)
		{
			unsigned char productName[256];
			result = libusb_get_string_descriptor_ascii(handle, desc.iProduct, productName, 256);

			if (strcmp("Xsens COM port", (const char *)productName) != 0)
			{
				libusb_close(handle);
				continue;
			}
		}

		libusb_config_descriptor *configDesc;
		result = libusb_get_active_config_descriptor(device, &configDesc);
		if (result != LIBUSB_SUCCESS)
			continue;

		bool kernelActive = false;
		for (uint8_t ifCount = 0; ifCount < configDesc->bNumInterfaces; ++ifCount) {
			int res = libusb_kernel_driver_active(handle, ifCount);
			kernelActive |= (res == 1);

		}

		libusb_free_config_descriptor(configDesc);

		if (!kernelActive)
		{
			char name[256];
			sprintf(name, "USB%03u:%03u", libusb_get_bus_number(device),
								libusb_get_device_address(device));
			current.setPortName(name);

			int id = 0;
			sscanf((const char *)serialNumber, "%d", &id);
			current.setDeviceId((uint32_t) id);
			ports.push_back(current);
		}
		else
		{
			JLDEBUG(gJournal, "Kernel driver active on USB" <<
				libusb_get_bus_number(device) << ":" << libusb_get_device_address(device) <<
					" device " << serialNumber);

		}
		libusb_close(handle);
	}
	libusb_free_device_list(deviceList, 1);
	libusb_exit(context);
	return true;
#endif
}
