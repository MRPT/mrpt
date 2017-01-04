/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/config.h>

#include <mrpt/utils/utils_defs.h>
#include <cstring>

using namespace mrpt;
using namespace mrpt::utils;

#if defined(MRPT_OS_LINUX) || defined(MRPT_OS_APPLE)

#if MRPT_HAS_FTDI
	#include <ftdi.h>
#	if MRPT_FTDI_VERSION>=0x120
	#include <libusb-1.0/libusb.h>
#else
	#include <usb.h>
#	endif
#endif

#include <mrpt/hwdrivers/CInterfaceFTDI.h>

#include <iostream>

using namespace mrpt::hwdrivers;
using namespace std;

/*-------------------------------------------------------------
					CInterfaceFTDI
-------------------------------------------------------------*/
CInterfaceFTDI::CInterfaceFTDI() :
	m_readBuffer(4096)
{
	MRPT_TRY_START;

#if MRPT_HAS_FTDI
	// Alloc mem:
	ftdi_context *newCtx = new ftdi_context[1];
	ASSERT_(newCtx);

	// Init:
	int ret = ftdi_init(newCtx);
	if (ret) THROW_EXCEPTION("There was a problem initializing ftdi_context.");

	// Save in member:
	m_ftdi_context = static_cast<void*>(newCtx);
#else
	THROW_EXCEPTION("MRPT has been compiled without FTDI support. Please, reconfigure and recompile MRPT.")
#endif

#if MRPT_FTDI_VERSION>=0x120
	libusb_init(NULL);
#endif
	MRPT_TRY_END;
}

/*-------------------------------------------------------------
					~CInterfaceFTDI
-------------------------------------------------------------*/
CInterfaceFTDI::~CInterfaceFTDI()
{
#if MRPT_HAS_FTDI
	// Close USB:
	if (isOpen())
		Close();
	ftdi_context *ctx = static_cast<ftdi_context *>(m_ftdi_context);

	// Close context:
	ftdi_deinit( ctx );

	// Free mem:
	delete[] ctx;
	ctx = NULL;
#endif
#if MRPT_FTDI_VERSION>=0x120
	libusb_exit(NULL);
#endif
}

/** This object cannot be copied */
CInterfaceFTDI::CInterfaceFTDI(const CInterfaceFTDI &) :
	m_readBuffer(4096)
{
	MRPT_TRY_START
	THROW_EXCEPTION("This object cannot be copied");
	MRPT_TRY_END
}
CInterfaceFTDI& CInterfaceFTDI::operator =(const CInterfaceFTDI &)
{
	MRPT_TRY_START
	THROW_EXCEPTION("This object cannot be copied");
	MRPT_TRY_END
}

/*-------------------------------------------------------------
					OpenBySerialNumber
-------------------------------------------------------------*/
void  CInterfaceFTDI::OpenBySerialNumber( const std::string &serialNumber )
{
#if MRPT_HAS_FTDI
	MRPT_TRY_START

	m_readBuffer.clear();

	// Close previous connection:
	Close();


	// ftdi_usb_open_desc ...

	// Create a list of all the devices:
	TFTDIDeviceList lstDevs;
	ListAllDevices( lstDevs );

	// Look for the one we want:
	void 	*myDev = NULL;

	for (TFTDIDeviceList::iterator it=lstDevs.begin();it!=lstDevs.end();++it)
	{
		if (it->ftdi_serial == serialNumber)
		{
			myDev = it->usb_device_struct;
			break;
		}
	}

	if (!myDev)
		THROW_EXCEPTION_CUSTOM_MSG1("USB device with serial number '%s' not found.",serialNumber.c_str());

	// Open it:
	ftdi_context *ctx = static_cast<ftdi_context *>(m_ftdi_context);

	int ret=ftdi_usb_open_dev( ctx, 
#	if MRPT_FTDI_VERSION>=0x120
		(struct libusb_device*) myDev 
#else
		(struct usb_device*) myDev 
#endif
		);

	if (ret) THROW_EXCEPTION( string(ftdi_get_error_string(ctx)) );

	MRPT_TRY_END
#else
	MRPT_UNUSED_PARAM(serialNumber);
#endif
}

/*-------------------------------------------------------------
					ListAllDevices
-------------------------------------------------------------*/
void CInterfaceFTDI::ListAllDevices( TFTDIDeviceList &outList )
{
	MRPT_TRY_START
#if MRPT_HAS_FTDI

	outList.clear();

#if MRPT_FTDI_VERSION>=0x120
	// For new libftdi1-dev
	// Use libusb-1.0 

	libusb_device **list;
	ssize_t nDevices = libusb_get_device_list(NULL,&list);

	for (unsigned int i = 0; i < nDevices; i++) {
		libusb_device *device = list[i];
		struct libusb_device_descriptor desc;
		if (0!=libusb_get_device_descriptor(device,&desc))
			continue;
		if (!desc.idVendor) continue;

		TFTDIDevice	newEntry;
		newEntry.usb_device_struct = (void*)device;
		newEntry.usb_idProduct = desc.idProduct;
		newEntry.usb_idVendor  = desc.idVendor;
		newEntry.usb_serialNumber  = desc.iSerialNumber;

		// Open the device temporally so we can get more info:
		libusb_device_handle *handle;
		if (0!=libusb_open(device,&handle))
			continue;

		char buf[1024];
		int ret;
		// manufacturer
		ret=libusb_get_string_descriptor_ascii(handle,desc.iManufacturer,(unsigned char*)buf,sizeof(buf)-1);
		if (ret<0) continue;
		buf[ret]='\0';
		newEntry.ftdi_manufacturer = buf;
			
		// description
		ret=libusb_get_string_descriptor_ascii(handle,desc.iProduct,(unsigned char*)buf,sizeof(buf)-1);
		if (ret<0) continue;
		buf[ret]='\0';
		newEntry.ftdi_description = buf;

		// serial
		ret=libusb_get_string_descriptor_ascii(handle,desc.iSerialNumber,(unsigned char*)buf,sizeof(buf)-1);
		if (ret<0) continue;
		buf[ret]='\0';
		newEntry.ftdi_serial = buf;
		
		outList.push_back(newEntry);
	}

#else
	// For old libftdi-dev
	// Use old usb.h interface
	struct usb_bus *bus;
	struct usb_device *dev;

	usb_init();
	if (usb_find_busses() < 0)
		THROW_EXCEPTION("usb_find_busses() failed");
	if (usb_find_devices() < 0)
		THROW_EXCEPTION("usb_find_devices() failed");

	for (bus = usb_busses; bus; bus = bus->next)
		for (dev = bus->devices; dev; dev = dev->next)
			recursive_fill_list_devices( dev, outList ); // Process this node and its children:

#endif
	if (getenv("VERBOSE")!=NULL)
	{
		printf("[CInterfaceFTDI::ListAllDevices] List: \n");
		for (std::deque<TFTDIDevice>::const_iterator i=outList.begin();i!=outList.end();++i)
			printf("USB DEV: V=%04X P=%04X S=%s\n",i->usb_idVendor,i->usb_idProduct, i->ftdi_serial.c_str());
	}

#else
	MRPT_UNUSED_PARAM(outList);
#endif
	MRPT_TRY_END
}


void CInterfaceFTDI::recursive_fill_list_devices( void *usb_device_structure , TFTDIDeviceList &outList )
{
#if MRPT_HAS_FTDI
#if MRPT_FTDI_VERSION>=0x120
	// For new libftdi1-dev
	throw std::runtime_error("Should not have got to this function!");
#else
	// For old libftdi-dev
	struct usb_device *dev = (struct usb_device *) usb_device_structure;

	if (dev->descriptor.idProduct && dev->descriptor.idVendor)
	{
		TFTDIDevice	newEntry;
		newEntry.usb_idProduct = dev->descriptor.idProduct;
		newEntry.usb_idVendor  = dev->descriptor.idVendor;
		newEntry.usb_device_struct = (void*)dev;

		int strLen;

		// Open the device temporally so we can get more info:
		usb_dev_handle * hUSB = usb_open(dev);

		if (hUSB)
		{
			char 	manufacturer[3000];
			if ((strLen=usb_get_string_simple(hUSB, dev->descriptor.iManufacturer, manufacturer, sizeof(manufacturer))) <=0)
			{
				cerr << "Couldn't open " << (int)dev->descriptor.iManufacturer << endl;
				//usb_close(hUSB); hUSB=NULL;
			}
			else
			{
				manufacturer[strLen]='\0';
				//cout << "Manuf: " << manufacturer << endl;
				newEntry.ftdi_manufacturer = manufacturer;
			}
		}

		if (hUSB)
		{
			char 	description[3000];
			if ((strLen=usb_get_string_simple(hUSB, dev->descriptor.iProduct, description, sizeof(description))) <=0)
			{
				//usb_close(hUSB); hUSB=NULL;
			}
			else
			{
				description[strLen]='\0';
				newEntry.ftdi_description = description;
			}
		}

		if (hUSB)
		{
			char 	serial[300];
			if ((strLen=usb_get_string_simple(hUSB, dev->descriptor.iSerialNumber, serial, sizeof(serial))) <=0)
			{
				//usb_close(hUSB); hUSB=NULL;
			}
			else
			{
				serial[strLen]='\0';
				newEntry.ftdi_serial = serial;
			}
		}

		if (hUSB)
		{
			outList.push_back( newEntry );
			usb_close( hUSB );
		}

		// And now its children:
		// -----------------------------------
		for (unsigned char j=0;j<dev->num_children;j++)
			recursive_fill_list_devices( (void*)dev->children[j], outList );
	}
#endif
#else
	MRPT_UNUSED_PARAM(usb_device_structure); MRPT_UNUSED_PARAM(outList);
#endif
}

/*-------------------------------------------------------------
					ftdi_read
-------------------------------------------------------------*/
void  CInterfaceFTDI::ftdi_read(void  *lpvBuffer, unsigned long dwBuffSize, unsigned long  *lpdwBytesRead)
{
#if MRPT_HAS_FTDI
	MRPT_TRY_START
	ftdi_context *ctx = static_cast<ftdi_context *>(m_ftdi_context);

	int ret = ftdi_read_data(ctx,(unsigned char*)lpvBuffer, dwBuffSize );
	if (ret>=0)
		*lpdwBytesRead = ret;
	else
	{
		if (!strcmp("usb bulk read failed",ctx->error_str))
		{
			*lpdwBytesRead = 0;
			return;
		}
		THROW_EXCEPTION( string(ftdi_get_error_string(ctx)) );
	}

	MRPT_TRY_END
#else
	MRPT_UNUSED_PARAM(lpvBuffer); MRPT_UNUSED_PARAM(dwBuffSize); MRPT_UNUSED_PARAM(lpdwBytesRead);
#endif
}

/*-------------------------------------------------------------
					ftdi_write
-------------------------------------------------------------*/
void  CInterfaceFTDI::ftdi_write(const void  *lpvBuffer, unsigned long dwBuffSize, unsigned long  *lpdwBytes)
{
#if MRPT_HAS_FTDI
	MRPT_TRY_START
	ftdi_context *ctx = static_cast<ftdi_context *>(m_ftdi_context);

	int ret = ftdi_write_data(ctx,(unsigned char*)lpvBuffer, dwBuffSize );
	if (ret>=0)
		*lpdwBytes = ret;
	else
		THROW_EXCEPTION( string(ftdi_get_error_string(ctx)) );

	MRPT_TRY_END
#else
	MRPT_UNUSED_PARAM(lpvBuffer); MRPT_UNUSED_PARAM(dwBuffSize); MRPT_UNUSED_PARAM(lpdwBytes);
#endif
}

/*-------------------------------------------------------------
					isOpen
-------------------------------------------------------------*/
bool  CInterfaceFTDI::isOpen()
{
#if MRPT_HAS_FTDI
	ftdi_context *ctx = static_cast<ftdi_context *>(m_ftdi_context);
	return ctx->usb_dev != NULL;
#else
	return false;
#endif
}

/*-------------------------------------------------------------
					Close
-------------------------------------------------------------*/
void  CInterfaceFTDI::Close()
{
#if MRPT_HAS_FTDI
	ftdi_context *ctx = static_cast<ftdi_context *>(m_ftdi_context);
	if (ctx->usb_dev)
	{
		ftdi_usb_close( ctx );
		ctx->usb_dev = NULL;
	}

	// To assure this is as a "reset", re-init the ftdi context again:
	ftdi_deinit( ctx );
	ftdi_init( ctx );

	m_readBuffer.clear();
#endif
}

/*-------------------------------------------------------------
					ResetDevice
-------------------------------------------------------------*/
void  CInterfaceFTDI::ResetDevice()
{
#if MRPT_HAS_FTDI
	ftdi_context *ctx = static_cast<ftdi_context *>(m_ftdi_context);
	ASSERT_(ctx->usb_dev);

	if (ftdi_usb_reset(ctx)<0)
		THROW_EXCEPTION("Error resetting device");

	m_readBuffer.clear();
#endif
}

/*-------------------------------------------------------------
					Purge
-------------------------------------------------------------*/
void  CInterfaceFTDI::Purge()
{
#if MRPT_HAS_FTDI
	ftdi_context *ctx = static_cast<ftdi_context *>(m_ftdi_context);
	ASSERT_(ctx->usb_dev);

	if (ftdi_usb_purge_buffers(ctx)<0)
		THROW_EXCEPTION("Error purging device buffers");

	m_readBuffer.clear();
#endif
}

/*-------------------------------------------------------------
					SetLatencyTimer
-------------------------------------------------------------*/
void  CInterfaceFTDI::SetLatencyTimer (unsigned char latency_ms)
{
#if MRPT_HAS_FTDI
	ftdi_context *ctx = static_cast<ftdi_context *>(m_ftdi_context);
	ASSERT_(ctx->usb_dev);

	if (ftdi_set_latency_timer(ctx, latency_ms)<0)
		THROW_EXCEPTION("Error setting latency timer");
#else
	MRPT_UNUSED_PARAM(latency_ms);
#endif
}

/*-------------------------------------------------------------
					SetTimeouts
-------------------------------------------------------------*/
void  CInterfaceFTDI::SetTimeouts(unsigned long dwReadTimeout_ms, unsigned long dwWriteTimeout_ms)
{
#if MRPT_HAS_FTDI
	ftdi_context *ctx = static_cast<ftdi_context *>(m_ftdi_context);
	ASSERT_(ctx->usb_dev);

	// JL: It seems it works worse with timeouts...
//	ctx->usb_read_timeout  = dwReadTimeout_ms;
//	ctx->usb_write_timeout = dwWriteTimeout_ms;
#else
	MRPT_UNUSED_PARAM(dwReadTimeout_ms); MRPT_UNUSED_PARAM(dwWriteTimeout_ms);
#endif
}


/*-------------------------------------------------------------
					OpenByDescription
-------------------------------------------------------------*/
std::ostream & mrpt::hwdrivers::operator << ( std::ostream &o, const TFTDIDevice &d)
{
	o << "Manufacturer            : " << d.ftdi_manufacturer << endl
	  << "Description             : " << d.ftdi_description << endl
	  << "FTDI serial             : " << d.ftdi_serial << endl
	  << "USB ID (Vendor/Product) : " << format("%04X / %04X", d.usb_idVendor, d.usb_idProduct) << endl
	  << "USB serial              : " << d.usb_serialNumber << endl;

	return o;
}

#endif




