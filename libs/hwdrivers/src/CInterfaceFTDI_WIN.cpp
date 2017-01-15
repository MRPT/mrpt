/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h"   // Precompiled headers

#ifdef MRPT_OS_WINDOWS

/*===========================================================================
	~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
								START OF FTD2XX.H
	~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  ===========================================================================*/
namespace hwdrivers
{
// The following ifdef block is the standard way of creating macros which make exporting
// from a DLL simpler. All files within this DLL are compiled with the FTD2XX_EXPORTS
// symbol defined on the command line. this symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see
// FTD2XX_API functions as being imported from a DLL, wheras this DLL sees symbols
// defined with this macro as being exported.
#ifdef FTD2XX_EXPORTS
#define FTD2XX_API __declspec(dllexport)
#else
#define FTD2XX_API __declspec(dllimport)
#endif


typedef unsigned long FT_HANDLE;

//
// FT_OpenEx Flags
//

#define FT_OPEN_BY_SERIAL_NUMBER    1
#define FT_OPEN_BY_DESCRIPTION      2

//
// FT_ListDevices Flags (used in conjunction with FT_OpenEx Flags
//

#define FT_LIST_NUMBER_ONLY			0x80000000
#define FT_LIST_BY_INDEX			0x40000000
#define FT_LIST_ALL					0x20000000

#define FT_LIST_MASK (FT_LIST_NUMBER_ONLY|FT_LIST_BY_INDEX|FT_LIST_ALL)

//
// Baud Rates
//

#define FT_BAUD_300			300
#define FT_BAUD_600			600
#define FT_BAUD_1200		1200
#define FT_BAUD_2400		2400
#define FT_BAUD_4800		4800
#define FT_BAUD_9600		9600
#define FT_BAUD_14400		14400
#define FT_BAUD_19200		19200
#define FT_BAUD_38400		38400
#define FT_BAUD_57600		57600
#define FT_BAUD_115200		115200
#define FT_BAUD_230400		230400
#define FT_BAUD_460800		460800
#define FT_BAUD_921600		921600

//
// Word Lengths
//

#define FT_BITS_8			(unsigned char) 8
#define FT_BITS_7			(unsigned char) 7
#define FT_BITS_6			(unsigned char) 6
#define FT_BITS_5			(unsigned char) 5

//
// Stop Bits
//

#define FT_STOP_BITS_1		(unsigned char) 0
#define FT_STOP_BITS_1_5	(unsigned char) 1
#define FT_STOP_BITS_2		(unsigned char) 2

//
// Parity
//

#define FT_PARITY_NONE		(unsigned char) 0
#define FT_PARITY_ODD		(unsigned char) 1
#define FT_PARITY_EVEN		(unsigned char) 2
#define FT_PARITY_MARK		(unsigned char) 3
#define FT_PARITY_SPACE		(unsigned char) 4

//
// Flow Control
//

#define FT_FLOW_NONE        0x0000
#define FT_FLOW_RTS_CTS     0x0100
#define FT_FLOW_DTR_DSR     0x0200
#define FT_FLOW_XON_XOFF    0x0400

//
// Purge rx and tx buffers
//
#define FT_PURGE_RX         1
#define FT_PURGE_TX         2

//
// Events
//

typedef void (*PFT_EVENT_HANDLER)(unsigned long,unsigned long);

#define FT_EVENT_RXCHAR		    1
#define FT_EVENT_MODEM_STATUS   2

//
// Timeouts
//

#define FT_DEFAULT_RX_TIMEOUT   300
#define FT_DEFAULT_TX_TIMEOUT   300

} // end namespace hwdrivers
/*===========================================================================
	~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
								END OF FTD2XX.H
	~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  ===========================================================================*/

#include <windows.h>

#include <mrpt/hwdrivers/CInterfaceFTDI.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::hwdrivers;
using namespace std;


/*-------------------------------------------------------------
					CInterfaceFTDI
-------------------------------------------------------------*/
CInterfaceFTDI::CInterfaceFTDI()  :
	m_readBuffer(4096)
{
	MRPT_START

	m_ftHandle = 0;
	loadDriver();

	MRPT_END
}

/*-------------------------------------------------------------
					~CInterfaceFTDI
-------------------------------------------------------------*/
CInterfaceFTDI::~CInterfaceFTDI()
{
	if(m_hmodule != NULL)
	{
		// Close USB connection:
		this->Close();

		// Unload FT2XX DLL:
		FreeLibrary( (HMODULE)m_hmodule );
		m_hmodule=NULL;
	}
}

/** This object cannot be copied */
CInterfaceFTDI::CInterfaceFTDI(const CInterfaceFTDI &o) :
	m_readBuffer(4096)
{
	MRPT_START
	THROW_EXCEPTION("This object cannot be copied");
	MRPT_END
}
CInterfaceFTDI& CInterfaceFTDI::operator =(const CInterfaceFTDI &o)
{
	MRPT_START
	THROW_EXCEPTION("This object cannot be copied");
	MRPT_END
}


/*-------------------------------------------------------------
					isOpen
-------------------------------------------------------------*/
bool  CInterfaceFTDI::isOpen()
{
	return m_ftHandle!=0;
}

/*-------------------------------------------------------------
					loadDriver
-------------------------------------------------------------*/
void	 CInterfaceFTDI::loadDriver()
{
	MRPT_START
	// ------------------------------------------------------
	//				Windoze version
	// ------------------------------------------------------
	m_hmodule = ::LoadLibraryA("Ftd2xx.dll");
	if(m_hmodule == NULL)	THROW_EXCEPTION("Error: Cannot load Ftd2xx.dll");

	m_pWrite = (PtrToWrite)GetProcAddress((HMODULE)m_hmodule, "FT_Write");
	m_pRead = (PtrToRead)GetProcAddress((HMODULE)m_hmodule, "FT_Read");
	m_pOpen = (PtrToOpen)GetProcAddress((HMODULE)m_hmodule, "FT_Open");
	m_pOpenEx = (PtrToOpenEx)GetProcAddress((HMODULE)m_hmodule, "FT_OpenEx");
	m_pListDevices = (PtrToListDevices)GetProcAddress((HMODULE)m_hmodule, "FT_ListDevices");
	m_pClose = (PtrToClose)GetProcAddress((HMODULE)m_hmodule, "FT_Close");
	m_pResetDevice = (PtrToResetDevice)GetProcAddress((HMODULE)m_hmodule, "FT_ResetDevice");
	m_pPurge = (PtrToPurge)GetProcAddress((HMODULE)m_hmodule, "FT_Purge");
	m_pSetTimeouts = (PtrToSetTimeouts)GetProcAddress((HMODULE)m_hmodule, "FT_SetTimeouts");
	m_pGetQueueStatus = (PtrToGetQueueStatus)GetProcAddress((HMODULE)m_hmodule, "FT_GetQueueStatus");
	m_pSetLatencyTimer = (PtrToSetLatencyTimer)GetProcAddress((HMODULE)m_hmodule, "FT_SetLatencyTimer");

	if( !m_pWrite || !m_pRead || !m_pOpen ||
		!m_pOpenEx || !m_pListDevices || !m_pClose ||
		!m_pResetDevice || !m_pPurge || !m_pSetTimeouts ||
		!m_pGetQueueStatus || !m_pSetLatencyTimer)
			THROW_EXCEPTION("Error loading FTD2XX.DLL");

	MRPT_END
}

/*-------------------------------------------------------------
				FTD2XX.DLL INTERFACE FUNCTIONS
-------------------------------------------------------------*/
void   CInterfaceFTDI::ftdi_open(void* pvDevice)
{
	MRPT_START
	if (isOpen()) Close();

	ASSERT_(m_pOpen);
	checkErrorAndRaise( (*m_pOpen)(pvDevice, &m_ftHandle ) );

	MRPT_END
}

void   CInterfaceFTDI::ftdi_openEx(void*pArg1, unsigned long dwFlags)
{
	MRPT_START
	if (isOpen()) Close();

	ASSERT_(m_pOpenEx);
	checkErrorAndRaise( (*m_pOpenEx)(pArg1,dwFlags,&m_ftHandle) );

	MRPT_END
}

/*-------------------------------------------------------------
					ListAllDevices
-------------------------------------------------------------*/
void CInterfaceFTDI::ListAllDevices( TFTDIDeviceList &outList )
{
	MRPT_START

	outList.clear();

	unsigned long		nConectedDevices;
	char				str[100];

	// Get the number of devices:
	ftdi_listDevices(&nConectedDevices,NULL, 0x80000000);

	for (size_t i=0;i<nConectedDevices;i++)
	{
		TFTDIDevice		newEntry;

		// Serial number:
		ftdi_listDevices( (void*)(i),(void*)str, (unsigned long)(0x40000000 | 1));
		newEntry.ftdi_serial = str;

		// Description:
		ftdi_listDevices( (void*)(i),(void*)str, (unsigned long)(0x40000000 | 2));
		newEntry.ftdi_description = str;

		outList.push_back(newEntry);
	}

	MRPT_END
}


void CInterfaceFTDI::ftdi_listDevices(void*pArg1, void*pArg2, unsigned long dwFlags)
{
	MRPT_START

	ASSERT_(m_pListDevices);
	checkErrorAndRaise( (*m_pListDevices)(pArg1, pArg2, dwFlags) );

	MRPT_END
}

void   CInterfaceFTDI::Close()
{
	MRPT_START

	if (m_ftHandle)
	{
		ASSERT_(m_pClose);
		(*m_pClose)( m_ftHandle );
		m_ftHandle = 0;
	}

	m_readBuffer.clear();

	MRPT_END
}

void   CInterfaceFTDI::ftdi_read(void  *lpvBuffer, unsigned long dwBuffSize, unsigned long  *lpdwBytesRead)
{
	MRPT_START

	ASSERT_(m_pRead);
	checkErrorAndRaise( (*m_pRead)( m_ftHandle, lpvBuffer, dwBuffSize, lpdwBytesRead ) );

	MRPT_END
}

void   CInterfaceFTDI::ftdi_write(const void  *lpvBuffer, unsigned long dwBuffSize, unsigned long  *lpdwBytes)
{
	MRPT_START

	ASSERT_(m_pWrite);
	checkErrorAndRaise(  (*m_pWrite)( m_ftHandle, lpvBuffer, dwBuffSize, lpdwBytes ) );

	MRPT_END
}

void   CInterfaceFTDI::ResetDevice()
{
	MRPT_START

	ASSERT_(m_pResetDevice);
	checkErrorAndRaise( (*m_pResetDevice) ( m_ftHandle ));

	m_readBuffer.clear();

	MRPT_END
}

void   CInterfaceFTDI::Purge()
{
	MRPT_START

	ASSERT_(m_pPurge);
	unsigned long dwMask = FT_PURGE_RX | FT_PURGE_TX;
	checkErrorAndRaise( (*m_pPurge)( m_ftHandle, dwMask ) );

	m_readBuffer.clear();
	MRPT_END
}

void   CInterfaceFTDI::SetTimeouts(unsigned long dwReadTimeout_ms, unsigned long dwWriteTimeout_ms)
{
	MRPT_START

	ASSERT_(m_pSetTimeouts);
	checkErrorAndRaise( (*m_pSetTimeouts)( m_ftHandle, dwReadTimeout_ms,dwWriteTimeout_ms  ) );

	MRPT_END
}

void   CInterfaceFTDI::ftdi_getQueueStatus(unsigned long  *lpdwAmountInRxQueue)
{
	MRPT_START

	ASSERT_(m_pGetQueueStatus);
	checkErrorAndRaise( (*m_pGetQueueStatus) ( m_ftHandle, lpdwAmountInRxQueue  ) );

	MRPT_END
}

void   CInterfaceFTDI::SetLatencyTimer (unsigned char latency_ms)
{
	MRPT_START

	ASSERT_(m_pSetLatencyTimer);
	checkErrorAndRaise( (*m_pSetLatencyTimer)( m_ftHandle, latency_ms ) );

	MRPT_END
}

/*-------------------------------------------------------------
					checkErrorAndRaise
-------------------------------------------------------------*/
void  CInterfaceFTDI::checkErrorAndRaise(int errorCode)
{
	/** Possible responses from the driver
	enum FT_STATUS
	{
		FT_OK = 0,
		FT_INVALID_HANDLE,
		FT_DEVICE_NOT_FOUND,
		FT_DEVICE_NOT_OPENED,
		FT_IO_ERROR,
		FT_INSUFFICIENT_RESOURCES,
		FT_INVALID_PARAMETER
	};  */
	switch (errorCode)
	{
	case 0: return;
	case 1: Close(); THROW_EXCEPTION("*** FTD2XX ERROR ***: FT_INVALID_HANDLE");
	case 2: Close(); THROW_EXCEPTION("*** FTD2XX ERROR ***: FT_DEVICE_NOT_FOUND");
	case 3: Close(); THROW_EXCEPTION("*** FTD2XX ERROR ***: FT_DEVICE_NOT_OPENED");
	case 4: Close(); THROW_EXCEPTION("*** FTD2XX ERROR ***: FT_IO_ERROR");
	case 5: THROW_EXCEPTION("*** FTD2XX ERROR ***: FT_INSUFFICIENT_RESOURCES");
	case 6: THROW_EXCEPTION("*** FTD2XX ERROR ***: FT_INVALID_PARAMETER");
	default: THROW_EXCEPTION("*** FTD2XX ERROR ***: Invalid error code!?!?!?");
	};
}


/*-------------------------------------------------------------
					OpenBySerialNumber
-------------------------------------------------------------*/
void  CInterfaceFTDI::OpenBySerialNumber( const std::string &serialNumber )
{
	MRPT_START
	m_readBuffer.clear();

	ftdi_openEx( (void*)serialNumber.c_str(), FT_OPEN_BY_SERIAL_NUMBER);
	MRPT_END
}

/*-------------------------------------------------------------
					OpenByDescription
-------------------------------------------------------------*/
void  CInterfaceFTDI::OpenByDescription( const std::string &description )
{
	MRPT_START
	m_readBuffer.clear();

	ftdi_openEx( (void*)description.c_str(), FT_OPEN_BY_DESCRIPTION);
	MRPT_END
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
