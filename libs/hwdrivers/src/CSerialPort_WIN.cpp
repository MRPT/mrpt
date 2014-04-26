/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h"   // Precompiled headers

#include <mrpt/system/os.h>

#ifdef MRPT_OS_WINDOWS

#include <mrpt/hwdrivers/CSerialPort.h>

#include <windows.h>

using namespace std;
using namespace mrpt::utils;
using namespace mrpt::hwdrivers;

/* -----------------------------------------------------
                Constructor
   ----------------------------------------------------- */
CSerialPort::CSerialPort( const std::string &portName, bool openNow )
{
	hCOM = NULL;
	m_serialName = portName;
	if (openNow) open();
}

/* -----------------------------------------------------
                Default constructor
   ----------------------------------------------------- */
CSerialPort::CSerialPort()
{
	hCOM = NULL;
	m_serialName = "";
}

/* -----------------------------------------------------
                Destructor
   ----------------------------------------------------- */
CSerialPort::~CSerialPort()
{
	if ( isOpen() )
		close();
}

/* -----------------------------------------------------
                Open
   ----------------------------------------------------- */
void  CSerialPort::open( )
{
	MRPT_START

    // Check name:
    if (!m_serialName.size()) THROW_EXCEPTION("Serial port name is empty!!")

    // Is it COMX, X>4? ->  "\\.\COMX"
    if ( tolower( m_serialName[0]) =='c' && tolower( m_serialName[1]) =='o' && tolower( m_serialName[2]) =='m' )
    {
        // Need to add "\\.\"?
        if (m_serialName.size()>4 || m_serialName[3]>'4')
			m_serialName = std::string("\\\\.\\") + m_serialName;
    }


	// Open the serial port:
	if ( INVALID_HANDLE_VALUE == (
	            hCOM = CreateFileA(
	                       m_serialName.c_str(), // Serial Port name
	                       GENERIC_READ | GENERIC_WRITE,
	                       0,
	                       NULL,
	                       OPEN_EXISTING,
	                       0,
	                       0) ) )
	{
		hCOM = NULL;
		THROW_EXCEPTION_CUSTOM_MSG1("Error trying to open serial port: %s",m_serialName.c_str() );
	}

	// Set recommended buffer sizes:
	SetupComm(hCOM,4096,4096);

	// Success!
	MRPT_END
}


/* -----------------------------------------------------
                isOpen
   ----------------------------------------------------- */
bool  CSerialPort::isOpen() const 
{
	return hCOM != NULL;
}

/* -----------------------------------------------------
                setConfig
   ----------------------------------------------------- */
void  CSerialPort::setConfig(
    int		baudRate,
    int		parity,
    int		bits,
    int		nStopBits,
	bool	enableFlowControl)
{
    MRPT_START

	DCB			dcb_conf;
	dcb_conf.DCBlength = sizeof(DCB);

	// Port must be open!
	if (!isOpen()) THROW_EXCEPTION("The serial port is not open");

	if (!GetCommState( hCOM,  &dcb_conf ) ) THROW_EXCEPTION("Error retrieving COM state");

    //
    // Apply baud rate
    //
    int BR;
    switch (baudRate)
    {
    	case 300: BR = CBR_300; break;
    	case 600: BR = CBR_600; break;
    	case 1200: BR = CBR_1200; break;
    	case 2400: BR = CBR_2400; break;
    	case 4800: BR = CBR_4800; break;
    	case 9600: BR = CBR_9600; break;
    	case 19200: BR = CBR_19200; break;
    	case 38400: BR = CBR_38400; break;
    	case 57600: BR = CBR_57600; break;
    	case 115200: BR = CBR_115200; break;
    default:
		BR = baudRate;
        //THROW_EXCEPTION_CUSTOM_MSG1("Invalid desired baud rate value: %i",baudRate ) ;
        break;
    }

	dcb_conf.BaudRate = BR;


	dcb_conf.ByteSize = (BYTE)bits;
	dcb_conf.Parity = (BYTE) parity;

    // stop bits:
    switch ( nStopBits )
    {
    case 1:
        dcb_conf.StopBits = ONESTOPBIT;
        break ;
    case 2:
        dcb_conf.StopBits = TWOSTOPBITS;
        break ;
    default:
        THROW_EXCEPTION_CUSTOM_MSG1("Invalid number of stop bits: %i",nStopBits) ;
        break;
    }

	dcb_conf.fBinary = true;
	dcb_conf.fParity = parity!=0;

	dcb_conf.fRtsControl = enableFlowControl ? RTS_CONTROL_HANDSHAKE : RTS_CONTROL_DISABLE;
	dcb_conf.fOutxCtsFlow = enableFlowControl;

	// Apply:
	if (! SetCommState( hCOM, &dcb_conf ) )
		THROW_EXCEPTION("Error changing COM state");

	// Assure:
	if (! GetCommState( hCOM,  &dcb_conf ) )
		THROW_EXCEPTION("Error retrieving COM state");
	if ( ((int)dcb_conf.BaudRate) != baudRate )
		THROW_EXCEPTION("COM state verification after writing failed");

	m_baudRate = baudRate;

    MRPT_END
}

/* -----------------------------------------------------
                setConfig
   ----------------------------------------------------- */
void  CSerialPort::setTimeouts(
    int		ReadIntervalTimeout,
    int		ReadTotalTimeoutMultiplier,
    int		ReadTotalTimeoutConstant,
    int		WriteTotalTimeoutMultiplier,
    int		WriteTotalTimeoutConstant )
{
	COMMTIMEOUTS	timeouts;

	// Port must be open!
	if (!isOpen())
		THROW_EXCEPTION("The COM port is not open");

	// Config:
	timeouts.ReadIntervalTimeout = ReadIntervalTimeout;             // Milisegundos entre dos bytes recibidos
	timeouts.ReadTotalTimeoutMultiplier = ReadTotalTimeoutMultiplier;       // Milisegundos de espera por cada byte a recibir
	timeouts.ReadTotalTimeoutConstant = ReadTotalTimeoutConstant;        // Milisegundos de espera en cada operacion de recepcion
	timeouts.WriteTotalTimeoutMultiplier = WriteTotalTimeoutMultiplier;       // Timeout de escritura no usado
	timeouts.WriteTotalTimeoutConstant = WriteTotalTimeoutConstant;         // Timeout de escritura no usado

	if (!SetCommTimeouts( hCOM, &timeouts ) )
		THROW_EXCEPTION("Error changing COM port timeout config");

	// Success
}

/* -----------------------------------------------------
                Close
   ----------------------------------------------------- */
void  CSerialPort::close(  )
{
	if (hCOM)
		CloseHandle(hCOM);
	hCOM=NULL;
}

/* -----------------------------------------------------
                read
   ----------------------------------------------------- */
size_t  CSerialPort::Read(void *Buffer, size_t Count)
{
	// Port must be open!
	if (!isOpen())
		THROW_EXCEPTION("The port is not open yet!");

	DWORD actuallyRead;

	if (! ReadFile(
	            hCOM,         // Handle,
	            Buffer,          // Buffer
	            (DWORD)Count,  // Max expected bytes
	            &actuallyRead, // Actually read bytes
	            NULL) )
		THROW_EXCEPTION("Error reading from port!");

	return actuallyRead;
}

/** Reads one text line from the serial port in POSIX "canonical mode".
  *  This method reads from the serial port until one of the characters in \a eol are found.
  */
std::string CSerialPort::ReadString(
	const int total_timeout_ms,
	bool *out_timeout,
	const char *eol_chars)
{
    MRPT_TRY_START
    // Calling ::ReadBuffer() many times would be even worse, so replicate its code here:

    ASSERT_(eol_chars!=NULL)

    // Port must be open!
    if (!isOpen()) THROW_EXCEPTION("The port is not open yet!");

    if (out_timeout) *out_timeout = false; // Will be set to true on timeout

    m_timer.Tic();
    string receivedStr; // Rx buffer

    while ( total_timeout_ms<0 || ( m_timer.Tac()*1e3 < total_timeout_ms ) )
    {
		// Read just 1 byte:
		char buf[1];

		DWORD actuallyRead;
		if (! ReadFile(
					hCOM,         // Handle,
					buf,          // Buffer
					1,  // Max expected bytes
					&actuallyRead, // Actually read bytes
					NULL) )
			THROW_EXCEPTION("Error reading from port!");

		if (actuallyRead)
		{	// Append to string, if it's not a control char:
			if (!strchr(eol_chars, buf[0] ) )
				receivedStr.push_back( buf[0] );
			else
			{	// end of string!
				return receivedStr;
			}
		}
		// If we are still here, string is not finished:
		mrpt::system::sleep( 1 ); // Wait 1 more ms for new data to arrive.
    }

	// Timeout:
    if (out_timeout) *out_timeout = true;
    return receivedStr;
    MRPT_TRY_END
}


/* -----------------------------------------------------
                write
   ----------------------------------------------------- */
size_t  CSerialPort::Write(const void *Buffer, size_t Count)
{
	// Port must be open!
	if (!isOpen())
		THROW_EXCEPTION("The port is not open yet!");

	DWORD actuallyWritten;

	if (!WriteFile(
	            hCOM,
	            Buffer,
	            (DWORD)Count,
	            &actuallyWritten,
	            NULL) )
		THROW_EXCEPTION("Error writing to port!");

	return actuallyWritten;
}

/* -----------------------------------------------------
                purgeBuffers
   ----------------------------------------------------- */
void  CSerialPort::purgeBuffers()
{
	// Port must be open!
	if (!isOpen())
		THROW_EXCEPTION("The COM port is not open");

	if ( !PurgeComm( hCOM, PURGE_RXABORT | PURGE_RXCLEAR | PURGE_TXABORT | PURGE_TXCLEAR ) )
		THROW_EXCEPTION("Error during COM port purge");
}


#endif // windows

