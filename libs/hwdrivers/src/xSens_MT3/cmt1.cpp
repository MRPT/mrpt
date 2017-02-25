/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/*! \file Cmt1.cpp

	For information about objects in this file, see the appropriate header:
	\ref Cmt1.h

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
	\par 2006-04-12, v0.0.1
	\li Job Mulder:	Created
	\par 2006-07-21, v0.1.0
	\li Job Mulder:	Updated file for release 0.1.0
*/

#include "cmt1.h"
#include <errno.h>

#include <mrpt/config.h> // For HAVE_MALLOC_H

#ifndef _WIN32
#	include <unistd.h>		// close
#	include <sys/ioctl.h>	// ioctl
#	include <fcntl.h>		// open, O_RDWR
#	include <string.h>		// strcpy

/* Jerome Monceaux : bilock@gmail.com
 * Add a specific case for apple
 */
#ifdef HAVE_MALLOC_H
# include <malloc.h>
#elif defined(HAVE_MALLOC_MALLOC_H)
# include <malloc/malloc.h>
#endif

#	include <stdarg.h>		// va_start, etc...
#   include <sys/param.h>
// We have to redefine PATH_MAX from 4096 to CMT_MAX_FILENAME_LENGTH to mainain compatibility
// The PATH_MAX definition is used by realpath() to determine the maximum path length. According
// to the realpath (3) man page, the function is best avoided and it might be necessary to
// write a custom function for it (couldn't find a proper replacement).
#   undef PATH_MAX
#   define PATH_MAX CMT_MAX_FILENAME_LENGTH
#   include <stdlib.h>
#else
#	include <stdio.h>		// fseek
#   include <io.h>
#endif

#ifndef _CRT_SECURE_NO_DEPRECATE
#	define _CRT_SECURE_NO_DEPRECATE
#	ifdef _WIN32
#		pragma warning(disable:4996)
#	endif
#endif

#ifdef _WIN32
#   ifdef _MSC_VER
#	    define FSEEK(x)		_fseeki64(m_handle, x, SEEK_SET)
#	    define FSEEK_R(x)	_fseeki64(m_handle, x, SEEK_END)
#	    define FTELL()		_ftelli64(m_handle)
#   else
#	    define FSEEK(x)		fseek(m_handle, x, SEEK_SET)
#	    define FSEEK_R(x)	fseek(m_handle, x, SEEK_END)
#	    define FTELL()		ftell(m_handle)
#   endif
#else
#	define FSEEK(x)		fseeko(m_handle, x, SEEK_SET)
#	define FSEEK_R(x)	fseeko(m_handle, x, SEEK_END)
#	define FTELL()		ftello(m_handle)
#endif

// The namespace of all Xsens software since 2006.
namespace xsens {

#ifndef _WIN32
int _wcsnicmp(const wchar_t* s1, const wchar_t* s2,int count)
{
	for (int i = 0; i < count; ++i, ++s1, ++s2)
		if (*s1 == L'\0')
			if (*s2 == L'\0')
				return 0;
			else
				return -1;
		else
			if (*s2 == L'\0')
				return 1;
			else
				if (*s1 < *s2)
					return -1;
				else if (*s1 > *s2)
					return 1;
	return 0;
}
#endif


#if defined(_DEBUG) || defined(_LOG_ALWAYS)
	#if !defined(_LOG_TO_DBVIEW)
		#ifdef _LOG_TO_STDOUT
		#else	// !dbview && !stdout
			FILE* debug_log_fp = NULL;
			int32_t debug_log_valid = 0;

			FILE* debug_qlog_fp = NULL;
			int32_t debug_qlog_valid = 0;
		#endif
	#endif

// write to a log file/screen/debug-stream
void CMTLOG(const char *str, ...)
{
	#ifdef _LOG_TO_STDOUT
		va_list ptr;
		va_start(ptr,str);
		vprintf(str,ptr);
	#else
	#ifdef _LOG_TO_DBVIEW
		char buf[2048];

		va_list ptr;
		va_start(ptr,str);
		vsprintf(buf,str,ptr);

		OutputDebugString(buf);
	#else
		if (debug_log_valid == 0)
		{
			debug_log_fp = fopen("debug_log_cmt.log","w");
			if (debug_log_fp != NULL)
				debug_log_valid = 1;
			else
				debug_log_valid = -1;
		}
		if (debug_log_valid == 1)
		{
			char buf[2048];

			va_list ptr;
			va_start(ptr,str);
			int32_t sz = vsprintf_s(buf,str,ptr);

			uint32_t nw = getTimeOfDay();
			fprintf(debug_log_fp,"%5u.%03u %s",nw/1000,nw%1000,buf);
			//fwrite(buf,1,sz,debug_log_fp);
			fflush(debug_log_fp);
		}
	#endif
	#endif
}
#endif

// maybe log to nothing at this level
#ifdef _LOG_CMT1
	#define CMT1LOG		CMTLOG
#else
	#define CMT1LOG(...)
#endif

//////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////// Cmt1s  /////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////
// Default constructor, initializes all members to their default values.
Cmt1s::Cmt1s() :
	m_onBytesReceived(NULL)
{
	m_port = 0;
	m_isOpen = false;
	m_lastResult = XRV_OK;
	m_timeout = CMT1_DEFAULT_TIMEOUT;
	m_endTime = 0;
	m_baudrate = 0;

	#ifdef _LOG_RX_TX
		rx_log = NULL;
		tx_log = NULL;
	#endif
}

//////////////////////////////////////////////////////////////////////////////////////////
// Destructor, de-initializes, frees memory allocated for buffers, etc.
Cmt1s::~Cmt1s()
{
	close();
}

//////////////////////////////////////////////////////////////////////////////////////////
// Close the serial communication port.
XsensResultValue Cmt1s::close (void)
{
	#ifdef _LOG_RX_TX
		if (rx_log != NULL)
			fclose(rx_log);
		if (tx_log != NULL)
			fclose(tx_log);
		rx_log = NULL;
		tx_log = NULL;
	#endif
	if (!m_isOpen)
		return m_lastResult = XRV_NOPORTOPEN;

	#ifdef _WIN32
		::FlushFileBuffers(m_handle);
		// read all data before closing the handle, a Flush is not enough for FTDI devices unfortunately
		// we first need to set the COMM timeouts to instantly return when no more data is available
			COMMTIMEOUTS cto;
			::GetCommTimeouts(m_handle,&cto);
			cto.ReadIntervalTimeout = MAXDWORD;
			cto.ReadTotalTimeoutConstant = 0;
			cto.ReadTotalTimeoutMultiplier = 0;
			::SetCommTimeouts(m_handle,&cto);
			char buffer[1024];
			DWORD length;
			do {
				::ReadFile(m_handle, buffer, 1024, &length, NULL);
			} while (length > 0);
		::CloseHandle(m_handle);
	#else
		::close(m_handle);
	#endif
	m_isOpen = false;
	m_endTime = 0;

	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Manipulate the Serial control lines
XsensResultValue Cmt1s::escape (const CmtControlLine mask, const CmtControlLine state)
{
	if (!m_isOpen)
		return (m_lastResult = XRV_NOPORTOPEN);
#ifdef _WIN32
	BOOL rv = 0;
	if (mask & CMT_CONTROL_DTR)
	{
		if (state & CMT_CONTROL_DTR)
			rv = EscapeCommFunction(m_handle,SETDTR);
		else
			rv = EscapeCommFunction(m_handle,CLRDTR);
	}

	if (mask & CMT_CONTROL_RTS)
	{
		if (state & CMT_CONTROL_RTS)
			rv = EscapeCommFunction(m_handle,SETRTS);
		else
			rv = EscapeCommFunction(m_handle,CLRRTS);
	}
	if (rv)
		return m_lastResult = XRV_OK;
	else
		return m_lastResult = XRV_ERROR;
#else
	bool rv = true;
	int32_t status;
	if (mask & CMT_CONTROL_DTR)
	{
		if (ioctl(m_handle, TIOCMGET, &status) == -1)
		{
			if (state & CMT_CONTROL_DTR) status |= TIOCM_DTR;
			else status &= ~TIOCM_DTR;
			rv = (ioctl(m_handle, TIOCMSET, &status) == -1);
		}
		else
			rv = false;
	}
	if (rv && (mask & CMT_CONTROL_RTS))
	{
		if (ioctl(m_handle, TIOCMGET, &status) == -1)
		{
			if (state & CMT_CONTROL_RTS) status |= TIOCM_RTS;
			else status &= ~TIOCM_RTS;
			rv = (ioctl(m_handle, TIOCMSET, &status) == -1);
		}
		else
			rv = false;
	}
	if (rv)
		return m_lastResult = XRV_OK;
	else
		return m_lastResult = XRV_ERROR;
#endif
}

//////////////////////////////////////////////////////////////////////////////////////////
// Flush all data to be transmitted / received.
XsensResultValue Cmt1s::flushData (void)
{
	#ifdef _WIN32
		// Remove any 'old' data in buffer
		PurgeComm(m_handle, PURGE_TXCLEAR | PURGE_RXCLEAR);
	#else
		tcflush(m_handle, TCIOFLUSH);
	#endif
	m_endTime = 0;
	return (m_lastResult = XRV_OK);
}

//////////////////////////////////////////////////////////////////////////////////////////
// Open a communication channel to the given serial port name.
XsensResultValue Cmt1s::open(  const char *portName,
						const uint32_t baudRate,
						uint32_t readBufSize,
						uint32_t writeBufSize)
{
	MRPT_UNUSED_PARAM(readBufSize); MRPT_UNUSED_PARAM(writeBufSize);
	m_endTime = 0;

	CMT1LOG("L1: Open port %s at %d baud\n", portName, baudRate);

	if (m_isOpen)
	{
		CMT1LOG("L1: Port already open\n");
		return (m_lastResult = XRV_ALREADYOPEN);
	}
	m_baudrate = baudRate;

#ifdef _WIN32
	char winPortName[32];

	// Open port
	sprintf(winPortName, "\\\\.\\%s", portName);
	m_handle = CreateFileA(winPortName, GENERIC_READ | GENERIC_WRITE, 0, NULL,
									OPEN_EXISTING, 0, NULL);
	if (m_handle == INVALID_HANDLE_VALUE)
	{
		CMT1LOG("L1: Port cannot be opened\n");
		return (m_lastResult = XRV_INPUTCANNOTBEOPENED);
	}

	// Once here, port is open
	m_isOpen = true;

	//Get the current state & then change it
	GetCommState(m_handle, &m_commState);	// Get current state

	m_commState.BaudRate = baudRate;			// Setup the baud rate
	m_commState.Parity = NOPARITY;				// Setup the Parity
	m_commState.ByteSize = 8;					// Setup the data bits
	m_commState.StopBits = TWOSTOPBITS;			// Setup the stop bits
	m_commState.fDsrSensitivity = FALSE;		// Setup the flow control
	m_commState.fOutxCtsFlow = FALSE;			// NoFlowControl:
	m_commState.fOutxDsrFlow = FALSE;
	m_commState.fOutX = FALSE;
	m_commState.fInX = FALSE;
	if (!SetCommState(m_handle, (LPDCB)&m_commState)) {// Set new state
		// Bluetooth ports cannot always be opened with 2 stopbits
		// Now try to open port with 1 stopbit.
		m_commState.StopBits = ONESTOPBIT;
		if (!SetCommState(m_handle, (LPDCB)&m_commState)) {
			CloseHandle(m_handle);
			m_handle = INVALID_HANDLE_VALUE;
			m_isOpen = false;
			return (m_lastResult = XRV_INPUTCANNOTBEOPENED);
		}
	}
	m_port = atoi(&portName[3]);
	sprintf(m_portname, "%s", portName);

	setTimeout(m_timeout);

	// Other initialization functions
	EscapeCommFunction(m_handle, SETRTS);		// Enable RTS (for Xbus Master use)
	// Set DTR (Calibration sensors need DTR to startup, won't hurt otherwise
	EscapeCommFunction(m_handle, SETDTR);
	SetupComm(m_handle,readBufSize,writeBufSize);	// Set queue size

	// Remove any 'old' data in buffer
	//PurgeComm(m_handle, PURGE_TXCLEAR | PURGE_RXCLEAR);
	PurgeComm(m_handle, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);
#else // !_WIN32
	// Open port
	m_handle = ::open(portName, O_RDWR | O_NOCTTY);
	// O_RDWR: Read+Write
	// O_NOCTTY: Raw input, no "controlling terminal"
	// O_NDELAY: Don't care about DCD signal

	if (m_handle < 0) {
		// Port not open
		return m_lastResult = XRV_INPUTCANNOTBEOPENED;
	}

	// Once here, port is open
	m_isOpen = true;

	/* Start configuring of port for non-canonical transfer mode */
	// Get current options for the port
	tcgetattr(m_handle, &m_commState);

	// Set baudrate.
	cfsetispeed(&m_commState, baudRate);
	cfsetospeed(&m_commState, baudRate);

	// Enable the receiver and set local mode
	m_commState.c_cflag |= (CLOCAL | CREAD);
	// Set character size to data bits and set no parity Mask the characte size bits
	m_commState.c_cflag &= ~(CSIZE|PARENB);
	m_commState.c_cflag |= CS8;		// Select 8 data bits
	m_commState.c_cflag |= CSTOPB;	// send 2 stop bits
	// Disable hardware flow control
	m_commState.c_cflag &= ~CRTSCTS;
	m_commState.c_lflag &= ~(ECHO|ECHONL|ICANON|ISIG|IEXTEN);
	// Disable software flow control
	m_commState.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL|IXON);
	// Set Raw output
	m_commState.c_oflag &= ~OPOST;
	// Timeout 0.001 sec for first byte, read minimum of 0 bytes
	m_commState.c_cc[VMIN]     = 0;
	m_commState.c_cc[VTIME]    = (m_timeout+99)/100;	// 1

	// Set the new options for the port
	tcsetattr(m_handle,TCSANOW, &m_commState);

	m_port = 0;
	sprintf(m_portname, "%s", portName);

	tcflush(m_handle, TCIOFLUSH);

	// setting RTS and DTR; RTS for Xbus Master, DTR for calibration sensors
	int cmbits;
	if (ioctl(m_handle, TIOCMGET, &cmbits) < 0)
	{
		return (m_lastResult = XRV_ERROR);
	}

	cmbits |= TIOCM_RTS|TIOCM_DTR;

	if (ioctl(m_handle, TIOCMSET, &cmbits) < 0)
	{
		return (m_lastResult = XRV_ERROR);
	}
#endif // !_WIN32

	CMT1LOG("L1: Port opened\n");
	return (m_lastResult = XRV_OK);
}

#ifdef _WIN32
//////////////////////////////////////////////////////////////////////////////////////////
// Open a communication channel to the given COM port number.
XsensResultValue Cmt1s::open (	const uint32_t portNumber,
						const uint32_t baudRate,
						uint32_t readBufSize,
						uint32_t writeBufSize)
{
	char comFileName[32];

	// Create file name
	sprintf(comFileName, "COM%u", portNumber);

	return Cmt1s::open(comFileName, baudRate, readBufSize, writeBufSize);
}
#endif

//////////////////////////////////////////////////////////////////////////////////////////
// Read data from the serial port and put it into the data buffer.
XsensResultValue Cmt1s::readData (const uint32_t maxLength, uint8_t* data,
													uint32_t* length)
{
	CMT1LOG("L1: readData, maxlength=%u, length=%p\n",maxLength,length);
	uint32_t ln;
	if (length == NULL)
		length = &ln;

	if (!m_isOpen)
		return (m_lastResult = XRV_NOPORTOPEN);

#ifdef _WIN32
	BOOL rres = ::ReadFile(m_handle, data, maxLength, (DWORD*)length, NULL);
	if (m_onBytesReceived != NULL && *length > 0)
	{
		CmtBinaryData* bytes = (CmtBinaryData*) malloc(sizeof(CmtBinaryData));
		bytes->m_size = *length;
		bytes->m_portNr = m_port;
		memcpy(bytes->m_data,data,*length);
#ifdef _LOG_CALLBACKS
		CMTLOG("C1: onBytesReceived(%d,(%d,%d),%p)\n",(int32_t) m_onBytesReceivedInstance, (int32_t) bytes->m_size, (int32_t) bytes->m_portNr, m_onBytesReceivedParam);
#endif
		m_onBytesReceived(m_onBytesReceivedInstance,CMT_CALLBACK_ONBYTESRECEIVED,bytes,m_onBytesReceivedParam);
	}

	if (!rres)
	{
		CMT1LOG("L1: readData, ReadFile returned error %u\n",::GetLastError());
		return (m_lastResult = XRV_ERROR);
	}
#else
	*length = read(m_handle, data, maxLength);
#endif

#ifdef _LOG_RX_TX
	if (*length > 0)
	{
		if (rx_log == NULL)
		{
			char fname[CMT_MAX_FILENAME_LENGTH];
			sprintf(fname,"rx_%03d_%d.log",(int32_t) m_port,m_baudrate);
			rx_log = fopen(fname,"wb");
		}
		fwrite(data,1,*length,rx_log);
	}
#endif

	CMT1LOG((length[0]?"L1: readData returned success, read %u of %u bytes, first: %02x\n":"L1: readData returned success, read %u bytes\n"),length[0],maxLength,data[0]);
	return (m_lastResult = XRV_OK);
}

//////////////////////////////////////////////////////////////////////////////////////////
// Set the callback function for when bytes have been received
XsensResultValue Cmt1s::setCallbackFunction(CmtCallbackSelector tp, int32_t instance, CmtCallbackFunction func, void* param)
{
	if (tp == CMT_CALLBACK_ONBYTESRECEIVED)
	{
		m_onBytesReceived = func;
		m_onBytesReceivedInstance = instance;
		m_onBytesReceivedParam = param;
		return m_lastResult = XRV_OK;
	}
	return m_lastResult = XRV_INVALIDPARAM;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Set the default timeout value to use in blocking operations.
XsensResultValue Cmt1s::setTimeout (const uint32_t ms)
{
	CMT1LOG("L1: Setting timeout to %u ms\n",ms);

	m_timeout = ms;
#ifdef _WIN32
	// Set COM timeouts
	COMMTIMEOUTS commTimeouts;

	GetCommTimeouts(m_handle,&commTimeouts);	// Fill CommTimeouts structure

	// immediate return if data is available, wait 1ms otherwise
	if (m_timeout > 0)
	{
		commTimeouts.ReadIntervalTimeout = 0;
		commTimeouts.ReadTotalTimeoutConstant = m_timeout;	// ms time
		commTimeouts.ReadTotalTimeoutMultiplier = 0;
		commTimeouts.WriteTotalTimeoutConstant = m_timeout;
		commTimeouts.WriteTotalTimeoutMultiplier = 0;
	}
	else
	{
	// immediate return whether data is available or not
		commTimeouts.ReadIntervalTimeout = MAXDWORD;
		commTimeouts.ReadTotalTimeoutConstant = 0;
		commTimeouts.ReadTotalTimeoutMultiplier = 0;
		commTimeouts.WriteTotalTimeoutConstant = 0;
		commTimeouts.WriteTotalTimeoutMultiplier = 0;
	}

	SetCommTimeouts(m_handle, &commTimeouts);	// Set CommTimeouts structure
#else
	// Timeout 0.1 sec for first byte, read minimum of 0 bytes
	m_commState.c_cc[VMIN]     = 0;
	m_commState.c_cc[VTIME]    = (m_timeout+99)/100;		// ds time

	// Set the new options for the port if it is open
	if (m_isOpen)
		tcsetattr(m_handle,TCSANOW, &m_commState);
#endif
	return (m_lastResult = XRV_OK);
}

//////////////////////////////////////////////////////////////////////////////////////////
// Wait for data to arrive or a timeout to occur.
XsensResultValue Cmt1s::waitForData (const uint32_t maxLength,
							  uint8_t* data, uint32_t* length)
{
	CMT1LOG("L1: waitForData, mto=%u, length=%p\n",m_timeout,length);
	uint32_t timeout = m_timeout;

	uint32_t ln;
	if (length == NULL)
		length = &ln;
	uint32_t eTime = getTimeOfDay(NULL) + timeout;
	uint32_t newLength = 0;

	*length = 0;
	while ((*length < maxLength) && (getTimeOfDay() <= eTime))
	{
		readData(maxLength - *length, data + *length, &newLength);
		*length += newLength;
	}
	CMT1LOG("L1: waitForData result: read %u of %u bytes\n",length[0],maxLength);

	if (length[0] < maxLength)
		return (m_lastResult = XRV_TIMEOUT);
	else
		return (m_lastResult = XRV_OK);
}

//////////////////////////////////////////////////////////////////////////////////////////
// Write the data to the serial port.
XsensResultValue Cmt1s::writeData (const uint32_t length,  const uint8_t* data,
								uint32_t* written)
{
	uint32_t bytes;
	if (written == NULL)
		written = &bytes;

	if (!m_isOpen)
		return (m_lastResult = XRV_NOPORTOPEN);

#ifdef _WIN32
	if (WriteFile(m_handle, data, length, (DWORD*)written, NULL))
	{
#ifdef _LOG_RX_TX
		if (written[0] > 0)
		{
			if (tx_log == NULL)
			{
				char fname[CMT_MAX_FILENAME_LENGTH];
				sprintf(fname,"tx_%03d_%d.log",(int32_t) m_port,m_baudrate);
				tx_log = fopen(fname,"wb");
			}
			fwrite(data,1,*written,tx_log);
		}
#endif
		return (m_lastResult = XRV_OK);
	}
	else
		return (m_lastResult = XRV_ERROR);
#else
	*written = write(m_handle, data, length);
//	if (*written == length)
		return (m_lastResult = XRV_OK);
//	else
//		return (m_lastResult = XRV_ERROR);
#endif
}

//////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////// Cmt1f  /////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////
// Default constructor, initializes all members to their default values.
Cmt1f::Cmt1f()
{
	m_readPos = 0;
	m_writePos = 0;
	m_lastResult = XRV_OK;
	m_reading = true;
	m_isOpen = false;
	m_filename[0] = '\0';
	m_fileSize = 0;
	m_readOnly = false;
	m_unicode = false;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Destructor.
Cmt1f::~Cmt1f()
{
	close();
}

//////////////////////////////////////////////////////////////////////////////////////////
// Write data to the end of the file.
XsensResultValue Cmt1f::appendData (const uint32_t length,  const void* data)
{
	if (!m_isOpen)
		return m_lastResult = XRV_NOFILEOPEN;
	if (m_readOnly)
		return m_lastResult = XRV_READONLY;

	if (m_reading || m_writePos != m_fileSize)
	{
		m_reading = false;
		FSEEK_R(0);
	}
	fwrite(data, 1, length, m_handle);
	m_writePos = FTELL();
	m_fileSize = m_writePos;

	return (m_lastResult = XRV_OK);
}

//////////////////////////////////////////////////////////////////////////////////////////
// Close the file.
XsensResultValue Cmt1f::close (void)
{
	if (m_isOpen)
	{
	#ifdef _WIN32
		fflush(m_handle);
		fclose(m_handle);
	#else
		::fflush(m_handle);
		::fclose(m_handle);
	#endif
	}
	m_isOpen = false;
	m_readPos = 0;
	m_writePos = 0;
	m_reading = true;
	m_fileSize = 0;
	m_readOnly = false;

	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Close the file and delete it.
XsensResultValue Cmt1f::closeAndDelete(void)
{
	if (m_isOpen)
	{
	#ifdef _WIN32
		fflush(m_handle);
		fclose(m_handle);
	#else
		::fflush(m_handle);
		::fclose(m_handle);
	#endif
		if (m_readOnly)
			m_lastResult = XRV_READONLY;
		else
		{
#ifdef _WIN32
			if (m_unicode)
			{
				if (_wremove(m_filename_w) != 0)
					m_lastResult = XRV_READONLY;
				else
					m_lastResult = XRV_OK;
			}
			else
#endif
			{
#ifdef _WIN32
				if (_unlink(m_filename) != 0)
#else
				if (unlink(m_filename) != 0)
#endif
					m_lastResult = XRV_READONLY;
				else
					m_lastResult = XRV_OK;
			}
		}
	}
	else
		m_lastResult = XRV_NOFILEOPEN;

	m_isOpen = false;
	m_readPos = 0;
	m_writePos = 0;
	m_reading = true;
	m_fileSize = 0;
	m_readOnly = false;

	return m_lastResult;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Create a new file.
XsensResultValue Cmt1f::create (const char* filename)
{
	if (m_isOpen)
		return m_lastResult = XRV_ALREADYOPEN;

	//! \test does this work for non-existing files? Or do we need a check and create?
	m_handle = fopen(filename, "w+b");	// open for update (r/w)
	if (m_handle == NULL)
		return m_lastResult = XRV_OUTPUTCANNOTBEOPENED;

	#ifdef _WIN32
		if (_fullpath(m_filename,filename,CMT_MAX_FILENAME_LENGTH) == NULL)
		{
			fclose(m_handle);
			remove(filename);
			return m_lastResult = XRV_INVALIDPARAM;
		}
	#else
		// based on the assumption that this doesn't concern the serial port, handle
		// it the same way using realpath(). Apparently realpath() doesn't require a
		// maximum length. One would possibly want to write a wrapper for it.
		if (realpath(filename, m_filename) == NULL)
        {
            fclose(m_handle);
            remove(filename);
            return m_lastResult = XRV_INVALIDPARAM;
        }
	#endif
	mbstowcs(m_filename_w,m_filename,CMT_MAX_FILENAME_LENGTH);
	m_unicode = false;

	m_isOpen = true;
	m_readPos = 0;
	m_writePos = 0;
	m_fileSize = 0;
	m_reading = true;
	m_readOnly = false;
	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Create a new file.
XsensResultValue Cmt1f::create (const wchar_t* filename)
{
	if (m_isOpen)
		return m_lastResult = XRV_ALREADYOPEN;

#ifdef _WIN32
	//! \test does this work for non-existing files? Or do we need a check and create?
	m_handle = _wfopen(filename, L"w+b");	// open for update (r/w)
	if (m_handle == NULL)
		return m_lastResult = XRV_OUTPUTCANNOTBEOPENED;

	if (_wfullpath(m_filename_w,filename,CMT_MAX_FILENAME_LENGTH) == NULL)
	{
		fclose(m_handle);
		_wremove(filename);
		return m_lastResult = XRV_INVALIDPARAM;
	}
	wcstombs(m_filename,m_filename_w,CMT_MAX_FILENAME_LENGTH);

	m_isOpen = true;
	m_readPos = 0;
	m_writePos = 0;
	m_fileSize = 0;
	m_reading = true;
	m_readOnly = false;
#else
	MRPT_UNUSED_PARAM(filename);
	char tFilename[CMT_MAX_FILENAME_LENGTH*2];
	wcstombs(tFilename,m_filename_w,CMT_MAX_FILENAME_LENGTH);
	XsensResultValue res = create(tFilename);
	if (res != XRV_OK)
		return res;
#endif
	m_unicode = true;
	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Delete the given data from the file.
XsensResultValue Cmt1f::deleteData (const CmtFilePos start, const uint32_t length)
{
	if (!m_isOpen)
		return m_lastResult = XRV_NOFILEOPEN;
	if (m_readOnly)
		return m_lastResult = XRV_READONLY;

	gotoWrite();

	CmtFilePos wPos = start;
	CmtFilePos rPos = wPos + length;

	size_t read1;
	CmtFilePos endPos = (start + (CmtFilePos) length);
	if (endPos < m_fileSize)
	{
		CmtFilePos remaining = m_fileSize - endPos;
		char buffer[512];

		// copy data
		FSEEK(rPos);

		while (remaining > 0)
		{
			if (remaining >= 512)
				read1 = fread(buffer,1,512,m_handle);
			else
				read1 = fread(buffer,1,(size_t) remaining,m_handle);

			remaining -= read1;
			rPos += read1;

			// write block to the correct position
			FSEEK(wPos);
			wPos += fwrite(buffer, 1, read1, m_handle);
			FSEEK(rPos);
		}
		m_fileSize -= length;
	}
	else
	{
		m_fileSize = start;
	}

#ifdef _WIN32
	int32_t rv = _chsize(_fileno(m_handle),(int32_t) m_fileSize);
#else
	int32_t rv = (ftruncate(fileno(m_handle),(int32_t) m_fileSize) == 0);
#endif
	int32_t eno = 0;
	if (rv != 0)
		eno = errno;
	m_writePos = start;
	FSEEK(wPos);
	if (rv != 0)
	{
		switch(eno)
		{
		case EACCES:
			return m_lastResult = XRV_BUSY;
		case EBADF:
			return m_lastResult = XRV_INVALIDINSTANCE;
		case ENOSPC:
			return m_lastResult = XRV_OUTOFMEMORY;
		case EINVAL:
			return m_lastResult = XRV_INVALIDPARAM;
		default:
			return m_lastResult = XRV_ERROR;
		}
	}

	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Find a string of bytes in the file
XsensResultValue Cmt1f::find (const void* needleV, const uint32_t needleLength, CmtFilePos& pos)
{
	if (!m_isOpen)
		return m_lastResult = XRV_NOFILEOPEN;

	const char* needle = (const char*) needleV;

	gotoRead();

	pos = 0;

	char buffer[512];
	uint32_t bufferPos, needlePos = 0;
	size_t readBytes;
	if (m_readPos & 0x1FF)										// read a block of data
		readBytes = fread(buffer,1,(512-((size_t) m_readPos & 0x1FF)),m_handle);
	else
		readBytes = fread(buffer,1,512,m_handle);		// read a block of data

	while (readBytes > 0)
	{
		m_readPos += readBytes;
		bufferPos = 0;

		while (bufferPos < readBytes && needlePos < needleLength)
		{
			if (buffer[bufferPos] == needle[needlePos])
			{
				// found a byte
				++needlePos;
			}
			else
			{
				if (needlePos > 0)
					needlePos = 0;
				else
				if (buffer[bufferPos] == needle[0])
				{
					// found a byte
					needlePos = 1;
				}
			}
			++bufferPos;
		}
		if (needlePos < needleLength)
			readBytes = fread(buffer,1,512,m_handle);	// read next block
		else
		{
			m_readPos = m_readPos + bufferPos - readBytes - needleLength; // or without needleLength
			pos = m_readPos; // - needleLength;
			FSEEK(m_readPos);
			return m_lastResult = XRV_OK;
		}
	}
	return m_lastResult = XRV_ENDOFFILE;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Flush all data to be written.
XsensResultValue Cmt1f::flushData (void)
{
	fflush(m_handle);

	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Retrieve the filename that was last successfully opened.
XsensResultValue Cmt1f::getName(char* filename) const
{
	strcpy(filename, m_filename);
	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Retrieve the filename that was last successfully opened.
XsensResultValue Cmt1f::getName(wchar_t* filename) const
{
#ifdef _WIN32
	wcscpy(filename, m_filename_w);
#else
	mbstowcs(filename, m_filename, CMT_MAX_FILENAME_LENGTH);
#endif
	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Change from writing to reading mode
void Cmt1f::gotoRead(void)
{
	if (m_reading)
		return;

	FSEEK(m_readPos);
	m_reading = true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Change from reading to writing mode
void Cmt1f::gotoWrite(void)
{
	if (!m_reading)
		return;

	FSEEK(m_writePos);
	m_reading = false;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Insert the given data into the file.
XsensResultValue Cmt1f::insertData (const CmtFilePos start, const uint32_t length, const void* data)
{
	if (!m_isOpen)
		return m_lastResult = XRV_NOFILEOPEN;
	if (m_readOnly)
		return m_lastResult = XRV_READONLY;

	gotoWrite();

	CmtFilePos rPos = start;
	CmtFilePos wPos = rPos + length;

	size_t read1, read2;
	CmtFilePos remaining = m_fileSize - start;
	size_t bsize = (length > 512)?length:512;
	char* buffer1 = (char*) malloc(bsize);
	char* buffer2 = (char*) malloc(bsize);
	char* btemp;

	// copy data
	FSEEK(rPos);

	if (remaining >= (CmtFilePos) bsize)
		read1 = fread(buffer1,1,bsize,m_handle);
	else
		read1 = fread(buffer1,1,(size_t) remaining,m_handle);

	remaining -= read1;
	rPos += read1;

	while(remaining > 0)
	{
		// move data to correct buffer
		read2 = read1;
		btemp = buffer1; buffer1 = buffer2; buffer2 = btemp;

		// read next block
		if (remaining >= (CmtFilePos) bsize)
			read1 = fread(buffer1,1,bsize,m_handle);
		else
			read1 = fread(buffer1,1,(size_t) remaining,m_handle);

		remaining -= read1;
		rPos += read1;

		// write block to the correct position
		FSEEK(wPos);
		wPos += fwrite(buffer2, 1, read2, m_handle);
		FSEEK(rPos);
	}

	FSEEK(wPos);
	wPos += fwrite(buffer1, 1, read1, m_handle);

	FSEEK(start);
	m_writePos = start + fwrite(data, 1, length, m_handle);
	m_fileSize += length;

	free(buffer1);
	free(buffer2);
	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Open a file.
XsensResultValue Cmt1f::open(const char* filename, const bool create, const bool readOnly)
{
	if (m_isOpen)
		return m_lastResult = XRV_ALREADYOPEN;

	//! \test does this work for non-existing files? Or do we need a check and create?
	m_readOnly = readOnly;
	if (readOnly)
		m_handle = fopen(filename, "rb");	// open for read only (r)
	else
		m_handle = fopen(filename, "r+b");	// open for update (r/w)
	if (m_handle == NULL)
	{
		if (create)
			m_handle = fopen(filename, "w+b");	// create for update (r/w)
		else
		{
			m_handle = fopen(filename, "rb");	// open for read only (r)
			m_readOnly = true;
		}
	}
	if (m_handle == NULL)
		return m_lastResult = XRV_INPUTCANNOTBEOPENED;

	#ifdef _WIN32
		if (_fullpath(m_filename,filename,CMT_MAX_FILENAME_LENGTH) == NULL)
		{
			fclose(m_handle);
			return m_lastResult = XRV_INVALIDPARAM;
		}
	#else
	    // use the same trick again.
		if (realpath(filename, m_filename) == NULL)
		{
		    fclose(m_handle);
		    return m_lastResult = XRV_INVALIDPARAM;
		}
	#endif
	mbstowcs(m_filename_w,m_filename,CMT_MAX_FILENAME_LENGTH);
	m_unicode = false;

	m_isOpen = true;
	m_readPos = 0;
	m_writePos = 0;
	m_reading = true;
	FSEEK_R(0);
	m_fileSize = FTELL();
	FSEEK(0);
	return (m_lastResult = XRV_OK);
}

//////////////////////////////////////////////////////////////////////////////////////////
// Open a file.
XsensResultValue Cmt1f::open(const wchar_t* filename, const bool create, const bool readOnly)
{
	if (m_isOpen)
		return m_lastResult = XRV_ALREADYOPEN;

#ifdef _WIN32
	//! \test does this work for non-existing files? Or do we need a check and create?
	m_readOnly = readOnly;
	if (readOnly)
		m_handle = _wfopen(filename, L"rb");	// open for read only (r)
	else
		m_handle = _wfopen(filename, L"r+b");	// open for update (r/w)
	if (m_handle == NULL)
	{
		if (create)
			m_handle = _wfopen(filename, L"w+b");	// create for update (r/w)
		else
		{
			m_handle = _wfopen(filename, L"rb");	// open for read only (r)
			m_readOnly = true;
		}
	}
	if (m_handle == NULL)
		return m_lastResult = XRV_INPUTCANNOTBEOPENED;

	if (_wfullpath(m_filename_w,filename,CMT_MAX_FILENAME_LENGTH) == NULL)
	{
		fclose(m_handle);
		return m_lastResult = XRV_INVALIDPARAM;
	}
	wcstombs(m_filename,m_filename_w,CMT_MAX_FILENAME_LENGTH);

	m_isOpen = true;
	m_readPos = 0;
	m_writePos = 0;
	m_reading = true;
	FSEEK_R(0);
	m_fileSize = FTELL();
	FSEEK(0);
#else
	char tFilename[CMT_MAX_FILENAME_LENGTH*2];
	wcstombs(tFilename,filename,CMT_MAX_FILENAME_LENGTH*2);
	XsensResultValue res = open(tFilename,create,readOnly);
	if (res != XRV_OK)
		return res;
#endif
	m_unicode = true;
	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Read data from the file and put it into the data buffer.
XsensResultValue Cmt1f::readData(const uint32_t maxLength, void* data, uint32_t* length)
{
	if (!m_isOpen)
		return m_lastResult = XRV_NOFILEOPEN;

	if (maxLength == 0)
		return m_lastResult = XRV_OK;

	uint32_t len;
	if (length == NULL)
		length = &len;

	gotoRead();

	length[0] = (uint32_t) fread(data,1,maxLength,m_handle);
	if (length[0] == 0)
		return (m_lastResult = XRV_ENDOFFILE);

	m_readPos += length[0];
	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Read data from the file until the terminator and put it into the data buffer.
XsensResultValue Cmt1f::readData (const uint32_t maxLength, const char terminator, void* dataV, uint32_t* length)
{
	if (!m_isOpen)
		return m_lastResult = XRV_NOFILEOPEN;

	uint32_t len;
	if (length == NULL)
		length = &len;

	char* data = (char*) dataV;
	int32_t readChar;

	gotoRead();

	*length = 0;
	readChar = (uint32_t) fgetc(m_handle);

	while (!feof(m_handle) && !ferror(m_handle))
	{
		data[*length] = (char) readChar;
		++(*length);
		++m_readPos;

		if (((char) readChar == terminator) || ((*length) >= maxLength))
			return m_lastResult = XRV_OK;
	}
	return m_lastResult = XRV_ENDOFFILE;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Set the new absolute read position
XsensResultValue Cmt1f::setReadPos (const CmtFilePos pos)
{
	if (!m_isOpen)
		return m_lastResult = XRV_NOFILEOPEN;

	if (m_readPos != pos)
	{
		m_readPos = pos;
		if (m_reading)
			FSEEK(m_readPos);
	}

	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Set the new absolute write position
XsensResultValue Cmt1f::setWritePos(const CmtFilePos pos)
{
	if (!m_isOpen)
		return m_lastResult = XRV_NOFILEOPEN;
	if (m_readOnly)
		return m_lastResult = XRV_READONLY;

	if (pos == -1)
	{
		if (m_reading)
			m_reading = false;
		FSEEK_R(0);
		m_writePos = FTELL();
	}
	else
	{
		if (m_writePos != pos)
		{
			m_writePos = pos;
			if (!m_reading)
				FSEEK(m_writePos);
		}
	}

	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Write data to the file.
XsensResultValue Cmt1f::writeData (const uint32_t length,  const void* data)
{
	if (!m_isOpen)
		return m_lastResult = XRV_NOFILEOPEN;
	if (m_readOnly)
		return m_lastResult = XRV_READONLY;

	gotoWrite();
	m_writePos += fwrite(data, 1, length, m_handle);

	if (m_writePos > m_fileSize)
		m_fileSize = m_writePos;

	return m_lastResult = XRV_OK;
}

}	// end of xsens namespace
