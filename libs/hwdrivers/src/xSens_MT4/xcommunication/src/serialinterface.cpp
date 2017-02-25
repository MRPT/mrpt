/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "serialinterface.h"
#include <xsens/xsportinfo.h>
#include <xsens/xscontrolline.h>

#include <errno.h>
#ifndef _WIN32
#	include <unistd.h>		// close
#	include <sys/ioctl.h>	// ioctl
#	include <fcntl.h>		// open, O_RDWR
#	include <string.h>		// strcpy
#	include <sys/param.h>
#	include <sys/file.h>
#	include <stdarg.h>
#else
#	include <winbase.h>
#   include <io.h>
#endif

#ifndef _CRT_SECURE_NO_DEPRECATE
#	define _CRT_SECURE_NO_DEPRECATE
#	ifdef _WIN32
#		pragma warning(disable:4996)
#	endif
#endif

//lint -emacro(534, FSEEK, FSEEK_R)
//lint -emacro({534}, FSEEK, FSEEK_R)
#ifdef _WIN32
#	define FSEEK(x)		_fseeki64(m_handle, x, SEEK_SET)
#	define FSEEK_R(x)	_fseeki64(m_handle, x, SEEK_END)
#	define FTELL()		_ftelli64(m_handle)
#else
#	define FSEEK(x)		fseeko(m_handle, x, SEEK_SET)
#	define FSEEK_R(x)	fseeko(m_handle, x, SEEK_END)
#	define FTELL()		ftello(m_handle)
#endif

// maybe log to nothing at this level
#ifdef LOG_CMT1
#	include "xslog.h"
#	define XDA1LOG_OBSOLETE		XSENSLOG
#else
#	define XDA1LOG_OBSOLETE(...)	(void)0
#endif

//////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////// SerialInterface  /////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

//! \brief Default constructor, initializes all members to their default values.
SerialInterface::SerialInterface()
{
	m_port = 0;
	m_lastResult = XRV_OK;
	m_timeout = 0;
	m_endTime = 0;
	m_baudrate = XBR_Invalid;
	m_portname[0] = 0;
#ifdef _WIN32
	m_handle = INVALID_HANDLE_VALUE;
#else
	m_handle = -1;
	memset(&m_commState, 0, sizeof(m_commState));
#endif

	rx_log = NULL;
	tx_log = NULL;
}

//! Destructor, de-initializes, frees memory allocated for buffers, etc.
SerialInterface::~SerialInterface()
{
	try {
		closeLive();	//lint !e534
	} catch(...)
	{}
}

//! \brief Close the serial communication port.
XsResultValue SerialInterface::close(void)
{
	return closeLive();
}

//! \brief Close the serial communication port.
XsResultValue SerialInterface::closeLive(void)
{
#ifdef LOG_RX_TX
	if (rx_log != NULL)
		fclose(rx_log);
	if (tx_log != NULL)
		fclose(tx_log);
	rx_log = NULL;
	tx_log = NULL;
#endif
	if (!isOpen())
		return m_lastResult = XRV_NOPORTOPEN;

	m_lastResult = XRV_OK;
#ifdef _WIN32
	if (::FlushFileBuffers(m_handle))
	{
		// read all data before closing the handle, a Flush is not enough for FTDI devices unfortunately
		// we first need to set the COMM timeouts to instantly return when no more data is available
		COMMTIMEOUTS cto;
		if (::GetCommTimeouts(m_handle,&cto))
		{
			cto.ReadIntervalTimeout = MAXDWORD;
			cto.ReadTotalTimeoutConstant = 0;
			cto.ReadTotalTimeoutMultiplier = 0;
			if (::SetCommTimeouts(m_handle,&cto))
			{
				char buffer[1024];
				DWORD length;
				do {
					if (!::ReadFile(m_handle, buffer, 1024, &length, NULL))
						break;
				} while (length > 0);
			}
			else
				m_lastResult = XRV_ERROR;
		}
		else
			m_lastResult = XRV_ERROR;
	}
	if (!::CloseHandle(m_handle))
		m_lastResult = XRV_ERROR;
	m_handle = INVALID_HANDLE_VALUE;
#else
	flushData();
	::close(m_handle);
	m_handle = -1;
#endif
	m_endTime = 0;

	return m_lastResult;
}

/*! \brief Manipulate the Serial control lines

	The function manipulates the serial control lines that are indicated by the
	mask parameter. Note that only the DTR and RTS lines can be set by win32.
	\param mask		Indicates which lines are to be manipulated and which should be
					left alone.
	\param state	Contains the new state of the control lines.
	\returns XRV_OK if the function succeeded
*/
XsResultValue SerialInterface::escape (const XsControlLine mask, const XsControlLine state)
{
	//lint --e{655} bitwise operations are intended on these enums
	if (!isOpen())
		return (m_lastResult = XRV_NOPORTOPEN);
#ifdef _WIN32
	BOOL rv = 0;
	if (mask & XCL_DTR)
	{
		if (state & XCL_DTR)
			rv = EscapeCommFunction(m_handle,SETDTR);
		else
			rv = EscapeCommFunction(m_handle,CLRDTR);
	}

	if (mask & XCL_RTS)
	{
		if (state & XCL_RTS)
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
	if (mask & XCL_DTR)
	{
		if (ioctl(m_handle, TIOCMGET, &status) == -1)
		{
			if (state & XCL_DTR) status |= TIOCM_DTR;
			else status &= ~TIOCM_DTR;
			rv = (ioctl(m_handle, TIOCMSET, &status) == -1);
		}
		else
			rv = false;
	}
	if (rv && (mask & XCL_RTS))
	{
		if (ioctl(m_handle, TIOCMGET, &status) == -1)
		{
			if (state & XCL_RTS) status |= TIOCM_RTS;
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

/*! \copydoc IoInterface::flushData
	\note This function tries to send and receive any remaining data immediately
	and does not return until the buffers are empty.
*/
XsResultValue SerialInterface::flushData (void)
{
	m_lastResult = XRV_OK;
#ifdef _WIN32
	// Remove any 'old' data in buffer
	if (!PurgeComm(m_handle, PURGE_TXCLEAR | PURGE_RXCLEAR))
		m_lastResult = XRV_ERROR;
#else
	tcflush(m_handle, TCIOFLUSH);
#endif
	m_endTime = 0;
	return m_lastResult;
}

//! Return the baudrate that is currently being used by the port
XsBaudRate SerialInterface::getBaudrate(void) const
{
	if (isOpen())
		return m_baudrate;
	return XBR_Invalid;
}
//! Return the handle of the port
XsIoHandle SerialInterface::getHandle(void) const { return m_handle; }
//! Retrieve the port number that was last successfully opened.
uint16_t SerialInterface::getPortNumber (void) const { return m_port; }
//! Retrieve the port name that was last successfully opened.
void SerialInterface::getPortName(XsString& portname) const { portname = m_portname; }
//! Return the error code of the last operation.
XsResultValue SerialInterface::getLastResult(void) const { return m_lastResult; }
//! Return the current timeout value
uint32_t SerialInterface::getTimeout (void) const { return m_timeout; }
//! Return whether the communication port is open or not.
bool SerialInterface::isOpen (void) const
{
#ifdef _WIN32
	return m_handle != INVALID_HANDLE_VALUE;
#else
	return m_handle >= 0;
#endif
}

/*! \brief Open a communication channel to the given port info.
	\details If the baudrate in \a portInfo is set to XBR_Invalid, the baud rate is automatically
				detected if possible.
	\param portInfo The details of the port that should be opened. Depending on the type of interface,
				parts of this parameter may be ignored.
	\param readBufSize The size of the read buffer in bytes (if appliccable to the device)
	\param writeBufSize The size of the write buffer in bytes (if appliccable to the device)
	\returns XRV_OK if the device was opened successfully
*/
XsResultValue SerialInterface::open(const XsPortInfo& portInfo,
						uint32_t readBufSize,
						uint32_t writeBufSize)
{
	m_endTime = 0;

	JLDEBUG(gJournal, "port " << portInfo.portName().toStdString() << " at " << portInfo.baudrate() << " baud");

	if (isOpen())
	{
		JLALERT(gJournal, "Port " << portInfo.portName().toStdString() << " is already open");
		return (m_lastResult = XRV_ALREADYOPEN);
	}
	m_baudrate = portInfo.baudrate();

#ifdef _WIN32
	XsResultValue fail = XRV_OK;
	char winPortName[256];

	// Open port
	sprintf(winPortName, "\\\\.\\%s", portInfo.portName().c_str());
	m_handle = CreateFileA(winPortName, GENERIC_READ | GENERIC_WRITE, 0, NULL,
									OPEN_EXISTING, 0, NULL);
	if (m_handle == INVALID_HANDLE_VALUE)
	{
		JLDEBUG(gJournal, "Port " << portInfo.portName().toStdString() << " cannot be opened");
		return (m_lastResult = XRV_INPUTCANNOTBEOPENED);
	}

	DCB commState;		//!< Stored settings about the serial port

	commState.DCBlength = sizeof(DCB);

	//Get the current state & then change it
	if (!GetCommState(m_handle, &commState))	// Get current state
		fail = XRV_ERROR;

	commState.BaudRate = (int) portInfo.baudrate();		// Setup the baud rate
	commState.Parity = NOPARITY;				// Setup the Parity
	commState.ByteSize = 8;					// Setup the data bits
	commState.StopBits = TWOSTOPBITS;			// Setup the stop bits
	commState.fDsrSensitivity = FALSE;		// Setup the flow control
	commState.fOutxCtsFlow = FALSE;			// NoFlowControl:
	commState.fOutxDsrFlow = FALSE;
	commState.fOutX = FALSE;
	commState.fInX = FALSE;
	commState.fRtsControl = RTS_CONTROL_ENABLE;
	if (!SetCommState(m_handle, (LPDCB)&commState)) // Set new state
	{
		// Bluetooth ports cannot always be opened with 2 stopbits
		// Now try to open port with 1 stopbit.
		commState.StopBits = ONESTOPBIT;
		if (!SetCommState(m_handle, (LPDCB)&commState))
			fail = XRV_INPUTCANNOTBEOPENED;
	}
	std::string tmp = portInfo.portName().toStdString();
	m_port = atoi(&tmp.c_str()[3]);
	sprintf(m_portname, "%s", tmp.c_str());

	if (setTimeout(20))
		fail = m_lastResult;

	// Other initialization functions
	if (!EscapeCommFunction(m_handle, SETDTR))			// Set DTR (Calibration sensors need DTR to startup, won't hurt otherwise
		fail = XRV_ERROR;
	if (!SetupComm(m_handle,readBufSize,writeBufSize))	// Set queue size
		fail = XRV_ERROR;

	// Remove any 'old' data in buffer
	//PurgeComm(m_handle, PURGE_TXCLEAR | PURGE_RXCLEAR);
	if (!PurgeComm(m_handle, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR))
		fail = XRV_ERROR;

	if (fail != XRV_OK)
	{
		CloseHandle(m_handle);		//lint !e534
		m_handle = INVALID_HANDLE_VALUE;
		return (m_lastResult = fail);
	}

#else // !_WIN32
	(void)readBufSize;
	(void)writeBufSize;
	// Open port
	std::string pn = portInfo.portName().toStdString();
	m_handle = ::open(pn.c_str(), O_RDWR | O_NOCTTY);

	// O_RDWR: Read+Write
	// O_NOCTTY: Raw input, no "controlling terminal"
	// O_NDELAY: Don't care about DCD signal

	if (m_handle < 0) {
		// Port not open
		return m_lastResult = XRV_INPUTCANNOTBEOPENED;
	}

	// Check if the file is already opened by someome else (other thread/process)
	if (flock(m_handle, LOCK_EX | LOCK_NB))
	{
		closeLive();
		return m_lastResult = XRV_INPUTCANNOTBEOPENED;
	}

	/* Start configuring of port for non-canonical transfer mode */
	// Get current options for the port
	if (tcgetattr(m_handle, &m_commState) != 0)
		return XRV_ERROR;

	// Set baudrate.
	if (cfsetispeed(&m_commState, portInfo.baudrate()) != 0)
		return XRV_ERROR;

	if (cfsetospeed(&m_commState, portInfo.baudrate()) != 0)
		return XRV_ERROR;

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
	if (tcsetattr(m_handle,TCSANOW, &m_commState) != 0)
		return XRV_INPUTCANNOTBEOPENED;

	termios checkCommState;
	if (tcgetattr(m_handle, &checkCommState) != 0)
		return XRV_ERROR;

	if ((m_commState.c_cflag != checkCommState.c_cflag) ||
		(m_commState.c_iflag != checkCommState.c_iflag) ||
		(m_commState.c_oflag != checkCommState.c_oflag) ||
		(m_commState.c_cc[VMIN] != checkCommState.c_cc[VMIN]) ||
		(m_commState.c_cc[VTIME] != checkCommState.c_cc[VTIME]))
	{
		JLDEBUG(gJournal, "commstates do not match, which is OK for USB connected MkIV devices");
	}

	m_port = 1;
	sprintf(m_portname, "%s", pn.c_str());

	tcflush(m_handle, TCIOFLUSH);

	// setting RTS and DTR; RTS for Xbus Master, DTR for calibration sensors
	int cmbits;
	if (ioctl(m_handle, TIOCMGET, &cmbits) < 0)
	{
		JLDEBUG(gJournal, "TIOCMGET failed, which is OK for USB connected MkIV devices");
	}

	cmbits |= TIOCM_RTS|TIOCM_DTR;

	if (ioctl(m_handle, TIOCMSET, &cmbits) < 0)
	{
		JLDEBUG(gJournal, "TIOCMSET failed, which is OK for USB connected MkIV devices");
	}
#endif // !_WIN32

	JLDEBUG(gJournal, "Port " << portInfo.portName().toStdString() << " opened");
	return (m_lastResult = XRV_OK);
}

/*! \brief Helper function for making filename of log file unique
*/
bool doesFileExist(char* filename)
{
	FILE* pf = fopen(filename, "r");
	if (pf == NULL)
		return false;
	fclose (pf);
	return true;
}

/*! \brief Helper function for making filename of log file unique
*/
void makeFilenameUnique(char* filename)
{
	if (doesFileExist(filename))	// if a file already exist with the same name,
	{
		// create a unique filename by adding a counter:
		char filename2[XS_MAX_FILENAME_LENGTH];
		char basename[XS_MAX_FILENAME_LENGTH];
		strcpy(basename, filename);
		basename[strlen(basename) - 4] = 0;	// remove .log extension
		int counter = 1;
		do
		{
			sprintf(filename2, "%s_%d.log", basename, counter++);
			if (counter > 10)	// don't count further than 10
			{
				sprintf(filename2, "%s_n.log", basename);
				break;
			}
		}
		while (doesFileExist(filename2));
		strcpy(filename, filename2);
	}
}

/*! \brief Read data from the serial port and put it into the data buffer.
	\details This function reads up to \a maxLength bytes from the port (non-blocking) and
	puts it into the \a data buffer.
	\param maxLength The maximum amount of data read.
	\param data The buffer that will store the received data.
	\returns XRV_OK if no error occurred. It can be that no data is available and XRV_OK will be
			returned. Check data.size() for the number of bytes that were read.
*/
XsResultValue SerialInterface::readData(XsSize maxLength, XsByteArray& data)
{
	if (!isOpen())
		return (m_lastResult = XRV_NOPORTOPEN);

#ifdef _WIN32
	DWORD length;
	data.setSize(maxLength);
	BOOL rres = ::ReadFile(m_handle, data.data(), (DWORD) maxLength, &length, NULL);
	data.pop_back(maxLength-length);
	JLTRACE(gJournal, "ReadFile result " << rres << ", length " << length);

	if (!rres)
	{
		DWORD wErr = ::GetLastError();
		JLALERT(gJournal, "ReadFile returned windows error " << wErr);
		if (wErr >= ERROR_INVALID_FUNCTION && wErr <= ERROR_INVALID_HANDLE)
			return (m_lastResult = XRV_NOFILEORPORTOPEN);
		return (m_lastResult = XRV_ERROR);
	}

	if (length == 0)
		return (m_lastResult = XRV_TIMEOUT);
#else
	fd_set fd;
	fd_set err;
	timeval timeout;
	FD_ZERO(&fd);
	FD_ZERO(&err);
	FD_SET(m_handle, &fd);
	FD_SET(m_handle, &err);

	timeout.tv_sec = m_timeout/1000;
	timeout.tv_usec = (m_timeout - (timeout.tv_sec * 1000)) * 1000;

	int res = select(FD_SETSIZE, &fd, NULL, &err, &timeout);
	if (res < 0 || FD_ISSET(m_handle, &err))
	{
		data.clear();
		return (m_lastResult = XRV_ERROR);
	} else if (res == 0) {
		data.clear();
		return (m_lastResult = XRV_TIMEOUT);
	}

	data.setSize(maxLength);
	int length = read(m_handle, (void*)data.data(), maxLength);
	data.pop_back(maxLength - length);
#endif

#ifdef LOG_RX_TX
	if (length > 0)
	{
		if (rx_log == NULL)
		{
			char fname[XS_MAX_FILENAME_LENGTH];
#ifdef _WIN32
			sprintf(fname, "rx_%03d_%d.log", (int32_t) m_port, m_baudrate);
#else
			char *devname = strrchr(m_portname, '/');
			sprintf(fname, "rx_%s_%d.log", devname + 1, XsBaud::rateToNumeric(m_baudrate));
#endif
			makeFilenameUnique(fname);
			
			rx_log = fopen(fname, "wb");
		}
		fwrite(data.data(), 1, length, rx_log);
#ifdef LOG_RX_TX_FLUSH
		fflush(rx_log);
#endif
	}
#endif

	JLTRACE(gJournal, "returned success, read " << length << " of " << maxLength << " bytes, first: " << JLHEXLOG(data[0]));
	return (m_lastResult = XRV_OK);
}

/*! \brief Set the default timeout value to use in blocking operations.
	\details This function sets the value of m_timeout. There is no infinity value. The value 0
	means that all blocking operations now become polling (non-blocking) operations.
	If the value is set to or from 0, the low-level serial port settings may be
	changed in addition to the m_timeout value.
	\param ms The new timeout in milliseconds
	\returns XRV_OK if the function succeeded
*/
XsResultValue SerialInterface::setTimeout (const uint32_t ms)
{
	JLDEBUG(gJournal, "Setting timeout to " << ms << " ms");

	m_timeout = ms;
#ifdef _WIN32
	// Set COM timeouts
	COMMTIMEOUTS commTimeouts;

	if (!GetCommTimeouts(m_handle,&commTimeouts))	// Fill CommTimeouts structure
		return m_lastResult = XRV_ERROR;

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

	if (!SetCommTimeouts(m_handle, &commTimeouts))	// Set CommTimeouts structure
		return m_lastResult = XRV_ERROR;
#else
	// Timeout 0.1 sec for first byte, read minimum of 0 bytes
	m_commState.c_cc[VMIN]     = 0;
	m_commState.c_cc[VTIME]    = (m_timeout+99)/100;		// ds time

	// Set the new options for the port if it is open
	if (isOpen())
		tcsetattr(m_handle,TCSANOW, &m_commState);
#endif
	return (m_lastResult = XRV_OK);
}

/*! \brief Wait for data to arrive or a timeout to occur.
	\details The function waits until \c maxLength data is available or until a timeout occurs.
	The function returns success if data is available or XsResultValue::TIMEOUT if a
	timeout occurred. A timeout value of 0 indicates that the default timeout stored
	in the class should be used.
	\param maxLength The maximum number of bytes to read before returning
	\param data The buffer to put the read data in.
	\returns XRV_OK if \a maxLength bytes were read, XRV_TIMEOUT if less was read, XRV_TIMEOUTNODATA if nothing was read
*/
XsResultValue SerialInterface::waitForData(XsSize maxLength, XsByteArray& data)
{
	data.clear();
	data.reserve(maxLength);

	//char *data = (char *)&_data[0];
	JLTRACE(gJournal, "timeout=" << m_timeout << ", maxLength=" << maxLength);
	uint32_t timeout = m_timeout;

	uint32_t eTime = XsTime_getTimeOfDay(NULL, NULL) + timeout;
//	uint32_t newLength = 0;

	while ((data.size() < maxLength) && (XsTime_getTimeOfDay(NULL, NULL) <= eTime))
	{
		XsByteArray raw;

		if (readData(maxLength - data.size(), raw) != XRV_OK)
			return m_lastResult;
		data.append(raw);
	}
	JLTRACE(gJournal, "Read " << data.size() << " of " << maxLength << " bytes");

	if (data.size() < maxLength)
		return (m_lastResult = XRV_TIMEOUT);
	else
		return (m_lastResult = XRV_OK);
}

/*! \copydoc IoInterface::writeData
	\note The default timeout is respected in this operation.
*/
XsResultValue SerialInterface::writeData (const XsByteArray& data, XsSize* written)
{
	XsSize bytes;
	if (written == NULL)
		written = &bytes;

	if (!isOpen())
		return (m_lastResult = XRV_NOPORTOPEN);

	*written = 0;

#ifdef _WIN32
	DWORD lwritten = 0;
	if (WriteFile(m_handle, data.data(), (DWORD) data.size(), &lwritten, NULL) == 0)
		return (m_lastResult = XRV_ERROR);

	*written = lwritten;
#else
	ssize_t result = write(m_handle, (const void*)data.data(), data.size());
	if (result < 0)
		return (m_lastResult = XRV_ERROR);

	*written = result;
#endif

#ifdef LOG_RX_TX
	if (written[0] > 0)
	{
		if (tx_log == NULL)
		{
			char fname[XS_MAX_FILENAME_LENGTH];
#ifdef _WIN32
			sprintf(fname, "tx_%03d_%d.log", (int32_t) m_port, m_baudrate);
#else
			char *devname = strrchr(m_portname, '/');
			sprintf(fname,"tx_%s_%d.log", devname + 1, XsBaud::rateToNumeric(m_baudrate));
#endif
			makeFilenameUnique(fname);
			
			tx_log = fopen(fname, "wb");
		}
		fwrite(data.data(), 1, *written, tx_log);
#ifdef LOG_RX_TX_FLUSH
		fflush(tx_log);
#endif
	}
#endif

	return (m_lastResult = XRV_OK);
}

/*! \brief Cancel any pending io requests */
void SerialInterface::cancelIo() const
{
#ifdef _WIN32
	/* This function is only available on Windows Vista and higher.
	   When a read action hangs, this function can cancel IO operations from another thread.
	*/
	//CancelIoEx(m_handle, NULL);
#endif
}
