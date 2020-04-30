
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

#include "serialinterface.h"
#include <xstypes/xsportinfo.h>
#include <xstypes/xscontrolline.h>
#include "rx_tx_log.h"

#include <errno.h>
#ifndef _WIN32
#	include <unistd.h>		// close
#	include <sys/ioctl.h>	// ioctl
#	include <fcntl.h>		// open, O_RDWR
#	include <string.h>		// strcpy
#	include <sys/param.h>
#	include <sys/file.h>
#	include <stdarg.h>
#if !defined(__APPLE__) && !defined(ANDROID)
#	include <linux/serial.h>
#endif
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
}

//! Destructor, de-initializes, frees memory allocated for buffers, etc.
SerialInterface::~SerialInterface()
{
	try {
		closeLive();
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
	rx_log.close();
	tx_log.close();
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
			if (state & XCL_DTR)
				status |= TIOCM_DTR;
			else
				status &= ~TIOCM_DTR;
			rv = (ioctl(m_handle, TIOCMSET, &status) == -1);
		}
		else
			rv = false;
	}
	if (rv && (mask & XCL_RTS))
	{
		if (ioctl(m_handle, TIOCMGET, &status) == -1)
		{
			if (state & XCL_RTS)
				status |= TIOCM_RTS;
			else
				status &= ~TIOCM_RTS;
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

#ifndef _WIN32
template <typename T>
static T setBitsEnabled(T field, T bits, bool cond)
{
	if (cond)
		field |= bits;
	else
		field &= (~bits);
	return field;
}
#endif

/*! \brief Open a communication channel to the given port info.
	\details If the baudrate in \a portInfo is set to XBR_Invalid, the baud rate is automatically
				detected if possible.
	\param portInfo The details of the port that should be opened. Depending on the type of interface,
				parts of this parameter may be ignored.
	\param readBufSize The size of the read buffer in bytes (if appliccable to the device)
	\param writeBufSize The size of the write buffer in bytes (if appliccable to the device)
	\param options The options to enable (flow control, stop bits)
	\returns XRV_OK if the device was opened successfully
*/
XsResultValue SerialInterface::open(const XsPortInfo& portInfo,
						XsFilePos readBufSize,
						XsFilePos writeBufSize,
						PortOptions options)
{
	m_endTime = 0;

	JLDEBUGG(portInfo);

	if (isOpen())
	{
		JLALERTG("Port " << portInfo.portName() << " is already open");
		return (m_lastResult = XRV_ALREADYOPEN);
	}
	m_baudrate = portInfo.baudrate();

	if (options & PO_RtsCtsFlowControl || (portInfo.linesOptions() & XPLO_RtsCtsFlowControl))
		JLTRACEG("Requested RTS/CTS flow control");
	if (options & PO_DtrDsrFlowControl)
		JLTRACEG("Requested DTR/DSR flow control");
	if (options & PO_XonXoffFlowControl)
		JLTRACEG("Requested Xon/Xoff flow control");

#ifdef _WIN32
	XsResultValue fail = XRV_OK;
	char winPortName[256];

	// Open port
	sprintf(winPortName, "\\\\.\\%s", portInfo.portName().c_str());
	m_handle = CreateFileA(winPortName, GENERIC_READ | GENERIC_WRITE, 0, NULL,
									OPEN_EXISTING, 0, NULL);
	if (m_handle == INVALID_HANDLE_VALUE)
	{
		JLDEBUGG("Port " << portInfo.portName() << " cannot be opened");
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
	commState.StopBits = (options&PO_TwoStopBits)?TWOSTOPBITS:ONESTOPBIT;

	// Setup flow control
	commState.fDsrSensitivity = (options&PO_DtrDsrFlowControl)?TRUE:FALSE;
	commState.fOutxDsrFlow = (options&PO_DtrDsrFlowControl)?DTR_CONTROL_HANDSHAKE:DTR_CONTROL_DISABLE;

	commState.fOutxCtsFlow = (options & PO_RtsCtsFlowControl || (portInfo.linesOptions() & XPLO_RtsCtsFlowControl)) ? TRUE : FALSE;
	commState.fRtsControl  = (options & PO_RtsCtsFlowControl || (portInfo.linesOptions() & XPLO_RtsCtsFlowControl)) ? RTS_CONTROL_HANDSHAKE : RTS_CONTROL_ENABLE;

	commState.fOutX = (options&PO_XonXoffFlowControl)?TRUE:FALSE;
	commState.fInX = commState.fOutX;

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
	int tmpFail = fail;
	applyHwControlLinesOptions(options, portInfo.linesOptions(), tmpFail);
	fail = static_cast<XsResultValue>(tmpFail);

	if (!SetupComm(m_handle, (DWORD) readBufSize, (DWORD) writeBufSize))	// Set queue size
		fail = XRV_ERROR;

	// Remove any 'old' data in buffer
	//PurgeComm(m_handle, PURGE_TXCLEAR | PURGE_RXCLEAR);
	if (!PurgeComm(m_handle, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR))
		fail = XRV_ERROR;

	if (fail != XRV_OK)
	{
		CloseHandle(m_handle);
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
	m_commState.c_cflag &= ~(CSIZE|PARENB|PARODD);
	m_commState.c_cflag |= CS8;		// Select 8 data bits

	m_commState.c_cflag = setBitsEnabled(m_commState.c_cflag, (tcflag_t)CSTOPB, (options&PO_TwoStopBits) == PO_TwoStopBits);

	// Hardware flow control
	m_commState.c_cflag = setBitsEnabled(m_commState.c_cflag, (tcflag_t)CRTSCTS, (options&PO_RtsCtsFlowControl) == PO_RtsCtsFlowControl);
#ifdef CDTRDSR
	m_commState.c_cflag = setBitsEnabled(m_commState.c_cflag, (tcflag_t)CDTRDSR, (options&PO_DtrDsrFlowControl) == PO_DtrDsrFlowControl);
#endif

	m_commState.c_lflag &= ~(ECHO|ECHOE|ECHOK|ECHONL|ICANON|ISIG|IEXTEN);
	// Software flow control
	m_commState.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|INPCK|ISTRIP|INLCR|IGNCR|ICRNL);
	m_commState.c_iflag = setBitsEnabled(m_commState.c_iflag, (tcflag_t)(IXON|IXOFF), options&PO_XonXoffFlowControl);
	// Set Raw output
	m_commState.c_oflag &= ~OPOST;
	// Timeout 0.001 sec for first byte, read minimum of 0 bytes
	m_commState.c_cc[VMIN]     = 0;
	m_commState.c_cc[VTIME]    = (m_timeout+99)/100;	// 1

	// Set the new options for the port
	if (tcsetattr(m_handle,TCSANOW, &m_commState) != 0)
		return XRV_INPUTCANNOTBEOPENED;

#if defined(JLDEF_BUILD) && JLDEF_BUILD <= JLL_ALERT
	termios checkCommState;
	if (tcgetattr(m_handle, &checkCommState) != 0)
		return XRV_ERROR;

	if (cfgetispeed(&checkCommState) != portInfo.baudrate())
		JLALERTG("Set baudrate doesn't match requested baudrate");

	if (cfgetospeed(&checkCommState) != portInfo.baudrate())
		JLALERTG("Set baudrate doesn't match requested baudrate");

	if (options&PO_RtsCtsFlowControl && !(checkCommState.c_cflag&CRTSCTS))
		JLALERTG("Requested RTS/CTS flow control, but could not be set.");

	if (options&PO_DtrDsrFlowControl &&
#ifdef CDTRDSR
		!(checkCommState.c_cflag&CDTRDSR)
#else
		false
#endif
		)
		JLALERTG("Requested DTR/DSR flow control, but could not be set.");

	if (options&PO_XonXoffFlowControl && !((checkCommState.c_iflag&(IXON|IXOFF)) == (IXON|IXOFF)))
		JLALERTG("Requested Xon/Xoff flow control, but could not be set.");
#endif // JLDEF_BUILD < JLL_ALERT

#if defined(JLDEF_BUILD) && JLDEF_BUILD <= JLL_DEBUG
#define CHECK_COMMSTATE(req, res, field)\
	if (req.field != res.field) \
	{\
		JLDEBUGG("field " << #field << " does not match");\
		JLDEBUGG("actual  : " << std::oct << (uint64_t)res.field);\
		JLDEBUGG("expected: " << std::oct << (uint64_t)req.field);\
	}
#else
#define CHECK_COMMSTATE(req, res, field)
#endif
	CHECK_COMMSTATE(m_commState, checkCommState, c_cflag);
	CHECK_COMMSTATE(m_commState, checkCommState, c_iflag);
	CHECK_COMMSTATE(m_commState, checkCommState, c_oflag);
	CHECK_COMMSTATE(m_commState, checkCommState, c_cc[VMIN]);
	CHECK_COMMSTATE(m_commState, checkCommState, c_cc[VTIME]);

	m_port = 1;
	sprintf(m_portname, "%s", pn.c_str());

	tcflush(m_handle, TCIOFLUSH);

	// setting RTS and DTR;
	int cmbits;
	if (ioctl(m_handle, TIOCMGET, &cmbits) < 0)
	{
		JLDEBUGG("TIOCMGET failed, which is OK for USB connected MkIV devices");
	}

	// Port Lines Options
	applyHwControlLinesOptions(options, portInfo.linesOptions(), cmbits);

	if (ioctl(m_handle, TIOCMSET, &cmbits) < 0)
	{
		JLDEBUGG("TIOCMSET failed, which is OK for USB connected MkIV devices");
	}
#endif // !_WIN32

	JLDEBUGG("Port " << portInfo.portName().toStdString() << " opened");
	return (m_lastResult = XRV_OK);
}

/*! \brief Read data from the serial port and put it into the data buffer.
	\details This function reads up to \a maxLength bytes from the port (non-blocking) and
	puts it into the \a data buffer.
	\param maxLength The maximum amount of data read.
	\param data The buffer that will store the received data.
	\returns XRV_OK if no error occurred. It can be that no data is available and XRV_OK will be
			returned. Check data.size() for the number of bytes that were read.
*/
XsResultValue SerialInterface::readData(XsFilePos maxLength, XsByteArray& data)
{
	if (!isOpen())
		return (m_lastResult = XRV_NOPORTOPEN);

#ifdef _WIN32
	DWORD length;
	data.setSize((XsSize) maxLength);
	BOOL rres = ::ReadFile(m_handle, data.data(), (DWORD) maxLength, &length, NULL);
	data.pop_back((XsSize) (maxLength-length));
	JLTRACEG("ReadFile result " << rres << ", length " << length);

	if (!rres)
	{
		DWORD wErr = ::GetLastError();
		JLALERTG("ReadFile returned windows error " << wErr);
		if (wErr == ERROR_ACCESS_DENIED)
			return (m_lastResult = XRV_UNEXPECTED_DISCONNECT);
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
	if (length > 0)
	{
		data.pop_back(maxLength - length);
	}
	else
	{
		int err = errno;

		data.clear();
		switch (err)
		{
		case EAGAIN:
#if defined(EWOULDBLOCK) && EWOULDBLOCK != EAGAIN
		case EWOULDBLOCK:
#endif
			return XRV_TIMEOUT;

		case EIO:
			return XRV_UNEXPECTED_DISCONNECT;

		default:
			break;
		}
	}
#if defined(JLDEF_BUILD) && JLDEF_BUILD <= JLL_TRACE && !defined(ANDROID)
	serial_icounter_struct ic;
	res = ioctl(m_handle, TIOCGICOUNT, &ic);
	if (res == 0)
	{
		JLTRACEG("rx: " << ic.rx);
		JLTRACEG("tx: " << ic.tx);
		JLTRACEG("frame " << ic.frame);
		JLTRACEG("overrun " << ic.overrun);
		JLTRACEG("buf_overrun " << ic.buf_overrun);
	}
#endif
#endif

#ifdef LOG_RX_TX
	if (length > 0)
	{
		CHECK_STATE_RX(length, data, rx_log);

		if (!rx_log.isOpen())
		{
			char fname[XS_MAX_FILENAME_LENGTH];
#ifdef _WIN32
			sprintf(fname, "rx_%03d_%d.log", (int32_t) m_port, m_baudrate);
#else
			char *devname = strrchr(m_portname, '/');
			sprintf(fname, "rx_%s_%d.log", devname + 1, XsBaud::rateToNumeric(m_baudrate));
#endif
			makeFilenameUnique(fname, state);

			rx_log.create(XsString(fname), true);
		}
		rx_log.write(data.data(), 1, length);
#ifdef LOG_RX_TX_FLUSH
		rx_log.flush();
#endif
	}
#endif

	JLTRACEG("returned success, read " << length << " of " << maxLength << " bytes, first: " << JLHEXLOG(data[0]));
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
	JLDEBUGG("Setting timeout to " << ms << " ms");

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
XsResultValue SerialInterface::waitForData(XsFilePos maxLength, XsByteArray& data)
{
	data.clear();
	data.reserve((XsSize) maxLength);

	//char *data = (char *)&_data[0];
	JLTRACEG("timeout=" << m_timeout << ", maxLength=" << maxLength);
	uint32_t timeout = m_timeout;

	uint32_t eTime = XsTime_getTimeOfDay(NULL, NULL) + timeout;
//	uint32_t newLength = 0;

	while (((XsFilePos) data.size() < maxLength) && (XsTime_getTimeOfDay(NULL, NULL) <= eTime))
	{
		XsByteArray raw;

		if (readData(maxLength - data.size(), raw) != XRV_OK)
			return m_lastResult;
		data.append(raw);
	}
	JLTRACEG("Read " << data.size() << " of " << maxLength << " bytes");

	if ((XsFilePos) data.size() < maxLength)
		return (m_lastResult = XRV_TIMEOUT);
	else
		return (m_lastResult = XRV_OK);
}

/*! \copydoc IoInterface::writeData
	\note The default timeout is respected in this operation.
*/
XsResultValue SerialInterface::writeData(const XsByteArray& data, XsFilePos* written)
{
	XsFilePos bytes;
	if (written == NULL)
		written = &bytes;

	if (!isOpen())
		return (m_lastResult = XRV_NOPORTOPEN);

	*written = 0;

#ifdef _WIN32
	DWORD lwritten = 0;
	if (WriteFile(m_handle, data.data(), (DWORD) data.size(), &lwritten, NULL) == 0)
	{
		DWORD wErr = ::GetLastError();
		JLALERTG("WriteFile returned windows error " << wErr);
		if (wErr == ERROR_ACCESS_DENIED)
			return (m_lastResult = XRV_UNEXPECTED_DISCONNECT);
		return (m_lastResult = XRV_ERROR);
	}

	*written = lwritten;
#else
	ssize_t result = write(m_handle, (const void*)data.data(), data.size());
	if (result <= 0)
	{
		int err = errno;
		*written = 0;
		switch (err)
		{
		case EAGAIN:
#if defined(EWOULDBLOCK) && EWOULDBLOCK != EAGAIN
		case EWOULDBLOCK:
#endif
			return XRV_TIMEOUT;

		case EIO:
			return XRV_UNEXPECTED_DISCONNECT;

		/* we don't expect any other errors to actually occur */
		default:
			break;
		}
	}

	if (result < 0)
		*written = 0;
	else
		*written = result;
#endif

#ifdef LOG_RX_TX
	if (written[0] > 0)
	{
		CHECK_STATE_TX(written[0], data, tx_log);
		if (!tx_log.isOpen())
		{
			char fname[XS_MAX_FILENAME_LENGTH];
#ifdef _WIN32
			sprintf(fname, "tx_%03d_%d.log", (int32_t) m_port, m_baudrate);
#else
			char *devname = strrchr(m_portname, '/');
			sprintf(fname,"tx_%s_%d.log", devname + 1, XsBaud::rateToNumeric(m_baudrate));
#endif
			makeFilenameUnique(fname, state);

			tx_log.create(XsString(fname), true);
		}
		tx_log.write(data.data(), 1, *written);
#ifdef LOG_RX_TX_FLUSH
		tx_log.flush();
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

/*! \brief Logical \a or operator for flow controls */
SerialInterface::PortOptions operator|(SerialInterface::PortOptions lhs, SerialInterface::PortOptions rhs)
{
	return static_cast<SerialInterface::PortOptions>(static_cast<int>(lhs) | static_cast<int>(rhs));
}

/*! \brief Logical \a and operator for flow controls */
SerialInterface::PortOptions operator&(SerialInterface::PortOptions lhs, SerialInterface::PortOptions rhs)
{
	return static_cast<SerialInterface::PortOptions>(static_cast<int>(lhs) & static_cast<int>(rhs));
}

/*! \brief Logical inversion operator for flow controls */
SerialInterface::PortOptions operator~(SerialInterface::PortOptions lhs)
{
	return static_cast<SerialInterface::PortOptions>(~static_cast<int>(lhs));
}

/*! \brief Apply the specified options for the hardware control lines.
	\param options The options to enable (flow control, stop bits)
	\param portLinesOptions The options for the hardware control lines (RTS/DTR levels)
	\param p The reference to an enabled bits
*/
void SerialInterface::applyHwControlLinesOptions(PortOptions options, int portLinesOptions, int& p)
{
	const XsPortLinesOptions pLinesOpts = static_cast<XsPortLinesOptions>(portLinesOptions);

#ifdef _WIN32

	// DTR Line Options
	if (!(options&PO_DtrDsrFlowControl))
	{
		// Flow Control is disabled
		if ((pLinesOpts == XPLO_Invalid) || (pLinesOpts & XPLO_DTR_Ignore))
		{
			// Default behaviour
			if (!EscapeCommFunction(m_handle, SETDTR))	// Set DTR (Calibration sensors need DTR to startup, won't hurt otherwise)
				p = XRV_ERROR;
		}
		else
		{
			// Handle flow control line options
			if (((pLinesOpts & (XPLO_DTR_Set | XPLO_DTR_Clear)) == (XPLO_DTR_Set | XPLO_DTR_Clear))
				|| ((pLinesOpts & (XPLO_DTR_Set | XPLO_DTR_Clear)) == 0))
			{
				// Line options are not valid, both Set and Clear are high or down
				if (!EscapeCommFunction(m_handle, SETDTR))	// Default behaviour
					p = XRV_ERROR;
			}
			else if (pLinesOpts & XPLO_DTR_Set)
			{
				if (!EscapeCommFunction(m_handle, SETDTR))	// Set DTR
					p = XRV_ERROR;
			}
			else
			{
				// XPLO_DTR_Clear is Set
				assert(pLinesOpts & XPLO_DTR_Clear);

				if (!EscapeCommFunction(m_handle, CLRDTR))	// Clear DTR
					p = XRV_ERROR;
			}
		}
	}

	// RTS Line Options
	if (!(options&PO_RtsCtsFlowControl))
	{
		// Flow Control is disabled
		if ((pLinesOpts != XPLO_Invalid) && !(pLinesOpts & XPLO_RTS_Ignore))
		{
			if ((((pLinesOpts & XPLO_RTS_Set) && (pLinesOpts & XPLO_RTS_Clear)) == 0)
				&& (((pLinesOpts & XPLO_RTS_Set) || (pLinesOpts & XPLO_RTS_Clear)) != 0))
			{
				// Only one between Set and Clear is high
				if (pLinesOpts & XPLO_RTS_Set)
				{
					if (!EscapeCommFunction(m_handle, SETRTS))	// Set RTS
						p = XRV_ERROR;
				}
				else
				{
					// XPLO_RTS_Clear is Set
					assert(pLinesOpts & XPLO_RTS_Clear);

					if (!EscapeCommFunction(m_handle, CLRRTS))	// Clear RTS
						p = XRV_ERROR;
				}
			}
		}
	}

#else

	// DTR Line
	if (!(options&PO_DtrDsrFlowControl))
	{
		// Flow control is disabled
		if ((pLinesOpts != XPLO_Invalid) && !(pLinesOpts & XPLO_DTR_Ignore))
		{
			if (((pLinesOpts & (XPLO_DTR_Set | XPLO_DTR_Clear)) == (XPLO_DTR_Set | XPLO_DTR_Clear))
				|| ((pLinesOpts & (XPLO_DTR_Set | XPLO_DTR_Clear)) == 0))
			{
				// Line options are not valid, both Set and Clear are high or down
				p = setBitsEnabled(p, TIOCM_DTR, true); // Default behaviour
			}
			else
				p = setBitsEnabled(p, TIOCM_DTR, (pLinesOpts & XPLO_DTR_Set));
		}
		else
			p = setBitsEnabled(p, TIOCM_DTR, true); // Default behaviour
	}

	// RTS Line
	if (!(options&PO_RtsCtsFlowControl))
	{
		if ((pLinesOpts != XPLO_Invalid) && !(pLinesOpts & XPLO_RTS_Ignore))
		{
			if (((pLinesOpts & (XPLO_RTS_Set | XPLO_RTS_Clear)) == (XPLO_RTS_Set | XPLO_RTS_Clear))
				|| ((pLinesOpts & (XPLO_RTS_Set | XPLO_RTS_Clear)) == 0))
			{
				// Line options are not valid, both Set and Clear are high or down
				p = setBitsEnabled(p, TIOCM_RTS, true); // Default behaviour
			}
			else
				p = setBitsEnabled(p, TIOCM_RTS, (pLinesOpts & XPLO_RTS_Set));
		}
		else
			p = setBitsEnabled(p, TIOCM_RTS, true); // Default behaviour
	}

#endif

}
