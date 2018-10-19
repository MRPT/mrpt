/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#ifndef SERIALINTERFACE_H
#define SERIALINTERFACE_H

#include "streaminterface.h"
#include <xsens/xsplatform.h>
#include <xsens/xsmessage.h>
#include <xsens/xsbaud.h>
#include <xsens/xscontrolline.h>

struct XsPortInfo;

//////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////// SerialInterface
////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

/*! \brief The low-level serial communication class.
 */
class SerialInterface : public StreamInterface
{
   private:
	XSENS_DISABLE_COPY(SerialInterface)

	XsFileHandle* rx_log;
	XsFileHandle* tx_log;

   protected:
	//! The baudrate that was last set to be used by the port
	XsBaudRate m_baudrate;
	//! The time at which an operation will end in ms, used by several
	//! functions.
	uint32_t m_endTime;
	//! The last result of an operation
	mutable XsResultValue m_lastResult;
	//! The opened COM port nr
	uint16_t m_port;
	//! The name of the open serial port
	char m_portname[32];
	/*! The default timeout value to use during blocking operations.
		A value of 0 means that all operations become non-blocking.
	*/
	uint32_t m_timeout;

#ifdef _WIN32
	/** The serial port handle, also indicates if the port is open or not. */
	XsIoHandle m_handle;
#else
	/** Stored settings about the serial port */
	termios m_commState;
	/** The serial port handle, also indicates if the port is open or not. */
	int32_t m_handle;
#endif
   public:
	SerialInterface();
	~SerialInterface() override;

	// Function overrides
	XsResultValue close(void) override;
	XsResultValue closeLive(void);
	XsResultValue flushData(void) override;
	bool isOpen(void) const override;
	XsResultValue getLastResult(void) const override;
	XsResultValue writeData(
		const XsByteArray& data, XsSize* written = 0) override;
	XsResultValue readData(XsSize maxLength, XsByteArray& data) override;

	void cancelIo(void) const override;

	// Other functions
	XsResultValue escape(XsControlLine mask, XsControlLine state);
	XsBaudRate getBaudrate(void) const;
	XsIoHandle getHandle(void) const;
	uint16_t getPortNumber(void) const;
	void getPortName(XsString& portname) const;
	uint32_t getTimeout(void) const override;
	XsResultValue open(
		const XsPortInfo& portInfo,
		uint32_t readBufSize = XS_DEFAULT_READ_BUFFER_SIZE,
		uint32_t writeBufSize = XS_DEFAULT_WRITE_BUFFER_SIZE) override;
	XsResultValue setTimeout(uint32_t ms) override;
	XsResultValue waitForData(XsSize maxLength, XsByteArray& data) override;
};

#endif  // file guard
