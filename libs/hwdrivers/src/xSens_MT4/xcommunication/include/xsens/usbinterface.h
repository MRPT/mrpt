/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef USBINTERFACE_H
#define USBINTERFACE_H

#include <xsens/xstime.h>

#include <stdlib.h>
#include <stdio.h>
#ifdef _WIN32
#	include <windows.h>
//#	include <sys/types.h>
#else
#	include <termios.h>
// these are not required by level 1, but to keep the higher levels platform-independent they are put here
#	include <string.h>
#	include <stddef.h>
#define _strnicmp	strncasecmp
#endif

#include "streaminterface.h"
#include <stdio.h>

struct XsPortInfo;

class UsbInterfacePrivate;

class UsbInterface : public StreamInterface {
public:
	UsbInterface();
	~UsbInterface();

	XsResultValue open(const XsPortInfo &portInfo, uint32_t readBufSize = 0, uint32_t writeBufSize = 0);
	XsResultValue close(void);
	XsResultValue closeUsb(void);
	XsResultValue flushData (void);

	bool isOpen (void) const;
	uint8_t usbBus() const;
	uint8_t usbAddress() const;

	XsResultValue getLastResult(void) const;

	XsResultValue setTimeout (uint32_t ms);
	uint32_t getTimeout (void) const;

	void setRawIo(bool enable);
	bool getRawIo(void);

	virtual XsResultValue writeData(const XsByteArray& data, XsSize* written = NULL);
	virtual XsResultValue readData(XsSize maxLength, XsByteArray& data);
	using IoInterface::waitForData;

	//lint -e1411 inherited definitions are also available (see above)
	XsResultValue writeData(XsSize length, const void *data, XsSize* written = NULL);
	XsResultValue readData(XsSize maxLength, void *data, XsSize* length = NULL);
	XsResultValue waitForData(XsSize maxLength, void *data, XsSize* length = NULL);
	//lint +e1411

	void getPortName(XsString& portname) const;

private:
	XSENS_DISABLE_COPY(UsbInterface)
	UsbInterfacePrivate *d;
};

#endif	// file guard
