/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#ifndef USBINTERFACE_H
#define USBINTERFACE_H

#include <xsens/xstime.h>

#include <stdlib.h>
#include <stdio.h>
#ifdef _WIN32
#include <windows.h>
//#	include <sys/types.h>
#else
#include <termios.h>
// these are not required by level 1, but to keep the higher levels
// platform-independent they are put here
#include <string.h>
#include <stddef.h>
#define _strnicmp strncasecmp
#endif

#include "streaminterface.h"
#include <stdio.h>

struct XsPortInfo;

class UsbInterfacePrivate;

class UsbInterface : public StreamInterface
{
   public:
	UsbInterface();
	~UsbInterface() override;

	XsResultValue open(
		const XsPortInfo& portInfo, uint32_t readBufSize = 0,
		uint32_t writeBufSize = 0) override;
	XsResultValue close(void) override;
	XsResultValue closeUsb(void);
	XsResultValue flushData(void) override;

	bool isOpen(void) const override;
	uint8_t usbBus() const;
	uint8_t usbAddress() const;

	XsResultValue getLastResult(void) const override;

	XsResultValue setTimeout(uint32_t ms) override;
	uint32_t getTimeout(void) const override;

	void setRawIo(bool enable);
	bool getRawIo(void);

	XsResultValue writeData(
		const XsByteArray& data, XsSize* written = nullptr) override;
	XsResultValue readData(XsSize maxLength, XsByteArray& data) override;
	using IoInterface::waitForData;

	// lint -e1411 inherited definitions are also available (see above)
	XsResultValue writeData(
		XsSize length, const void* data, XsSize* written = nullptr);
	XsResultValue readData(
		XsSize maxLength, void* data, XsSize* length = nullptr);
	XsResultValue waitForData(
		XsSize maxLength, void* data, XsSize* length = nullptr);
	// lint +e1411

	void getPortName(XsString& portname) const;

   private:
	XSENS_DISABLE_COPY(UsbInterface)
	UsbInterfacePrivate* d;
};

#endif  // file guard
