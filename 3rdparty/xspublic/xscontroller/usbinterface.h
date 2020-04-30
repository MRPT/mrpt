
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

#ifndef USBINTERFACE_H
#define USBINTERFACE_H

#include <xstypes/xstime.h>

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
#endif

#include "streaminterface.h"
#include <stdio.h>

struct XsPortInfo;

class UsbInterfacePrivate;

class UsbInterface : public StreamInterface {
public:
	UsbInterface();
	~UsbInterface();

	XsResultValue open(const XsPortInfo &portInfo, XsFilePos readBufSize = 0, XsFilePos writeBufSize = 0, PortOptions = PO_XsensDefaults) override;
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

	XsResultValue writeData(const XsByteArray& data, XsFilePos* written = NULL) override;
	XsResultValue readData(XsFilePos maxLength, XsByteArray& data) override;
	using IoInterface::waitForData;

	XsResultValue writeData(XsFilePos length, const void *data, XsFilePos* written = NULL);
	XsResultValue readData(XsFilePos maxLength, void *data, XsFilePos* length = NULL);
	XsResultValue waitForData(XsFilePos maxLength, void *data, XsFilePos* length = NULL);

	void getPortName(XsString& portname) const;

private:
	XSENS_DISABLE_COPY(UsbInterface)
	UsbInterfacePrivate *d;
};

#endif	// file guard
