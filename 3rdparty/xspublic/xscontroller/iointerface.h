
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

#ifndef IOINTERFACE_H
#define IOINTERFACE_H

#include "xscontrollerconfig.h"
#include <xstypes/xsplatform.h>
#include <xstypes/xsmessage.h>
#include <xstypes/xstime.h>
#include <xstypes/xsfilepos.h>
#include <xstypes/xsbytearray.h>
#include <xstypes/xsresultvalue.h>
#include <xstypes/xsfile.h>

#include <stdlib.h>

//! The default size of the serial read buffer in bytes
#define XS_DEFAULT_READ_BUFFER_SIZE		(XS_MAXMSGLEN*8)
//! The default size of the serial write buffer in bytes
#define XS_DEFAULT_WRITE_BUFFER_SIZE		XS_MAXMSGLEN

#ifdef _MSC_VER
#	define XS_MAX_FILENAME_LENGTH	512
#else
#	define XS_MAX_FILENAME_LENGTH	PATH_MAX
#endif

#ifndef _WIN32
typedef int32_t XsIoHandle;
#else
#define XsIoHandle HANDLE
#endif

struct XsPortInfo;

struct IoInterface {
public:
	/*! \brief Destructor */
	virtual ~IoInterface() {}

	/*! \brief Close the connection to the device.
		\returns XRV_OK if the connection was closed successfully
	*/
	virtual XsResultValue close(void) = 0;

	/*!	\brief Flush all data in the buffers to and from the device
		\returns XRV_OK if the data was flushed successfully
	*/
	virtual XsResultValue flushData(void) = 0;

	/*!	\brief Returns true if the object has a connection to a device
		\returns true if the object has a connection to a device
	*/
	virtual bool isOpen (void) const = 0;

	/*! \brief Returns the last result value produced by this interface
		\returns The last result value produced by this interface.
	*/
	virtual XsResultValue getLastResult(void) const = 0;

	/*! \brief Write the data contained in \a data to the device
		\param data The data to write to the device.
		\param written An optional %XsFilePos value that will receive the number of bytes that were
		actually written.
		\returns XRV_OK if the data was written successfully
	*/
	virtual XsResultValue writeData(const XsByteArray& data, XsFilePos* written = NULL) = 0;

	/*! \brief Read at most \a maxLength bytes from the device into \a data
		\param maxLength The maximum number of bytes to read. Depending on the device type and timeout
		settings, the function may return with less than this number of bytes read.
		\param data A buffer that will contain the read data.
		\returns XRV_OK if all data was read successfully, XRV_TIMEOUT if some data was read, but not
		\a maxLength, XRV_TIMEOUTNODATA if no data was read at all.
	*/
	virtual XsResultValue readData(XsFilePos maxLength, XsByteArray& data) = 0;

	//! Options for flow control and stopbits which must be used when opening a port
	enum PortOptions {
		PO_NoFlowControl	= 0,
		PO_RtsCtsFlowControl	= (1 << 0),
		PO_DtrDsrFlowControl	= (1 << 1),
		PO_XonXoffFlowControl	= (1 << 2),
		PO_OneStopBit		= 0,
		PO_TwoStopBits		= (1 << 3),
		PO_XsensDefaults	= (PO_NoFlowControl|PO_TwoStopBits)
	};
	// SerialInterface overridable functions
	virtual XsResultValue open(const XsPortInfo& portInfo, XsFilePos readBufSize = XS_DEFAULT_READ_BUFFER_SIZE, XsFilePos writeBufSize = XS_DEFAULT_WRITE_BUFFER_SIZE, PortOptions options = PO_XsensDefaults);
	virtual XsResultValue setTimeout (uint32_t ms);
	virtual XsResultValue waitForData (XsFilePos maxLength, XsByteArray& data);
	virtual void cancelIo(void) const;

	// IOInterfaceFile overridable functions
	virtual XsResultValue appendData(const XsByteArray& bdata);
	virtual XsResultValue closeAndDelete(void);
	virtual XsResultValue create(const XsString& filename);
	virtual XsResultValue deleteData(XsFilePos start, XsFilePos length);
	virtual XsResultValue find(const XsByteArray& needleV, XsFilePos& pos);
	virtual XsFilePos getFileSize(void) const;
	virtual XsResultValue getName(XsString& filename) const;
	virtual XsFilePos getReadPosition(void) const;
	virtual XsFilePos getWritePosition(void) const;
	virtual XsResultValue insertData(XsFilePos start, const XsByteArray& data);
	virtual bool isReadOnly(void) const;
	virtual XsResultValue open(const XsString& filename, bool createNew, bool readOnly);
	virtual XsResultValue setReadPosition(XsFilePos pos);
	virtual XsResultValue setWritePosition(XsFilePos pos = -1);

	XSENS_DISABLE_COPY(IoInterface)
protected:
	/*! \brief Constructor */
	IoInterface()
	{}
};
#endif	// file guard
