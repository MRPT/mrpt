/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef IOINTERFACE_H
#define IOINTERFACE_H

#include "xcommunicationconfig.h"
#include <xsens/xsplatform.h>
#include <xsens/xsmessage.h>
#include <xsens/xstime.h>
#include <xsens/xsfilepos.h>
#include <xsens/xsbytearray.h>
#include <xsens/xsresultvalue.h>

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
#define XsFileHandle FILE

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
		\param written An optional %XsSize value that will receive the number of bytes that were
		actually written.
		\returns XRV_OK if the data was written successfully
	*/
	virtual XsResultValue writeData(const XsByteArray& data, XsSize* written = NULL) = 0;

	/*! \brief Read at most \a maxLength bytes from the device into \a data
		\param maxLength The maximum number of bytes to read. Depending on the device type and timeout
		settings, the function may return with less than this number of bytes read.
		\param data A buffer that will contain the read data.
		\returns XRV_OK if all data was read successfully, XRV_TIMEOUT if some data was read, but not
		\a maxLength, XRV_TIMEOUTNODATA if no data was read at all.
	*/
	virtual XsResultValue readData(XsSize maxLength, XsByteArray& data) = 0;

	// SerialInterface overridable functions
	virtual XsResultValue open(const XsPortInfo& portInfo, uint32_t readBufSize = XS_DEFAULT_READ_BUFFER_SIZE, uint32_t writeBufSize = XS_DEFAULT_WRITE_BUFFER_SIZE);
	virtual XsResultValue setTimeout (uint32_t ms);
	virtual XsResultValue waitForData (XsSize maxLength, XsByteArray& data);
	virtual void cancelIo(void) const;

	// IOInterfaceFile overridable functions
	virtual XsResultValue appendData(const XsByteArray& bdata);
	virtual XsResultValue closeAndDelete(void);
	virtual XsResultValue create(const XsString& filename);
	virtual XsResultValue deleteData(XsFilePos start, XsSize length);
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
