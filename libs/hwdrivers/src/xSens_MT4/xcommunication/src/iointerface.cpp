/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include <xsens/xsthread.h>
#include "iointerface.h"
#include <errno.h>
#ifndef _WIN32
#	include <unistd.h>		// close
#	include <sys/ioctl.h>	// ioctl
#	include <fcntl.h>		// open, O_RDWR
#	include <string.h>		// strcpy
#	include <sys/param.h>
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

/*! \class IoInterface
	\brief An abstract IO interface
	\details An %IoInterface provides a generic interface for dealing with an interface device such
	as a file, a com port or a USB port.
	\note The class is not thread safe.
*/

// IOInterface functions
/*! \copydoc SerialInterface::open(const XsPortInfo&, uint32_t, uint32_t) */
XsResultValue IoInterface::open ( const XsPortInfo&, uint32_t readBufSize, uint32_t writeBufSize)
{
	(void) readBufSize;
	(void) writeBufSize;
	return XRV_INVALIDOPERATION;
}
/*! \copydoc SerialInterface::setTimeout(uint32_t) */
XsResultValue IoInterface::setTimeout (uint32_t ms)
{
	(void) ms;
	return XRV_INVALIDOPERATION;
}
/*! \copydoc SerialInterface::waitForData(XsSize, XsByteArray&) */
XsResultValue IoInterface::waitForData (XsSize maxLength, XsByteArray& data)
{
	(void) maxLength;
	(void) data;
	return XRV_INVALIDOPERATION;
}
/*! \copydoc SerialInterface::cancelIo(void) const */
void IoInterface::cancelIo(void) const
{
	
}

// IOInterfaceFile functions
/*! \copydoc IoInterfaceFile::appendData() */
XsResultValue IoInterface::appendData(const XsByteArray& bdata)
{
	(void) bdata;
	return XRV_INVALIDOPERATION;
}
/*! \copydoc IoInterfaceFile::closeAndDelete() */
XsResultValue IoInterface::closeAndDelete(void)
{
	return XRV_INVALIDOPERATION;
}
/*! \copydoc IoInterfaceFile::create(const XsString&) */
XsResultValue IoInterface::create(const XsString& filename)
{
	(void) filename;
	return XRV_INVALIDOPERATION;
}
/*! \copydoc IoInterfaceFile::deleteData(XsFilePos, XsSize) */
XsResultValue IoInterface::deleteData(XsFilePos start, XsSize length)
{
	(void) start;
	(void) length;
	return XRV_INVALIDOPERATION;
}
/*! \copydoc IoInterfaceFile::find(const XsByteArray&, XsFilePos&) */
XsResultValue IoInterface::find(const XsByteArray& needleV, XsFilePos& pos)
{
	(void) needleV;
	(void) pos;
	return XRV_INVALIDOPERATION;
}
/*! \copydoc IoInterfaceFile::getFileSize() const */
XsFilePos IoInterface::getFileSize(void) const
{
	return 0;
}
/*! \copydoc IoInterfaceFile::getName(XsString&) const */
XsResultValue IoInterface::getName(XsString& filename) const
{
	(void) filename;
	return XRV_INVALIDOPERATION;
}
/*! \copydoc IoInterfaceFile::getReadPosition() const */
XsFilePos IoInterface::getReadPosition(void) const
{
	return 0;
}
/*! \copydoc IoInterfaceFile::getWritePosition() const */
XsFilePos IoInterface::getWritePosition(void) const
{
	return 0;
}
/*! \copydoc IoInterfaceFile::insertData(XsFilePos, const XsByteArray&) */
XsResultValue IoInterface::insertData(XsFilePos start, const XsByteArray& data)
{
	(void) start;
	(void) data;
	return XRV_INVALIDOPERATION;
}
/*! \copydoc IoInterfaceFile::isReadOnly() const */
bool IoInterface::isReadOnly(void) const
{
	return true;
}
/*! \copydoc IoInterfaceFile::open(const XsString&, bool, bool) */
XsResultValue IoInterface::open(const XsString& filename, bool createNew, bool readOnly)
{
	(void) filename;
	(void) createNew;
	(void) readOnly;
	return XRV_INVALIDOPERATION;
}
/*! \copydoc IoInterfaceFile::setReadPosition(XsFilePos) */
XsResultValue IoInterface::setReadPosition(XsFilePos pos)
{
	(void) pos;
	return XRV_INVALIDOPERATION;
}
/*! \copydoc IoInterfaceFile::setWritePosition(XsFilePos) */
XsResultValue IoInterface::setWritePosition(XsFilePos pos)
{
	(void) pos;
	return XRV_INVALIDOPERATION;
}
