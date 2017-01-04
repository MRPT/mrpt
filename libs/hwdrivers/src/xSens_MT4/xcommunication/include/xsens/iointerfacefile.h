/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef IOINTERFACEFILE_H
#define IOINTERFACEFILE_H

#include <xsens/xsplatform.h>
#include "streaminterface.h"

enum XsResultValue;

//////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////// IoInterfaceFile  /////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

/*! \brief The low-level file communication class.
*/
class IoInterfaceFile : public IoInterface {
private:
	XSENS_DISABLE_COPY(IoInterfaceFile)

protected:
		//! The file handlem, also indicates if the file is open or not.
	XsFileHandle* m_handle;
		//! Contains the size of the file
	XsFilePos m_fileSize;
		//! The last read position in the file
	XsFilePos m_readPos;
		//! The last write position in the file
	XsFilePos m_writePos;
		//! The last result of an operation
	mutable XsResultValue m_lastResult;
		//! Contains the name of the file that was last successfully opened.
	XsString m_filename;
	/*! \brief Indicates whether the last operation was a read or write operation.

		This value is used to check whether or not a seek is required to perform a
		requested read or write operation.
	*/
	bool m_reading;
		//! Indicates if the file was opened in read-only mode
	bool m_readOnly;

	void gotoRead(void);
	void gotoWrite(void);
public:
	IoInterfaceFile();
	~IoInterfaceFile();

	// Function overrides
	XsResultValue close(void);
	XsResultValue closeFile(void);
	XsResultValue flushData(void);
	bool isOpen(void) const;
	XsResultValue getLastResult(void) const;
	XsResultValue writeData(const XsByteArray& data, XsSize *written = NULL);
	XsResultValue readData(XsSize maxLength, XsByteArray& data);
	XsResultValue readTerminatedData(XsSize maxLength, unsigned char terminator, XsByteArray& bdata);

	// Other functions
	XsResultValue appendData(const XsByteArray& bdata);
	XsResultValue closeAndDelete(void);
	XsResultValue create(const XsString& filename);
	XsResultValue deleteData(XsFilePos start, XsSize length);
	XsResultValue find(const XsByteArray& data, XsFilePos& pos);
	XsFilePos getFileSize(void) const;
	XsTimeStamp getFileDate(void) const;
	XsResultValue getName(XsString& filename) const;
	XsFilePos getReadPosition(void) const;
	XsFilePos getWritePosition(void) const;
	XsResultValue insertData(XsFilePos start, const XsByteArray& data);
	bool isReadOnly(void) const;
	XsResultValue open(const XsString& filename, bool createNew, bool readOnly);
	XsResultValue setReadPosition(XsFilePos pos);
	XsResultValue setWritePosition(XsFilePos pos = -1);
};

#endif	// file guard
