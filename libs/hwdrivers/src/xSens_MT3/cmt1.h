/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef _CMT1_H_2006_04_12
#define _CMT1_H_2006_04_12

#ifndef _CMTDEF_H_2006_05_01
#	include "cmtdef.h"
#endif

#ifdef _WIN32
#	include <windows.h>
//#	include <sys/types.h>
#else
#	include <termios.h>
// these are not required by level 1, but to keep the higher levels platform-independent they are put here
#	include <stdlib.h>
#	include <string.h>
#	include <stddef.h>
#define _strnicmp	strncasecmp
#endif

#include <stdio.h>

//! The namespace of all Xsens software since 2006.
namespace xsens {

#ifndef CMTLOG
#	if defined(_DEBUG) || defined(_LOG_ALWAYS)
		void CMTLOG(const char *str, ...);
#		define CMTEXITLOG(str)	JanitorFunc2<const char*,XsensResultValue> _cmtExitLog(CMTLOG,str " returns %u",m_lastResult);
#	else
#		define CMTLOG(...)
#		define CMTEXITLOG(...)
#	endif
#endif

#ifndef _WIN32
int _wcsnicmp(const wchar_t* s1, const wchar_t* s2,int count);
#endif

//////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////// Cmt1s  /////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

/*! \brief The low-level serial communication class.
*/
class Cmt1s {
private:
	//! This object cannot be copied, so this function is not implemented.
	Cmt1s(const Cmt1s& ref);

	#ifdef _LOG_RX_TX
		FILE* rx_log;
		FILE* tx_log;
	#endif

	/*! \brief The bytes received function.
	
		This function is automatically called every time binary data is read from the 
		connected COM port.
	*/
	CmtCallbackFunction m_onBytesReceived;
	//! Custom, user supplied parameter for the OnBytesReceived callback function, passed as the first argument
	int32_t m_onBytesReceivedInstance;
	//! Custom, user supplied parameter for the OnBytesReceived callback function, passed as the last argument
	void* m_onBytesReceivedParam;
protected:
		//! The baudrate that was last set to be used by the port
	uint32_t m_baudrate;
		//! The time at which an operation will end in ms, used by several functions.
	uint32_t m_endTime;
		//! Indicates if the port is open or not
	bool m_isOpen;
		//! The last result of an operation
	mutable XsensResultValue m_lastResult;
		//! The opened COM port nr
	uint8_t m_port;
	char m_portname[32];
	/*! The default timeout value to use during blocking operations.
		A value of 0 means that all operations become non-blocking.
	*/
	uint32_t m_timeout;

	#ifdef _WIN32
		DCB		m_commState;		//!< Stored settings about the serial port
		HANDLE	m_handle;			//!< The serial port handle
	#else
		termios	m_commState;		//!< Stored settings about the serial port
		int32_t		m_handle;			//!< The serial port handle
		typedef int32_t HANDLE;
	#endif
public:
		//! Default constructor, initializes all members to their default values.
	Cmt1s();
		//! Destructor, de-initializes, frees memory allocated for buffers, etc.
	~Cmt1s();
		//! \brief Close the serial communication port.
	XsensResultValue close (void);
	/*! \brief Manipulate the Serial control lines

		The function manipulates the serial control lines that are indicated by the 
		mask parameter. Note that only the DTR and RTS lines can be set by win32.
		\param mask		Indicates which lines are to be manipulated and which should be
						left alone.
		\param state	Contains the new state of the control lines.
	*/
	XsensResultValue escape (const CmtControlLine mask, const CmtControlLine state);
	/*! \brief Flush all data to be transmitted / received.
	
		This function tries to send and receive any remaining data immediately 
		and does not return until the buffers are empty.
	*/
	XsensResultValue flushData (void);
		//! Return the baudrate that is currently being used by the port
	uint32_t getBaudrate(void) const { return m_baudrate; }
		//! Return the handle of the port
	HANDLE getHandle(void) const { return m_handle; }
		//! Retrieve the port number that was last successfully opened.
	uint8_t getPortNr (void) const { return m_port; }
		//! Retrieve the port name that was last successfully opened.
	void getPortName(char *portname) const { sprintf(portname, "%s", m_portname); }
		//! Return the error code of the last operation.
	XsensResultValue getLastResult(void) const { return m_lastResult; }
		//! Return the current timeout value
	uint32_t getTimeout (void) const { return m_timeout; }
		//! Return whether the communication port is open or not.
	bool isOpen (void) const { return m_isOpen; }
	
	/*! \brief Open a communcation channel to the given serial port name.
	
		The port is automatically initialized to the given baudrate.
		If the baudrate is set to 0, the baud rate is automatically detected. If possible.
	*/
	XsensResultValue open ( const char *portName,
							const uint32_t baudRate = CMT_DEFAULT_BAUD_RATE,
							uint32_t readBufSize = CMT_DEFAULT_READ_BUFFER_SIZE,
							uint32_t writeBufSize = CMT_DEFAULT_WRITE_BUFFER_SIZE);

#ifdef _WIN32
	/*! \brief Open a communication channel to the given COM port number.
	
		The port is automatically initialized to the given baud rate.
		If the baudrate is set to 0, the baud rate is automatically detected. If possible.
	*/
	XsensResultValue open ( const uint32_t portNumber,
							const uint32_t baudRate = CMT_DEFAULT_BAUD_RATE,
							uint32_t readBufSize = CMT_DEFAULT_READ_BUFFER_SIZE,
							uint32_t writeBufSize = CMT_DEFAULT_WRITE_BUFFER_SIZE);
#endif
	/*! \brief Read data from the serial port and put it into the data buffer.

		This function reads as much data as possible from the com port (non-blocking) and 
		put as much data as will fit into the data buffer. Any excess data is stored in 
		the \c m_readBuffer member variable. If there was enough data in m_readBuffer to 
		fulfill the request, the data parameter is first filled and the port is polled 
		afterwards.
		\param maxLength	The maximum amount of data read.
		\param data			Pointer to a buffer that will store the received data.
		\param length		The number of bytes placed into \c data.
	*/
	XsensResultValue readData (const uint32_t maxLength, uint8_t* data,
								uint32_t* length = NULL);
	//! Set the callback function for when bytes have been received
	XsensResultValue setCallbackFunction(CmtCallbackSelector tp, int32_t instance, CmtCallbackFunction func, void* param);
	/*! \brief Set the default timeout value to use in blocking operations.

		This function sets the value of m_timeout. There is no infinity value. The value 0 
		means that all blocking operations now become polling (non-blocking) operations. 
		If the value is set to or from 0, the low-level serial port settings may be 
		changed in addition to the m_timeout value.
	*/
	XsensResultValue setTimeout (const uint32_t ms = CMT1_DEFAULT_TIMEOUT);
	/*! \brief Wait for data to arrive or a timeout to occur.

		The function waits until \c maxLength data is available or until a timeout occurs.
		The function returns success if data is available or XsensResultValue::TIMEOUT if a 
		timeout occurred. A timeout value of 0 indicates that the default timeout stored
		in the class should be used.
	*/
	XsensResultValue waitForData (const uint32_t maxLength, uint8_t* data,
								uint32_t* length = NULL);
	/*! \brief Write the data to the serial port.

		The function writes the given data to the connected COM port.
		The default timeout is respected in this operation.
	*/
	XsensResultValue writeData (const uint32_t length, const uint8_t* data,
								uint32_t* written);

};

//////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////// Cmt1f  /////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

/*! \brief The low-level file communication class.
*/
class Cmt1f {
private:
		//! This object cannot be copied, so this function is not implemented.
	Cmt1f(const Cmt1f& ref);

protected:
		//! The file handle
	FILE* m_handle;
		//! Contains the size of the file
	CmtFilePos m_fileSize;
		//! The last read position in the file
	CmtFilePos m_readPos;
		//! The last write position in the file
	CmtFilePos m_writePos;
		//! The last result of an operation
	mutable XsensResultValue m_lastResult;
		//! Contains the name of the file that was last successfully opened.
	char m_filename[CMT_MAX_FILENAME_LENGTH];
		//! Contains the name of the file that was last successfully opened using unicode.
	wchar_t m_filename_w[CMT_MAX_FILENAME_LENGTH];
		//! Indicates if the file is open or not.
	bool m_isOpen;
		//! Indicates if we're using the unicode filename or the regular filename
	bool m_unicode;
	/*! \brief Indicates whether the last operation was a read or write operation.

		This value is used to check whether or not a seek is required to perform a 
		requested read or write operation.
	*/
	bool m_reading;
		//! Indicates if the file was opened in read-only mode
	bool m_readOnly;
		//! Change from writing to reading mode
	void gotoRead(void);
		//! Change from reading to writing mode
	void gotoWrite(void);
public:
		//! Default constructor, initializes all members to their default values.
	Cmt1f();
		//! Destructor.
	~Cmt1f();
	/*! \brief Write data to the end of the file.

		The function writes the given data to the file at the end. The current write
		position is also moved to the end of the file.
	*/
	XsensResultValue appendData(const uint32_t length,  const void* data);
		//! Close the file.
	XsensResultValue close(void);
		//! Close the file and delete it.
	XsensResultValue closeAndDelete(void);
		//! Open an empty file.
	XsensResultValue create(const char* filename);
		//! Open an empty file using a unicode path + filename.
	XsensResultValue create(const wchar_t* filename);
	/*! \brief Delete the given data from the file.
	
		The function erases the given data from the file at the given write position. This
		operation may take a while to complete, but is faster than insertData.

		The write position is not changed and the read position	is checked for validity.
	*/
	XsensResultValue deleteData(const CmtFilePos start, const uint32_t length);
	/*! \brief Find a string of bytes in the file

		The function searches from the current read position until the given \c needle is 
		found. If the needle is not found, XsensResultValue::NOT_FOUND is returned. The function
		will update the seek position to the first character of the found needle.
		\param needle		The byte string to find.
		\param needleLength	The length of the byte string.
		\param pos			Out: The position where the needle was found. This will point
							to the first character of the found needle.
	*/
	XsensResultValue find(const void* needle, const uint32_t needleLength, CmtFilePos& pos);
	/*! \brief Flush all data to be written.
		This function writes any remaining data immediately and does not return
		until this is done.
	*/
	XsensResultValue flushData(void);
		//! Return the size of the file.
	CmtFilePos getFileSize(void) const { return m_fileSize; }
		//! Return the result code of the last operation.
	XsensResultValue getLastResult(void) const	{ return m_lastResult; }
	/*! \brief Retrieve the filename that was last successfully opened.

		\param filename	A buffer for storing the filename. The buffer should be able 
				to hold the filename. A safe size is to make it at least 256 bytes.
	*/
	XsensResultValue getName(char* filename) const;
	/*! \brief Retrieve the filename that was last successfully opened in unicode.

		\param filename	A buffer for storing the filename. The buffer should be able 
				to hold the filename. A safe size is to make it at least 256 wide characters.
	*/
	XsensResultValue getName(wchar_t* filename) const;
		//! Return the current read position.
	CmtFilePos getReadPos(void) const { return m_readPos; }
		//! Return the current write position.
	CmtFilePos getWritePos(void) const { return m_writePos; }
	/*! \brief Insert the given data into the file.
	
		The function writes the given data to the file at the current write position. This
		operation may take a while to complete.
		
		The write position is placed at the end of the inserted data.
	*/
	XsensResultValue insertData(const CmtFilePos start, const uint32_t length,
								const void* data);
		//! Return whether the file is open or not.
	bool isOpen(void) const { return m_isOpen; }
		//! Return whether the file is readonly or not.
	bool isReadOnly(void) const { return !m_isOpen || m_readOnly; }
		//! Open a file.
	XsensResultValue open(const char* filename, const bool create, const bool readOnly);
		//! Open a file using a unicode filename.
	XsensResultValue open(const wchar_t* filename, const bool create, const bool readOnly);
	/*! \brief Read data from the file and put it into the data buffer.

		This function reads exactly the number of bytes as requested from the file.
		\param maxLength	The amount of data that will be read.
		\param data			Pointer to a buffer that will store the read data.
		\param length		pointer to a variable that will store the number of bytes
							that were actually read. The parameter may be NULL.
	*/
	XsensResultValue readData(const uint32_t maxLength, void* data, uint32_t* length);
	/*! \brief Read data from the file and put it into the data buffer.

		This function reads upp to the number of bytes as requested from the file.
		The function will also stop if the given terminator character is encountered.
		The terminator is included in the output buffer.
		\param maxLength	The amount of data that will be read.
		\param data			Pointer to a buffer that will store the read data.
		\param terminator	A character that will end the read operation if encountered.
		\param length		The actual number of bytes read, including the terminator
							character, if encountered.
	*/
	XsensResultValue readData(const uint32_t maxLength, const char terminator, void* data, uint32_t* length);
	/*! \brief Set the new absolute read position

		The read position is checked against the filesize before committing.
	*/
	XsensResultValue setReadPos(const CmtFilePos pos);
	/*! \brief Set the new absolute write position

		The write position is checked against the filesize before committing.
	*/
	XsensResultValue setWritePos(const CmtFilePos pos = -1);

	/*! \brief Write data to the file.

		The function writes the given data to the file at the current write position.
	*/
	XsensResultValue writeData(const uint32_t length,  const void* data);
};

}	// end of xsens namespace

#endif	// _CMT1_H_2006_04_12
