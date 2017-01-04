/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef _CMT2_H_2006_04_13
#define _CMT2_H_2006_04_13

#ifndef _CMT_MONOLITHIC
#	include "cmt1.h"
#	include "cmtmessage.h"
#	include "xsens_fifoqueue.h"
#endif

namespace xsens {

//////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// Support  functions ///////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

//! Find a valid message in the given buffer. If nothing is found, the function returns -1. Otherwise the index of the first character of the message is returned.
int32_t findValidMessage(const uint8_t* buffer, const uint16_t bufferLength);


//////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////// Cmt2s  /////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
/*! \brief Mid-level serial communication class.

	The class uses CMT level 1, but does not inherit from it. If software needs to access 
	the level 1 component, it needs to be done through the getCmt1s() function.
*/
class Cmt2s {
private:
	//! This object cannot be copied, so this function is not implemented.
	Cmt2s(const Cmt2s& ref);

	/*! \brief The message received function.
	
		This function is automatically called every time a complete message was read from the 
		connected COM port.
	*/
	CmtCallbackFunction m_onMessageReceived;
	//! Custom, user supplied parameter for the OnMessageReceived callback function, passed as the first argument
	int32_t m_onMessageReceivedInstance;
	//! Custom, user supplied parameter for the OnMessageReceived callback function, passed as the last argument
	void* m_onMessageReceivedParam;

	/*! \brief The message sent function.
	
		This function is automatically called every time a complete message was sent to the 
		connected COM port.
	*/
	CmtCallbackFunction m_onMessageSent;
	//! Custom, user supplied parameter for the OnMessageSent callback function, passed as the first argument
	int32_t m_onMessageSentInstance;
	//! Custom, user supplied parameter for the OnMessageSent callback function, passed as the last argument
	void* m_onMessageSentParam;

protected:
		//! The baudrate that was last set to be used by the port
	uint32_t m_baudrate;
		//! The CMT level 1 object that this class operates on
	Cmt1s m_cmt1s;
		//! The last result of an operation
	mutable XsensResultValue m_lastResult;
		//! Buffer for reading data until a valid message is read. Should be rarely used.
	uint8_t m_readBuffer[CMT_DEFAULT_READ_BUFFER_SIZE];
		//! The number of valid bytes in the readBuffer.
	uint16_t m_readBufferCount;
		//! Timeout in ms for blocking operations
	uint32_t m_timeout;
		//! The timestamp at which to end an operation
	uint32_t m_toEnd;
public:
		//! Default constructor, initialize all members to their default values.
	Cmt2s();

		//! Destructor, de-initialize, free memory allocated for buffers, etc.
	~Cmt2s();

		//! Close the serial communication port.
	XsensResultValue close(void);

		//! Return the baudrate that is currently being used by the port
	uint32_t getBaudrate(void) { return (m_baudrate = m_cmt1s.getBaudrate()); }
	/*! \brief Return a reference to the embedded Cmt1s object.
	
		Any manipulation of the object should be done through Cmt2s. Cmt2s integrity is 
		not guaranteed if the Cmt1s object is manipulated directly.
	*/
	Cmt1s* getCmt1s(void) { return &m_cmt1s; }
		//! Return the error code of the last operation.
	XsensResultValue getLastResult(void) const { return m_lastResult; }
		//! Retrieve the port that the object is connected to.
	XsensResultValue getPortNr(uint8_t& port) const;
	XsensResultValue getPortNr(int32_t& port) const;
	XsensResultValue getPortName(char *portname) const;
		//! Return the current timeout value in ms.
	uint32_t getTimeout(void) const { return m_timeout; }
		//! Return whether the communication port is open or not.
	bool isOpen (void) const { return (m_cmt1s.isOpen()); }
		//! Open a communication channel to the given serial port name.
	XsensResultValue open(const char *portName, 
								const uint32_t baudRate = CMT_DEFAULT_BAUD_RATE);
#ifdef _WIN32
		//! Open a communication channel to the given COM port number.
	XsensResultValue open(const uint32_t portNumber,
								const uint32_t baudRate = CMT_DEFAULT_BAUD_RATE);
#endif	
	/*! \brief Read a message from the COM port.
	
		The function reads data from the embedded Cmt1s object. The data is then converted
		into a Message object.
		If an error occurred, a NULL pointer is returned and the error code can be
		retrieved with getLastError().
	*/
	XsensResultValue readMessage(Message* rcv);

	//! Set the callback function for when a message has been received or sent
	XsensResultValue setCallbackFunction(CmtCallbackSelector tp, int32_t instance, CmtCallbackFunction func, void* param);

	/*! \brief Set the default timeout value to use in blocking operations.

		This function sets the level 2 timeout value. The L1 value is set to half the given
		timeout value.
	*/
	XsensResultValue setTimeout(const uint32_t ms = CMT2_DEFAULT_TIMEOUT);

	/*! \brief Wait for a message to arrive.
	
		The function waits for a message to arrive or until a timeout occurs.
		If the msgId parameter is set to a value other than 0, the function will wait
		until a message has arrived with that particular msgId.
		
		\note msgId 0 is the ReqDeviceID MID, but that is an outgoing only message. It is
		illogical to wait for a message that will never be sent to the host. So 0 is a
		safe value for the 'all' messages option.
		
		\note If an error message is received, the contents are stored in the m_lastResult
		field and a NULL value is returned immediately.
	*/
	XsensResultValue waitForMessage(Message* rcv, const uint8_t msgId, uint32_t timeoutOverride, bool acceptErrorMessage);
	/*! \brief Send a message over the COM port.
	
		The function attempts to write the message over the connected COM port.
		\param msg	The message to send.
	*/
	XsensResultValue writeMessage(Message* msg);
};

//////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////// Cmt2f  /////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

/*! \brief The mid-level file communication class.

	The class uses CMT level 1, but does not inherit from it. If software needs to access 
	the level 1 component, it needs to be done through the getCmt1f() function.
*/
class Cmt2f {
private:
		//! This object cannot be copied, so this function is not implemented.
	Cmt2f(const Cmt2f& ref);

protected:
		//! The Cmt1f object that is used for the low-level operations
	Cmt1f m_cmt1f;
		//! The last result of an operation
	mutable XsensResultValue m_lastResult;

	bool m_readOnly;				//!< When set to true, the file is read-only and attempts to write to it will fail

public:
		//! Default constructor
	Cmt2f();
		//! Destructor.
	~Cmt2f();
		//! Close the file.
	XsensResultValue close(void);
		//! Close the file and delete it.
	XsensResultValue closeAndDelete(void);
		//! Create a new file with level 2 header
	XsensResultValue create(const char* filename);
		//! Create a new file with level 2 header
	XsensResultValue create(const wchar_t* filename);
		//! Get a reference to the embedded Cmt1f object
	Cmt1f* getCmt1f(void);

		//! Return the error code of the last operation.
	XsensResultValue getLastResult(void) const;
		//! Retrieve the filename that was last successfully opened.
	XsensResultValue getName(char* filename) const;
		//! Retrieve the filename that was last successfully opened.
	XsensResultValue getName(wchar_t* filename) const;
		//! Return whether the file is open or not.
	bool isOpen(void) const;
		//! Open a file and read the header
	XsensResultValue open(const char* filename, const bool readOnly = false);
		//! Open a file and read the header
	XsensResultValue open(const wchar_t* filename, const bool readOnly = false);
		//! Read the next message from the file, when msgId is non-zero, the first matching message will be returned
	XsensResultValue readMessage(Message* msg, const uint8_t msgId = 0);
		//! Get the current file size
	CmtFilePos getFileSize(void);
		//! Get the current read position
	CmtFilePos getReadPosition(void);
		//! Set the read position to the given position
	XsensResultValue setReadPosition(CmtFilePos pos);
		//! Write a message to the end of the file
	XsensResultValue writeMessage(const Message* msg);
};

} // end of xsens namespace

#endif	// _CMT2_H_2006_04_13
