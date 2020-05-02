
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

#ifndef COMMUNICATOR_H
#define COMMUNICATOR_H

#include "callbackmanagerxda.h"
#include "openportstage.h"
#include "protocolmanager.h"
#include "replyobject.h"

#include <xscommon/xsens_mutex.h>
#include "iointerfacefile.h"
#include "iprotocolhandler.h"
#include "serialinterface.h"
#include <xstypes/xsresultvalue.h>
#include <xstypes/xstimestamp.h>

#include <memory>
#include <vector>

struct XsByteArray;
struct XsString;
struct XsMessage;
struct XsDeviceConfiguration;
namespace xsens {
class ReplyMonitor;
}

/*! \class Communicator
	\brief A base struct for a communication interface
*/
struct Communicator : public CallbackManagerXda
{
public:
	void destroy();

	//! \brief The communicator deleter
	struct Deleter
	{
		//! \brief A function that destroys communicator
		void operator()(Communicator* c) {if (c != nullptr) c->destroy();}
	};

	//! \brief Initializes of UniquePtr<T>
	template <typename T>
	using UniquePtr = std::unique_ptr<T, Deleter>;

	//! \brief Constructs a new Communicator of type T and returns it as a UniquePtr<T>
	template <typename T>
	static UniquePtr<T> createUniquePtr()
	{
		return UniquePtr<T>(new T(), Deleter());
	}

	//! \brief Creates a UniquePtr<T> from a Communicator*
	template <typename T>
	static UniquePtr<T> createUniquePtr(T* communicator)
	{
		return UniquePtr<T>(communicator, Deleter());
	}

	Communicator(void);

	bool doTransaction(const XsMessage &message);
	bool doTransaction(const XsMessage &message, uint32_t timeout);
	bool doTransaction(const XsMessage &message, XsMessage &rcv);

	//! \copybrief Communicator::doTransaction
	virtual bool doTransaction(const XsMessage &message, XsMessage &rcv, uint32_t timeout) = 0;

	//! \brief Sets a default \a timeout
	void setDefaultTimeout(uint32_t timeout) { m_defaultTimeout = timeout; }

	//! \returns a default timeout
	uint32_t defaultTimeout() const { return m_defaultTimeout; }

	XsResultValue lastResult() const;
	XsString lastResultText() const;
	virtual void handleMessage(const XsMessage &message);
	XsSize childDeviceCount() const;
	XsDeviceId masterDeviceId() const;

	std::shared_ptr<ReplyObject> addReplyObject(uint8_t mid);
	std::shared_ptr<ReplyObject> addReplyObject(uint8_t mid, XsSize offset, XsSize size, uint8_t const * data);
	std::shared_ptr<ReplyObject> addReplyObject(ReplyObject* obj);

	// live stuff

	/*! \brief Request a device to go to config mode
		\param detectRs485 when set to true it will try to detect and use an RS485 interface
		\returns XRV_OK if succeeded
	*/
	virtual XsResultValue gotoConfig(bool detectRs485 = false) = 0;

	/*! \brief Request a device to go to measurement mode
		\returns XRV_OK if succeeded
	*/
	virtual XsResultValue gotoMeasurement() = 0;

	/*! \brief Request a device to get device ID
		\returns XRV_OK if succeeded
	*/
	virtual XsResultValue getDeviceId() = 0;

	/*! \brief Set the timeout for the gotoConfig function
		\param timeout The desired timeout value in ms, if 0 the default value is used
	*/
	virtual void setGotoConfigTimeout(uint32_t timeout) = 0;

	/*! \brief Write message to the device
		\param message a message
		\returns true on successful write, false otherwise. This doesn't guarantee proper delivery of the message. Use doTransaction for that
	*/
	virtual bool writeMessage(const XsMessage &message) = 0;

	/*! \brief Flushes all remaining data on the open port
	*/
	virtual void flushPort() = 0;

	/*! \brief Closes the open port
	*/
	virtual void closePort() = 0;

	/*! \brief Schedules to close the open port
	*/
	virtual void scheduleClosePort() { closePort(); }

	/*! \returns true if the port is open
	*/
	virtual bool isPortOpen() const = 0;

	/*! \returns XsPortInfo of the current port
	*/
	virtual XsPortInfo portInfo() const = 0;

	/*! \brief Opens a port
		\param portInfo A port information that you want to open
		\param stage A openning stage of a communication port
		\param detectRs485 When set to true it will try to detect and use an RS485 interface
		\returns true if port is successfully open
	*/
	virtual bool openPort(const XsPortInfo &portInfo, OpenPortStage stage = OPS_Full, bool detectRs485 = false) = 0;

	/*! \brief Reopens the port
		\param stage A openning stage of a communication port
		\param skipDeviceIdCheck When set to true it will skip device ID check
		\returns true if port is successfully reopened
	*/
	virtual bool reopenPort(OpenPortStage stage = OPS_Full, bool skipDeviceIdCheck = false) = 0;

	/*! \brief Returns true if the \a other device is docked at this device
	*/
	virtual bool isDockedAt(Communicator *other) const = 0;

	/*! \brief Either disable or enable (default) the keep alive mechanism (if supported by the device)
	*/
	virtual void setKeepAlive(bool enable) = 0;

	// file stuff

	/*! \brief Close the log file
		\returns true if the log file was successfully closed or never open
	*/
	virtual void closeLogFile() = 0;

	/*! \brief Read a message from the open file
		\param msgId an ID of message
		\returns The message that was read or if no matching message was found a cleared message
	*/
	virtual XsMessage readMessage(uint8_t msgId = 0) = 0;

	/*! \brief Read a message from the start of the open file
		\param msgId an ID of message
		\param maxMsgs a maximum of messages to read
		\returns The message that was read or if no matching message was found a cleared message
	*/
	virtual XsMessage readMessageFromStartOfFile(uint8_t msgId, int maxMsgs = 0) = 0;

	/*! \brief Read multiple similar messages from the start of the open file
		\param msgId an ID of message
		\param maxMsgs a maximum of messages to read
		\returns The message that was read or if no matching message was found a cleared message
	*/
	virtual std::deque<XsMessage> readMessagesFromStartOfFile(uint8_t msgId, int maxMsgs = 0) = 0;

	/*! \brief Load a complete logfile
	*/
	virtual void loadLogFile(XsDevice* device) = 0;

	/*! \brief Aborts loading a logfile
	*/
	virtual void abortLoadLogFile() = 0;

	/*!	\brief Open the log file
	`	\param filename a name of file to open
		\returns true if the file was opened successfully
	*/
	virtual bool openLogFile(const XsString &filename) = 0;

	/*! \returns The name of the logfile
	*/
	virtual XsString logFileName() const = 0;

	/*! \returns The size of the logfile
	*/
	virtual XsFilePos logFileSize() const = 0;

	/*! \returns The date of the logfile
	*/
	virtual XsTimeStamp logFileDate() const = 0;

	/*! \returns The read position of the logfile
	*/
	virtual XsFilePos logFileReadPosition() const = 0;

	/*! \brief Resets the logfile read position
	*/
	virtual void resetLogFileReadPosition(void) = 0;

	/*! \returns true if we are reading from the file
	*/
	virtual bool isReadingFromFile() const = 0;

	/*! \brief Wait for the last processing task to complete in the threadpool
	*/
	virtual void waitForLastTaskCompletion() = 0;

	/*! \returns true if the file operation started by loadLogFile is still in progress
	*/
	virtual bool isLoadLogFileInProgress() const;

	/*! \returns true if reprocessing is allowed
	*/
	virtual bool allowReprocessing() const;

	/*!	\returns The maximum set of messages, from the beginning of a file, which must contain a Configuration message
		\note SerialCommunicator & MTi: 5 (GotoConfig, ReqDeviceId, GotoConfig, Initbus, ReqConfiguration)
		\note BodyPack requires more because we also may have some other meta-data in the file, but base is: 7 (ReqDeviceId, RequestControl, SetDataPort, GotoConfig [Communicator & XsDevice], Initbus, ReqConfiguration)
	*/
	static int configurationMessageSearchLimit() { return 16; }

	virtual void setCredentials(XsString const& id, XsString const& key);

	bool sanityCheck(XsMessage const & msg) const;

	/*! \brief Adds a protocol handler
		\param handler an \ref IProtocolHandler
	*/
	virtual void addProtocolHandler(IProtocolHandler *handler);
	void removeProtocolHandler(XsProtocolType type);
	bool hasProtocol(XsProtocolType type) const;

	void setMasterDevice(XsDevice *masterDevice);

protected:
	virtual ~Communicator();
	virtual void prepareForDestruction();

	XsDevice *masterDevice() const;
	std::shared_ptr<ProtocolManager> protocolManager() const;

	// these may be a bit surprising:
	void setLastResult(XsResultValue lastResult, XsString const& text = XsString()) const;
	XsResultValue setAndReturnLastResult(XsResultValue lastResult, XsString const& text = XsString()) const;

	void setMasterDeviceId(const XsDeviceId &deviceId);

protected:

	//! \brief Prepared for destruction boolean variable
	bool m_preparedForDestruction;

	//! \brief A master device object
	XsDevice *m_masterInfo;

	//! \brief A shared pointer to protocl manager
	std::shared_ptr<ProtocolManager> m_protocolManager;

	//! \brief A master device ID
	XsDeviceId m_masterDeviceId;

	//! \brief An unique pointer to a reply monitor
	std::unique_ptr<xsens::ReplyMonitor> m_replyMonitor;

	//! \brief A last result variable
	mutable XsResultValue m_lastResult;

	//! \brief A last result string
	mutable XsString m_lastResultText;

	//! \brief A handle mutex
	mutable xsens::Mutex m_handleMux;

	//! \brief A default timeout variable
	uint32_t m_defaultTimeout;
};

#endif
