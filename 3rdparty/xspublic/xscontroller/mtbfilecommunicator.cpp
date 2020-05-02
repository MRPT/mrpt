
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

#include "mtbfilecommunicator.h"
#include "xsdevice_def.h"
#include "xsdeviceconfiguration.h"

#include <xscommon/threading.h>
#include <xscommon/xprintf.h>
#include <xscommon/xsens_debugtools.h>
#include <xscommon/xsens_janitors.h>
#include <xstypes/xsportinfo.h>
#include "iointerfacefile.h"
#include "messageextractor.h"

using namespace xsens;

static const XsFilePos fileBlockSize = 4096;

/*! \class MtbFileCommunicator
	\brief A class that is used for the communcation with a mtb file
*/

/*! \brief Constructs new MtbFileCommunicator
*/
Communicator *MtbFileCommunicator::construct()
{
	return new MtbFileCommunicator;
}

/*! \brief Default constructor
*/
MtbFileCommunicator::MtbFileCommunicator()
	: Communicator()
	, m_abortLoadLogFile(false)
	, m_loadFileTaskId(0)
	, m_extractor(nullptr)
	, m_extractedMessages(new std::deque<XsMessage>)
{
	m_extractor = new MessageExtractor(protocolManager());
	for (auto p : *protocolManager())
		p->ignoreMaximumMessageSize(true);
}

/*! \brief Constructor that uses \a ioInterfaceFile
*/
MtbFileCommunicator::MtbFileCommunicator(std::shared_ptr<IoInterfaceFile> ioInterfaceFile)
	: Communicator()
	, m_ioInterfaceFile(ioInterfaceFile)
	, m_abortLoadLogFile(false)
	, m_loadFileTaskId(0)
	, m_extractor(nullptr)
	, m_extractedMessages(new std::deque<XsMessage>)
{
	m_extractor = new MessageExtractor(protocolManager());
	for (auto p : *protocolManager())
		p->ignoreMaximumMessageSize(true);
}

/*! Default destructor
*/
MtbFileCommunicator::~MtbFileCommunicator()
{
	delete m_extractor;
	delete m_extractedMessages;
}

/*! \brief A rather stupid function that tries to convert a live timeout into a number of messages
*/
uint32_t MtbFileCommunicator::timeoutToMaxMessages(uint32_t timeout)
{
	return timeout / 20; // 100 ms corresponds to 5 messages. Reasonable, no?
}

/*! \brief Pretend to be a live system
	\param msg The message to send
	\param rcv The message to receive
	\param timeout The timeout in ms
	\details This one is nowhere near finished, but it does the trick for simple systems if a reply is available.
	\returns True if successful
 */
bool MtbFileCommunicator::doTransaction(const XsMessage &msg, XsMessage &rcv, uint32_t timeout)
{
	XsXbusMessageId expected = static_cast<XsXbusMessageId>(msg.getMessageId() + 1);
	std::deque<XsMessage> messages = readMessagesFromStartOfFile(expected, timeoutToMaxMessages(timeout));
	rcv.clear();
	for (const XsMessage &r : messages)
	{
		if (r.getBusId() != msg.getBusId())
			continue;

		switch (msg.getMessageId())
		{
		case XMID_ReqFrameRates:
			if (msg.getDataShort() != r.getDataShort()) // data identifier
				continue;
			break;
		default:
			break;
		}
		rcv = r;
		return true;
	}
	return (lastResult() == XRV_OK && !rcv.empty());
}

/*! \brief Prepares for the destruction
*/
void MtbFileCommunicator::prepareForDestruction()
{
	abortLoadLogFile();
	waitForLastTaskCompletion();

	m_abortLoadLogFile = true;
	completeAllThreadedWork();

	Communicator::prepareForDestruction();
}

/*! \brief Completes all threaded work
*/
void MtbFileCommunicator::completeAllThreadedWork()
{
	ThreadPool::instance()->waitForCompletion(m_loadFileTaskId);
}

/*! \brief Wait for the last processing task to complete in the threadpool
	\details This function is usually called after abort() to make sure that no more processing
	is going on.
*/
void MtbFileCommunicator::waitForLastTaskCompletion()
{
	completeAllThreadedWork();
}

/* \brief Closes the log file
*/
void MtbFileCommunicator::closeLogFile()
{
	m_ioInterfaceFile.reset();
}

/*! \brief Read a message from the start of the open file
	\details This function will reset the read position in the file to the start and will
	then search for the message with the given message ID. After the message has been found (or not)
	the read position will be restored to its original position.
	\param msgId The ID of the message to search for
	\param maxMsgs Optional parameter to limit the maximum number of messages to search. When 0, the
	function will continue until the message has been found or the end of the file has been reached.
	\returns The messsage that was read
*/
XsMessage MtbFileCommunicator::readMessageFromStartOfFile(uint8_t msgId, int maxMsgs)
{
	if (!m_ioInterfaceFile)
	{
		setLastResult(XRV_INVALIDOPERATION);
		return XsMessage();
	}

	// store current read state and make sure we reset to it when we're done
	auto oldPos = logFileReadPosition();
	MessageExtractor* ex = m_extractor;
	auto exm = m_extractedMessages;
	m_extractor = new MessageExtractor(protocolManager());
	m_extractedMessages = new std::deque<XsMessage>;
	JanitorStdFunc0<> resetExtractor([this, ex, exm, oldPos]()
	{
		delete this->m_extractor;
		this->m_extractor = ex;
		this->m_extractedMessages = exm;
		this->m_ioInterfaceFile->setReadPosition(oldPos);
	});

	// start reading from start of file
	m_ioInterfaceFile->setReadPosition(0);

	if (maxMsgs == 0)
		return readMessage(msgId);

	for (int count = 0; count < maxMsgs; ++count)
	{
		XsMessage msg = readMessage();
		if (msgId == 0 || msg.getMessageId() == msgId)
			return msg;
		if (m_ioInterfaceFile->getLastResult() != XRV_OK)
		{
			setLastResult(m_ioInterfaceFile->getLastResult());
			return XsMessage();
		}
	}

	setLastResult(XRV_OTHER);
	return XsMessage();
}

/*! \brief Read multiple similar messages from the start of the open file
	\details This function will reset the read position in the file to the start and will
	then search for all messages with the given message ID. Afterwards the read position will
	be restored to its original position.
	\param msgId The ID of the message to search for.
	\param maxMsgs Optional parameter to limit the maximum number of messages to search. When 0, the
	function will continue until the end of the file has been reached.
	\returns The messsage that was read
*/
std::deque<XsMessage> MtbFileCommunicator::readMessagesFromStartOfFile(uint8_t msgId, int maxMsgs)
{
	std::deque<XsMessage> rv;

	if (!m_ioInterfaceFile)
	{
		setLastResult(XRV_INVALIDOPERATION);
		return rv;
	}

	// store current read state and make sure we reset to it when we're done
	auto oldPos = logFileReadPosition();
	MessageExtractor* ex = m_extractor;
	auto exm = m_extractedMessages;
	m_extractor = new MessageExtractor(protocolManager());
	m_extractedMessages = new std::deque<XsMessage>;
	JanitorStdFunc0<> resetExtractor([this, ex, exm, oldPos]()
	{
		delete this->m_extractedMessages;
		delete this->m_extractor;
		this->m_extractor = ex;
		this->m_extractedMessages = exm;
		this->m_ioInterfaceFile->setReadPosition(oldPos);
	});

	// start reading from start of file
	m_ioInterfaceFile->setReadPosition(0);

	if (maxMsgs)
	{
		for (int count = 0; count < maxMsgs; ++count)
		{
			XsMessage msg = readMessage();
			if (lastResult() != XRV_OK)
				break;
			else if (msgId == 0 || msg.getMessageId() == msgId)
				rv.push_back(msg);
		}
	}
	else
	{
		XsMessage msg = readMessage(msgId);
		while (lastResult() == XRV_OK)
		{
			if (msgId == 0 || msg.getMessageId() == msgId)
				rv.push_back(msg);
			msg = readMessage(msgId);
		}
	}

	if (lastResult() == XRV_OTHER || (lastResult() == XRV_ENDOFFILE && m_ioInterfaceFile->getFileSize()))
		setLastResult(XRV_OK);
	if (lastResult() == XRV_ENDOFFILE)
		setLastResult(XRV_NODATA);

	return rv;
}

/*!	\class Xs4FileTask
	\brief A class for handling file loading process in a separate thread
	\details This task will delegate the loading of the file to yet another thread, keeping the task in the
	threadpool as a monitoring task. This is because ThreadPoolTask tasks should be small tasks and file loading
	can take a while.
*/
class Xs4FileTask : public ThreadPoolTask
{
public:

	/*! \brief Construct a file task, but does not schedule it. */
	Xs4FileTask(FileLoader* inf, XsDevice* device)
		: ThreadPoolTask()
		, m_loader(inf)
		, m_device(device)
		, m_thread(this)
	{
		assert(m_loader != 0);
		assert(m_device != 0);
	}

	/*! \brief Check if the file load is complete
	*/
	virtual bool exec()
	{
		if (m_thread.m_done)
			return true;

		if (!m_thread.isRunning())
		{
			JLDEBUGG("Starting dedicated read thread");
			m_thread.startThread();
		}
		return false;	// reschedule
	}

	/*!
		\brief Destroy this process task.
		\details Virtual destructor, required but empty.
	*/
	virtual ~Xs4FileTask()
	{
		if (m_thread.isAlive())
			m_thread.stopThread();
	}

private:
	FileLoader* m_loader;	//!< The Communicator object that contains information on calibration and filtering
	XsDevice* m_device;

	class ReaderThread : public StandardThread
	{
	public:
		ReaderThread(Xs4FileTask* task);
		int32_t innerFunction(void) override;
		Xs4FileTask* m_task;
		bool m_done;
	} m_thread;

	friend class ReaderThread;
};

Xs4FileTask::ReaderThread::ReaderThread(Xs4FileTask* task)
	: m_task(task)
	, m_done(false)
{
}

int32_t Xs4FileTask::ReaderThread::innerFunction(void)
{
	{
		XsString filename = m_task->m_device->logFileName();
		std::string name = xprintf("FileReader: %s", filename.c_str());
		xsNameThisThread(name.c_str());
		JLDEBUGG("Loading file " << filename);
	}

	try
	{
		m_task->m_loader->readLogFile(m_task->m_device);
	}
	catch (XsException const& e)
	{
		m_task->m_device->onError(m_task->m_device, e.code());
		JLALERTG(e.what());
	}
	m_done = true;
	stopThread();
	return 0;
}

/*! \brief Load a log file with thread pool
*/
void MtbFileCommunicator::loadLogFile(XsDevice* device)
{
	assert(device != 0);

	abortLoadLogFile();
	waitForLastTaskCompletion();

	m_abortLoadLogFile = false;
	Xs4FileTask* tsk = new Xs4FileTask(this, device);
	m_loadFileTaskId = ThreadPool::instance()->addTask(tsk, m_loadFileTaskId);
}

/*! \brief Read a log file into cache
	\param device : The device to read log from
	\details Read all data for \a inf into the cache
	\sa XsControl::loadLogFile
	\returns XRV_OK if successful
*/
XsResultValue MtbFileCommunicator::readLogFile(XsDevice* device)
{
	assert(device != 0);
	JLDEBUGG("");

	XsResultValue res = XRV_OK;
	XsFilePos prevFilePos = -1;
	XsString id = logFileName();

	do
	{
		try
		{
			res = readSinglePacketFromFile();
		}
		catch(...)
		{
			// Simply ignore the error and continue.
			continue;
		}
		// Only emit progress updates at complete percentages, once per percentage
		XsFilePos pos = logFileReadPosition();
		XsFilePos size = logFileSize();

		if (size)
		{
			const int percentage = (int)(pos * 100 / size);
			if (prevFilePos != percentage && percentage < 100)
			{
				onProgressUpdated(device, percentage < 100 ? percentage : 99, 100, &id);	// never report 100% until we're really done
				prevFilePos = percentage;
			}
		}
	} while (!m_abortLoadLogFile && (res == XRV_OK || res == XRV_OTHER));

	if (!m_abortLoadLogFile)
	{
		masterDevice()->onEofReached();
		onProgressUpdated(device, 100, 100, &id);
	}

	return setAndReturnLastResult(res);
}

/*! \brief Read a single XsDataPacket from an open log file
	\details Read a single XsDataPacket from the log file and place it in the correct data cache(s)
	\returns XRV_OK if successful
*/
XsResultValue MtbFileCommunicator::readSinglePacketFromFile()
{
	JLTRACEG("Reading from file");
	XsMessage msg = readMessage(0);
	if (lastResult())
		return lastResult();

	handleMessage(msg);
	return lastResult();
}

/*! \brief Abort a process that takes a long time to complete
	\details This currently only includes readLogFile()
*/
void MtbFileCommunicator::abortLoadLogFile()
{
	m_abortLoadLogFile = true;
}

/*! \brief Open a log file for input.
	\details This function opens the supplied log file for reading. The function will fail if a serial connection is currently open.
	\param filename The name of the file to open. It is recommended to use a fully qualified path+filename.
	\note This function is only available in configuration mode.
	\see closeLogFile
	\returns True if successful
*/
bool MtbFileCommunicator::openLogFile(const XsString &filename)
{
	if (m_ioInterfaceFile)
	{
		setLastResult(XRV_ALREADYOPEN);
		if (filename == logFileName())
			return true;
		return false;
	}
	m_ioInterfaceFile = std::shared_ptr<IoInterfaceFile>(new IoInterfaceFile, [](IoInterfaceFile *f) { f->close(); });

	setLastResult(m_ioInterfaceFile->open(filename, false, true));
	if (lastResult() != XRV_OK)
	{
		m_ioInterfaceFile.reset();
		return false;
	}

	// start with a sanity check on the file
	XsByteArray hdrBuf;
	XsResultValue rv;

	// Is the file readable and does it at least start with 0xFA marker?
	rv = m_ioInterfaceFile->readData(1, hdrBuf);
	if (rv != XRV_OK || hdrBuf.size() < 1 || hdrBuf[0] != 0xFA)
	{
		setLastResult(XRV_DATACORRUPT);
		m_ioInterfaceFile.reset();
		return false;
	}

	resetLogFileReadPosition();

	// try to read any message from the start
	XsMessage check = readMessage();
	if (check.getMessageId() == XMID_InvalidMessage)
	{
		setLastResult(XRV_DATACORRUPT);
		m_ioInterfaceFile.reset();
		return false;
	}

	resetLogFileReadPosition();

	// Now let's try to find the actual message we are looking for somewhere...
	XsMessage rcv = readMessageFromStartOfFile(XMID_Configuration, Communicator::configurationMessageSearchLimit());

	if (rcv.getMessageId() != XMID_Configuration)
	{
		setLastResult(XRV_DATACORRUPT);
		m_ioInterfaceFile.reset();
		return false;
	}

	XsDeviceConfiguration config;
	config.readFromMessage(rcv);
	setMasterDeviceId(XsDeviceId((char*)config.masterInfo().m_productCode, 0, 0, config.masterInfo().m_masterDeviceId));

	return true;
}

/*! \brief Retrieve the name of the open log file or an empty string if no log file is open
*/
XsString MtbFileCommunicator::logFileName() const
{
	XsString retString;
	if (!m_ioInterfaceFile)
		return retString;

	if (m_ioInterfaceFile->getName(retString) != XRV_OK)
		return XsString();
	return retString;
}

/*! \brief Retrieve the size of the open log file in bytes
*/
XsFilePos MtbFileCommunicator::logFileSize() const
{
	if (!m_ioInterfaceFile)
		return 0;

	return m_ioInterfaceFile->getFileSize();
}

/*! \brief Retrieve the date of the open log file
*/
XsTimeStamp MtbFileCommunicator::logFileDate() const
{
	if (!m_ioInterfaceFile)
		return XsTimeStamp();

	return m_ioInterfaceFile->getFileDate();
}

/*! \brief Retrieve the read position of the log file.
	\details This function will return the current read position in the open log file in bytes from the start.
	\note The read and write positions of log files are completely independent of each other.
	\note There is a look-ahead cache in place so even when the read position is already at the end of the file, there may still be some messages left to be read.
	For this reason, this function will at most return filesize-1 until the message queue is empty.
	\remarks To reset the read position, use resetLogFileReadPosition.
	\see resetLogFileReadPosition
	\returns The file read position
*/
XsFilePos MtbFileCommunicator::logFileReadPosition() const
{
	if (!m_ioInterfaceFile)
		return 0;

	auto pos = m_ioInterfaceFile->getReadPosition();
	if (pos < m_ioInterfaceFile->getFileSize())
		return pos;

	if (m_extractedMessages->empty())
		return pos;

	return pos > 0 ? pos-1 : 0;
}

/*! \brief Restart reading from the start of the open log file.
	\details This function resets the read position to the start of the open log file. Only the read position is affected, the write position remains the same.
	\see openLogFile
*/
void MtbFileCommunicator::resetLogFileReadPosition(void)
{
	if (m_ioInterfaceFile)
	{
		m_extractor->clearBuffer();
		m_extractedMessages->clear();
		setLastResult(m_ioInterfaceFile->setReadPosition(0));
	}
	else
		setLastResult(XRV_NOFILEOPEN);
}

/*! \brief Return whether we are reading from file */
bool MtbFileCommunicator::isReadingFromFile() const
{
	return true;
}

/*! \brief Read a message from the open file
	\details This function will attempt to read a full message from the open device (file or COM port
	or USB port). If msgId is non-0, the function will look for a specific message ID.
	The function will read from the device, but it won't wait for data to become available.
	\param msgId Either 0 to read the first available message or non-0 to look for a specific message
	with this ID.
	\returns The message that was read or if no matching message was found a cleared message.
*/
XsMessage MtbFileCommunicator::readMessage(uint8_t msgId)
{
	if (!m_ioInterfaceFile)
	{
		setLastResult(XRV_INVALIDOPERATION);
		return XsMessage();
	}

	XsMessage msg;

	do
	{
		msg = readNextMessage();
	}
	while (!msg.empty() && msgId != 0 && msg.getMessageId() != msgId);

	if (msgId == 0 || msg.getMessageId() == msgId)
		return msg;

	setLastResult(XRV_OTHER);
	return XsMessage();
}

/*!	\brief Read the next message from the open file.
 *	\returns The message that was read or an empty message if no message was found (end-of-file for example).
 */
XsMessage MtbFileCommunicator::readNextMessage()
{
	while (m_extractedMessages->empty())
	{
		XsByteArray raw;
		XsResultValue res = m_ioInterfaceFile->readDataBlocks(1, raw); (void)res;
		if (raw.empty())
		{
			// end of file really reached
			setLastResult(XRV_ENDOFFILE);
			return XsMessage();
		}
		m_extractor->processNewData(masterDevice(), raw, *m_extractedMessages);
	}

	setLastResult(XRV_OK);
	XsMessage msg = *m_extractedMessages->begin();
	m_extractedMessages->pop_front();
	return msg;
}

/*! \returns True if load log file is in progress
*/
bool MtbFileCommunicator::isLoadLogFileInProgress() const
{
	return 	ThreadPool::instance()->doesTaskExist(m_loadFileTaskId);
}

/*! \brief Add the protocol handler
\param handler : The protocol hanlder to add
*/
void MtbFileCommunicator::addProtocolHandler(IProtocolHandler *handler)
{
	handler->ignoreMaximumMessageSize(true);
	Communicator::addProtocolHandler(handler);
}

/* Now the disabled stuff starts */

//! \cond DO_NOT_DOCUMENT
XsResultValue MtbFileCommunicator::gotoConfig(bool detectRs485)
{
	(void)detectRs485;
	return XRV_UNSUPPORTED;
}

XsResultValue MtbFileCommunicator::gotoMeasurement()
{
	return XRV_UNSUPPORTED;
}

XsResultValue MtbFileCommunicator::getDeviceId()
{
	return XRV_UNSUPPORTED;
}

void MtbFileCommunicator::setGotoConfigTimeout(uint32_t timeout)
{
	(void)timeout;
}

bool MtbFileCommunicator::writeMessage(const XsMessage &message)
{
	(void)message;
	return false;
}

void MtbFileCommunicator::flushPort()
{
}

void MtbFileCommunicator::closePort()
{
}

bool MtbFileCommunicator::isPortOpen() const
{
	return false;
}

XsPortInfo MtbFileCommunicator::portInfo() const
{
	return XsPortInfo();
}

bool MtbFileCommunicator::openPort(const XsPortInfo &portInfo, OpenPortStage stage, bool detectRs485)
{
	(void)portInfo;
	(void)stage;
	(void)detectRs485;
	return false;
}

bool MtbFileCommunicator::reopenPort(OpenPortStage stage, bool skipDeviceIdCheck)
{
	(void)stage;
	(void)skipDeviceIdCheck;
	return false;
}

bool MtbFileCommunicator::isDockedAt(Communicator *other) const
{
	(void)other;
	return false;
}

void MtbFileCommunicator::setKeepAlive(bool enable)
{
	(void)enable;
}
//! \endcond
