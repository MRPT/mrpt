
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

#ifndef XSCALLBACK_H
#define XSCALLBACK_H

#include "xscallbackplainc.h"

#ifdef __cplusplus

/*! \brief Structure that contains callback functions for the Xsens Device API
	\details When programming in C++, simply overload the callback that you want to use and supply
	your XsCallbackPlainC-overloaded class to one of the setCallback functions

	When programming in C, create an XsCallbackPlainC structure and initialize each function pointer to
	the right function to call. The supplied first parameter is the address of the XsCallbackPlainC
	object that you supplied to the setCallback function. If you do not wish to receive a specific
	callback in C, set the function pointer to 0.

	In both cases, the calling application remains in control of the XsCallbackPlainC object and thus
	remains responsible for cleaning it up when the it is no longer necessary.

	\note Any callback function in %XsCallback that is not overloaded will only be called once to
	minimize callback overhead.
*/
class XsCallback : public XsCallbackPlainC
{
public:
	/*! \brief Constructor */
	XsCallback()
	{
		m_onDeviceStateChanged = sonDeviceStateChanged;
		m_onLiveDataAvailable = sonLiveDataAvailable;
		m_onMissedPackets = sonMissedPackets;
		m_onWakeupReceived = sonWakeupReceived;
		m_onProgressUpdated = sonProgressUpdated;
		m_onWriteMessageToLogFile = sonWriteMessageToLogFile;
		m_onBufferedDataAvailable = sonBufferedDataAvailable;
		m_onConnectivityChanged = sonConnectivityChanged;
		m_onInfoResponse = sonInfoResponse;
		m_onError = sonError;
		m_onNonDataMessage = sonNonDataMessage;
		m_onMessageDetected = sonMessageDetected;
		m_onMessageReceivedFromDevice = sonMessageReceivedFromDevice;
		m_onMessageSentToDevice = sonMessageSentToDevice;
		m_onAllLiveDataAvailable = sonAllLiveDataAvailable;
		m_onAllBufferedDataAvailable = sonAllBufferedDataAvailable;
		m_onDataUnavailable = sonDataUnavailable;
		m_onDataAvailable = sonDataAvailable;
		m_onAllDataAvailable = sonAllDataAvailable;
		m_onRecordedDataAvailable = sonRecordedDataAvailable;
		m_onAllRecordedDataAvailable = sonAllRecordedDataAvailable;
		m_onTransmissionRequest = sonTransmissionRequest;
		m_onRestoreCommunication = sonRestoreCommunication;
	}

	/*! \brief Destructor
		\note Make sure that the object is removed from callback generating objects before destroying it!
	*/
	virtual ~XsCallback() {}

// Swig needs these functions to be protected, not private, otherwise they are ignored.
protected:
/*! \protectedsection
	\addtogroup Callbacks
	@{
*/
	//! \copybrief m_onDeviceStateChanged
	virtual void onDeviceStateChanged(XsDevice* dev, XsDeviceState newState, XsDeviceState oldState)
	{ (void) dev; (void) newState; (void) oldState; m_onDeviceStateChanged = 0; }
	//! \copybrief m_onLiveDataAvailable
	virtual void onLiveDataAvailable(XsDevice* dev, const XsDataPacket* packet)
	{ (void) dev; (void) packet; m_onLiveDataAvailable = 0; }
	//! \copybrief m_onMissedPackets
	virtual void onMissedPackets(XsDevice* dev, int count, int first, int last)
	{ (void) dev; (void)count; (void) first; (void) last; m_onMissedPackets = 0; }
	//! \copybrief m_onWakeupReceived
	virtual void onWakeupReceived(XsDevice* dev)
	{ (void) dev; m_onWakeupReceived = 0; }
	//! \copybrief m_onProgressUpdated
	virtual void onProgressUpdated(XsDevice* dev, int current, int total, const XsString* identifier)
	{ (void) dev; (void) current; (void) total; (void)identifier; m_onProgressUpdated = 0; }
	//! \copydoc m_onWriteMessageToLogFile
	virtual int  onWriteMessageToLogFile(XsDevice* dev, const XsMessage* message)
	{ (void) dev; (void) message; m_onWriteMessageToLogFile = 0; return 1; }
	//! \copydoc m_onBufferedDataAvailable
	virtual void onBufferedDataAvailable(XsDevice* dev, const XsDataPacket* packet)
	{ (void)dev; (void)packet; m_onBufferedDataAvailable = 0; }
	//! \copydoc m_onConnectivityChanged
	virtual void onConnectivityChanged(XsDevice* dev, XsConnectivityState newState)
	{ (void) dev; (void) newState; m_onConnectivityChanged = 0; }
	//! \copydoc m_onInfoResponse
	virtual void onInfoResponse(XsDevice* dev, XsInfoRequest request)
	{ (void) dev; (void) request; m_onInfoResponse = 0; }
	//! \copydoc m_onError
	virtual void onError(XsDevice* dev, XsResultValue error)
	{ (void) dev; (void) error; m_onError = 0; }
	//! \copydoc m_onNonDataMessage
	virtual void onNonDataMessage(XsDevice* dev, XsMessage const * message)
	{ (void) dev; (void) message; m_onNonDataMessage = 0; }
	//! \copydoc m_onMessageDetected
	virtual void onMessageDetected(XsDevice* dev, XsProtocolType type, XsByteArray const * rawMessage)
	{ (void)dev; (void)type;  (void)rawMessage; m_onMessageDetected = 0; }
	//! \copydoc m_onMessageReceivedFromDevice
	virtual void onMessageReceivedFromDevice(XsDevice* dev, XsMessage const * message)
	{ (void) dev; (void) message; m_onMessageReceivedFromDevice = 0; }
	//! \copydoc m_onMessageSentToDevice
	virtual void onMessageSentToDevice(XsDevice* dev, XsMessage const * message)
	{ (void) dev; (void) message; m_onMessageSentToDevice = 0; }
	//! \copydoc m_onAllLiveDataAvailable
	virtual void onAllLiveDataAvailable(XsDevicePtrArray* devs, const XsDataPacketPtrArray* packets)
	{ (void) devs; (void) packets; m_onAllLiveDataAvailable = 0; }
	//! \copydoc m_onAllBufferedDataAvailable
	virtual void onAllBufferedDataAvailable(XsDevicePtrArray* devs, const XsDataPacketPtrArray* packets)
	{ (void)devs; (void)packets; m_onAllBufferedDataAvailable = 0; }
	//! \copybrief m_onDataUnavailable
	virtual void onDataUnavailable(XsDevice* dev, int64_t packetId)
	{ (void) dev; (void)packetId; m_onMissedPackets = 0; }
	//! \copydoc m_onDataAvailable
	virtual void onDataAvailable(XsDevice* dev, const XsDataPacket* packet)
	{ (void) dev; (void) packet; m_onDataAvailable = 0; }
	//! \copydoc m_onAllDataAvailable
	virtual void onAllDataAvailable(XsDevicePtrArray* devs, const XsDataPacketPtrArray* packets)
	{ (void) devs; (void) packets; m_onAllDataAvailable = 0; }
	//! \copydoc m_onRecordedDataAvailable
	virtual void onRecordedDataAvailable(XsDevice* dev, const XsDataPacket* packet)
	{ (void) dev; (void) packet; m_onRecordedDataAvailable = 0; }
	//! \copydoc m_onAllRecordedDataAvailable
	virtual void onAllRecordedDataAvailable(XsDevicePtrArray* devs, const XsDataPacketPtrArray* packets)
	{ (void) devs; (void) packets; m_onAllRecordedDataAvailable = 0; }
	//! \copydoc m_onTransmissionRequest
	virtual void onTransmissionRequest(int channelId, const XsByteArray* data)
	{ (void) channelId; (void)data; }
	//! \copydoc m_onRestoreCommunication
	virtual void onRestoreCommunication(const XsString* portName, XsResultValue result)
	{ (void)portName; (void)result; m_onRestoreCommunication = 0; }

//! @}

private:
/*! \privatesection */
	static void sonDeviceStateChanged(XsCallbackPlainC* cb, XsDevice* dev, XsDeviceState newState, XsDeviceState oldState) { ((XsCallback*)cb)->onDeviceStateChanged(dev, newState, oldState); }
	static void sonLiveDataAvailable(XsCallbackPlainC* cb, XsDevice* dev, const XsDataPacket* packet) { ((XsCallback*)cb)->onLiveDataAvailable(dev, packet); }
	static void sonMissedPackets(XsCallbackPlainC* cb, XsDevice* dev, int count, int first, int last) { ((XsCallback*)cb)->onMissedPackets(dev, count, first, last); }
	static void sonWakeupReceived(XsCallbackPlainC* cb, XsDevice* dev) { ((XsCallback*)cb)->onWakeupReceived(dev); }
	static void sonProgressUpdated(XsCallbackPlainC* cb, XsDevice* dev, int current, int total, const XsString* identifier) { ((XsCallback*)cb)->onProgressUpdated(dev, current, total, identifier); }
	static int  sonWriteMessageToLogFile(XsCallbackPlainC* cb, XsDevice* dev, const XsMessage* message) { return ((XsCallback*)cb)->onWriteMessageToLogFile(dev, message); }
	static void sonBufferedDataAvailable(XsCallbackPlainC* cb, XsDevice* dev, const XsDataPacket* packet) { ((XsCallback*)cb)->onBufferedDataAvailable(dev, packet); }
	static void sonConnectivityChanged(XsCallbackPlainC* cb, XsDevice* dev, XsConnectivityState newState) { ((XsCallback*)cb)->onConnectivityChanged(dev, newState); }
	static void sonInfoResponse(XsCallbackPlainC* cb, XsDevice* dev, XsInfoRequest request) { ((XsCallback*)cb)->onInfoResponse(dev, request); }
	static void sonError(XsCallbackPlainC* cb, XsDevice* dev, XsResultValue error) { ((XsCallback*)cb)->onError(dev, error); }
	static void sonNonDataMessage(XsCallbackPlainC* cb, XsDevice* dev, XsMessage const * message) { ((XsCallback*)cb)->onNonDataMessage(dev, message); }
	static void sonMessageDetected(XsCallbackPlainC* cb, XsDevice* dev, XsProtocolType type, XsByteArray const * rawMessage) { ((XsCallback*)cb)->onMessageDetected(dev, type, rawMessage); }
	static void sonMessageReceivedFromDevice(XsCallbackPlainC* cb, XsDevice* dev, XsMessage const * message) { ((XsCallback*)cb)->onMessageReceivedFromDevice(dev, message); }
	static void sonMessageSentToDevice(XsCallbackPlainC* cb, XsDevice* dev, XsMessage const * message) { ((XsCallback*)cb)->onMessageSentToDevice(dev, message); }
	static void sonAllLiveDataAvailable(XsCallbackPlainC* cb, XsDevicePtrArray* devs, const XsDataPacketPtrArray* packets) { ((XsCallback*)cb)->onAllLiveDataAvailable(devs, packets); }
	static void sonAllBufferedDataAvailable(XsCallbackPlainC* cb, XsDevicePtrArray* devs, const XsDataPacketPtrArray* packets) { ((XsCallback*)cb)->onAllBufferedDataAvailable(devs, packets); }
	static void sonDataUnavailable(XsCallbackPlainC* cb, XsDevice* dev, int64_t packetId) { ((XsCallback*)cb)->onDataUnavailable(dev, packetId); }
	static void sonDataAvailable(XsCallbackPlainC* cb, XsDevice* dev, const XsDataPacket* packet) { ((XsCallback*)cb)->onDataAvailable(dev, packet); }
	static void sonAllDataAvailable(XsCallbackPlainC* cb, XsDevicePtrArray* devs, const XsDataPacketPtrArray* packets) { ((XsCallback*)cb)->onAllDataAvailable(devs, packets); }
	static void sonRecordedDataAvailable(XsCallbackPlainC* cb, XsDevice* dev, const XsDataPacket* packet) { ((XsCallback*)cb)->onRecordedDataAvailable(dev, packet); }
	static void sonAllRecordedDataAvailable(XsCallbackPlainC* cb, XsDevicePtrArray* devs, const XsDataPacketPtrArray* packets) { ((XsCallback*)cb)->onAllRecordedDataAvailable(devs, packets); }
	static void sonTransmissionRequest(XsCallbackPlainC* cb, int channelId, const XsByteArray* data) { ((XsCallback*)cb)->onTransmissionRequest(channelId, data); }
	static void sonRestoreCommunication(XsCallbackPlainC* cb, const XsString* portName, XsResultValue result) { ((XsCallback*)cb)->onRestoreCommunication(portName, result); }
};
#endif

#endif
