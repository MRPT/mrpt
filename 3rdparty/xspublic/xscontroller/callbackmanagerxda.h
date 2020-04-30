
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

#ifndef CALLBACKMANAGERXDA_H
#define CALLBACKMANAGERXDA_H

#include "xscallback.h"

struct CallbackHandlerXdaItem;
struct CallbackManagerItem;
namespace xsens {
	class MutexReadWrite;
}
struct MtwInfo;

class CallbackManagerXda : public XsCallback {
public:
	void onDeviceStateChanged(XsDevice* dev, XsDeviceState newState, XsDeviceState oldState) override;
	void onLiveDataAvailable(XsDevice* dev, const XsDataPacket* packet) override;
	void onAllLiveDataAvailable(XsDevicePtrArray* devs, const XsDataPacketPtrArray* packets) override;
	void onMissedPackets(XsDevice* dev, int count, int first, int last) override;
	void onDataUnavailable(XsDevice* dev, int64_t packetId) override;
	void onWakeupReceived(XsDevice* dev) override;
	void onProgressUpdated(XsDevice* dev, int current, int total, const XsString* identifier) override;
	int  onWriteMessageToLogFile(XsDevice* dev, const XsMessage* message) override;
	void onBufferedDataAvailable(XsDevice* dev, const XsDataPacket* data) override;
	void onAllBufferedDataAvailable(XsDevicePtrArray* devs, const XsDataPacketPtrArray* packets) override;
	void onConnectivityChanged(XsDevice* dev, XsConnectivityState newState) override;
	void onInfoResponse(XsDevice* dev, XsInfoRequest request) override;
	void onError(XsDevice* dev, XsResultValue error) override;
	void onNonDataMessage(XsDevice* dev, XsMessage const * message) override;
	void onMessageDetected(XsDevice* dev, XsProtocolType type, XsByteArray const * rawMessage) override;
	void onMessageReceivedFromDevice(XsDevice* dev, XsMessage const * message) override;
	void onMessageSentToDevice(XsDevice* dev, XsMessage const * message) override;
	void onDataAvailable(XsDevice* dev, const XsDataPacket* data) override;
	void onAllDataAvailable(XsDevicePtrArray* devs, const XsDataPacketPtrArray* packets) override;
	void onRecordedDataAvailable(XsDevice* dev, const XsDataPacket* data) override;
	void onAllRecordedDataAvailable(XsDevicePtrArray* devs, const XsDataPacketPtrArray* packets) override;
	void onTransmissionRequest(int channelId, const XsByteArray* data) override;
	void onRestoreCommunication(const XsString* portName, XsResultValue result) override;

	CallbackManagerXda();
	~CallbackManagerXda();

	void clearCallbackHandlers(bool chain = true);
	void addCallbackHandler(XsCallbackPlainC* cb, bool chain = true);
	void removeCallbackHandler(XsCallbackPlainC* cb, bool chain = true);

	void clearChainedManagers();
	void addChainedManager(CallbackManagerXda* cm);
	void removeChainedManager(CallbackManagerXda* cm);

	void copyCallbackHandlersTo(CallbackManagerXda* cm, bool chain = true);
	void copyCallbackHandlersFrom(CallbackManagerXda* cm, bool chain = true);

	bool haveCallback(size_t functionOffset) const;

private:
	xsens::MutexReadWrite* m_callbackMutex;		//!< Administration mutex
	CallbackHandlerXdaItem* m_handlerList;		//!< The first item in the linked list of callback handlers
	CallbackManagerItem* m_managerList;			//!< The first item in the linked list of child callback managers
};

#endif
