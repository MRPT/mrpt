
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

#ifndef XSCALLBACKPLAINC_H
#define XSCALLBACKPLAINC_H

#include <xstypes/pstdint.h>
#include <xstypes/xsresultvalue.h>
#include <xstypes/xsinforequest.h>
#include "xsdevicestate.h"
#include "xsconnectivitystate.h"
#include "xsprotocoltype.h"

#ifndef __cplusplus
#define XSCALLBACK_INITIALIZER		{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
#endif

struct XsDevice;
struct XsDevicePtrArray;
struct XsDataPacket;
struct XsDataPacketPtrArray;
struct XsString;
struct XsMessage;
struct XsByteArray;

/*! \brief Structure that contains callback functions for the Xsens Device API
	\details When using C++, please use the overloaded class XsCallback instead.

	This structure contains pointers to functions that will be called by XDA when certain
	events occur. To use it in C, set any callback you do not wish to use to 0 and put a valid
	function pointer in the others. Then pass the object to an XsControl or XsDevice object's
	addCallbackHandler function.

	\note XDA does not copy the structure contents and does not take ownership of it. So make sure it
	is allocated on the heap or at least removed from wherever it was added by calling
	removeCallbackHandler before it is destroyed.
*/
typedef struct XsCallbackPlainC
{
/*! \defgroup Callbacks Callback functions.
	\addtogroup Callbacks
	@{
*/
	/*! \brief Called when a device's state has changed (ie config mode, measurement mode, recording mode)
		\param dev The device that initiated the callback. This may be 0 in some cases.
		\param newState The new device state
		\param oldState The old device state
	*/
	void (*m_onDeviceStateChanged)(struct XsCallbackPlainC* thisPtr, struct XsDevice* dev, XsDeviceState newState, XsDeviceState oldState);

	/*! \brief Called when new data has been received from a device or read from a file. When processing on PC is enabled, this callback occurs after processing has been done and so the packet will contain the processing output.
		\details This callback is for the Live stream, so there may be gaps in the data, but it will always contain the latest data.
		\param dev The device that initiated the callback.
		\param packet The data packet that has been received (and processed). This may be 0 when the callback originates from a non-device, such as the XsDataBundler.
		\note For most applications, attaching to the m_onDataAvailable callback is sufficient, the specific stream callbacks are only provided for exceptional cases
		\sa m_onAllLiveDataAvailable \sa m_onBufferedDataAvailable \sa m_onAllBufferedDataAvailable \sa m_onDataAvailable
	*/
	void (*m_onLiveDataAvailable)(struct XsCallbackPlainC* thisPtr, struct XsDevice* dev, const struct XsDataPacket* packet);

	/*! \brief Called when XDA detects that packets have been missed.
		\param dev The device that initiated the callback.
		\param count The number of samples that were missed
		\param first The sample counter / packet identifier of the first missed sample
		\param last The sample counter / packet identifier of the last missed sample
	*/
	void (*m_onMissedPackets)(struct XsCallbackPlainC* thisPtr, struct XsDevice* dev, int count, int first, int last);

	/*! \brief Called when a wakeup message has been received from a device. This indicates that the device has just been reset or plugged in.
		\param dev The device that initiated the callback.
	*/
	void (*m_onWakeupReceived)(struct XsCallbackPlainC* thisPtr, struct XsDevice* dev);

	/*! \brief Called when a long-duration operation has made some progress or has completed. Examples include loadLogFile and flushing of retransmissions (Awinda). When \a current == \a total the operation has completed.
		\param dev The device that initiated the callback.
		\param current The current progress.
		\param total The total work to be done. When \a current equals \a total, the task is completed.
		\param identifier An identifier for the task. This may for example be a filename for file read operations.
	*/
	void (*m_onProgressUpdated)(struct XsCallbackPlainC* thisPtr, struct XsDevice* dev, int current, int total, const struct XsString* identifier);

	/*! \brief Called when XDA has a message that could be written to a log file. \returns 0 to prevent the message from being written, non-0 to allow the write. This includes data packets. \param message The message that is ready to be written to file \sa m_onWriteDataToLogFile
		\param dev The device that initiated the callback.
		\param message The message that will be written.
		\returns true if the write to file should be allowed. Note that if ANY callback decides that the write is not allowed, it will be disallowed.
	*/
	int  (*m_onWriteMessageToLogFile)(struct XsCallbackPlainC* thisPtr, struct XsDevice* dev, const struct XsMessage* message);

	/*! \brief Called when XDA has a data packet that could be written to a log file.
		\details This callback is for the Buffered stream, which will attempt to retransmit missed data when in Recording mode. So there should be no gaps (when recording), but the data arrival may be delayed a bit. When not recording, the behaviour is identical to the Live stream.
		\param dev The device that initiated the callback.
		\param packet The data message that is ready to be written to file \sa onWriteMessageToLogFile
		\sa m_onAllLiveDataAvailable \sa m_onLiveDataAvailable \sa m_onAllBufferedDataAvailable \sa m_onDataAvailable
		\note For most applications, attaching to the m_onDataAvailable callback is sufficient, the specific stream callbacks are only provided for exceptional cases
	*/
	void (*m_onBufferedDataAvailable)(struct XsCallbackPlainC* thisPtr, struct XsDevice* dev, const struct XsDataPacket* packet);

	/*! \brief Called when XDA has detected a change in the connectivity state of a device
		\param dev The device that initiated the callback.
		\param newState The new connectivity state
	*/
	void (*m_onConnectivityChanged)(struct XsCallbackPlainC* thisPtr, struct XsDevice* dev, XsConnectivityState newState);

	/*! \brief Called when an information request has resulted in a response
		\details When the information request has completed, the data can be retrieved through the usual
		functions. Ie. when a requestBatteryLevel() resulted in an onInfoResponse(.., XIR_BatteryLevel),
		the XsDevice::batteryLevel function will return the received battery level.
		\param dev The device that initiated the callback.
		\param request The type of request that was completed
	*/
	void (*m_onInfoResponse)(struct XsCallbackPlainC* thisPtr, struct XsDevice* dev, XsInfoRequest request);


	/*! \brief Called when an error has occurred while handling incoming data
		\param dev The device that generated the error message
		\param error The error code that specifies exactly what problem occurred
	*/
	void (*m_onError)(struct XsCallbackPlainC* thisPtr, struct XsDevice* dev, XsResultValue error);

	/*! \brief Called when a non data, non reply message has been received
		\param dev The device that generated the error message
		\param message The message that has been received
	*/
	void (*m_onNonDataMessage)(struct XsCallbackPlainC* thisPtr, struct XsDevice* dev, struct XsMessage const * message);

	/*! \brief Called just after a message is detected in raw data from the device.
		\param dev The device that sent the message
		\param type The protocol type that detected a message
		\param rawMessage The raw message that has been detected
		\note This message can be invalid, since it wasn't checked for sanity
	*/
	void(*m_onMessageDetected)(struct XsCallbackPlainC* thisPtr, struct XsDevice* dev, XsProtocolType type, struct XsByteArray const * rawMessage);

	/*! \brief Called just after a valid message (after parsing) is received from the device.
		\param dev The device that sent the message
		\param message The message that has been received
	*/
	void (*m_onMessageReceivedFromDevice)(struct XsCallbackPlainC* thisPtr, struct XsDevice* dev, struct XsMessage const * message);

	/*! \brief Called just after a message is sent to the device.
		\param dev The device that will receive the message
		\param message The message that will be sent
	*/
	void (*m_onMessageSentToDevice)(struct XsCallbackPlainC* thisPtr, struct XsDevice* dev, struct XsMessage const * message);

	/*! \brief Called when new data has been received for devices connected to the same main device. When processing on PC is enabled, this callback occurs after processing has been done and so the packet will contain the processing output.
		\details This callback is for the Live stream, so there may be gaps in the data, but it will always contain the latest data. This stream will interpolate missing data to provide the fastest data output.
		\param devs A managed array of devices for which data is available. The array will be cleaned up by XDA after the callback returns.
		\param packets A plain array of pointers to data packets matching the order of \a devs. The array will be cleaned up by XDA after the callback returns.
		\sa m_onLiveDataAvailable \sa m_onBufferedDataAvailable \sa m_onAllBufferedDataAvailable \sa m_onDataAvailable
		\note For most applications, attaching to the m_onAllDataAvailable callback is sufficient, the specific stream callbacks are only provided for exceptional cases
	*/
	void (*m_onAllLiveDataAvailable)(struct XsCallbackPlainC* thisPtr, struct XsDevicePtrArray* devs, const struct XsDataPacketPtrArray* packets);

	/*! \brief Called when new data has been received for devices connected to the same main device. When processing on PC is enabled, this callback occurs after processing has been done and so the packet will contain the processing output.
		\details This callback is for the Buffered stream, which will attempt to retransmit missed data when in Recording mode. So there should be no gaps (when recording), but the data arrival may be delayed a bit. When not recording, the behaviour is identical to the Live stream. This stream will only interpolate data that is knon to be permanently unavailable.
		\param devs A managed array of devices for which data is available. The array will be cleaned up by XDA after the callback returns.
		\param packets A plain array of pointers to data packets matching the order of \a devs. The array will be cleaned up by XDA after the callback returns.
		\sa m_onAllLiveDataAvailable \sa m_onLiveDataAvailable \sa m_onBufferedDataAvailable \sa m_onDataAvailable
		\note For most applications, attaching to the m_onAllDataAvailable callback is sufficient, the specific stream callbacks are only provided for exceptional cases
	*/
	void (*m_onAllBufferedDataAvailable)(struct XsCallbackPlainC* thisPtr, struct XsDevicePtrArray* devs, const struct XsDataPacketPtrArray* packets);

	/*! \brief Called when XDA detects that data is forever unavailable
		\details This differs from onMissedPackets, since missed packets may be retransmitted, while unavailable
		data can no longer be retransmitted.
		\param dev The device that initiated the callback.
		\param packetId The sample counter / packet identifier of the unavailable sample
	*/
	void (*m_onDataUnavailable)(struct XsCallbackPlainC* thisPtr, struct XsDevice* dev, int64_t packetId);

	/*! \brief Called when new data has been received from a device or read from a file. When processing on PC is enabled, this callback occurs after processing has been done and so the packet will contain the processing output.
		\details This callback is dynamically attached to either the Live or the Buffered stream and behaves accordingly. When reading from a device, it will be attached to the Live stream, while it will be attached to the Buffered stream when reading from a file.
		\param dev The device that initiated the callback.
		\param packet The data packet that has been received (and processed). This may be 0 when the callback originates from a non-device, such as the XsDataBundler.
		\sa m_onAllLiveDataAvailable \sa m_onLiveDataAvailable \sa m_onAllBufferedDataAvailable \sa m_onBufferedDataAvailable \sa m_onAllDataAvailable
	*/
	void (*m_onDataAvailable)(struct XsCallbackPlainC* thisPtr, struct XsDevice* dev, const struct XsDataPacket* packet);

	/*! \brief Called when new data has been received for devices connected to the same main device. When processing on PC is enabled, this callback occurs after processing has been done and so the packet will contain the processing output.
		\details This callback is dynamically attached to either the Live or the Buffered stream and behaves accordingly. When reading from a device, it will be attached to the Live stream, while it will be attached to the Buffered stream when reading from a file.
		\param devs A managed array of devices for which data is available. The array will be cleaned up by XDA after the callback returns.
		\param packets A plain array of pointers to data packets matching the order of \a devs. The array will be cleaned up by XDA after the callback returns.
		\sa m_onAllLiveDataAvailable \sa m_onAllBufferedDataAvailable \sa m_onDataAvailable
	*/
	void (*m_onAllDataAvailable)(struct XsCallbackPlainC* thisPtr, struct XsDevicePtrArray* devs, const struct XsDataPacketPtrArray* packets);

	/*! \brief Called when new data has been received from a device in a recording state or read from a file. When processing on PC is enabled, this callback occurs after processing has been done and so the packet will contain the processing output.
		\details This callback is attached to the Buffered stream and behaves accordingly, it will only be called with data that should be recorded.
		\param dev The device that initiated the callback.
		\param packet The data packet that has been received (and processed). This may be 0 when the callback originates from a non-device, such as the XsDataBundler.
		\sa m_onAllLiveDataAvailable \sa m_onLiveDataAvailable \sa m_onAllBufferedDataAvailable \sa m_onBufferedDataAvailable \sa m_onAllDataAvailable
	*/
	void (*m_onRecordedDataAvailable)(struct XsCallbackPlainC* thisPtr, struct XsDevice* dev, const struct XsDataPacket* packet);

	/*! \brief Called when new data has been received for devices connected to the same main device in a recording state or read from file. When processing on PC is enabled, this callback occurs after processing has been done and so the packet will contain the processing output.
		\details This callback is attached to the Buffered stream and behaves accordingly, it will only be called with data that should be recorded.
		\param devs A managed array of devices for which data is available. The array will be cleaned up by XDA after the callback returns.
		\param packets A plain array of pointers to data packets matching the order of \a devs. The array will be cleaned up by XDA after the callback returns.
		\sa m_onAllLiveDataAvailable \sa m_onAllBufferedDataAvailable \sa m_onDataAvailable
	*/
	void (*m_onAllRecordedDataAvailable)(struct XsCallbackPlainC* thisPtr, struct XsDevicePtrArray* devs, const struct XsDataPacketPtrArray* packets);

	/*! \brief Called when XDA needs to send raw data to a device connected using a custom communication channel.
		\param channelId The user provided identifier associated with the custom channel.
		\param data The array of bytes that must be forwarded to the device
	*/
	void (*m_onTransmissionRequest)(struct XsCallbackPlainC* thisPtr, int channelId, const struct XsByteArray* data);

	/*! \brief Called when restore communication is completed, stopped or an error occured.
		\param portName A name of port to which device is attached.
		\param result The result code.
	*/
	void (*m_onRestoreCommunication)(struct XsCallbackPlainC* thisPtr, const struct XsString* portName, XsResultValue result);

//! @}
#ifdef __cplusplus
	// Make sure that this struct is not used in C++ (except as base class for XsCallback)
	friend class XsCallback;
protected:
	XsCallbackPlainC() {}
	~XsCallbackPlainC() throw() {}
private:
	XsCallbackPlainC(XsCallbackPlainC const &);
	XsCallbackPlainC& operator = (XsCallbackPlainC const &);
#endif

} XsCallbackPlainC;

#endif
