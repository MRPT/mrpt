
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

#ifndef XSCONTROL_DEF_H
#define XSCONTROL_DEF_H

#include "xsdef.h"
#include <vector>
#include "xscallback.h"
#include <atomic>

#include <xstypes/xsdeviceid.h>
#include <xstypes/xsfilepos.h>
#include <xstypes/xsbaud.h>
#include <xstypes/xsresultvalue.h>
#include <xstypes/xsmessage.h>
#include <xscommon/xsens_mutex.h>
#include <xstypes/xsresetmethod.h>
#include "devicefactory.h"
#include "callbackmanagerxda.h"
#include <xstypes/xsvector3.h>
#include "lastresultmanager.h"
#include <xstypes/xsoption.h>

struct XSNOEXPORT Communicator;
class XSNOEXPORT BroadcastDevice;
class XSNOEXPORT XdaCommunicatorFactory;
class XSNOEXPORT ProxyCommunicator;
class XSNOEXPORT RestoreCommunication;

struct XSNOEXPORT XsDevice;
class XSNOEXPORT EmtsManager;
class XSNOEXPORT NetworkScanner;

//AUTO namespace xstypes {
//AUTO+chdr struct PstdInt;
//AUTO+chdr struct XsTypeDefs;
//AUTO+chdr struct XsException;
struct XsString;
struct XsPortInfo;
struct XsPortInfoArray;
struct XsSyncSetting;
struct XsDataPacket;
struct XsMessage;
struct XsFilterProfile;

//AUTO enum XsResultValue;
//AUTO enum XsBaud;
//AUTO enum XsXbusMessageId;
//AUTO struct XsDeviceIdArray;
//AUTO struct XsIntArray;
//AUTO struct XsSyncSettingArray;
//AUTO typename XsDevice;
//AUTO enum XsFilePos;
//AUTO struct XsFilterProfileArray;
//AUTO enum XsOption;

//AUTO }

//AUTO namespace xscontroller {
struct XsDeviceConfiguration;
struct XsDevicePtrArray;
//AUTO }

#define XsensThreadReturn	XSENS_THREAD_RETURN	// for generator
#define XsensThreadParam	XSENS_THREAD_PARAM	// for generator

struct XsControl : public CallbackManagerXda, protected DeviceFactory::DeviceManager
{
public:
	XsControl();
	~XsControl();

	void flushInputBuffers();

	static XsString resultText(XsResultValue resultCode);

	void clearHardwareError();
	void close();

	bool openPort(const XsString& portname, XsBaudRate baudrate, uint32_t timeout = 0, bool detectRs485 = false);
	bool openPort(XsPortInfo& portinfo, uint32_t timeout = 0, bool detectRs485 = false);
	bool openPortWithCredentials(XsPortInfo& portinfo, XsString const& id, XsString const& key, uint32_t timeout = 0);
	bool openCustomPort(int channelId, uint32_t channelLatency, bool detectRs485 = false);
	virtual bool openImarPort_internal(const XsString& portname, XsBaudRate baudrate, int imarType, uint32_t timeout = 0);
#ifndef XSENS_NO_PORT_NUMBERS
	XSNOLINUXEXPORT bool openPort(int portNr, XsBaudRate baudrate, uint32_t timeout = 0, bool detectRs485 = false);
#endif
	void closePort(const XsString& portname);
	void closePort(const XsDeviceId& deviceId);
	void closePort(const XsPortInfo& portinfo);
	void closeCustomPort(int channelId);
#ifndef XSENS_NO_PORT_NUMBERS
	XSNOLINUXEXPORT void closePort (int portNr);
#endif
	void closePort(XsDevice* device);

	XsPortInfo customPortInfo(int channelId) const;

	bool openLogFile(const XsString& filename);

	XsResultValue lastResult() const;
	XsString lastResultText() const;
	XsResultValue lastHardwareError() const;
	XsDeviceId lastHardwareErrorDeviceId() const;

	int deviceCount() const;
	int mainDeviceCount() const;
	std::vector<XsDeviceId> mainDeviceIds() const;
	virtual int mtCount() const;
	virtual std::vector<XsDeviceId> mtDeviceIds() const;
	virtual std::vector<XsDeviceId> deviceIds() const;
	XsDevice* getDeviceFromLocationId(uint16_t locationId) const;
	XsDeviceId dockDeviceId(const XsDeviceId& deviceId) const;

	virtual bool isDeviceWireless(const XsDeviceId& deviceId) const;
	virtual bool isDeviceDocked(const XsDeviceId& deviceId) const;

	virtual bool loadFilterProfiles(const XsString& filename);

	XsOption enabledOptions() const;
	XsOption disabledOptions() const;
	void setOptions(XsOption enable, XsOption disable);
	void setOptionsForce(XsOption enabled);

	bool setInitialPositionLLA(const XsVector& lla);

	XsDevice* device(const XsDeviceId& deviceId) const;
	XsDevicePtrArray mainDevices() const;
	XsDevice* broadcast() const;

	void transmissionReceived(int channelId, const XsByteArray& data);
#ifdef DOXYGEN
	// Explicit inheritance for generator and doxygen
	void XSNOCOMEXPORT clearCallbackHandlers(bool chain = true);
	void XSNOCOMEXPORT addCallbackHandler(XsCallbackPlainC* cb, bool chain = true);
	void XSNOCOMEXPORT removeCallbackHandler(XsCallbackPlainC* cb, bool chain = true);
#endif

	// these are only required to allow using the lib the same way as to using the dll
	static XSNOEXPORT XsControl* construct();
	XSNOEXPORT void destruct()
	{ delete this; }

	virtual bool XSNOEXPORT finalizeOpenPort(Communicator *communicator, XsPortInfo &portinfo, uint32_t timeout, bool detectRs485);
	void gotoConfig();
	void gotoMeasurement();

	XsResultValue startRestoreCommunication(const XsString &portName);
	void stopRestoreCommunication();

protected:
	virtual XsDevice* XSNOCOMEXPORT addMasterDevice(Communicator* communicator);

#ifndef DOXYGEN
	XsControl(const XsControl& ref);
#endif

	//void gotoOperational(const XsDeviceId& stationId = XsDeviceId());

	//! Boolean variable for enabling/disabling the use of fake messages
	bool m_useFakeMessages;

	//! \see setSynchronousDataReport
	bool m_synchronousDataReport;

	//! This list contains device-information and cached data per device.
	std::vector<XsDevice*> m_deviceList;

	//! This map contains the proxy channels
	std::map<int, ProxyCommunicator*> m_proxyChannels;

	//! The last result of an operation
	mutable LastResultManager m_lastResult;

	//! Contains the last serious error reported by CMT3
	XsResultValue m_lastHwError;

	//! Contains the XsDevice ID of the device that caused the last hardware error
	XsDeviceId m_lastHwErrorDeviceId;

	////////////////////
	// thread management, multiple locks should always use this same order
	//! Controls access to the serial ports, also used to suspend the thread.
	mutable xsens::MutexReadWrite m_portMutex;
	//! Always held by the thread when it is running
	mutable xsens::Mutex m_runMutex;

	//! AwindaStationIndication of threads started or not
	volatile std::atomic_bool m_recording;

#ifndef SWIG
	//! The start of the thread
	XSNOEXPORT static XsensThreadReturn XSENS_THREAD_TYPE threadInit(XsensThreadParam);
	//! The start of the treadEx thread
	XSNOEXPORT static XsensThreadReturn XSENS_THREAD_TYPE threadExInit(XsensThreadParam);
#endif

	void updateRecordingState();

	XsDevice *findDevice(const XsDeviceId &deviceId) const;

	virtual void removeExistingDevice(XsDeviceId const & deviceId);

	//! Find the xs3 info of the given id
	Communicator* findXbusInterface(const XsDeviceId &deviceId) const;
	Communicator* findXbusInterface(const XsPortInfo &portInfo) const;
	Communicator* findXbusInterface(const XsString &portName) const;

	void closePortByIndex(uint32_t index);

	//! The broadcast device object
	BroadcastDevice* m_broadcaster;

	//! Contains all enable options
	XsOption m_optionsEnable;

	//! Contsins all disabled options
	XsOption m_optionsDisable;

	//! This vector contains the latitude, longitude and altitude
	XsVector3 m_latLonAlt;

	void setPersistentSettings(XsDevice* dev);

	//! The device factory object
	DeviceFactory *m_deviceFactory;

	//! The communicator factory object
	XdaCommunicatorFactory *m_communicatorFactory;

	//! The restore communication object
	RestoreCommunication *m_restoreCommunication;

/*! \cond XS_INTERNAL */
protected:
	friend class BroadcastDevice;
	friend class BroadcastForwardFunc;
/*! \endcond */ // XS_INTERNAL
};

#endif
