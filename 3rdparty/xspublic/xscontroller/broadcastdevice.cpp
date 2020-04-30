
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

#include "broadcastdevice.h"
#include "xscontrol_def.h"
#include <xstypes/xssyncsetting.h>
#include <xstypes/xsoutputconfigurationarray.h>

using namespace xsens;

/*! \class BroadcastDevice
	\brief Intimately entangled class with XsControl that allows broadcasting to all main devices
	\details This class is intended to prevent interface duplication from XsDevice to XsControl.
	When broadcasting, the user can use XsControl::findDevice(XsDeviceId::broadcast()) to get the
	BroadcastDevice and use it as any other device.

	The BroadcastDevice is not actually part of the device hierarchy, but it pretends to be the
	parent of all mian devices is XsControl.

	The functions that are broadcast make use of the BroadcastForwardFunc class.
*/

/*! \brief Pure virtual base class for N-argument specific function forwarding
	\details The class provides the necessary locking to safely use XsControl.
*/
class BroadcastForwardFunc {
public:
	BroadcastForwardFunc(BroadcastDevice* bc) : m_broadcaster(bc) {}
	virtual ~BroadcastForwardFunc() {}
	bool operator()()
	{
		LockReadWrite portLock(&m_broadcaster->m_control->m_portMutex);
		portLock.lock(true);

		std::vector<XsDevice*>& ch = m_broadcaster->m_control->m_deviceList;

		bool ok = true;
		XsResultValue res = (ok?XRV_OK:XRV_NOFILEORPORTOPEN);
		for (std::vector<XsDevice*>::iterator it = ch.begin(); it != ch.end(); ++it)
		{
			if (!call(*it))
				ok = false;
		}
		m_broadcaster->m_control->m_lastResult = res;
		return ok;
	}

	virtual bool call(XsDevice* device) = 0;

	BroadcastDevice* const m_broadcaster;
};

//! \brief 0-argument const forwarding function class
class ForwardConstFunc : public BroadcastForwardFunc {
public:
	typedef bool (XsDevice::*FuncType)() const;
	inline ForwardConstFunc (BroadcastDevice* bc, FuncType func)
		: BroadcastForwardFunc(bc), m_func(func) {}
	virtual bool call(XsDevice* device) { return (device->*m_func)(); }
private:
	FuncType m_func;
};

//! \brief 1-argument const forwarding function class
template <typename Arg1>
class ForwardConstFunc1Arg : public BroadcastForwardFunc {
public:
	typedef bool (XsDevice::*FuncType)(Arg1) const;
	inline ForwardConstFunc1Arg (BroadcastDevice* bc, FuncType func, Arg1 arg1)
		: BroadcastForwardFunc(bc), m_func(func), m_arg1(arg1) {}
	virtual bool call(XsDevice* device) { return (device->*m_func)(m_arg1); }
private:
	FuncType m_func;
	Arg1 m_arg1;
};

//! \brief 0-argument forwarding function class
class BroadcastForwardFunc0Arg : public BroadcastForwardFunc {
public:
	typedef bool (XsDevice::*FuncType)();
	inline BroadcastForwardFunc0Arg (BroadcastDevice* bc, FuncType func)
		: BroadcastForwardFunc(bc), m_func(func) {}
	virtual bool call(XsDevice* device) { return (device->*m_func)(); }
private:
	FuncType m_func;
};

//! \brief 1-argument forwarding function class
template <typename Arg1>
class BroadcastForwardFunc1Arg : public BroadcastForwardFunc {
public:
	typedef bool (XsDevice::*FuncType)(Arg1);
	inline BroadcastForwardFunc1Arg (BroadcastDevice* bc, FuncType func, Arg1 arg1)
		: BroadcastForwardFunc(bc), m_func(func), m_arg1(arg1) {}
	virtual bool call(XsDevice* device) { return (device->*m_func)(m_arg1); }
private:
	FuncType m_func;
	Arg1 m_arg1;
};

//! \brief 2-argument forwarding function class
template <typename Arg1, typename Arg2>
class BroadcastForwardFunc2Arg : public BroadcastForwardFunc {
public:
	typedef bool (XsDevice::*FuncType)(Arg1, Arg2);
	inline BroadcastForwardFunc2Arg (BroadcastDevice* bc, FuncType func, Arg1 arg1, Arg2 arg2)
		: BroadcastForwardFunc(bc), m_func(func), m_arg1(arg1), m_arg2(arg2) {}
	virtual bool call(XsDevice* device) { return (device->*m_func)(m_arg1, m_arg2); }
private:
	FuncType m_func;
	Arg1 m_arg1;
	Arg2 m_arg2;
};

//! \brief 3-argument forwarding function class
template <typename Arg1, typename Arg2, typename Arg3>
class BroadcastForwardFunc3Arg : public BroadcastForwardFunc {
public:
	typedef bool (XsDevice::*FuncType)(Arg1, Arg2, Arg3);
	inline BroadcastForwardFunc3Arg (BroadcastDevice* bc, FuncType func, Arg1 arg1, Arg2 arg2, Arg3 arg3)
		: BroadcastForwardFunc(bc), m_func(func), m_arg1(arg1), m_arg2(arg2), m_arg3(arg3) {}
	virtual bool call(XsDevice* device) { return (device->*m_func)(m_arg1, m_arg2, m_arg3); }
private:
	FuncType m_func;
	Arg1 m_arg1;
	Arg2 m_arg2;
	Arg3 m_arg3;
};

//! \brief 0-argument forwarding function class, void return
class BroadcastForwardFunc0ArgVoid : public BroadcastForwardFunc {
public:
	typedef void (XsDevice::*FuncType)();
	inline BroadcastForwardFunc0ArgVoid (BroadcastDevice* bc, FuncType func)
		: BroadcastForwardFunc(bc), m_func(func) {}
	virtual bool call(XsDevice* device)
	{
		(device->*m_func)();
		return true;
	}
private:
	FuncType m_func;
};

//! \brief 1-argument forwarding function class, void return
template <typename Arg1>
class BroadcastForwardFunc1ArgVoid : public BroadcastForwardFunc {
public:
	typedef void (XsDevice::*FuncType)(Arg1);
	inline BroadcastForwardFunc1ArgVoid (BroadcastDevice* bc, FuncType func, Arg1 arg1)
		: BroadcastForwardFunc(bc), m_func(func), m_arg1(arg1) {}
	virtual bool call(XsDevice* device)
	{
		(device->*m_func)(m_arg1);
		return true;
	}
private:
	FuncType m_func;
	Arg1 m_arg1;
};

//! \brief 2-argument forwarding function class, void return
template <typename Arg1, typename Arg2>
class BroadcastForwardFunc2ArgVoid : public BroadcastForwardFunc {
public:
	typedef void (XsDevice::*FuncType)(Arg1, Arg2);
	inline BroadcastForwardFunc2ArgVoid (BroadcastDevice* bc, FuncType func, Arg1 arg1, Arg2 arg2)
		: BroadcastForwardFunc(bc), m_func(func), m_arg1(arg1), m_arg2(arg2) {}
	virtual bool call(XsDevice* device)
	{
		(device->*m_func)(m_arg1, m_arg2);
		return true;
	}
private:
	FuncType m_func;
	Arg1 m_arg1;
	Arg2 m_arg2;
};

/*! \brief Constructor, sets up the XsControl reference
*/
BroadcastDevice::BroadcastDevice(XsControl* control_)
: XsDevice(XsDeviceId(0))
, m_control(control_)
{
}

/*! \brief Destructor, no special actions taken
*/
BroadcastDevice::~BroadcastDevice()
{
	setTerminationPrepared(true);
	m_control = NULL;
}

/*! \brief Return the children of this device, actually returns the main devices in the XsControl object
*/
inline std::vector<XsDevice*> BroadcastDevice::children() const
{
	return m_control->m_deviceList;
}

//! required function to prevent pure-virtualness
bool BroadcastDevice::initialize()
{
	return false;
}
//! required function to prevent pure-virtualness
std::vector<int> BroadcastDevice::supportedUpdateRates(XsDataIdentifier dataType) const
{
	(void)dataType;
	return std::vector<int>();
}
//! required function to prevent pure-virtualness
XsString BroadcastDevice::productCode() const
{
	return XsString("Broadcaster");
}
//! required function to prevent pure-virtualness
XsVersion BroadcastDevice::hardwareVersion() const
{
	return XsVersion();
}

/*!	\brief Sets the object alignment of a device to the given \a matrix.
	\param matrix The matrix to set
	\details If an error is encountered, the lastResult value is set and the function returns false.
	\returns True if successful
	\sa objectAlignmentMatrix(), headingOffset(), setHeadingOffset(), lastResult()
*/
bool BroadcastDevice::setObjectAlignment(const XsMatrix &matrix)
{
	return BroadcastForwardFunc1Arg<const XsMatrix&>(this, &XsDevice::setObjectAlignment, matrix)();
}

/*! \brief Start recording data
*/
bool BroadcastDevice::startRecording()
{
	JLDEBUGG("");
	bool rv = BroadcastForwardFunc0Arg(this, &XsDevice::startRecording)();
	m_control->updateRecordingState();
	return rv;
}

/*! \brief Stop recording data
*/
bool BroadcastDevice::stopRecording()
{
	JLDEBUGG("");
	bool rv = BroadcastForwardFunc0Arg(this, &XsDevice::stopRecording)();
	m_control->updateRecordingState();
	return rv;
}

/*! \brief Change the serial baudrate to baudrate
*/
bool BroadcastDevice::setSerialBaudRate(XsBaudRate baudrate)
{
	return BroadcastForwardFunc1Arg<XsBaudRate>(this, &XsDevice::setSerialBaudRate, baudrate)();
}

bool BroadcastDevice::loadLogFile()
{
	return BroadcastForwardFunc0Arg(this, &XsDevice::loadLogFile)();
}

bool BroadcastDevice::closeLogFile()
{
	return BroadcastForwardFunc0Arg(this, &XsDevice::closeLogFile)();
}

bool BroadcastDevice::setNoRotation(uint16_t duration)
{
	return BroadcastForwardFunc1Arg<uint16_t>(this, &XsDevice::setNoRotation, duration)();
}

bool BroadcastDevice::requestBatteryLevel()
{
	return BroadcastForwardFunc0Arg(this, &XsDevice::requestBatteryLevel)();
}

XsTimeStamp BroadcastDevice::batteryLevelTime()
{
	return XsDevice::batteryLevelTime();
}

bool BroadcastDevice::abortFlushing()
{
	return BroadcastForwardFunc0Arg(this, &XsDevice::abortFlushing)();
}

bool BroadcastDevice::isMeasuring() const
{
	return ForwardConstFunc(const_cast<BroadcastDevice* const>(this), &XsDevice::isMeasuring)();
}

bool BroadcastDevice::isRecording() const
{
	return ForwardConstFunc(const_cast<BroadcastDevice* const>(this), &XsDevice::isRecording)();
}

bool BroadcastDevice::isReadingFromFile() const
{
	return ForwardConstFunc(const_cast<BroadcastDevice* const>(this), &XsDevice::isReadingFromFile)();
}

bool BroadcastDevice::gotoConfig()
{
	// this one needs to go through the list in reverse order in case of multiple devices
	// for synchronization reasons
	std::vector<XsDevice*>& ch = m_control->m_deviceList;

	bool ok = true;
	XsResultValue res = (ok?XRV_OK:XRV_NOFILEORPORTOPEN);
	for (std::vector<XsDevice*>::reverse_iterator it = ch.rbegin(); it != ch.rend(); ++it)
	{
		if (!(*it)->gotoConfig())
			ok = false;
	}
	m_control->m_lastResult = res;
	return ok;
}

bool BroadcastDevice::gotoMeasurement()
{
	if (!BroadcastForwardFunc0Arg(this, &XsDevice::gotoMeasurement)())
	{
		gotoConfig();
		return false;
	}
	return true;
}

bool BroadcastDevice::setSyncSettings(const XsSyncSettingArray& s)
{
	return BroadcastForwardFunc1Arg<const XsSyncSettingArray&>(this, &XsDevice::setSyncSettings, s)();
}

bool BroadcastDevice::setHeadingOffset(double offset)
{
	return BroadcastForwardFunc1Arg<double>(this, &XsDevice::setHeadingOffset, offset)();
}

bool BroadcastDevice::setLocationId(int id)
{
	return BroadcastForwardFunc1Arg<int>(this, &XsDevice::setLocationId, id)();
}

bool BroadcastDevice::resetOrientation(XsResetMethod resetmethod)
{
	return BroadcastForwardFunc1Arg<XsResetMethod>(this, &XsDevice::resetOrientation, resetmethod)();
}

bool BroadcastDevice::restoreFactoryDefaults()
{
	return BroadcastForwardFunc0Arg(this, &XsDevice::restoreFactoryDefaults)();
}

bool BroadcastDevice::resetLogFileReadPosition()
{
	return BroadcastForwardFunc0Arg(this, &XsDevice::resetLogFileReadPosition)();
}

bool BroadcastDevice::reset(bool skipDeviceIdCheck)
{
	return BroadcastForwardFunc1Arg<bool>(this, &XsDevice::reset, skipDeviceIdCheck)();
}

bool BroadcastDevice::setTransportMode(bool transportModeEnabled)
{
	return BroadcastForwardFunc1Arg<bool>(this, &XsDevice::setTransportMode, transportModeEnabled)();
}

bool BroadcastDevice::updateCachedDeviceInformation()
{
	return BroadcastForwardFunc0Arg(this, &XsDevice::updateCachedDeviceInformation)();
}

bool BroadcastDevice::setXdaFilterProfile(int profileType)
{
	return BroadcastForwardFunc1Arg<int>(this, &XsDevice::setXdaFilterProfile, profileType)();
}

bool BroadcastDevice::setXdaFilterProfile(XsString const& profileType)
{
	return BroadcastForwardFunc1Arg<XsString const&>(this, &XsDevice::setXdaFilterProfile, profileType)();
}

bool BroadcastDevice::setOnboardFilterProfile(int profileType)
{
	return BroadcastForwardFunc1Arg<int>(this, &XsDevice::setOnboardFilterProfile, profileType)();
}

bool BroadcastDevice::setOnboardFilterProfile(XsString const& profileType)
{
	return BroadcastForwardFunc1Arg<XsString const&>(this, &XsDevice::setOnboardFilterProfile, profileType)();
}

bool BroadcastDevice::setGravityMagnitude(double mag)
{
	return BroadcastForwardFunc1Arg<double>(this, &XsDevice::setGravityMagnitude, mag)();
}

bool BroadcastDevice::storeFilterState()
{
	return BroadcastForwardFunc0Arg(this, &XsDevice::storeFilterState)();
}

bool BroadcastDevice::setInitialPositionLLA(const XsVector& lla)
{
	return BroadcastForwardFunc1Arg<const XsVector&>(this, &XsDevice::setInitialPositionLLA, lla)();
}

void BroadcastDevice::setOptions(XsOption enable, XsOption disable)
{
	BroadcastForwardFunc2ArgVoid<XsOption, XsOption>(this, &XsDevice::setOptions, enable, disable)();
}

void BroadcastDevice::flushInputBuffers()
{
	BroadcastForwardFunc0ArgVoid(this, &XsDevice::flushInputBuffers)();
}
