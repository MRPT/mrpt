
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

#ifndef MTDEVICE_H
#define MTDEVICE_H

#include "xsdevice_def.h"
#include <xstypes/xsstringarray.h>
#include <xstypes/xsfilterprofilearray.h>

struct XsFilterProfile;

namespace xsens {
	class Emts5Public;
}

/*! \class MtDevice
	\brief MT device base class
*/

class MtDevice : public XsDeviceEx
{
public:
	MtDevice();
	virtual ~MtDevice();

	bool initialize() override;

	bool isMotionTracker() const override;
	int updateRateForDataIdentifier(XsDataIdentifier dataType) const override;

	uint16_t stringOutputType() const override;
	uint16_t stringSamplePeriod() const override;
	uint16_t stringSkipFactor() const override;

	XsDeviceOptionFlag deviceOptionFlags() const override;

	XsGnssPlatform gnssPlatform() const override;

	XsOutputConfigurationArray outputConfiguration() const override;

	double headingOffset() const;

	virtual bool canDoOrientationResetInFirmware(XsResetMethod method);
	virtual bool scheduleOrientationReset(XsResetMethod method);
	virtual bool storeAlignmentMatrix();

	virtual bool setLocationId(int id);
	int locationId() const;

	XsString productCode() const;

	XsBaudRate serialBaudRate() const override;

	bool reinitialize();

	XsFilterProfile onboardFilterProfile() const override;
	bool setOnboardFilterProfile(int profileType) override;
	bool setOnboardFilterProfile(XsString const& profileType) override;

	XsVersion hardwareVersion() const;

	XsFilterProfileArray availableOnboardFilterProfiles() const override;

	bool resetLogFileReadPosition() override;

	bool restoreFactoryDefaults();

	double accelerometerRange() const;
	double gyroscopeRange() const;

	void writeDeviceSettingsToFile() override;

	bool setNoRotation(uint16_t duration);

	XsVector initialPositionLLA() const override;
	bool setInitialPositionLLA(const XsVector& lla) override;

	XsErrorMode errorMode() const;
	bool setErrorMode(XsErrorMode errorMode);

	uint16_t rs485TransmissionDelay() const;
	bool setRs485TransmissionDelay(uint16_t delay);

	XsSelfTestResult runSelfTest();

	bool requestData();
	bool storeFilterState() override;

	static int calcFrequency(int baseFrequency, uint16_t skipFactor);

	bool messageLooksSane(const XsMessage &msg) const;
	uint32_t supportedStatusFlags() const override;

protected:
	explicit MtDevice(Communicator* comm);
	explicit MtDevice(MtContainer *, const XsDeviceId &);

	virtual void updateFilterProfiles();

	XsFilterProfileArray readFilterProfilesFromDevice() const;
	virtual void fetchAvailableHardwareScenarios();

	uint32_t syncTicksToUs(uint32_t ticks) const;
	uint32_t usToSyncTicks(uint32_t us) const;

	//! A vector of hardware filter profiles
	XsFilterProfileArray m_hardwareFilterProfiles;

protected:

	//! \brief A hardware filter profile
	XsFilterProfile m_hardwareFilterProfile;
};

#ifndef XDA_PRIVATE_BUILD

/*! \class MtDeviceEx
	\brief An abstract struct of MT device
*/
struct MtDeviceEx : public MtDevice
{
protected:

	//!	Construct a device using \a comm for communication
	explicit MtDeviceEx(Communicator* comm) : MtDevice(comm) {}

	//! Construct a device with device id \a childDeviceId for master \a master
	explicit MtDeviceEx(MtContainer *master, const XsDeviceId &childDeviceId) : MtDevice(master, childDeviceId) {}
};
#else
#include "mtdeviceex.h"
#endif

#endif
