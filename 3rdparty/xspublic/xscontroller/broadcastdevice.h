
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

#ifndef BROADCASTDEVICE_H
#define BROADCASTDEVICE_H

#include "xsdevice_def.h"
struct XsControl;

class BroadcastDevice : public XsDevice {
public:
	explicit BroadcastDevice(XsControl* control);
	~BroadcastDevice() override;

	std::vector<XsDevice*> children() const;

	bool initialize() override;

	std::vector<int> supportedUpdateRates(XsDataIdentifier dataType) const override;
	XsString productCode() const override;
	XsVersion hardwareVersion() const override;

	bool startRecording() override;
	bool stopRecording() override;

	bool isMeasuring() const override;
	bool isRecording() const override;
	bool isReadingFromFile() const override;

	bool setSerialBaudRate(XsBaudRate baudrate) override;
	bool setSyncSettings(const XsSyncSettingArray& s) override;
	bool gotoMeasurement() override;
	bool gotoConfig() override;
	bool resetOrientation(XsResetMethod resetMethod) override;
	bool restoreFactoryDefaults() override;
	bool reset(bool skipDeviceIdCheck = false) override;
	bool loadLogFile() override;
	bool closeLogFile() override;
	//bool gotoOperational() override;
	bool abortFlushing() override;
	bool resetLogFileReadPosition() override;
	bool updateCachedDeviceInformation() override;
	bool setHeadingOffset(double offset) override;
	bool setLocationId(int id) override;
	bool setObjectAlignment(const XsMatrix &matrix) override;
	bool setGravityMagnitude(double mag) override;
	bool setXdaFilterProfile(int profileType) override;
	bool setXdaFilterProfile(XsString const& profileType) override;
	bool setOnboardFilterProfile(int profileType) override;
	bool setOnboardFilterProfile(XsString const& profileType) override;
	bool setNoRotation(uint16_t duration) override;
	bool setInitialPositionLLA(const XsVector& lla) override;
	bool storeFilterState() override;
	bool requestBatteryLevel() override;
	XsTimeStamp batteryLevelTime() override;
	bool setTransportMode(bool transportModeEnabled) override;
	void setOptions(XsOption enable, XsOption disable) override;
	void flushInputBuffers() override;

private:
	friend class BroadcastForwardFunc;
	XsControl* m_control;
};

#endif
