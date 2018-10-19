/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#pragma once

namespace rp::standalone::rplidar
{
class RPlidarDriverSerialImpl : public RPlidarDriver
{
   public:
	enum
	{
		MAX_SCAN_NODES = 2048,
	};

	enum
	{
		LEGACY_SAMPLE_DURATION = 476,
	};

	RPlidarDriverSerialImpl();
	~RPlidarDriverSerialImpl() override;

   public:
	u_result connect(const char* port_path, _u32 baudrate, _u32 flag) override;
	void disconnect() override;
	bool isConnected() override;

	u_result reset(_u32 timeout = DEFAULT_TIMEOUT) override;

	u_result getHealth(
		rplidar_response_device_health_t&,
		_u32 timeout = DEFAULT_TIMEOUT) override;
	u_result getDeviceInfo(
		rplidar_response_device_info_t&,
		_u32 timeout = DEFAULT_TIMEOUT) override;
	u_result getSampleDuration_uS(
		rplidar_response_sample_rate_t& rateInfo,
		_u32 timeout = DEFAULT_TIMEOUT) override;

	u_result setMotorPWM(_u16 pwm) override;
	u_result startMotor() override;
	u_result stopMotor() override;
	u_result checkMotorCtrlSupport(
		bool& support, _u32 timeout = DEFAULT_TIMEOUT) override;
	u_result getFrequency(
		bool inExpressMode, size_t count, float& frequency,
		bool& is4kmode) override;

	u_result startScan(
		bool force = false, bool autoExpressMode = true) override;
	u_result startScanNormal(
		bool force, _u32 timeout = DEFAULT_TIMEOUT) override;
	u_result startScanExpress(
		bool fixedAngle, _u32 timeout = DEFAULT_TIMEOUT) override;
	u_result checkExpressScanSupported(
		bool& support, _u32 timeout = DEFAULT_TIMEOUT) override;

	u_result stop(_u32 timeout = DEFAULT_TIMEOUT) override;
	u_result grabScanData(
		rplidar_response_measurement_node_t* nodebuffer, size_t& count,
		_u32 timeout = DEFAULT_TIMEOUT) override;
	u_result ascendScanData(
		rplidar_response_measurement_node_t* nodebuffer, size_t count) override;

   protected:
	u_result _waitNode(
		rplidar_response_measurement_node_t* node,
		_u32 timeout = DEFAULT_TIMEOUT);
	u_result _waitScanData(
		rplidar_response_measurement_node_t* nodebuffer, size_t& count,
		_u32 timeout = DEFAULT_TIMEOUT);
	u_result _cacheScanData();
	void _capsuleToNormal(
		const rplidar_response_capsule_measurement_nodes_t& capsule,
		rplidar_response_measurement_node_t* nodebuffer, size_t& nodeCount);
	u_result _waitCapsuledNode(
		rplidar_response_capsule_measurement_nodes_t& node,
		_u32 timeout = DEFAULT_TIMEOUT);
	u_result _cacheCapsuledScanData();
	u_result _sendCommand(
		_u8 cmd, const void* payload = nullptr, size_t payloadsize = 0);
	u_result _waitResponseHeader(
		rplidar_ans_header_t* header, _u32 timeout = DEFAULT_TIMEOUT);
	u_result _waitSampleRate(
		rplidar_response_sample_rate_t* res, _u32 timeout = DEFAULT_TIMEOUT);

	void _disableDataGrabbing();

	bool _isConnected{false};
	bool _isScanning{false};
	bool _isSupportingMotorCtrl{false};

	rp::hal::Locker _lock;
	rp::hal::Event _dataEvt;
	rp::hal::serial_rxtx* _rxtx;
	rplidar_response_measurement_node_t _cached_scan_node_buf[2048];
	size_t _cached_scan_node_count;

	_u16 _cached_sampleduration_std;
	_u16 _cached_sampleduration_express;

	rplidar_response_capsule_measurement_nodes_t _cached_previous_capsuledata;
	bool _is_previous_capsuledataRdy;

	rp::hal::Thread _cachethread;
};
}  // namespace rp::standalone::rplidar
