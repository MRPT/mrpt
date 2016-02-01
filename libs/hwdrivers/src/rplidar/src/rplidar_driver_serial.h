/*
 * Copyright (c) 2014, RoboPeak
 * All rights reserved.
 *
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
/*
 *  RoboPeak LIDAR System
 *  Serial based RPlidar Driver
 *
 *  Copyright 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 * 
 */

#pragma once

namespace rp { namespace standalone{ namespace rplidar {

class RPlidarDriverSerialImpl : public RPlidarDriver
{
public:

    enum {
        MAX_SCAN_NODES = 2048,
    };

    RPlidarDriverSerialImpl();
    virtual ~RPlidarDriverSerialImpl();

public:
    virtual u_result connect(const char * port_path, _u32 baudrate, _u32 flag);
    virtual void disconnect();
    virtual bool isConnected();

    virtual u_result reset(_u32 timeout = DEFAULT_TIMEOUT);

    virtual u_result getHealth(rplidar_response_device_health_t &, _u32 timeout = DEFAULT_TIMEOUT);
    virtual u_result getDeviceInfo(rplidar_response_device_info_t &, _u32 timeout = DEFAULT_TIMEOUT);
	virtual u_result getFrequency(rplidar_response_measurement_node_t * nodebuffer, size_t count, float & frequency);

    virtual u_result startScan(bool force, _u32 timeout = DEFAULT_TIMEOUT);
    virtual u_result stop(_u32 timeout = DEFAULT_TIMEOUT);
    virtual u_result grabScanData(rplidar_response_measurement_node_t * nodebuffer, size_t & count, _u32 timeout = DEFAULT_TIMEOUT);
    virtual u_result ascendScanData(rplidar_response_measurement_node_t * nodebuffer, size_t count);

	virtual u_result stopMotor();
	virtual u_result startMotor();
	
	
protected:
    u_result _waitNode(rplidar_response_measurement_node_t * node, _u32 timeout);
    u_result _waitScanData(rplidar_response_measurement_node_t * nodebuffer, size_t & count, _u32 timeout = DEFAULT_TIMEOUT);
	u_result _cacheScanData();
    u_result _sendCommand(_u8 cmd, const void * payload = NULL, size_t payloadsize = 0);
    u_result _waitResponseHeader(rplidar_ans_header_t * header, _u32 timeout);

    void     _disableDataGrabbing();

    bool     _isConnected;
    bool     _isScanning;

	rp::hal::Locker         _lock;
    rp::hal::Event          _dataEvt;
    rp::hal::serial_rxtx  * _rxtx;
    rplidar_response_measurement_node_t      _cached_scan_node_buf[2048];
    size_t                                   _cached_scan_node_count;
	rp::hal::Thread _cachethread;

    
};


}}}
