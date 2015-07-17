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
 *  Driver Interface
 *
 *  Copyright 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 * 
 */

#pragma once


#ifndef __cplusplus
#error "The RPlidar SDK requires a C++ compiler to be built"
#endif

namespace rp { namespace standalone{ namespace rplidar {

class RPlidarDriver {
public:
    enum {
        DEFAULT_TIMEOUT = 2000, //2000 ms
    };

    enum {
        DRIVER_TYPE_SERIALPORT = 0x0,
    };
public:
    /// Create an RPLIDAR Driver Instance
    /// This interface should be invoked first before any other operations
    ///
    /// \param drivertype the connection type used by the driver. 
    static RPlidarDriver * CreateDriver(_u32 drivertype = DRIVER_TYPE_SERIALPORT);

    /// Dispose the RPLIDAR Driver Instance specified by the drv parameter
    /// Applications should invoke this interface when the driver instance is no longer used in order to free memory
    static void DisposeDriver(RPlidarDriver * drv);


public:
    /// Open the specified serial port and connect to a target RPLIDAR device
    ///
    /// \param port_path     the device path of the serial port 
    ///        e.g. on Windows, it may be com3 or \\.\com10 
    ///             on Unix-Like OS, it may be /dev/ttyS1, /dev/ttyUSB2, etc
    ///
    /// \param baudrate      the baudrate used
    ///        For most RPLIDAR models, the baudrate should be set to 115200
    ///
    /// \param flag          other flags
    ///        Reserved for future use, always set to Zero
    virtual u_result connect(const char * port_path, _u32 baudrate, _u32 flag = 0) = 0;


    /// Disconnect with the RPLIDAR and close the serial port
    virtual void disconnect() = 0;

    /// Returns TRUE when the connection has been established
    virtual bool isConnected() = 0;

    /// Ask the RPLIDAR core system to reset it self
    /// The host system can use the Reset operation to help RPLIDAR escape the self-protection mode.
    ///
    //  \param timeout       The operation timeout value (in millisecond) for the serial port communication                     
    virtual u_result reset(_u32 timeout = DEFAULT_TIMEOUT) = 0;

    /// Retrieve the health status of the RPLIDAR
    /// The host system can use this operation to check whether RPLIDAR is in the self-protection mode.
    ///
    /// \param health        The health status info returned from the RPLIDAR
    ///
    /// \param timeout       The operation timeout value (in millisecond) for the serial port communication     
    virtual u_result getHealth(rplidar_response_device_health_t & health, _u32 timeout = DEFAULT_TIMEOUT) = 0;

    /// Get the device information of the RPLIDAR include the serial number, firmware version, device model etc.
    /// 
    /// \param info          The device information returned from the RPLIDAR
    ///
    /// \param timeout       The operation timeout value (in millisecond) for the serial port communication  
    virtual u_result getDeviceInfo(rplidar_response_device_info_t & info, _u32 timeout = DEFAULT_TIMEOUT) = 0;


    /// Calculate RPLIDAR's current scanning frequency from the given scan data
    /// Please refer to the application note doc for details
    /// Remark: the calculation will be incorrect if the specified scan data doesn't contain enough data
    ///
    /// \param nodebuffer    The buffer belongs to a 360degress scan data
    ///
    /// \param count         The number of sample nodes inside the given buffer
    ///
    //  \param frequency     The scanning frequency (in HZ) calcuated by the interface.
    virtual u_result getFrequency(rplidar_response_measurement_node_t * nodebuffer, size_t count, float & frequency) = 0;

    /// Ask the RPLIDAR core system to enter the scan mode
    /// A background thread will be created by the RPLIDAR driver to fetch the scan data continuously.
    /// User Application can use the grabScanData() interface to retrieved the scan data cached previous by this background thread.
    ///
    /// \param force         Force the core system to output scan data regardless whether the scanning motor is rotating or not.
    ///
    /// \param timeout       The operation timeout value (in millisecond) for the serial port communication 
    virtual u_result startScan(bool force = false, _u32 timeout = DEFAULT_TIMEOUT) = 0;


    /// Ask the RPLIDAR core system to stop the current scan operation and enter idle state. The background thread will be terminated
    ///
    /// \param timeout       The operation timeout value (in millisecond) for the serial port communication 
    virtual u_result stop(_u32 timeout = DEFAULT_TIMEOUT) = 0;


    /// Wait and grab a complete 0-360 degree scan data previously received. 
    /// The grabbed scan data returned by this interface always has the following charactistics:
    ///
    /// 1) The first node of the grabbed data array (nodebuffer[0]) must be the first sample of a scan, i.e. the start_bit == 1
    /// 2) All data nodes are belong to exactly ONE complete 360-degrees's scan
    /// 3) Note, the angle data in one scan may not be ascending. You can use API ascendScanData to reorder the nodebuffer.
    ///
    /// \param nodebuffer     Buffer provided by the caller application to store the scan data
    ///
    /// \param count          The caller must initialize this parameter to set the max data count of the provided buffer (in unit of rplidar_response_measurement_node_t).
    ///                       Once the interface returns, this parameter will store the actual received data count.
    ///
    /// \param timeout        Max duration allowed to wait for a complete scan data, nothing will be stored to the nodebuffer if a complete 360-degrees' scan data cannot to be ready timely.
    ///
    /// The interface will return RESULT_OPERATION_TIMEOUT to indicate that no complete 360-degrees' scan can be retrieved withing the given timeout duration. 
    ///
    /// \The caller application can set the timeout value to Zero(0) to make this interface always returns immediately to achieve non-block operation.
	virtual u_result grabScanData(rplidar_response_measurement_node_t * nodebuffer, size_t & count, _u32 timeout = DEFAULT_TIMEOUT) = 0;

    /// Ascending the scan data according to the angle value in the scan.
    ///
    /// \param nodebuffer     Buffer provided by the caller application to do the reorder. Should be retrived from the grabScanData
    ///
    /// \param count          The caller must initialize this parameter to set the max data count of the provided buffer (in unit of rplidar_response_measurement_node_t).
    ///                       Once the interface returns, this parameter will store the actual received data count.
    /// The interface will return RESULT_OPERATION_FAIL when all the scan data is invalid. 
    virtual u_result ascendScanData(rplidar_response_measurement_node_t * nodebuffer, size_t count) = 0;

	virtual u_result stopMotor() = 0;
	virtual u_result startMotor() = 0;
	
protected:
    RPlidarDriver() {}
    virtual ~RPlidarDriver() {}
};


}}}
