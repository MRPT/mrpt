/*
 *  RPLIDAR SDK
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2016 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
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

#include "sdkcommon.h"
#include "hal/abs_rxtx.h"
#include "hal/thread.h"
#include "hal/locker.h"
#include "hal/event.h"
#include "rplidar_driver_serial.h"

#ifndef min
#define min(a,b)            (((a) < (b)) ? (a) : (b))
#endif

namespace rp { namespace standalone{ namespace rplidar {


// Factory Impl
RPlidarDriver * RPlidarDriver::CreateDriver(_u32 drivertype)
{
    switch (drivertype) {
    case DRIVER_TYPE_SERIALPORT:
        return new RPlidarDriverSerialImpl();
    default:
        return NULL;
    }
}


void RPlidarDriver::DisposeDriver(RPlidarDriver * drv)
{
    delete drv;
}



// Serial Driver Impl

RPlidarDriverSerialImpl::RPlidarDriverSerialImpl() 
    : _isConnected(false)
    , _isScanning(false)
    , _isSupportingMotorCtrl(false)
{
    _rxtx = rp::hal::serial_rxtx::CreateRxTx();
    _cached_scan_node_count = 0;
    _cached_sampleduration_std = LEGACY_SAMPLE_DURATION;
    _cached_sampleduration_express = LEGACY_SAMPLE_DURATION;
}

RPlidarDriverSerialImpl::~RPlidarDriverSerialImpl()
{
    // force disconnection
    disconnect();

    rp::hal::serial_rxtx::ReleaseRxTx(_rxtx);
}

u_result RPlidarDriverSerialImpl::connect(const char * port_path, _u32 baudrate, _u32 flag)
{
    if (isConnected()) return RESULT_ALREADY_DONE;

    if (!_rxtx) return RESULT_INSUFFICIENT_MEMORY;

    {
        rp::hal::AutoLocker l(_lock);

        // establish the serial connection...
        if (!_rxtx->bind(port_path, baudrate)  ||  !_rxtx->open()) {
            return RESULT_INVALID_DATA;
        }

        _rxtx->flush(0);
    }

    _isConnected = true;

    checkMotorCtrlSupport(_isSupportingMotorCtrl);
    stopMotor();

    return RESULT_OK;
}

void RPlidarDriverSerialImpl::disconnect()
{
    if (!_isConnected) return ;
    stop();
    _rxtx->close();
}

bool RPlidarDriverSerialImpl::isConnected()
{
    return _isConnected;
}


u_result RPlidarDriverSerialImpl::reset(_u32 timeout)
{
    u_result ans;

    {
        rp::hal::AutoLocker l(_lock);

        if (IS_FAIL(ans = _sendCommand(RPLIDAR_CMD_RESET))) {
            return ans;
        }
    }
    return RESULT_OK;
}

u_result RPlidarDriverSerialImpl::getHealth(rplidar_response_device_health_t & healthinfo, _u32 timeout)
{
    u_result  ans;
    
    if (!isConnected()) return RESULT_OPERATION_FAIL;
    
    _disableDataGrabbing();

    {
        rp::hal::AutoLocker l(_lock);

        if (IS_FAIL(ans = _sendCommand(RPLIDAR_CMD_GET_DEVICE_HEALTH))) {
            return ans;
        }

        rplidar_ans_header_t response_header;
        if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
            return ans;
        }

        // verify whether we got a correct header
        if (response_header.type != RPLIDAR_ANS_TYPE_DEVHEALTH) {
            return RESULT_INVALID_DATA;
        }

        _u32 header_size = (response_header.size_q30_subtype & RPLIDAR_ANS_HEADER_SIZE_MASK);
        if ( header_size < sizeof(rplidar_response_device_health_t)) {
            return RESULT_INVALID_DATA;
        }

        if (_rxtx->waitfordata(header_size, timeout) != rp::hal::serial_rxtx::ANS_OK) {
            return RESULT_OPERATION_TIMEOUT;
        }
        _rxtx->recvdata(reinterpret_cast<_u8 *>(&healthinfo), sizeof(healthinfo));
    }
    return RESULT_OK;
}

u_result RPlidarDriverSerialImpl::getDeviceInfo(rplidar_response_device_info_t & info, _u32 timeout)
{
    u_result  ans;
    
    if (!isConnected()) return RESULT_OPERATION_FAIL;

    _disableDataGrabbing();

    {
        rp::hal::AutoLocker l(_lock);

        if (IS_FAIL(ans = _sendCommand(RPLIDAR_CMD_GET_DEVICE_INFO))) {
            return ans;
        }

        rplidar_ans_header_t response_header;
        if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
            return ans;
        }

        // verify whether we got a correct header
        if (response_header.type != RPLIDAR_ANS_TYPE_DEVINFO) {
            return RESULT_INVALID_DATA;
        }

        _u32 header_size = (response_header.size_q30_subtype & RPLIDAR_ANS_HEADER_SIZE_MASK);
        if (header_size < sizeof(rplidar_response_device_info_t)) {
            return RESULT_INVALID_DATA;
        }

        if (_rxtx->waitfordata(header_size, timeout) != rp::hal::serial_rxtx::ANS_OK) {
            return RESULT_OPERATION_TIMEOUT;
        }

        _rxtx->recvdata(reinterpret_cast<_u8 *>(&info), sizeof(info));
    }
    return RESULT_OK;
}

u_result RPlidarDriverSerialImpl::getFrequency(bool inExpressMode, size_t count, float & frequency, bool & is4kmode)
{
    _u16 sample_duration = inExpressMode?_cached_sampleduration_express:_cached_sampleduration_std;
    frequency = 1000000.0f/(count * sample_duration);

    if (sample_duration <= 277) {
        is4kmode = true;
    } else {
        is4kmode = false;
    }

	return RESULT_OK;
}

u_result RPlidarDriverSerialImpl::startScanNormal(bool force, _u32 timeout)
{
    u_result ans;
    if (!isConnected()) return RESULT_OPERATION_FAIL;
    if (_isScanning) return RESULT_ALREADY_DONE;

    stop(); //force the previous operation to stop

    {
        rp::hal::AutoLocker l(_lock);

        if (IS_FAIL(ans = _sendCommand(force?RPLIDAR_CMD_FORCE_SCAN:RPLIDAR_CMD_SCAN))) {
            return ans;
        }

        // waiting for confirmation
        rplidar_ans_header_t response_header;
        if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
            return ans;
        }

        // verify whether we got a correct header
        if (response_header.type != RPLIDAR_ANS_TYPE_MEASUREMENT) {
            return RESULT_INVALID_DATA;
        }

        _u32 header_size = (response_header.size_q30_subtype & RPLIDAR_ANS_HEADER_SIZE_MASK);
        if (header_size < sizeof(rplidar_response_measurement_node_t)) {
            return RESULT_INVALID_DATA;
        }

        _isScanning = true;
        _cachethread = CLASS_THREAD(RPlidarDriverSerialImpl, _cacheScanData);
        if (_cachethread.getHandle() == 0) {
            return RESULT_OPERATION_FAIL;
        }
    }
    return RESULT_OK;
}

u_result RPlidarDriverSerialImpl::checkExpressScanSupported(bool & support, _u32 timeout)
{
    rplidar_response_device_info_t devinfo;

    support = false;
    u_result ans = getDeviceInfo(devinfo, timeout);

    if (IS_FAIL(ans)) return ans;

    if (devinfo.firmware_version >= ((0x1<<8) | 17)) {
        support = true;
        rplidar_response_sample_rate_t sample_rate;
        getSampleDuration_uS(sample_rate);
        _cached_sampleduration_express = sample_rate.express_sample_duration_us;
        _cached_sampleduration_std = sample_rate.std_sample_duration_us;
    }

    return RESULT_OK;
}

u_result RPlidarDriverSerialImpl::startScanExpress(bool fixedAngle, _u32 timeout)
{
    u_result ans;
    if (!isConnected()) return RESULT_OPERATION_FAIL;
    if (_isScanning) return RESULT_ALREADY_DONE;

    stop(); //force the previous operation to stop

    {
        rp::hal::AutoLocker l(_lock);

        rplidar_payload_express_scan_t scanReq;
        scanReq.working_mode = (fixedAngle?RPLIDAR_EXPRESS_SCAN_MODE_FIXANGLE:RPLIDAR_EXPRESS_SCAN_MODE_NORMAL);
        scanReq.reserved = 0;

        if (IS_FAIL(ans = _sendCommand(RPLIDAR_CMD_EXPRESS_SCAN,&scanReq, sizeof(scanReq)))) {
            return ans;
        }

        // waiting for confirmation
        rplidar_ans_header_t response_header;
        if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
            return ans;
        }

        // verify whether we got a correct header
        if (response_header.type != RPLIDAR_ANS_TYPE_MEASUREMENT_CAPSULED) {
            return RESULT_INVALID_DATA;
        }

        _u32 header_size = (response_header.size_q30_subtype & RPLIDAR_ANS_HEADER_SIZE_MASK);
        if (header_size < sizeof(rplidar_response_capsule_measurement_nodes_t)) {
            return RESULT_INVALID_DATA;
        }

        _isScanning = true;
        _cachethread = CLASS_THREAD(RPlidarDriverSerialImpl, _cacheCapsuledScanData);
        if (_cachethread.getHandle() == 0) {
            return RESULT_OPERATION_FAIL;
        }
    }
    return RESULT_OK;
}


u_result RPlidarDriverSerialImpl::startScan(bool force, bool autoExpressMode)
{
    bool isExpressModeSupported;
    u_result ans;

    if (autoExpressMode) {
        ans = checkExpressScanSupported(isExpressModeSupported);

        if (IS_FAIL(ans)) return ans;
    
        if (isExpressModeSupported) {
            return startScanExpress(false);
        }
    }

    return startScanNormal(force);
}

u_result RPlidarDriverSerialImpl::stop(_u32 timeout)
{
    u_result ans;
    _disableDataGrabbing();

    {
        rp::hal::AutoLocker l(_lock);

        if (IS_FAIL(ans = _sendCommand(RPLIDAR_CMD_STOP))) {
            return ans;
        }
    }

    return RESULT_OK;
}

u_result RPlidarDriverSerialImpl::_cacheScanData()
{
    rplidar_response_measurement_node_t      local_buf[128];
    size_t                                   count = 128;
    rplidar_response_measurement_node_t      local_scan[MAX_SCAN_NODES];
    size_t                                   scan_count = 0;
    u_result                                 ans;
    memset(local_scan, 0, sizeof(local_scan));

    _waitScanData(local_buf, count); // // always discard the first data since it may be incomplete

    while(_isScanning)
    {
        if (IS_FAIL(ans=_waitScanData(local_buf, count))) {
            if (ans != RESULT_OPERATION_TIMEOUT) {
                _isScanning = false;
                return RESULT_OPERATION_FAIL;
            }
        }

        for (size_t pos = 0; pos < count; ++pos)
        {
            if (local_buf[pos].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT)
            {
                // only publish the data when it contains a full 360 degree scan 
                
                if ((local_scan[0].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT)) {
                    _lock.lock();
                    memcpy(_cached_scan_node_buf, local_scan, scan_count*sizeof(rplidar_response_measurement_node_t));
                    _cached_scan_node_count = scan_count;
                    _dataEvt.set();
                    _lock.unlock();
                }
                scan_count = 0;
            }
            local_scan[scan_count++] = local_buf[pos];
            if (scan_count == _countof(local_scan)) scan_count-=1; // prevent overflow
        }
    }
    _isScanning = false;
    return RESULT_OK;
}

void     RPlidarDriverSerialImpl::_capsuleToNormal(const rplidar_response_capsule_measurement_nodes_t & capsule, rplidar_response_measurement_node_t *nodebuffer, size_t &nodeCount)
{
    nodeCount = 0;
    if (_is_previous_capsuledataRdy) {
        int diffAngle_q8;
        int currentStartAngle_q8 = ((capsule.start_angle_sync_q6 & 0x7FFF)<< 2);
        int prevStartAngle_q8 = ((_cached_previous_capsuledata.start_angle_sync_q6 & 0x7FFF) << 2);

        diffAngle_q8 = (currentStartAngle_q8) - (prevStartAngle_q8);
        if (prevStartAngle_q8 >  currentStartAngle_q8) {
            diffAngle_q8 += (360<<8);
        }

        int angleInc_q16 = (diffAngle_q8 << 3);
        int currentAngle_raw_q16 = (prevStartAngle_q8 << 8);
        for (size_t pos = 0; pos < _countof(_cached_previous_capsuledata.cabins); ++pos)
        {
            int dist_q2[2];
            int angle_q6[2];
            int syncBit[2];

            dist_q2[0] = (_cached_previous_capsuledata.cabins[pos].distance_angle_1 & 0xFFFC);
            dist_q2[1] = (_cached_previous_capsuledata.cabins[pos].distance_angle_2 & 0xFFFC);

            int angle_offset1_q3 = ( (_cached_previous_capsuledata.cabins[pos].offset_angles_q3 & 0xF) | ((_cached_previous_capsuledata.cabins[pos].distance_angle_1 & 0x3)<<4));
            int angle_offset2_q3 = ( (_cached_previous_capsuledata.cabins[pos].offset_angles_q3 >> 4) | ((_cached_previous_capsuledata.cabins[pos].distance_angle_2 & 0x3)<<4));

            angle_q6[0] = ((currentAngle_raw_q16 - (angle_offset1_q3<<13))>>10);
            syncBit[0] =  (( (currentAngle_raw_q16 + angleInc_q16) % (360<<16)) < angleInc_q16 )?1:0;
            currentAngle_raw_q16 += angleInc_q16;


            angle_q6[1] = ((currentAngle_raw_q16 - (angle_offset2_q3<<13))>>10);
            syncBit[1] =  (( (currentAngle_raw_q16 + angleInc_q16) % (360<<16)) < angleInc_q16 )?1:0;
            currentAngle_raw_q16 += angleInc_q16;

            for (int cpos = 0; cpos < 2; ++cpos) {

                if (angle_q6[cpos] < 0) angle_q6[cpos] += (360<<6);
                if (angle_q6[cpos] >= (360<<6)) angle_q6[cpos] -= (360<<6);

                rplidar_response_measurement_node_t node;

                node.sync_quality = (syncBit[cpos] | ((!syncBit[cpos]) << 1));
                if (dist_q2[cpos]) node.sync_quality |= (0x2F << RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);

                node.angle_q6_checkbit = (1 | (angle_q6[cpos]<<1));
                node.distance_q2 = dist_q2[cpos];

                nodebuffer[nodeCount++] = node;
             }

        }
    }

    _cached_previous_capsuledata = capsule;
    _is_previous_capsuledataRdy = true;
}


u_result RPlidarDriverSerialImpl::_cacheCapsuledScanData()
{
    rplidar_response_capsule_measurement_nodes_t    capsule_node;
    rplidar_response_measurement_node_t      local_buf[128];
    size_t                                   count = 128;
    rplidar_response_measurement_node_t      local_scan[MAX_SCAN_NODES];
    size_t                                   scan_count = 0;
    u_result                                 ans;
    memset(local_scan, 0, sizeof(local_scan));

    _waitCapsuledNode(capsule_node); // // always discard the first data since it may be incomplete

    while(_isScanning)
    {
        if (IS_FAIL(ans=_waitCapsuledNode(capsule_node))) {
            if (ans != RESULT_OPERATION_TIMEOUT && ans != RESULT_INVALID_DATA) {
                _isScanning = false;
                return RESULT_OPERATION_FAIL;
            } else {
                // current data is invalid, do not use it.
                continue;
            }
        }

        _capsuleToNormal(capsule_node, local_buf, count);

        for (size_t pos = 0; pos < count; ++pos)
        {
            if (local_buf[pos].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT)
            {
                // only publish the data when it contains a full 360 degree scan 
                
                if ((local_scan[0].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT)) {
                    _lock.lock();
                    memcpy(_cached_scan_node_buf, local_scan, scan_count*sizeof(rplidar_response_measurement_node_t));
                    _cached_scan_node_count = scan_count;
                    _dataEvt.set();
                    _lock.unlock();
                }
                scan_count = 0;
            }
            local_scan[scan_count++] = local_buf[pos];
            if (scan_count == _countof(local_scan)) scan_count-=1; // prevent overflow
        }
    }
    _isScanning = false;

    return RESULT_OK;
}

u_result RPlidarDriverSerialImpl::grabScanData(rplidar_response_measurement_node_t * nodebuffer, size_t & count, _u32 timeout)
{
    switch (_dataEvt.wait(timeout))
    {
    case rp::hal::Event::EVENT_TIMEOUT:
        count = 0;
        return RESULT_OPERATION_TIMEOUT;
    case rp::hal::Event::EVENT_OK:
        {
            if(_cached_scan_node_count == 0) return RESULT_OPERATION_TIMEOUT; //consider as timeout

            rp::hal::AutoLocker l(_lock);

            size_t size_to_copy = min(count, _cached_scan_node_count);

            memcpy(nodebuffer, _cached_scan_node_buf, size_to_copy*sizeof(rplidar_response_measurement_node_t));
            count = size_to_copy;
            _cached_scan_node_count = 0;
        }
        return RESULT_OK;

    default:
        count = 0;
        return RESULT_OPERATION_FAIL;
    }
}

u_result RPlidarDriverSerialImpl::ascendScanData(rplidar_response_measurement_node_t * nodebuffer, size_t count)
{
    float inc_origin_angle = 360.0/count;
    size_t i = 0;

    //Tune head
    for (i = 0; i < count; i++) {
        if(nodebuffer[i].distance_q2 == 0) {
            continue;
        } else {
            while(i != 0) {
                i--;
                float expect_angle = (nodebuffer[i+1].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f - inc_origin_angle;
                if (expect_angle < 0.0f) expect_angle = 0.0f;
                _u16 checkbit = nodebuffer[i].angle_q6_checkbit & RPLIDAR_RESP_MEASUREMENT_CHECKBIT;
                nodebuffer[i].angle_q6_checkbit = (((_u16)(expect_angle * 64.0f)) << RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + checkbit;
            }
            break;
        }
    }

    // all the data is invalid
    if (i == count) return RESULT_OPERATION_FAIL;

    //Tune tail
    for (i = count - 1; i >= 0; i--) {
        if(nodebuffer[i].distance_q2 == 0) {
            continue;
        } else {
            while(i != (count - 1)) {
                i++;
                float expect_angle = (nodebuffer[i-1].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f + inc_origin_angle;
                if (expect_angle > 360.0f) expect_angle -= 360.0f;
                _u16 checkbit = nodebuffer[i].angle_q6_checkbit & RPLIDAR_RESP_MEASUREMENT_CHECKBIT;
                nodebuffer[i].angle_q6_checkbit = (((_u16)(expect_angle * 64.0f)) << RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + checkbit;
            }
            break;
        }
    }

    //Fill invalid angle in the scan
    float frontAngle = (nodebuffer[0].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;
    for (i = 1; i < count; i++) {
        if(nodebuffer[i].distance_q2 == 0) {
            float expect_angle =  frontAngle + i * inc_origin_angle;
            if (expect_angle > 360.0f) expect_angle -= 360.0f;
            _u16 checkbit = nodebuffer[i].angle_q6_checkbit & RPLIDAR_RESP_MEASUREMENT_CHECKBIT;
            nodebuffer[i].angle_q6_checkbit = (((_u16)(expect_angle * 64.0f)) << RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + checkbit;
        }
    }

    // Reorder the scan according to the angle value
    for (i = 0; i < (count-1); i++){
        for (size_t j = (i+1); j < count; j++){
            if(nodebuffer[i].angle_q6_checkbit > nodebuffer[j].angle_q6_checkbit){
                rplidar_response_measurement_node_t temp = nodebuffer[i];
                nodebuffer[i] = nodebuffer[j];
                nodebuffer[j] = temp;
            }
        }
    }

    return RESULT_OK;
}

u_result RPlidarDriverSerialImpl::_waitNode(rplidar_response_measurement_node_t * node, _u32 timeout)
{
    int  recvPos = 0;
    _u32 startTs = getms();
    _u8  recvBuffer[sizeof(rplidar_response_measurement_node_t)];
    _u8 *nodeBuffer = (_u8*)node;
    _u32 waitTime;

   while ((waitTime=getms() - startTs) <= timeout) {
        size_t remainSize = sizeof(rplidar_response_measurement_node_t) - recvPos;
        size_t recvSize;

        int ans = _rxtx->waitfordata(remainSize, timeout-waitTime, &recvSize);
        if (ans == rp::hal::serial_rxtx::ANS_DEV_ERR) 
            return RESULT_OPERATION_FAIL;
        else if (ans == rp::hal::serial_rxtx::ANS_TIMEOUT)
            return RESULT_OPERATION_TIMEOUT;

        if (recvSize > remainSize) recvSize = remainSize;
        
        _rxtx->recvdata(recvBuffer, recvSize);

        for (size_t pos = 0; pos < recvSize; ++pos) {
            _u8 currentByte = recvBuffer[pos];
            switch (recvPos) {
            case 0: // expect the sync bit and its reverse in this byte
                {
                    _u8 tmp = (currentByte>>1);
                    if ( (tmp ^ currentByte) & 0x1 ) {
                        // pass
                    } else {
                        continue;
                    }

                }
                break;
            case 1: // expect the highest bit to be 1
                {
                    if (currentByte & RPLIDAR_RESP_MEASUREMENT_CHECKBIT) {
                        // pass
                    } else {
                        recvPos = 0;
                        continue;
                    }
                }
                break;
            }
            nodeBuffer[recvPos++] = currentByte;

            if (recvPos == sizeof(rplidar_response_measurement_node_t)) {
                return RESULT_OK;
            }
        }
    }

    return RESULT_OPERATION_TIMEOUT;
}


u_result RPlidarDriverSerialImpl::_waitScanData(rplidar_response_measurement_node_t * nodebuffer, size_t & count, _u32 timeout)
{
    if (!_isConnected) {
        count = 0;
        return RESULT_OPERATION_FAIL;
    }

    size_t   recvNodeCount =  0;
    _u32     startTs = getms();
    _u32     waitTime;
    u_result ans;

    while ((waitTime = getms() - startTs) <= timeout && recvNodeCount < count) {
        rplidar_response_measurement_node_t node;
        if (IS_FAIL(ans = _waitNode(&node, timeout - waitTime))) {
            return ans;
        }
        
        nodebuffer[recvNodeCount++] = node;

        if (recvNodeCount == count) return RESULT_OK;
    }
    count = recvNodeCount;
    return RESULT_OPERATION_TIMEOUT;
}


u_result RPlidarDriverSerialImpl::_waitCapsuledNode(rplidar_response_capsule_measurement_nodes_t & node, _u32 timeout)
{
    int  recvPos = 0;
    _u32 startTs = getms();
    _u8  recvBuffer[sizeof(rplidar_response_capsule_measurement_nodes_t)];
    _u8 *nodeBuffer = (_u8*)&node;
    _u32 waitTime;

   while ((waitTime=getms() - startTs) <= timeout) {
        size_t remainSize = sizeof(rplidar_response_capsule_measurement_nodes_t) - recvPos;
        size_t recvSize;

        int ans = _rxtx->waitfordata(remainSize, timeout-waitTime, &recvSize);
        if (ans == rp::hal::serial_rxtx::ANS_DEV_ERR) 
            return RESULT_OPERATION_FAIL;
        else if (ans == rp::hal::serial_rxtx::ANS_TIMEOUT)
            return RESULT_OPERATION_TIMEOUT;

        if (recvSize > remainSize) recvSize = remainSize;
        
        _rxtx->recvdata(recvBuffer, recvSize);

        for (size_t pos = 0; pos < recvSize; ++pos) {
            _u8 currentByte = recvBuffer[pos];
            switch (recvPos) {
            case 0: // expect the sync bit 1
                {
                    _u8 tmp = (currentByte>>4);
                    if ( tmp == RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_1 ) {
                        // pass
                    } else {
                        _is_previous_capsuledataRdy = false;
                        continue;
                    }

                }
                break;
            case 1: // expect the sync bit 2
                {
                    _u8 tmp = (currentByte>>4);
                    if (tmp == RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_2) {
                        // pass
                    } else {
                        recvPos = 0;
                        _is_previous_capsuledataRdy = false;
                        continue;
                    }
                }
                break;
            }
            nodeBuffer[recvPos++] = currentByte;

            if (recvPos == sizeof(rplidar_response_capsule_measurement_nodes_t)) {
                // calc the checksum ...
                _u8 checksum = 0;
                _u8 recvChecksum = ((node.s_checksum_1 & 0xF) | (node.s_checksum_2<<4));
                for (size_t cpos = offsetof(rplidar_response_capsule_measurement_nodes_t, start_angle_sync_q6);
                    cpos < sizeof(rplidar_response_capsule_measurement_nodes_t); ++cpos)
                {
                    checksum ^= nodeBuffer[cpos];
                }
                if (recvChecksum == checksum)
                {
                    // only consider vaild if the checksum matches...
                    if (node.start_angle_sync_q6 & RPLIDAR_RESP_MEASUREMENT_EXP_SYNCBIT) 
                    {
                        // this is the first capsule frame in logic, discard the previous cached data...
                        _is_previous_capsuledataRdy = false;
                        return RESULT_OK;
                    }
                    return RESULT_OK;
                }
                _is_previous_capsuledataRdy = false;
                return RESULT_INVALID_DATA;
            }
        }
    }
    _is_previous_capsuledataRdy = false;
    return RESULT_OPERATION_TIMEOUT;
}


u_result RPlidarDriverSerialImpl::_sendCommand(_u8 cmd, const void * payload, size_t payloadsize)
{
    _u8 pkt_header[10];
    rplidar_cmd_packet_t * header = reinterpret_cast<rplidar_cmd_packet_t * >(pkt_header);
    _u8 checksum = 0;

    if (!_isConnected) return RESULT_OPERATION_FAIL;

    if (payloadsize && payload) {
        cmd |= RPLIDAR_CMDFLAG_HAS_PAYLOAD;
    }

    header->syncByte = RPLIDAR_CMD_SYNC_BYTE;
    header->cmd_flag = cmd;

    // send header first
    _rxtx->senddata(pkt_header, 2) ;

    if (cmd & RPLIDAR_CMDFLAG_HAS_PAYLOAD) {
        checksum ^= RPLIDAR_CMD_SYNC_BYTE;
        checksum ^= cmd;
        checksum ^= (payloadsize & 0xFF);

        // calc checksum
        for (size_t pos = 0; pos < payloadsize; ++pos) {
            checksum ^= ((_u8 *)payload)[pos];
        }

        // send size
        _u8 sizebyte = payloadsize;
        _rxtx->senddata(&sizebyte, 1);

        // send payload
        _rxtx->senddata((const _u8 *)payload, sizebyte);

        // send checksum
        _rxtx->senddata(&checksum, 1);
    }

    return RESULT_OK;
}


u_result RPlidarDriverSerialImpl::_waitResponseHeader(rplidar_ans_header_t * header, _u32 timeout)
{
    int  recvPos = 0;
    _u32 startTs = getms();
    _u8  recvBuffer[sizeof(rplidar_ans_header_t)];
    _u8  *headerBuffer = reinterpret_cast<_u8 *>(header);
    _u32 waitTime;

    while ((waitTime=getms() - startTs) <= timeout) {
        size_t remainSize = sizeof(rplidar_ans_header_t) - recvPos;
        size_t recvSize;
        
        int ans = _rxtx->waitfordata(remainSize, timeout - waitTime, &recvSize);
        if (ans == rp::hal::serial_rxtx::ANS_DEV_ERR) 
            return RESULT_OPERATION_FAIL;
        else if (ans == rp::hal::serial_rxtx::ANS_TIMEOUT)
            return RESULT_OPERATION_TIMEOUT;
        
        if(recvSize > remainSize) recvSize = remainSize;
        
        _rxtx->recvdata(recvBuffer, recvSize);

        for (size_t pos = 0; pos < recvSize; ++pos) {
            _u8 currentByte = recvBuffer[pos];
            switch (recvPos) {
            case 0:
                if (currentByte != RPLIDAR_ANS_SYNC_BYTE1) {
                   continue;
                }
                
                break;
            case 1:
                if (currentByte != RPLIDAR_ANS_SYNC_BYTE2) {
                    recvPos = 0;
                    continue;
                }
                break;
            }
            headerBuffer[recvPos++] = currentByte;

            if (recvPos == sizeof(rplidar_ans_header_t)) {
                return RESULT_OK;
            }
        }
    }

    return RESULT_OPERATION_TIMEOUT;
}



void RPlidarDriverSerialImpl::_disableDataGrabbing()
{
    _isScanning = false;
    _cachethread.join();
}

u_result RPlidarDriverSerialImpl::getSampleDuration_uS(rplidar_response_sample_rate_t & rateInfo, _u32 timeout)
{  
    if (!isConnected()) return RESULT_OPERATION_FAIL;
    
    _disableDataGrabbing();
    
    rplidar_response_device_info_t devinfo;
    // 1. fetch the device version first...
    u_result ans = getDeviceInfo(devinfo, timeout);

    rateInfo.express_sample_duration_us = _cached_sampleduration_express;
    rateInfo.std_sample_duration_us = _cached_sampleduration_std;

    if (devinfo.firmware_version < ((0x1<<8) | 17)) {
        // provide fake data...

        return RESULT_OK;
    }


    {
        rp::hal::AutoLocker l(_lock);

        if (IS_FAIL(ans = _sendCommand(RPLIDAR_CMD_GET_SAMPLERATE))) {
            return ans;
        }

        rplidar_ans_header_t response_header;
        if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
            return ans;
        }

        // verify whether we got a correct header
        if (response_header.type != RPLIDAR_ANS_TYPE_SAMPLE_RATE) {
            return RESULT_INVALID_DATA;
        }

        _u32 header_size = (response_header.size_q30_subtype & RPLIDAR_ANS_HEADER_SIZE_MASK);
        if ( header_size < sizeof(rplidar_response_sample_rate_t)) {
            return RESULT_INVALID_DATA;
        }

        if (_rxtx->waitfordata(header_size, timeout) != rp::hal::serial_rxtx::ANS_OK) {
            return RESULT_OPERATION_TIMEOUT;
        }
        _rxtx->recvdata(reinterpret_cast<_u8 *>(&rateInfo), sizeof(rateInfo));
    }
    return RESULT_OK;
}

u_result RPlidarDriverSerialImpl::checkMotorCtrlSupport(bool & support, _u32 timeout)
{
    u_result  ans;
    support = false;
    
    if (!isConnected()) return RESULT_OPERATION_FAIL;
    
    _disableDataGrabbing();

    {
        rp::hal::AutoLocker l(_lock);

        rplidar_payload_acc_board_flag_t flag;
        flag.reserved = 0;

        if (IS_FAIL(ans = _sendCommand(RPLIDAR_CMD_GET_ACC_BOARD_FLAG, &flag, sizeof(flag)))) {
            return ans;
        }

        rplidar_ans_header_t response_header;
        if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
            return ans;
        }
        
        // verify whether we got a correct header
        if (response_header.type != RPLIDAR_ANS_TYPE_ACC_BOARD_FLAG) {
            return RESULT_INVALID_DATA;
        }

        _u32 header_size = (response_header.size_q30_subtype & RPLIDAR_ANS_HEADER_SIZE_MASK);
        if ( header_size < sizeof(rplidar_response_acc_board_flag_t)) {
            return RESULT_INVALID_DATA;
        }

        if (_rxtx->waitfordata(header_size, timeout) != rp::hal::serial_rxtx::ANS_OK) {
            return RESULT_OPERATION_TIMEOUT;
        }
        rplidar_response_acc_board_flag_t acc_board_flag;
        _rxtx->recvdata(reinterpret_cast<_u8 *>(&acc_board_flag), sizeof(acc_board_flag));

        if (acc_board_flag.support_flag & RPLIDAR_RESP_ACC_BOARD_FLAG_MOTOR_CTRL_SUPPORT_MASK) {
            support = true;
        }
    }
    return RESULT_OK;
}

u_result RPlidarDriverSerialImpl::setMotorPWM(_u16 pwm)
{
    u_result ans;
    rplidar_payload_motor_pwm_t motor_pwm;
    motor_pwm.pwm_value = pwm;

    {
        rp::hal::AutoLocker l(_lock);

        if (IS_FAIL(ans = _sendCommand(RPLIDAR_CMD_SET_MOTOR_PWM,(const _u8 *)&motor_pwm, sizeof(motor_pwm)))) {
            return ans;
        }
    }

    return RESULT_OK;
}

u_result RPlidarDriverSerialImpl::startMotor()
{
    if (_isSupportingMotorCtrl) { // RPLIDAR A2
        setMotorPWM(DEFAULT_MOTOR_PWM);
        delay(500);
        return RESULT_OK;
    } else { // RPLIDAR A1
        rp::hal::AutoLocker l(_lock);
        _rxtx->clearDTR();
        delay(500);
        return RESULT_OK;
    }
}

u_result RPlidarDriverSerialImpl::stopMotor()
{
    if (_isSupportingMotorCtrl) { // RPLIDAR A2
        setMotorPWM(0);
        delay(500);
        return RESULT_OK;
    } else { // RPLIDAR A1
        rp::hal::AutoLocker l(_lock);
        _rxtx->setDTR();
        delay(500);
        return RESULT_OK;
    }
}

}}}
