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

#pragma once

#include "hal/abs_rxtx.h"

namespace rp{ namespace arch{ namespace net{

class raw_serial : public rp::hal::serial_rxtx
{
public:
    enum{
        SERIAL_RX_BUFFER_SIZE = 512,
        SERIAL_TX_BUFFER_SIZE = 128,
        SERIAL_RX_TIMEOUT     = 2000,
        SERIAL_TX_TIMEOUT     = 2000,
    };

    raw_serial();
    virtual ~raw_serial();
    virtual bool bind(const char * portname, _u32 baudrate, _u32 flags = 0);
    virtual bool open();
    virtual void close();
    virtual void flush( _u32 flags);
    
    virtual int waitfordata(size_t data_count,_u32 timeout = -1, size_t * returned_size = NULL);

    virtual int senddata(const unsigned char * data, size_t size);
    virtual int recvdata(unsigned char * data, size_t size);

    virtual int waitforsent(_u32 timeout = -1, size_t * returned_size = NULL);
    virtual int waitforrecv(_u32 timeout = -1, size_t * returned_size = NULL);

    virtual size_t rxqueue_count();

    virtual void setDTR();
    virtual void clearDTR();

protected:
    bool open(const char * portname, _u32 baudrate, _u32 flags);
    void _init();

    char _portName[20];
    uint32_t _baudrate;
    uint32_t _flags;

    OVERLAPPED _ro, _wo;
    OVERLAPPED _wait_o;
    volatile HANDLE _serial_handle;
    DCB _dcb;
    COMMTIMEOUTS _co;
};

}}}
