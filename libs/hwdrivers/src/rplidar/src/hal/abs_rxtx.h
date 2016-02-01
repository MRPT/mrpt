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
 *  Serial Driver Interface
 *
 *  Copyright 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 * 
 */

#pragma once 

#include "rptypes.h"

namespace rp{ namespace hal{

class serial_rxtx
{
public:
    enum{
        ANS_OK      = 0,
        ANS_TIMEOUT = -1,
        ANS_DEV_ERR = -2,
    };

    static serial_rxtx * CreateRxTx();
    static void ReleaseRxTx( serial_rxtx * );

    serial_rxtx():_is_serial_opened(false){}
    virtual ~serial_rxtx(){}

    virtual void flush( _u32 flags) = 0;

    virtual bool bind(const char * portname, _u32 baudrate, _u32 flags = 0) = 0;
    virtual bool open() = 0;
    virtual void close()  = 0;
    
    virtual int waitfordata(size_t data_count,_u32 timeout = -1, size_t * returned_size = NULL) = 0;

    virtual int senddata(const unsigned char * data, size_t size) = 0;
    virtual int recvdata(unsigned char * data, size_t size) = 0;
	
    virtual int waitforsent(_u32 timeout = -1, size_t * returned_size = NULL) = 0;
    virtual int waitforrecv(_u32 timeout = -1, size_t * returned_size = NULL) = 0;

    virtual size_t rxqueue_count() = 0;

	virtual void setDTR() = 0;
	virtual void clearDTR() = 0;
	
    virtual bool isOpened()
    {
        return _is_serial_opened;
    }

protected:
    volatile bool   _is_serial_opened;
};

}}



