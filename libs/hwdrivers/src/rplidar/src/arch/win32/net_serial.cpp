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
#include "net_serial.h"

namespace rp{ namespace arch{ namespace net{

raw_serial::raw_serial()
    : rp::hal::serial_rxtx()
    , _serial_handle(NULL)
    , _baudrate(0)
    , _flags(0)
{
    _init();
}

raw_serial::~raw_serial()
{
    close();

    CloseHandle(_ro.hEvent);
    CloseHandle(_wo.hEvent);
    CloseHandle(_wait_o.hEvent);
}

bool raw_serial::open()
{
    return open(_portName, _baudrate, _flags);
}

bool raw_serial::bind(const char * portname, _u32 baudrate, _u32 flags)
{   
    strncpy(_portName, portname, sizeof(_portName));
    _baudrate = baudrate;
    _flags    = flags;
    return true;
}

bool raw_serial::open(const char * portname, _u32 baudrate, _u32 flags)
{
    if (isOpened()) close();
    
    _serial_handle = CreateFile(
        portname,
        GENERIC_READ | GENERIC_WRITE,
        0,
        NULL,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED,
        NULL
        );

    if (_serial_handle == INVALID_HANDLE_VALUE) return false;

    if (!SetupComm(_serial_handle, SERIAL_RX_BUFFER_SIZE, SERIAL_TX_BUFFER_SIZE))
    {
        close();
        return false;
    }
    
    _dcb.BaudRate = baudrate;
    _dcb.ByteSize = 8;
    _dcb.Parity   = NOPARITY;
    _dcb.StopBits = ONESTOPBIT;
    _dcb.fDtrControl = DTR_CONTROL_ENABLE;

    if (!SetCommState(_serial_handle, &_dcb))
    {
        close();
        return false;
    }

    if (!SetCommTimeouts(_serial_handle, &_co))
    {
        close();
        return false;
    }

    if (!SetCommMask(_serial_handle, EV_RXCHAR | EV_ERR ))
    {
        close();
        return false;
    }

    if (!PurgeComm(_serial_handle, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR ))
    {
        close();
        return false;
    }

    Sleep(30); 
    _is_serial_opened = true;

    //Clear the DTR bit set DTR=high
    clearDTR();

    return true;
}

void raw_serial::close()
{
    SetCommMask(_serial_handle, 0);
    ResetEvent(_wait_o.hEvent);

    CloseHandle(_serial_handle);
    _serial_handle = INVALID_HANDLE_VALUE;
    
    _is_serial_opened = false;
}

int raw_serial::senddata(const unsigned char * data, size_t size)
{
    DWORD    error;
    DWORD w_len = 0, o_len = -1;
    if (!isOpened()) return ANS_DEV_ERR;

    if (data == NULL || size ==0) return 0;
    
    if(ClearCommError(_serial_handle, &error, NULL) && error > 0)
        PurgeComm(_serial_handle, PURGE_TXABORT | PURGE_TXCLEAR);

    if(!WriteFile(_serial_handle, data, size, &w_len, &_wo))
        if(GetLastError() != ERROR_IO_PENDING)
            w_len = ANS_DEV_ERR;

    return w_len;
}

int raw_serial::recvdata(unsigned char * data, size_t size)
{
    if (!isOpened()) return 0;
    DWORD r_len = 0;


    if(!ReadFile(_serial_handle, data, size, &r_len, &_ro))
    {
        if(GetLastError() == ERROR_IO_PENDING) 
        {
            if(!GetOverlappedResult(_serial_handle, &_ro, &r_len, FALSE))
            {
                if(GetLastError() != ERROR_IO_INCOMPLETE)
                    r_len = 0;
            }
        }
        else
            r_len = 0;
    }

    return r_len;
}

void raw_serial::flush( _u32 flags)
{
    PurgeComm(_serial_handle, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR );
}

int raw_serial::waitforsent(_u32 timeout, size_t * returned_size)
{
    if (!isOpened() ) return ANS_DEV_ERR;
    DWORD w_len = 0;
    _word_size_t ans =0;

    if (WaitForSingleObject(_wo.hEvent, timeout) == WAIT_TIMEOUT)
    {
        ans = ANS_TIMEOUT;
        goto _final;
    }
    if(!GetOverlappedResult(_serial_handle, &_wo, &w_len, FALSE))
    {
        ans = ANS_DEV_ERR;
    }
_final:
    if (returned_size) *returned_size = w_len;
    return ans;
}

int raw_serial::waitforrecv(_u32 timeout, size_t * returned_size)
{
    if (!isOpened() ) return -1;
    DWORD r_len = 0;
    _word_size_t ans =0;

    if (WaitForSingleObject(_ro.hEvent, timeout) == WAIT_TIMEOUT)
    {
        ans = ANS_TIMEOUT;
    }
    if(!GetOverlappedResult(_serial_handle, &_ro, &r_len, FALSE))
    {
        ans = ANS_DEV_ERR;
    }
    if (returned_size) *returned_size = r_len;
    return ans;
}

int raw_serial::waitfordata(size_t data_count, _u32 timeout, size_t * returned_size)
{
    COMSTAT  stat;
    DWORD error;
    DWORD msk,length;
    size_t dummy_length;

    if (returned_size==NULL) returned_size=(size_t *)&dummy_length;

    
    if ( isOpened()) {
        size_t rxqueue_remaining =  rxqueue_count();
        if (rxqueue_remaining >= data_count) {
            *returned_size = rxqueue_remaining;
            return 0;
        }
    }

    while ( isOpened() )
    {
        msk = 0;
        SetCommMask(_serial_handle, EV_RXCHAR | EV_ERR );
        if(!WaitCommEvent(_serial_handle, &msk, &_wait_o))
        {
            if(GetLastError() == ERROR_IO_PENDING)
            {
                if (WaitForSingleObject(_wait_o.hEvent, timeout) == WAIT_TIMEOUT)
                {
                    *returned_size =0;
                    return ANS_TIMEOUT;
                }

                GetOverlappedResult(_serial_handle, &_wait_o, &length, TRUE);

                ::ResetEvent(_wait_o.hEvent);
            }else
            {
                ClearCommError(_serial_handle, &error, &stat);
                 *returned_size = stat.cbInQue;
                return ANS_DEV_ERR;
            }
        }

        if(msk & EV_ERR){
            // FIXME: may cause problem here
            ClearCommError(_serial_handle, &error, &stat);
        }

        if(msk & EV_RXCHAR){
            ClearCommError(_serial_handle, &error, &stat);
            if(stat.cbInQue >= data_count)
            {
                *returned_size = stat.cbInQue;
                return 0;
            }
        }
    }
    *returned_size=0;
    return ANS_DEV_ERR;
}

size_t raw_serial::rxqueue_count()
{
    if  ( !isOpened() ) return 0;
    COMSTAT  com_stat;
    DWORD error;
    DWORD r_len = 0;

    if(ClearCommError(_serial_handle, &error, &com_stat) && error > 0)
    {
        PurgeComm(_serial_handle, PURGE_RXABORT | PURGE_RXCLEAR);
        return 0;
    }
    return com_stat.cbInQue;
}

void raw_serial::setDTR()
{
    if ( !isOpened() ) return;

    EscapeCommFunction(_serial_handle, SETDTR);
}

void raw_serial::clearDTR()
{
    if ( !isOpened() ) return;

    EscapeCommFunction(_serial_handle, CLRDTR);
}


void raw_serial::_init()
{
    memset(&_dcb, 0, sizeof(_dcb));
    _dcb.DCBlength = sizeof(_dcb);
    _serial_handle = INVALID_HANDLE_VALUE;
    memset(&_co, 0, sizeof(_co));
    _co.ReadIntervalTimeout = 0;
    _co.ReadTotalTimeoutMultiplier = 0;
    _co.ReadTotalTimeoutConstant = 0;
    _co.WriteTotalTimeoutMultiplier = 0;
    _co.WriteTotalTimeoutConstant = 0;

    memset(&_ro, 0, sizeof(_ro));
    memset(&_wo, 0, sizeof(_wo));
    memset(&_wait_o, 0, sizeof(_wait_o));

    _ro.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    _wo.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    _wait_o.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

    _portName[0] = 0;
}

}}} //end rp::arch::net


//begin rp::hal
namespace rp{ namespace hal{

serial_rxtx * serial_rxtx::CreateRxTx()
{
    return new rp::arch::net::raw_serial();
}

void  serial_rxtx::ReleaseRxTx( serial_rxtx * rxtx)
{
    delete rxtx;
}


}} //end rp::hal
