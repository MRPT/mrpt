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
#include <process.h>

namespace rp{ namespace hal{

Thread Thread::create(thread_proc_t proc, void * data)
{
    Thread newborn(proc, data);

    newborn._handle = (_word_size_t)( 
        _beginthreadex(NULL, 0, (unsigned int (_stdcall * )( void * ))proc,
                        data, 0, NULL));
    return newborn;
}

u_result Thread::terminate()
{
    if (!this->_handle) return RESULT_OK;
    if (TerminateThread( reinterpret_cast<HANDLE>(this->_handle), -1))
    {
        CloseHandle(reinterpret_cast<HANDLE>(this->_handle));
        this->_handle = NULL;
        return RESULT_OK;
    }else
    {
        return RESULT_OPERATION_FAIL;
    }
}

u_result Thread::setPriority( priority_val_t p)
{
	if (!this->_handle) return RESULT_OPERATION_FAIL;

	int win_priority =  THREAD_PRIORITY_NORMAL;
	switch(p)
	{
	case PRIORITY_REALTIME:
		win_priority = THREAD_PRIORITY_TIME_CRITICAL;
		break;
	case PRIORITY_HIGH:
		win_priority = THREAD_PRIORITY_HIGHEST;
		break;
	case PRIORITY_NORMAL:
		win_priority = THREAD_PRIORITY_NORMAL;
		break;
	case PRIORITY_LOW:
		win_priority = THREAD_PRIORITY_LOWEST;
		break;
	case PRIORITY_IDLE:
		win_priority = THREAD_PRIORITY_IDLE;
		break;
	}

	if (SetThreadPriority(reinterpret_cast<HANDLE>(this->_handle), win_priority))
	{
		return RESULT_OK;
	}
	return RESULT_OPERATION_FAIL;
}

Thread::priority_val_t Thread::getPriority()
{
	if (!this->_handle) return PRIORITY_NORMAL;
	int win_priority =  ::GetThreadPriority(reinterpret_cast<HANDLE>(this->_handle));
	
	if (win_priority == THREAD_PRIORITY_ERROR_RETURN)
	{
		return PRIORITY_NORMAL;
	}

	if (win_priority >= THREAD_PRIORITY_TIME_CRITICAL )
	{
		return PRIORITY_REALTIME;
	}
	else if (win_priority<THREAD_PRIORITY_TIME_CRITICAL && win_priority>=THREAD_PRIORITY_ABOVE_NORMAL)
	{	
		return PRIORITY_HIGH;
	}
	else if (win_priority<THREAD_PRIORITY_ABOVE_NORMAL && win_priority>THREAD_PRIORITY_BELOW_NORMAL)
	{
		return PRIORITY_NORMAL;
	}else if (win_priority<=THREAD_PRIORITY_BELOW_NORMAL && win_priority>THREAD_PRIORITY_IDLE)
	{
		return PRIORITY_LOW;
	}else if (win_priority<=THREAD_PRIORITY_IDLE)
	{
		return PRIORITY_IDLE;
	}
	return PRIORITY_NORMAL;
}

u_result Thread::join(unsigned long timeout)
{
    if (!this->_handle) return RESULT_OK;
    switch ( WaitForSingleObject(reinterpret_cast<HANDLE>(this->_handle), timeout))
    {
    case WAIT_OBJECT_0:
        CloseHandle(reinterpret_cast<HANDLE>(this->_handle));
        this->_handle = NULL;
        return RESULT_OK;
    case WAIT_ABANDONED:
        return RESULT_OPERATION_FAIL;
    case WAIT_TIMEOUT:
        return RESULT_OPERATION_TIMEOUT;
    }

    return RESULT_OK;
}

}}
