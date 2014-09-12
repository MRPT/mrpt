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
 *  Thread abstract layer
 *
 *  Copyright 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 * 
 */

#pragma once

#include "rptypes.h"
#define CLASS_THREAD(c , x ) \
	rp::hal::Thread::create_member<c, &c::x>(this )

namespace rp{ namespace hal{

class Thread
{
public:
    enum priority_val_t
	{
		PRIORITY_REALTIME = 0,
		PRIORITY_HIGH     = 1,
		PRIORITY_NORMAL   = 2,
		PRIORITY_LOW      = 3,
		PRIORITY_IDLE     = 4,
	};

    template <class T, u_result (T::*PROC)(void)>
    static Thread create_member(T * pthis)
    {
		return create(_thread_thunk<T,PROC>, pthis);
	}

	template <class T, u_result (T::*PROC)(void) >
	static _word_size_t THREAD_PROC _thread_thunk(void * data)
	{
		return (static_cast<T *>(data)->*PROC)();
	}
	static Thread create(thread_proc_t proc, void * data = NULL );

public:
    ~Thread() { }
    Thread():  _data(NULL),_func(NULL),_handle(0)  {}
    _word_size_t getHandle(){ return _handle;}
    u_result terminate();
    void *getData() { return _data;}
    u_result join(unsigned long timeout = -1);
	u_result setPriority( priority_val_t p);
	priority_val_t getPriority();

    bool operator== ( const Thread & right) { return this->_handle == right._handle; }
protected:
    Thread( thread_proc_t proc, void * data ): _data(data),_func(proc), _handle(0)  {}
    void * _data;
    thread_proc_t _func;
    _word_size_t _handle;
};

}}

