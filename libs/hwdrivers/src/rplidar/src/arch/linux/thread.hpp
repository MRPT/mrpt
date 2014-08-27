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
 *  Thread abstract layer implementation
 *
 *  Copyright 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 * 
 */

#include "arch/linux/arch_linux.h"

#include <sched.h>

namespace rp{ namespace hal{

Thread Thread::create(thread_proc_t proc, void * data)
{
    Thread newborn(proc, data);
    
    // tricky code, we assume pthread_t is not a structure but a word size value
    assert( sizeof(newborn._handle) >= sizeof(pthread_t));

    pthread_create((pthread_t *)&newborn._handle, NULL, (void * (*)(void *))proc, data);

    return newborn;
}

u_result Thread::terminate()
{
    if (!this->_handle) return RESULT_OK;
    
    return pthread_cancel((pthread_t)this->_handle)==0?RESULT_OK:RESULT_OPERATION_FAIL;
}

u_result Thread::setPriority( priority_val_t p)
{
    if (!this->_handle) return RESULT_OPERATION_FAIL;
    
    // check whether current schedule policy supports priority levels
    
    int current_policy;
    struct sched_param current_param;
    int ans;
    if (pthread_getschedparam( (pthread_t) this->_handle, &current_policy, &current_param))
    {
        // cannot retreieve values
        return RESULT_OPERATION_FAIL;
    }   

    //int pthread_priority = 0 ;

    switch(p)
    {
    case PRIORITY_REALTIME:
        //pthread_priority = pthread_priority_max;
        current_policy = SCHED_RR;
        break;
    case PRIORITY_HIGH:
        //pthread_priority = (pthread_priority_max + pthread_priority_min)/2;
        current_policy = SCHED_RR;
        break;
    case PRIORITY_NORMAL:
    case PRIORITY_LOW:
    case PRIORITY_IDLE:
        //pthread_priority = 0;
        current_policy = SCHED_OTHER;
        break;
    }

    current_param.__sched_priority = current_policy;
    if ( (ans = pthread_setschedparam( (pthread_t) this->_handle, current_policy, &current_param)) )
    {
        return RESULT_OPERATION_FAIL;
    }
    return  RESULT_OK;
}

Thread::priority_val_t Thread::getPriority()
{
    if (!this->_handle) return PRIORITY_NORMAL;

    int current_policy;
    struct sched_param current_param;
    if (pthread_getschedparam( (pthread_t) this->_handle, &current_policy, &current_param))
    {
        // cannot retreieve values
        return PRIORITY_NORMAL;
    }   

    int pthread_priority_max = sched_get_priority_max(SCHED_RR);
    int pthread_priority_min = sched_get_priority_min(SCHED_RR);

    if (current_param.__sched_priority ==(pthread_priority_max ))
    {
        return PRIORITY_REALTIME;
    }
    if (current_param.__sched_priority >=(pthread_priority_max + pthread_priority_min)/2)
    {
        return PRIORITY_HIGH;
    }
    return PRIORITY_NORMAL;
}

u_result Thread::join(unsigned long timeout)
{
    if (!this->_handle) return RESULT_OK;
    
    pthread_join((pthread_t)(this->_handle), NULL);
    return RESULT_OK;
}

}}
