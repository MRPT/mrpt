/*
 * Copyright (c) 2014, RoboPeak
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY 
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR 
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*
 *  RoboPeak LIDAR System
 *  Common Types definition
 *
 *  Copyright 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 * 
 */

#pragma once


#ifdef _WIN32

//fake stdint.h for VC only

typedef signed   char     int8_t;
typedef unsigned char     uint8_t;

typedef __int16           int16_t;
typedef unsigned __int16  uint16_t;

typedef __int32           int32_t;
typedef unsigned __int32  uint32_t;

typedef __int64           int64_t;
typedef unsigned __int64  uint64_t;

#else

#include <stdint.h>

#endif


//based on stdint.h
typedef int8_t         _s8;
typedef uint8_t        _u8;

typedef int16_t        _s16;
typedef uint16_t       _u16;

typedef int32_t        _s32;
typedef uint32_t       _u32;

typedef int64_t        _s64;
typedef uint64_t       _u64;

#define __small_endian

#ifndef __GNUC__
#define __attribute__(x)
#endif


// The _word_size_t uses actual data bus width of the current CPU
#ifdef _AVR_
typedef _u8            _word_size_t;
#define THREAD_PROC    
#elif defined (WIN64)
typedef _u64           _word_size_t;
#define THREAD_PROC    __stdcall
#elif defined (WIN32)
typedef _u32           _word_size_t;
#define THREAD_PROC    __stdcall
#elif defined (__GNUC__)
typedef unsigned long  _word_size_t;
#define THREAD_PROC   
#elif defined (__ICCARM__)
typedef _u32            _word_size_t;
#define THREAD_PROC  
#endif


typedef uint32_t u_result;

#define RESULT_OK              0
#define RESULT_FAIL_BIT        0x80000000
#define RESULT_ALREADY_DONE    0x20
#define RESULT_INVALID_DATA    (0x8000 | RESULT_FAIL_BIT)
#define RESULT_OPERATION_FAIL  (0x8001 | RESULT_FAIL_BIT)
#define RESULT_OPERATION_TIMEOUT  (0x8002 | RESULT_FAIL_BIT)
#define RESULT_OPERATION_STOP    (0x8003 | RESULT_FAIL_BIT)
#define RESULT_OPERATION_NOT_SUPPORT    (0x8004 | RESULT_FAIL_BIT)
#define RESULT_FORMAT_NOT_SUPPORT    (0x8005 | RESULT_FAIL_BIT)
#define RESULT_INSUFFICIENT_MEMORY   (0x8006 | RESULT_FAIL_BIT)

#define IS_OK(x)    ( ((x) & RESULT_FAIL_BIT) == 0 )
#define IS_FAIL(x)  ( ((x) & RESULT_FAIL_BIT) )

typedef _word_size_t (THREAD_PROC * thread_proc_t ) ( void * );