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

// RP-Lidar Input Packets

#define RPLIDAR_CMD_SYNC_BYTE        0xA5
#define RPLIDAR_CMDFLAG_HAS_PAYLOAD  0x80


#define RPLIDAR_ANS_SYNC_BYTE1       0xA5
#define RPLIDAR_ANS_SYNC_BYTE2       0x5A

#define RPLIDAR_ANS_PKTFLAG_LOOP     0x1

#define RPLIDAR_ANS_HEADER_SIZE_MASK        0x3FFFFFFF
#define RPLIDAR_ANS_HEADER_SUBTYPE_SHIFT    (30)

#if defined(_WIN32)
#pragma pack(1)
#endif

typedef struct _rplidar_cmd_packet_t {
    _u8 syncByte; //must be RPLIDAR_CMD_SYNC_BYTE
    _u8 cmd_flag; 
    _u8 size;
    _u8 data[0];
} __attribute__((packed)) rplidar_cmd_packet_t;


typedef struct _rplidar_ans_header_t {
    _u8  syncByte1; // must be RPLIDAR_ANS_SYNC_BYTE1
    _u8  syncByte2; // must be RPLIDAR_ANS_SYNC_BYTE2
    _u32 size_q30_subtype; // see _u32 size:30; _u32 subType:2;
    _u8  type;
} __attribute__((packed)) rplidar_ans_header_t;

#if defined(_WIN32)
#pragma pack()
#endif
