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

#include "rplidar_protocol.h"

// Commands
//-----------------------------------------

// Commands without payload and response
#define RPLIDAR_CMD_STOP               0x25
#define RPLIDAR_CMD_SCAN               0x20
#define RPLIDAR_CMD_FORCE_SCAN         0x21
#define RPLIDAR_CMD_RESET              0x40


// Commands without payload but have response
#define RPLIDAR_CMD_GET_DEVICE_INFO    0x50
#define RPLIDAR_CMD_GET_DEVICE_HEALTH  0x52

#define RPLIDAR_CMD_GET_SAMPLERATE     0x59 //added in fw 1.17

// Commands with payload and have response
#define RPLIDAR_CMD_EXPRESS_SCAN       0x82 //added in fw 1.17

//add for A2 to set RPLIDAR motor pwm when using accessory board
#define RPLIDAR_CMD_SET_MOTOR_PWM      0xF0
#define RPLIDAR_CMD_GET_ACC_BOARD_FLAG 0xFF

#if defined(_WIN32)
#pragma pack(1)
#endif


// Payloads
// ------------------------------------------
#define RPLIDAR_EXPRESS_SCAN_MODE_NORMAL      0 
#define RPLIDAR_EXPRESS_SCAN_MODE_FIXANGLE    1
typedef struct _rplidar_payload_express_scan_t {
    _u8   working_mode;
    _u32  reserved;
} __attribute__((packed)) rplidar_payload_express_scan_t;

#define MAX_MOTOR_PWM               1023
#define DEFAULT_MOTOR_PWM           660
typedef struct _rplidar_payload_motor_pwm_t {
    _u16 pwm_value;
} __attribute__((packed)) rplidar_payload_motor_pwm_t;

typedef struct _rplidar_payload_acc_board_flag_t {
    _u32 reserved;
} __attribute__((packed)) rplidar_payload_acc_board_flag_t;

// Response
// ------------------------------------------
#define RPLIDAR_ANS_TYPE_DEVINFO          0x4
#define RPLIDAR_ANS_TYPE_DEVHEALTH        0x6

#define RPLIDAR_ANS_TYPE_MEASUREMENT                0x81
// Added in FW ver 1.17
#define RPLIDAR_ANS_TYPE_MEASUREMENT_CAPSULED       0x82

// Added in FW ver 1.17
#define RPLIDAR_ANS_TYPE_SAMPLE_RATE      0x15

#define RPLIDAR_ANS_TYPE_ACC_BOARD_FLAG   0xFF

#define RPLIDAR_RESP_ACC_BOARD_FLAG_MOTOR_CTRL_SUPPORT_MASK      (0x1)
typedef struct _rplidar_response_acc_board_flag_t {
    _u32 support_flag;
} __attribute__((packed)) rplidar_response_acc_board_flag_t;


#define RPLIDAR_STATUS_OK                 0x0
#define RPLIDAR_STATUS_WARNING            0x1
#define RPLIDAR_STATUS_ERROR              0x2

#define RPLIDAR_RESP_MEASUREMENT_SYNCBIT        (0x1<<0)
#define RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT  2
#define RPLIDAR_RESP_MEASUREMENT_CHECKBIT       (0x1<<0)
#define RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT    1

typedef struct _rplidar_response_sample_rate_t {
    _u16  std_sample_duration_us;
    _u16  express_sample_duration_us;
} __attribute__((packed)) rplidar_response_sample_rate_t;

typedef struct _rplidar_response_measurement_node_t {
    _u8    sync_quality;      // syncbit:1;syncbit_inverse:1;quality:6;
    _u16   angle_q6_checkbit; // check_bit:1;angle_q6:15;
    _u16   distance_q2;
} __attribute__((packed)) rplidar_response_measurement_node_t;

//[distance_sync flags]
#define RPLIDAR_RESP_MEASUREMENT_EXP_ANGLE_MASK           (0x3)
#define RPLIDAR_RESP_MEASUREMENT_EXP_DISTANCE_MASK        (0xFC)

typedef struct _rplidar_response_cabin_nodes_t {
    _u16   distance_angle_1; // see [distance_sync flags]
    _u16   distance_angle_2; // see [distance_sync flags]
    _u8    offset_angles_q3;  
} __attribute__((packed)) rplidar_response_cabin_nodes_t;   


#define RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_1               0xA
#define RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_2               0x5

#define RPLIDAR_RESP_MEASUREMENT_EXP_SYNCBIT              (0x1<<15)

typedef struct _rplidar_response_capsule_measurement_nodes_t {
    _u8                             s_checksum_1; // see [s_checksum_1]
    _u8                             s_checksum_2; // see [s_checksum_1]
    _u16                            start_angle_sync_q6;
    rplidar_response_cabin_nodes_t  cabins[16];
} __attribute__((packed)) rplidar_response_capsule_measurement_nodes_t;

typedef struct _rplidar_response_device_info_t {
    _u8   model;
    _u16  firmware_version;
    _u8   hardware_version;
    _u8   serialnum[16];
} __attribute__((packed)) rplidar_response_device_info_t;

typedef struct _rplidar_response_device_health_t {
    _u8   status;
    _u16  error_code;
} __attribute__((packed)) rplidar_response_device_health_t;

#if defined(_WIN32)
#pragma pack()
#endif
