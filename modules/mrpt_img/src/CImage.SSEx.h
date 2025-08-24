/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
#pragma once

#include <cstddef>
#include <cstdint>

// See documentation in the .cpp files CImage_SSE*.cpp

void image_SSE2_scale_half_1c8u(
    const uint8_t* in, uint8_t* out, int w, int h, size_t in_step, size_t out_step);
void image_SSSE3_scale_half_3c8u(
    const uint8_t* in, uint8_t* out, int w, int h, size_t in_step, size_t out_step);
void image_SSE2_scale_half_smooth_1c8u(
    const uint8_t* in, uint8_t* out, int w, int h, size_t in_step, size_t out_step);
void image_SSSE3_rgb_to_gray_8u(
    const uint8_t* in, uint8_t* out, int w, int h, size_t in_step, size_t out_step);
void image_SSSE3_bgr_to_gray_8u(
    const uint8_t* in, uint8_t* out, int w, int h, size_t in_step, size_t out_step);
