/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
#pragma once

// Private interface to the Cloudini point cloud codec. Its implementation is the
// only translation unit including Cloudini headers (which require C++20), so the
// rest of the library stays C++17.

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace mrpt::maps::internal
{
/** Whether this build of MRPT can encode and decode Cloudini streams. */
bool cloudini_available();

/** \exception std::exception If this build of MRPT does not support Cloudini. */
void cloudini_require();

/** Encodes `n` values of one point field (a contiguous array) into `out`.
 *  `resolution`: quantization step for float/double fields (max error
 *  resolution/2); std::nullopt stores the values exactly. Integer fields are
 *  always lossless.
 *  \exception std::exception If built without Cloudini.
 */
template <typename T>
void cloudini_encode(
    const T* data, std::size_t n, std::optional<float> resolution, std::vector<uint8_t>& out);

/** Decodes a stream written by cloudini_encode<T>() into `n` values at `data`.
 *  \exception std::exception If built without Cloudini, or if the stream does
 *  not hold `n` values of type T.
 */
template <typename T>
void cloudini_decode(const uint8_t* stream, std::size_t streamSize, T* data, std::size_t n);

}  // namespace mrpt::maps::internal
