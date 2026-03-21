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

#include <mrpt/img/CImage.h>

#include <Eigen/Core>
#include <cassert>
#include <cstdint>

namespace mrpt::img
{
// ---------------------------------------------------------------------------
// Zero-copy Eigen::Map views over CImage pixel data
// ---------------------------------------------------------------------------

/** Zero-copy read-only Eigen::Map<RowMajor uint8> over a grayscale CImage.
 *  No memory is allocated; the Map shares the CImage buffer.
 *
 *  Requirements: img must be CH_GRAY, PixelDepth::D8U, and already loaded.
 *  The Map remains valid as long as `img` is alive and not resized.
 *
 *  Row stride is handled via Eigen::OuterStride so images with padding are
 *  also supported (though MRPT's STB backend typically has no padding).
 */
using EigenGrayMapConst = Eigen::Map<
    const Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>,
    Eigen::Unaligned,
    Eigen::OuterStride<Eigen::Dynamic>>;

using EigenGrayMap = Eigen::Map<
    Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>,
    Eigen::Unaligned,
    Eigen::OuterStride<Eigen::Dynamic>>;

[[nodiscard]] inline EigenGrayMapConst asEigenMap(const mrpt::img::CImage& img)
{
  assert(img.channels() == mrpt::img::CH_GRAY);
  assert(img.getPixelDepth() == mrpt::img::PixelDepth::D8U);
  const int rows = img.getHeight();
  const int cols = img.getWidth();
  const size_t stride = img.getRowStride();
  return {
      img.ptrLine<uint8_t>(0), rows, cols,
      Eigen::OuterStride<Eigen::Dynamic>(static_cast<int>(stride))};
}

[[nodiscard]] inline EigenGrayMap asEigenMap(mrpt::img::CImage& img)
{
  assert(img.channels() == mrpt::img::CH_GRAY);
  assert(img.getPixelDepth() == mrpt::img::PixelDepth::D8U);
  const int rows = img.getHeight();
  const int cols = img.getWidth();
  const size_t stride = img.getRowStride();
  return {
      img.ptrLine<uint8_t>(0), rows, cols,
      Eigen::OuterStride<Eigen::Dynamic>(static_cast<int>(stride))};
}

}  // namespace mrpt::img
