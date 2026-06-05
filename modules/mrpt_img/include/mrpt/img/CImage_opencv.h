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

#include <opencv2/core.hpp>

namespace mrpt::img
{
/** Returns a zero-copy, non-owning cv::Mat view over a mutable CImage buffer.
 *
 *  No pixel data is copied. The returned Mat shares the CImage memory and
 *  remains valid as long as `img` is alive and not resized.
 *
 *  Requirements: img must be PixelDepth::D8U (the default) and already loaded.
 *  Supports CH_GRAY (CV_8UC1), CH_RGB (CV_8UC3), and CH_RGBA (CV_8UC4).
 */
[[nodiscard]] inline cv::Mat toOpenCVMat(mrpt::img::CImage& img)
{
  const int nCh = static_cast<int>(img.channels());
  const int cvType = (nCh == 1) ? CV_8UC1 : (nCh == 4) ? CV_8UC4 : CV_8UC3;
  return {img.getHeight(), img.getWidth(), cvType, img.ptrLine<uint8_t>(0), img.getRowStride()};
}

/** Returns a zero-copy, read-only cv::Mat view over a const CImage buffer.
 *
 *  The const_cast is safe as long as the Mat is not written to; it exists
 *  because cv::Mat has no const-data constructor. Prefer toOpenCVMat() on
 *  mutable images; use `.clone()` if you need an independent copy.
 */
[[nodiscard]] inline cv::Mat toOpenCVMat(const mrpt::img::CImage& img)
{
  const int nCh = static_cast<int>(img.channels());
  const int cvType = (nCh == 1) ? CV_8UC1 : (nCh == 4) ? CV_8UC4 : CV_8UC3;
  return {
      img.getHeight(), img.getWidth(), cvType,
      // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
      const_cast<uint8_t*>(img.ptrLine<uint8_t>(0)), img.getRowStride()};
}

/** Copies pixel data from a cv::Mat into a newly allocated CImage.
 *
 *  Supported types: CV_8UC1 (→ CH_GRAY), CV_8UC3 (→ CH_RGB), CV_8UC4 (→ CH_RGBA).
 *  The image data is deep-copied; the returned CImage is independent of `mat`.
 */
[[nodiscard]] inline CImage fromOpenCVMat(const cv::Mat& mat)
{
  ASSERT_(mat.depth() == CV_8U);
  const int nCh = mat.channels();
  TImageChannels ch = (nCh == 1) ? CH_GRAY : (nCh == 4) ? CH_RGBA : CH_RGB;
  CImage out(mat.cols, mat.rows, ch);
  for (int r = 0; r < mat.rows; r++)
  {
    std::memcpy(
        out.ptrLine<uint8_t>(r), mat.ptr<uint8_t>(r),
        static_cast<size_t>(mat.cols) * static_cast<size_t>(nCh));
  }
  return out;
}

}  // namespace mrpt::img
