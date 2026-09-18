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

/** Shared helpers for the offscreen-rendering tests, which compare a freshly
 *  rendered frame against a reference ("golden") image stored in `tests/`.
 */
#pragma once

#include <gtest/gtest.h>
#include <mrpt/img/CImage.h>

#include <Eigen/Dense>
#include <cstdlib>
#include <iostream>
#include <string>

namespace mrpt::opengl::testing
{
/** Sum of the per-channel absolute differences between two RGB images. */
inline float imageDiff(const mrpt::img::CImage& im1, const mrpt::img::CImage& im2)
{
  const auto [r1, g1, b1] = im1.getAsRGBMatricesFloat();
  const auto [r2, g2, b2] = im2.getAsRGBMatricesFloat();

  return (r1 - r2).asEigen().array().abs().sum() + (g1 - g2).asEigen().array().abs().sum() +
         (b1 - b2).asEigen().array().abs().sum();
}

/** Compares `rendered` against the reference image in `referenceFile`.
 *
 *  Setting the environment variable `MRPT_UPDATE_RENDER_REFERENCES=1`
 *  overwrites the reference image with the rendered one instead of comparing,
 *  which is how these files are regenerated after an intended change in any
 *  of the visual objects they exercise.
 */
inline void expectMatchesReference(
    const mrpt::img::CImage& rendered,
    const std::string& referenceFile,
    float maxDiff,
    const std::string& label)
{
  const char* updateEnv = ::getenv("MRPT_UPDATE_RENDER_REFERENCES");
  if (updateEnv != nullptr && std::string(updateEnv) == "1")
  {
    EXPECT_TRUE(rendered.saveToFile(referenceFile));
    std::cout << "[render_reference] Updated: " << referenceFile << "\n";
    return;
  }

  mrpt::img::CImage reference;
  const bool readOk = reference.loadFromFile(referenceFile);
  EXPECT_TRUE(readOk) << "Could not read reference image: " << referenceFile;
  if (!readOk)
  {
    return;
  }

  const float diff = imageDiff(reference, rendered);
  std::cout << label << "=" << diff << "\n";
  EXPECT_LT(diff, maxDiff);
}
}  // namespace mrpt::opengl::testing
