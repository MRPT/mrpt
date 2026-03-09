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

#include <mrpt/config/CLoadableOptions.h>
#include <mrpt/img/CImage.h>
#include <mrpt/vision/TKeyPoint.h>
#include <mrpt/vision/types.h>

#include <cstdint>
#include <vector>

namespace mrpt::vision
{
/** A single feature: keypoint + optional descriptor.
 * Stub for v3.0; to be expanded in the future.
 */
struct CFeature
{
  TKeyPoint keypoint;
  std::vector<uint8_t> descriptor;

  float descriptorDistanceTo(const CFeature& o) const
  {
    if (descriptor.size() != o.descriptor.size()) return 1e6f;
    float d = 0;
    for (size_t i = 0; i < descriptor.size(); i++)
      d += std::abs(static_cast<float>(descriptor[i]) - static_cast<float>(o.descriptor[i]));
    return d;
  }

  void getFirstDescriptorAsMatrix(mrpt::math::CMatrixFloat& m) const
  {
    m.resize(1, descriptor.size());
    for (size_t i = 0; i < descriptor.size(); i++) m(0, i) = static_cast<float>(descriptor[i]);
  }
};

/** A list of visual features. */
using CFeatureList = std::vector<CFeature>;

/** Stub feature extraction class for v3.0.
 * Methods will throw THROW_EXCEPTION("Not implemented") at runtime.
 * To be filled in the future.
 * \ingroup mrpt_vision_grp
 */
class CFeatureExtraction
{
 public:
  struct TOptions : public mrpt::config::CLoadableOptions
  {
    size_t patchSize{0};

    void loadFromConfigFile(const mrpt::config::CConfigFileBase&, const std::string&) override {}
    void saveToConfigFile(mrpt::config::CConfigFileBase&, const std::string&) const override {}
  };

  TOptions options;

  void detectFeatures(
      const mrpt::img::CImage& img,
      CFeatureList& feats,
      unsigned int init_ID = 0,
      size_t nDesiredFeatures = 0) const
  {
    THROW_EXCEPTION("CFeatureExtraction::detectFeatures() not yet implemented in v3.0");
  }

  void computeDescriptors(
      const mrpt::img::CImage& img, CFeatureList& feats, TDescriptorType descriptors) const
  {
    THROW_EXCEPTION("CFeatureExtraction::computeDescriptors() not yet implemented in v3.0");
  }
};

}  // namespace mrpt::vision
