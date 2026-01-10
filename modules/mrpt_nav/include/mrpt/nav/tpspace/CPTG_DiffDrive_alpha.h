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

#include <mrpt/nav/tpspace/CPTG_DiffDrive_CollisionGridBased.h>

namespace mrpt::nav
{
/** The "a(symptotic)-alpha PTG", as named in PTG papers.
 * - **Compatible kinematics**: differential-driven / Ackermann steering
 * - **Compatible robot shape**: Arbitrary 2D polygon
 * - **PTG parameters**: Use the app `ptg-configurator`
 *
 * This PT generator functions are:
 *
 * \f[ v(\alpha) = V_{MAX} e^{ -\left( \dfrac{\alpha-\phi}{cte_{a0v}} \right)^2}
 * \f]
 * \f[ \omega(\alpha) = W_{MAX} \left( -\dfrac{1}{2} +\dfrac{1}{1+ e^{ -
 * \dfrac{\alpha-\phi}{cte_{a0w}} } } \right) \f]
 *
 * So, the radius of curvature of each trajectory is NOT constant for each
 * "alpha" value in this PTG:
 *
 *  ![C-PTG path examples](PTG2_paths.png)
 *
 * \note [Before MRPT 1.5.0 this was named CPTG2]
 *  \ingroup nav_tpspace
 */
class CPTG_DiffDrive_alpha : public CPTG_DiffDrive_CollisionGridBased
{
  DEFINE_SERIALIZABLE(CPTG_DiffDrive_alpha, mrpt::nav)
 public:
  CPTG_DiffDrive_alpha() = default;
  CPTG_DiffDrive_alpha(const mrpt::config::CConfigFileBase& cfg, const std::string& sSection)
  {
    CPTG_DiffDrive_alpha::loadFromConfigFile(cfg, sSection);
  }
  void loadFromConfigFile(
      const mrpt::config::CConfigFileBase& cfg, const std::string& sSection) override;
  void saveToConfigFile(
      mrpt::config::CConfigFileBase& cfg, const std::string& sSection) const override;

  std::string getDescription() const override;
  void ptgDiffDriveSteeringFunction(
      float alpha, float t, float x, float y, float phi, float& v, float& w) const override;
  void loadDefaultParams() override;

 protected:
  double cte_a0v = 0, cte_a0w = 0, K = 1.0;
};
}  // namespace mrpt::nav
