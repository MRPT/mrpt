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
/** A PTG for optimal paths of type "C|C,S" (as named in PTG papers).
 * - **Compatible kinematics**: differential-driven / Ackermann steering
 * - **Compatible robot shape**: Arbitrary 2D polygon
 * - **PTG parameters**: Use the app `ptg-configurator`
 *
 *  See also "Obstacle Distance for Car-Like Robots", IEEE Trans. Rob. And
 * Autom, 1999.
 * \note [Before MRPT 1.5.0 this was named CPTG3]
 *  \ingroup nav_tpspace
 */
class CPTG_DiffDrive_CCS : public CPTG_DiffDrive_CollisionGridBased
{
  DEFINE_SERIALIZABLE(CPTG_DiffDrive_CCS, mrpt::nav)
 public:
  CPTG_DiffDrive_CCS() = default;
  CPTG_DiffDrive_CCS(const mrpt::config::CConfigFileBase& cfg, const std::string& sSection)
  {
    CPTG_DiffDrive_CCS::loadFromConfigFile(cfg, sSection);
  }
  void loadFromConfigFile(
      const mrpt::config::CConfigFileBase& cfg, const std::string& sSection) override;
  void saveToConfigFile(
      mrpt::config::CConfigFileBase& cfg, const std::string& sSection) const override;

  std::string getDescription() const override;
  bool PTG_IsIntoDomain(double x, double y) const override;
  void ptgDiffDriveSteeringFunction(
      float alpha, float t, float x, float y, float phi, float& v, float& w) const override;
  void loadDefaultParams() override;

 protected:
  double R{0}, K{0};
};
}  // namespace mrpt::nav
