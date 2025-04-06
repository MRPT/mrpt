/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "nav-precomp.h"  // Precomp header
//
#include <mrpt/math/wrap2pi.h>
#include <mrpt/nav/tpspace/CPTG_DiffDrive_alpha.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/os.h>

using namespace mrpt;
using namespace mrpt::nav;
using namespace mrpt::system;

IMPLEMENTS_SERIALIZABLE(CPTG_DiffDrive_alpha, CParameterizedTrajectoryGenerator, mrpt::nav)

void CPTG_DiffDrive_alpha::loadFromConfigFile(
    const mrpt::config::CConfigFileBase& cfg, const std::string& sSection)
{
  CPTG_DiffDrive_CollisionGridBased::loadFromConfigFile(cfg, sSection);

  MRPT_LOAD_HERE_CONFIG_VAR_DEGREES_NO_DEFAULT(cte_a0v_deg, double, cte_a0v, cfg, sSection);
  MRPT_LOAD_HERE_CONFIG_VAR_DEGREES_NO_DEFAULT(cte_a0w_deg, double, cte_a0w, cfg, sSection);
  MRPT_LOAD_HERE_CONFIG_VAR(K, double, K, cfg, sSection);
}
void CPTG_DiffDrive_alpha::saveToConfigFile(
    mrpt::config::CConfigFileBase& cfg, const std::string& sSection) const
{
  MRPT_START
  const int WN = 25, WV = 30;
  CPTG_DiffDrive_CollisionGridBased::saveToConfigFile(cfg, sSection);

  cfg.write(
      sSection, "cte_a0v_deg", mrpt::RAD2DEG(cte_a0v), WN, WV, "Contant for vel profile [deg].");
  cfg.write(
      sSection, "cte_a0w_deg", mrpt::RAD2DEG(cte_a0v), WN, WV, "Contant for omega profile [deg].");
  cfg.write(sSection, "K", K, WN, WV, "1: forward, -1: backwards");

  MRPT_END
}

std::string CPTG_DiffDrive_alpha::getDescription() const
{
  char str[100];
  os::sprintf(
      str, 100, "CPTG_DiffDrive_alpha,av=%udeg,aw=%udeg,K=%i", (int)RAD2DEG(cte_a0v),
      (int)RAD2DEG(cte_a0w), (int)K);
  return std::string(str);
}

void CPTG_DiffDrive_alpha::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  CPTG_DiffDrive_CollisionGridBased::internal_readFromStream(in);

  switch (version)
  {
    case 0:
    case 1:
      in >> cte_a0v >> cte_a0w;
      if (version >= 1) in >> K;
      break;

    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };
}

uint8_t CPTG_DiffDrive_alpha::serializeGetVersion() const { return 1; }
void CPTG_DiffDrive_alpha::serializeTo(mrpt::serialization::CArchive& out) const
{
  CPTG_DiffDrive_CollisionGridBased::internal_writeToStream(out);
  out << cte_a0v << cte_a0w << K;
}
/*---------------------------------------------------------------
            ptgDiffDriveSteeringFunction
  ---------------------------------------------------------------*/
void CPTG_DiffDrive_alpha::ptgDiffDriveSteeringFunction(
    float alpha,
    [[maybe_unused]] float t,
    [[maybe_unused]] float x,
    [[maybe_unused]] float y,
    float phi,
    float& v,
    float& w) const
{
  const float correctedPhi = sign(K) * phi;
  float At_a = mrpt::math::wrapToPi(alpha - correctedPhi);

  v = sign(K) * V_MAX * exp(-square(At_a / cte_a0v));
  w = sign(K) * W_MAX * (-0.5f + (1 / (1 + exp(-At_a / cte_a0w))));
}

void CPTG_DiffDrive_alpha::loadDefaultParams()
{
  CPTG_DiffDrive_CollisionGridBased::loadDefaultParams();

  cte_a0v = mrpt::DEG2RAD(45.0);
  cte_a0w = mrpt::DEG2RAD(45.0);
}
