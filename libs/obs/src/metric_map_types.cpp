/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "obs-precomp.h"  // Precompiled headers
//
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/config/CLoadableOptions.h>
#include <mrpt/maps/metric_map_types.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt::obs;
using namespace mrpt::maps;

IMPLEMENTS_SERIALIZABLE(TMapGenericParams, CSerializable, mrpt::maps)

void TMapGenericParams::loadFromConfigFile(
    const mrpt::config::CConfigFileBase& source, const std::string& sct)
{
  MRPT_LOAD_CONFIG_VAR(enableSaveAs3DObject, bool, source, sct);
  MRPT_LOAD_CONFIG_VAR(enableObservationLikelihood, bool, source, sct);
  MRPT_LOAD_CONFIG_VAR(enableObservationInsertion, bool, source, sct);
}
void TMapGenericParams::saveToConfigFile(
    mrpt::config::CConfigFileBase& c, const std::string& s) const
{
  // Common:
  MRPT_SAVE_CONFIG_VAR_COMMENT(enableSaveAs3DObject, "");
  MRPT_SAVE_CONFIG_VAR_COMMENT(enableObservationLikelihood, "");
  MRPT_SAVE_CONFIG_VAR_COMMENT(enableObservationInsertion, "");
}

uint8_t TMapGenericParams::serializeGetVersion() const { return 0; }
void TMapGenericParams::serializeTo(mrpt::serialization::CArchive& out) const
{
  out << enableSaveAs3DObject << enableObservationLikelihood << enableObservationInsertion;
}
void TMapGenericParams::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 0:
    {
      in >> enableSaveAs3DObject >> enableObservationLikelihood >> enableObservationInsertion;
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };
}
