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
#include <mrpt/obs/CObservationComment.h>
#include <mrpt/serialization/CArchive.h>

#include <iostream>

using namespace mrpt::obs;
using namespace mrpt::poses;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationComment, CObservation, mrpt::obs)

uint8_t CObservationComment::serializeGetVersion() const { return 1; }
void CObservationComment::serializeTo(mrpt::serialization::CArchive& out) const
{
  out << text << timestamp;
  out << sensorLabel;  // v1
}

void CObservationComment::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 0:
    case 1:
      in >> text >> timestamp;
      if (version >= 1) in >> sensorLabel;
      break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };
}

void CObservationComment::getDescriptionAsText(std::ostream& o) const
{
  CObservation::getDescriptionAsText(o);
  o << "Comment content:\n'" << text << "'\n";
}
