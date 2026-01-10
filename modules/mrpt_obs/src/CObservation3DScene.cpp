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

#include <mrpt/obs/CObservation3DScene.h>
#include <mrpt/serialization/CArchive.h>

#include <iostream>

using namespace mrpt::obs;
using namespace mrpt::poses;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservation3DScene, CObservation, mrpt::obs)

uint8_t CObservation3DScene::serializeGetVersion() const { return 0; }
void CObservation3DScene::serializeTo(mrpt::serialization::CArchive& out) const
{
  out << scene << timestamp;
}

void CObservation3DScene::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 0:
      in >> scene >> timestamp;
      break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };
}

void CObservation3DScene::getDescriptionAsText(std::ostream& o) const
{
  CObservation::getDescriptionAsText(o);
  o << "3D scene:\n'";
  if (!scene)
  {
    o << "nullptr\n";
  }
  else
  {
    auto d = scene->asYAML();
    d.printAsYAML(o);
  }
}

void CObservation3DScene::getVisualizationInto(mrpt::viz::CSetOfObjects& o) const
{
  if (!scene) return;

  auto mainView = scene->getViewport();
  if (!mainView) return;

  for (const auto& obj : *mainView)
  {
    if (!obj) continue;
    o.insert(obj);
  }
}
