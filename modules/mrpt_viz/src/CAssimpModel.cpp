/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

// This file contains portions of code from Assimp's example:
// "Sample_SimpleOpenGL.c"

#include "viz-precomp.h"  // Precompiled header
//
#include <mrpt/core/lock_helper.h>
#include <mrpt/core/round.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/viz/CAssimpModel.h>

using namespace mrpt;
using namespace mrpt::viz;
using namespace mrpt::math;
using mrpt::img::CImage;

IMPLEMENTS_SERIALIZABLE(CAssimpModel, CVisualObject, mrpt::viz)

uint8_t CAssimpModel::serializeGetVersion() const { return 0; }
void CAssimpModel::serializeTo(mrpt::serialization::CArchive& out) const
{
  writeToStreamRender(out);
  out << m_modelPath;                       // v2
  out << m_split_triangles_rendering_bbox;  // v3
}

void CAssimpModel::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 0:
    {
      readFromStreamRender(in);
      clear();

      in >> m_modelPath;
      in >> m_split_triangles_rendering_bbox;
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };
  CVisualObject::notifyChange();
}

CAssimpModel::CAssimpModel() {}

CAssimpModel::~CAssimpModel() { clear(); }

void CAssimpModel::clear() { m_modelPath.clear(); }

void CAssimpModel::loadScene(const std::string& filepath, int flags)
{
  clear();
  CVisualObject::notifyChange();

  m_modelLoadFlags = flags;
  m_modelPath = filepath;
}

auto CAssimpModel::internalBoundingBoxLocal() const -> mrpt::math::TBoundingBoxf
{
  THROW_EXCEPTION("Not implemented");
}

void CAssimpModel::split_triangles_rendering_bbox(const float bbox_size)
{
  m_split_triangles_rendering_bbox = bbox_size;
}

bool CAssimpModel::traceRay(
    [[maybe_unused]] const mrpt::poses::CPose3D& o, [[maybe_unused]] double& dist) const
{
  // TODO
  return false;
}
