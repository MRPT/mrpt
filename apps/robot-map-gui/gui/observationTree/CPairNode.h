/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          https://www.mrpt.org/                            |
   |                                                                           |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file        |
   | See: https://www.mrpt.org/Authors - All rights reserved.                  |
   | Released under BSD License. See details in https://www.mrpt.org/License   |
   +---------------------------------------------------------------------------+
   */
#pragma once
#include <string>

#include "CNode.h"
#include "mrpt/maps/CSimpleMap.h"

class CObservationsNode;
class CPosesNode;

class CPairNode : public CNode
{
 public:
  CPairNode(
      CNode* parent,
      const mrpt::maps::CSimpleMap::Keyframe& poseSensFramePair,
      size_t indexInSimpleMap);
  ~CPairNode() override;

  int childCount() const override;

  CNode* child(int id) override;
  CNode* child(int id) const;
  ObjectType type() const override;
  std::string displayName() const override;

 private:
  CNode* getChild(int id) const;

  std::unique_ptr<CPosesNode> m_pose;
  std::unique_ptr<CObservationsNode> m_observations;
  size_t m_indexInSimpleMap;
};
