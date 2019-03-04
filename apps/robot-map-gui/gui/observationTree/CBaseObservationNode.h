/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          https://www.mrpt.org/                            |
   |                                                                           |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file        |
   | See: https://www.mrpt.org/Authors - All rights reserved.                  |
   | Released under BSD License. See details in https://www.mrpt.org/License   |
   +---------------------------------------------------------------------------+
   */
#pragma once
#include "CNode.h"

#include "mrpt/poses/CPose3D.h"

class CBaseObservationNode : public CNode
{
   public:
	CBaseObservationNode(CNode* parent, const mrpt::poses::CPose3D& pose);
	~CBaseObservationNode() override = default;
	// CNode interface
	int childCount() const override;
	CNode* child(int id) override;

	mrpt::poses::CPose3D getPose() const;

   private:
	mrpt::poses::CPose3D m_pose;
};
