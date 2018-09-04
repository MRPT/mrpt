/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
   */
#pragma once
#include <string>

#include "CNode.h"

#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>

class CPosesNode : public CNode
{
   public:
	CPosesNode(CNode* parent, const mrpt::poses::CPose3D& pose);
	~CPosesNode() override = default;

	int childCount() const override;
	CNode* child(int id) override;
	std::string displayName() const override;
	ObjectType type() const override;

	mrpt::poses::CPose3D getPose() const;

   private:
	mrpt::poses::CPose3D m_pose;
};
