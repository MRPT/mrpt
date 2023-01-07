/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          https://www.mrpt.org/                            |
   |                                                                           |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file        |
   | See: https://www.mrpt.org/Authors - All rights reserved.                  |
   | Released under BSD License. See details in https://www.mrpt.org/License   |
   +---------------------------------------------------------------------------+
   */
#pragma once
#include "mrpt/opengl/CSetOfObjects.h"

class CRobotPose : public mrpt::opengl::CSetOfObjects
{
   public:
	using Ptr = std::shared_ptr<CRobotPose>;
	CRobotPose(size_t id);

	~CRobotPose() override = default;
	size_t getId() const;

	void setSelected(bool is);

   private:
	CSetOfObjects::Ptr m_currentObj;
	size_t m_id;
};
