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
#include "CBaseObservationNode.h"
#include "mrpt/obs/CObservationImage.h"

class CObservationImageNode : public CBaseObservationNode
{
   public:
	CObservationImageNode(
		CNode* parent, mrpt::obs::CObservationImage::Ptr observation,
		const mrpt::poses::CPose3D& pose);

	// CNode interface
	ObjectType type() const override;
	std::string displayName() const override;
	mrpt::obs::CObservationImage::Ptr observation() const;

   private:
	mrpt::obs::CObservationImage::Ptr m_observation;
};
