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
#include "CBaseObservationNode.h"

#include "mrpt/obs/CObservationStereoImages.h"

class CObservationStereoImagesNode : public CBaseObservationNode
{
   public:
	CObservationStereoImagesNode(
		CNode* parent, mrpt::obs::CObservationStereoImages::Ptr observation,
		const mrpt::poses::CPose3D& pose);

	// CNode interface
	ObjectType type() const override;
	std::string displayName() const override;

	mrpt::obs::CObservationStereoImages::Ptr observation() const;

   private:
	mrpt::obs::CObservationStereoImages::Ptr m_observation;
};
