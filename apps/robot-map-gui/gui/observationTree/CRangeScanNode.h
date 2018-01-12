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

#include "mrpt/obs/CObservation2DRangeScan.h"

class CRangeScanNode : public CBaseObservationNode
{
   public:
	CRangeScanNode(
		CNode* parent, mrpt::obs::CObservation2DRangeScan::Ptr observation,
		const mrpt::poses::CPose3D& pose);

	// CNode interface
	ObjectType type() const override;
	std::string displayName() const override;

	mrpt::obs::CObservation2DRangeScan::Ptr observation() const;

   private:
	mrpt::obs::CObservation2DRangeScan::Ptr m_observation;
};
