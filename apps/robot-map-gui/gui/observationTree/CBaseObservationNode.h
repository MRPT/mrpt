/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include "CNode.h"


class CBaseObservationNode : public CNode
{
public:

	CBaseObservationNode(CNode* parent);
	virtual ~CBaseObservationNode() = default;
	// CNode interface
	int childCount() const override;
	CNode *child(int id) override;
};
