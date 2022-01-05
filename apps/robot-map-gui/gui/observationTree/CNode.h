/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          https://www.mrpt.org/                            |
   |                                                                           |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file        |
   | See: https://www.mrpt.org/Authors - All rights reserved.                  |
   | Released under BSD License. See details in https://www.mrpt.org/License   |
   +---------------------------------------------------------------------------+
   */
#pragma once
#include <memory>
#include <string>

class CNode
{
   public:
	enum class ObjectType
	{
		Root = 0,
		PosWithObservationPair = 1,
		Observations = 3,
		Pos = 4,
		RangeScan = 5,
		Image = 6,
		StereoImage = 7

	};

	CNode(CNode* parent);
	virtual ~CNode() = default;

	virtual int childCount() const = 0;
	virtual CNode* child(int id) = 0;
	virtual ObjectType type() const = 0;
	virtual std::string displayName() const = 0;

	virtual const CNode* parentItem() const;

	/// Add new child to node. Optional to override.
	virtual void addNewChild();

	const CNode* child(int id) const;

   private:
	CNode* m_parent;
};
