#pragma once
#include <string>

#include "CNode.h"

#include "mrpt/poses.h"


class CPoseNode :public CNode
{
public:
	CPoseNode(CNode *parent, const mrpt::poses::CPose3D &pose);
	~CPoseNode() = default;

	int childCount() const override;
	CNode* child(int id) override;
	std::string displayName() const override;
	ObjectType type() const override;

	mrpt::poses::CPose3D getPose() const;

private:
	mrpt::poses::CPose3D m_pose;

};


