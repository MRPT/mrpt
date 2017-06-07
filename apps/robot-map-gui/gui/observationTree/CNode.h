#pragma once
#include <string>


class CNode
{
public:
	enum class TypeObject
	{
		Node = 0,
		Pose = 1,
		Frame = 2
	};

	CNode(const std::string& name);
	virtual ~CNode() = default;

	virtual int childCount() const = 0;
	virtual const CNode* parentItem() const = 0;
	virtual CNode* child(int id) = 0;
	virtual void addNewChild() = 0;

	const CNode* child(int id) const;

	virtual TypeObject type() const;
	virtual const std::string& displayName() const;
private:

	std::string name_;
};


