/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/obs/CObservation2DRangeScan.h>
#include <string>

namespace mrpt::graphslam::detail
{
template <class GRAPH_T>
struct TNodeProps
{
	typename GRAPH_T::global_pose_t pose;
	mrpt::obs::CObservation2DRangeScan::Ptr scan;

	TNodeProps operator=(const TNodeProps& other)
	{
		this->pose = other.pose;
		this->scan = other.scan;
		return *this;
	}

	void getAsString(std::string* str) const
	{
		ASSERTDEB_(str);
		str->clear();
		*str += mrpt::format("Pose: %s|\t", this->pose.asString().c_str());
		if (this->scan)
		{
			*str += mrpt::format("Scan #%lu", this->scan->getScanSize());
		}
		else
		{
			*str += "Scan: NONE";
		}
		*str += "\n";
	}
	std::string getAsString() const
	{
		std::string str;
		this->getAsString(&str);
		return str;
	}

	friend std::ostream& operator<<(std::ostream& o, const TNodeProps& obj)
	{
		o << obj.getAsString() << endl;
		return o;
	}
};
}  // namespace mrpt::graphslam::detail
