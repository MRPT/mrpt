/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#include "bindings.h"

/* MRPT */
#include <mrpt/nav/planners/PlannerSimple2D.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/poses/CPose2D.h>

/* namespaces */
using namespace boost::python;
using namespace mrpt::nav;
using namespace mrpt::poses;
using namespace mrpt::maps;

// PlannerSimple2D
tuple PlannerSimple2D_computePath(
	PlannerSimple2D& self, COccupancyGridMap2D& theMap, CPose2D& origin,
	CPose2D& target)
{
	// function args
	std::deque<mrpt::math::TPoint2D> path;
	bool notFound;
	// invoke function
	self.computePath(theMap, origin, target, path, notFound);
	// convert to python compatible
	boost::python::list ret_val;
	boost::python::list py_path;
	for (auto i : path)
	{
		py_path.append(i);
	}
	// setup return value
	ret_val.append(py_path);
	ret_val.append(notFound);
	return tuple(ret_val);
}
// end of PlannerSimple2D

// exporter
void export_nav()
{
	// map namespace to be submodule of mrpt package
	MAKE_SUBMODULE(nav)

	// PlannerSimple2D
	{
		class_<PlannerSimple2D>("PlannerSimple2D", init<>())
			.def(
				"computePath", &PlannerSimple2D_computePath,
				"This method compute the optimal path for a circular robot, in "
				"the given occupancy grid map, from the origin location to a "
				"target point.")
			.def_readwrite("robotRadius", &PlannerSimple2D::robotRadius);
	}
}
