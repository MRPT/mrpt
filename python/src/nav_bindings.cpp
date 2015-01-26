/* bindings */
#include "bindings.h"

/* MRPT */
#include <mrpt/nav/planners/CPathPlanningCircularRobot.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/poses/CPose2D.h>

/* namespaces */
using namespace boost::python;
using namespace mrpt::nav;
using namespace mrpt::poses;
using namespace mrpt::maps;

// CPathPlanningCircularRobot
tuple CPathPlanningCircularRobot_computePath(CPathPlanningCircularRobot &self, COccupancyGridMap2D &theMap, CPose2D &origin, CPose2D &target)
{
    // function args
    std::deque<mrpt::math::TPoint2D> path;
    bool notFound;
    // invoke function
    self.computePath(theMap, origin, target, path, notFound);
    // convert to python compatible
    boost::python::list ret_val;
    boost::python::list py_path;
    for (uint32_t i = 0; i < path.size(); ++i) {
        py_path.append(path[i]);
    }
    // setup return value
    ret_val.append(py_path);
    ret_val.append(notFound);
    return tuple(ret_val);
}
// end of CPathPlanningCircularRobot


// exporter
void export_nav()
{
    // map namespace to be submodule of mrpt package
    MAKE_SUBMODULE(nav)

    // CPathPlanningCircularRobot
    {
        class_<CPathPlanningCircularRobot>("CPathPlanningCircularRobot", init<>())
            .def("computePath", &CPathPlanningCircularRobot_computePath, "This method compute the optimal path for a circular robot, in the given occupancy grid map, from the origin location to a target point.")
            .def_readwrite("robotRadius", &CPathPlanningCircularRobot::robotRadius)
        ;
    }
}
