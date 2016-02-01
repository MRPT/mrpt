#ifndef __SYSTEM_BINDINGS_H__
#define __SYSTEM_BINDINGS_H__

/* bindings */
#include "bindings.h"

/* MRPT */
#include <mrpt/system/datetime.h>

// time conversion
boost::python::object TTimeStamp_to_ROS_Time(boost::python::long_ timestamp);
boost::python::long_ TTimeStamp_from_ROS_Time(boost::python::object ros_time);

#endif
