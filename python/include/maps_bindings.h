#ifndef __MAPS_BINDINGS_H__
#define __MAPS_BINDINGS_H__

/* MRPT */
#include <mrpt/maps/CMetricMap.h>

/* BOOST */
#include <boost/python.hpp>

// CMetricMap
struct CMetricMapWrap : mrpt::maps::CMetricMap, boost::python::wrapper<mrpt::maps::CMetricMap>
{
    void internal_clear();
    bool isEmpty();
    double computeObservationLikelihood(const mrpt::obs::CObservation *obs, const mrpt::poses::CPose3D &takenFrom);
    void saveMetricMapRepresentationToFile(const std::string &filNamePrefix);
    void getAs3DObject(mrpt::opengl::CSetOfObjectsPtr &outObj);
};


#endif
