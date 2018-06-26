/* bindings */
#include "bindings.h"

/* MRPT */
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/CMultiMetricMapPDF.h>
#include <mrpt/maps/TMetricMapInitializer.h>
#include <mrpt/maps/metric_map_types.h>

#include <mrpt/poses/CPosePDFGaussian.h>

#include <mrpt/system/datetime.h>

#include <mrpt/opengl/CSetOfObjects.h>

/* std */
#include <stdint.h>

/* namespaces */
using namespace boost::python;
using namespace mrpt::opengl;
using namespace mrpt::poses;
using namespace mrpt::slam;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::utils;

// CMultiMetricMapPDF
CPose2D CMultiMetricMapPDF_getLastPose(CMultiMetricMapPDF& self, size_t i)
{
	bool is_valid;
	mrpt::math::TPose3D last_pose = self.getLastPose(i,is_valid);
	return CPose2D(CPose3D(last_pose));
}

list CMultiMetricMapPDF_getPath(CMultiMetricMapPDF& self, size_t i)
{
    std::deque<mrpt::math::TPose3D> path;
    list ret_val;
    self.getPath(i, path);
    for (size_t i = 0; i < path.size(); ++i) ret_val.append(path[i]);
    return ret_val;
}

CPose3DPDFParticles CMultiMetricMapPDF_getEstimatedPosePDFAtTime(CMultiMetricMapPDF& self, size_t timeStep)
{
    CPose3DPDFParticles out_estimation;
    self.getEstimatedPosePDFAtTime(timeStep, out_estimation);
    return out_estimation;
}
// end of CMultiMetricMapPDF

// TSetOfMetricMapInitializers
void TSetOfMetricMapInitializers_push_back(TSetOfMetricMapInitializers& self, TMetricMapInitializerPtr& o)
{
    self.push_back(o);
}
// end of TSetOfMetricMapInitializers

// CMultiMetricMap
mrpt::opengl::CSetOfObjectsPtr CMultiMetricMap_getAs3DObject(CMultiMetricMap &self)
{
    mrpt::opengl::CSetOfObjectsPtr outObj = mrpt::opengl::CSetOfObjects::Create();
    self.getAs3DObject(outObj);
    return outObj;
}

void CMultiMetricMap_setListOfMaps(CMultiMetricMap& self, TSetOfMetricMapInitializers& initializers)
{
    self.setListOfMaps(initializers);
}

CSimplePointsMapPtr CMultiMetricMap_getAsSimplePointsMap(CMultiMetricMap& self)
{
    CSimplePointsMapPtr points_map = CSimplePointsMapPtr(new CSimplePointsMap);
    CSimplePointsMap* points_map_ptr = self.getAsSimplePointsMap();
    *points_map = *points_map_ptr;
    return points_map;
}
// end of CMultiMetricMap

// smart pointer contents
MAKE_PTR_CTX(CMultiMetricMap)
MAKE_PTR_CTX(CMultiMetricMapPDF)


void export_maps2()
{
    // map namespace to be submodule of package
    MAKE_SUBMODULE(maps)

    // TMetricMapInitializer
    {
        class_<TMetricMapInitializerPtr>("TMetricMapInitializerPtr", init<TMetricMapInitializer*>())
        ;

        scope s = class_<TMetricMapInitializer, boost::noncopyable, bases<CLoadableOptions> >("TMetricMapInitializer", no_init)
            .def("factory", &TMetricMapInitializer::factory, return_value_policy<manage_new_object>()).staticmethod("factory")
        ;
    }

    // TSetOfMetricMapInitializers
    {
        class_<TSetOfMetricMapInitializers, bases<CLoadableOptions> >("TSetOfMetricMapInitializers", init<>())
             .def("size", &TSetOfMetricMapInitializers::size)
             .def("push_back", &TSetOfMetricMapInitializers_push_back)
             .def("clear", &TSetOfMetricMapInitializers::clear)
        ;
    }

    // CMultiMetricMap
    {
        MAKE_PTR_BASE(CMultiMetricMap, CMetricMap)

        scope s = class_<CMultiMetricMap, bases<CMetricMap> >("CMultiMetricMap", init<optional<TSetOfMetricMapInitializers*> >())
            .def("getAs3DObject", &CMultiMetricMap_getAs3DObject, "Returns a 3D object representing the map.")
            .def("setListOfMaps", &CMultiMetricMap_setListOfMaps, "Sets the list of internal map according to the passed list of map initializers (Current maps' content will be deleted!).")
            .def("isEmpty", &CMultiMetricMap::isEmpty, "Returns true if all maps returns true to their isEmpty() method, which is map-dependent.")
            .def("getAsSimplePointsMap", &CMultiMetricMap_getAsSimplePointsMap, "If the map is a simple point map or it's a multi-metric map that contains EXACTLY one simple point map, return it. Otherwise, return None.")
            .def_readwrite("maps", &CMultiMetricMap::maps, "The list of MRPT metric maps in this object.")
            MAKE_CREATE(CMultiMetricMap)
        ;

        // TListMaps
        class_<CMultiMetricMap::TListMaps>("TListMaps", init<>())
            .def("__len__", &CMultiMetricMap::TListMaps::size)
            .def("clear", &CMultiMetricMap::TListMaps::clear)
            .def("append", &StlListLike<CMultiMetricMap::TListMaps>::add, with_custodian_and_ward<1,2>()) // to let container keep value
            .def("__getitem__", &StlListLike<CMultiMetricMap::TListMaps>::get, return_value_policy<copy_non_const_reference>())
            .def("__setitem__", &StlListLike<CMultiMetricMap::TListMaps>::set, with_custodian_and_ward<1,2>()) // to let container keep value
            .def("__delitem__", &StlListLike<CMultiMetricMap::TListMaps>::del)
        ;
    }

    // CMultiMetricMapPDF
    {
        MAKE_PTR(CMultiMetricMapPDF)

        scope s = class_<CMultiMetricMapPDF>("CMultiMetricMapPDF", init<>())
            .def("getLastPose", &CMultiMetricMapPDF_getLastPose, "Return the last robot pose for the i'th particle.")
            .def("getPath", &CMultiMetricMapPDF_getPath, "Return the path (in absolute coordinate poses) for the i'th particle.")
            .def("getEstimatedPosePDFAtTime", &CMultiMetricMapPDF_getEstimatedPosePDFAtTime, "Returns the estimate of the robot pose as a particles PDF for the instant of time \"timeStep\", from 0 to N-1.")
            .def("getCurrentMostLikelyMetricMap", &CMultiMetricMapPDF::getCurrentMostLikelyMetricMap, return_value_policy<reference_existing_object>(), "Returns a pointer to the current most likely map (associated to the most likely particle).")
            .def_readwrite("options", &CMultiMetricMapPDF::options)
            MAKE_CREATE(CMultiMetricMapPDF)
        ;

        // TConfigParams
        class_<CMultiMetricMapPDF::TPredictionParams, bases<CLoadableOptions> >("TPredictionParams", init<>())
            .def_readwrite("pfOptimalProposal_mapSelection", &CMultiMetricMapPDF::TPredictionParams::pfOptimalProposal_mapSelection)
            .def_readwrite("ICPGlobalAlign_MinQuality", &CMultiMetricMapPDF::TPredictionParams::ICPGlobalAlign_MinQuality)
            .def_readwrite("update_gridMapLikelihoodOptions", &CMultiMetricMapPDF::TPredictionParams::update_gridMapLikelihoodOptions)
            .def_readwrite("KLD_params", &CMultiMetricMapPDF::TPredictionParams::KLD_params)
            .def_readwrite("icp_params", &CMultiMetricMapPDF::TPredictionParams::icp_params)
        ;
    }

}
