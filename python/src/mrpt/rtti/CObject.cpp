#include <chrono>
#include <iterator>
#include <memory>
#include <mrpt/containers/CDynamicGrid.h>
#include <mrpt/core/Clock.h>
#include <mrpt/core/aligned_allocator.h>
#include <mrpt/img/CImage.h>
#include <mrpt/maps/CBeacon.h>
#include <mrpt/maps/CBeaconMap.h>
#include <mrpt/maps/CColouredOctoMap.h>
#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/maps/CGasConcentrationGridMap2D.h>
#include <mrpt/maps/CHeightGridMap2D.h>
#include <mrpt/maps/CHeightGridMap2D_Base.h>
#include <mrpt/maps/CHeightGridMap2D_MRF.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/COccupancyGridMap3D.h>
#include <mrpt/maps/COctoMap.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/maps/CPointsMapXYZI.h>
#include <mrpt/maps/CRandomFieldGridMap2D.h>
#include <mrpt/maps/CReflectivityGridMap2D.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/CVoxelMap.h>
#include <mrpt/maps/CVoxelMapRGB.h>
#include <mrpt/maps/CWeightedPointsMap.h>
#include <mrpt/maps/CWirelessPowerGridMap2D.h>
#include <mrpt/maps/TMetricMapInitializer.h>
#include <mrpt/maps/metric_map_types.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationBeaconRanges.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationRange.h>
#include <mrpt/opengl/COctoMapVoxels.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/tfest/TMatchingPair.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <ostream>
#include <ratio>
#include <sstream> // __str__
#include <string>
#include <vector>

#include <functional>
#include <pybind11/pybind11.h>
#include <string>
#include <pybind11/stl.h>


#ifndef BINDER_PYBIND11_TYPE_CASTER
	#define BINDER_PYBIND11_TYPE_CASTER
	PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>)
	PYBIND11_DECLARE_HOLDER_TYPE(T, T*)
	PYBIND11_MAKE_OPAQUE(std::shared_ptr<void>)
#endif

void bind_mrpt_rtti_CObject(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::rtti::TRuntimeClassId file:mrpt/rtti/CObject.h line:32
		pybind11::class_<mrpt::rtti::TRuntimeClassId, std::shared_ptr<mrpt::rtti::TRuntimeClassId>> cl(M("mrpt::rtti"), "TRuntimeClassId", "A structure that holds runtime class type information. Use\n CLASS_ID(<class_name>) to get a reference to the class_name's TRuntimeClassId\n descriptor.\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::rtti::TRuntimeClassId(); } ) );
		cl.def( pybind11::init( [](mrpt::rtti::TRuntimeClassId const &o){ return new mrpt::rtti::TRuntimeClassId(o); } ) );
		cl.def_readwrite("ptrCreateObject", &mrpt::rtti::TRuntimeClassId::ptrCreateObject);
		cl.def("createObject", (class std::shared_ptr<class mrpt::rtti::CObject> (mrpt::rtti::TRuntimeClassId::*)() const) &mrpt::rtti::TRuntimeClassId::createObject, "C++: mrpt::rtti::TRuntimeClassId::createObject() const --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("derivedFrom", (bool (mrpt::rtti::TRuntimeClassId::*)(const struct mrpt::rtti::TRuntimeClassId *) const) &mrpt::rtti::TRuntimeClassId::derivedFrom, "C++: mrpt::rtti::TRuntimeClassId::derivedFrom(const struct mrpt::rtti::TRuntimeClassId *) const --> bool", pybind11::arg("pBaseClass"));
		cl.def("derivedFrom", (bool (mrpt::rtti::TRuntimeClassId::*)(const char *) const) &mrpt::rtti::TRuntimeClassId::derivedFrom, "C++: mrpt::rtti::TRuntimeClassId::derivedFrom(const char *) const --> bool", pybind11::arg("pBaseClass_name"));
		cl.def("assign", (struct mrpt::rtti::TRuntimeClassId & (mrpt::rtti::TRuntimeClassId::*)(const struct mrpt::rtti::TRuntimeClassId &)) &mrpt::rtti::TRuntimeClassId::operator=, "C++: mrpt::rtti::TRuntimeClassId::operator=(const struct mrpt::rtti::TRuntimeClassId &) --> struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	// mrpt::rtti::registerClass(const struct mrpt::rtti::TRuntimeClassId *) file:mrpt/rtti/CObject.h line:51
	M("mrpt::rtti").def("registerClass", (void (*)(const struct mrpt::rtti::TRuntimeClassId *)) &mrpt::rtti::registerClass, "Register a class into the MRPT internal list of \"CObject\" descendents.\n  Used internally in the macros DEFINE_SERIALIZABLE, etc...\n \n\n getAllRegisteredClasses\n\nC++: mrpt::rtti::registerClass(const struct mrpt::rtti::TRuntimeClassId *) --> void", pybind11::arg("pNewClass"));

	// mrpt::rtti::registerClassCustomName(const char *, const struct mrpt::rtti::TRuntimeClassId *) file:mrpt/rtti/CObject.h line:57
	M("mrpt::rtti").def("registerClassCustomName", (void (*)(const char *, const struct mrpt::rtti::TRuntimeClassId *)) &mrpt::rtti::registerClassCustomName, "Mostly for internal use within mrpt sources, to handle exceptional cases\n with multiple serialization names for backward compatibility\n (CMultiMetricMaps, CImage,...)\n\nC++: mrpt::rtti::registerClassCustomName(const char *, const struct mrpt::rtti::TRuntimeClassId *) --> void", pybind11::arg("customName"), pybind11::arg("pNewClass"));

	// mrpt::rtti::findRegisteredClass(const std::string &, const bool) file:mrpt/rtti/CObject.h line:87
	M("mrpt::rtti").def("findRegisteredClass", [](const std::string & a0) -> const mrpt::rtti::TRuntimeClassId * { return mrpt::rtti::findRegisteredClass(a0); }, "", pybind11::return_value_policy::automatic, pybind11::arg("className"));
	M("mrpt::rtti").def("findRegisteredClass", (const struct mrpt::rtti::TRuntimeClassId * (*)(const std::string &, const bool)) &mrpt::rtti::findRegisteredClass, "Return info about a given class by its name, or nullptr if the class is not\n registered.\n\n The list of registered \"namespaces::class_name\" will be looked up first. If\n no match is found, **and** `allow_ignore_namespace=true`, then a second\n search will be performed looking for a match of the class name without the\n namespace part. Note that this is enabled by default since namespaces were\n not used while serializing classes in MRPT older than v2.0, so this option\n allows reading from older datasets transparently. It could be set to false if\n it is ensured that only mrpt2 datasets will be read.\n\n \n The name of the class to look up\n \n\n See discussion above\n\n \n registerClass, getAllRegisteredClasses\n\nC++: mrpt::rtti::findRegisteredClass(const std::string &, const bool) --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic, pybind11::arg("className"), pybind11::arg("allow_ignore_namespace"));

	{ // mrpt::rtti::CLASS_ID_impl file:mrpt/rtti/CObject.h line:91
		pybind11::class_<mrpt::rtti::CLASS_ID_impl<mrpt::rtti::CObject>, std::shared_ptr<mrpt::rtti::CLASS_ID_impl<mrpt::rtti::CObject>>> cl(M("mrpt::rtti"), "CLASS_ID_impl_mrpt_rtti_CObject_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::rtti::CLASS_ID_impl<mrpt::rtti::CObject>(); } ) );
		cl.def_static("get", (const struct mrpt::rtti::TRuntimeClassId * (*)()) &mrpt::rtti::CLASS_ID_impl<mrpt::rtti::CObject>::get, "C++: mrpt::rtti::CLASS_ID_impl<mrpt::rtti::CObject>::get() --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::rtti::CLASS_ID_impl file:mrpt/rtti/CObject.h line:91
		pybind11::class_<mrpt::rtti::CLASS_ID_impl<mrpt::maps::CBeaconMap>, std::shared_ptr<mrpt::rtti::CLASS_ID_impl<mrpt::maps::CBeaconMap>>> cl(M("mrpt::rtti"), "CLASS_ID_impl_mrpt_maps_CBeaconMap_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::rtti::CLASS_ID_impl<mrpt::maps::CBeaconMap>(); } ) );
		cl.def_static("get", (const struct mrpt::rtti::TRuntimeClassId * (*)()) &mrpt::rtti::CLASS_ID_impl<mrpt::maps::CBeaconMap>::get, "C++: mrpt::rtti::CLASS_ID_impl<mrpt::maps::CBeaconMap>::get() --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::rtti::CLASS_ID_impl file:mrpt/rtti/CObject.h line:91
		pybind11::class_<mrpt::rtti::CLASS_ID_impl<mrpt::maps::CColouredOctoMap>, std::shared_ptr<mrpt::rtti::CLASS_ID_impl<mrpt::maps::CColouredOctoMap>>> cl(M("mrpt::rtti"), "CLASS_ID_impl_mrpt_maps_CColouredOctoMap_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::rtti::CLASS_ID_impl<mrpt::maps::CColouredOctoMap>(); } ) );
		cl.def_static("get", (const struct mrpt::rtti::TRuntimeClassId * (*)()) &mrpt::rtti::CLASS_ID_impl<mrpt::maps::CColouredOctoMap>::get, "C++: mrpt::rtti::CLASS_ID_impl<mrpt::maps::CColouredOctoMap>::get() --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::rtti::CLASS_ID_impl file:mrpt/rtti/CObject.h line:91
		pybind11::class_<mrpt::rtti::CLASS_ID_impl<mrpt::maps::CColouredPointsMap>, std::shared_ptr<mrpt::rtti::CLASS_ID_impl<mrpt::maps::CColouredPointsMap>>> cl(M("mrpt::rtti"), "CLASS_ID_impl_mrpt_maps_CColouredPointsMap_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::rtti::CLASS_ID_impl<mrpt::maps::CColouredPointsMap>(); } ) );
		cl.def_static("get", (const struct mrpt::rtti::TRuntimeClassId * (*)()) &mrpt::rtti::CLASS_ID_impl<mrpt::maps::CColouredPointsMap>::get, "C++: mrpt::rtti::CLASS_ID_impl<mrpt::maps::CColouredPointsMap>::get() --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::rtti::CLASS_ID_impl file:mrpt/rtti/CObject.h line:91
		pybind11::class_<mrpt::rtti::CLASS_ID_impl<mrpt::maps::CGasConcentrationGridMap2D>, std::shared_ptr<mrpt::rtti::CLASS_ID_impl<mrpt::maps::CGasConcentrationGridMap2D>>> cl(M("mrpt::rtti"), "CLASS_ID_impl_mrpt_maps_CGasConcentrationGridMap2D_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::rtti::CLASS_ID_impl<mrpt::maps::CGasConcentrationGridMap2D>(); } ) );
		cl.def_static("get", (const struct mrpt::rtti::TRuntimeClassId * (*)()) &mrpt::rtti::CLASS_ID_impl<mrpt::maps::CGasConcentrationGridMap2D>::get, "C++: mrpt::rtti::CLASS_ID_impl<mrpt::maps::CGasConcentrationGridMap2D>::get() --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::rtti::CLASS_ID_impl file:mrpt/rtti/CObject.h line:91
		pybind11::class_<mrpt::rtti::CLASS_ID_impl<mrpt::maps::CHeightGridMap2D>, std::shared_ptr<mrpt::rtti::CLASS_ID_impl<mrpt::maps::CHeightGridMap2D>>> cl(M("mrpt::rtti"), "CLASS_ID_impl_mrpt_maps_CHeightGridMap2D_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::rtti::CLASS_ID_impl<mrpt::maps::CHeightGridMap2D>(); } ) );
		cl.def_static("get", (const struct mrpt::rtti::TRuntimeClassId * (*)()) &mrpt::rtti::CLASS_ID_impl<mrpt::maps::CHeightGridMap2D>::get, "C++: mrpt::rtti::CLASS_ID_impl<mrpt::maps::CHeightGridMap2D>::get() --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::rtti::CLASS_ID_impl file:mrpt/rtti/CObject.h line:91
		pybind11::class_<mrpt::rtti::CLASS_ID_impl<mrpt::maps::CHeightGridMap2D_MRF>, std::shared_ptr<mrpt::rtti::CLASS_ID_impl<mrpt::maps::CHeightGridMap2D_MRF>>> cl(M("mrpt::rtti"), "CLASS_ID_impl_mrpt_maps_CHeightGridMap2D_MRF_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::rtti::CLASS_ID_impl<mrpt::maps::CHeightGridMap2D_MRF>(); } ) );
		cl.def_static("get", (const struct mrpt::rtti::TRuntimeClassId * (*)()) &mrpt::rtti::CLASS_ID_impl<mrpt::maps::CHeightGridMap2D_MRF>::get, "C++: mrpt::rtti::CLASS_ID_impl<mrpt::maps::CHeightGridMap2D_MRF>::get() --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::rtti::CLASS_ID_impl file:mrpt/rtti/CObject.h line:91
		pybind11::class_<mrpt::rtti::CLASS_ID_impl<mrpt::maps::COccupancyGridMap2D>, std::shared_ptr<mrpt::rtti::CLASS_ID_impl<mrpt::maps::COccupancyGridMap2D>>> cl(M("mrpt::rtti"), "CLASS_ID_impl_mrpt_maps_COccupancyGridMap2D_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::rtti::CLASS_ID_impl<mrpt::maps::COccupancyGridMap2D>(); } ) );
		cl.def_static("get", (const struct mrpt::rtti::TRuntimeClassId * (*)()) &mrpt::rtti::CLASS_ID_impl<mrpt::maps::COccupancyGridMap2D>::get, "C++: mrpt::rtti::CLASS_ID_impl<mrpt::maps::COccupancyGridMap2D>::get() --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::rtti::CLASS_ID_impl file:mrpt/rtti/CObject.h line:91
		pybind11::class_<mrpt::rtti::CLASS_ID_impl<mrpt::maps::COccupancyGridMap3D>, std::shared_ptr<mrpt::rtti::CLASS_ID_impl<mrpt::maps::COccupancyGridMap3D>>> cl(M("mrpt::rtti"), "CLASS_ID_impl_mrpt_maps_COccupancyGridMap3D_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::rtti::CLASS_ID_impl<mrpt::maps::COccupancyGridMap3D>(); } ) );
		cl.def_static("get", (const struct mrpt::rtti::TRuntimeClassId * (*)()) &mrpt::rtti::CLASS_ID_impl<mrpt::maps::COccupancyGridMap3D>::get, "C++: mrpt::rtti::CLASS_ID_impl<mrpt::maps::COccupancyGridMap3D>::get() --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::rtti::CLASS_ID_impl file:mrpt/rtti/CObject.h line:91
		pybind11::class_<mrpt::rtti::CLASS_ID_impl<mrpt::maps::COctoMap>, std::shared_ptr<mrpt::rtti::CLASS_ID_impl<mrpt::maps::COctoMap>>> cl(M("mrpt::rtti"), "CLASS_ID_impl_mrpt_maps_COctoMap_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::rtti::CLASS_ID_impl<mrpt::maps::COctoMap>(); } ) );
		cl.def_static("get", (const struct mrpt::rtti::TRuntimeClassId * (*)()) &mrpt::rtti::CLASS_ID_impl<mrpt::maps::COctoMap>::get, "C++: mrpt::rtti::CLASS_ID_impl<mrpt::maps::COctoMap>::get() --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::rtti::CLASS_ID_impl file:mrpt/rtti/CObject.h line:91
		pybind11::class_<mrpt::rtti::CLASS_ID_impl<mrpt::maps::CSimplePointsMap>, std::shared_ptr<mrpt::rtti::CLASS_ID_impl<mrpt::maps::CSimplePointsMap>>> cl(M("mrpt::rtti"), "CLASS_ID_impl_mrpt_maps_CSimplePointsMap_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::rtti::CLASS_ID_impl<mrpt::maps::CSimplePointsMap>(); } ) );
		cl.def_static("get", (const struct mrpt::rtti::TRuntimeClassId * (*)()) &mrpt::rtti::CLASS_ID_impl<mrpt::maps::CSimplePointsMap>::get, "C++: mrpt::rtti::CLASS_ID_impl<mrpt::maps::CSimplePointsMap>::get() --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::rtti::CLASS_ID_impl file:mrpt/rtti/CObject.h line:91
		pybind11::class_<mrpt::rtti::CLASS_ID_impl<mrpt::maps::CPointsMapXYZI>, std::shared_ptr<mrpt::rtti::CLASS_ID_impl<mrpt::maps::CPointsMapXYZI>>> cl(M("mrpt::rtti"), "CLASS_ID_impl_mrpt_maps_CPointsMapXYZI_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::rtti::CLASS_ID_impl<mrpt::maps::CPointsMapXYZI>(); } ) );
		cl.def_static("get", (const struct mrpt::rtti::TRuntimeClassId * (*)()) &mrpt::rtti::CLASS_ID_impl<mrpt::maps::CPointsMapXYZI>::get, "C++: mrpt::rtti::CLASS_ID_impl<mrpt::maps::CPointsMapXYZI>::get() --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::rtti::CLASS_ID_impl file:mrpt/rtti/CObject.h line:91
		pybind11::class_<mrpt::rtti::CLASS_ID_impl<mrpt::maps::CReflectivityGridMap2D>, std::shared_ptr<mrpt::rtti::CLASS_ID_impl<mrpt::maps::CReflectivityGridMap2D>>> cl(M("mrpt::rtti"), "CLASS_ID_impl_mrpt_maps_CReflectivityGridMap2D_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::rtti::CLASS_ID_impl<mrpt::maps::CReflectivityGridMap2D>(); } ) );
		cl.def_static("get", (const struct mrpt::rtti::TRuntimeClassId * (*)()) &mrpt::rtti::CLASS_ID_impl<mrpt::maps::CReflectivityGridMap2D>::get, "C++: mrpt::rtti::CLASS_ID_impl<mrpt::maps::CReflectivityGridMap2D>::get() --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::rtti::CLASS_ID_impl file:mrpt/rtti/CObject.h line:91
		pybind11::class_<mrpt::rtti::CLASS_ID_impl<mrpt::maps::CWeightedPointsMap>, std::shared_ptr<mrpt::rtti::CLASS_ID_impl<mrpt::maps::CWeightedPointsMap>>> cl(M("mrpt::rtti"), "CLASS_ID_impl_mrpt_maps_CWeightedPointsMap_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::rtti::CLASS_ID_impl<mrpt::maps::CWeightedPointsMap>(); } ) );
		cl.def_static("get", (const struct mrpt::rtti::TRuntimeClassId * (*)()) &mrpt::rtti::CLASS_ID_impl<mrpt::maps::CWeightedPointsMap>::get, "C++: mrpt::rtti::CLASS_ID_impl<mrpt::maps::CWeightedPointsMap>::get() --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::rtti::CLASS_ID_impl file:mrpt/rtti/CObject.h line:91
		pybind11::class_<mrpt::rtti::CLASS_ID_impl<mrpt::maps::CWirelessPowerGridMap2D>, std::shared_ptr<mrpt::rtti::CLASS_ID_impl<mrpt::maps::CWirelessPowerGridMap2D>>> cl(M("mrpt::rtti"), "CLASS_ID_impl_mrpt_maps_CWirelessPowerGridMap2D_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::rtti::CLASS_ID_impl<mrpt::maps::CWirelessPowerGridMap2D>(); } ) );
		cl.def_static("get", (const struct mrpt::rtti::TRuntimeClassId * (*)()) &mrpt::rtti::CLASS_ID_impl<mrpt::maps::CWirelessPowerGridMap2D>::get, "C++: mrpt::rtti::CLASS_ID_impl<mrpt::maps::CWirelessPowerGridMap2D>::get() --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::rtti::CLASS_ID_impl file:mrpt/rtti/CObject.h line:91
		pybind11::class_<mrpt::rtti::CLASS_ID_impl<mrpt::maps::CVoxelMap>, std::shared_ptr<mrpt::rtti::CLASS_ID_impl<mrpt::maps::CVoxelMap>>> cl(M("mrpt::rtti"), "CLASS_ID_impl_mrpt_maps_CVoxelMap_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::rtti::CLASS_ID_impl<mrpt::maps::CVoxelMap>(); } ) );
		cl.def_static("get", (const struct mrpt::rtti::TRuntimeClassId * (*)()) &mrpt::rtti::CLASS_ID_impl<mrpt::maps::CVoxelMap>::get, "C++: mrpt::rtti::CLASS_ID_impl<mrpt::maps::CVoxelMap>::get() --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::rtti::CLASS_ID_impl file:mrpt/rtti/CObject.h line:91
		pybind11::class_<mrpt::rtti::CLASS_ID_impl<mrpt::maps::CVoxelMapRGB>, std::shared_ptr<mrpt::rtti::CLASS_ID_impl<mrpt::maps::CVoxelMapRGB>>> cl(M("mrpt::rtti"), "CLASS_ID_impl_mrpt_maps_CVoxelMapRGB_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::rtti::CLASS_ID_impl<mrpt::maps::CVoxelMapRGB>(); } ) );
		cl.def_static("get", (const struct mrpt::rtti::TRuntimeClassId * (*)()) &mrpt::rtti::CLASS_ID_impl<mrpt::maps::CVoxelMapRGB>::get, "C++: mrpt::rtti::CLASS_ID_impl<mrpt::maps::CVoxelMapRGB>::get() --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::rtti::CLASS_ID_impl file:mrpt/rtti/CObject.h line:91
		pybind11::class_<mrpt::rtti::CLASS_ID_impl<mrpt::obs::CObservation>, std::shared_ptr<mrpt::rtti::CLASS_ID_impl<mrpt::obs::CObservation>>> cl(M("mrpt::rtti"), "CLASS_ID_impl_mrpt_obs_CObservation_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::rtti::CLASS_ID_impl<mrpt::obs::CObservation>(); } ) );
		cl.def_static("get", (const struct mrpt::rtti::TRuntimeClassId * (*)()) &mrpt::rtti::CLASS_ID_impl<mrpt::obs::CObservation>::get, "C++: mrpt::rtti::CLASS_ID_impl<mrpt::obs::CObservation>::get() --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
	}
}
