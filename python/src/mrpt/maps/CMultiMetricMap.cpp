#include <any>
#include <chrono>
#include <deque>
#include <functional>
#include <ios>
#include <istream>
#include <iterator>
#include <map>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/containers/CommentPosition.h>
#include <mrpt/containers/YamlEmitOptions.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/Clock.h>
#include <mrpt/core/aligned_allocator.h>
#include <mrpt/core/safe_pointers.h>
#include <mrpt/img/CImage.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/img/TColor.h>
#include <mrpt/img/color_maps.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/TMetricMapInitializer.h>
#include <mrpt/maps/metric_map_types.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CPolygon.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TPose3DQuat.h>
#include <mrpt/math/TTwist3D.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/T2DScanProperties.h>
#include <mrpt/obs/T3DPointsTo2DScanParams.h>
#include <mrpt/obs/TRangeImageFilter.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CText.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPointPDF.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPose3DQuatPDF.h>
#include <mrpt/poses/CPoseOrPoint.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/tfest/TMatchingPair.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <ostream>
#include <ratio>
#include <sstream> // __str__
#include <streambuf>
#include <string>
#include <string_view>
#include <type_traits>
#include <typeinfo>
#include <utility>
#include <variant>
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

// mrpt::maps::CMultiMetricMap file:mrpt/maps/CMultiMetricMap.h line:120
struct PyCallBack_mrpt_maps_CMultiMetricMap : public mrpt::maps::CMultiMetricMap {
	using mrpt::maps::CMultiMetricMap::CMultiMetricMap;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CMultiMetricMap *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CMultiMetricMap::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CMultiMetricMap *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CMultiMetricMap::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CMultiMetricMap *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CMultiMetricMap::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CMultiMetricMap *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMultiMetricMap::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CMultiMetricMap *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMultiMetricMap::serializeFrom(a0, a1);
	}
	bool isEmpty() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CMultiMetricMap *>(this), "isEmpty");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CMultiMetricMap::isEmpty();
	}
	void determineMatching2D(const class mrpt::maps::CMetricMap * a0, const class mrpt::poses::CPose2D & a1, class mrpt::tfest::TMatchingPairListTempl<float> & a2, const struct mrpt::maps::TMatchingParams & a3, struct mrpt::maps::TMatchingExtraResults & a4) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CMultiMetricMap *>(this), "determineMatching2D");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMultiMetricMap::determineMatching2D(a0, a1, a2, a3, a4);
	}
	float compute3DMatchingRatio(const class mrpt::maps::CMetricMap * a0, const class mrpt::poses::CPose3D & a1, const struct mrpt::maps::TMatchingRatioParams & a2) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CMultiMetricMap *>(this), "compute3DMatchingRatio");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<float>::value) {
				static pybind11::detail::override_caster_t<float> caster;
				return pybind11::detail::cast_ref<float>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<float>(std::move(o));
		}
		return CMultiMetricMap::compute3DMatchingRatio(a0, a1, a2);
	}
	void saveMetricMapRepresentationToFile(const std::string & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CMultiMetricMap *>(this), "saveMetricMapRepresentationToFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMultiMetricMap::saveMetricMapRepresentationToFile(a0);
	}
	void auxParticleFilterCleanUp() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CMultiMetricMap *>(this), "auxParticleFilterCleanUp");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMultiMetricMap::auxParticleFilterCleanUp();
	}
	void getVisualizationInto(class mrpt::opengl::CSetOfObjects & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CMultiMetricMap *>(this), "getVisualizationInto");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMultiMetricMap::getVisualizationInto(a0);
	}
	const class mrpt::maps::CSimplePointsMap * getAsSimplePointsMap() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CMultiMetricMap *>(this), "getAsSimplePointsMap");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const class mrpt::maps::CSimplePointsMap *>::value) {
				static pybind11::detail::override_caster_t<const class mrpt::maps::CSimplePointsMap *> caster;
				return pybind11::detail::cast_ref<const class mrpt::maps::CSimplePointsMap *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const class mrpt::maps::CSimplePointsMap *>(std::move(o));
		}
		return CMultiMetricMap::getAsSimplePointsMap();
	}
	std::string asString() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CMultiMetricMap *>(this), "asString");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CMultiMetricMap::asString();
	}
	void internal_clear() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CMultiMetricMap *>(this), "internal_clear");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMultiMetricMap::internal_clear();
	}
	bool internal_canComputeObservationLikelihood(const class mrpt::obs::CObservation & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CMultiMetricMap *>(this), "internal_canComputeObservationLikelihood");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CMultiMetricMap::internal_canComputeObservationLikelihood(a0);
	}
	double internal_computeObservationLikelihood(const class mrpt::obs::CObservation & a0, const class mrpt::poses::CPose3D & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CMultiMetricMap *>(this), "internal_computeObservationLikelihood");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CMultiMetricMap::internal_computeObservationLikelihood(a0, a1);
	}
	struct mrpt::math::TBoundingBox_<float> boundingBox() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CMultiMetricMap *>(this), "boundingBox");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TBoundingBox_<float>>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TBoundingBox_<float>> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TBoundingBox_<float>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TBoundingBox_<float>>(std::move(o));
		}
		return CMetricMap::boundingBox();
	}
	bool canComputeObservationLikelihood(const class mrpt::obs::CObservation & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CMultiMetricMap *>(this), "canComputeObservationLikelihood");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CMetricMap::canComputeObservationLikelihood(a0);
	}
	void determineMatching3D(const class mrpt::maps::CMetricMap * a0, const class mrpt::poses::CPose3D & a1, class mrpt::tfest::TMatchingPairListTempl<float> & a2, const struct mrpt::maps::TMatchingParams & a3, struct mrpt::maps::TMatchingExtraResults & a4) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CMultiMetricMap *>(this), "determineMatching3D");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMetricMap::determineMatching3D(a0, a1, a2, a3, a4);
	}
	float squareDistanceToClosestCorrespondence(float a0, float a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CMultiMetricMap *>(this), "squareDistanceToClosestCorrespondence");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<float>::value) {
				static pybind11::detail::override_caster_t<float> caster;
				return pybind11::detail::cast_ref<float>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<float>(std::move(o));
		}
		return CMetricMap::squareDistanceToClosestCorrespondence(a0, a1);
	}
};

void bind_mrpt_maps_CMultiMetricMap(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::maps::CMultiMetricMap file:mrpt/maps/CMultiMetricMap.h line:120
		pybind11::class_<mrpt::maps::CMultiMetricMap, std::shared_ptr<mrpt::maps::CMultiMetricMap>, PyCallBack_mrpt_maps_CMultiMetricMap, mrpt::maps::CMetricMap> cl(M("mrpt::maps"), "CMultiMetricMap", "This class stores any customizable set of metric maps.\n The internal metric maps can be accessed directly by the user as smart\npointers with CMultiMetricMap::mapByIndex() or via `iterator`s.\n The utility of this container is to operate on several maps simultaneously:\nupdate them by inserting observations,\n evaluate the likelihood of one observation by fusing (multiplying) the\nlikelihoods over the different maps, etc.\n\n These kinds of metric maps can be kept inside (list may be\n incomplete, refer to classes derived from mrpt::maps::CMetricMap):\n	- mrpt::maps::CSimplePointsMap: For 2D or 3D range scans, ...\n	- mrpt::maps::COccupancyGridMap2D: 2D, horizontal  laser range\n    scans, at different altitudes.\n	- mrpt::maps::COccupancyGridMap3D: 3D occupancy voxel map.\n	- mrpt::maps::COctoMap: For 3D occupancy grids of variable resolution,\n    with octrees (based on the library `octomap`).\n  - mrpt::maps::CVoxelMap or mrpt::maps::CVoxelMapRGB: 3D sparse voxel maps.\n	- mrpt::maps::CColouredOctoMap: The same than above, but nodes can store\n    RGB data appart from occupancy.\n	- mrpt::maps::CLandmarksMap: For visual landmarks,etc...\n	- mrpt::maps::CGasConcentrationGridMap2D: For gas concentration maps.\n	- mrpt::maps::CWirelessPowerGridMap2D: For wifi power maps.\n	- mrpt::maps::CBeaconMap: For range-only SLAM.\n	- mrpt::maps::CHeightGridMap2D: For elevation maps of height for each\n    (x,y) location (Digital elevation model, DEM)\n	- mrpt::maps::CHeightGridMap2D_MRF: DEMs as Markov Random Field (MRF)\n	- mrpt::maps::CReflectivityGridMap2D: For maps of \"reflectivity\" for\n    each (x,y) location.\n	- mrpt::maps::CColouredPointsMap: For point map with color.\n	- mrpt::maps::CWeightedPointsMap: For point map with weights (capable of\n    \"fusing\").\n\n See CMultiMetricMap::setListOfMaps() for the method for initializing this\nclass programmatically.\n See also TSetOfMetricMapInitializers::loadFromConfigFile for a template of\n\".ini\"-like configuration\n file that can be used to define which maps to create and all their\nparameters.\n Alternatively, the list of maps is public so it can be directly\nmanipulated/accessed in CMultiMetricMap::maps\n\n  Configuring the list of maps: Alternatives\n --------------------------------------------\n\n  **Method #1: Using map definition structures**\n \n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n  **Method #2: Using a configuration file**\n See TSetOfMetricMapInitializers::loadFromConfigFile() for details on expected\nfile format.\n\n \n\n\n\n\n\n\n\n  **Method #3: Manual manipulation**\n\n \n\n\n\n\n\n\n \n [New in MRPT 1.3.0]: `likelihoodMapSelection`, which selected the map\nto be used when\n  computing the likelihood of an observation, has been removed. Use the\n`enableObservationLikelihood`\n  property of each individual map declaration.\n\n \n [New in MRPT 1.3.0]: `enableInsertion_{pointsMap,...}` have been also\nremoved.\n  Use the `enableObservationInsertion` property of each map declaration.\n\n \n This class belongs to [mrpt-slam] instead of [mrpt-maps] due to the\ndependency on map classes in mrpt-vision.\n \n\n CMetricMap  \n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::maps::CMultiMetricMap(); }, [](){ return new PyCallBack_mrpt_maps_CMultiMetricMap(); } ) );
		cl.def( pybind11::init<const class mrpt::maps::TSetOfMetricMapInitializers &>(), pybind11::arg("initializers") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_maps_CMultiMetricMap const &o){ return new PyCallBack_mrpt_maps_CMultiMetricMap(o); } ) );
		cl.def( pybind11::init( [](mrpt::maps::CMultiMetricMap const &o){ return new mrpt::maps::CMultiMetricMap(o); } ) );
		cl.def_readwrite("maps", &mrpt::maps::CMultiMetricMap::maps);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::maps::CMultiMetricMap::GetRuntimeClassIdStatic, "C++: mrpt::maps::CMultiMetricMap::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::maps::CMultiMetricMap::*)() const) &mrpt::maps::CMultiMetricMap::GetRuntimeClass, "C++: mrpt::maps::CMultiMetricMap::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::maps::CMultiMetricMap::*)() const) &mrpt::maps::CMultiMetricMap::clone, "C++: mrpt::maps::CMultiMetricMap::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::maps::CMultiMetricMap::CreateObject, "C++: mrpt::maps::CMultiMetricMap::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("assign", (class mrpt::maps::CMultiMetricMap & (mrpt::maps::CMultiMetricMap::*)(const class mrpt::maps::CMultiMetricMap &)) &mrpt::maps::CMultiMetricMap::operator=, "Creates a deep copy \n\nC++: mrpt::maps::CMultiMetricMap::operator=(const class mrpt::maps::CMultiMetricMap &) --> class mrpt::maps::CMultiMetricMap &", pybind11::return_value_policy::automatic, pybind11::arg("o"));
		cl.def("mapByIndex", (class std::shared_ptr<class mrpt::maps::CMetricMap> (mrpt::maps::CMultiMetricMap::*)(size_t) const) &mrpt::maps::CMultiMetricMap::mapByIndex, "Gets the i-th map \n std::runtime_error On out-of-bounds \n\nC++: mrpt::maps::CMultiMetricMap::mapByIndex(size_t) const --> class std::shared_ptr<class mrpt::maps::CMetricMap>", pybind11::arg("idx"));
		cl.def("setListOfMaps", (void (mrpt::maps::CMultiMetricMap::*)(const class mrpt::maps::TSetOfMetricMapInitializers &)) &mrpt::maps::CMultiMetricMap::setListOfMaps, "Sets the list of internal map according to the passed list of map\n initializers (current maps will be deleted) \n\nC++: mrpt::maps::CMultiMetricMap::setListOfMaps(const class mrpt::maps::TSetOfMetricMapInitializers &) --> void", pybind11::arg("init"));
		cl.def("isEmpty", (bool (mrpt::maps::CMultiMetricMap::*)() const) &mrpt::maps::CMultiMetricMap::isEmpty, "Returns true if **all** maps returns true in their isEmpty() method \n\nC++: mrpt::maps::CMultiMetricMap::isEmpty() const --> bool");
		cl.def("determineMatching2D", (void (mrpt::maps::CMultiMetricMap::*)(const class mrpt::maps::CMetricMap *, const class mrpt::poses::CPose2D &, class mrpt::tfest::TMatchingPairListTempl<float> &, const struct mrpt::maps::TMatchingParams &, struct mrpt::maps::TMatchingExtraResults &) const) &mrpt::maps::CMultiMetricMap::determineMatching2D, "C++: mrpt::maps::CMultiMetricMap::determineMatching2D(const class mrpt::maps::CMetricMap *, const class mrpt::poses::CPose2D &, class mrpt::tfest::TMatchingPairListTempl<float> &, const struct mrpt::maps::TMatchingParams &, struct mrpt::maps::TMatchingExtraResults &) const --> void", pybind11::arg("otherMap"), pybind11::arg("otherMapPose"), pybind11::arg("correspondences"), pybind11::arg("params"), pybind11::arg("extraResults"));
		cl.def("compute3DMatchingRatio", (float (mrpt::maps::CMultiMetricMap::*)(const class mrpt::maps::CMetricMap *, const class mrpt::poses::CPose3D &, const struct mrpt::maps::TMatchingRatioParams &) const) &mrpt::maps::CMultiMetricMap::compute3DMatchingRatio, "C++: mrpt::maps::CMultiMetricMap::compute3DMatchingRatio(const class mrpt::maps::CMetricMap *, const class mrpt::poses::CPose3D &, const struct mrpt::maps::TMatchingRatioParams &) const --> float", pybind11::arg("otherMap"), pybind11::arg("otherMapPose"), pybind11::arg("params"));
		cl.def("saveMetricMapRepresentationToFile", (void (mrpt::maps::CMultiMetricMap::*)(const std::string &) const) &mrpt::maps::CMultiMetricMap::saveMetricMapRepresentationToFile, "C++: mrpt::maps::CMultiMetricMap::saveMetricMapRepresentationToFile(const std::string &) const --> void", pybind11::arg("filNamePrefix"));
		cl.def("auxParticleFilterCleanUp", (void (mrpt::maps::CMultiMetricMap::*)()) &mrpt::maps::CMultiMetricMap::auxParticleFilterCleanUp, "C++: mrpt::maps::CMultiMetricMap::auxParticleFilterCleanUp() --> void");
		cl.def("getVisualizationInto", (void (mrpt::maps::CMultiMetricMap::*)(class mrpt::opengl::CSetOfObjects &) const) &mrpt::maps::CMultiMetricMap::getVisualizationInto, "C++: mrpt::maps::CMultiMetricMap::getVisualizationInto(class mrpt::opengl::CSetOfObjects &) const --> void", pybind11::arg("outObj"));
		cl.def("getAsSimplePointsMap", (const class mrpt::maps::CSimplePointsMap * (mrpt::maps::CMultiMetricMap::*)() const) &mrpt::maps::CMultiMetricMap::getAsSimplePointsMap, "C++: mrpt::maps::CMultiMetricMap::getAsSimplePointsMap() const --> const class mrpt::maps::CSimplePointsMap *", pybind11::return_value_policy::automatic);
		cl.def("asString", (std::string (mrpt::maps::CMultiMetricMap::*)() const) &mrpt::maps::CMultiMetricMap::asString, "Returns a short description of the map. \n\nC++: mrpt::maps::CMultiMetricMap::asString() const --> std::string");
	}
}
