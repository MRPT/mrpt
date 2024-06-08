#include <deque>
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TTwist3D.h>
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPoseOrPoint.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/slam/CIncrementalMapPartitioner.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <sstream> // __str__
#include <string>
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

// mrpt::slam::CIncrementalMapPartitioner file:mrpt/slam/CIncrementalMapPartitioner.h line:57
struct PyCallBack_mrpt_slam_CIncrementalMapPartitioner : public mrpt::slam::CIncrementalMapPartitioner {
	using mrpt::slam::CIncrementalMapPartitioner::CIncrementalMapPartitioner;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CIncrementalMapPartitioner *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CIncrementalMapPartitioner::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CIncrementalMapPartitioner *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CIncrementalMapPartitioner::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CIncrementalMapPartitioner *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CIncrementalMapPartitioner::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CIncrementalMapPartitioner *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CIncrementalMapPartitioner::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CIncrementalMapPartitioner *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CIncrementalMapPartitioner::serializeFrom(a0, a1);
	}
};

// mrpt::slam::CIncrementalMapPartitioner::TOptions file:mrpt/slam/CIncrementalMapPartitioner.h line:69
struct PyCallBack_mrpt_slam_CIncrementalMapPartitioner_TOptions : public mrpt::slam::CIncrementalMapPartitioner::TOptions {
	using mrpt::slam::CIncrementalMapPartitioner::TOptions::TOptions;

	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CIncrementalMapPartitioner::TOptions *>(this), "loadFromConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TOptions::loadFromConfigFile(a0, a1);
	}
	void saveToConfigFile(class mrpt::config::CConfigFileBase & a0, const std::string & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CIncrementalMapPartitioner::TOptions *>(this), "saveToConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TOptions::saveToConfigFile(a0, a1);
	}
};

void bind_mrpt_slam_CIncrementalMapPartitioner(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::slam::similarity_method_t file:mrpt/slam/CIncrementalMapPartitioner.h line:29
	pybind11::enum_<mrpt::slam::similarity_method_t>(M("mrpt::slam"), "similarity_method_t", pybind11::arithmetic(), "For use in CIncrementalMapPartitioner\n \n")
		.value("smMETRIC_MAP_MATCHING", mrpt::slam::smMETRIC_MAP_MATCHING)
		.value("smOBSERVATION_OVERLAP", mrpt::slam::smOBSERVATION_OVERLAP)
		.value("smCUSTOM_FUNCTION", mrpt::slam::smCUSTOM_FUNCTION)
		.export_values();

;

	{ // mrpt::slam::map_keyframe_t file:mrpt/slam/CIncrementalMapPartitioner.h line:39
		pybind11::class_<mrpt::slam::map_keyframe_t, std::shared_ptr<mrpt::slam::map_keyframe_t>> cl(M("mrpt::slam"), "map_keyframe_t", "Map keyframe, comprising raw observations and they as a metric map.\n For use in CIncrementalMapPartitioner\n \n");
		cl.def( pybind11::init( [](){ return new mrpt::slam::map_keyframe_t(); } ) );
		cl.def( pybind11::init( [](mrpt::slam::map_keyframe_t const &o){ return new mrpt::slam::map_keyframe_t(o); } ) );
		cl.def_readwrite("kf_id", &mrpt::slam::map_keyframe_t::kf_id);
		cl.def_readwrite("metric_map", &mrpt::slam::map_keyframe_t::metric_map);
		cl.def_readwrite("raw_observations", &mrpt::slam::map_keyframe_t::raw_observations);
		cl.def("assign", (struct mrpt::slam::map_keyframe_t & (mrpt::slam::map_keyframe_t::*)(const struct mrpt::slam::map_keyframe_t &)) &mrpt::slam::map_keyframe_t::operator=, "C++: mrpt::slam::map_keyframe_t::operator=(const struct mrpt::slam::map_keyframe_t &) --> struct mrpt::slam::map_keyframe_t &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::slam::CIncrementalMapPartitioner file:mrpt/slam/CIncrementalMapPartitioner.h line:57
		pybind11::class_<mrpt::slam::CIncrementalMapPartitioner, std::shared_ptr<mrpt::slam::CIncrementalMapPartitioner>, PyCallBack_mrpt_slam_CIncrementalMapPartitioner, mrpt::serialization::CSerializable> cl(M("mrpt::slam"), "CIncrementalMapPartitioner", "Finds partitions in metric maps based on N-cut graph partition theory.\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::slam::CIncrementalMapPartitioner(); }, [](){ return new PyCallBack_mrpt_slam_CIncrementalMapPartitioner(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_slam_CIncrementalMapPartitioner const &o){ return new PyCallBack_mrpt_slam_CIncrementalMapPartitioner(o); } ) );
		cl.def( pybind11::init( [](mrpt::slam::CIncrementalMapPartitioner const &o){ return new mrpt::slam::CIncrementalMapPartitioner(o); } ) );
		cl.def_readwrite("options", &mrpt::slam::CIncrementalMapPartitioner::options);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::slam::CIncrementalMapPartitioner::GetRuntimeClassIdStatic, "C++: mrpt::slam::CIncrementalMapPartitioner::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::slam::CIncrementalMapPartitioner::*)() const) &mrpt::slam::CIncrementalMapPartitioner::GetRuntimeClass, "C++: mrpt::slam::CIncrementalMapPartitioner::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::slam::CIncrementalMapPartitioner::*)() const) &mrpt::slam::CIncrementalMapPartitioner::clone, "C++: mrpt::slam::CIncrementalMapPartitioner::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::slam::CIncrementalMapPartitioner::CreateObject, "C++: mrpt::slam::CIncrementalMapPartitioner::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("clear", (void (mrpt::slam::CIncrementalMapPartitioner::*)()) &mrpt::slam::CIncrementalMapPartitioner::clear, "C++: mrpt::slam::CIncrementalMapPartitioner::clear() --> void");
		cl.def("addMapFrame", (uint32_t (mrpt::slam::CIncrementalMapPartitioner::*)(const class mrpt::obs::CSensoryFrame &, const class mrpt::poses::CPose3DPDF &)) &mrpt::slam::CIncrementalMapPartitioner::addMapFrame, "Insert a new keyframe to the graph.\n\n Call this method each time a new observation is added to the map/graph.\n Afterwards, call updatePartitions() to get the updated partitions.\n\n \n The sensed data\n \n\n An estimation of the robot global pose.\n\n \n The index of the new pose in the graph, which can be used to\n refer to this pose in the future.\n\n \n updatePartitions\n\nC++: mrpt::slam::CIncrementalMapPartitioner::addMapFrame(const class mrpt::obs::CSensoryFrame &, const class mrpt::poses::CPose3DPDF &) --> uint32_t", pybind11::arg("frame"), pybind11::arg("robotPose3D"));
		cl.def("getNodesCount", (size_t (mrpt::slam::CIncrementalMapPartitioner::*)()) &mrpt::slam::CIncrementalMapPartitioner::getNodesCount, "Get the total node count currently in the internal map/graph. \n\nC++: mrpt::slam::CIncrementalMapPartitioner::getNodesCount() --> size_t");
		cl.def("changeCoordinatesOrigin", (void (mrpt::slam::CIncrementalMapPartitioner::*)(const class mrpt::poses::CPose3D &)) &mrpt::slam::CIncrementalMapPartitioner::changeCoordinatesOrigin, "Change the coordinate origin of all stored poses\n\n Used for consistency with future new poses to enter in the system.\n\nC++: mrpt::slam::CIncrementalMapPartitioner::changeCoordinatesOrigin(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("newOrigin"));
		cl.def("changeCoordinatesOriginPoseIndex", (void (mrpt::slam::CIncrementalMapPartitioner::*)(unsigned int)) &mrpt::slam::CIncrementalMapPartitioner::changeCoordinatesOriginPoseIndex, "The new origin is given by the index of the pose that is to\n become the new origin.\n\nC++: mrpt::slam::CIncrementalMapPartitioner::changeCoordinatesOriginPoseIndex(unsigned int) --> void", pybind11::arg("newOriginPose"));
		cl.def("setSimilarityMethod", (void (mrpt::slam::CIncrementalMapPartitioner::*)(enum mrpt::slam::similarity_method_t)) &mrpt::slam::CIncrementalMapPartitioner::setSimilarityMethod, "Select the similarity method to use for newly inserted keyframes \n\nC++: mrpt::slam::CIncrementalMapPartitioner::setSimilarityMethod(enum mrpt::slam::similarity_method_t) --> void", pybind11::arg("method"));
		cl.def("setSimilarityMethod", (void (mrpt::slam::CIncrementalMapPartitioner::*)(class std::function<double (const struct mrpt::slam::map_keyframe_t &, const struct mrpt::slam::map_keyframe_t &, const class mrpt::poses::CPose3D &)>)) &mrpt::slam::CIncrementalMapPartitioner::setSimilarityMethod, "Sets a custom function for the similarity of new keyframes \n\nC++: mrpt::slam::CIncrementalMapPartitioner::setSimilarityMethod(class std::function<double (const struct mrpt::slam::map_keyframe_t &, const struct mrpt::slam::map_keyframe_t &, const class mrpt::poses::CPose3D &)>) --> void", pybind11::arg("func"));
		cl.def("getAdjacencyMatrix", (const class mrpt::math::CMatrixDynamic<double> & (mrpt::slam::CIncrementalMapPartitioner::*)() const) &mrpt::slam::CIncrementalMapPartitioner::getAdjacencyMatrix, "Return a const ref to the internal adjacency matrix.  \n\nC++: mrpt::slam::CIncrementalMapPartitioner::getAdjacencyMatrix() const --> const class mrpt::math::CMatrixDynamic<double> &", pybind11::return_value_policy::automatic);
		cl.def("getSequenceOfFrames", (class mrpt::maps::CSimpleMap * (mrpt::slam::CIncrementalMapPartitioner::*)()) &mrpt::slam::CIncrementalMapPartitioner::getSequenceOfFrames, "Access to the sequence of Sensory Frames \n\nC++: mrpt::slam::CIncrementalMapPartitioner::getSequenceOfFrames() --> class mrpt::maps::CSimpleMap *", pybind11::return_value_policy::automatic);
		cl.def("assign", (class mrpt::slam::CIncrementalMapPartitioner & (mrpt::slam::CIncrementalMapPartitioner::*)(const class mrpt::slam::CIncrementalMapPartitioner &)) &mrpt::slam::CIncrementalMapPartitioner::operator=, "C++: mrpt::slam::CIncrementalMapPartitioner::operator=(const class mrpt::slam::CIncrementalMapPartitioner &) --> class mrpt::slam::CIncrementalMapPartitioner &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::slam::CIncrementalMapPartitioner::TOptions file:mrpt/slam/CIncrementalMapPartitioner.h line:69
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::slam::CIncrementalMapPartitioner::TOptions, std::shared_ptr<mrpt::slam::CIncrementalMapPartitioner::TOptions>, PyCallBack_mrpt_slam_CIncrementalMapPartitioner_TOptions, mrpt::config::CLoadableOptions> cl(enclosing_class, "TOptions", "Configuration parameters ");
			cl.def( pybind11::init( [](){ return new mrpt::slam::CIncrementalMapPartitioner::TOptions(); }, [](){ return new PyCallBack_mrpt_slam_CIncrementalMapPartitioner_TOptions(); } ) );
			cl.def( pybind11::init( [](PyCallBack_mrpt_slam_CIncrementalMapPartitioner_TOptions const &o){ return new PyCallBack_mrpt_slam_CIncrementalMapPartitioner_TOptions(o); } ) );
			cl.def( pybind11::init( [](mrpt::slam::CIncrementalMapPartitioner::TOptions const &o){ return new mrpt::slam::CIncrementalMapPartitioner::TOptions(o); } ) );
			cl.def_readwrite("partitionThreshold", &mrpt::slam::CIncrementalMapPartitioner::TOptions::partitionThreshold);
			cl.def_readwrite("mrp", &mrpt::slam::CIncrementalMapPartitioner::TOptions::mrp);
			cl.def_readwrite("forceBisectionOnly", &mrpt::slam::CIncrementalMapPartitioner::TOptions::forceBisectionOnly);
			cl.def_readwrite("simil_method", &mrpt::slam::CIncrementalMapPartitioner::TOptions::simil_method);
			cl.def_readwrite("minimumNumberElementsEachCluster", &mrpt::slam::CIncrementalMapPartitioner::TOptions::minimumNumberElementsEachCluster);
			cl.def_readwrite("metricmap", &mrpt::slam::CIncrementalMapPartitioner::TOptions::metricmap);
			cl.def_readwrite("maxKeyFrameDistanceToEval", &mrpt::slam::CIncrementalMapPartitioner::TOptions::maxKeyFrameDistanceToEval);
			cl.def("loadFromConfigFile", (void (mrpt::slam::CIncrementalMapPartitioner::TOptions::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::slam::CIncrementalMapPartitioner::TOptions::loadFromConfigFile, "C++: mrpt::slam::CIncrementalMapPartitioner::TOptions::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("source"), pybind11::arg("section"));
			cl.def("saveToConfigFile", (void (mrpt::slam::CIncrementalMapPartitioner::TOptions::*)(class mrpt::config::CConfigFileBase &, const std::string &) const) &mrpt::slam::CIncrementalMapPartitioner::TOptions::saveToConfigFile, "C++: mrpt::slam::CIncrementalMapPartitioner::TOptions::saveToConfigFile(class mrpt::config::CConfigFileBase &, const std::string &) const --> void", pybind11::arg("target"), pybind11::arg("section"));
			cl.def("assign", (struct mrpt::slam::CIncrementalMapPartitioner::TOptions & (mrpt::slam::CIncrementalMapPartitioner::TOptions::*)(const struct mrpt::slam::CIncrementalMapPartitioner::TOptions &)) &mrpt::slam::CIncrementalMapPartitioner::TOptions::operator=, "C++: mrpt::slam::CIncrementalMapPartitioner::TOptions::operator=(const struct mrpt::slam::CIncrementalMapPartitioner::TOptions &) --> struct mrpt::slam::CIncrementalMapPartitioner::TOptions &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
}
