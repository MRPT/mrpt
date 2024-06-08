#include <deque>
#include <ios>
#include <iterator>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/img/CCanvas.h>
#include <mrpt/img/CImage.h>
#include <mrpt/img/TColor.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/TMetricMapInitializer.h>
#include <mrpt/maps/metric_map_types.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TTwist3D.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/obs/CAction.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint2DPDFGaussian.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPointPDF.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPose3DQuatPDF.h>
#include <mrpt/poses/CPose3DQuatPDFGaussian.h>
#include <mrpt/poses/CPoseOrPoint.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/slam/CMetricMapBuilder.h>
#include <mrpt/slam/CMetricMapBuilderRBPF.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/tfest/TMatchingPair.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <ostream>
#include <sstream> // __str__
#include <streambuf>
#include <string>
#include <tuple>
#include <type_traits>
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

// mrpt::slam::CMetricMapBuilder file:mrpt/slam/CMetricMapBuilder.h line:32
struct PyCallBack_mrpt_slam_CMetricMapBuilder : public mrpt::slam::CMetricMapBuilder {
	using mrpt::slam::CMetricMapBuilder::CMetricMapBuilder;

	void initialize(const class mrpt::maps::CSimpleMap & a0, const class mrpt::poses::CPosePDF * a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMetricMapBuilder *>(this), "initialize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CMetricMapBuilder::initialize\"");
	}
	class std::shared_ptr<class mrpt::poses::CPose3DPDF> getCurrentPoseEstimation() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMetricMapBuilder *>(this), "getCurrentPoseEstimation");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class std::shared_ptr<class mrpt::poses::CPose3DPDF>>::value) {
				static pybind11::detail::override_caster_t<class std::shared_ptr<class mrpt::poses::CPose3DPDF>> caster;
				return pybind11::detail::cast_ref<class std::shared_ptr<class mrpt::poses::CPose3DPDF>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class std::shared_ptr<class mrpt::poses::CPose3DPDF>>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CMetricMapBuilder::getCurrentPoseEstimation\"");
	}
	void processActionObservation(class mrpt::obs::CActionCollection & a0, class mrpt::obs::CSensoryFrame & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMetricMapBuilder *>(this), "processActionObservation");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CMetricMapBuilder::processActionObservation\"");
	}
	void getCurrentlyBuiltMap(class mrpt::maps::CSimpleMap & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMetricMapBuilder *>(this), "getCurrentlyBuiltMap");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CMetricMapBuilder::getCurrentlyBuiltMap\"");
	}
	unsigned int getCurrentlyBuiltMapSize() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMetricMapBuilder *>(this), "getCurrentlyBuiltMapSize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<unsigned int>::value) {
				static pybind11::detail::override_caster_t<unsigned int> caster;
				return pybind11::detail::cast_ref<unsigned int>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<unsigned int>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CMetricMapBuilder::getCurrentlyBuiltMapSize\"");
	}
	const class mrpt::maps::CMultiMetricMap & getCurrentlyBuiltMetricMap() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMetricMapBuilder *>(this), "getCurrentlyBuiltMetricMap");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const class mrpt::maps::CMultiMetricMap &>::value) {
				static pybind11::detail::override_caster_t<const class mrpt::maps::CMultiMetricMap &> caster;
				return pybind11::detail::cast_ref<const class mrpt::maps::CMultiMetricMap &>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const class mrpt::maps::CMultiMetricMap &>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CMetricMapBuilder::getCurrentlyBuiltMetricMap\"");
	}
	void saveCurrentEstimationToImage(const std::string & a0, bool a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMetricMapBuilder *>(this), "saveCurrentEstimationToImage");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CMetricMapBuilder::saveCurrentEstimationToImage\"");
	}
};

// mrpt::slam::CMetricMapBuilderRBPF file:mrpt/slam/CMetricMapBuilderRBPF.h line:55
struct PyCallBack_mrpt_slam_CMetricMapBuilderRBPF : public mrpt::slam::CMetricMapBuilderRBPF {
	using mrpt::slam::CMetricMapBuilderRBPF::CMetricMapBuilderRBPF;

	void initialize(const class mrpt::maps::CSimpleMap & a0, const class mrpt::poses::CPosePDF * a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMetricMapBuilderRBPF *>(this), "initialize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMetricMapBuilderRBPF::initialize(a0, a1);
	}
	class std::shared_ptr<class mrpt::poses::CPose3DPDF> getCurrentPoseEstimation() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMetricMapBuilderRBPF *>(this), "getCurrentPoseEstimation");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class std::shared_ptr<class mrpt::poses::CPose3DPDF>>::value) {
				static pybind11::detail::override_caster_t<class std::shared_ptr<class mrpt::poses::CPose3DPDF>> caster;
				return pybind11::detail::cast_ref<class std::shared_ptr<class mrpt::poses::CPose3DPDF>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class std::shared_ptr<class mrpt::poses::CPose3DPDF>>(std::move(o));
		}
		return CMetricMapBuilderRBPF::getCurrentPoseEstimation();
	}
	void processActionObservation(class mrpt::obs::CActionCollection & a0, class mrpt::obs::CSensoryFrame & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMetricMapBuilderRBPF *>(this), "processActionObservation");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMetricMapBuilderRBPF::processActionObservation(a0, a1);
	}
	void getCurrentlyBuiltMap(class mrpt::maps::CSimpleMap & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMetricMapBuilderRBPF *>(this), "getCurrentlyBuiltMap");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMetricMapBuilderRBPF::getCurrentlyBuiltMap(a0);
	}
	const class mrpt::maps::CMultiMetricMap & getCurrentlyBuiltMetricMap() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMetricMapBuilderRBPF *>(this), "getCurrentlyBuiltMetricMap");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const class mrpt::maps::CMultiMetricMap &>::value) {
				static pybind11::detail::override_caster_t<const class mrpt::maps::CMultiMetricMap &> caster;
				return pybind11::detail::cast_ref<const class mrpt::maps::CMultiMetricMap &>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const class mrpt::maps::CMultiMetricMap &>(std::move(o));
		}
		return CMetricMapBuilderRBPF::getCurrentlyBuiltMetricMap();
	}
	unsigned int getCurrentlyBuiltMapSize() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMetricMapBuilderRBPF *>(this), "getCurrentlyBuiltMapSize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<unsigned int>::value) {
				static pybind11::detail::override_caster_t<unsigned int> caster;
				return pybind11::detail::cast_ref<unsigned int>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<unsigned int>(std::move(o));
		}
		return CMetricMapBuilderRBPF::getCurrentlyBuiltMapSize();
	}
	void saveCurrentEstimationToImage(const std::string & a0, bool a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMetricMapBuilderRBPF *>(this), "saveCurrentEstimationToImage");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMetricMapBuilderRBPF::saveCurrentEstimationToImage(a0, a1);
	}
};

// mrpt::slam::CMetricMapBuilderRBPF::TConstructionOptions file:mrpt/slam/CMetricMapBuilderRBPF.h line:82
struct PyCallBack_mrpt_slam_CMetricMapBuilderRBPF_TConstructionOptions : public mrpt::slam::CMetricMapBuilderRBPF::TConstructionOptions {
	using mrpt::slam::CMetricMapBuilderRBPF::TConstructionOptions::TConstructionOptions;

	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMetricMapBuilderRBPF::TConstructionOptions *>(this), "loadFromConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TConstructionOptions::loadFromConfigFile(a0, a1);
	}
	void saveToConfigFile(class mrpt::config::CConfigFileBase & a0, const std::string & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMetricMapBuilderRBPF::TConstructionOptions *>(this), "saveToConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CLoadableOptions::saveToConfigFile(a0, a1);
	}
};

void bind_mrpt_slam_CMetricMapBuilder(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::slam::CMetricMapBuilder file:mrpt/slam/CMetricMapBuilder.h line:32
		pybind11::class_<mrpt::slam::CMetricMapBuilder, std::shared_ptr<mrpt::slam::CMetricMapBuilder>, PyCallBack_mrpt_slam_CMetricMapBuilder> cl(M("mrpt::slam"), "CMetricMapBuilder", "This virtual class is the base for SLAM implementations. See derived classes\n for more information.\n\n \n CMetricMap  \n\n ");
		cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_slam_CMetricMapBuilder(); } ) );
		cl.def("initialize", [](mrpt::slam::CMetricMapBuilder &o) -> void { return o.initialize(); }, "");
		cl.def("initialize", [](mrpt::slam::CMetricMapBuilder &o, const class mrpt::maps::CSimpleMap & a0) -> void { return o.initialize(a0); }, "", pybind11::arg("initialMap"));
		cl.def("initialize", (void (mrpt::slam::CMetricMapBuilder::*)(const class mrpt::maps::CSimpleMap &, const class mrpt::poses::CPosePDF *)) &mrpt::slam::CMetricMapBuilder::initialize, "Initialize the method, starting with a known location PDF \"x0\"(if\n supplied, set to nullptr to left unmodified) and a given fixed, past map.\n\nC++: mrpt::slam::CMetricMapBuilder::initialize(const class mrpt::maps::CSimpleMap &, const class mrpt::poses::CPosePDF *) --> void", pybind11::arg("initialMap"), pybind11::arg("x0"));
		cl.def("getCurrentPoseEstimation", (class std::shared_ptr<class mrpt::poses::CPose3DPDF> (mrpt::slam::CMetricMapBuilder::*)() const) &mrpt::slam::CMetricMapBuilder::getCurrentPoseEstimation, "Returns a copy of the current best pose estimation as a pose PDF. \n\nC++: mrpt::slam::CMetricMapBuilder::getCurrentPoseEstimation() const --> class std::shared_ptr<class mrpt::poses::CPose3DPDF>");
		cl.def("processActionObservation", (void (mrpt::slam::CMetricMapBuilder::*)(class mrpt::obs::CActionCollection &, class mrpt::obs::CSensoryFrame &)) &mrpt::slam::CMetricMapBuilder::processActionObservation, "Process a new action and observations pair to update this map: See the\ndescription of the class at the top of this page to see a more complete\ndescription.\n  \n\n The estimation of the incremental pose change in the robot\npose.\n	\n\n The set of observations that robot senses at the new\npose.\n\nC++: mrpt::slam::CMetricMapBuilder::processActionObservation(class mrpt::obs::CActionCollection &, class mrpt::obs::CSensoryFrame &) --> void", pybind11::arg("action"), pybind11::arg("observations"));
		cl.def("getCurrentlyBuiltMap", (void (mrpt::slam::CMetricMapBuilder::*)(class mrpt::maps::CSimpleMap &) const) &mrpt::slam::CMetricMapBuilder::getCurrentlyBuiltMap, "Fills \"out_map\" with the set of \"poses\"-\"sensory-frames\", thus the so\n far built map. \n\nC++: mrpt::slam::CMetricMapBuilder::getCurrentlyBuiltMap(class mrpt::maps::CSimpleMap &) const --> void", pybind11::arg("out_map"));
		cl.def("getCurrentlyBuiltMapSize", (unsigned int (mrpt::slam::CMetricMapBuilder::*)()) &mrpt::slam::CMetricMapBuilder::getCurrentlyBuiltMapSize, "Returns just how many sensory-frames are stored in the currently build\n map. \n\nC++: mrpt::slam::CMetricMapBuilder::getCurrentlyBuiltMapSize() --> unsigned int");
		cl.def("getCurrentlyBuiltMetricMap", (const class mrpt::maps::CMultiMetricMap & (mrpt::slam::CMetricMapBuilder::*)() const) &mrpt::slam::CMetricMapBuilder::getCurrentlyBuiltMetricMap, "Returns the map built so far. NOTE that for efficiency a pointer to the\n internal object is passed, DO NOT delete nor modify the object in any\n way, if desired, make a copy of ir with \"clone()\". \n\nC++: mrpt::slam::CMetricMapBuilder::getCurrentlyBuiltMetricMap() const --> const class mrpt::maps::CMultiMetricMap &", pybind11::return_value_policy::automatic);
		cl.def("saveCurrentEstimationToImage", [](mrpt::slam::CMetricMapBuilder &o, const std::string & a0) -> void { return o.saveCurrentEstimationToImage(a0); }, "", pybind11::arg("file"));
		cl.def("saveCurrentEstimationToImage", (void (mrpt::slam::CMetricMapBuilder::*)(const std::string &, bool)) &mrpt::slam::CMetricMapBuilder::saveCurrentEstimationToImage, "A useful method for debugging: the current map (and/or poses) estimation\n is dumped to an image file.\n \n\n The output file name\n \n\n Output format = true:EMF, false:BMP\n\nC++: mrpt::slam::CMetricMapBuilder::saveCurrentEstimationToImage(const std::string &, bool) --> void", pybind11::arg("file"), pybind11::arg("formatEMF_BMP"));
		cl.def("clear", (void (mrpt::slam::CMetricMapBuilder::*)()) &mrpt::slam::CMetricMapBuilder::clear, "Clear all elements of the maps, and reset localization to (0,0,0deg). \n\nC++: mrpt::slam::CMetricMapBuilder::clear() --> void");
		cl.def("enableMapUpdating", (void (mrpt::slam::CMetricMapBuilder::*)(bool)) &mrpt::slam::CMetricMapBuilder::enableMapUpdating, "Enables or disables the map updating (default state is enabled) \n\nC++: mrpt::slam::CMetricMapBuilder::enableMapUpdating(bool) --> void", pybind11::arg("enable"));
		cl.def("loadCurrentMapFromFile", (void (mrpt::slam::CMetricMapBuilder::*)(const std::string &)) &mrpt::slam::CMetricMapBuilder::loadCurrentMapFromFile, "Load map (mrpt::maps::CSimpleMap) from a \".simplemap\" file \n\nC++: mrpt::slam::CMetricMapBuilder::loadCurrentMapFromFile(const std::string &) --> void", pybind11::arg("fileName"));
		cl.def("saveCurrentMapToFile", [](mrpt::slam::CMetricMapBuilder const &o, const std::string & a0) -> void { return o.saveCurrentMapToFile(a0); }, "", pybind11::arg("fileName"));
		cl.def("saveCurrentMapToFile", (void (mrpt::slam::CMetricMapBuilder::*)(const std::string &, bool) const) &mrpt::slam::CMetricMapBuilder::saveCurrentMapToFile, "Save map (mrpt::maps::CSimpleMap) to a \".simplemap\" file. \n\nC++: mrpt::slam::CMetricMapBuilder::saveCurrentMapToFile(const std::string &, bool) const --> void", pybind11::arg("fileName"), pybind11::arg("compressGZ"));

		{ // mrpt::slam::CMetricMapBuilder::TOptions file:mrpt/slam/CMetricMapBuilder.h line:108
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::slam::CMetricMapBuilder::TOptions, std::shared_ptr<mrpt::slam::CMetricMapBuilder::TOptions>> cl(enclosing_class, "TOptions", "Options for the algorithm ");
			cl.def( pybind11::init<enum mrpt::system::VerbosityLevel &>(), pybind11::arg("verb_level_ref") );

			cl.def_readwrite("enableMapUpdating", &mrpt::slam::CMetricMapBuilder::TOptions::enableMapUpdating);
			cl.def_readwrite("debugForceInsertion", &mrpt::slam::CMetricMapBuilder::TOptions::debugForceInsertion);
			cl.def_readwrite("alwaysInsertByClass", &mrpt::slam::CMetricMapBuilder::TOptions::alwaysInsertByClass);
		}

	}
	{ // mrpt::slam::CMetricMapBuilderRBPF file:mrpt/slam/CMetricMapBuilderRBPF.h line:55
		pybind11::class_<mrpt::slam::CMetricMapBuilderRBPF, std::shared_ptr<mrpt::slam::CMetricMapBuilderRBPF>, PyCallBack_mrpt_slam_CMetricMapBuilderRBPF, mrpt::slam::CMetricMapBuilder> cl(M("mrpt::slam"), "CMetricMapBuilderRBPF", "This class implements a Rao-Blackwelized Particle Filter (RBPF) approach to\n map building (SLAM).\n   Internally, the list of particles, each containing a hypothesis for the\n robot path plus its associated\n    metric map, is stored in an object of class CMultiMetricMapPDF.\n\n  This class processes robot actions and observations sequentially (through\n the method CMetricMapBuilderRBPF::processActionObservation)\n   and exploits the generic design of metric map classes in MRPT to deal with\n any number and combination of maps simultaneously: the likelihood\n   of observations is the product of the likelihood in the different maps,\n etc.\n\n   A number of particle filter methods are implemented as well, by selecting\n the appropriate values in TConstructionOptions::PF_options.\n   Not all the PF algorithms are implemented for all kinds of maps.\n\n  For an example of usage, check the application \"rbpf-slam\", in\n \"apps/RBPF-SLAM\". See also the \n* href=\"http://www.mrpt.org/Application:RBPF-SLAM\" >wiki page.\n\n  \n Since MRPT 0.7.2, the new variables\n \"localizeLinDistance,localizeAngDistance\" are introduced to provide a way to\n update the robot pose at a different rate than the map is updated.\n  \n\n Since MRPT 0.7.1 the semantics of the parameters\n \"insertionLinDistance\" and \"insertionAngDistance\" changes: the entire RBFP is\n now NOT updated unless odometry increments surpass the threshold (previously,\n only the map was NOT updated). This is done to gain efficiency.\n  \n\n Since MRPT 0.6.2 this class implements full 6D SLAM. Previous versions\n worked in 2D + heading only.\n\n \n CMetricMap   \n\n ");
		cl.def( pybind11::init<const struct mrpt::slam::CMetricMapBuilderRBPF::TConstructionOptions &>(), pybind11::arg("initializationOptions") );

		cl.def( pybind11::init( [](){ return new mrpt::slam::CMetricMapBuilderRBPF(); }, [](){ return new PyCallBack_mrpt_slam_CMetricMapBuilderRBPF(); } ) );
		cl.def_readwrite("mapPDF", &mrpt::slam::CMetricMapBuilderRBPF::mapPDF);
		cl.def_readwrite("m_statsLastIteration", &mrpt::slam::CMetricMapBuilderRBPF::m_statsLastIteration);
		cl.def("assign", (class mrpt::slam::CMetricMapBuilderRBPF & (mrpt::slam::CMetricMapBuilderRBPF::*)(const class mrpt::slam::CMetricMapBuilderRBPF &)) &mrpt::slam::CMetricMapBuilderRBPF::operator=, "Copy Operator. \n\nC++: mrpt::slam::CMetricMapBuilderRBPF::operator=(const class mrpt::slam::CMetricMapBuilderRBPF &) --> class mrpt::slam::CMetricMapBuilderRBPF &", pybind11::return_value_policy::automatic, pybind11::arg("src"));
		cl.def("initialize", [](mrpt::slam::CMetricMapBuilderRBPF &o) -> void { return o.initialize(); }, "");
		cl.def("initialize", [](mrpt::slam::CMetricMapBuilderRBPF &o, const class mrpt::maps::CSimpleMap & a0) -> void { return o.initialize(a0); }, "", pybind11::arg("initialMap"));
		cl.def("initialize", (void (mrpt::slam::CMetricMapBuilderRBPF::*)(const class mrpt::maps::CSimpleMap &, const class mrpt::poses::CPosePDF *)) &mrpt::slam::CMetricMapBuilderRBPF::initialize, "Initialize the method, starting with a known location PDF \"x0\"(if\n supplied, set to nullptr to left unmodified) and a given fixed, past map.\n\nC++: mrpt::slam::CMetricMapBuilderRBPF::initialize(const class mrpt::maps::CSimpleMap &, const class mrpt::poses::CPosePDF *) --> void", pybind11::arg("initialMap"), pybind11::arg("x0"));
		cl.def("clear", (void (mrpt::slam::CMetricMapBuilderRBPF::*)()) &mrpt::slam::CMetricMapBuilderRBPF::clear, "Clear all elements of the maps.\n\nC++: mrpt::slam::CMetricMapBuilderRBPF::clear() --> void");
		cl.def("getCurrentPoseEstimation", (class std::shared_ptr<class mrpt::poses::CPose3DPDF> (mrpt::slam::CMetricMapBuilderRBPF::*)() const) &mrpt::slam::CMetricMapBuilderRBPF::getCurrentPoseEstimation, "Returns a copy of the current best pose estimation as a pose PDF.\n\nC++: mrpt::slam::CMetricMapBuilderRBPF::getCurrentPoseEstimation() const --> class std::shared_ptr<class mrpt::poses::CPose3DPDF>");
		cl.def("getCurrentMostLikelyPath", (void (mrpt::slam::CMetricMapBuilderRBPF::*)(class std::deque<struct mrpt::math::TPose3D> &) const) &mrpt::slam::CMetricMapBuilderRBPF::getCurrentMostLikelyPath, "Returns the current most-likely path estimation (the path associated to\n the most likely particle).\n\nC++: mrpt::slam::CMetricMapBuilderRBPF::getCurrentMostLikelyPath(class std::deque<struct mrpt::math::TPose3D> &) const --> void", pybind11::arg("outPath"));
		cl.def("processActionObservation", (void (mrpt::slam::CMetricMapBuilderRBPF::*)(class mrpt::obs::CActionCollection &, class mrpt::obs::CSensoryFrame &)) &mrpt::slam::CMetricMapBuilderRBPF::processActionObservation, "Appends a new action and observations to update this map: See the\ndescription of the class at the top of this page to see a more complete\ndescription.\n  \n\n The incremental 2D pose change in the robot pose. This\nvalue is deterministic.\n	\n\n The set of observations that robot senses at the new\npose.\n  Statistics will be saved to statsLastIteration\n\nC++: mrpt::slam::CMetricMapBuilderRBPF::processActionObservation(class mrpt::obs::CActionCollection &, class mrpt::obs::CSensoryFrame &) --> void", pybind11::arg("action"), pybind11::arg("observations"));
		cl.def("getCurrentlyBuiltMap", (void (mrpt::slam::CMetricMapBuilderRBPF::*)(class mrpt::maps::CSimpleMap &) const) &mrpt::slam::CMetricMapBuilderRBPF::getCurrentlyBuiltMap, "Fills \"out_map\" with the set of \"poses\"-\"sensory-frames\", thus the so\n far built map.\n\nC++: mrpt::slam::CMetricMapBuilderRBPF::getCurrentlyBuiltMap(class mrpt::maps::CSimpleMap &) const --> void", pybind11::arg("out_map"));
		cl.def("getCurrentlyBuiltMetricMap", (const class mrpt::maps::CMultiMetricMap & (mrpt::slam::CMetricMapBuilderRBPF::*)() const) &mrpt::slam::CMetricMapBuilderRBPF::getCurrentlyBuiltMetricMap, "Returns the map built so far. NOTE that for efficiency a pointer to the\n internal object is passed, DO NOT delete nor modify the object in any\n way, if desired, make a copy of ir with \"clone()\".\n\nC++: mrpt::slam::CMetricMapBuilderRBPF::getCurrentlyBuiltMetricMap() const --> const class mrpt::maps::CMultiMetricMap &", pybind11::return_value_policy::automatic);
		cl.def("getCurrentlyBuiltMapSize", (unsigned int (mrpt::slam::CMetricMapBuilderRBPF::*)()) &mrpt::slam::CMetricMapBuilderRBPF::getCurrentlyBuiltMapSize, "Returns just how many sensory-frames are stored in the currently build\n map.\n\nC++: mrpt::slam::CMetricMapBuilderRBPF::getCurrentlyBuiltMapSize() --> unsigned int");
		cl.def("saveCurrentEstimationToImage", [](mrpt::slam::CMetricMapBuilderRBPF &o, const std::string & a0) -> void { return o.saveCurrentEstimationToImage(a0); }, "", pybind11::arg("file"));
		cl.def("saveCurrentEstimationToImage", (void (mrpt::slam::CMetricMapBuilderRBPF::*)(const std::string &, bool)) &mrpt::slam::CMetricMapBuilderRBPF::saveCurrentEstimationToImage, "A useful method for debugging: the current map (and/or poses) estimation\n is dumped to an image file.\n \n\n The output file name\n \n\n Output format = true:EMF, false:BMP\n\nC++: mrpt::slam::CMetricMapBuilderRBPF::saveCurrentEstimationToImage(const std::string &, bool) --> void", pybind11::arg("file"), pybind11::arg("formatEMF_BMP"));
		cl.def("drawCurrentEstimationToImage", (void (mrpt::slam::CMetricMapBuilderRBPF::*)(class mrpt::img::CCanvas *)) &mrpt::slam::CMetricMapBuilderRBPF::drawCurrentEstimationToImage, "A useful method for debugging: draws the current map and path hypotheses\n to a CCanvas  \n\nC++: mrpt::slam::CMetricMapBuilderRBPF::drawCurrentEstimationToImage(class mrpt::img::CCanvas *) --> void", pybind11::arg("img"));
		cl.def("saveCurrentPathEstimationToTextFile", (void (mrpt::slam::CMetricMapBuilderRBPF::*)(const std::string &)) &mrpt::slam::CMetricMapBuilderRBPF::saveCurrentPathEstimationToTextFile, "A logging utility: saves the current path estimation for each particle\n in a text file (a row per particle, each 3-column-entry is a set\n [x,y,phi], respectively).\n\nC++: mrpt::slam::CMetricMapBuilderRBPF::saveCurrentPathEstimationToTextFile(const std::string &) --> void", pybind11::arg("fil"));
		cl.def("getCurrentJointEntropy", (double (mrpt::slam::CMetricMapBuilderRBPF::*)()) &mrpt::slam::CMetricMapBuilderRBPF::getCurrentJointEntropy, "C++: mrpt::slam::CMetricMapBuilderRBPF::getCurrentJointEntropy() --> double");

		{ // mrpt::slam::CMetricMapBuilderRBPF::TConstructionOptions file:mrpt/slam/CMetricMapBuilderRBPF.h line:82
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::slam::CMetricMapBuilderRBPF::TConstructionOptions, std::shared_ptr<mrpt::slam::CMetricMapBuilderRBPF::TConstructionOptions>, PyCallBack_mrpt_slam_CMetricMapBuilderRBPF_TConstructionOptions, mrpt::config::CLoadableOptions> cl(enclosing_class, "TConstructionOptions", "Options for building a CMetricMapBuilderRBPF object, passed to the\n constructor.");
			cl.def( pybind11::init( [](){ return new mrpt::slam::CMetricMapBuilderRBPF::TConstructionOptions(); }, [](){ return new PyCallBack_mrpt_slam_CMetricMapBuilderRBPF_TConstructionOptions(); } ) );
			cl.def( pybind11::init( [](PyCallBack_mrpt_slam_CMetricMapBuilderRBPF_TConstructionOptions const &o){ return new PyCallBack_mrpt_slam_CMetricMapBuilderRBPF_TConstructionOptions(o); } ) );
			cl.def( pybind11::init( [](mrpt::slam::CMetricMapBuilderRBPF::TConstructionOptions const &o){ return new mrpt::slam::CMetricMapBuilderRBPF::TConstructionOptions(o); } ) );
			cl.def_readwrite("insertionLinDistance", &mrpt::slam::CMetricMapBuilderRBPF::TConstructionOptions::insertionLinDistance);
			cl.def_readwrite("insertionAngDistance", &mrpt::slam::CMetricMapBuilderRBPF::TConstructionOptions::insertionAngDistance);
			cl.def_readwrite("localizeLinDistance", &mrpt::slam::CMetricMapBuilderRBPF::TConstructionOptions::localizeLinDistance);
			cl.def_readwrite("localizeAngDistance", &mrpt::slam::CMetricMapBuilderRBPF::TConstructionOptions::localizeAngDistance);
			cl.def_readwrite("PF_options", &mrpt::slam::CMetricMapBuilderRBPF::TConstructionOptions::PF_options);
			cl.def_readwrite("mapsInitializers", &mrpt::slam::CMetricMapBuilderRBPF::TConstructionOptions::mapsInitializers);
			cl.def_readwrite("predictionOptions", &mrpt::slam::CMetricMapBuilderRBPF::TConstructionOptions::predictionOptions);
			cl.def_readwrite("verbosity_level", &mrpt::slam::CMetricMapBuilderRBPF::TConstructionOptions::verbosity_level);
			cl.def("loadFromConfigFile", (void (mrpt::slam::CMetricMapBuilderRBPF::TConstructionOptions::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::slam::CMetricMapBuilderRBPF::TConstructionOptions::loadFromConfigFile, "C++: mrpt::slam::CMetricMapBuilderRBPF::TConstructionOptions::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("source"), pybind11::arg("section"));
			cl.def("assign", (struct mrpt::slam::CMetricMapBuilderRBPF::TConstructionOptions & (mrpt::slam::CMetricMapBuilderRBPF::TConstructionOptions::*)(const struct mrpt::slam::CMetricMapBuilderRBPF::TConstructionOptions &)) &mrpt::slam::CMetricMapBuilderRBPF::TConstructionOptions::operator=, "C++: mrpt::slam::CMetricMapBuilderRBPF::TConstructionOptions::operator=(const struct mrpt::slam::CMetricMapBuilderRBPF::TConstructionOptions &) --> struct mrpt::slam::CMetricMapBuilderRBPF::TConstructionOptions &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

		{ // mrpt::slam::CMetricMapBuilderRBPF::TStats file:mrpt/slam/CMetricMapBuilderRBPF.h line:189
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::slam::CMetricMapBuilderRBPF::TStats, std::shared_ptr<mrpt::slam::CMetricMapBuilderRBPF::TStats>> cl(enclosing_class, "TStats", "This structure will hold stats after each execution of\n processActionObservation");
			cl.def( pybind11::init( [](){ return new mrpt::slam::CMetricMapBuilderRBPF::TStats(); } ) );
			cl.def_readwrite("observationsInserted", &mrpt::slam::CMetricMapBuilderRBPF::TStats::observationsInserted);
		}

	}
}
