#include <chrono>
#include <iterator>
#include <memory>
#include <mrpt/apps/BaseAppDataSource.h>
#include <mrpt/apps/RBPF_SLAM_App.h>
#include <mrpt/apps/RawlogEditApp.h>
#include <mrpt/apps/RawlogGrabberApp.h>
#include <mrpt/core/Clock.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/obs/CAction.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <ostream>
#include <ratio>
#include <sstream> // __str__
#include <string>

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

// mrpt::apps::RBPF_SLAM_App_Rawlog file:mrpt/apps/RBPF_SLAM_App.h line:83
struct PyCallBack_mrpt_apps_RBPF_SLAM_App_Rawlog : public mrpt::apps::RBPF_SLAM_App_Rawlog {
	using mrpt::apps::RBPF_SLAM_App_Rawlog::RBPF_SLAM_App_Rawlog;

	std::string impl_get_usage() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::apps::RBPF_SLAM_App_Rawlog *>(this), "impl_get_usage");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return RBPF_SLAM_App_Rawlog::impl_get_usage();
	}
	bool impl_get_next_observations(class std::shared_ptr<class mrpt::obs::CActionCollection> & a0, class std::shared_ptr<class mrpt::obs::CSensoryFrame> & a1, class std::shared_ptr<class mrpt::obs::CObservation> & a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::apps::RBPF_SLAM_App_Rawlog *>(this), "impl_get_next_observations");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"BaseAppDataSource::impl_get_next_observations\"");
	}
};

void bind_mrpt_apps_RBPF_SLAM_App(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::apps::RBPF_SLAM_App_Base file:mrpt/apps/RBPF_SLAM_App.h line:32
		pybind11::class_<mrpt::apps::RBPF_SLAM_App_Base, std::shared_ptr<mrpt::apps::RBPF_SLAM_App_Base>, mrpt::apps::BaseAppInitializableCLI, mrpt::apps::BaseAppDataSource> cl(M("mrpt::apps"), "RBPF_SLAM_App_Base", "RBPF-SLAM virtual base class for application wrappers.\n\n This virtual base provides the common code to the application rbpf-slam.\n It can be used by users to build their own RBPF-SLAM solution.\n\n \n  mrpt::slam::CMetricMapBuilderRBPF\n \n\n\n ");
		cl.def_readwrite("params", &mrpt::apps::RBPF_SLAM_App_Base::params);
		cl.def_readwrite("quits_with_esc_key", &mrpt::apps::RBPF_SLAM_App_Base::quits_with_esc_key);
		cl.def_readwrite("mapBuilder", &mrpt::apps::RBPF_SLAM_App_Base::mapBuilder);
		cl.def_readwrite("out_estimated_path", &mrpt::apps::RBPF_SLAM_App_Base::out_estimated_path);
		cl.def("run", (void (mrpt::apps::RBPF_SLAM_App_Base::*)()) &mrpt::apps::RBPF_SLAM_App_Base::run, "Runs with the current parameter set. Throws on errors. \n\nC++: mrpt::apps::RBPF_SLAM_App_Base::run() --> void");
		cl.def("assign", (class mrpt::apps::RBPF_SLAM_App_Base & (mrpt::apps::RBPF_SLAM_App_Base::*)(const class mrpt::apps::RBPF_SLAM_App_Base &)) &mrpt::apps::RBPF_SLAM_App_Base::operator=, "C++: mrpt::apps::RBPF_SLAM_App_Base::operator=(const class mrpt::apps::RBPF_SLAM_App_Base &) --> class mrpt::apps::RBPF_SLAM_App_Base &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::apps::RBPF_SLAM_App_Rawlog file:mrpt/apps/RBPF_SLAM_App.h line:83
		pybind11::class_<mrpt::apps::RBPF_SLAM_App_Rawlog, std::shared_ptr<mrpt::apps::RBPF_SLAM_App_Rawlog>, PyCallBack_mrpt_apps_RBPF_SLAM_App_Rawlog, mrpt::apps::RBPF_SLAM_App_Base, mrpt::apps::DataSourceRawlog> cl(M("mrpt::apps"), "RBPF_SLAM_App_Rawlog", "Instance of RBPF_SLAM_App_Base to run mapping from an offline dataset file.");
		cl.def( pybind11::init( [](){ return new mrpt::apps::RBPF_SLAM_App_Rawlog(); }, [](){ return new PyCallBack_mrpt_apps_RBPF_SLAM_App_Rawlog(); } ) );
		cl.def("init", (void (mrpt::apps::RBPF_SLAM_App_Rawlog::*)(const std::string &, const std::string &)) &mrpt::apps::RBPF_SLAM_App_Rawlog::init, "C++: mrpt::apps::RBPF_SLAM_App_Rawlog::init(const std::string &, const std::string &) --> void", pybind11::arg("iniConfigFile"), pybind11::arg("rawlogFile"));
	}
	{ // mrpt::apps::RawlogEditApp file:mrpt/apps/RawlogEditApp.h line:20
		pybind11::class_<mrpt::apps::RawlogEditApp, std::shared_ptr<mrpt::apps::RawlogEditApp>> cl(M("mrpt::apps"), "RawlogEditApp", "The C++ class behind the rawlog-edit CLI tool.\n\n  Refer to the online documentation for rawlog-edit.\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::apps::RawlogEditApp(); } ) );
	}
	{ // mrpt::apps::RawlogGrabberApp file:mrpt/apps/RawlogGrabberApp.h line:30
		pybind11::class_<mrpt::apps::RawlogGrabberApp, std::shared_ptr<mrpt::apps::RawlogGrabberApp>> cl(M("mrpt::apps"), "RawlogGrabberApp", "RawlogGrabber application wrapper class.\n\n \n If the environment variable `MRPT_HWDRIVERS_VERBOSE=1` is defined\n before calling initialize(), verbosity level will be changed to LVL_DEBUG.\n\n \n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::apps::RawlogGrabberApp(); } ) );
		cl.def_readwrite("run_for_seconds", &mrpt::apps::RawlogGrabberApp::run_for_seconds);
		cl.def_readwrite("show_sensor_thread_exceptions", &mrpt::apps::RawlogGrabberApp::show_sensor_thread_exceptions);
		cl.def_readwrite("params", &mrpt::apps::RawlogGrabberApp::params);
		cl.def_readwrite("rawlog_filename", &mrpt::apps::RawlogGrabberApp::rawlog_filename);
		cl.def_readwrite("rawlog_saved_objects", &mrpt::apps::RawlogGrabberApp::rawlog_saved_objects);
		cl.def("run", (void (mrpt::apps::RawlogGrabberApp::*)()) &mrpt::apps::RawlogGrabberApp::run, "Runs with the current parameter set. Throws on errors. \n\nC++: mrpt::apps::RawlogGrabberApp::run() --> void");
		cl.def("isRunning", (bool (mrpt::apps::RawlogGrabberApp::*)() const) &mrpt::apps::RawlogGrabberApp::isRunning, "C++: mrpt::apps::RawlogGrabberApp::isRunning() const --> bool");
		cl.def("SensorThread", (void (mrpt::apps::RawlogGrabberApp::*)(std::string)) &mrpt::apps::RawlogGrabberApp::SensorThread, "@} \n\nC++: mrpt::apps::RawlogGrabberApp::SensorThread(std::string) --> void", pybind11::arg("sensor_label"));
	}
}
