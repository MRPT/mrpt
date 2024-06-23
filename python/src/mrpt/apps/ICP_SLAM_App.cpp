#include <chrono>
#include <iterator>
#include <memory>
#include <mrpt/apps/BaseAppDataSource.h>
#include <mrpt/apps/ICP_SLAM_App.h>
#include <mrpt/apps/KFSLAMApp.h>
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

// mrpt::apps::ICP_SLAM_App_Rawlog file:mrpt/apps/ICP_SLAM_App.h line:82
struct PyCallBack_mrpt_apps_ICP_SLAM_App_Rawlog : public mrpt::apps::ICP_SLAM_App_Rawlog {
	using mrpt::apps::ICP_SLAM_App_Rawlog::ICP_SLAM_App_Rawlog;

	std::string impl_get_usage() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::apps::ICP_SLAM_App_Rawlog *>(this), "impl_get_usage");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return ICP_SLAM_App_Rawlog::impl_get_usage();
	}
	bool impl_get_next_observations(class std::shared_ptr<class mrpt::obs::CActionCollection> & a0, class std::shared_ptr<class mrpt::obs::CSensoryFrame> & a1, class std::shared_ptr<class mrpt::obs::CObservation> & a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::apps::ICP_SLAM_App_Rawlog *>(this), "impl_get_next_observations");
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

// mrpt::apps::ICP_SLAM_App_Live file:mrpt/apps/ICP_SLAM_App.h line:94
struct PyCallBack_mrpt_apps_ICP_SLAM_App_Live : public mrpt::apps::ICP_SLAM_App_Live {
	using mrpt::apps::ICP_SLAM_App_Live::ICP_SLAM_App_Live;

	std::string impl_get_usage() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::apps::ICP_SLAM_App_Live *>(this), "impl_get_usage");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return ICP_SLAM_App_Live::impl_get_usage();
	}
	bool impl_get_next_observations(class std::shared_ptr<class mrpt::obs::CActionCollection> & a0, class std::shared_ptr<class mrpt::obs::CSensoryFrame> & a1, class std::shared_ptr<class mrpt::obs::CObservation> & a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::apps::ICP_SLAM_App_Live *>(this), "impl_get_next_observations");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return ICP_SLAM_App_Live::impl_get_next_observations(a0, a1, a2);
	}
};

void bind_mrpt_apps_ICP_SLAM_App(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::apps::ICP_SLAM_App_Base file:mrpt/apps/ICP_SLAM_App.h line:32
		pybind11::class_<mrpt::apps::ICP_SLAM_App_Base, std::shared_ptr<mrpt::apps::ICP_SLAM_App_Base>, mrpt::apps::BaseAppInitializableCLI, mrpt::apps::BaseAppDataSource> cl(M("mrpt::apps"), "ICP_SLAM_App_Base", "ICP-SLAM virtual base class for application wrappers.\n\n This virtual base provides the common code to the applications icp-slam and\n icp-slam-live, and could be used by users to build their own ICP-SLAM\n solution.\n\n \n  mrpt::slam::CMetricMapBuilderICP\n \n\n\n ");
		cl.def_readwrite("params", &mrpt::apps::ICP_SLAM_App_Base::params);
		cl.def_readwrite("quits_with_esc_key", &mrpt::apps::ICP_SLAM_App_Base::quits_with_esc_key);
		cl.def_readwrite("out_estimated_path", &mrpt::apps::ICP_SLAM_App_Base::out_estimated_path);
		cl.def("run", (void (mrpt::apps::ICP_SLAM_App_Base::*)()) &mrpt::apps::ICP_SLAM_App_Base::run, "Runs with the current parameter set. Throws on errors. \n\nC++: mrpt::apps::ICP_SLAM_App_Base::run() --> void");
		cl.def("assign", (class mrpt::apps::ICP_SLAM_App_Base & (mrpt::apps::ICP_SLAM_App_Base::*)(const class mrpt::apps::ICP_SLAM_App_Base &)) &mrpt::apps::ICP_SLAM_App_Base::operator=, "C++: mrpt::apps::ICP_SLAM_App_Base::operator=(const class mrpt::apps::ICP_SLAM_App_Base &) --> class mrpt::apps::ICP_SLAM_App_Base &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::apps::ICP_SLAM_App_Rawlog file:mrpt/apps/ICP_SLAM_App.h line:82
		pybind11::class_<mrpt::apps::ICP_SLAM_App_Rawlog, std::shared_ptr<mrpt::apps::ICP_SLAM_App_Rawlog>, PyCallBack_mrpt_apps_ICP_SLAM_App_Rawlog, mrpt::apps::ICP_SLAM_App_Base, mrpt::apps::DataSourceRawlog> cl(M("mrpt::apps"), "ICP_SLAM_App_Rawlog", "Instance of ICP_SLAM_App_Base to run mapping from an offline dataset file.");
		cl.def( pybind11::init( [](){ return new mrpt::apps::ICP_SLAM_App_Rawlog(); }, [](){ return new PyCallBack_mrpt_apps_ICP_SLAM_App_Rawlog(); } ) );
	}
	{ // mrpt::apps::ICP_SLAM_App_Live file:mrpt/apps/ICP_SLAM_App.h line:94
		pybind11::class_<mrpt::apps::ICP_SLAM_App_Live, std::shared_ptr<mrpt::apps::ICP_SLAM_App_Live>, PyCallBack_mrpt_apps_ICP_SLAM_App_Live, mrpt::apps::ICP_SLAM_App_Base> cl(M("mrpt::apps"), "ICP_SLAM_App_Live", "Instance of ICP_SLAM_App_Base to run mapping from a live LIDAR sensor.");
		cl.def( pybind11::init( [](){ return new mrpt::apps::ICP_SLAM_App_Live(); }, [](){ return new PyCallBack_mrpt_apps_ICP_SLAM_App_Live(); } ) );
	}
	{ // mrpt::apps::KFSLAMApp file:mrpt/apps/KFSLAMApp.h line:24
		pybind11::class_<mrpt::apps::KFSLAMApp, std::shared_ptr<mrpt::apps::KFSLAMApp>> cl(M("mrpt::apps"), "KFSLAMApp", "EKF-SLAM application wrapper class.\n\n \n mrpt::slam::CRangeBearingKFSLAM2D, mrpt::slam::CRangeBearingKFSLAM\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::apps::KFSLAMApp(); } ) );
		cl.def( pybind11::init( [](mrpt::apps::KFSLAMApp const &o){ return new mrpt::apps::KFSLAMApp(o); } ) );
		cl.def_readwrite("params", &mrpt::apps::KFSLAMApp::params);
		cl.def_readwrite("rawlogFileName", &mrpt::apps::KFSLAMApp::rawlogFileName);
		cl.def_readwrite("loc_error_wrt_gt", &mrpt::apps::KFSLAMApp::loc_error_wrt_gt);
		cl.def("run", (void (mrpt::apps::KFSLAMApp::*)()) &mrpt::apps::KFSLAMApp::run, "Runs with the current parameter set. Throws on errors. \n\nC++: mrpt::apps::KFSLAMApp::run() --> void");
		cl.def("assign", (class mrpt::apps::KFSLAMApp & (mrpt::apps::KFSLAMApp::*)(const class mrpt::apps::KFSLAMApp &)) &mrpt::apps::KFSLAMApp::operator=, "C++: mrpt::apps::KFSLAMApp::operator=(const class mrpt::apps::KFSLAMApp &) --> class mrpt::apps::KFSLAMApp &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
