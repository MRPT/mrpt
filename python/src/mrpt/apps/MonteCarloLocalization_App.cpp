#include <chrono>
#include <iterator>
#include <memory>
#include <mrpt/apps/BaseAppDataSource.h>
#include <mrpt/apps/MonteCarloLocalization_App.h>
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

// mrpt::apps::MonteCarloLocalization_Rawlog file:mrpt/apps/MonteCarloLocalization_App.h line:100
struct PyCallBack_mrpt_apps_MonteCarloLocalization_Rawlog : public mrpt::apps::MonteCarloLocalization_Rawlog {
	using mrpt::apps::MonteCarloLocalization_Rawlog::MonteCarloLocalization_Rawlog;

	std::string impl_get_usage() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::apps::MonteCarloLocalization_Rawlog *>(this), "impl_get_usage");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return MonteCarloLocalization_Rawlog::impl_get_usage();
	}
	bool impl_get_next_observations(class std::shared_ptr<class mrpt::obs::CActionCollection> & a0, class std::shared_ptr<class mrpt::obs::CSensoryFrame> & a1, class std::shared_ptr<class mrpt::obs::CObservation> & a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::apps::MonteCarloLocalization_Rawlog *>(this), "impl_get_next_observations");
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

void bind_mrpt_apps_MonteCarloLocalization_App(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::apps::MonteCarloLocalization_Base file:mrpt/apps/MonteCarloLocalization_App.h line:27
		pybind11::class_<mrpt::apps::MonteCarloLocalization_Base, std::shared_ptr<mrpt::apps::MonteCarloLocalization_Base>, mrpt::apps::BaseAppInitializableCLI, mrpt::apps::BaseAppDataSource> cl(M("mrpt::apps"), "MonteCarloLocalization_Base", "MonteCarlo (Particle filter) localization wrapper class for CLI or custom\n applications: virtual base class, which can be used as base for custom\n user robotic applications.\n\n \n\n ");
		cl.def_readwrite("params", &mrpt::apps::MonteCarloLocalization_Base::params);
		cl.def_readwrite("allow_quit_on_esc_key", &mrpt::apps::MonteCarloLocalization_Base::allow_quit_on_esc_key);
		cl.def_readwrite("fill_out_estimated_path", &mrpt::apps::MonteCarloLocalization_Base::fill_out_estimated_path);
		cl.def_readwrite("out_estimated_path", &mrpt::apps::MonteCarloLocalization_Base::out_estimated_path);
		cl.def("run", (void (mrpt::apps::MonteCarloLocalization_Base::*)()) &mrpt::apps::MonteCarloLocalization_Base::run, "Runs with the current parameter set. Throws on errors. \n\nC++: mrpt::apps::MonteCarloLocalization_Base::run() --> void");
		cl.def("assign", (class mrpt::apps::MonteCarloLocalization_Base & (mrpt::apps::MonteCarloLocalization_Base::*)(const class mrpt::apps::MonteCarloLocalization_Base &)) &mrpt::apps::MonteCarloLocalization_Base::operator=, "C++: mrpt::apps::MonteCarloLocalization_Base::operator=(const class mrpt::apps::MonteCarloLocalization_Base &) --> class mrpt::apps::MonteCarloLocalization_Base &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::apps::MonteCarloLocalization_Rawlog file:mrpt/apps/MonteCarloLocalization_App.h line:100
		pybind11::class_<mrpt::apps::MonteCarloLocalization_Rawlog, std::shared_ptr<mrpt::apps::MonteCarloLocalization_Rawlog>, PyCallBack_mrpt_apps_MonteCarloLocalization_Rawlog, mrpt::apps::MonteCarloLocalization_Base, mrpt::apps::DataSourceRawlog> cl(M("mrpt::apps"), "MonteCarloLocalization_Rawlog", "MonteCarlo (Particle filter) localization wrapper class, reading from a\n rawlog dataset.\n\n \n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::apps::MonteCarloLocalization_Rawlog(); }, [](){ return new PyCallBack_mrpt_apps_MonteCarloLocalization_Rawlog(); } ) );
	}
}
