#include <chrono>
#include <iterator>
#include <memory>
#include <mrpt/apps/BaseAppDataSource.h>
#include <mrpt/apps/BaseAppInitializableCLI.h>
#include <mrpt/apps/DataSourceRawlog.h>
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

// mrpt::apps::BaseAppDataSource file:mrpt/apps/BaseAppDataSource.h line:21
struct PyCallBack_mrpt_apps_BaseAppDataSource : public mrpt::apps::BaseAppDataSource {
	using mrpt::apps::BaseAppDataSource::BaseAppDataSource;

	bool impl_get_next_observations(class std::shared_ptr<class mrpt::obs::CActionCollection> & a0, class std::shared_ptr<class mrpt::obs::CSensoryFrame> & a1, class std::shared_ptr<class mrpt::obs::CObservation> & a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::apps::BaseAppDataSource *>(this), "impl_get_next_observations");
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

// mrpt::apps::DataSourceRawlog file:mrpt/apps/DataSourceRawlog.h line:22
struct PyCallBack_mrpt_apps_DataSourceRawlog : public mrpt::apps::DataSourceRawlog {
	using mrpt::apps::DataSourceRawlog::DataSourceRawlog;

	bool impl_get_next_observations(class std::shared_ptr<class mrpt::obs::CActionCollection> & a0, class std::shared_ptr<class mrpt::obs::CSensoryFrame> & a1, class std::shared_ptr<class mrpt::obs::CObservation> & a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::apps::DataSourceRawlog *>(this), "impl_get_next_observations");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return DataSourceRawlog::impl_get_next_observations(a0, a1, a2);
	}
};

void bind_mrpt_apps_BaseAppDataSource(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::apps::BaseAppDataSource file:mrpt/apps/BaseAppDataSource.h line:21
		pybind11::class_<mrpt::apps::BaseAppDataSource, std::shared_ptr<mrpt::apps::BaseAppDataSource>, PyCallBack_mrpt_apps_BaseAppDataSource> cl(M("mrpt::apps"), "BaseAppDataSource", "Virtual interface for offline datasets (rawlog) or live sensors.\n\n \n\n ");
		cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_apps_BaseAppDataSource(); } ) );
		cl.def(pybind11::init<PyCallBack_mrpt_apps_BaseAppDataSource const &>());
		cl.def("assign", (class mrpt::apps::BaseAppDataSource & (mrpt::apps::BaseAppDataSource::*)(const class mrpt::apps::BaseAppDataSource &)) &mrpt::apps::BaseAppDataSource::operator=, "C++: mrpt::apps::BaseAppDataSource::operator=(const class mrpt::apps::BaseAppDataSource &) --> class mrpt::apps::BaseAppDataSource &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::apps::BaseAppInitializableCLI file:mrpt/apps/BaseAppInitializableCLI.h line:19
		pybind11::class_<mrpt::apps::BaseAppInitializableCLI, std::shared_ptr<mrpt::apps::BaseAppInitializableCLI>> cl(M("mrpt::apps"), "BaseAppInitializableCLI", "Virtual interface for applications that initialize from CLI parameters.\n\n \n\n ");
		cl.def("assign", (class mrpt::apps::BaseAppInitializableCLI & (mrpt::apps::BaseAppInitializableCLI::*)(const class mrpt::apps::BaseAppInitializableCLI &)) &mrpt::apps::BaseAppInitializableCLI::operator=, "C++: mrpt::apps::BaseAppInitializableCLI::operator=(const class mrpt::apps::BaseAppInitializableCLI &) --> class mrpt::apps::BaseAppInitializableCLI &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::apps::DataSourceRawlog file:mrpt/apps/DataSourceRawlog.h line:22
		pybind11::class_<mrpt::apps::DataSourceRawlog, std::shared_ptr<mrpt::apps::DataSourceRawlog>, PyCallBack_mrpt_apps_DataSourceRawlog, mrpt::apps::BaseAppDataSource> cl(M("mrpt::apps"), "DataSourceRawlog", "Implementation of BaseAppDataSource for reading from a rawlog file\n\n \n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::apps::DataSourceRawlog(); }, [](){ return new PyCallBack_mrpt_apps_DataSourceRawlog(); } ) );
	}
}
