#include <ios>
#include <iterator>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/config/CLoadableOptions.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/MatrixVectorBase.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/CTimeLogger.h>
#include <ostream>
#include <sstream> // __str__
#include <streambuf>
#include <string>
#include <string_view>
#include <vector>

#include <functional>
#include <pybind11/pybind11.h>
#include <string>
#include <stl_binders.hpp>


#ifndef BINDER_PYBIND11_TYPE_CASTER
	#define BINDER_PYBIND11_TYPE_CASTER
	PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>)
	PYBIND11_DECLARE_HOLDER_TYPE(T, T*)
	PYBIND11_MAKE_OPAQUE(std::shared_ptr<void>)
#endif

// mrpt::bayes::TKF_options file:mrpt/bayes/CKalmanFilterCapable.h line:59
struct PyCallBack_mrpt_bayes_TKF_options : public mrpt::bayes::TKF_options {
	using mrpt::bayes::TKF_options::TKF_options;

	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::bayes::TKF_options *>(this), "loadFromConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TKF_options::loadFromConfigFile(a0, a1);
	}
	void saveToConfigFile(class mrpt::config::CConfigFileBase & a0, const std::string & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::bayes::TKF_options *>(this), "saveToConfigFile");
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

void bind_mrpt_bayes_CKalmanFilterCapable(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::bayes::TKFMethod file:mrpt/bayes/CKalmanFilterCapable.h line:42
	pybind11::enum_<mrpt::bayes::TKFMethod>(M("mrpt::bayes"), "TKFMethod", pybind11::arithmetic(), "The Kalman Filter algorithm to employ in bayes::CKalmanFilterCapable\n  For further details on each algorithm see the tutorial:\n https://www.mrpt.org/Kalman_Filters\n \n\n bayes::CKalmanFilterCapable::KF_options\n \n\n\n ")
		.value("kfEKFNaive", mrpt::bayes::kfEKFNaive)
		.value("kfEKFAlaDavison", mrpt::bayes::kfEKFAlaDavison)
		.value("kfIKFFull", mrpt::bayes::kfIKFFull)
		.value("kfIKF", mrpt::bayes::kfIKF)
		.export_values();

;

	{ // mrpt::bayes::CKalmanFilterCapable file:mrpt/bayes/CKalmanFilterCapable.h line:225
		pybind11::class_<mrpt::bayes::CKalmanFilterCapable<7,3,3,7,double>, std::shared_ptr<mrpt::bayes::CKalmanFilterCapable<7,3,3,7,double>>> cl(M("mrpt::bayes"), "CKalmanFilterCapable_7_3_3_7_double_t", "");
		cl.def_readonly("KF_options", &mrpt::bayes::CKalmanFilterCapable<7,3,3,7,double>::KF_options);
		cl.def_static("get_vehicle_size", (size_t (*)()) &mrpt::bayes::CKalmanFilterCapable<7, 3, 3, 7, double>::get_vehicle_size, "C++: mrpt::bayes::CKalmanFilterCapable<7, 3, 3, 7, double>::get_vehicle_size() --> size_t");
		cl.def_static("get_observation_size", (size_t (*)()) &mrpt::bayes::CKalmanFilterCapable<7, 3, 3, 7, double>::get_observation_size, "C++: mrpt::bayes::CKalmanFilterCapable<7, 3, 3, 7, double>::get_observation_size() --> size_t");
		cl.def_static("get_feature_size", (size_t (*)()) &mrpt::bayes::CKalmanFilterCapable<7, 3, 3, 7, double>::get_feature_size, "C++: mrpt::bayes::CKalmanFilterCapable<7, 3, 3, 7, double>::get_feature_size() --> size_t");
		cl.def_static("get_action_size", (size_t (*)()) &mrpt::bayes::CKalmanFilterCapable<7, 3, 3, 7, double>::get_action_size, "C++: mrpt::bayes::CKalmanFilterCapable<7, 3, 3, 7, double>::get_action_size() --> size_t");
		cl.def("getNumberOfLandmarksInTheMap", (size_t (mrpt::bayes::CKalmanFilterCapable<7,3,3,7,double>::*)() const) &mrpt::bayes::CKalmanFilterCapable<7, 3, 3, 7, double>::getNumberOfLandmarksInTheMap, "C++: mrpt::bayes::CKalmanFilterCapable<7, 3, 3, 7, double>::getNumberOfLandmarksInTheMap() const --> size_t");
		cl.def("isMapEmpty", (bool (mrpt::bayes::CKalmanFilterCapable<7,3,3,7,double>::*)() const) &mrpt::bayes::CKalmanFilterCapable<7, 3, 3, 7, double>::isMapEmpty, "C++: mrpt::bayes::CKalmanFilterCapable<7, 3, 3, 7, double>::isMapEmpty() const --> bool");
		cl.def("getStateVectorLength", (size_t (mrpt::bayes::CKalmanFilterCapable<7,3,3,7,double>::*)() const) &mrpt::bayes::CKalmanFilterCapable<7, 3, 3, 7, double>::getStateVectorLength, "C++: mrpt::bayes::CKalmanFilterCapable<7, 3, 3, 7, double>::getStateVectorLength() const --> size_t");
		cl.def("internal_getXkk", (class mrpt::math::CVectorDynamic<double> & (mrpt::bayes::CKalmanFilterCapable<7,3,3,7,double>::*)()) &mrpt::bayes::CKalmanFilterCapable<7, 3, 3, 7, double>::internal_getXkk, "C++: mrpt::bayes::CKalmanFilterCapable<7, 3, 3, 7, double>::internal_getXkk() --> class mrpt::math::CVectorDynamic<double> &", pybind11::return_value_policy::automatic);
		cl.def("internal_getPkk", (class mrpt::math::CMatrixDynamic<double> & (mrpt::bayes::CKalmanFilterCapable<7,3,3,7,double>::*)()) &mrpt::bayes::CKalmanFilterCapable<7, 3, 3, 7, double>::internal_getPkk, "C++: mrpt::bayes::CKalmanFilterCapable<7, 3, 3, 7, double>::internal_getPkk() --> class mrpt::math::CMatrixDynamic<double> &", pybind11::return_value_policy::automatic);
		cl.def("OnNewLandmarkAddedToMap", (void (mrpt::bayes::CKalmanFilterCapable<7,3,3,7,double>::*)(const unsigned long, const unsigned long)) &mrpt::bayes::CKalmanFilterCapable<7, 3, 3, 7, double>::OnNewLandmarkAddedToMap, "C++: mrpt::bayes::CKalmanFilterCapable<7, 3, 3, 7, double>::OnNewLandmarkAddedToMap(const unsigned long, const unsigned long) --> void", pybind11::arg("in_obsIdx"), pybind11::arg("in_idxNewFeat"));
		cl.def("OnNormalizeStateVector", (void (mrpt::bayes::CKalmanFilterCapable<7,3,3,7,double>::*)()) &mrpt::bayes::CKalmanFilterCapable<7, 3, 3, 7, double>::OnNormalizeStateVector, "C++: mrpt::bayes::CKalmanFilterCapable<7, 3, 3, 7, double>::OnNormalizeStateVector() --> void");
		cl.def("OnPostIteration", (void (mrpt::bayes::CKalmanFilterCapable<7,3,3,7,double>::*)()) &mrpt::bayes::CKalmanFilterCapable<7, 3, 3, 7, double>::OnPostIteration, "C++: mrpt::bayes::CKalmanFilterCapable<7, 3, 3, 7, double>::OnPostIteration() --> void");
		cl.def("getProfiler", (class mrpt::system::CTimeLogger & (mrpt::bayes::CKalmanFilterCapable<7,3,3,7,double>::*)()) &mrpt::bayes::CKalmanFilterCapable<7, 3, 3, 7, double>::getProfiler, "C++: mrpt::bayes::CKalmanFilterCapable<7, 3, 3, 7, double>::getProfiler() --> class mrpt::system::CTimeLogger &", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::bayes::CKalmanFilterCapable file:mrpt/bayes/CKalmanFilterCapable.h line:225
		pybind11::class_<mrpt::bayes::CKalmanFilterCapable<3,2,2,3,double>, std::shared_ptr<mrpt::bayes::CKalmanFilterCapable<3,2,2,3,double>>> cl(M("mrpt::bayes"), "CKalmanFilterCapable_3_2_2_3_double_t", "");
		cl.def_readonly("KF_options", &mrpt::bayes::CKalmanFilterCapable<3,2,2,3,double>::KF_options);
		cl.def_static("get_vehicle_size", (size_t (*)()) &mrpt::bayes::CKalmanFilterCapable<3, 2, 2, 3, double>::get_vehicle_size, "C++: mrpt::bayes::CKalmanFilterCapable<3, 2, 2, 3, double>::get_vehicle_size() --> size_t");
		cl.def_static("get_observation_size", (size_t (*)()) &mrpt::bayes::CKalmanFilterCapable<3, 2, 2, 3, double>::get_observation_size, "C++: mrpt::bayes::CKalmanFilterCapable<3, 2, 2, 3, double>::get_observation_size() --> size_t");
		cl.def_static("get_feature_size", (size_t (*)()) &mrpt::bayes::CKalmanFilterCapable<3, 2, 2, 3, double>::get_feature_size, "C++: mrpt::bayes::CKalmanFilterCapable<3, 2, 2, 3, double>::get_feature_size() --> size_t");
		cl.def_static("get_action_size", (size_t (*)()) &mrpt::bayes::CKalmanFilterCapable<3, 2, 2, 3, double>::get_action_size, "C++: mrpt::bayes::CKalmanFilterCapable<3, 2, 2, 3, double>::get_action_size() --> size_t");
		cl.def("getNumberOfLandmarksInTheMap", (size_t (mrpt::bayes::CKalmanFilterCapable<3,2,2,3,double>::*)() const) &mrpt::bayes::CKalmanFilterCapable<3, 2, 2, 3, double>::getNumberOfLandmarksInTheMap, "C++: mrpt::bayes::CKalmanFilterCapable<3, 2, 2, 3, double>::getNumberOfLandmarksInTheMap() const --> size_t");
		cl.def("isMapEmpty", (bool (mrpt::bayes::CKalmanFilterCapable<3,2,2,3,double>::*)() const) &mrpt::bayes::CKalmanFilterCapable<3, 2, 2, 3, double>::isMapEmpty, "C++: mrpt::bayes::CKalmanFilterCapable<3, 2, 2, 3, double>::isMapEmpty() const --> bool");
		cl.def("getStateVectorLength", (size_t (mrpt::bayes::CKalmanFilterCapable<3,2,2,3,double>::*)() const) &mrpt::bayes::CKalmanFilterCapable<3, 2, 2, 3, double>::getStateVectorLength, "C++: mrpt::bayes::CKalmanFilterCapable<3, 2, 2, 3, double>::getStateVectorLength() const --> size_t");
		cl.def("internal_getXkk", (class mrpt::math::CVectorDynamic<double> & (mrpt::bayes::CKalmanFilterCapable<3,2,2,3,double>::*)()) &mrpt::bayes::CKalmanFilterCapable<3, 2, 2, 3, double>::internal_getXkk, "C++: mrpt::bayes::CKalmanFilterCapable<3, 2, 2, 3, double>::internal_getXkk() --> class mrpt::math::CVectorDynamic<double> &", pybind11::return_value_policy::automatic);
		cl.def("internal_getPkk", (class mrpt::math::CMatrixDynamic<double> & (mrpt::bayes::CKalmanFilterCapable<3,2,2,3,double>::*)()) &mrpt::bayes::CKalmanFilterCapable<3, 2, 2, 3, double>::internal_getPkk, "C++: mrpt::bayes::CKalmanFilterCapable<3, 2, 2, 3, double>::internal_getPkk() --> class mrpt::math::CMatrixDynamic<double> &", pybind11::return_value_policy::automatic);
		cl.def("OnNewLandmarkAddedToMap", (void (mrpt::bayes::CKalmanFilterCapable<3,2,2,3,double>::*)(const unsigned long, const unsigned long)) &mrpt::bayes::CKalmanFilterCapable<3, 2, 2, 3, double>::OnNewLandmarkAddedToMap, "C++: mrpt::bayes::CKalmanFilterCapable<3, 2, 2, 3, double>::OnNewLandmarkAddedToMap(const unsigned long, const unsigned long) --> void", pybind11::arg("in_obsIdx"), pybind11::arg("in_idxNewFeat"));
		cl.def("OnNormalizeStateVector", (void (mrpt::bayes::CKalmanFilterCapable<3,2,2,3,double>::*)()) &mrpt::bayes::CKalmanFilterCapable<3, 2, 2, 3, double>::OnNormalizeStateVector, "C++: mrpt::bayes::CKalmanFilterCapable<3, 2, 2, 3, double>::OnNormalizeStateVector() --> void");
		cl.def("OnPostIteration", (void (mrpt::bayes::CKalmanFilterCapable<3,2,2,3,double>::*)()) &mrpt::bayes::CKalmanFilterCapable<3, 2, 2, 3, double>::OnPostIteration, "C++: mrpt::bayes::CKalmanFilterCapable<3, 2, 2, 3, double>::OnPostIteration() --> void");
		cl.def("getProfiler", (class mrpt::system::CTimeLogger & (mrpt::bayes::CKalmanFilterCapable<3,2,2,3,double>::*)()) &mrpt::bayes::CKalmanFilterCapable<3, 2, 2, 3, double>::getProfiler, "C++: mrpt::bayes::CKalmanFilterCapable<3, 2, 2, 3, double>::getProfiler() --> class mrpt::system::CTimeLogger &", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::bayes::TKF_options file:mrpt/bayes/CKalmanFilterCapable.h line:59
		pybind11::class_<mrpt::bayes::TKF_options, std::shared_ptr<mrpt::bayes::TKF_options>, PyCallBack_mrpt_bayes_TKF_options, mrpt::config::CLoadableOptions> cl(M("mrpt::bayes"), "TKF_options", "Generic options for the Kalman Filter algorithm in itself.\n \n\n\n ");
		cl.def( pybind11::init<enum mrpt::system::VerbosityLevel &>(), pybind11::arg("verb_level_ref") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_bayes_TKF_options const &o){ return new PyCallBack_mrpt_bayes_TKF_options(o); } ) );
		cl.def( pybind11::init( [](mrpt::bayes::TKF_options const &o){ return new mrpt::bayes::TKF_options(o); } ) );
		cl.def_readwrite("method", &mrpt::bayes::TKF_options::method);
		cl.def_readwrite("IKF_iterations", &mrpt::bayes::TKF_options::IKF_iterations);
		cl.def_readwrite("enable_profiler", &mrpt::bayes::TKF_options::enable_profiler);
		cl.def_readwrite("use_analytic_transition_jacobian", &mrpt::bayes::TKF_options::use_analytic_transition_jacobian);
		cl.def_readwrite("use_analytic_observation_jacobian", &mrpt::bayes::TKF_options::use_analytic_observation_jacobian);
		cl.def_readwrite("debug_verify_analytic_jacobians", &mrpt::bayes::TKF_options::debug_verify_analytic_jacobians);
		cl.def_readwrite("debug_verify_analytic_jacobians_threshold", &mrpt::bayes::TKF_options::debug_verify_analytic_jacobians_threshold);
		cl.def("loadFromConfigFile", (void (mrpt::bayes::TKF_options::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::bayes::TKF_options::loadFromConfigFile, "C++: mrpt::bayes::TKF_options::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("iniFile"), pybind11::arg("section"));
	}
}
