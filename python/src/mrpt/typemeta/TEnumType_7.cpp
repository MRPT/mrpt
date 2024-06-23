#include <iterator>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/typemeta/TEnumType.h>
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

void bind_mrpt_typemeta_TEnumType_7(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::typemeta::TEnumType file:mrpt/typemeta/TEnumType.h line:91
		pybind11::class_<mrpt::typemeta::TEnumType<mrpt::bayes::TKFMethod>, std::shared_ptr<mrpt::typemeta::TEnumType<mrpt::bayes::TKFMethod>>> cl(M("mrpt::typemeta"), "TEnumType_mrpt_bayes_TKFMethod_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::typemeta::TEnumType<mrpt::bayes::TKFMethod>(); } ) );
		cl.def_static("name2value", (enum mrpt::bayes::TKFMethod (*)(const std::string &)) &mrpt::typemeta::TEnumType<mrpt::bayes::TKFMethod>::name2value, "C++: mrpt::typemeta::TEnumType<mrpt::bayes::TKFMethod>::name2value(const std::string &) --> enum mrpt::bayes::TKFMethod", pybind11::arg("name"));
		cl.def_static("value2name", (std::string (*)(const enum mrpt::bayes::TKFMethod)) &mrpt::typemeta::TEnumType<mrpt::bayes::TKFMethod>::value2name, "C++: mrpt::typemeta::TEnumType<mrpt::bayes::TKFMethod>::value2name(const enum mrpt::bayes::TKFMethod) --> std::string", pybind11::arg("val"));
		cl.def_static("getBimap", (struct mrpt::typemeta::internal::bimap<enum mrpt::bayes::TKFMethod, std::string > & (*)()) &mrpt::typemeta::TEnumType<mrpt::bayes::TKFMethod>::getBimap, "C++: mrpt::typemeta::TEnumType<mrpt::bayes::TKFMethod>::getBimap() --> struct mrpt::typemeta::internal::bimap<enum mrpt::bayes::TKFMethod, std::string > &", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::typemeta::TEnumType file:mrpt/typemeta/TEnumType.h line:91
		pybind11::class_<mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>, std::shared_ptr<mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>>> cl(M("mrpt::typemeta"), "TEnumType_mrpt_system_VerbosityLevel_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>(); } ) );
		cl.def_static("name2value", (enum mrpt::system::VerbosityLevel (*)(const std::string &)) &mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>::name2value, "C++: mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>::name2value(const std::string &) --> enum mrpt::system::VerbosityLevel", pybind11::arg("name"));
		cl.def_static("value2name", (std::string (*)(const enum mrpt::system::VerbosityLevel)) &mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>::value2name, "C++: mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>::value2name(const enum mrpt::system::VerbosityLevel) --> std::string", pybind11::arg("val"));
		cl.def_static("getBimap", (struct mrpt::typemeta::internal::bimap<enum mrpt::system::VerbosityLevel, std::string > & (*)()) &mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>::getBimap, "C++: mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>::getBimap() --> struct mrpt::typemeta::internal::bimap<enum mrpt::system::VerbosityLevel, std::string > &", pybind11::return_value_policy::automatic);
	}
}
