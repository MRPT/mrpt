#include <iterator>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/slam/CIncrementalMapPartitioner.h>
#include <mrpt/slam/data_association.h>
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

void bind_mrpt_typemeta_TEnumType_5(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::typemeta::internal::bimap file:mrpt/typemeta/TEnumType.h line:22
		pybind11::class_<mrpt::typemeta::internal::bimap<mrpt::slam::similarity_method_t,std::string>, std::shared_ptr<mrpt::typemeta::internal::bimap<mrpt::slam::similarity_method_t,std::string>>> cl(M("mrpt::typemeta::internal"), "bimap_mrpt_slam_similarity_method_t_std_string_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::typemeta::internal::bimap<mrpt::slam::similarity_method_t,std::string>(); } ) );
		cl.def( pybind11::init( [](mrpt::typemeta::internal::bimap<mrpt::slam::similarity_method_t,std::string> const &o){ return new mrpt::typemeta::internal::bimap<mrpt::slam::similarity_method_t,std::string>(o); } ) );
		cl.def_readwrite("m_k2v", &mrpt::typemeta::internal::bimap<mrpt::slam::similarity_method_t,std::string>::m_k2v);
		cl.def_readwrite("m_v2k", &mrpt::typemeta::internal::bimap<mrpt::slam::similarity_method_t,std::string>::m_v2k);
		cl.def("direct", (bool (mrpt::typemeta::internal::bimap<mrpt::slam::similarity_method_t,std::string>::*)(const enum mrpt::slam::similarity_method_t &, std::string &) const) &mrpt::typemeta::internal::bimap<mrpt::slam::similarity_method_t, std::string>::direct, "C++: mrpt::typemeta::internal::bimap<mrpt::slam::similarity_method_t, std::string>::direct(const enum mrpt::slam::similarity_method_t &, std::string &) const --> bool", pybind11::arg("k"), pybind11::arg("out_v"));
		cl.def("inverse", (bool (mrpt::typemeta::internal::bimap<mrpt::slam::similarity_method_t,std::string>::*)(const std::string &, enum mrpt::slam::similarity_method_t &) const) &mrpt::typemeta::internal::bimap<mrpt::slam::similarity_method_t, std::string>::inverse, "C++: mrpt::typemeta::internal::bimap<mrpt::slam::similarity_method_t, std::string>::inverse(const std::string &, enum mrpt::slam::similarity_method_t &) const --> bool", pybind11::arg("v"), pybind11::arg("out_k"));
		cl.def("insert", (void (mrpt::typemeta::internal::bimap<mrpt::slam::similarity_method_t,std::string>::*)(const enum mrpt::slam::similarity_method_t &, const std::string &)) &mrpt::typemeta::internal::bimap<mrpt::slam::similarity_method_t, std::string>::insert, "C++: mrpt::typemeta::internal::bimap<mrpt::slam::similarity_method_t, std::string>::insert(const enum mrpt::slam::similarity_method_t &, const std::string &) --> void", pybind11::arg("k"), pybind11::arg("v"));
		cl.def("assign", (struct mrpt::typemeta::internal::bimap<enum mrpt::slam::similarity_method_t, std::string > & (mrpt::typemeta::internal::bimap<mrpt::slam::similarity_method_t,std::string>::*)(const struct mrpt::typemeta::internal::bimap<enum mrpt::slam::similarity_method_t, std::string > &)) &mrpt::typemeta::internal::bimap<mrpt::slam::similarity_method_t, std::string>::operator=, "C++: mrpt::typemeta::internal::bimap<mrpt::slam::similarity_method_t, std::string>::operator=(const struct mrpt::typemeta::internal::bimap<enum mrpt::slam::similarity_method_t, std::string > &) --> struct mrpt::typemeta::internal::bimap<enum mrpt::slam::similarity_method_t, std::string > &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::typemeta::internal::bimap file:mrpt/typemeta/TEnumType.h line:22
		pybind11::class_<mrpt::typemeta::internal::bimap<mrpt::bayes::TKFMethod,std::string>, std::shared_ptr<mrpt::typemeta::internal::bimap<mrpt::bayes::TKFMethod,std::string>>> cl(M("mrpt::typemeta::internal"), "bimap_mrpt_bayes_TKFMethod_std_string_t", "");
		cl.def( pybind11::init( [](mrpt::typemeta::internal::bimap<mrpt::bayes::TKFMethod,std::string> const &o){ return new mrpt::typemeta::internal::bimap<mrpt::bayes::TKFMethod,std::string>(o); } ) );
		cl.def( pybind11::init( [](){ return new mrpt::typemeta::internal::bimap<mrpt::bayes::TKFMethod,std::string>(); } ) );
		cl.def_readwrite("m_k2v", &mrpt::typemeta::internal::bimap<mrpt::bayes::TKFMethod,std::string>::m_k2v);
		cl.def_readwrite("m_v2k", &mrpt::typemeta::internal::bimap<mrpt::bayes::TKFMethod,std::string>::m_v2k);
		cl.def("direct", (bool (mrpt::typemeta::internal::bimap<mrpt::bayes::TKFMethod,std::string>::*)(const enum mrpt::bayes::TKFMethod &, std::string &) const) &mrpt::typemeta::internal::bimap<mrpt::bayes::TKFMethod, std::string>::direct, "C++: mrpt::typemeta::internal::bimap<mrpt::bayes::TKFMethod, std::string>::direct(const enum mrpt::bayes::TKFMethod &, std::string &) const --> bool", pybind11::arg("k"), pybind11::arg("out_v"));
		cl.def("inverse", (bool (mrpt::typemeta::internal::bimap<mrpt::bayes::TKFMethod,std::string>::*)(const std::string &, enum mrpt::bayes::TKFMethod &) const) &mrpt::typemeta::internal::bimap<mrpt::bayes::TKFMethod, std::string>::inverse, "C++: mrpt::typemeta::internal::bimap<mrpt::bayes::TKFMethod, std::string>::inverse(const std::string &, enum mrpt::bayes::TKFMethod &) const --> bool", pybind11::arg("v"), pybind11::arg("out_k"));
		cl.def("insert", (void (mrpt::typemeta::internal::bimap<mrpt::bayes::TKFMethod,std::string>::*)(const enum mrpt::bayes::TKFMethod &, const std::string &)) &mrpt::typemeta::internal::bimap<mrpt::bayes::TKFMethod, std::string>::insert, "C++: mrpt::typemeta::internal::bimap<mrpt::bayes::TKFMethod, std::string>::insert(const enum mrpt::bayes::TKFMethod &, const std::string &) --> void", pybind11::arg("k"), pybind11::arg("v"));
		cl.def("assign", (struct mrpt::typemeta::internal::bimap<enum mrpt::bayes::TKFMethod, std::string > & (mrpt::typemeta::internal::bimap<mrpt::bayes::TKFMethod,std::string>::*)(const struct mrpt::typemeta::internal::bimap<enum mrpt::bayes::TKFMethod, std::string > &)) &mrpt::typemeta::internal::bimap<mrpt::bayes::TKFMethod, std::string>::operator=, "C++: mrpt::typemeta::internal::bimap<mrpt::bayes::TKFMethod, std::string>::operator=(const struct mrpt::typemeta::internal::bimap<enum mrpt::bayes::TKFMethod, std::string > &) --> struct mrpt::typemeta::internal::bimap<enum mrpt::bayes::TKFMethod, std::string > &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::typemeta::internal::bimap file:mrpt/typemeta/TEnumType.h line:22
		pybind11::class_<mrpt::typemeta::internal::bimap<mrpt::slam::TDataAssociationMethod,std::string>, std::shared_ptr<mrpt::typemeta::internal::bimap<mrpt::slam::TDataAssociationMethod,std::string>>> cl(M("mrpt::typemeta::internal"), "bimap_mrpt_slam_TDataAssociationMethod_std_string_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::typemeta::internal::bimap<mrpt::slam::TDataAssociationMethod,std::string>(); } ) );
		cl.def( pybind11::init( [](mrpt::typemeta::internal::bimap<mrpt::slam::TDataAssociationMethod,std::string> const &o){ return new mrpt::typemeta::internal::bimap<mrpt::slam::TDataAssociationMethod,std::string>(o); } ) );
		cl.def_readwrite("m_k2v", &mrpt::typemeta::internal::bimap<mrpt::slam::TDataAssociationMethod,std::string>::m_k2v);
		cl.def_readwrite("m_v2k", &mrpt::typemeta::internal::bimap<mrpt::slam::TDataAssociationMethod,std::string>::m_v2k);
		cl.def("direct", (bool (mrpt::typemeta::internal::bimap<mrpt::slam::TDataAssociationMethod,std::string>::*)(const enum mrpt::slam::TDataAssociationMethod &, std::string &) const) &mrpt::typemeta::internal::bimap<mrpt::slam::TDataAssociationMethod, std::string>::direct, "C++: mrpt::typemeta::internal::bimap<mrpt::slam::TDataAssociationMethod, std::string>::direct(const enum mrpt::slam::TDataAssociationMethod &, std::string &) const --> bool", pybind11::arg("k"), pybind11::arg("out_v"));
		cl.def("inverse", (bool (mrpt::typemeta::internal::bimap<mrpt::slam::TDataAssociationMethod,std::string>::*)(const std::string &, enum mrpt::slam::TDataAssociationMethod &) const) &mrpt::typemeta::internal::bimap<mrpt::slam::TDataAssociationMethod, std::string>::inverse, "C++: mrpt::typemeta::internal::bimap<mrpt::slam::TDataAssociationMethod, std::string>::inverse(const std::string &, enum mrpt::slam::TDataAssociationMethod &) const --> bool", pybind11::arg("v"), pybind11::arg("out_k"));
		cl.def("insert", (void (mrpt::typemeta::internal::bimap<mrpt::slam::TDataAssociationMethod,std::string>::*)(const enum mrpt::slam::TDataAssociationMethod &, const std::string &)) &mrpt::typemeta::internal::bimap<mrpt::slam::TDataAssociationMethod, std::string>::insert, "C++: mrpt::typemeta::internal::bimap<mrpt::slam::TDataAssociationMethod, std::string>::insert(const enum mrpt::slam::TDataAssociationMethod &, const std::string &) --> void", pybind11::arg("k"), pybind11::arg("v"));
		cl.def("assign", (struct mrpt::typemeta::internal::bimap<enum mrpt::slam::TDataAssociationMethod, std::string > & (mrpt::typemeta::internal::bimap<mrpt::slam::TDataAssociationMethod,std::string>::*)(const struct mrpt::typemeta::internal::bimap<enum mrpt::slam::TDataAssociationMethod, std::string > &)) &mrpt::typemeta::internal::bimap<mrpt::slam::TDataAssociationMethod, std::string>::operator=, "C++: mrpt::typemeta::internal::bimap<mrpt::slam::TDataAssociationMethod, std::string>::operator=(const struct mrpt::typemeta::internal::bimap<enum mrpt::slam::TDataAssociationMethod, std::string > &) --> struct mrpt::typemeta::internal::bimap<enum mrpt::slam::TDataAssociationMethod, std::string > &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::typemeta::internal::bimap file:mrpt/typemeta/TEnumType.h line:22
		pybind11::class_<mrpt::typemeta::internal::bimap<mrpt::slam::TDataAssociationMetric,std::string>, std::shared_ptr<mrpt::typemeta::internal::bimap<mrpt::slam::TDataAssociationMetric,std::string>>> cl(M("mrpt::typemeta::internal"), "bimap_mrpt_slam_TDataAssociationMetric_std_string_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::typemeta::internal::bimap<mrpt::slam::TDataAssociationMetric,std::string>(); } ) );
		cl.def( pybind11::init( [](mrpt::typemeta::internal::bimap<mrpt::slam::TDataAssociationMetric,std::string> const &o){ return new mrpt::typemeta::internal::bimap<mrpt::slam::TDataAssociationMetric,std::string>(o); } ) );
		cl.def_readwrite("m_k2v", &mrpt::typemeta::internal::bimap<mrpt::slam::TDataAssociationMetric,std::string>::m_k2v);
		cl.def_readwrite("m_v2k", &mrpt::typemeta::internal::bimap<mrpt::slam::TDataAssociationMetric,std::string>::m_v2k);
		cl.def("direct", (bool (mrpt::typemeta::internal::bimap<mrpt::slam::TDataAssociationMetric,std::string>::*)(const enum mrpt::slam::TDataAssociationMetric &, std::string &) const) &mrpt::typemeta::internal::bimap<mrpt::slam::TDataAssociationMetric, std::string>::direct, "C++: mrpt::typemeta::internal::bimap<mrpt::slam::TDataAssociationMetric, std::string>::direct(const enum mrpt::slam::TDataAssociationMetric &, std::string &) const --> bool", pybind11::arg("k"), pybind11::arg("out_v"));
		cl.def("inverse", (bool (mrpt::typemeta::internal::bimap<mrpt::slam::TDataAssociationMetric,std::string>::*)(const std::string &, enum mrpt::slam::TDataAssociationMetric &) const) &mrpt::typemeta::internal::bimap<mrpt::slam::TDataAssociationMetric, std::string>::inverse, "C++: mrpt::typemeta::internal::bimap<mrpt::slam::TDataAssociationMetric, std::string>::inverse(const std::string &, enum mrpt::slam::TDataAssociationMetric &) const --> bool", pybind11::arg("v"), pybind11::arg("out_k"));
		cl.def("insert", (void (mrpt::typemeta::internal::bimap<mrpt::slam::TDataAssociationMetric,std::string>::*)(const enum mrpt::slam::TDataAssociationMetric &, const std::string &)) &mrpt::typemeta::internal::bimap<mrpt::slam::TDataAssociationMetric, std::string>::insert, "C++: mrpt::typemeta::internal::bimap<mrpt::slam::TDataAssociationMetric, std::string>::insert(const enum mrpt::slam::TDataAssociationMetric &, const std::string &) --> void", pybind11::arg("k"), pybind11::arg("v"));
		cl.def("assign", (struct mrpt::typemeta::internal::bimap<enum mrpt::slam::TDataAssociationMetric, std::string > & (mrpt::typemeta::internal::bimap<mrpt::slam::TDataAssociationMetric,std::string>::*)(const struct mrpt::typemeta::internal::bimap<enum mrpt::slam::TDataAssociationMetric, std::string > &)) &mrpt::typemeta::internal::bimap<mrpt::slam::TDataAssociationMetric, std::string>::operator=, "C++: mrpt::typemeta::internal::bimap<mrpt::slam::TDataAssociationMetric, std::string>::operator=(const struct mrpt::typemeta::internal::bimap<enum mrpt::slam::TDataAssociationMetric, std::string > &) --> struct mrpt::typemeta::internal::bimap<enum mrpt::slam::TDataAssociationMetric, std::string > &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
