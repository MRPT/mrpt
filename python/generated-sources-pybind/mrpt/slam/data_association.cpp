#include <mrpt/slam/data_association.h>
#include <sstream> // __str__

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

void bind_mrpt_slam_data_association(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::slam::TDataAssociationMethod file:mrpt/slam/data_association.h line:29
	pybind11::enum_<mrpt::slam::TDataAssociationMethod>(M("mrpt::slam"), "TDataAssociationMethod", pybind11::arithmetic(), "Different algorithms for data association, used in\n mrpt::slam::data_association")
		.value("assocNN", mrpt::slam::assocNN)
		.value("assocJCBB", mrpt::slam::assocJCBB)
		.export_values();

;

	// mrpt::slam::TDataAssociationMetric file:mrpt/slam/data_association.h line:40
	pybind11::enum_<mrpt::slam::TDataAssociationMetric>(M("mrpt::slam"), "TDataAssociationMetric", pybind11::arithmetic(), "Different metrics for data association, used in mrpt::slam::data_association\n  For a comparison of both methods see paper \n ")
		.value("metricMaha", mrpt::slam::metricMaha)
		.value("metricML", mrpt::slam::metricML)
		.export_values();

;

	{ // mrpt::slam::TDataAssociationResults file:mrpt/slam/data_association.h line:56
		pybind11::class_<mrpt::slam::TDataAssociationResults, std::shared_ptr<mrpt::slam::TDataAssociationResults>> cl(M("mrpt::slam"), "TDataAssociationResults", "The results from mrpt::slam::data_association_independent_predictions()");
		cl.def( pybind11::init( [](){ return new mrpt::slam::TDataAssociationResults(); } ) );
		cl.def( pybind11::init( [](mrpt::slam::TDataAssociationResults const &o){ return new mrpt::slam::TDataAssociationResults(o); } ) );
		cl.def_readwrite("associations", &mrpt::slam::TDataAssociationResults::associations);
		cl.def_readwrite("distance", &mrpt::slam::TDataAssociationResults::distance);
		cl.def_readwrite("indiv_distances", &mrpt::slam::TDataAssociationResults::indiv_distances);
		cl.def_readwrite("indiv_compatibility", &mrpt::slam::TDataAssociationResults::indiv_compatibility);
		cl.def_readwrite("indiv_compatibility_counts", &mrpt::slam::TDataAssociationResults::indiv_compatibility_counts);
		cl.def_readwrite("nNodesExploredInJCBB", &mrpt::slam::TDataAssociationResults::nNodesExploredInJCBB);
		cl.def("clear", (void (mrpt::slam::TDataAssociationResults::*)()) &mrpt::slam::TDataAssociationResults::clear, "C++: mrpt::slam::TDataAssociationResults::clear() --> void");
		cl.def("assign", (struct mrpt::slam::TDataAssociationResults & (mrpt::slam::TDataAssociationResults::*)(const struct mrpt::slam::TDataAssociationResults &)) &mrpt::slam::TDataAssociationResults::operator=, "C++: mrpt::slam::TDataAssociationResults::operator=(const struct mrpt::slam::TDataAssociationResults &) --> struct mrpt::slam::TDataAssociationResults &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
