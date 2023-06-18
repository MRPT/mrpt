#include <functional>
#include <iterator>
#include <memory>
#include <mrpt/graphs/CDirectedTree.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/nav/planners/TMoveTree.h>
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

void bind_mrpt_graphs_CDirectedTree(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::graphs::CDirectedTree file:mrpt/graphs/CDirectedTree.h line:54
		pybind11::class_<mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP>, std::shared_ptr<mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP>>> cl(M("mrpt::graphs"), "CDirectedTree_mrpt_nav_TMoveEdgeSE2_TP_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP>(); } ) );
		cl.def( pybind11::init( [](mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP> const &o){ return new mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP>(o); } ) );
		cl.def_readwrite("root", &mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP>::root);
		cl.def_readwrite("edges_to_children", &mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP>::edges_to_children);
		cl.def("clear", (void (mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP>::*)()) &mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP>::clear, "C++: mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP>::clear() --> void");
		cl.def("getAsTextDescription", (std::string (mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP>::*)() const) &mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP>::getAsTextDescription, "C++: mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP>::getAsTextDescription() const --> std::string");
		cl.def("assign", (class mrpt::graphs::CDirectedTree<struct mrpt::nav::TMoveEdgeSE2_TP> & (mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP>::*)(const class mrpt::graphs::CDirectedTree<struct mrpt::nav::TMoveEdgeSE2_TP> &)) &mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP>::operator=, "C++: mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP>::operator=(const class mrpt::graphs::CDirectedTree<struct mrpt::nav::TMoveEdgeSE2_TP> &) --> class mrpt::graphs::CDirectedTree<struct mrpt::nav::TMoveEdgeSE2_TP> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
