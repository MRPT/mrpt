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
#include <stl_binders.hpp>


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
		cl.def("visitDepthFirst", [](mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP> const &o, const unsigned long & a0, const class std::function<void (unsigned long, const struct mrpt::graphs::CDirectedTree<struct mrpt::nav::TMoveEdgeSE2_TP>::TEdgeInfo &, unsigned long)> & a1) -> void { return o.visitDepthFirst(a0, a1); }, "", pybind11::arg("vroot"), pybind11::arg("user_visitor"));
		cl.def("visitDepthFirst", (void (mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP>::*)(const unsigned long, const class std::function<void (unsigned long, const struct mrpt::graphs::CDirectedTree<struct mrpt::nav::TMoveEdgeSE2_TP>::TEdgeInfo &, unsigned long)> &, const unsigned long) const) &mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP>::visitDepthFirst, "C++: mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP>::visitDepthFirst(const unsigned long, const class std::function<void (unsigned long, const struct mrpt::graphs::CDirectedTree<struct mrpt::nav::TMoveEdgeSE2_TP>::TEdgeInfo &, unsigned long)> &, const unsigned long) const --> void", pybind11::arg("vroot"), pybind11::arg("user_visitor"), pybind11::arg("root_depth_level"));
		cl.def("visitBreadthFirst", [](mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP> const &o, const unsigned long & a0, const class std::function<void (unsigned long, const struct mrpt::graphs::CDirectedTree<struct mrpt::nav::TMoveEdgeSE2_TP>::TEdgeInfo &, unsigned long)> & a1) -> void { return o.visitBreadthFirst(a0, a1); }, "", pybind11::arg("vroot"), pybind11::arg("user_visitor"));
		cl.def("visitBreadthFirst", (void (mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP>::*)(const unsigned long, const class std::function<void (unsigned long, const struct mrpt::graphs::CDirectedTree<struct mrpt::nav::TMoveEdgeSE2_TP>::TEdgeInfo &, unsigned long)> &, const unsigned long) const) &mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP>::visitBreadthFirst, "C++: mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP>::visitBreadthFirst(const unsigned long, const class std::function<void (unsigned long, const struct mrpt::graphs::CDirectedTree<struct mrpt::nav::TMoveEdgeSE2_TP>::TEdgeInfo &, unsigned long)> &, const unsigned long) const --> void", pybind11::arg("vroot"), pybind11::arg("user_visitor"), pybind11::arg("root_depth_level"));
		cl.def("getAsTextDescription", (std::string (mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP>::*)() const) &mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP>::getAsTextDescription, "C++: mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP>::getAsTextDescription() const --> std::string");
		cl.def("assign", (class mrpt::graphs::CDirectedTree<struct mrpt::nav::TMoveEdgeSE2_TP> & (mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP>::*)(const class mrpt::graphs::CDirectedTree<struct mrpt::nav::TMoveEdgeSE2_TP> &)) &mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP>::operator=, "C++: mrpt::graphs::CDirectedTree<mrpt::nav::TMoveEdgeSE2_TP>::operator=(const class mrpt::graphs::CDirectedTree<struct mrpt::nav::TMoveEdgeSE2_TP> &) --> class mrpt::graphs::CDirectedTree<struct mrpt::nav::TMoveEdgeSE2_TP> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
