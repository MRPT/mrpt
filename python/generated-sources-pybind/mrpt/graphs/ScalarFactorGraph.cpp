#include <mrpt/graphs/ScalarFactorGraph.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/MatrixVectorBase.h>
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

// mrpt::graphs::ScalarFactorGraph::FactorBase file:mrpt/graphs/ScalarFactorGraph.h line:46
struct PyCallBack_mrpt_graphs_ScalarFactorGraph_FactorBase : public mrpt::graphs::ScalarFactorGraph::FactorBase {
	using mrpt::graphs::ScalarFactorGraph::FactorBase::FactorBase;

	double evaluateResidual() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::graphs::ScalarFactorGraph::FactorBase *>(this), "evaluateResidual");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"FactorBase::evaluateResidual\"");
	}
	double getInformation() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::graphs::ScalarFactorGraph::FactorBase *>(this), "getInformation");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"FactorBase::getInformation\"");
	}
};

// mrpt::graphs::ScalarFactorGraph::UnaryFactorVirtualBase file:mrpt/graphs/ScalarFactorGraph.h line:56
struct PyCallBack_mrpt_graphs_ScalarFactorGraph_UnaryFactorVirtualBase : public mrpt::graphs::ScalarFactorGraph::UnaryFactorVirtualBase {
	using mrpt::graphs::ScalarFactorGraph::UnaryFactorVirtualBase::UnaryFactorVirtualBase;

	void evalJacobian(double & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::graphs::ScalarFactorGraph::UnaryFactorVirtualBase *>(this), "evalJacobian");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"UnaryFactorVirtualBase::evalJacobian\"");
	}
	double evaluateResidual() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::graphs::ScalarFactorGraph::UnaryFactorVirtualBase *>(this), "evaluateResidual");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"FactorBase::evaluateResidual\"");
	}
	double getInformation() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::graphs::ScalarFactorGraph::UnaryFactorVirtualBase *>(this), "getInformation");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"FactorBase::getInformation\"");
	}
};

// mrpt::graphs::ScalarFactorGraph::BinaryFactorVirtualBase file:mrpt/graphs/ScalarFactorGraph.h line:64
struct PyCallBack_mrpt_graphs_ScalarFactorGraph_BinaryFactorVirtualBase : public mrpt::graphs::ScalarFactorGraph::BinaryFactorVirtualBase {
	using mrpt::graphs::ScalarFactorGraph::BinaryFactorVirtualBase::BinaryFactorVirtualBase;

	void evalJacobian(double & a0, double & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::graphs::ScalarFactorGraph::BinaryFactorVirtualBase *>(this), "evalJacobian");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"BinaryFactorVirtualBase::evalJacobian\"");
	}
	double evaluateResidual() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::graphs::ScalarFactorGraph::BinaryFactorVirtualBase *>(this), "evaluateResidual");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"FactorBase::evaluateResidual\"");
	}
	double getInformation() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::graphs::ScalarFactorGraph::BinaryFactorVirtualBase *>(this), "getInformation");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"FactorBase::getInformation\"");
	}
};

void bind_mrpt_graphs_ScalarFactorGraph(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::graphs::ScalarFactorGraph file:mrpt/graphs/ScalarFactorGraph.h line:41
		pybind11::class_<mrpt::graphs::ScalarFactorGraph, std::shared_ptr<mrpt::graphs::ScalarFactorGraph>> cl(M("mrpt::graphs"), "ScalarFactorGraph", "Sparse solver for GMRF (Gaussian Markov Random Fields) graphical models.\n  The design of this class is optimized for large problems (e.g. >1e3 nodes,\n >1e4 constrainst)\n  by leaving to the user/caller the responsibility of allocating all \"nodes\"\n and constraints.\n  This class can be seen as an intermediary solution between current methods\n in mrpt::graphslam and the well-known G2O library:\n\n  Assumptions/limitations:\n   - Linear error functions (for now).\n   - Scalar (1-dim) error functions.\n   - Gaussian factors.\n   - Solver: Eigen SparseQR.\n\n  Usage:\n   - Call initialize() to set the number of nodes.\n   - Call addConstraints() to insert constraints. This may be called more than\n once.\n   - Call updateEstimation() to run one step of the linear SparseQR solver.\n\n \n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::graphs::ScalarFactorGraph(); } ) );
		cl.def( pybind11::init( [](mrpt::graphs::ScalarFactorGraph const &o){ return new mrpt::graphs::ScalarFactorGraph(o); } ) );
		cl.def("clear", (void (mrpt::graphs::ScalarFactorGraph::*)()) &mrpt::graphs::ScalarFactorGraph::clear, "Reset state: remove all constraints and nodes. \n\nC++: mrpt::graphs::ScalarFactorGraph::clear() --> void");
		cl.def("initialize", (void (mrpt::graphs::ScalarFactorGraph::*)(size_t)) &mrpt::graphs::ScalarFactorGraph::initialize, "Initialize the GMRF internal state and copy the prior factors. \n\nC++: mrpt::graphs::ScalarFactorGraph::initialize(size_t) --> void", pybind11::arg("nodeCount"));
		cl.def("addConstraint", (void (mrpt::graphs::ScalarFactorGraph::*)(const struct mrpt::graphs::ScalarFactorGraph::UnaryFactorVirtualBase &)) &mrpt::graphs::ScalarFactorGraph::addConstraint, "Insert constraints into the GMRF problem.\n \n\n List of user-implemented constraints.\n **A pointer to the passed object is kept**, but memory ownship *REMAINS*\n being responsability of the caller. This is\n done such that arrays/vectors of constraints can be more efficiently\n allocated if their type is known at build time.\n\nC++: mrpt::graphs::ScalarFactorGraph::addConstraint(const struct mrpt::graphs::ScalarFactorGraph::UnaryFactorVirtualBase &) --> void", pybind11::arg("listOfConstraints"));
		cl.def("addConstraint", (void (mrpt::graphs::ScalarFactorGraph::*)(const struct mrpt::graphs::ScalarFactorGraph::BinaryFactorVirtualBase &)) &mrpt::graphs::ScalarFactorGraph::addConstraint, "C++: mrpt::graphs::ScalarFactorGraph::addConstraint(const struct mrpt::graphs::ScalarFactorGraph::BinaryFactorVirtualBase &) --> void", pybind11::arg("listOfConstraints"));
		cl.def("eraseConstraint", (bool (mrpt::graphs::ScalarFactorGraph::*)(const struct mrpt::graphs::ScalarFactorGraph::FactorBase &)) &mrpt::graphs::ScalarFactorGraph::eraseConstraint, "Removes a constraint. Return true if found and deleted correctly. \n\nC++: mrpt::graphs::ScalarFactorGraph::eraseConstraint(const struct mrpt::graphs::ScalarFactorGraph::FactorBase &) --> bool", pybind11::arg("c"));
		cl.def("clearAllConstraintsByType_Unary", (void (mrpt::graphs::ScalarFactorGraph::*)()) &mrpt::graphs::ScalarFactorGraph::clearAllConstraintsByType_Unary, "C++: mrpt::graphs::ScalarFactorGraph::clearAllConstraintsByType_Unary() --> void");
		cl.def("clearAllConstraintsByType_Binary", (void (mrpt::graphs::ScalarFactorGraph::*)()) &mrpt::graphs::ScalarFactorGraph::clearAllConstraintsByType_Binary, "C++: mrpt::graphs::ScalarFactorGraph::clearAllConstraintsByType_Binary() --> void");
		cl.def("updateEstimation", [](mrpt::graphs::ScalarFactorGraph &o, class mrpt::math::CVectorDynamic<double> & a0) -> void { return o.updateEstimation(a0); }, "", pybind11::arg("solved_x_inc"));
		cl.def("updateEstimation", (void (mrpt::graphs::ScalarFactorGraph::*)(class mrpt::math::CVectorDynamic<double> &, class mrpt::math::CVectorDynamic<double> *)) &mrpt::graphs::ScalarFactorGraph::updateEstimation, "C++: mrpt::graphs::ScalarFactorGraph::updateEstimation(class mrpt::math::CVectorDynamic<double> &, class mrpt::math::CVectorDynamic<double> *) --> void", pybind11::arg("solved_x_inc"), pybind11::arg("solved_variances"));
		cl.def("isProfilerEnabled", (bool (mrpt::graphs::ScalarFactorGraph::*)() const) &mrpt::graphs::ScalarFactorGraph::isProfilerEnabled, "C++: mrpt::graphs::ScalarFactorGraph::isProfilerEnabled() const --> bool");
		cl.def("enableProfiler", [](mrpt::graphs::ScalarFactorGraph &o) -> void { return o.enableProfiler(); }, "");
		cl.def("enableProfiler", (void (mrpt::graphs::ScalarFactorGraph::*)(bool)) &mrpt::graphs::ScalarFactorGraph::enableProfiler, "C++: mrpt::graphs::ScalarFactorGraph::enableProfiler(bool) --> void", pybind11::arg("enable"));
		cl.def("assign", (class mrpt::graphs::ScalarFactorGraph & (mrpt::graphs::ScalarFactorGraph::*)(const class mrpt::graphs::ScalarFactorGraph &)) &mrpt::graphs::ScalarFactorGraph::operator=, "C++: mrpt::graphs::ScalarFactorGraph::operator=(const class mrpt::graphs::ScalarFactorGraph &) --> class mrpt::graphs::ScalarFactorGraph &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::graphs::ScalarFactorGraph::FactorBase file:mrpt/graphs/ScalarFactorGraph.h line:46
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::graphs::ScalarFactorGraph::FactorBase, std::shared_ptr<mrpt::graphs::ScalarFactorGraph::FactorBase>, PyCallBack_mrpt_graphs_ScalarFactorGraph_FactorBase> cl(enclosing_class, "FactorBase", "");
			cl.def(pybind11::init<PyCallBack_mrpt_graphs_ScalarFactorGraph_FactorBase const &>());
			cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_graphs_ScalarFactorGraph_FactorBase(); } ) );
			cl.def("evaluateResidual", (double (mrpt::graphs::ScalarFactorGraph::FactorBase::*)() const) &mrpt::graphs::ScalarFactorGraph::FactorBase::evaluateResidual, "Return the residual/error of this observation. \n\nC++: mrpt::graphs::ScalarFactorGraph::FactorBase::evaluateResidual() const --> double");
			cl.def("getInformation", (double (mrpt::graphs::ScalarFactorGraph::FactorBase::*)() const) &mrpt::graphs::ScalarFactorGraph::FactorBase::getInformation, "Return the inverse of the variance of this constraint \n\nC++: mrpt::graphs::ScalarFactorGraph::FactorBase::getInformation() const --> double");
			cl.def("assign", (struct mrpt::graphs::ScalarFactorGraph::FactorBase & (mrpt::graphs::ScalarFactorGraph::FactorBase::*)(const struct mrpt::graphs::ScalarFactorGraph::FactorBase &)) &mrpt::graphs::ScalarFactorGraph::FactorBase::operator=, "C++: mrpt::graphs::ScalarFactorGraph::FactorBase::operator=(const struct mrpt::graphs::ScalarFactorGraph::FactorBase &) --> struct mrpt::graphs::ScalarFactorGraph::FactorBase &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

		{ // mrpt::graphs::ScalarFactorGraph::UnaryFactorVirtualBase file:mrpt/graphs/ScalarFactorGraph.h line:56
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::graphs::ScalarFactorGraph::UnaryFactorVirtualBase, std::shared_ptr<mrpt::graphs::ScalarFactorGraph::UnaryFactorVirtualBase>, PyCallBack_mrpt_graphs_ScalarFactorGraph_UnaryFactorVirtualBase, mrpt::graphs::ScalarFactorGraph::FactorBase> cl(enclosing_class, "UnaryFactorVirtualBase", "Simple, scalar (1-dim) constraint (edge) for a GMRF ");
			cl.def(pybind11::init<PyCallBack_mrpt_graphs_ScalarFactorGraph_UnaryFactorVirtualBase const &>());
			cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_graphs_ScalarFactorGraph_UnaryFactorVirtualBase(); } ) );
			cl.def_readwrite("node_id", &mrpt::graphs::ScalarFactorGraph::UnaryFactorVirtualBase::node_id);
			cl.def("evalJacobian", (void (mrpt::graphs::ScalarFactorGraph::UnaryFactorVirtualBase::*)(double &) const) &mrpt::graphs::ScalarFactorGraph::UnaryFactorVirtualBase::evalJacobian, "Returns the derivative of the residual wrt the node value \n\nC++: mrpt::graphs::ScalarFactorGraph::UnaryFactorVirtualBase::evalJacobian(double &) const --> void", pybind11::arg("dr_dx"));
			cl.def("assign", (struct mrpt::graphs::ScalarFactorGraph::UnaryFactorVirtualBase & (mrpt::graphs::ScalarFactorGraph::UnaryFactorVirtualBase::*)(const struct mrpt::graphs::ScalarFactorGraph::UnaryFactorVirtualBase &)) &mrpt::graphs::ScalarFactorGraph::UnaryFactorVirtualBase::operator=, "C++: mrpt::graphs::ScalarFactorGraph::UnaryFactorVirtualBase::operator=(const struct mrpt::graphs::ScalarFactorGraph::UnaryFactorVirtualBase &) --> struct mrpt::graphs::ScalarFactorGraph::UnaryFactorVirtualBase &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

		{ // mrpt::graphs::ScalarFactorGraph::BinaryFactorVirtualBase file:mrpt/graphs/ScalarFactorGraph.h line:64
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::graphs::ScalarFactorGraph::BinaryFactorVirtualBase, std::shared_ptr<mrpt::graphs::ScalarFactorGraph::BinaryFactorVirtualBase>, PyCallBack_mrpt_graphs_ScalarFactorGraph_BinaryFactorVirtualBase, mrpt::graphs::ScalarFactorGraph::FactorBase> cl(enclosing_class, "BinaryFactorVirtualBase", "Simple, scalar (1-dim) constraint (edge) for a GMRF ");
			cl.def(pybind11::init<PyCallBack_mrpt_graphs_ScalarFactorGraph_BinaryFactorVirtualBase const &>());
			cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_graphs_ScalarFactorGraph_BinaryFactorVirtualBase(); } ) );
			cl.def_readwrite("node_id_i", &mrpt::graphs::ScalarFactorGraph::BinaryFactorVirtualBase::node_id_i);
			cl.def_readwrite("node_id_j", &mrpt::graphs::ScalarFactorGraph::BinaryFactorVirtualBase::node_id_j);
			cl.def("evalJacobian", (void (mrpt::graphs::ScalarFactorGraph::BinaryFactorVirtualBase::*)(double &, double &) const) &mrpt::graphs::ScalarFactorGraph::BinaryFactorVirtualBase::evalJacobian, "Returns the derivative of the residual wrt the node values \n\nC++: mrpt::graphs::ScalarFactorGraph::BinaryFactorVirtualBase::evalJacobian(double &, double &) const --> void", pybind11::arg("dr_dxi"), pybind11::arg("dr_dxj"));
			cl.def("assign", (struct mrpt::graphs::ScalarFactorGraph::BinaryFactorVirtualBase & (mrpt::graphs::ScalarFactorGraph::BinaryFactorVirtualBase::*)(const struct mrpt::graphs::ScalarFactorGraph::BinaryFactorVirtualBase &)) &mrpt::graphs::ScalarFactorGraph::BinaryFactorVirtualBase::operator=, "C++: mrpt::graphs::ScalarFactorGraph::BinaryFactorVirtualBase::operator=(const struct mrpt::graphs::ScalarFactorGraph::BinaryFactorVirtualBase &) --> struct mrpt::graphs::ScalarFactorGraph::BinaryFactorVirtualBase &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
}
