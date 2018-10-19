/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/math/types_math.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/CTimeLogger.h>
#include <deque>

namespace mrpt::graphs
{
/** Sparse solver for GMRF (Gaussian Markov Random Fields) graphical models.
 *  The design of this class is optimized for large problems (e.g. >1e3 nodes,
 * >1e4 constrainst)
 *  by leaving to the user/caller the responsibility of allocating all "nodes"
 * and constraints.
 *  This class can be seen as an intermediary solution between current methods
 * in mrpt::graphslam and the well-known G2O library:
 *
 *  Assumptions/limitations:
 *   - Linear error functions (for now).
 *   - Scalar (1-dim) error functions.
 *   - Gaussian factors.
 *   - Solver: Eigen SparseQR.
 *
 *  Usage:
 *   - Call initialize() to set the number of nodes.
 *   - Call addConstraints() to insert constraints. This may be called more than
 * once.
 *   - Call updateEstimation() to run one step of the linear SparseQR solver.
 *
 * \ingroup mrpt_graph_grp
 * \note [New in MRPT 1.5.0] Requires Eigen>=3.1
 */
class ScalarFactorGraph : public mrpt::system::COutputLogger
{
   public:
	ScalarFactorGraph();

	struct FactorBase
	{
		virtual ~FactorBase();
		/** Return the residual/error of this observation. */
		virtual double evaluateResidual() const = 0;
		/** Return the inverse of the variance of this constraint */
		virtual double getInformation() const = 0;
	};

	/** Simple, scalar (1-dim) constraint (edge) for a GMRF */
	struct UnaryFactorVirtualBase : public FactorBase
	{
		size_t node_id;
		/** Returns the derivative of the residual wrt the node value */
		virtual void evalJacobian(double& dr_dx) const = 0;
	};

	/** Simple, scalar (1-dim) constraint (edge) for a GMRF */
	struct BinaryFactorVirtualBase : public FactorBase
	{
		size_t node_id_i, node_id_j;
		/** Returns the derivative of the residual wrt the node values */
		virtual void evalJacobian(double& dr_dxi, double& dr_dxj) const = 0;
	};

	/** Reset state: remove all constraints and nodes. */
	void clear();

	/** Initialize the GMRF internal state and copy the prior factors. */
	void initialize(
		/** Number of unknown nodes in the MRF graph */
		const size_t nodeCount);

	/** Insert constraints into the GMRF problem.
	 * \param listOfConstraints List of user-implemented constraints.
	 * **A pointer to the passed object is kept**, but memory ownship *REMAINS*
	 * being responsability of the caller. This is
	 * done such that arrays/vectors of constraints can be more efficiently
	 * allocated if their type is known at build time.
	 */
	void addConstraint(const UnaryFactorVirtualBase& listOfConstraints);
	void addConstraint(const BinaryFactorVirtualBase& listOfConstraints);

	/** Removes a constraint. Return true if found and deleted correctly. */
	bool eraseConstraint(const FactorBase& c);

	void clearAllConstraintsByType_Unary() { m_factors_unary.clear(); }
	void clearAllConstraintsByType_Binary() { m_factors_binary.clear(); }
	void updateEstimation(
		/** Output increment of the current estimate. Caller must add this
		   vector to current state vector to obtain the optimal estimation. */
		Eigen::VectorXd& solved_x_inc,
		/** If !=nullptr, the variances of each estimate will be stored here. */
		Eigen::VectorXd* solved_variances = nullptr);

	bool isProfilerEnabled() const { return m_enable_profiler; }
	void enableProfiler(bool enable = true) { m_enable_profiler = enable; }

   private:
	/** number of nodes in the graph */
	size_t m_numNodes;

	std::deque<const UnaryFactorVirtualBase*> m_factors_unary;
	std::deque<const BinaryFactorVirtualBase*> m_factors_binary;

	mrpt::system::CTimeLogger m_timelogger;
	bool m_enable_profiler{false};

};  // End of class def.

}  // namespace mrpt::graphs
