/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/utils/core_defs.h>
#include <mrpt/utils/types_math.h>
#include <mrpt/utils/COutputLogger.h>
#include <mrpt/utils/CTimeLogger.h>
#include <deque>

#if EIGEN_VERSION_AT_LEAST(3,1,0) // Requires Eigen>=3.1
#	include <Eigen/SparseCore>
#endif

#include <mrpt/graphs/link_pragmas.h>

namespace mrpt
{
namespace graphs
{
	/** Sparse solver for GMRF (Gaussian Markov Random Fields) graphical models.
	 *  The design of this class is optimized for large problems (e.g. >1e3 nodes, >1e4 constrainst) 
	 *  by leaving to the user/caller the responsibility of allocating all "nodes" and constraints.
	 *  This class can be seen as an intermediary solution between current methods in mrpt::graphslam and the well-known G2O library:
	 *
	 *  Assumptions/limitations:
	 *   - Linear error functions (for now).
	 *   - Scalar (1-dim) error functions.
	 *   - Solver: Eigen SparseQR.
	 *
	 *  Usage:
	 *   - Call initialize() to set the number of nodes.
	 *   - Call addConstraints() to insert constraints. This may be called more than once.
	 *   - Call updateEstimation() to run one step of the linear solver, using a SparseQR solver.
	 *
	 * \ingroup mrpt_graph_grp
	 * \note [New in MRPT 1.5.0] Requires Eigen>=3.1
	 */
	class GRAPHS_IMPEXP GaussianMarkovRandomField : 
		public mrpt::utils::COutputLogger
	{
	public:
		GaussianMarkovRandomField();

		/** Simple, scalar (1-dim) constraint (edge) for a GMRF */
		struct GRAPHS_IMPEXP UnaryFactorVirtualBase
		{
			size_t node_id;
			virtual double evaluateResidual() const = 0; //!< Return the residual/error of this observation.
			virtual double getInformation() const = 0; //!< Return the inverse of the variance of this constraint
			virtual void evalJacobian(double &dr_dx) const = 0; //!< Returns the derivative of the residual wrt the node value
		};

		/** Simple, scalar (1-dim) constraint (edge) for a GMRF */
		struct GRAPHS_IMPEXP BinaryFactorVirtualBase
		{
			size_t node_id_i, node_id_j;
			virtual double evaluateResidual() const = 0; //!< Return the residual/error of this observation.
			virtual double getInformation() const = 0; //!< Return the inverse of the variance of this constraint
			virtual void evalJacobian(double &dr_dxi, double &dr_dxj) const = 0; //!< Returns the derivative of the residual wrt the node values
		};

		void clear(); //!< Reset state: remove all constraints and nodes.
		
		/** Initialize the GMRF internal state and copy the prior factors. */
		void initialize(
			const size_t nodeCount            //!< Number of unknown nodes in the MRF graph
		);

		/** Insert constraints into the GMRF problem.
		  * \param listOfConstraints List of user-implemented constraints. 
		  * **A pointer to the passed object is kept**, but memory ownship *REMAINS* being responsability of the caller. This is 
		  * done such that arrays/vectors of constraints can be more efficiently allocated if their type is known at build time.
		  */
		void addConstraint(const UnaryFactorVirtualBase &listOfConstraints);
		void addConstraint(const BinaryFactorVirtualBase &listOfConstraints);

#if EIGEN_VERSION_AT_LEAST(3,1,0) // Requires Eigen>=3.1
		void updateEstimation(
			Eigen::VectorXd & solved_x_inc,                       //!< Output increment of the current estimate. Caller must add this vector to current state vector to obtain the optimal estimation.
			Eigen::SparseMatrix<double> *solved_covariance = NULL //!< If !=NULL, the covariance of the estimate will be stored here.
		);
#endif

	private:
		Eigen::VectorXd m_g;       //!< Gradient vector. The length of this vector implicitly stores the number of nodes in the graph.

		std::deque<const UnaryFactorVirtualBase*>  m_factors_unary;
		std::deque<const BinaryFactorVirtualBase*> m_factors_binary;

		mrpt::utils::CTimeLogger m_timelogger;
		bool m_enable_profiler;

	}; // End of class def.

} // End of namespace
} // End of namespace

