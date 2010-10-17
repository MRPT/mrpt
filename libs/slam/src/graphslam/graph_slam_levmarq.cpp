/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/slam.h>  // Precompiled header

#include <mrpt/poses/CNetworkOfPoses.h>
#include <mrpt/slam/graph_slam.h>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::slam;
using namespace mrpt::graphslam;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace std;

/*
   The method implemented in this file is based on this work:
     - "Efficient Sparse Pose Adjustment for 2D Mapping", Kurt Konolige et al., 2010.

   But modified in:
     - It's generalized for not only 2D but 2D and 3D poses.
	 - It uses on-manifold optimization, specially different for the 3D case.
*/

// Compute, at once, jacobians and the error vectors for each constraint in "lstObservationData", returns the overall squared error.
template <class ET, class MI>
double computeJacobiansAndErrors(
	const typename graphslam_traits<ET,MI>::graph_t &graph,
	const vector<typename graphslam_traits<ET,MI>::observation_info_t>  &lstObservationData,
	map<TPairNodeIDs,typename graphslam_traits<ET,MI>::TPairJacobs>   &lstJacobians,
	vector<typename graphslam_traits<ET,MI>::Array_O> &errs
	)
{
	typedef graphslam_traits<ET,MI> gst;

	lstJacobians.clear();
	errs.clear();

	const size_t nObservations = lstObservationData.size();

	for (size_t i=0;i<nObservations;i++)
	{
		const typename gst::observation_info_t & obs = lstObservationData[i];
		typename gst::edge_const_iterator it = obs.edge;
		const typename gst::graph_t::contraint_t::type_value* EDGE_POSE = obs.edge_mean;
		typename gst::graph_t::contraint_t::type_value* P1 = obs.P1;
		typename gst::graph_t::contraint_t::type_value* P2 = obs.P2;

		const TPairNodeIDs                   &ids  = it->first;
		//const typename gst::graph_t::edge_t  &edge = it->second;

		// Compute the residual pose error of these pair of nodes + its constraint,
		//  that is: P1DP2inv = P1 * EDGE * inv(P2)
		typename gst::graph_t::contraint_t::type_value P1DP2inv(UNINITIALIZED_POSE);
		{
			typename gst::graph_t::contraint_t::type_value P1D(UNINITIALIZED_POSE);
			P1D.composeFrom(*P1,*EDGE_POSE);
			const typename gst::graph_t::contraint_t::type_value P2inv = -(*P2); // Pose inverse (NOT just switching signs!)
			P1DP2inv.composeFrom(P1D,P2inv);
		}

		// Add to vector of errors:
		{
			errs.resize(errs.size()+1);
			gst::SE_TYPE::pseudo_ln(P1DP2inv, errs[errs.size()-1]);
		}

		// Compute the jacobians:
		std::pair<TPairNodeIDs,typename gst::TPairJacobs> newMapEntry;
		newMapEntry.first = ids;
		gst::SE_TYPE::jacobian_dP1DP2inv_depsilon(P1DP2inv, &newMapEntry.second.first,&newMapEntry.second.second);

		// And insert into map of jacobians:
		lstJacobians.insert(lstJacobians.end(),newMapEntry );
	}

	// return overall square error:
	return std::accumulate( errs.begin(), errs.end(),0.0, mrpt::math::squareNorm_accum<typename gst::Array_O> );
}


/*---------------------------------------------------------------
			optimize_graph_spa_levmarq
   See .h for docs.
  ---------------------------------------------------------------*/
template <class EDGE_TYPE, class MAPS_IMPLEMENTATION>
void mrpt::graphslam::optimize_graph_spa_levmarq(
	CNetworkOfPoses<EDGE_TYPE,MAPS_IMPLEMENTATION>  &graph,
	const set<TNodeID>         * in_nodes_to_optimize,
	const mrpt::utils::TParametersDouble  &extra_params
	)
{
	MRPT_START
#define VERBOSE_PREFIX "[optimize_graph_spa_levmarq] "

	// Typedefs to make life easier:
	typedef graphslam_traits<EDGE_TYPE,MAPS_IMPLEMENTATION> gst;

	typename gst::Array_O array_O_zeros; array_O_zeros.fill(0); // Auxiliary var with all zeros

	// The size of things here (because size matters...)
	static const unsigned int DIMS_POSE = gst::SE_TYPE::VECTOR_SIZE;

	// Read extra params:
	const bool verbose            = 0!=extra_params.getWithDefaultVal("verbose",0);
	const size_t max_iters        = extra_params.getWithDefaultVal("max_iterations",50);
	const bool   enable_profiler  = 0!=extra_params.getWithDefaultVal("profiler",0);
	// LM params:
	const double initial_lambda = extra_params.getWithDefaultVal("initial_lambda", 0);  // <=0: means auto guess
	const double tau = extra_params.getWithDefaultVal("tau", 1e-3);
	const double e1 = extra_params.getWithDefaultVal("e1",1e-8);
	const double e2 = extra_params.getWithDefaultVal("e2",1e-8);

	const double MIN_LAMBDA = extra_params.getWithDefaultVal("min_lambda",1e-290);

	mrpt::utils::CTimeLogger  profiler(enable_profiler);
	profiler.enter("optimize_graph_spa_levmarq (entire)");

	// Make list of node IDs to optimize, since the user may want only a subset of them to be optimized:
	profiler.enter("optimize_graph_spa_levmarq.list_IDs"); // ---------------\  .
	const set<TNodeID> * nodes_to_optimize;
	set<TNodeID> nodes_to_optimize_auxlist;  // Used only if in_nodes_to_optimize==NULL
	if (in_nodes_to_optimize)
	{
		nodes_to_optimize = in_nodes_to_optimize;
	}
	else
	{
		// We have to make the list of IDs:
		for (typename gst::graph_t::global_poses_t::const_iterator it=graph.nodes.begin();it!=graph.nodes.end();++it)
			if (it->first!=graph.root) // Root node is fixed.
				nodes_to_optimize_auxlist.insert(nodes_to_optimize_auxlist.end(), it->first ); // Provide the "first guess" insert position for efficiency
		nodes_to_optimize = &nodes_to_optimize_auxlist;
	}
	profiler.leave("optimize_graph_spa_levmarq.list_IDs"); // ---------------/

	// Number of nodes to optimize, or free variables:
	const size_t nFreeNodes = nodes_to_optimize->size();
	ASSERT_ABOVE_(nFreeNodes,0)

	if (verbose)
	{
		cout << VERBOSE_PREFIX << nFreeNodes << " nodes to optimize: ";
		if (nFreeNodes<14)
		{
			ostream_iterator<TNodeID> out_it (cout,", ");
			std::copy(nodes_to_optimize->begin(), nodes_to_optimize->end(), out_it );
		}
		cout << endl;
	}

	// The list of those edges that will be considered in this optimization (many may be discarded
	//  if we are optimizing just a subset of all the nodes):
	typedef typename gst::observation_info_t observation_info_t;
	vector<observation_info_t>  lstObservationData;

	// Note: We'll need those Jacobians{i->j} where at least one "i" or "j"
	//        is a free variable (i.e. it's in nodes_to_optimize)
	// Now, build the list of all relevent "observations":
	for (typename gst::edge_const_iterator it=graph.edges.begin();it!=graph.edges.end();++it)
	{
		const TPairNodeIDs                   &ids  = it->first;
		const typename gst::graph_t::edge_t  &edge = it->second;

		if (nodes_to_optimize->find(ids.first)==nodes_to_optimize->end() &&
			nodes_to_optimize->find(ids.second)==nodes_to_optimize->end())
				continue; // Skip this edge, none of the IDs are free variables.

		// get the current global poses of both nodes in this constraint:
		typename gst::graph_t::global_poses_t::iterator itP1 = graph.nodes.find(ids.first);
		typename gst::graph_t::global_poses_t::iterator itP2 = graph.nodes.find(ids.second);
		ASSERTDEBMSG_(itP1!=graph.nodes.end(),"Node1 in an edge does not have a global pose in 'graph.nodes'.")
		ASSERTDEBMSG_(itP2!=graph.nodes.end(),"Node2 in an edge does not have a global pose in 'graph.nodes'.")

		const typename gst::graph_t::contraint_t::type_value &EDGE_POSE  = mrpt::poses::getPoseMean<typename gst::graph_t::contraint_t,typename gst::graph_t::contraint_t::type_value>(edge);

		// Add all the data to the list of relevant observations:
		typename gst::observation_info_t new_entry;
		new_entry.edge = it;
		new_entry.edge_mean = &EDGE_POSE;
		new_entry.P1 = &itP1->second;
		new_entry.P2 = &itP2->second;

		lstObservationData.push_back(new_entry);
	}

	// The number of constraints, or observations actually implied in this problem:
	const size_t nObservations = lstObservationData.size();
	ASSERT_ABOVE_(nObservations,0)

	// Cholesky object, as a pointer to reuse it between iterations:
	std::auto_ptr<CSparseMatrix::CholeskyDecomp>  ptrCh;

	// The list of Jacobians: for each constraint i->j,
	//  we need the pair of Jacobians: { dh(xi,xj)_dxi, dh(xi,xj)_dxj },
	//  which are "first" and "second" in each pair.
	// Index of the map are the node IDs {i,j} for each contraint.
	map<TPairNodeIDs,typename gst::TPairJacobs>   lstJacobians;
	// The vector of errors: err_k = SE(2/3)::pseudo_Ln( P_i * EDGE_ij * inv(P_j) )
	vector<typename gst::Array_O> errs; // Separated vectors for each edge. i \in [0,nObservations-1], in same order than lstObservationData

	// ===================================
	// Compute Jacobians & errors
	// ===================================
	profiler.enter("optimize_graph_spa_levmarq.Jacobians&err");// ------------------------------\  .
	double total_sqr_err = computeJacobiansAndErrors<EDGE_TYPE,MAPS_IMPLEMENTATION>(
		graph, lstObservationData,
		lstJacobians, errs);
	profiler.leave("optimize_graph_spa_levmarq.Jacobians&err");  // ------------------------------/


	// Only once (since this will be static along iterations), build a quick look-up table with the
	//  indices of the free nodes associated to the (first_id,second_id) of each Jacobian pair:
	// -----------------------------------------------------------------------------------------------
	vector<pair<size_t,size_t> >  observationIndex_to_relatedFreeNodeIndex; // "relatedFreeNodeIndex" means into [0,nFreeNodes-1], or "-1" if that node is fixed, as ordered in "nodes_to_optimize"
	observationIndex_to_relatedFreeNodeIndex.reserve(nObservations);
	ASSERTDEB_(lstJacobians.size()==nObservations)
	for (typename map<TPairNodeIDs,typename gst::TPairJacobs>::const_iterator itJ=lstJacobians.begin();itJ!=lstJacobians.end();++itJ)
	{
		const TNodeID id1 = itJ->first.first;
		const TNodeID id2 = itJ->first.second;
		observationIndex_to_relatedFreeNodeIndex.push_back(
			std::make_pair(
				mrpt::utils::find_in_vector(id1,*nodes_to_optimize),
				mrpt::utils::find_in_vector(id2,*nodes_to_optimize) ));
	}

	// other important vars for the main loop:
	vector_double grad(nFreeNodes*DIMS_POSE);
	vector<map<TNodeID,typename gst::matrix_VxV_t> >  H_map(nFreeNodes);

	double	lambda = initial_lambda; // Will be actually set on first iteration.
	double  rho = 0.5; // value such as lambda is not modified in the first pass
	double	v = 1; // was 2, changed since it's modified in the first pass.
	bool    have_to_recompute_H_and_grad = true;

	// -----------------------------------------------------------
	// Main iterative loop of Levenberg-Marquardt algorithm
	//  For notation and overall algorithm overview, see:
	//  http://www.mrpt.org/Levenberg%E2%80%93Marquardt_algorithm
	// -----------------------------------------------------------

	for (size_t iter=0;iter<max_iters;++iter)
	{
		if (have_to_recompute_H_and_grad)  // This will be false only when the delta leads to a worst solution and only a change in lambda is needed.
		{
			have_to_recompute_H_and_grad = false;

			// ========================================================================
			// Compute the gradient: grad = J^t * errs
			// ========================================================================
			//  "grad" can be seen as composed of N independent arrays, each one being:
			//   grad_i = \sum_k J^t_{k->i} errs_k
			// that is: g_i is the "dot-product" of the i'th (transposed) block-column of J and the vector of errors "errs"
			profiler.enter("optimize_graph_spa_levmarq.grad"); // ------------------------------\  .
			vector<typename gst::Array_O> grad_parts(nFreeNodes, array_O_zeros);

			// "lstJacobians" is sorted in the same order than "lstObservationData":
			ASSERT_EQUAL_(lstJacobians.size(),lstObservationData.size())

			{
				size_t idx_obs;
				typename map<TPairNodeIDs,typename gst::TPairJacobs>::const_iterator itJ;

				for (idx_obs=0, itJ=lstJacobians.begin();
					itJ!=lstJacobians.end();
					++itJ,++idx_obs)
				{
					ASSERTDEB_(itJ->first==lstObservationData[idx_obs].edge->first) // make sure they're in the expected order!

					//  grad[k] += J^t_{i->k} * errs_i
					//    k: [0,nFreeNodes-1]     <-- IDs.first & IDs.second
					//    i: [0,nObservations-1]  <--- idx_obs

					// Get the corresponding indices in the vector of "free variables" being optimized:
					const size_t idx1 = observationIndex_to_relatedFreeNodeIndex[idx_obs].first;
					const size_t idx2 = observationIndex_to_relatedFreeNodeIndex[idx_obs].second;

					if (idx1!=string::npos)
						itJ->second.first.multiply_Atb(
							errs[idx_obs] /*in*/,
							grad_parts[idx1] /*out*/,
							true /* accumulate in output */ );

					if (idx2!=string::npos)
						itJ->second.second.multiply_Atb(
							errs[idx_obs] /*in*/,
							grad_parts[idx2] /*out*/,
							true /* accumulate in output */ );
				}
			}

			// build the gradient as a single vector:
			::memcpy(&grad[0],&grad_parts[0], nFreeNodes*DIMS_POSE*sizeof(grad[0]));  // Ohh yeahh!
			profiler.leave("optimize_graph_spa_levmarq.grad"); // ------------------------------/

			// End condition #1
			const double grad_norm_inf = math::norm_inf(grad); // inf-norm (abs. maximum value) of the gradient
			if (grad_norm_inf<=e1)
			{
				// Change is too small
				if (verbose) printf(VERBOSE_PREFIX "End condition #1: math::norm_inf(g)<=e1 :%f\n",grad_norm_inf);
				break;
			}


			profiler.enter("optimize_graph_spa_levmarq.sp_H:build map"); // ------------------------------\  .
			// ======================================================================
			// Build sparse representation of the upper triangular part of
			//  the Hessian matrix H = J^t * J
			//
			// Sparse memory structure is a vector of maps, such as:
			//  - H_map[i]: corresponds to the i'th column of H.
			//              Here "i" corresponds to [0,N-1] indices of appearance in the map "*nodes_to_optimize".
			//  - H_map[i][j] is the entry for the j'th row, with "j" also in the range [0,N-1] as ordered in "*nodes_to_optimize".
			// ======================================================================
			{
				size_t idxObs;
				typename map<TPairNodeIDs,typename gst::TPairJacobs>::const_iterator itJacobPair;

				for (idxObs=0, itJacobPair=lstJacobians.begin();
				     idxObs<nObservations;
				     ++itJacobPair,++idxObs)
				{
					// We sort IDs such as "i" < "j" and we can build just the upper triangular part of the Hessian.
					const bool edge_straight = itJacobPair->first.first < itJacobPair->first.second;

					// Indices in the "H_map" vector:
					const size_t idx_i = edge_straight ? observationIndex_to_relatedFreeNodeIndex[idxObs].first  : observationIndex_to_relatedFreeNodeIndex[idxObs].second;
					const size_t idx_j = edge_straight ? observationIndex_to_relatedFreeNodeIndex[idxObs].second : observationIndex_to_relatedFreeNodeIndex[idxObs].first;

					const bool is_i_free_node = idx_i!=string::npos;
					const bool is_j_free_node = idx_j!=string::npos;

					// Take references to both Jacobians (wrt pose "i" and pose "j"), taking into account the possible
					// switch in their order:

					const typename gst::matrix_VxV_t &J1 = edge_straight ? itJacobPair->second.first : itJacobPair->second.second;
					const typename gst::matrix_VxV_t &J2 = edge_straight ? itJacobPair->second.second : itJacobPair->second.first;

					// Is "i" a free (to be optimized) node? -> Ji^t * Inf *  Ji
					if (is_i_free_node)
					{
						typename gst::matrix_VxV_t JtJ(UNINITIALIZED_MATRIX);
						MRPT_TODO("Take into account the case with information matrix")
						JtJ.multiply_AtA(J1);
						H_map[idx_i][idx_i] += JtJ;
					}
					// Is "j" a free (to be optimized) node? -> Jj^t * Inf *  Jj
					if (is_j_free_node)
					{
						typename gst::matrix_VxV_t JtJ(UNINITIALIZED_MATRIX);
						JtJ.multiply_AtA(J2);
						H_map[idx_j][idx_j] += JtJ;
					}
					// Are both "i" and "j" free nodes? -> Ji^t * Inf *  Jj
					if (is_i_free_node && is_j_free_node)
					{
						typename gst::matrix_VxV_t JtJ(UNINITIALIZED_MATRIX);
						JtJ.multiply_AtB(J1,J2);
						H_map[idx_j][idx_i] += JtJ;
					}
				}
			}
			profiler.leave("optimize_graph_spa_levmarq.sp_H:build map");  // ------------------------------/

			// Just in the first iteration, we need to calculate an estimate for the first value of "lamdba":
			if (lambda<=0 && iter==0)
			{
				profiler.enter("optimize_graph_spa_levmarq.lambda_init");  // ---\  .
				double H_diagonal_max = 0;
				for (size_t i=0;i<nFreeNodes;i++)
					for (typename map<TNodeID,typename gst::matrix_VxV_t>::const_iterator it=H_map[i].begin();it!=H_map[i].end();++it)
					{
						const size_t j = it->first; // entry submatrix is for (i,j).
						if (i==j)
						{
							for (size_t k=0;k<DIMS_POSE;k++)
								mrpt::utils::keep_max(H_diagonal_max, it->second.get_unsafe(k,k) );
						}
					}
				lambda = tau * H_diagonal_max;

				profiler.leave("optimize_graph_spa_levmarq.lambda_init");  // ---/
			}
#if 0
			{ CMatrixDouble H; sp_H.get_dense(H); H.saveToTextFile("d:\\H.txt"); }
#endif


			// After recomputing H and the grad, we update lambda:
			lambda *= std::max(0.333, 1-pow(2*rho-1,3.0) );
			v = 2;

			// JL: Additional ending condition: extremely small lambda:
			if (lambda<MIN_LAMBDA)
			{
				if (verbose ) cout << VERBOSE_PREFIX "End condition #3: lambda < " << MIN_LAMBDA << "\n";
				break;
			}

		} // end "have_to_recompute_H_and_grad"

		if (verbose )
			cout << VERBOSE_PREFIX "Iter: " << iter << " ,total sqr. err: " << total_sqr_err  << ", avrg. err per edge: " << std::sqrt(total_sqr_err/nObservations) << " lambda: " << lambda << endl;

		profiler.enter("optimize_graph_spa_levmarq.sp_H:build"); // ------------------------------\  .
		// Now, build the actual sparse matrix H:
		// Note: we only need to fill out the upper diagonal part, since Cholesky will later on ignore the other part.
		CSparseMatrix  sp_H(nFreeNodes*DIMS_POSE,nFreeNodes*DIMS_POSE);
		for (size_t i=0;i<nFreeNodes;i++)
		{
			const size_t i_offset = i*DIMS_POSE;

			for (typename map<TNodeID,typename gst::matrix_VxV_t>::const_iterator it=H_map[i].begin();it!=H_map[i].end();++it)
			{
				const size_t j = it->first; // entry submatrix is for (i,j).
				const size_t j_offset = j*DIMS_POSE;

				// For i==j (diagonal blocks), it's different, since we only need to insert their
				// upper-diagonal half and also we have to add the lambda*I to the diagonal from the Lev-Marq. algorithm:
				if (i==j)
				{
					for (size_t r=0;r<DIMS_POSE;r++)
					{
						// c=r: add lambda from LM
						sp_H.insert_entry_fast(j_offset+r,i_offset+r, it->second.get_unsafe(r,r) + lambda );
						// c>r:
						for (size_t c=r+1;c<DIMS_POSE;c++)
							sp_H.insert_entry_fast(j_offset+r,i_offset+c, it->second.get_unsafe(r,c));
					}
				}
				else
				{
					sp_H.insert_submatrix(j_offset,i_offset, it->second);
				}
			}
		} // end for each free node, build sp_H

		sp_H.compressFromTriplet();
		profiler.leave("optimize_graph_spa_levmarq.sp_H:build"); // ------------------------------/

		// Use the cparse Cholesky decomposition to efficiently solve:
		//   (H+\lambda*I) \delta = -J^t * (f(x)-z)
		//          A         x   =  b         -->       x = A^{-1} * b
		//
		vector_double  delta; // The (minus) increment to be added to the current solution in this step
		try
		{
			profiler.enter("optimize_graph_spa_levmarq.sp_H:chol");
			if (!ptrCh.get())
					ptrCh = std::auto_ptr<CSparseMatrix::CholeskyDecomp>(new CSparseMatrix::CholeskyDecomp(sp_H) );
			else ptrCh.get()->update(sp_H);
			profiler.leave("optimize_graph_spa_levmarq.sp_H:chol");

			profiler.enter("optimize_graph_spa_levmarq.sp_H:backsub");
			ptrCh->backsub(grad,delta);
			profiler.leave("optimize_graph_spa_levmarq.sp_H:backsub");
		}
		catch (CExceptionNotDefPos &)
		{
			// not positive definite so increase mu and try again
			if (verbose ) cout << VERBOSE_PREFIX "Got non-definite positive matrix, retrying with a larger lambda...\n";
			lambda *= v;
			v*= 2;
			if (lambda>1e9)
			{	// enough!
				break;
			}
			continue; // try again with this params
		}

		// Compute norm of the increment vector:
		profiler.enter("optimize_graph_spa_levmarq.delta_norm");
		const double delta_norm = math::norm(delta);
		profiler.leave("optimize_graph_spa_levmarq.delta_norm");

		// Compute norm of the current solution vector:
		profiler.enter("optimize_graph_spa_levmarq.x_norm" );
		double x_norm = 0;
		{
			for (set<TNodeID>::const_iterator it=nodes_to_optimize->begin();it!=nodes_to_optimize->end();++it)
			{
				typename gst::graph_t::global_poses_t::const_iterator itP = graph.nodes.find(*it);
				const typename gst::graph_t::contraint_t::type_value &P = itP->second;
				for (size_t i=0;i<DIMS_POSE;i++)
					x_norm+=square(P[i]);
			}
			x_norm=std::sqrt(x_norm);
		}
		profiler.leave("optimize_graph_spa_levmarq.x_norm" );

		// Test end condition #2:
		if (delta_norm<e2*(x_norm+e2))
		{
			// The change is too small: we're done here...
			if (verbose ) cout << VERBOSE_PREFIX << format("End condition #2: %e < %e\n", delta_norm, e2*(x_norm+e2) );
			break;
		}
		else
		{
			// =====================================================================================
			// Accept this delta? Try it and look at the increase/decrease of the error:
			//  new_x = old_x [+] (-delta)    , with [+] being the "manifold exp()+add" operation.
			// =====================================================================================
			typename gst::graph_t::global_poses_t  old_poses_backup;

			{
				ASSERTDEB_(delta.size()==nodes_to_optimize->size()*DIMS_POSE)
				const double *delta_ptr = &delta[0];
				for (set<TNodeID>::const_iterator it=nodes_to_optimize->begin();it!=nodes_to_optimize->end();++it)
				{
					// exp_delta_i = Exp_SE( delta_i )
					typename gst::graph_t::contraint_t::type_value exp_delta_pose(UNINITIALIZED_POSE);
					typename gst::Array_O exp_delta;
					for (size_t i=0;i<DIMS_POSE;i++)
						exp_delta[i]= - *delta_ptr++;  // The "-" sign is for the missing "-" carried all this time from above
					gst::SE_TYPE::exp(exp_delta,exp_delta_pose);

					// new_x_i =  exp_delta_i (+) old_x_i
					typename gst::graph_t::global_poses_t::iterator it_old_value = graph.nodes.find(*it);
					old_poses_backup[*it] = it_old_value->second; // back up the old pose as a copy
					it_old_value->second.composeFrom(exp_delta_pose, it_old_value->second);
				}
			}

			// =============================================================
			// Compute Jacobians & errors with the new "graph.nodes" info:
			// =============================================================
			map<TPairNodeIDs,typename gst::TPairJacobs>   new_lstJacobians;
			vector<typename gst::Array_O>                 new_errs;

			profiler.enter("optimize_graph_spa_levmarq.Jacobians&err");// ------------------------------\  .
			double new_total_sqr_err = computeJacobiansAndErrors<EDGE_TYPE,MAPS_IMPLEMENTATION>(
				graph, lstObservationData,
				new_lstJacobians, new_errs);
			profiler.leave("optimize_graph_spa_levmarq.Jacobians&err");// ------------------------------/

			// Now, to decide whether to accept the change, compute the quantity:
			//  l =  (old_error - new_error) / denom;
			//  denom = delta^t * ( \lambda * delta - grad )
			double denom;
			{
				vector_double aux = delta;
				aux*=lambda;
				aux+=grad;  // -= (-grad), read the why of the extra "-" sign above.
				denom = mrpt::math::dotProduct<vector_double,vector_double>(delta,aux);
			}
			rho = (total_sqr_err - new_total_sqr_err) / denom;

			if (rho>0)
			{
				// Accept the new point:
				std::swap(new_lstJacobians, lstJacobians);
				std::swap(new_errs, errs);
				std::swap(new_total_sqr_err, total_sqr_err);

				// Instruct to recompute H and grad from the new Jacobians.
				have_to_recompute_H_and_grad = true;
			}
			else
			{
				// Nope...
				// We have to revert the "graph.nodes" to "old_poses_backup"
				for (typename gst::graph_t::global_poses_t::const_iterator it=old_poses_backup.begin();it!=old_poses_backup.end();++it)
					graph.nodes[it->first] = it->second;

				if (verbose ) cout << VERBOSE_PREFIX "Got larger error=" << new_total_sqr_err << ", retrying with a larger lambda...\n";
				// Change params and try again:
				lambda *= v;
				v*= 2;
			}

		} // end else end condition #2

	} // end for each iter


	profiler.leave("optimize_graph_spa_levmarq (entire)");

	MRPT_END
}

// Explicit instantations:
template SLAM_IMPEXP void mrpt::graphslam::optimize_graph_spa_levmarq<CPose2D,map_traits_stdmap>(CNetworkOfPoses<CPose2D,map_traits_stdmap >&,const set<TNodeID>*, const mrpt::utils::TParametersDouble  &extra_params);
template SLAM_IMPEXP void mrpt::graphslam::optimize_graph_spa_levmarq<CPose3D,map_traits_stdmap>(CNetworkOfPoses<CPose3D,map_traits_stdmap >&,const set<TNodeID>*, const mrpt::utils::TParametersDouble  &extra_params);



