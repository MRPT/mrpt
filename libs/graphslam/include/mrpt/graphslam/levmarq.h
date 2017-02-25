/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef GRAPH_SLAM_LEVMARQ_H
#define GRAPH_SLAM_LEVMARQ_H

#include <mrpt/graphslam/types.h>
#include <mrpt/utils/TParameters.h>
#include <mrpt/utils/stl_containers_utils.h> // find_in_vector()
#include <mrpt/graphslam/levmarq_impl.h> // Aux classes

#include <iterator> // ostream_iterator

namespace mrpt
{
	namespace graphslam
	{
		/** Optimize a graph of pose constraints using the Sparse Pose Adjustment (SPA) sparse representation and a Levenberg-Marquardt optimizer.
		  *  This method works for all types of graphs derived from \a CNetworkOfPoses (see its reference mrpt::graphs::CNetworkOfPoses for the list).
		  *  The input data are all the pose constraints in \a graph (graph.edges), and the gross first estimations of the "global" pose coordinates (in graph.nodes).
		  *
		  *  Note that these first coordinates can be obtained with mrpt::graphs::CNetworkOfPoses::dijkstra_nodes_estimate().
		  *
		  * The method implemented in this file is based on this work:
		  *  - "Efficient Sparse Pose Adjustment for 2D Mapping", Kurt Konolige et al., 2010.
		  * , but generalized for not only 2D but 2D and 3D poses, and using on-manifold optimization.
		  *
		  * \param[in,out] graph The input edges and output poses.
		  * \param[out] out_info Some basic output information on the process.
		  * \param[in] nodes_to_optimize The list of nodes whose global poses are to be optimized. If NULL (default), all the node IDs are optimized (but that marked as \a root in the graph).
		  * \param[in] extra_params Optional parameters, see below.
		  * \param[in] functor_feedback Optional: a pointer to a user function can be set here to be called on each LM loop iteration (eg to refresh the current state and error, refresh a GUI, etc.)
		  *
		  * List of optional parameters by name in "extra_params":
		  *		- "verbose": (default=0) If !=0, produce verbose ouput.
		  *		- "max_iterations": (default=100) Maximum number of Lev-Marq. iterations.
		  *		- "scale_hessian": (default=0.1) Multiplies the Hessian matrix by this scalar (may improve convergence speed).
		  *		- "initial_lambda": (default=0) <=0 means auto guess, otherwise, initial lambda value for the lev-marq algorithm.
		  *		- "tau": (default=1e-3) Initial tau value for the lev-marq algorithm.
		  *		- "e1": (default=1e-6) Lev-marq algorithm iteration stopping criterion #1: |gradient| < e1
		  *		- "e2": (default=1e-6) Lev-marq algorithm iteration stopping criterion #2: |delta_incr| < e2*(x_norm+e2)
		  *
		  * \note The following graph types are supported: mrpt::graphs::CNetworkOfPoses2D, mrpt::graphs::CNetworkOfPoses3D, mrpt::graphs::CNetworkOfPoses2DInf, mrpt::graphs::CNetworkOfPoses3DInf
		  *
		  * \tparam GRAPH_T Normally a mrpt::graphs::CNetworkOfPoses<EDGE_TYPE,MAPS_IMPLEMENTATION>. Users won't have to write this template argument by hand, since the compiler will auto-fit it depending on the type of the graph object.
		  * \sa The example "graph_slam_demo"
		  * \ingroup mrpt_graphslam_grp
		  * \note Implementation can be found in file \a levmarq_impl.h
		  */
		template <class GRAPH_T>
		void optimize_graph_spa_levmarq(
			GRAPH_T & graph,
			TResultInfoSpaLevMarq                                        & out_info,
			const std::set<mrpt::utils::TNodeID>            * in_nodes_to_optimize = NULL,
			const mrpt::utils::TParametersDouble            & extra_params = mrpt::utils::TParametersDouble(),
			typename graphslam_traits<GRAPH_T>::TFunctorFeedback  functor_feedback = NULL
			)
		{
			using namespace mrpt;
			using namespace mrpt::poses;
			using namespace mrpt::graphslam;
			using namespace mrpt::math;
			using namespace mrpt::utils;
			using namespace std;

			MRPT_START

			// Typedefs to make life easier:
			typedef graphslam_traits<GRAPH_T> gst;

			typename gst::Array_O array_O_zeros; array_O_zeros.fill(0); // Auxiliary var with all zeros

			// The size of things here (because size matters...)
			static const unsigned int DIMS_POSE = gst::SE_TYPE::VECTOR_SIZE;

			// Read extra params:
			const bool verbose            = 0!=extra_params.getWithDefaultVal("verbose",0);
			const size_t max_iters        = extra_params.getWithDefaultVal("max_iterations",100);
			const bool   enable_profiler  = 0!=extra_params.getWithDefaultVal("profiler",0);
			// LM params:
			const double initial_lambda = extra_params.getWithDefaultVal("initial_lambda", 0);  // <=0: means auto guess
			const double tau = extra_params.getWithDefaultVal("tau", 1e-3);
			const double e1 = extra_params.getWithDefaultVal("e1",1e-6);
			const double e2 = extra_params.getWithDefaultVal("e2",1e-6);

			const double SCALE_HESSIAN = extra_params.getWithDefaultVal("scale_hessian",1);


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
				cout << "["<<__CURRENT_FUNCTION_NAME__<<"] " << nFreeNodes << " nodes to optimize: ";
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

				const typename gst::graph_t::constraint_t::type_value &EDGE_POSE  = edge.getPoseMean();

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
#if MRPT_HAS_CXX11
			typedef std::unique_ptr<CSparseMatrix::CholeskyDecomp> SparseCholeskyDecompPtr;
#else
			typedef std::auto_ptr<CSparseMatrix::CholeskyDecomp> SparseCholeskyDecompPtr;
#endif
			SparseCholeskyDecompPtr ptrCh;

			// The list of Jacobians: for each constraint i->j,
			//  we need the pair of Jacobians: { dh(xi,xj)_dxi, dh(xi,xj)_dxj },
			//  which are "first" and "second" in each pair.
			// Index of the map are the node IDs {i,j} for each contraint.
			typename gst::map_pairIDs_pairJacobs_t   lstJacobians;
			// The vector of errors: err_k = SE(2/3)::pseudo_Ln( P_i * EDGE_ij * inv(P_j) )
			typename mrpt::aligned_containers<typename gst::Array_O>::vector_t  errs; // Separated vectors for each edge. i \in [0,nObservations-1], in same order than lstObservationData

			// ===================================
			// Compute Jacobians & errors
			// ===================================
			profiler.enter("optimize_graph_spa_levmarq.Jacobians&err");// ------------------------------\  .
			double total_sqr_err = computeJacobiansAndErrors<GRAPH_T>(
				graph, lstObservationData,
				lstJacobians, errs);
			profiler.leave("optimize_graph_spa_levmarq.Jacobians&err");  // ------------------------------/


			// Only once (since this will be static along iterations), build a quick look-up table with the
			//  indices of the free nodes associated to the (first_id,second_id) of each Jacobian pair:
			// -----------------------------------------------------------------------------------------------
			vector<pair<size_t,size_t> >  observationIndex_to_relatedFreeNodeIndex; // "relatedFreeNodeIndex" means into [0,nFreeNodes-1], or "-1" if that node is fixed, as ordered in "nodes_to_optimize"
			observationIndex_to_relatedFreeNodeIndex.reserve(nObservations);
			ASSERTDEB_(lstJacobians.size()==nObservations)
			for (typename gst::map_pairIDs_pairJacobs_t::const_iterator itJ=lstJacobians.begin();itJ!=lstJacobians.end();++itJ)
			{
				const TNodeID id1 = itJ->first.first;
				const TNodeID id2 = itJ->first.second;
				observationIndex_to_relatedFreeNodeIndex.push_back(
					std::make_pair(
						mrpt::utils::find_in_vector(id1,*nodes_to_optimize),
						mrpt::utils::find_in_vector(id2,*nodes_to_optimize) ));
			}

			// other important vars for the main loop:
			CVectorDouble grad(nFreeNodes*DIMS_POSE);
			grad.setZero();
			typedef typename mrpt::aligned_containers<TNodeID,typename gst::matrix_VxV_t>::map_t  map_ID2matrix_VxV_t;
			vector<map_ID2matrix_VxV_t>  H_map(nFreeNodes);

			double	lambda = initial_lambda; // Will be actually set on first iteration.
			double	v = 1; // was 2, changed since it's modified in the first pass.
			bool    have_to_recompute_H_and_grad = true;

			// -----------------------------------------------------------
			// Main iterative loop of Levenberg-Marquardt algorithm
			//  For notation and overall algorithm overview, see:
			//  http://www.mrpt.org/Levenberg%E2%80%93Marquardt_algorithm
			// -----------------------------------------------------------
			size_t last_iter = 0;

			for (size_t iter=0;iter<max_iters;++iter)
			{
				last_iter = iter;

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
					typename mrpt::aligned_containers<typename gst::Array_O>::vector_t  grad_parts(nFreeNodes, array_O_zeros);

					// "lstJacobians" is sorted in the same order than "lstObservationData":
					ASSERT_EQUAL_(lstJacobians.size(),lstObservationData.size())

					{
						size_t idx_obs;
						typename gst::map_pairIDs_pairJacobs_t::const_iterator itJ;

						for (idx_obs=0, itJ=lstJacobians.begin();
							itJ!=lstJacobians.end();
							++itJ,++idx_obs)
						{
							ASSERTDEB_(itJ->first==lstObservationData[idx_obs].edge->first) // make sure they're in the expected order!

							//  grad[k] += J^t_{i->k} * Inf.Matrix * errs_i
							//    k: [0,nFreeNodes-1]     <-- IDs.first & IDs.second
							//    i: [0,nObservations-1]  <--- idx_obs

							// Get the corresponding indices in the vector of "free variables" being optimized:
							const size_t idx1 = observationIndex_to_relatedFreeNodeIndex[idx_obs].first;
							const size_t idx2 = observationIndex_to_relatedFreeNodeIndex[idx_obs].second;

							if (idx1!=string::npos)
								detail::AuxErrorEval<typename gst::edge_t,gst>::multiply_Jt_W_err(
									itJ->second.first /* J */,
									lstObservationData[idx_obs].edge /* W */,
									errs[idx_obs] /* err */,
									grad_parts[idx1] /* out */
									);

							if (idx2!=string::npos)
								detail::AuxErrorEval<typename gst::edge_t,gst>::multiply_Jt_W_err(
									itJ->second.second /* J */,
									lstObservationData[idx_obs].edge /* W */,
									errs[idx_obs] /* err */,
									grad_parts[idx2] /* out */
									);
						}
					}

					// build the gradient as a single vector:
					::memcpy(&grad[0],&grad_parts[0], nFreeNodes*DIMS_POSE*sizeof(grad[0]));  // Ohh yeahh!
					grad /= SCALE_HESSIAN;
					profiler.leave("optimize_graph_spa_levmarq.grad"); // ------------------------------/

					// End condition #1
					const double grad_norm_inf = math::norm_inf(grad); // inf-norm (abs. maximum value) of the gradient
					if (grad_norm_inf<=e1)
					{
						// Change is too small
						if (verbose) cout << "["<<__CURRENT_FUNCTION_NAME__<<"] " << mrpt::format("End condition #1: math::norm_inf(g)<=e1 :%f<=%f\n",grad_norm_inf,e1);
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
						typename gst::map_pairIDs_pairJacobs_t::const_iterator itJacobPair;

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
								typename gst::matrix_VxV_t JtJ(mrpt::math::UNINITIALIZED_MATRIX);
								detail::AuxErrorEval<typename gst::edge_t,gst>::multiplyJtLambdaJ(J1,JtJ,lstObservationData[idxObs].edge);
								H_map[idx_i][idx_i] += JtJ;
							}
							// Is "j" a free (to be optimized) node? -> Jj^t * Inf *  Jj
							if (is_j_free_node)
							{
								typename gst::matrix_VxV_t JtJ(mrpt::math::UNINITIALIZED_MATRIX);
								detail::AuxErrorEval<typename gst::edge_t,gst>::multiplyJtLambdaJ(J2,JtJ,lstObservationData[idxObs].edge);
								H_map[idx_j][idx_j] += JtJ;
							}
							// Are both "i" and "j" free nodes? -> Ji^t * Inf *  Jj
							if (is_i_free_node && is_j_free_node)
							{
								typename gst::matrix_VxV_t JtJ(mrpt::math::UNINITIALIZED_MATRIX);
								detail::AuxErrorEval<typename gst::edge_t,gst>::multiplyJ1tLambdaJ2(J1,J2,JtJ,lstObservationData[idxObs].edge);
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
							for (typename map_ID2matrix_VxV_t::const_iterator it=H_map[i].begin();it!=H_map[i].end();++it)
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
					else
					{
						// After recomputing H and the grad, we update lambda:
						lambda *= 0.1; //std::max(0.333, 1-pow(2*rho-1,3.0) );
					}
					utils::keep_max(lambda, 1e-200);  // JL: Avoids underflow!
					v = 2;
		#if 0
					{ mrpt::math::CMatrixDouble H; sp_H.get_dense(H); H.saveToTextFile("d:\\H.txt"); }
		#endif
				} // end "have_to_recompute_H_and_grad"

				if (verbose )
					cout << "["<<__CURRENT_FUNCTION_NAME__<<"] Iter: " << iter << " ,total sqr. err: " << total_sqr_err  << ", avrg. err per edge: " << std::sqrt(total_sqr_err/nObservations) << " lambda: " << lambda << endl;

				// Feedback to the user:
				if (functor_feedback)
				{
					(*functor_feedback)(graph,iter,max_iters,total_sqr_err);
				}


				profiler.enter("optimize_graph_spa_levmarq.sp_H:build"); // ------------------------------\  .
				// Now, build the actual sparse matrix H:
				// Note: we only need to fill out the upper diagonal part, since Cholesky will later on ignore the other part.
				CSparseMatrix  sp_H(nFreeNodes*DIMS_POSE,nFreeNodes*DIMS_POSE);
				for (size_t i=0;i<nFreeNodes;i++)
				{
					const size_t i_offset = i*DIMS_POSE;

					for (typename map_ID2matrix_VxV_t::const_iterator it=H_map[i].begin();it!=H_map[i].end();++it)
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
				CVectorDouble  delta; // The (minus) increment to be added to the current solution in this step
				try
				{
					profiler.enter("optimize_graph_spa_levmarq.sp_H:chol");
					if (!ptrCh.get())
							ptrCh = SparseCholeskyDecompPtr(new CSparseMatrix::CholeskyDecomp(sp_H) );
					else ptrCh.get()->update(sp_H);
					profiler.leave("optimize_graph_spa_levmarq.sp_H:chol");

					profiler.enter("optimize_graph_spa_levmarq.sp_H:backsub");
					ptrCh->backsub(grad,delta);
					profiler.leave("optimize_graph_spa_levmarq.sp_H:backsub");
				}
				catch (CExceptionNotDefPos &)
				{
					// not positive definite so increase mu and try again
					if (verbose ) cout << "["<<__CURRENT_FUNCTION_NAME__<<"] Got non-definite positive matrix, retrying with a larger lambda...\n";
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
						const typename gst::graph_t::constraint_t::type_value &P = itP->second;
						for (size_t i=0;i<DIMS_POSE;i++)
							x_norm+=square(P[i]);
					}
					x_norm=std::sqrt(x_norm);
				}
				profiler.leave("optimize_graph_spa_levmarq.x_norm" );

				// Test end condition #2:
				const double thres_norm = e2*(x_norm+e2);
				if (delta_norm<thres_norm)
				{
					// The change is too small: we're done here...
					if (verbose ) cout << "["<<__CURRENT_FUNCTION_NAME__<<"] " << format("End condition #2: %e < %e\n", delta_norm, thres_norm );
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
							typename gst::graph_t::constraint_t::type_value exp_delta_pose(UNINITIALIZED_POSE);
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
					typename gst::map_pairIDs_pairJacobs_t  new_lstJacobians;
					typename mrpt::aligned_containers<typename gst::Array_O>::vector_t   new_errs;

					profiler.enter("optimize_graph_spa_levmarq.Jacobians&err");// ------------------------------\  .
					double new_total_sqr_err = computeJacobiansAndErrors<GRAPH_T>(
						graph, lstObservationData,
						new_lstJacobians, new_errs);
					profiler.leave("optimize_graph_spa_levmarq.Jacobians&err");// ------------------------------/

					// Now, to decide whether to accept the change:
					if (new_total_sqr_err < total_sqr_err) // rho>0)
					{
						// Accept the new point:
						new_lstJacobians.swap(lstJacobians);
						new_errs.swap(errs);
						std::swap( new_total_sqr_err, total_sqr_err);

						// Instruct to recompute H and grad from the new Jacobians.
						have_to_recompute_H_and_grad = true;
					}
					else
					{
						// Nope...
						// We have to revert the "graph.nodes" to "old_poses_backup"
						for (typename gst::graph_t::global_poses_t::const_iterator it=old_poses_backup.begin();it!=old_poses_backup.end();++it)
							graph.nodes[it->first] = it->second;

						if (verbose ) cout << "["<<__CURRENT_FUNCTION_NAME__<<"] Got larger error=" << new_total_sqr_err << ", retrying with a larger lambda...\n";
						// Change params and try again:
						lambda *= v;
						v*= 2;
					}

				} // end else end condition #2

			} // end for each iter

			profiler.leave("optimize_graph_spa_levmarq (entire)");


			// Fill out basic output data:
			// ------------------------------
			out_info.num_iters = last_iter;
			out_info.final_total_sq_error = total_sqr_err;

			MRPT_END
		} // end of optimize_graph_spa_levmarq()


	/**  @} */  // end of grouping

	} // End of namespace
} // End of namespace

#endif
