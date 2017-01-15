/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef GRAPH_SLAM_LEVMARQ_IMPL_H
#define GRAPH_SLAM_LEVMARQ_IMPL_H

#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/utils/CTimeLogger.h>
#include <mrpt/math/CSparseMatrix.h>

#include <memory>

namespace mrpt
{
	namespace graphslam
	{
		/// Internal auxiliary classes
		namespace detail
		{
			using namespace mrpt;
			using namespace mrpt::poses;
			using namespace mrpt::graphslam;
			using namespace mrpt::math;
			using namespace mrpt::utils;
			using namespace std;

			// An auxiliary struct to compute the pseudo-ln of a pose error, possibly modified with an information matrix.
			//  Specializations are below.
			template <class EDGE,class gst> struct AuxErrorEval;

			// For graphs of 2D constraints (no information matrix)
			template <class gst> struct AuxErrorEval<CPose2D,gst> {
				template <class POSE,class VEC,class EDGE_ITERATOR>
				static inline void computePseudoLnError(const POSE &P1DP2inv, VEC &err,
					const EDGE_ITERATOR &edge) {
					MRPT_UNUSED_PARAM(edge);
					gst::SE_TYPE::pseudo_ln(P1DP2inv, err);
				}

				template <class MAT,class EDGE_ITERATOR>
				static inline void 	multiplyJtLambdaJ(const MAT &J1, MAT &JtJ,
					const EDGE_ITERATOR &edge) {
					MRPT_UNUSED_PARAM(edge);
					JtJ.multiply_AtA(J1);
				}

				template <class MAT,class EDGE_ITERATOR>
				static inline void 	multiplyJ1tLambdaJ2(const MAT &J1, const MAT &J2, MAT &JtJ,
					const EDGE_ITERATOR &edge) {
					MRPT_UNUSED_PARAM(edge);
					JtJ.multiply_AtB(J1,J2);
				}

				template <class JAC,class EDGE_ITERATOR,class VEC1,class VEC2>
				static inline void multiply_Jt_W_err(const JAC &J,const EDGE_ITERATOR &edge,
					const VEC1 & ERR,VEC2 &OUT) {
					MRPT_UNUSED_PARAM(edge);
					J.multiply_Atb(ERR,OUT, true /* accumulate in output */ );
				}
			};

			// For graphs of 3D constraints (no information matrix)
			template <class gst> struct AuxErrorEval<CPose3D,gst>
			{
				template <class POSE,class VEC,class EDGE_ITERATOR>
				static inline void computePseudoLnError(const POSE &P1DP2inv, VEC &err,
					const EDGE_ITERATOR &edge) {
					MRPT_UNUSED_PARAM(edge);
					gst::SE_TYPE::pseudo_ln(P1DP2inv, err);
				}

				template <class MAT,class EDGE_ITERATOR>
				static inline void multiplyJtLambdaJ(const MAT &J1, MAT &JtJ,
					const EDGE_ITERATOR &edge) {
					MRPT_UNUSED_PARAM(edge);
					JtJ.multiply_AtA(J1);
				}

				template <class MAT,class EDGE_ITERATOR>
				static inline void 	multiplyJ1tLambdaJ2(const MAT &J1, const MAT &J2, MAT &JtJ,
					const EDGE_ITERATOR &edge) {
					MRPT_UNUSED_PARAM(edge);
					JtJ.multiply_AtB(J1,J2);
				}

				template <class JAC,class EDGE_ITERATOR,class VEC1,class VEC2>
				static inline void multiply_Jt_W_err(const JAC &J,const EDGE_ITERATOR &edge,
					const VEC1 & ERR,VEC2 &OUT) {
					MRPT_UNUSED_PARAM(edge);
					J.multiply_Atb(ERR,OUT, true /* accumulate in output */ );
				}
			};

			// For graphs of 2D constraints (with information matrix)
			template <class gst> struct AuxErrorEval<CPosePDFGaussianInf,gst>
			{
				template <class POSE,class VEC,class EDGE_ITERATOR>
				static inline void computePseudoLnError(const POSE &P1DP2inv, VEC &err,const EDGE_ITERATOR &edge)
				{
					gst::SE_TYPE::pseudo_ln(P1DP2inv, err);
				}

				template <class MAT,class EDGE_ITERATOR>
				static inline void multiplyJtLambdaJ(const MAT &J1, MAT &JtJ,const EDGE_ITERATOR &edge) { JtJ.multiply_AtBC(J1,edge->second.cov_inv,J1); }
				template <class MAT,class EDGE_ITERATOR>
				static inline void 	multiplyJ1tLambdaJ2(const MAT &J1, const MAT &J2, MAT &JtJ,const EDGE_ITERATOR &edge) { JtJ.multiply_AtBC(J1,edge->second.cov_inv,J2); }

				template <class JAC,class EDGE_ITERATOR,class VEC1,class VEC2>
				static inline void multiply_Jt_W_err(const JAC &J,const EDGE_ITERATOR &edge,const VEC1 & ERR,VEC2 &OUT)
				{
				 mrpt::math::CMatrixDouble33 JtInf(mrpt::math::UNINITIALIZED_MATRIX);
					JtInf.multiply_AtB(J,edge->second.cov_inv);
					JtInf.multiply_Ab(ERR,OUT, true /* accumulate in output */ );
				}
			};

			// For graphs of 3D constraints (with information matrix)
			template <class gst> struct AuxErrorEval<CPose3DPDFGaussianInf,gst> {
				template <class POSE,class VEC,class EDGE_ITERATOR>
				static inline void computePseudoLnError(const POSE &P1DP2inv, VEC &err,const EDGE_ITERATOR &edge) {
					MRPT_UNUSED_PARAM(edge);
					gst::SE_TYPE::pseudo_ln(P1DP2inv, err);
				}

				template <class MAT,class EDGE_ITERATOR>
				static inline void multiplyJtLambdaJ(const MAT &J1, MAT &JtJ,const EDGE_ITERATOR &edge) { JtJ.multiply_AtBC(J1,edge->second.cov_inv,J1); }

				template <class MAT,class EDGE_ITERATOR>
				static inline void 	multiplyJ1tLambdaJ2(const MAT &J1, const MAT &J2, MAT &JtJ,const EDGE_ITERATOR &edge) { JtJ.multiply_AtBC(J1,edge->second.cov_inv,J2); }

				template <class JAC,class EDGE_ITERATOR,class VEC1,class VEC2>
				static inline void multiply_Jt_W_err(const JAC &J,const EDGE_ITERATOR &edge,const VEC1 & ERR,VEC2 &OUT)
				{
				 mrpt::math::CMatrixDouble66 JtInf(mrpt::math::UNINITIALIZED_MATRIX);
					JtInf.multiply_AtB(J,edge->second.cov_inv);
					JtInf.multiply_Ab(ERR,OUT, true /* accumulate in output */ );
				}
			};

		} // end NS detail

		// Compute, at once, jacobians and the error vectors for each constraint in "lstObservationData", returns the overall squared error.
		template <class GRAPH_T>
		double computeJacobiansAndErrors(
			const GRAPH_T &graph,
			const std::vector<typename graphslam_traits<GRAPH_T>::observation_info_t>  &lstObservationData,
			typename graphslam_traits<GRAPH_T>::map_pairIDs_pairJacobs_t   &lstJacobians,
			typename mrpt::aligned_containers<typename graphslam_traits<GRAPH_T>::Array_O>::vector_t &errs
			)
		{
			MRPT_UNUSED_PARAM(graph);
			typedef graphslam_traits<GRAPH_T> gst;

			lstJacobians.clear();
			errs.clear();

			const size_t nObservations = lstObservationData.size();

			for (size_t i=0;i<nObservations;i++)
			{
				const typename gst::observation_info_t & obs = lstObservationData[i];
				typename gst::edge_const_iterator it = obs.edge;
				const typename gst::graph_t::constraint_t::type_value* EDGE_POSE = obs.edge_mean;
				typename gst::graph_t::constraint_t::type_value* P1 = obs.P1;
				typename gst::graph_t::constraint_t::type_value* P2 = obs.P2;

				const mrpt::utils::TPairNodeIDs                   &ids  = it->first;
				const typename gst::graph_t::edge_t  &edge = it->second;

				// Compute the residual pose error of these pair of nodes + its constraint,
				//  that is: P1DP2inv = P1 * EDGE * inv(P2)
				typename gst::graph_t::constraint_t::type_value P1DP2inv(mrpt::poses::UNINITIALIZED_POSE);
				{
					typename gst::graph_t::constraint_t::type_value P1D(mrpt::poses::UNINITIALIZED_POSE);
					P1D.composeFrom(*P1,*EDGE_POSE);
					const typename gst::graph_t::constraint_t::type_value P2inv = -(*P2); // Pose inverse (NOT just switching signs!)
					P1DP2inv.composeFrom(P1D,P2inv);
				}

				// Add to vector of errors:
				errs.resize(errs.size()+1);
				detail::AuxErrorEval<typename gst::edge_t,gst>::computePseudoLnError(P1DP2inv, errs.back(),edge);

				// Compute the jacobians:
				MRPT_ALIGN16 std::pair<mrpt::utils::TPairNodeIDs,typename gst::TPairJacobs> newMapEntry;
				newMapEntry.first = ids;
				gst::SE_TYPE::jacobian_dP1DP2inv_depsilon(P1DP2inv, &newMapEntry.second.first,&newMapEntry.second.second);

				// And insert into map of jacobians:
				lstJacobians.insert(lstJacobians.end(),newMapEntry );
			}

			// return overall square error:  (Was: std::accumulate(...,mrpt::math::squareNorm_accum<>), but led to GCC errors when enabling parallelization)
			double ret_err = 0.0;
			for (size_t i=0;i<errs.size();i++) ret_err+=errs[i].squaredNorm();
			return ret_err;
		}

	} // end of NS
} // end of NS

#endif
