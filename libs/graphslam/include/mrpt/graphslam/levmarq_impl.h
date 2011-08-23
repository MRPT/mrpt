/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
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
		using namespace mrpt;
		using namespace mrpt::poses;
		using namespace mrpt::graphslam;
		using namespace mrpt::math;
		using namespace mrpt::utils;
		using namespace std;

		/// Internal auxiliary classes
		namespace detail 
		{
			// An auxiliary struct to compute the pseudo-ln of a pose error, possibly modified with an information matrix.
			//  Specializations are below.
			template <class EDGE,class gst> struct AuxErrorEval;

			// For graphs of 2D constraints (no information matrix)
			template <class gst> struct AuxErrorEval<CPose2D,gst> {
				template <class POSE,class VEC,class EDGE_ITERATOR>
				static inline void computePseudoLnError(const POSE &P1DP2inv, VEC &err,const EDGE_ITERATOR &edge) { gst::SE_TYPE::pseudo_ln(P1DP2inv, err); }

				template <class MAT,class EDGE_ITERATOR>
				static inline void 	multiplyJtLambdaJ(const MAT &J1, MAT &JtJ,const EDGE_ITERATOR &edge) { JtJ.multiply_AtA(J1);  }

				template <class MAT,class EDGE_ITERATOR>
				static inline void 	multiplyJ1tLambdaJ2(const MAT &J1, const MAT &J2, MAT &JtJ,const EDGE_ITERATOR &edge) { JtJ.multiply_AtB(J1,J2); }

				template <class JAC,class EDGE_ITERATOR,class VEC1,class VEC2>
				static inline void multiply_Jt_W_err(const JAC &J,const EDGE_ITERATOR &edge,const VEC1 & ERR,VEC2 &OUT) { J.multiply_Atb(ERR,OUT, true /* accumulate in output */ ); }
			};

			// For graphs of 3D constraints (no information matrix)
			template <class gst> struct AuxErrorEval<CPose3D,gst> {
				template <class POSE,class VEC,class EDGE_ITERATOR>
				static inline void computePseudoLnError(const POSE &P1DP2inv, VEC &err,const EDGE_ITERATOR &edge) { gst::SE_TYPE::pseudo_ln(P1DP2inv, err); }

				template <class MAT,class EDGE_ITERATOR>
				static inline void multiplyJtLambdaJ(const MAT &J1, MAT &JtJ,const EDGE_ITERATOR &edge) { JtJ.multiply_AtA(J1); }

				template <class MAT,class EDGE_ITERATOR>
				static inline void 	multiplyJ1tLambdaJ2(const MAT &J1, const MAT &J2, MAT &JtJ,const EDGE_ITERATOR &edge) { JtJ.multiply_AtB(J1,J2); }

				template <class JAC,class EDGE_ITERATOR,class VEC1,class VEC2>
				static inline void multiply_Jt_W_err(const JAC &J,const EDGE_ITERATOR &edge,const VEC1 & ERR,VEC2 &OUT) { J.multiply_Atb(ERR,OUT, true /* accumulate in output */ ); }
			};

			// For graphs of 2D constraints (with information matrix)
			template <class gst> struct AuxErrorEval<CPosePDFGaussianInf,gst> {
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
					CMatrixDouble33 JtInf(UNINITIALIZED_MATRIX);
					JtInf.multiply_AtB(J,edge->second.cov_inv);
					JtInf.multiply_Ab(ERR,OUT, true /* accumulate in output */ );
				}
			};

			// For graphs of 3D constraints (with information matrix)
			template <class gst> struct AuxErrorEval<CPose3DPDFGaussianInf,gst> {
				template <class POSE,class VEC,class EDGE_ITERATOR>
				static inline void computePseudoLnError(const POSE &P1DP2inv, VEC &err,const EDGE_ITERATOR &edge) { gst::SE_TYPE::pseudo_ln(P1DP2inv, err); }

				template <class MAT,class EDGE_ITERATOR>
				static inline void multiplyJtLambdaJ(const MAT &J1, MAT &JtJ,const EDGE_ITERATOR &edge) { JtJ.multiply_AtBC(J1,edge->second.cov_inv,J1); }

				template <class MAT,class EDGE_ITERATOR>
				static inline void 	multiplyJ1tLambdaJ2(const MAT &J1, const MAT &J2, MAT &JtJ,const EDGE_ITERATOR &edge) { JtJ.multiply_AtBC(J1,edge->second.cov_inv,J2); }

				template <class JAC,class EDGE_ITERATOR,class VEC1,class VEC2>
				static inline void multiply_Jt_W_err(const JAC &J,const EDGE_ITERATOR &edge,const VEC1 & ERR,VEC2 &OUT)
				{
					CMatrixDouble66 JtInf(UNINITIALIZED_MATRIX);
					JtInf.multiply_AtB(J,edge->second.cov_inv);
					JtInf.multiply_Ab(ERR,OUT, true /* accumulate in output */ );
				}
			};

		} // end NS detail

		// Compute, at once, jacobians and the error vectors for each constraint in "lstObservationData", returns the overall squared error.
		template <class GRAPH_T>
		double computeJacobiansAndErrors(
			const GRAPH_T &graph,
			const vector<typename graphslam_traits<GRAPH_T>::observation_info_t>  &lstObservationData,
			typename graphslam_traits<GRAPH_T>::map_pairIDs_pairJacobs_t   &lstJacobians,
			typename mrpt::aligned_containers<typename graphslam_traits<GRAPH_T>::Array_O>::vector_t &errs
			)
		{
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

				const TPairNodeIDs                   &ids  = it->first;
				const typename gst::graph_t::edge_t  &edge = it->second;

				// Compute the residual pose error of these pair of nodes + its constraint,
				//  that is: P1DP2inv = P1 * EDGE * inv(P2)
				typename gst::graph_t::constraint_t::type_value P1DP2inv(UNINITIALIZED_POSE);
				{
					typename gst::graph_t::constraint_t::type_value P1D(UNINITIALIZED_POSE);
					P1D.composeFrom(*P1,*EDGE_POSE);
					const typename gst::graph_t::constraint_t::type_value P2inv = -(*P2); // Pose inverse (NOT just switching signs!)
					P1DP2inv.composeFrom(P1D,P2inv);
				}

				// Add to vector of errors:
				errs.resize(errs.size()+1);
				detail::AuxErrorEval<typename gst::edge_t,gst>::computePseudoLnError(P1DP2inv, errs.back(),edge);

				// Compute the jacobians:
				EIGEN_ALIGN16 std::pair<TPairNodeIDs,typename gst::TPairJacobs> newMapEntry;
				newMapEntry.first = ids;
				gst::SE_TYPE::jacobian_dP1DP2inv_depsilon(P1DP2inv, &newMapEntry.second.first,&newMapEntry.second.second);

				// And insert into map of jacobians:
				lstJacobians.insert(lstJacobians.end(),newMapEntry );
			}

			// return overall square error:
			return std::accumulate( errs.begin(), errs.end(),0.0, mrpt::math::squareNorm_accum<typename gst::Array_O> );
		}

	} // end of NS
} // end of NS

#endif
