/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
#pragma once

#include <Eigen/Dense>
#include <vector>

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
using namespace std;

// An auxiliary struct to compute the pseudo-ln of a pose error, possibly
// modified with an information matrix.
//  Specializations are below.
template <class EDGE, class gst>
struct AuxErrorEval;

// For graphs of constraints without uncertainty (unit information matrix)
template <class gst>
struct AuxErrorEvalNoInfo
{
  template <class OBS>
  static void computeInformation([[maybe_unused]] OBS& obs)
  {
  }

  template <class MAT, class OBS>
  static void multiplyJtLambdaJ(const MAT& J1, MAT& JtJ, [[maybe_unused]] const OBS& obs)
  {
    JtJ.matProductOf_AtA(J1);
  }

  template <class MAT, class OBS>
  static void multiplyJ1tLambdaJ2(
      const MAT& J1, const MAT& J2, MAT& JtJ, [[maybe_unused]] const OBS& obs)
  {
    JtJ.asEigen() = J1.transpose() * J2.asEigen();
  }

  template <class JAC, class OBS, class VEC1, class VEC2>
  static void multiply_Jt_W_err(
      const JAC& J, [[maybe_unused]] const OBS& obs, const VEC1& ERR, VEC2& OUT)
  {
    const auto grad_incr = (J.transpose() * ERR.asEigen()).eval();
    OUT.asEigen() += grad_incr;
  }

  template <class VEC, class OBS>
  static double squaredError(const VEC& ERR, [[maybe_unused]] const OBS& obs)
  {
    return ERR.asEigen().squaredNorm();
  }
};

// For graphs of constraints with an information matrix
template <class gst>
struct AuxErrorEvalInfo
{
  // The edge PDF holds the information of the pose parameters, convert it
  // into that of the local residual:
  template <class OBS>
  static void computeInformation(OBS& obs)
  {
    obs.information =
        mrpt::graphs::detail::infoParamsToLocal(*obs.edge_mean, obs.edge->second.cov_inv);
  }

  template <class MAT, class OBS>
  static void multiplyJtLambdaJ(const MAT& J1, MAT& JtJ, const OBS& obs)
  {
    JtJ.asEigen() = (J1.transpose() * obs.information.asEigen()) * J1.asEigen();
  }

  template <class MAT, class OBS>
  static void multiplyJ1tLambdaJ2(const MAT& J1, const MAT& J2, MAT& JtJ, const OBS& obs)
  {
    JtJ.asEigen() = (J1.transpose() * obs.information.asEigen()) * J2.asEigen();
  }

  template <class JAC, class OBS, class VEC1, class VEC2>
  static void multiply_Jt_W_err(const JAC& J, const OBS& obs, const VEC1& ERR, VEC2& OUT)
  {
    OUT.asEigen() += (J.transpose() * obs.information.asEigen()) * ERR.asEigen();
  }

  template <class VEC, class OBS>
  static double squaredError(const VEC& ERR, const OBS& obs)
  {
    return ERR.asEigen().dot(obs.information.asEigen() * ERR.asEigen());
  }
};

template <class gst>
struct AuxErrorEval<CPose2D, gst> : public AuxErrorEvalNoInfo<gst>
{
};
template <class gst>
struct AuxErrorEval<CPose3D, gst> : public AuxErrorEvalNoInfo<gst>
{
};
template <class gst>
struct AuxErrorEval<CPosePDFGaussianInf, gst> : public AuxErrorEvalInfo<gst>
{
};
template <class gst>
struct AuxErrorEval<CPose3DPDFGaussianInf, gst> : public AuxErrorEvalInfo<gst>
{
};

}  // namespace detail

// Compute, at once, jacobians and the error vectors for each constraint in
// "lstObservationData", returns the overall squared error, weighted by the
// information matrices of the edges, if any.
template <class GRAPH_T>
double computeJacobiansAndErrors(
    const GRAPH_T& graph,
    const std::vector<typename graphslam_traits<GRAPH_T>::observation_info_t>& lstObservationData,
    typename graphslam_traits<GRAPH_T>::map_pairIDs_pairJacobs_t& lstJacobians,
    std::vector<typename graphslam_traits<GRAPH_T>::Array_O>& errs)
{
  using gst = graphslam_traits<GRAPH_T>;

  lstJacobians.clear();
  errs.clear();

  const size_t nObservations = lstObservationData.size();
  // An inverse edge i->j holds i (-) j, so its residual is that of the
  // normal edge j->i:
  const bool inverseEdges = graph.edges_store_inverse_poses;

  double ret_err = 0.0;
  for (size_t i = 0; i < nObservations; i++)
  {
    const typename gst::observation_info_t& obs = lstObservationData[i];
    auto e = obs.edge;
    const typename gst::graph_t::constraint_t::type_value* EDGE_POSE = obs.edge_mean;
    // Poses of the edge origin and end nodes, as seen by the residual:
    const typename gst::graph_t::constraint_t::type_value* P1 = inverseEdges ? obs.P2 : obs.P1;
    const typename gst::graph_t::constraint_t::type_value* P2 = inverseEdges ? obs.P1 : obs.P2;

    const auto& ids = e->first;

    // Compute the residual pose error of these pair of nodes + its
    // constraint:
    // DinvP1invP2 = inv(EDGE) * inv(P1) * P2 = (P2 \ominus P1) \ominus EDGE
    typename gst::graph_t::constraint_t::type_value DinvP1invP2 = ((*P2) - (*P1)) - *EDGE_POSE;

    // Add to vector of errors:
    errs.resize(errs.size() + 1);
    errs.back() = gst::SE_TYPE::log(DinvP1invP2);
    ret_err += detail::AuxErrorEval<typename gst::edge_t, gst>::squaredError(errs.back(), obs);

    // Compute the jacobians, with respect to the poses of ids.first and
    // ids.second, in this order:
    alignas(MRPT_MAX_STATIC_ALIGN_BYTES)
        std::pair<mrpt::graphs::TPairNodeIDs, typename gst::TPairJacobs>
            newMapEntry;
    newMapEntry.first = ids;
    if (inverseEdges)
    {
      gst::SE_TYPE::jacob_dDinvP1invP2_de1e2(
          -(*EDGE_POSE), *P1, *P2, newMapEntry.second.second, newMapEntry.second.first);
    }
    else
    {
      gst::SE_TYPE::jacob_dDinvP1invP2_de1e2(
          -(*EDGE_POSE), *P1, *P2, newMapEntry.second.first, newMapEntry.second.second);
    }

    // And insert into map of jacobians:
    lstJacobians.insert(lstJacobians.end(), newMapEntry);
  }

  return ret_err;
}

}  // namespace graphslam
}  // namespace mrpt
