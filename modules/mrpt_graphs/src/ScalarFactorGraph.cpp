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

#include <mrpt/graphs/ScalarFactorGraph.h>
#include <mrpt/system/CTicTac.h>

#include <Eigen/Dense>

using namespace mrpt;
using namespace mrpt::graphs;
using namespace std;

#if EIGEN_VERSION_AT_LEAST(3, 1, 0)  // Requires Eigen>=3.1
#include <Eigen/SparseCore>
#include <Eigen/SparseQR>
#endif

ScalarFactorGraph::FactorBase::~FactorBase() = default;
ScalarFactorGraph::ScalarFactorGraph() : COutputLogger("GMRF") {}
void ScalarFactorGraph::clear()
{
  MRPT_LOG_DEBUG("clear() called");

  m_numNodes = 0;
  m_factors_unary.clear();
  m_factors_binary.clear();
}

void ScalarFactorGraph::initialize(size_t nodeCount)
{
  MRPT_LOG_DEBUG_STREAM("initialize() called, nodeCount=" << nodeCount);

  m_numNodes = nodeCount;
}

void ScalarFactorGraph::addConstraint(const UnaryFactorVirtualBase& c)
{
  m_factors_unary.push_back(&c);
}
void ScalarFactorGraph::addConstraint(const BinaryFactorVirtualBase& c)
{
  m_factors_binary.push_back(&c);
}

bool ScalarFactorGraph::eraseConstraint(const FactorBase& c)
{
  {
    auto it = std::find(m_factors_unary.begin(), m_factors_unary.end(), &c);
    if (it != m_factors_unary.end())
    {
      m_factors_unary.erase(it);
      return true;
    }
  }
  {
    auto it = std::find(m_factors_binary.begin(), m_factors_binary.end(), &c);
    if (it != m_factors_binary.end())
    {
      m_factors_binary.erase(it);
      return true;
    }
  }
  return false;
}

/* Method:
  (\Sigma)^{-1/2) *  d( h(x) )/d( x )  * x_incr = - (\Sigma)^{-1/2) * r(x)
  ===================================            ========================
        =A                                           =b

   A * x_incr = b         --> SparseQR.
*/
void ScalarFactorGraph::updateEstimation(
    /** Output increment of the current estimate. Caller must add this
       vector to current state vector to obtain the optimal estimation. */
    mrpt::math::CVectorDouble& solved_x_inc,
    /** If !=nullptr, the covariance of the estimate will be stored here. */
    mrpt::math::CVectorDouble* solved_variances)
{
  ASSERTMSG_(m_numNodes > 0, "numNodes=0. Have you called initialize()?");

  m_timelogger.enable(m_enable_profiler);

#if EIGEN_VERSION_AT_LEAST(3, 1, 0)

  // Number of vertices:
  const int n = static_cast<int>(m_numNodes);
  solved_x_inc.setZero(n, 1);

  // Number of edges:
  const int m1 = static_cast<int>(m_factors_unary.size());
  const int m2 = static_cast<int>(m_factors_binary.size());
  const int m = m1 + m2;

  // Build Ax=b
  // -----------------------
  m_timelogger.enter("GMRF.build_A_tri");

  std::vector<Eigen::Triplet<double>> A_tri;
  A_tri.reserve(m1 + 2 * m2);

  Eigen::VectorXd g;  // Error vector

  g.setZero(m);
  int edge_counter = 0;
  for (const auto& e : m_factors_unary)
  {
    ASSERT_(e != nullptr);
    // Jacob:
    const double w = std::sqrt(e->getInformation());
    double dr_dx;
    e->evalJacobian(dr_dx);
    const int node_id = static_cast<int>(e->node_id);
    A_tri.emplace_back(edge_counter, node_id, w * dr_dx);
    // gradient:
    g[edge_counter] -= w * e->evaluateResidual();

    ++edge_counter;
  }
  for (const auto& e : m_factors_binary)
  {
    ASSERT_(e != nullptr);
    // Jacob:
    const double w = std::sqrt(e->getInformation());
    double dr_dxi;
    double dr_dxj;
    e->evalJacobian(dr_dxi, dr_dxj);
    const int node_id_i = static_cast<int>(e->node_id_i);
    const int node_id_j = static_cast<int>(e->node_id_j);
    A_tri.emplace_back(edge_counter, node_id_i, w * dr_dxi);
    A_tri.emplace_back(edge_counter, node_id_j, w * dr_dxj);
    // gradient:
    g[edge_counter] -= w * e->evaluateResidual();

    ++edge_counter;
  }
  m_timelogger.leave("GMRF.build_A_tri");

  // Compress sparse
  // -----------------------
  Eigen::SparseMatrix<double> A(m, n);
  {
    mrpt::system::CTimeLoggerEntry tle(m_timelogger, "GMRF.build_A_compress");

    A.setFromTriplets(A_tri.begin(), A_tri.end());
    A.makeCompressed();
  }

  // Solve increment
  // -----------------------
  Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> solver;
  {
    mrpt::system::CTimeLoggerEntry tle(m_timelogger, "GMRF.solve");

    solver.compute(A);
    solved_x_inc = solver.solve(g);
  }

  // Recover covariance
  // -----------------------
  if (solved_variances)
  {
    mrpt::system::CTimeLoggerEntry tle(m_timelogger, "GMRF.variance");

    solved_variances->resize(n);

    // VARIANCE SIGMA = inv(P) * inv( P*H*inv(P) ) * P
    // Get triangular supperior P*H*inv(P) = UT' * UT = P * R'*R * inv(P)
    // (QR factor: Use UT=R)

    Eigen::SparseMatrix<double> UT = solver.matrixR();
    UT.makeCompressed();
    // Row-major view for O(nnz) row traversal instead of O(n) coeff() calls:
    Eigen::SparseMatrix<double, Eigen::RowMajor> UTrow = UT;

    Eigen::SparseMatrix<double> solved_covariance(n, n);
    solved_covariance.reserve(UT.nonZeros());

    // Apply custom equations to obtain the inverse -> inv( P*H*inv(P) )
    const int show_progress_steps = std::max(20, n / 20);
    for (int l = n - 1; l >= 0; l--)
    {
      if (!(l % show_progress_steps))
      {
        MRPT_LOG_DEBUG_FMT(
            "Computing variance %6.02f%%... \r", 100.0 * static_cast<double>(n - l - 1) / n);
      }

      const double utll = UT.coeff(l, l);

      // Iterate only over non-zero columns j > l in row l of UT:
      double subSigmas = 0.0;
      for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator jt(UTrow, l); jt; ++jt)
      {
        const int j = static_cast<int>(jt.col());
        if (j <= l)
        {
          continue;  // upper triangular only
        }

        // Compute off-diagonal variances Sigma(j,l) = Sigma(l,j)
        // using only non-zero entries in row l of UT (SUM 1 and SUM 2 merged):
        double sum = 0.0;
        for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(UTrow, l); it; ++it)
        {
          const int i = static_cast<int>(it.col());
          if (i <= l)
          {
            continue;
          }
          if (i <= j)
          {
            sum += it.value() * solved_covariance.coeff(i, j);
          }
          else
          {
            sum += it.value() * solved_covariance.coeff(j, i);
          }
        }
        solved_covariance.insert(l, j) = -sum / utll;
        subSigmas += jt.value() * solved_covariance.coeff(l, j);
      }

      solved_covariance.insert(l, l) = (1.0 / utll) * (1.0 / utll - subSigmas);
    }

    MRPT_LOG_DEBUG_FMT("Computing variance %6.02f%%... \r", 100.0);

    for (int i = 0; i < n; i++)
    {
      const int idx = static_cast<int>(solver.colsPermutation().indices().coeff(i));
      const double variance = solved_covariance.coeff(i, i);
      (*solved_variances)[idx] = variance;
    }

  }  // end calc variances

#else
  THROW_EXCEPTION("This method requires Eigen 3.1.0 or above");
#endif
}
