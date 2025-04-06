/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/math/CMatrixDynamic.h>

#include <vector>

namespace mrpt
{
/** Abstract graph and tree data structures, plus generic graph algorithms
 * \ingroup  mrpt_graphs_grp
 */
namespace graphs
{
/** Finds the min-normalized-cut of a weighted undirected graph.
 * Two methods are provided:
 *  - SpectralBisection(): Bisection.
 *  - RecursiveSpectralPartition(): Iterative N-parts partition.
 *
 *  This is an implementation of the Shi-Malik method proposed in:
 *  - J. Shi and J. Malik, "Normalized Cuts and Image Segmentation",
 *    IEEE Transactions on Pattern Analysis and Machine Intelligence,
 *    vol.22, no.8, pp. 888-905, Aug. 2000.
 *
 * \tparam GRAPH_MATRIX The type of square matrices used to represent the
 * connectivity in a graph. Supported types are: mrpt::math::CMatrixDouble,
 * mrpt::math::CMatrixD, mrpt::math::CMatrixFloat, mrpt::math::CMatrixF
 *
 * \tparam num_t The type of matrix elements, thresholds, etc. (double or
 * float). Defaults to the type of matrix elements.
 */
template <class GRAPH_MATRIX, typename num_t = typename GRAPH_MATRIX::Scalar>
class CGraphPartitioner
{
 public:
  /** Performs the spectral recursive partition into K-parts for a given
   * graph.
   * The default threshold for the N-cut is 1, which correspond to a cut
   * equal of the geometric mean of self-associations of each pair of groups.
   *
   * \param in_A  [IN] The weights matrix for the graph. It must be a square
   * matrix, where element W<sub>ij</sub> is the "likelihood" between nodes
   * "i" and "j", and typically W<sub>ii</sub> = 1.
   * \param out_parts [OUT] An array of partitions, where each partition is
   * represented as a vector of indexs for nodes.
   * \param threshold_Ncut [IN] If it is desired to use other than the default
   * threshold, it can be passed here.
   * \param forceSimetry	 [IN] If set to true (default) the elements
   * W<sub>ij</sub> and W<sub>ji</sub> are replaced by
   * 0.5*(W<sub>ij</sub>+W<sub>ji</sub>). Set to false if matrix is known to
   * be simetric.
   * \param useSpectralBisection [IN] If set to true (default) a quick
   * spectral bisection will be used. If set to false, a brute force, exact
   * finding of the min-cut is performed.
   * \param recursive [IN] Default=true, recursive algorithm for finding N
   * partitions. Set to false to force 1 bisection as maximum.
   * \param minSizeClusters [IN] Default=1, Minimum size of partitions to be
   * accepted.
   *
   * \sa SpectralBisection
   *
   * \exception Throws a std::logic_error if an invalid matrix is passed.
   */
  static void RecursiveSpectralPartition(
      GRAPH_MATRIX& in_A,
      std::vector<std::vector<uint32_t>>& out_parts,
      num_t threshold_Ncut = 1,
      bool forceSimetry = true,
      bool useSpectralBisection = true,
      bool recursive = true,
      unsigned minSizeClusters = 1,
      const bool verbose = false);

  /** Performs the spectral bisection of a graph. This method always perform
   *   the bisection, and a measure of the goodness for this cut is returned.
   *
   * \param in_A			[IN] The weights matrix for the graph. It must be a
   * square
   * matrix, where element W<sub>ij</sub> is the "likelihood" between nodes
   * "i" and "j", and typically W<sub>ii</sub> = 1.
   * \param out_part1		[OUT] The indexs of the nodes that fall into the
   * first
   * group.
   * \param out_part2		[OUT] The indexs of the nodes that fall into the
   * second
   * group.
   * \param out_cut_value	[OUT] The N-cut value for the proposed cut, in the
   * range [0-2].
   * \param forceSimetry	[IN] If set to true (default) the elements
   * W<sub>ij</sub> and W<sub>ji</sub> are replaced by
   * 0.5*(W<sub>ij</sub>+W<sub>ji</sub>). Set to false if matrix is known to
   * be simetric.
   *
   * \sa mrpt::math::CMatrixF, RecursiveSpectralPartition
   *
   * \exception Throws a std::logic_error if an invalid matrix is passed.
   */
  static void SpectralBisection(
      GRAPH_MATRIX& in_A,
      std::vector<uint32_t>& out_part1,
      std::vector<uint32_t>& out_part2,
      num_t& out_cut_value,
      bool forceSimetry = true);

  /** Performs an EXACT minimum n-Cut graph bisection, (Use
   * CGraphPartitioner::SpectralBisection for a faster algorithm)
   *
   * \param in_A			[IN] The weights matrix for the graph. It must be a
   * square
   * matrix, where element W<sub>ij</sub> is the "likelihood" between nodes
   * "i" and "j", and typically W<sub>ii</sub> = 1.
   * \param out_part1		[OUT] The indexs of the nodes that fall into the
   * first
   * group.
   * \param out_part2		[OUT] The indexs of the nodes that fall into the
   * second
   * group.
   * \param out_cut_value	[OUT] The N-cut value for the proposed cut, in the
   * range [0-2].
   * \param forceSimetry	[IN] If set to true (default) the elements
   * W<sub>ij</sub> and W<sub>ji</sub> are replaced by
   * 0.5*(W<sub>ij</sub>+W<sub>ji</sub>). Set to false if matrix is known to
   * be simetric.
   *
   * \sa mrpt::math::CMatrixF, RecursiveSpectralPartition
   *
   * \exception Throws a std::logic_error if an invalid matrix is passed.
   */
  static void exactBisection(
      GRAPH_MATRIX& in_A,
      std::vector<uint32_t>& out_part1,
      std::vector<uint32_t>& out_part2,
      num_t& out_cut_value,
      bool forceSimetry = true);

  /** Returns the normaliced cut of a graph, given its adjacency matrix A and
   * a bisection:
   */
  static num_t nCut(
      const GRAPH_MATRIX& in_A,
      const std::vector<uint32_t>& in_part1,
      const std::vector<uint32_t>& in_part2);

};  // End of class def.

}  // namespace graphs
}  // namespace mrpt
