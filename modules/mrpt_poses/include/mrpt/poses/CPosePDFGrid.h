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

#include <mrpt/core/bits_math.h>  // .0_deg
#include <mrpt/poses/CPose2DGridTemplate.h>
#include <mrpt/poses/CPosePDF.h>

namespace mrpt::poses
{
/** Represents a Probability Distribution Function (PDF) of a 2D pose
 * (x, y, phi) as a discrete 3D grid over the SE(2) space.
 *
 * Each cell in the grid stores a (non-negative) probability value.
 * The grid spans a rectangular region in (x, y) and a range of heading
 * angles phi. Cell resolutions are set independently for the spatial and
 * angular dimensions.
 *
 * **Important:** The constructor initializes the grid with a *uniform*
 * distribution (all cells equal, summing to 1). If you intend to build a
 * custom distribution, call uniformDistribution() or manually zero out cells
 * before assigning probabilities, then call normalize().
 *
 * \sa CPose2D, CPosePDF, CPose2DGridTemplate
 * \ingroup poses_pdf_grp
 */
class CPosePDFGrid : public CPosePDF, public CPose2DGridTemplate<double>
{
  DEFINE_SERIALIZABLE(CPosePDFGrid, mrpt::poses)

 public:
  /** Constructor: Initializes a uniform distribution over the given range.
   *
   * \param[in] xMin     Minimum x coordinate (meters).
   * \param[in] xMax     Maximum x coordinate (meters).
   * \param[in] yMin     Minimum y coordinate (meters).
   * \param[in] yMax     Maximum y coordinate (meters).
   * \param[in] resolutionXY  Spatial cell size (meters).
   * \param[in] resolutionPhi Angular cell size (radians).
   * \param[in] phiMin   Minimum heading angle (radians, default -π).
   * \param[in] phiMax   Maximum heading angle (radians, default +π).
   *
   * \note After construction all cells have equal probability (uniform).
   *       Use getByPos() / getByIndex() to modify individual cells, then
   *       call normalize() before sampling or querying the distribution.
   */
  CPosePDFGrid(
      double xMin = -1.0f,
      double xMax = 1.0f,
      double yMin = -1.0f,
      double yMax = 1.0f,
      double resolutionXY = 0.5f,
      double resolutionPhi = mrpt::DEG2RAD(180.0),
      double phiMin = -M_PI,
      double phiMax = M_PI);

  ~CPosePDFGrid() override;

  /** Copy from another PDF, translating representations if needed. */
  void copyFrom(const CPosePDF& o) override;

  /** Normalizes the PDF so that all cells sum to 1. */
  void normalize();

  /** Resets all cells to a uniform distribution (all equal, summing to 1). */
  void uniformDistribution();

  /** Computes the mean pose of the distribution. */
  void getMean(CPose2D& mean_pose) const override;

  /** Returns the covariance matrix and the mean of the distribution. */
  std::tuple<cov_mat_t, type_value> getCovarianceAndMean() const override;

  /** Save the 3D grid to a text file as vertically concatenated matrices
   * (one per phi level). A companion file "<filename>_dims.txt" stores the
   * grid dimensions. \return false on error */
  bool saveToTextFile(const std::string& dataFile) const override;

  /** Applies a coordinate change: this = newReferenceBase (+) this.
   * Useful for converting from local to global coordinates. */
  void changeCoordinatesReference(const CPose3D& newReferenceBase) override;

  /** Bayesian fusion of two densities via pointwise multiplication.
   * \param[in] minMahalanobisDistToDrop Unused in the grid representation. */
  void bayesianFusion(
      const CPosePDF& p1, const CPosePDF& p2, const double minMahalanobisDistToDrop = 0) override;

  /** Returns the inverse PDF: NEW_PDF = (0,0,0) − THIS_PDF. */
  void inverse(CPosePDF& o) const override;

  /** Draws a single sample from the distribution.
   * \note The distribution must be normalized (call normalize() first). */
  void drawSingleSample(CPose2D& outPart) const override;

  /** Draws N samples from the distribution.
   * Each entry in \a outSamples is a 3-element vector [x, y, phi]. */
  void drawManySamples(size_t N, std::vector<mrpt::math::CVectorDouble>& outSamples) const override;

  void printTo(std::ostream& out) const override;

};  // End of class def.
}  // namespace mrpt::poses
