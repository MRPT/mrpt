/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/core/bits_math.h>  // .0_deg
#include <mrpt/poses/CPose2DGridTemplate.h>
#include <mrpt/poses/CPosePDF.h>

namespace mrpt::poses
{
/** Declares a class that represents a Probability Distribution
 *    function (PDF) of a 2D pose (x,y,phi).
 *   This class implements that PDF using a 3D grid.
 *
 * \sa CPose2D, CPosePDF, CPose2DGridTemplate
 * \ingroup poses_pdf_grp
 */
class CPosePDFGrid : public CPosePDF, public CPose2DGridTemplate<double>
{
	DEFINE_SERIALIZABLE(CPosePDFGrid, mrpt::poses)

   protected:
   public:
	/** Constructor: Initializes a, uniform distribution over the whole given
	 * range.
	 */
	CPosePDFGrid(
		double xMin = -1.0f, double xMax = 1.0f, double yMin = -1.0f,
		double yMax = 1.0f, double resolutionXY = 0.5f,
		double resolutionPhi = mrpt::DEG2RAD(180.0), double phiMin = -M_PI,
		double phiMax = M_PI);

	/** Destructor */
	~CPosePDFGrid() override;

	/** Copy operator, translating if necesary (for example, between particles
	 * and gaussian representations) */
	void copyFrom(const CPosePDF& o) override;

	/** Normalizes the PDF, such as all cells sum the unity. */
	void normalize();
	/** Assigns the same value to all the cells in the grid, so the sum 1. */
	void uniformDistribution();

	void getMean(CPose2D& mean_pose) const override;

	std::tuple<cov_mat_t, type_value> getCovarianceAndMean() const override;

	/** Save the contents of the 3D grid in one file, as a vertical
	 * concatenation of rectangular matrix for the different "PHI" discrete
	 * levels, and the size in X,Y,and PHI in another file named
	 * "<filename>_dims.txt". \return false on error */
	bool saveToTextFile(const std::string& dataFile) const override;

	/** this = p (+) this. This can be used to convert a PDF from local
	 * coordinates to global, providing the point (newReferenceBase) from which
	 *   "to project" the current pdf. Result PDF substituted the currently
	 * stored one in the object. */
	void changeCoordinatesReference(const CPose3D& newReferenceBase) override;
	/** Bayesian fusion of 2 densities (In the grid representation this becomes
	 * a pointwise multiplication) */
	void bayesianFusion(
		const CPosePDF& p1, const CPosePDF& p2,
		const double minMahalanobisDistToDrop = 0) override;
	/** Returns a new PDF such as: NEW_PDF = (0,0,0) - THIS_PDF */
	void inverse(CPosePDF& o) const override;
	/** Draws a single sample from the distribution (WARNING: weights are
	 * assumed to be normalized!) */
	void drawSingleSample(CPose2D& outPart) const override;
	/** Draws a number of samples from the distribution, and saves as a list of
	 * 1x3 vectors, where each row contains a (x,y,phi) datum. */
	void drawManySamples(
		size_t N,
		std::vector<mrpt::math::CVectorDouble>& outSamples) const override;

};	// End of class def.
}  // namespace mrpt::poses
