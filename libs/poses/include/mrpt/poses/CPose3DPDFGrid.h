/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/core/bits_math.h>  // .0_deg
#include <mrpt/math/TPose3D.h>
#include <mrpt/poses/CPose3DGridTemplate.h>
#include <mrpt/poses/CPose3DPDF.h>

namespace mrpt::poses
{
/** Declares a class that represents a Probability Distribution
 *  function (PDF) of a SE(3) pose (x,y,z, yaw, pitch, roll), in
 * the form of a 6-dimensional grid of "voxels".
 *
 * \sa CPose3D, CPose3DPDF, CPose3DGridTemplate
 * \ingroup poses_pdf_grp
 */
class CPose3DPDFGrid : public CPose3DPDF, public CPose3DGridTemplate<double>
{
	DEFINE_SERIALIZABLE(CPose3DPDFGrid, mrpt::poses)

   protected:
   public:
	/** Constructor: Initializes a, uniform distribution over the whole given
	 * range, given by a "rectangular" (6D) bounding box.
	 */
	CPose3DPDFGrid(
		const mrpt::math::TPose3D& bb_min =
			mrpt::math::TPose3D(-1., -1., -1., -M_PI, -.5 * M_PI, -.5 * M_PI),
		const mrpt::math::TPose3D& bb_max =
			mrpt::math::TPose3D(1., 1., 1., M_PI, .5 * M_PI, .5 * M_PI),
		double resolution_XYZ = 0.10,
		double resolution_YPR = mrpt::DEG2RAD(10.0));

	/** Destructor */
	~CPose3DPDFGrid() override = default;

	/** Copy operator, translating if necesary (for example, between
	 * particles and gaussian representations) */
	void copyFrom(const CPose3DPDF& o) override;

	/** Normalizes the PDF, such as all voxels sum the unity. */
	void normalize();

	/** Assigns the same value to all the cells in the grid, so the sum 1 */
	void uniformDistribution();

	void getMean(CPose3D& mean_pose) const override;

	std::tuple<cov_mat_t, type_value> getCovarianceAndMean() const override;

	/** Save the contents of the 3D grid in one file, as a concatenation of
	 * (X,Y) slices. The size in X,Y,and the values for Z,yaw, pitch, roll, PHI
	 * are stored in another file named `<filename>_dims.txt`
	 * \return false on error */
	bool saveToTextFile(const std::string& dataFile) const override;

	// See base docs
	void changeCoordinatesReference(const CPose3D& newReferenceBase) override;
	// See base docs. Not implemented in this class.
	void bayesianFusion(const CPose3DPDF& p1, const CPose3DPDF& p2) override;

	// See base docs. Not implemented in this class.
	void inverse(CPose3DPDF& o) const override;

	/** Draws a single sample from the distribution.
	 * \note Precondition: voxel weights are assumed to be normalized. */
	void drawSingleSample(CPose3D& outPart) const override;

	/** Draws a number of samples from the distribution, and saves as a list
	 * of 1x6 vectors, where each row contains a (x,y,z,yaw,pitch,roll) datum.
	 */
	void drawManySamples(
		size_t N,
		std::vector<mrpt::math::CVectorDouble>& outSamples) const override;

};  // End of class def.
}  // namespace mrpt::poses
