/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CPosePDFGrid_H
#define CPosePDFGrid_H

#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPose2DGridTemplate.h>
#include <mrpt/utils/bits.h> // DEG2RAD()

namespace mrpt
{
namespace poses
{
	// This must be added to any CSerializable derived class:
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE( CPosePDFGrid, CPosePDF   )

	/** Declares a class that represents a Probability Distribution
	 *    function (PDF) of a 2D pose (x,y,phi).
	 *   This class implements that PDF using a 3D grid.
	 *
	 * \sa CPose2D, CPosePDF, CPose2DGridTemplate
	 * \ingroup poses_pdf_grp
	 */
	class BASE_IMPEXP CPosePDFGrid : public CPosePDF, public CPose2DGridTemplate<double>
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CPosePDFGrid )

	 protected:


	 public:
		/** Constructor: Initializes a, uniform distribution over the whole given range.
		  */
		CPosePDFGrid(
			double		xMin = -1.0f,
			double		xMax = 1.0f,
			double		yMin = -1.0f,
			double		yMax = 1.0f,
			double		resolutionXY = 0.5f,
			double		resolutionPhi = mrpt::utils::DEG2RAD(180),
			double		phiMin = -M_PIf,
			double		phiMax = M_PIf
			);

		virtual ~CPosePDFGrid(); //!< Destructor

		void copyFrom(const CPosePDF &o) MRPT_OVERRIDE; //!< Copy operator, translating if necesary (for example, between particles and gaussian representations)

		void normalize(); //!< Normalizes the PDF, such as all cells sum the unity.
		void uniformDistribution(); //!< Assigns the same value to all the cells in the grid, so the sum 1.
		void getMean(CPose2D &mean_pose) const MRPT_OVERRIDE; //!< Returns an estimate of the pose, (the mean, or mathematical expectation of the PDF). \sa getCovariance
		void getCovarianceAndMean(mrpt::math::CMatrixDouble33 &cov,CPose2D &mean_point) const MRPT_OVERRIDE; //!< Returns an estimate of the pose covariance matrix (3x3 cov matrix) and the mean, both at once. \sa getMean
		void saveToTextFile(const std::string &dataFile) const MRPT_OVERRIDE; //!< Save the contents of the 3D grid in one file, as a vertical concatenation of rectangular matrix for the different "PHI" discrete levels, and the size in X,Y,and PHI in another file named "<filename>_dims.txt"

		/** this = p (+) this. This can be used to convert a PDF from local coordinates to global, providing the point (newReferenceBase) from which
		  *   "to project" the current pdf. Result PDF substituted the currently stored one in the object. */
		void  changeCoordinatesReference( const CPose3D &newReferenceBase ) MRPT_OVERRIDE;
		void bayesianFusion(const  CPosePDF &p1,const  CPosePDF &p2, const double &minMahalanobisDistToDrop = 0 ) MRPT_OVERRIDE; //!< Bayesian fusion of 2 densities (In the grid representation this becomes a pointwise multiplication)
		void inverse(CPosePDF &o) const MRPT_OVERRIDE; //!< Returns a new PDF such as: NEW_PDF = (0,0,0) - THIS_PDF
		void drawSingleSample( CPose2D &outPart ) const MRPT_OVERRIDE; //!< Draws a single sample from the distribution (WARNING: weights are assumed to be normalized!)
		void drawManySamples( size_t N, std::vector<mrpt::math::CVectorDouble> & outSamples ) const MRPT_OVERRIDE; //!< Draws a number of samples from the distribution, and saves as a list of 1x3 vectors, where each row contains a (x,y,phi) datum.

	}; // End of class def.
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE( CPosePDFGrid, CPosePDF   )
	} // End of namespace
} // End of namespace
#endif
