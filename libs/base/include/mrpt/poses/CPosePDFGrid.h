/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */
#ifndef CPosePDFGrid_H
#define CPosePDFGrid_H

#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPose2DGridTemplate.h>

namespace mrpt
{
namespace poses
{
	using namespace mrpt::utils;

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
			double		resolutionPhi = DEG2RAD(180),
			double		phiMin = -M_PIf,
			double		phiMax = M_PIf
			);

		/** Destructor
		 */
		virtual ~CPosePDFGrid();

		/** Copy operator, translating if necesary (for example, between particles and gaussian representations)
		  */
		void  copyFrom(const CPosePDF &o);

		/** Normalizes the PDF, such as all cells sum the unity.
		  */
		void  normalize();

		/** Assigns the same value to all the cells in the grid, so the sum 1.
		  */
		void  uniformDistribution();

		 /** Returns an estimate of the pose, (the mean, or mathematical expectation of the PDF).
		   * \sa getCovariance
		   */
		void getMean(CPose2D &mean_pose) const;

		/** Returns an estimate of the pose covariance matrix (3x3 cov matrix) and the mean, both at once.
		  * \sa getMean
		  */
		void getCovarianceAndMean(CMatrixDouble33 &cov,CPose2D &mean_point) const;

		/** Save the contents of the 3D grid in one file, as a vertical concatenation of rectangular matrix for the different "PHI" discrete levels, and the size in X,Y,and PHI in another file named "<filename>_dims.txt"
		 */
		void   saveToTextFile(const std::string &dataFile) const;

		/** this = p (+) this. This can be used to convert a PDF from local coordinates to global, providing the point (newReferenceBase) from which
		  *   "to project" the current pdf. Result PDF substituted the currently stored one in the object.
		  */
		void  changeCoordinatesReference( const CPose3D &newReferenceBase );

		/** Bayesian fusion of 2 densities (In the grid representation this becomes a pointwise multiplication)
		  */
		void  bayesianFusion(const  CPosePDF &p1,const  CPosePDF &p2, const double &minMahalanobisDistToDrop = 0 );

		/** Returns a new PDF such as: NEW_PDF = (0,0,0) - THIS_PDF
		  */
		void  inverse(CPosePDF &o) const;

		/** Draws a single sample from the distribution (WARNING: weights are assumed to be normalized!)
		  */
		void  drawSingleSample( CPose2D &outPart ) const;

		/** Draws a number of samples from the distribution, and saves as a list of 1x3 vectors, where each row contains a (x,y,phi) datum.
		  */
		void  drawManySamples( size_t N, std::vector<vector_double> & outSamples ) const;

	}; // End of class def.


	} // End of namespace
} // End of namespace

#endif
