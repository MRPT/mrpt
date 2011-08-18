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

		/** This can be used to convert a PDF from local coordinates to global, providing the point (newReferenceBase) from which
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
