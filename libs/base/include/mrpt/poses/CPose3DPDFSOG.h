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
#ifndef CPose3DPDFSOG_H
#define CPose3DPDFSOG_H

#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/math/CMatrix.h>

namespace mrpt
{
namespace poses
{
	// This must be added to any CSerializable derived class:
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE( CPose3DPDFSOG, CPose3DPDF )

	/** Declares a class that represents a Probability Density function (PDF) of a 3D(6D) pose \f$ p(\mathbf{x}) = [x ~ y ~ z ~ yaw ~ pitch ~ roll]^t \f$.
	 *   This class implements that PDF as the following multi-modal Gaussian distribution:
	 *
	 * \f$ p(\mathbf{x}) = \sum\limits_{i=1}^N \omega^i \mathcal{N}( \mathbf{x} ; \bar{\mathbf{x}}^i, \mathbf{\Sigma}^i )  \f$
	 *
	 *  Where the number of modes N is the size of CPose3DPDFSOG::m_modes. Angles are always in radians.
	 *
	 *  See mrpt::poses::CPose3DPDF for more details.
	 * \ingroup poses_pdf_grp
	 * \sa CPose3DPDF
	 */
	class BASE_IMPEXP CPose3DPDFSOG : public CPose3DPDF
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CPose3DPDFSOG )

	public:
		/** The struct for each mode:
		 */
		struct BASE_IMPEXP TGaussianMode
		{
			TGaussianMode() :
				val(),
				log_w(0)
			{ }

			CPose3DPDFGaussian		val;

			/** The log-weight
			  */
			double		log_w;
		};

		typedef mrpt::aligned_containers<TGaussianMode>::vector_t  TModesList;
		typedef TModesList::const_iterator const_iterator;
		typedef TModesList::iterator iterator;

	protected:
		/** Assures the symmetry of the covariance matrix (eventually certain operations in the math-coprocessor lead to non-symmetric matrixes!)
		  */
		void  assureSymmetry();

		/** Access directly to this array for modify the modes as desired.
		  *  Note that no weight can be zero!!
		  *  We must use pointers to satisfy the mem-alignment of the matrixes
		  */
		TModesList   m_modes;

	 public:
		/** Default constructor
		  * \param nModes The initial size of CPose3DPDFSOG::m_modes
		  */
		CPose3DPDFSOG( size_t nModes = 1 );

		void clear(); //!< Clear all the gaussian modes
		void resize(const size_t N); //!< Set the number of SOG modes
		size_t size() const { return m_modes.size(); } //!< Return the number of Gaussian modes.
		bool empty() const { return m_modes.empty(); } //!< Return whether there is any Gaussian mode.

		iterator begin() { return m_modes.begin(); }
		iterator end() { return m_modes.end(); }
		const_iterator begin() const { return m_modes.begin(); }
		const_iterator end()const { return m_modes.end(); }

		 /** Returns an estimate of the pose, (the mean, or mathematical expectation of the PDF), computed as a weighted average over all m_particles.
		   * \sa getCovariance
		   */
		void getMean(CPose3D &mean_pose) const;

		/** Returns an estimate of the pose covariance matrix (6x6 cov matrix) and the mean, both at once.
		  * \sa getMean
		  */
		void getCovarianceAndMean(CMatrixDouble66 &cov,CPose3D &mean_point) const;

		/** Normalize the weights in m_modes such as the maximum log-weight is 0.
		  */
		void  normalizeWeights();

		/** Return the Gaussian mode with the highest likelihood (or an empty Gaussian if there are no modes in this SOG) */
		void getMostLikelyMode( CPose3DPDFGaussian& outVal ) const;

		/** Copy operator, translating if necesary (for example, between particles and gaussian representations)
		  */
		void  copyFrom(const CPose3DPDF &o);

		/** Save the density to a text file, with the following format:
		  *  There is one row per Gaussian "mode", and each row contains 10 elements:
		  *   - w (The linear weight)
		  *   - x_mean (gaussian mean value)
		  *   - y_mean (gaussian mean value)
		  *   - x_mean (gaussian mean value)
		  *   - yaw_mean (gaussian mean value, in radians)
		  *   - pitch_mean (gaussian mean value, in radians)
		  *   - roll_mean (gaussian mean value, in radians)
		  *   - C11,C22,C33,C44,C55,C66 (Covariance elements)
		  *   - C12,C13,C14,C15,C16 (Covariance elements)
		  *   - C23,C24,C25,C25 (Covariance elements)
		  *   - C34,C35,C36 (Covariance elements)
		  *   - C45,C46 (Covariance elements)
		  *   - C56 (Covariance elements)
		  *
		 */
		void  saveToTextFile(const std::string &file) const;

		/** This can be used to convert a PDF from local coordinates to global, providing the point (newReferenceBase) from which
		  *   "to project" the current pdf. Result PDF substituted the currently stored one in the object.
		  */
		void  changeCoordinatesReference(const CPose3D &newReferenceBase );

		/** Bayesian fusion of two pose distributions, then save the result in this object (WARNING: Currently p1 must be a mrpt::poses::CPose3DPDFSOG object and p2 a mrpt::poses::CPose3DPDFSOG object)
		  */
		void  bayesianFusion( const CPose3DPDF &p1,const  CPose3DPDF &p2 );

		/** Draws a single sample from the distribution
		  */
		void  drawSingleSample( CPose3D &outPart ) const;

		/** Draws a number of samples from the distribution, and saves as a list of 1x6 vectors, where each row contains a (x,y,z,yaw,pitch,roll) datum.
		  */
		void  drawManySamples( size_t N, std::vector<vector_double> & outSamples ) const;

		/** Returns a new PDF such as: NEW_PDF = (0,0,0) - THIS_PDF
		  */
		void  inverse(CPose3DPDF &o) const;

		/** Append the Gaussian modes from "o" to the current set of modes of "this" density.
		  */
		void appendFrom( const CPose3DPDFSOG &o );

	}; // End of class def.


	} // End of namespace
} // End of namespace

#endif
