/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CPose3DPDFSOG_H
#define CPose3DPDFSOG_H

#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/utils/aligned_containers.h>

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

		void getMean(CPose3D &mean_pose) const MRPT_OVERRIDE; //!< Returns an estimate of the pose, (the mean, or mathematical expectation of the PDF), computed as a weighted average over all m_particles. \sa getCovariance
		void getCovarianceAndMean(mrpt::math::CMatrixDouble66 &cov,CPose3D &mean_point) const MRPT_OVERRIDE; //!< Returns an estimate of the pose covariance matrix (6x6 cov matrix) and the mean, both at once. \sa getMean
		void normalizeWeights(); //!< Normalize the weights in m_modes such as the maximum log-weight is 0.
		void getMostLikelyMode( CPose3DPDFGaussian& outVal ) const; //!< Return the Gaussian mode with the highest likelihood (or an empty Gaussian if there are no modes in this SOG)

		void copyFrom(const CPose3DPDF &o) MRPT_OVERRIDE; //!< Copy operator, translating if necesary (for example, between particles and gaussian representations)

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
		void saveToTextFile(const std::string &file) const MRPT_OVERRIDE;

		/** this = p (+) this. This can be used to convert a PDF from local coordinates to global, providing the point (newReferenceBase) from which
		  *   "to project" the current pdf. Result PDF substituted the currently stored one in the object. */
		void changeCoordinatesReference(const CPose3D &newReferenceBase ) MRPT_OVERRIDE;

		/** Bayesian fusion of two pose distributions, then save the result in this object (WARNING: Currently p1 must be a mrpt::poses::CPose3DPDFSOG object and p2 a mrpt::poses::CPose3DPDFSOG object) */
		void bayesianFusion( const CPose3DPDF &p1,const  CPose3DPDF &p2 ) MRPT_OVERRIDE;

		void drawSingleSample( CPose3D &outPart ) const MRPT_OVERRIDE; //!< Draws a single sample from the distribution
		void drawManySamples( size_t N, std::vector<mrpt::math::CVectorDouble> & outSamples ) const MRPT_OVERRIDE; //!< Draws a number of samples from the distribution, and saves as a list of 1x6 vectors, where each row contains a (x,y,z,yaw,pitch,roll) datum
		void inverse(CPose3DPDF &o) const MRPT_OVERRIDE; //!< Returns a new PDF such as: NEW_PDF = (0,0,0) - THIS_PDF

		void appendFrom( const CPose3DPDFSOG &o ); //!< Append the Gaussian modes from "o" to the current set of modes of "this" density

	}; // End of class def.
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE( CPose3DPDFSOG, CPose3DPDF )
	} // End of namespace
} // End of namespace
#endif
