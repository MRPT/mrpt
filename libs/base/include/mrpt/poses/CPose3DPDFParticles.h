/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CPose3DPDFParticles_H
#define CPose3DPDFParticles_H

#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/bayes/CProbabilityParticle.h>
#include <mrpt/bayes/CParticleFilterCapable.h>
#include <mrpt/bayes/CParticleFilterData.h>

namespace mrpt
{
	namespace poses
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE(CPose3DPDFParticles,CPose3DPDF)

		/** Declares a class that represents a Probability Density function (PDF) of a 3D pose
		 *
		 *  This class is also the base for the implementation of Monte-Carlo Localization (MCL), in mrpt::slam::CMonteCarloLocalization2D.
		 *
		 *  See the application "app/pf-localization" for an example of usage.
		 *
		 * \sa CPose3D, CPose3DPDF, CPoseGaussianPDF
		 * \ingroup poses_pdf_grp
		 */
		class BASE_IMPEXP CPose3DPDFParticles :
			public CPose3DPDF,
			public mrpt::bayes::CParticleFilterData<CPose3D>,
			public mrpt::bayes::CParticleFilterDataImpl<CPose3DPDFParticles,mrpt::bayes::CParticleFilterData<CPose3D>::CParticleList>
		{
			// This must be added to any CSerializable derived class:
			DEFINE_SERIALIZABLE( CPose3DPDFParticles )

		 public:
			/** Constructor
			  * \param M The number of m_particles.
			  */
			CPose3DPDFParticles( size_t M = 1 );

			void copyFrom(const CPose3DPDF &o) MRPT_OVERRIDE; //!< Copy operator, translating if necesary (for example, between m_particles and gaussian representations)

			/** Reset the PDF to a single point: All m_particles will be set exactly to the supplied pose.
			  * \param location The location to set all the m_particles.
			  * \param particlesCount If this is set to 0 the number of m_particles remains unchanged.
			  *  \sa resetUniform, resetUniformFreeSpace */
			void  resetDeterministic( const CPose3D &location, size_t particlesCount = 0);

			void getMean(CPose3D &mean_pose) const MRPT_OVERRIDE; //!< Returns an estimate of the pose, (the mean, or mathematical expectation of the PDF), computed as a weighted average over all m_particles. \sa getCovariance
			void getCovarianceAndMean(mrpt::math::CMatrixDouble66 &cov,CPose3D &mean_point) const MRPT_OVERRIDE; //!< Returns an estimate of the pose covariance matrix (6x6 cov matrix) and the mean, both at once. \sa getMean

			CPose3D	 getParticlePose(int i) const; //!< Returns the pose of the i'th particle

			void saveToTextFile(const std::string &file) const MRPT_OVERRIDE; //!< Save PDF's m_particles to a text file. In each line it will go: "x y z"

			/** Get the m_particles count (equivalent to "particlesCount") */
			size_t  size() const { return m_particles.size(); }

			/** this = p (+) this. This can be used to convert a PDF from local coordinates to global, providing the point (newReferenceBase) from which
			  *   "to project" the current pdf. Result PDF substituted the currently stored one in the object. */
			void changeCoordinatesReference( const CPose3D &newReferenceBase ) MRPT_OVERRIDE;
			void drawSingleSample( CPose3D &outPart ) const MRPT_OVERRIDE; //!< Draws a single sample from the distribution (WARNING: weights are assumed to be normalized!)
			void drawManySamples( size_t N, std::vector<mrpt::math::CVectorDouble> & outSamples ) const MRPT_OVERRIDE; //!< Draws a number of samples from the distribution, and saves as a list of 1x6 vectors, where each row contains a (x,y,phi) datum.
			void operator += ( const CPose3D &Ap); //!< Appends (pose-composition) a given pose "p" to each particle
			void append( CPose3DPDFParticles &o ); //!< Appends (add to the list) a set of m_particles to the existing ones, and then normalize weights.
			void inverse(CPose3DPDF &o) const MRPT_OVERRIDE; //!< Returns a new PDF such as: NEW_PDF = (0,0,0) - THIS_PDF
			CPose3D getMostLikelyParticle() const;  //!< Returns the particle with the highest weight.
			void  bayesianFusion( const CPose3DPDF &p1, const CPose3DPDF &p2 ) MRPT_OVERRIDE; //!< Bayesian fusion

		}; // End of class def.
		DEFINE_SERIALIZABLE_POST_CUSTOM_BASE(CPose3DPDFParticles,CPose3DPDF)
	} // End of namespace
} // End of namespace
#endif
