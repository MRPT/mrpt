/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CPosePDFParticles_H
#define CPosePDFParticles_H

#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPoseRandomSampler.h>
#include <mrpt/bayes/CParticleFilterCapable.h>
#include <mrpt/bayes/CParticleFilterData.h>

namespace mrpt
{
	namespace poses
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE( CPosePDFParticles , CPosePDF )

		/** Declares a class that represents a Probability Density Function (PDF) over a 2D pose (x,y,phi), using a set of weighted samples.
		 *
		 *  This class is also the base for the implementation of Monte-Carlo Localization (MCL), in mrpt::slam::CMonteCarloLocalization2D.
		 *
		 *  See the application "app/pf-localization" for an example of usage.
		 *
		 * \sa CPose2D, CPosePDF, CPoseGaussianPDF, CParticleFilterCapable
		 * \ingroup poses_pdf_grp
		 */
		class BASE_IMPEXP CPosePDFParticles :
			public CPosePDF,
			public mrpt::bayes::CParticleFilterData<CPose2D>,
			public mrpt::bayes::CParticleFilterDataImpl<CPosePDFParticles,mrpt::bayes::CParticleFilterData<CPose2D>::CParticleList>
		{
			// This must be added to any CSerializable derived class:
			DEFINE_SERIALIZABLE( CPosePDFParticles )

		public:
			void  clear(); //!< Free all the memory associated to m_particles, and set the number of parts = 0

			/** Constructor
			  * \param M The number of m_particles.
			  */
			CPosePDFParticles( size_t M = 1 );

			/** Copy operator, translating if necesary (for example, between m_particles and gaussian representations)
			  */
			void  copyFrom(const CPosePDF &o) MRPT_OVERRIDE;

			 /** Reset the PDF to a single point: All m_particles will be set exactly to the supplied pose.
			  * \param location The location to set all the m_particles.
			  * \param particlesCount If this is set to 0 the number of m_particles remains unchanged.
			  *  \sa resetUniform, resetUniformFreeSpace, resetAroundSetOfPoses
			  */
			void  resetDeterministic(
				const CPose2D &location,
				size_t particlesCount = 0);

			/** Reset the PDF to an uniformly distributed one, inside of the defined cube.
			  * If particlesCount is set to -1 the number of m_particles remains unchanged.
			  *  \sa resetDeterministic, resetUniformFreeSpace, resetAroundSetOfPoses
			  */
			void  resetUniform(
				const double & x_min,
				const double & x_max,
				const double & y_min,
				const double & y_max,
				const double & phi_min = -M_PI,
				const double & phi_max = M_PI,
				const int	&particlesCount = -1);

			/** Reset the PDF to a multimodal distribution over a set of "spots" (x,y,phi)
			  * The total number of particles will be `list_poses.size() * num_particles_per_pose`.
			  * \param[in] list_poses The poses (x,y,phi) around which particles will be spread. Must contains at least one pose.
			  * \param[in] num_particles_per_pose Number of particles to be spread around each of the "spots" in list_poses. Must be >=1.
			  * 
			  * Particles will be spread uniformly in a box of width `spread_{x,y,phi_rad}` in each of 
			  * the three coordinates (meters, radians), so it can be understood as the "initial uncertainty".
			  *
			  *  \sa resetDeterministic, resetUniformFreeSpace
			  */
			void  resetAroundSetOfPoses(
				const std::vector<mrpt::math::TPose2D> & list_poses,
				const size_t num_particles_per_pose,
				const double spread_x,
				const double spread_y,
				const double spread_phi_rad);

			 /** Returns an estimate of the pose, (the mean, or mathematical expectation of the PDF).
			   * \sa getCovariance
			   */
			void getMean(CPose2D &mean_pose) const MRPT_OVERRIDE;

			/** Returns an estimate of the pose covariance matrix (3x3 cov matrix) and the mean, both at once.
			  * \sa getMean
			  */
			void getCovarianceAndMean(mrpt::math::CMatrixDouble33 &cov,CPose2D &mean_point) const MRPT_OVERRIDE;

			/** Returns the pose of the i'th particle.
			  */
			CPose2D	 getParticlePose(size_t i) const;

			/** Save PDF's m_particles to a text file. In each line it will go: "x y phi weight"
			 */
			void  saveToTextFile(const std::string &file) const MRPT_OVERRIDE;

			/** Get the m_particles count (equivalent to "particlesCount")
			 */
			inline size_t  size() const { return m_particles.size(); }

			/** this = p (+) this. This can be used to convert a PDF from local coordinates to global, providing the point (newReferenceBase) from which
			  *   "to project" the current pdf. Result PDF substituted the currently stored one in the object.
			  */
			void  changeCoordinatesReference( const CPose3D &newReferenceBase ) MRPT_OVERRIDE;

			/** Draws a single sample from the distribution (WARNING: weights are assumed to be normalized!)
			  */
			void  drawSingleSample(CPose2D &outPart ) const MRPT_OVERRIDE;

			/** Appends (pose-composition) a given pose "p" to each particle
			  */
			void  operator += ( const CPose2D &Ap);

			/** Appends (add to the list) a set of m_particles to the existing ones, and then normalize weights.
			  */
			void  append( CPosePDFParticles &o );

			/** Returns a new PDF such as: NEW_PDF = (0,0,0) - THIS_PDF
			  */
			void	 inverse(CPosePDF &o) const MRPT_OVERRIDE;

			/** Returns the particle with the highest weight.
			  */
			CPose2D	 getMostLikelyParticle() const;

			/** Bayesian fusion.
			  */
			void  bayesianFusion( const CPosePDF &p1,const  CPosePDF &p2, const double & minMahalanobisDistToDrop = 0 ) MRPT_OVERRIDE;

			/** Evaluates the PDF at a given arbitrary point as reconstructed by a Parzen window.
			  * \sa saveParzenPDFToTextFile
			  */
			double  evaluatePDF_parzen(
				const double &	x,
				const double &	y,
				const double &	phi,
				const double &	stdXY,
				const double &	stdPhi ) const;

			/** Save a text file (compatible with matlab) representing the 2D evaluation of the PDF as reconstructed by a Parzen window.
			  * \sa evaluatePDF_parzen
			  */
			void  saveParzenPDFToTextFile(
				const char		*fileName,
				const double &			x_min,
				const double &			x_max,
				const double &			y_min,
				const double &			y_max,
				const double &			phi,
				const double &			stepSizeXY,
				const double &			stdXY,
				const double &			stdPhi ) const;

		}; // End of class def.
		DEFINE_SERIALIZABLE_POST_CUSTOM_BASE( CPosePDFParticles , CPosePDF )

	} // End of namespace
} // End of namespace

#endif
