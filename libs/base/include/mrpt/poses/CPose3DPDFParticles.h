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
		using namespace mrpt::bayes;
		using namespace mrpt::utils;

		/** \typedef CProbabilityParticle<CPose3D> CPose3DParticle
		 *  A type definition for m_particles containing a 3D pose.
		 */
		typedef CProbabilityParticle<CPose3D> CPose3DParticle;

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
			public mrpt::bayes::CParticleFilterCapable
		{
			// This must be added to any CSerializable derived class:
			DEFINE_SERIALIZABLE( CPose3DPDFParticles )

			// This uses CParticleFilterData to implement some methods required for CParticleFilterCapable:
			IMPLEMENT_PARTICLE_FILTER_CAPABLE(CPose3D)

		 public:
			/** Constructor
			  * \param M The number of m_particles.
			  */
			CPose3DPDFParticles( size_t M = 1 );

			/** Copy constructor:
			  */
			inline CPose3DPDFParticles( const CPose3DPDFParticles& obj ) :
				CPose3DPDF(),
				CParticleFilterData<CPose3D>()
			{
				copyFrom( obj );
			}

			/** Destructor
			 */
			virtual ~CPose3DPDFParticles();


			/** Copy operator, translating if necesary (for example, between m_particles and gaussian representations)
			  */
			void  copyFrom(const CPose3DPDF &o);

			/** Reset the PDF to a single point: All m_particles will be set exactly to the supplied pose.
			  * \param location The location to set all the m_particles.
			  * \param particlesCount If this is set to 0 the number of m_particles remains unchanged.
			  *  \sa resetUniform, resetUniformFreeSpace
			  */
			void  resetDeterministic( const CPose3D &location,
												size_t	particlesCount = 0);

			 /** Returns an estimate of the pose, (the mean, or mathematical expectation of the PDF), computed as a weighted average over all m_particles.
			   * \sa getCovariance
			   */
			void getMean(CPose3D &mean_pose) const;

			/** Returns an estimate of the pose covariance matrix (6x6 cov matrix) and the mean, both at once.
			  * \sa getMean
			  */
			void getCovarianceAndMean(CMatrixDouble66 &cov,CPose3D &mean_point) const;

			/** Returns the pose of the i'th particle.
			  */
			CPose3D	 getParticlePose(int i) const;

			/** Save PDF's m_particles to a text file. In each line it will go: "x y z"
			 */
			void  saveToTextFile(const std::string &file) const;

			/** Get the m_particles count (equivalent to "particlesCount")
			 */
			size_t  size() const { return m_particles.size(); }

			/** This can be used to convert a PDF from local coordinates to global, providing the point (newReferenceBase) from which
			  *   "to project" the current pdf. Result PDF substituted the currently stored one in the object.
			  */
			void  changeCoordinatesReference( const CPose3D &newReferenceBase );

			/** Draws a single sample from the distribution (WARNING: weights are assumed to be normalized!)
			  */
			void  drawSingleSample( CPose3D &outPart ) const;

			/** Draws a number of samples from the distribution, and saves as a list of 1x6 vectors, where each row contains a (x,y,phi) datum.
			  */
			void  drawManySamples( size_t N, std::vector<vector_double> & outSamples ) const;

			/** Appends (pose-composition) a given pose "p" to each particle
			  */
			void  operator += ( const CPose3D &Ap);

			/** Appends (add to the list) a set of m_particles to the existing ones, and then normalize weights.
			  */
			void append( CPose3DPDFParticles &o );

			/** Returns a new PDF such as: NEW_PDF = (0,0,0) - THIS_PDF
			  */
			void inverse(CPose3DPDF &o) const;

			/** Returns the particle with the highest weight.
			  */
			CPose3D	 getMostLikelyParticle() const;

			/** Bayesian fusion.
			  */
			void  bayesianFusion( const CPose3DPDF &p1, const CPose3DPDF &p2 );

		}; // End of class def.


	} // End of namespace
} // End of namespace

#endif
