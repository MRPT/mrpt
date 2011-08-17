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
#ifndef CMonteCarloLocalization3D_H
#define CMonteCarloLocalization3D_H

#include <mrpt/poses/CPose3DPDFParticles.h>
#include <mrpt/slam/PF_implementations_data.h>
#include <mrpt/slam/TMonteCarloLocalizationParams.h>

#include <mrpt/slam/link_pragmas.h>

namespace mrpt
{
	namespace slam
	{
		class CSensoryFrame;

		using namespace mrpt::poses;
		using namespace mrpt::slam;
		using namespace mrpt::bayes;

		/** Declares a class that represents a Probability Density Function (PDF) over a 3D pose (x,y,phi,yaw,pitch,roll), using a set of weighted samples.
		 *
		 *  This class also implements particle filtering for robot localization. See the MRPT
		 *   application "app/pf-localization" for an example of usage.
		 *
		 * \sa CMonteCarloLocalization2D, CPose2D, CPosePDF, CPoseGaussianPDF, CParticleFilterCapable
		 * \ingroup mrpt_slam_grp 
		 */
		class SLAM_IMPEXP CMonteCarloLocalization3D :
			public CPose3DPDFParticles,
			public PF_implementation<CPose3D,CMonteCarloLocalization3D>
		{
			//template <class PARTICLE_TYPE, class MYSELF> friend class PF_implementation;

		public:
			TMonteCarloLocalizationParams	options; //!< MCL parameters

			/** Constructor
			  * \param M The number of m_particles.
			  */
			CMonteCarloLocalization3D( size_t M = 1 );

			/** Destructor */
			virtual ~CMonteCarloLocalization3D();

			 /** Update the m_particles, predicting the posterior of robot pose and map after a movement command.
			  *  This method has additional configuration parameters in "options".
			  *  Performs the update stage of the RBPF, using the sensed CSensoryFrame:
			  *
			  *   \param action This is a pointer to CActionCollection, containing the pose change the robot has been commanded.
			  *   \param observation This must be a pointer to a CSensoryFrame object, with robot sensed observations.
			  *
			  * \sa options
			  */
			void  prediction_and_update_pfStandardProposal(
				const mrpt::slam::CActionCollection	* action,
				const mrpt::slam::CSensoryFrame		* observation,
				const bayes::CParticleFilter::TParticleFilterOptions &PF_options );

			 /** Update the m_particles, predicting the posterior of robot pose and map after a movement command.
			  *  This method has additional configuration parameters in "options".
			  *  Performs the update stage of the RBPF, using the sensed CSensoryFrame:
			  *
			  *   \param Action This is a pointer to CActionCollection, containing the pose change the robot has been commanded.
			  *   \param observation This must be a pointer to a CSensoryFrame object, with robot sensed observations.
			  *
			  * \sa options
			  */
			void  prediction_and_update_pfAuxiliaryPFStandard(
				const mrpt::slam::CActionCollection	* action,
				const mrpt::slam::CSensoryFrame		* observation,
				const bayes::CParticleFilter::TParticleFilterOptions &PF_options );

			 /** Update the m_particles, predicting the posterior of robot pose and map after a movement command.
			  *  This method has additional configuration parameters in "options".
			  *  Performs the update stage of the RBPF, using the sensed CSensoryFrame:
			  *
			  *   \param Action This is a pointer to CActionCollection, containing the pose change the robot has been commanded.
			  *   \param observation This must be a pointer to a CSensoryFrame object, with robot sensed observations.
			  *
			  * \sa options
			  */
			void  prediction_and_update_pfAuxiliaryPFOptimal(
				const mrpt::slam::CActionCollection	* action,
				const mrpt::slam::CSensoryFrame		* observation,
				const bayes::CParticleFilter::TParticleFilterOptions &PF_options );

		//protected:
			/** \name Virtual methods that the PF_implementations assume exist.
			    @{ */
			/** Return a pointer to the last robot pose in the i'th particle (or NULL if it's a path and it's empty). */
			const TPose3D * getLastPose(const size_t i) const;

			void PF_SLAM_implementation_custom_update_particle_with_new_pose(
				CParticleDataContent *particleData,
				const TPose3D &newPose) const;

			// We'll redefine this one:
			void PF_SLAM_implementation_replaceByNewParticleSet(
				CParticleList	&old_particles,
				const std::vector<TPose3D>	&newParticles,
				const vector_double		&newParticlesWeight,
				const std::vector<size_t>	&newParticlesDerivedFromIdx )  const;

			/** Evaluate the observation likelihood for one particle at a given location */
			double PF_SLAM_computeObservationLikelihoodForParticle(
				const CParticleFilter::TParticleFilterOptions	&PF_options,
				const size_t			particleIndexForMap,
				const CSensoryFrame		&observation,
				const CPose3D			&x ) const;
			/** @} */


		}; // End of class def.

	} // End of namespace
} // End of namespace

#endif
