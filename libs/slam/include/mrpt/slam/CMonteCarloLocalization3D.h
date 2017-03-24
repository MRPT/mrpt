/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CMonteCarloLocalization3D_H
#define CMonteCarloLocalization3D_H

#include <mrpt/poses/CPose3DPDFParticles.h>
#include <mrpt/slam/PF_implementations_data.h>
#include <mrpt/slam/TMonteCarloLocalizationParams.h>
#include <mrpt/obs/obs_frwds.h>

#include <mrpt/slam/link_pragmas.h>

namespace mrpt
{
	namespace slam
	{
		/** Declares a class that represents a Probability Density Function (PDF) over a 3D pose (x,y,phi,yaw,pitch,roll), using a set of weighted samples.
		 *
		 *  This class also implements particle filtering for robot localization. See the MRPT
		 *   application "app/pf-localization" for an example of usage.
		 *
		 * \sa CMonteCarloLocalization2D, CPose2D, CPosePDF, CPoseGaussianPDF, CParticleFilterCapable
		 * \ingroup mrpt_slam_grp
		 */
		class SLAM_IMPEXP CMonteCarloLocalization3D :
			public mrpt::poses::CPose3DPDFParticles,
			public PF_implementation<mrpt::poses::CPose3D,CMonteCarloLocalization3D>
		{
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
				const mrpt::obs::CActionCollection	* action,
				const mrpt::obs::CSensoryFrame		* observation,
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
				const mrpt::obs::CActionCollection	* action,
				const mrpt::obs::CSensoryFrame		* observation,
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
				const mrpt::obs::CActionCollection	* action,
				const mrpt::obs::CSensoryFrame		* observation,
				const bayes::CParticleFilter::TParticleFilterOptions &PF_options );

			/** \name Virtual methods that the PF_implementations assume exist.
			    @{ */
			/** Return a pointer to the last robot pose in the i'th particle (or NULL if it's a path and it's empty). */
			const mrpt::math::TPose3D * getLastPose(const size_t i) const;

			void PF_SLAM_implementation_custom_update_particle_with_new_pose(
				CParticleDataContent *particleData,
				const mrpt::math::TPose3D &newPose) const;

			// We'll redefine this one:
			void PF_SLAM_implementation_replaceByNewParticleSet(
				CParticleList	&old_particles,
				const std::vector<mrpt::math::TPose3D>	&newParticles,
				const std::vector<double>		&newParticlesWeight,
				const std::vector<size_t>	&newParticlesDerivedFromIdx )  const;

			/** Evaluate the observation likelihood for one particle at a given location */
			double PF_SLAM_computeObservationLikelihoodForParticle(
				const mrpt::bayes::CParticleFilter::TParticleFilterOptions	&PF_options,
				const size_t			particleIndexForMap,
				const mrpt::obs::CSensoryFrame		&observation,
				const mrpt::poses::CPose3D			&x ) const;
			/** @} */


		}; // End of class def.

	} // End of namespace
} // End of namespace

#endif
