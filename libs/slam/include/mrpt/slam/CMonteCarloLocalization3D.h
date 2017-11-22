/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#ifndef CMonteCarloLocalization3D_H
#define CMonteCarloLocalization3D_H

#include <mrpt/poses/CPose3DPDFParticles.h>
#include <mrpt/slam/PF_implementations_data.h>
#include <mrpt/slam/TMonteCarloLocalizationParams.h>
#include <mrpt/obs/obs_frwds.h>

namespace mrpt
{
namespace slam
{
/** Declares a class that represents a Probability Density Function (PDF) over a
 * 3D pose (x,y,phi,yaw,pitch,roll), using a set of weighted samples.
 *
 * This class also implements particle filtering for robot localization. See
 * the MRPT application "app/pf-localization" for an example of usage.
 *
 * \sa CMonteCarloLocalization2D, CPose2D, CPosePDF, CPoseGaussianPDF,
 * \ingroup mrpt_slam_grp
 */
class CMonteCarloLocalization3D
	: public PF_implementation<
		  mrpt::poses::CPose3D, mrpt::poses::CPose3DPDFParticles>
{
   public:
	using CParticleDataContent =
		mrpt::poses::CPose3DPDFParticles::CParticleDataContent;
	using CParticleList = mrpt::poses::CPose3DPDFParticles::CParticleList;

	mrpt::poses::CPose3DPDFParticles m_poseParticles;

	/** MCL parameters */
	TMonteCarloLocalizationParams options;

	/** Constructor
	 * \param M The number of m_particles.
	 */
	CMonteCarloLocalization3D(size_t M = 1);

	/** Destructor */
	virtual ~CMonteCarloLocalization3D();

	/** Update the m_particles, predicting the posterior of robot pose and map
	 * after a movement command.
	 *  This method has additional configuration parameters in "options".
	 *  Performs the update stage of the RBPF, using the sensed CSensoryFrame:
	 *
	 *   \param action This is a pointer to CActionCollection, containing the
	 * pose change the robot has been commanded.
	 *   \param observation This must be a pointer to a CSensoryFrame object,
	 * with robot sensed observations.
	 *
	 * \sa options
	 */
	template <typename T>
	void prediction_and_update(
		const mrpt::obs::CActionCollection* action,
		const mrpt::obs::CSensoryFrame* observation,
		const bayes::CParticleFilter::TParticleFilterOptions& PF_options);

	/** \name Virtual methods that the PF_implementations assume exist.
		@{ */
	/** Return the robot pose for the i'th particle. is_valid is
	 * always true in this class. */
	mrpt::math::TPose3D getLastPose(
		const size_t i, bool& is_valid_pose) const override;

	void PF_SLAM_implementation_custom_update_particle_with_new_pose(
		CParticleDataContent* particleData,
		const mrpt::math::TPose3D& newPose) const;

	// We'll redefine this one:
	void PF_SLAM_implementation_replaceByNewParticleSet(
		CParticleList& old_particles,
		const std::vector<mrpt::math::TPose3D>& newParticles,
		const std::vector<double>& newParticlesWeight,
		const std::vector<size_t>& newParticlesDerivedFromIdx) const;

	/** Evaluate the observation likelihood for one particle at a given location
	 */
	double PF_SLAM_computeObservationLikelihoodForParticle(
		const mrpt::bayes::CParticleFilter::TParticleFilterOptions& PF_options,
		const size_t particleIndexForMap,
		const mrpt::obs::CSensoryFrame& observation,
		const mrpt::poses::CPose3D& x) const;
	/** @} */

	void executeOn(
		mrpt::bayes::CParticleFilter& pf,
		const mrpt::obs::CActionCollection* action,
		const mrpt::obs::CSensoryFrame* observation,
		mrpt::bayes::CParticleFilter::TParticleFilterStats* stats,
		mrpt::bayes::CParticleFilter::TParticleFilterAlgorithm PF_algorithm);

};  // End of class def.

}  // namespace slam
}  // namespace mrpt

#endif
