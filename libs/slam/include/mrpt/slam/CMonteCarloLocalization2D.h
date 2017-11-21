/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#ifndef CMonteCarloLocalization2D_H
#define CMonteCarloLocalization2D_H

#include <mrpt/poses/CPosePDFParticles.h>
#include <mrpt/slam/PF_implementations_data.h>
#include <mrpt/slam/TMonteCarloLocalizationParams.h>
#include <mrpt/obs/obs_frwds.h>

#include <mrpt/slam/PF_aux_structs.h>

namespace mrpt
{
namespace maps
{
class COccupancyGridMap2D;
}

/** \ingroup mrpt_slam_grp */
namespace slam
{
/** Declares a class that represents a Probability Density Function (PDF) over a
 * 2D pose (x,y,phi), using a set of weighted samples.
 *
 *  This class also implements particle filtering for robot localization. See
 * the MRPT
 *   application "app/pf-localization" for an example of usage.
 *
 * \sa CMonteCarloLocalization3D, CPose2D, CPosePDF, CPoseGaussianPDF,
 * \ingroup mrpt_slam_grp
 */
class CMonteCarloLocalization2D
	: public PF_implementation<
		  mrpt::poses::CPose2D, mrpt::poses::CPosePDFParticles>
{
   public:
	using CParticleDataContent =
		mrpt::poses::CPosePDFParticles::CParticleDataContent;
	using CParticleList = mrpt::poses::CPosePDFParticles::CParticleList;

	mrpt::poses::CPosePDFParticles m_poseParticles;

	/** MCL parameters */
	TMonteCarloLocalizationParams options;

	/** Constructor
	 * \param M The number of m_particles.
	 */
	CMonteCarloLocalization2D(size_t M = 1);

	/** Destructor */
	virtual ~CMonteCarloLocalization2D();

	/** Reset the PDF to an uniformly distributed one, but only in the
	 * free-space
	 *   of a given 2D occupancy-grid-map. Orientation is randomly generated in
	 * the whole 2*PI range.
	 * \param theMap The occupancy grid map
	 * \param freeCellsThreshold The minimum free-probability to consider a
	 * cell as empty (default is 0.7)
	 * \param particlesCount If set to -1 the number of m_particles remains
	 * unchanged.
	 * \param x_min The limits of the area to look for free cells.
	 * \param x_max The limits of the area to look for free cells.
	 * \param y_min The limits of the area to look for free cells.
	 * \param y_max The limits of the area to look for free cells.
	 * \param phi_min The limits of the area to look for free cells.
	 * \param phi_max The limits of the area to look for free cells.
	 *  \sa resetDeterm32inistic
	 * \exception std::exception On any error (no free cell found in map,
	 * map=nullptr, etc...)
	 */
	void resetUniformFreeSpace(
		mrpt::maps::COccupancyGridMap2D* theMap,
		const double freeCellsThreshold = 0.7, const int particlesCount = -1,
		const double x_min = -1e10f, const double x_max = 1e10f,
		const double y_min = -1e10f, const double y_max = 1e10f,
		const double phi_min = -M_PI, const double phi_max = M_PI);

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
	template <class T>
	void prediction_and_update(
		const mrpt::obs::CActionCollection* actions,
		const mrpt::obs::CSensoryFrame* observation,
		const bayes::CParticleFilter::TParticleFilterOptions& PF_options)
	{
		MRPT_START

		if (observation)
		{  // A map MUST be supplied!
			ASSERT_(options.metricMap || options.metricMaps.size() > 0)
			if (!options.metricMap)
				ASSERT_(
					options.metricMaps.size() ==
					m_poseParticles.m_particles.size())
		}

		T::template PF_SLAM_implementation<
			mrpt::poses::CPose2D, CMonteCarloLocalization2D,
			mrpt::slam::detail::TPoseBin2D>(
			actions, observation, PF_options, options.KLD_params, *this);

		MRPT_END
	}
	// protected:
	/** \name Virtual methods that the PF_implementations assume exist.
		@{ */
	/** Return the robot pose for the i'th particle. is_valid is
	 * always true in this class. */
	mrpt::math::TPose3D getLastPose(
		const size_t i, bool& is_valid_pose) const override;

	void PF_SLAM_implementation_custom_update_particle_with_new_pose(
		CParticleDataContent* particleData,
		const mrpt::math::TPose3D& newPose) const override;

	// We'll redefine this one:
	void PF_SLAM_implementation_replaceByNewParticleSet(
		CParticleList& old_particles,
		const std::vector<mrpt::math::TPose3D>& newParticles,
		const std::vector<double>& newParticlesWeight,
		const std::vector<size_t>& newParticlesDerivedFromIdx) const override;

	/** Evaluate the observation likelihood for one particle at a given location
	 */
	double PF_SLAM_computeObservationLikelihoodForParticle(
		const mrpt::bayes::CParticleFilter::TParticleFilterOptions& PF_options,
		const size_t particleIndexForMap,
		const mrpt::obs::CSensoryFrame& observation,
		const mrpt::poses::CPose3D& x) const override;
	/** @} */

};  // End of class def.

}  // namespace slam
}  // namespace mrpt

#endif
