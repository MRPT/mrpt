/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/bayes/CRejectionSamplingCapable.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/obs/obs_frwds.h>

namespace mrpt
{
namespace maps
{
class CLandmarksMap;
}

namespace slam
{
/** An implementation of rejection sampling for generating 2D robot pose from
 * range-only measurements within a landmarks (beacons) map.
 *    Before calling the method "rejectionSampling" to generate the samples, you
 * must call "setParams".
 *    It is assumed a planar scenario, where the robot is at a fixed height
 * (default=0).
 * \sa bayes::CRejectionSamplingCapable  \ingroup mrpt_slam_grp
 */
class CRejectionSamplingRangeOnlyLocalization
	: public bayes::CRejectionSamplingCapable<mrpt::poses::CPose2D>
{
   public:
	/** Constructor
	 */
	CRejectionSamplingRangeOnlyLocalization();

	/** Destructor
	 */
	~CRejectionSamplingRangeOnlyLocalization() override = default;
	/** The parameters used in the generation of random samples:
	 * \param beaconsMap The map containing the N beacons (indexed by their
	 * "beacon ID"s). Only the mean 3D position of the beacons is used, the
	 * covariance is ignored.
	 * \param observation An observation with, at least ONE range measurement.
	 * \param sigmaRanges The standard deviation of the "range measurement
	 * noise".
	 * \param robot_z The height of the robot on the floor (default=0). Note
	 * that the beacon sensor on the robot may be at a different height,
	 * according to data within the observation object.
	 * \param autoCheckAngleRanges Whether to make a simple check for potential
	 * good angles from the beacons to generate samples (disable to speed-up the
	 * preparation vs. making slower the drawn).
	 *  This method fills out the member "m_dataPerBeacon".
	 * \return true if at least ONE beacon has been successfully loaded, false
	 * otherwise. In this case do not call "rejectionSampling" or an exception
	 * will be launch, since there is no information to generate samples.
	 */
	bool setParams(
		const mrpt::maps::CLandmarksMap& beaconsMap,
		const mrpt::obs::CObservationBeaconRanges& observation,
		float sigmaRanges, const mrpt::poses::CPose2D& oldPose,
		float robot_z = 0, bool autoCheckAngleRanges = true);

   protected:
	/** Generates one sample, drawing from some proposal distribution.
	 */
	void RS_drawFromProposal(mrpt::poses::CPose2D& outSample) override;

	/** Returns the NORMALIZED observation likelihood (linear, not
	 * exponential!!!) at a given point of the state space (values in the range
	 * [0,1]).
	 */
	double RS_observationLikelihood(const mrpt::poses::CPose2D& x) override;

	/** Z coordinate of the robot.
	 */
	float m_z_robot{0};

	float m_sigmaRanges{0.10f};
	mrpt::poses::CPose2D m_oldPose;

	/** The index in "m_dataPerBeacon" used to draw samples (the rest will be
	 * used to evaluate the likelihood)
	 */
	size_t m_drawIndex{0};

	/** Data for each beacon observation with a correspondence with the map.
	 */
	struct TDataPerBeacon
	{
		TDataPerBeacon() = default;

		mrpt::math::TPoint3D sensorOnRobot;
		mrpt::math::TPoint2D beaconPosition;
		float radiusAtRobotPlane{0};
		float minAngle{0}, maxAngle{0};
	};

	/** Data for each beacon observation with a correspondence with the map.
	 */
	std::deque<TDataPerBeacon> m_dataPerBeacon;
};

}  // namespace slam
}  // namespace mrpt
