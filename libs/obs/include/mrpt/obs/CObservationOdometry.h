/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#ifndef CObservationOdometry_H
#define CObservationOdometry_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/math/lightweight_geom_data.h>

namespace mrpt
{
namespace obs
{
/** An observation of the current (cumulative) odometry for a wheeled robot.
 *   This kind of observation will only occur in a "observation-only" rawlog
 * file, otherwise
 *    odometry are modeled with actions. Refer to the <a
 * href="http://www.mrpt.org/Rawlog_Format">page on rawlogs</a>.
 *
 * \sa CObservation, CActionRobotMovement2D
 * \ingroup mrpt_obs_grp
 */
class CObservationOdometry : public CObservation
{
	DEFINE_SERIALIZABLE(CObservationOdometry)

   public:
	/** Default ctor */
	CObservationOdometry();

	/** The absolute odometry measurement (IT IS NOT INCREMENTAL) */
	mrpt::poses::CPose2D odometry;

	/** "true" means that "encoderLeftTicks" and "encoderRightTicks" contain
	 * valid values. */
	bool hasEncodersInfo;
	/** For differential-driven robots: The ticks count for each wheel in
	 * ABSOLUTE VALUE (IT IS NOT INCREMENTAL) (positive means FORWARD, for both
	 * wheels); \sa hasEncodersInfo  */
	int32_t encoderLeftTicks, encoderRightTicks;

	/** "true" means that `velocityLocal` contains valid values. */
	bool hasVelocities;
	/** Velocity, in the robot (local) frame of reference (+X=forward). */
	mrpt::math::TTwist2D velocityLocal;

	// See base class docs
	void getSensorPose(mrpt::poses::CPose3D& out_sensorPose) const override
	{
		out_sensorPose = mrpt::poses::CPose3D(0, 0, 0);
	}
	void setSensorPose(const mrpt::poses::CPose3D&) override {}
	void getDescriptionAsText(std::ostream& o) const override;

};  // End of class def.

}  // End of namespace
}  // End of namespace

#endif
