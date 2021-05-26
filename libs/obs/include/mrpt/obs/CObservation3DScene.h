/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/obs/CObservation.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/serialization/CSerializable.h>

namespace mrpt::obs
{
/** Not a real sensor observation, it stores a 3D scene which can be used for
 * debugging or any other logging purposes.
 *
 * \sa CObservation
 * \ingroup mrpt_obs_grp
 * \note (New in MRPT 2.3.1)
 */
class CObservation3DScene : public CObservation
{
	DEFINE_SERIALIZABLE(CObservation3DScene, mrpt::obs)

   public:
	CObservation3DScene() = default;
	~CObservation3DScene() override = default;

	/** The payload: a 3D scene smart pointer. */
	mrpt::opengl::COpenGLScene::Ptr scene;

	/** The pose of the sensor on the robot. It does not have any predefined
	 * meaning in this particular class. Ignore it unless you want it to have
	 * any particular meaning related to the 3D scene. */
	mrpt::poses::CPose3D sensorPose;

	// See base class docs
	void getSensorPose(mrpt::poses::CPose3D& out_sensorPose) const override
	{
		out_sensorPose = sensorPose;
	}
	void setSensorPose(const mrpt::poses::CPose3D& newSensorPose) override
	{
		sensorPose = newSensorPose;
	}
	void getDescriptionAsText(std::ostream& o) const override;

};	// End of class def.

}  // namespace mrpt::obs
