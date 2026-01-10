/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
#pragma once

#include <mrpt/obs/CObservation.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/viz/Scene.h>
#include <mrpt/viz/Visualizable.h>

namespace mrpt::obs
{
/** Not a real sensor observation, it stores a 3D scene which can be used for
 * debugging or any other logging purposes.
 * If stored in a .rawlog file, RawLogViewer will show the contents of
 * the scene's main viewport when selecting it on the tree view.
 *
 * \sa CObservation
 * \ingroup mrpt_obs_grp
 * \note (New in MRPT 2.3.1)
 */
class CObservation3DScene : public CObservation, public mrpt::viz::Visualizable
{
  DEFINE_SERIALIZABLE(CObservation3DScene, mrpt::obs)

 public:
  CObservation3DScene() = default;
  ~CObservation3DScene() override = default;

  /** The payload: a 3D scene smart pointer. */
  mrpt::viz::Scene::Ptr scene;

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

  void getVisualizationInto(mrpt::viz::CSetOfObjects& o) const override;

};  // End of class def.

}  // namespace mrpt::obs
