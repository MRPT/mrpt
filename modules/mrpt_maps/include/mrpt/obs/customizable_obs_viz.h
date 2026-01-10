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

#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/img/color_maps.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationRotatingScan.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/CSetOfObjects.h>

namespace mrpt::obs
{
/** @name customizable_obs_viz_grp Customizable obstacle to 3D visualization API
 *  @{ */

/** Parameters for recolorize3Dpc(), or part of VisualizationParameters if using obs_to_viz()
 */
struct PointCloudRecoloringParameters
{
  PointCloudRecoloringParameters() = default;

  /** Any of the field names of a mrpt::maps::CPointsMap, like
   * - `x`,`y`,`z`,`intensity`,`ring`,`t`,`ambient`, etc.
   * - `rgb` for `{color_r,color_g,color_b}` (uint8_t, in range 0-255)
   * - `rgbf` for `{color_rf,color_gf,color_bf}` (float, in range 0-1)
   */
  std::string colorizeByField = "z";

  /// Whether to invert the colormap.
  bool invertColorMapping = false;

  /// The color map to use:
  mrpt::img::TColormap colorMap = mrpt::img::cmJET;

  /// If provided, this will be used as the coordinate for the lowest end of the color map. If not
  /// set, it will be dynamically computed from the data.
  std::optional<float> colorMapMinCoord;

  /// If provided, this will be used as the coordinate for the highest end of the color map. If not
  /// set, it will be dynamically computed from the data.
  std::optional<float> colorMapMaxCoord;

  void save_to_ini_file(
      mrpt::config::CConfigFileBase& cfg,
      const std::string& section = "ParametersView3DPoints") const;

  void load_from_ini_file(
      const mrpt::config::CConfigFileBase& cfg,
      const std::string& section = "ParametersView3DPoints");
};

/** Here we can customize the way observations will be rendered as 3D objects
 *  in obs_to_viz(), obs3Dscan_to_viz(), etc.
 *  \sa obs_to_viz(), obs3Dscan_to_viz()
 */
struct VisualizationParameters
{
  VisualizationParameters() = default;

  PointCloudRecoloringParameters coloring;

  bool showAxis = true;
  double axisTickFrequency = 1.0;
  double axisLimits = 20.0;
  double axisTickTextSize = 0.075;
  bool colorFromRGBimage = true;
  double pointSize = 4.0;
  bool drawSensorPose = true;
  double sensorPoseScale = 0.3;
  bool onlyPointsWithColor = false;

  bool showSurfaceIn2Dscans = true;
  bool showPointsIn2Dscans = true;
  mrpt::img::TColor surface2DscansColor = {0x00, 0x00, 0xff, 0x80};  // RGBA
  mrpt::img::TColor points2DscansColor = {0xff, 0x00, 0xff, 0xff};   // RGBA

  void save_to_ini_file(
      mrpt::config::CConfigFileBase& cfg,
      const std::string& section = "ParametersView3DPoints") const;

  void load_from_ini_file(
      const mrpt::config::CConfigFileBase& cfg,
      const std::string& section = "ParametersView3DPoints");
};

/** Clears `out` and creates a visualization of the given observation,
 *  dispatching the call according to the actual observation class.
 *  \return true if type has known visualizer, false if it does not (then, `out`
 *          will be empty)
 *
 *  \note This and the accompanying functions are defined in namespace
 *        mrpt::obs, but you must link against mrpt::maps too to have their
 *        definitions.
 */
bool obs_to_viz(
    const mrpt::obs::CObservation::Ptr& obs,
    const VisualizationParameters& p,
    mrpt::viz::CSetOfObjects& out);

/** Clears `out` and creates a visualization of the given sensory-frame,
 *  dispatching the call according to the actual observation classes inside the
 * SF.
 *
 *  \return true if type has known visualizer, false if it does not (then, `out`
 *          will be empty)
 *
 *  \note This and the accompanying functions are defined in namespace
 *        mrpt::obs, but you must link against mrpt::maps too to have their
 *        definitions.
 */
bool obs_to_viz(
    const mrpt::obs::CSensoryFrame& sf,
    const VisualizationParameters& p,
    mrpt::viz::CSetOfObjects& out);

/// Clears `out` and creates a visualization of the given observation.
void obs3Dscan_to_viz(
    const mrpt::obs::CObservation3DRangeScan::Ptr& obs,
    const VisualizationParameters& p,
    mrpt::viz::CSetOfObjects& out);

/// Clears `out` and creates a visualization of the given observation.
void obsVelodyne_to_viz(
    const mrpt::obs::CObservationVelodyneScan::Ptr& obs,
    const VisualizationParameters& p,
    mrpt::viz::CSetOfObjects& out);

/// Clears `out` and creates a visualization of the given observation.
void obsPointCloud_to_viz(
    const mrpt::obs::CObservationPointCloud::Ptr& obs,
    const VisualizationParameters& p,
    mrpt::viz::CSetOfObjects& out);

void obsRotatingScan_to_viz(
    const mrpt::obs::CObservationRotatingScan::Ptr& obs,
    const VisualizationParameters& p,
    mrpt::viz::CSetOfObjects& out);

/// Clears `out` and creates a visualization of the given observation.
void obs2Dscan_to_viz(
    const mrpt::obs::CObservation2DRangeScan::Ptr& obs,
    const VisualizationParameters& p,
    mrpt::viz::CSetOfObjects& out);

/// Recolorize a pointcloud according to the given parameters
/// Check mrpt::maps::CGenericPointsMap docs for reserved cloud field names related to coloring.
void recolorize3Dpc(
    const mrpt::viz::CPointCloudColoured::Ptr& pnts,
    const mrpt::maps::CPointsMap* originalPts,
    const PointCloudRecoloringParameters& p);

/** @} */

}  // namespace mrpt::obs
