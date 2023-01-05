/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/img/color_maps.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/CSetOfObjects.h>

namespace mrpt::obs
{
/** @name customizable_obs_viz_grp Customizable obstacle to 3D visualization API
 *  @{ */

/** Here we can customize the way observations will be rendered as 3D objects
 *  in obs_to_viz(), obs3Dscan_to_viz(), etc.
 *  \sa obs_to_viz(), obs3Dscan_to_viz()
 */
struct VisualizationParameters
{
	VisualizationParameters() = default;
	~VisualizationParameters() = default;

	bool showAxis = true;
	double axisTickFrequency = 1.0;
	double axisLimits = 20.0;
	double axisTickTextSize = 0.075;
	bool colorFromRGBimage = true;
	int colorizeByAxis = 0;	 // 0:x,1:y,2:z, anything else = none.
	bool invertColorMapping = false;
	mrpt::img::TColormap colorMap = mrpt::img::cmJET;
	double pointSize = 4.0;
	bool drawSensorPose = true;
	double sensorPoseScale = 0.3;
	bool onlyPointsWithColor = false;

	bool showSurfaceIn2Dscans = true;
	bool showPointsIn2Dscans = true;
	mrpt::img::TColor surface2DscansColor = {0x00, 0x00, 0xff, 0x80};  // RGBA
	mrpt::img::TColor points2DscansColor = {0xff, 0x00, 0xff, 0xff};  // RGBA

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
	const mrpt::obs::CObservation::Ptr& obs, const VisualizationParameters& p,
	mrpt::opengl::CSetOfObjects& out);

/// Clears `out` and creates a visualization of the given observation.
void obs3Dscan_to_viz(
	const mrpt::obs::CObservation3DRangeScan::Ptr& obs,
	const VisualizationParameters& p, mrpt::opengl::CSetOfObjects& out);

/// Clears `out` and creates a visualization of the given observation.
void obsVelodyne_to_viz(
	const mrpt::obs::CObservationVelodyneScan::Ptr& obs,
	const VisualizationParameters& p, mrpt::opengl::CSetOfObjects& out);

/// Clears `out` and creates a visualization of the given observation.
void obsPointCloud_to_viz(
	const mrpt::obs::CObservationPointCloud::Ptr& obs,
	const VisualizationParameters& p, mrpt::opengl::CSetOfObjects& out);

/// Clears `out` and creates a visualization of the given observation.
void obs2Dscan_to_viz(
	const mrpt::obs::CObservation2DRangeScan::Ptr& obs,
	const VisualizationParameters& p, mrpt::opengl::CSetOfObjects& out);

/// Recolorize a pointcloud according to the given parameters
void recolorize3Dpc(
	const mrpt::opengl::CPointCloudColoured::Ptr& pnts,
	const VisualizationParameters& p);

/** @} */

}  // namespace mrpt::obs