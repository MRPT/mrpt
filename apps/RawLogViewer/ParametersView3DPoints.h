/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/img/color_maps.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/CSetOfObjects.h>

class ViewOptions3DPoints;

struct ParametersView3DPoints
{
	ParametersView3DPoints() = default;

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

	void to_UI(ViewOptions3DPoints& ui) const;
	void from_UI(const ViewOptions3DPoints& ui);

	void save_to_ini_file() const;
	void load_from_ini_file();
};

void recolorize3Dpc(
	const mrpt::opengl::CPointCloudColoured::Ptr& pnts,
	const ParametersView3DPoints& p);

void obs3Dscan_to_viz(
	const mrpt::obs::CObservation3DRangeScan::Ptr& obs,
	const ParametersView3DPoints& p, mrpt::opengl::CSetOfObjects& out);

void obsVelodyne_to_viz(
	const mrpt::obs::CObservationVelodyneScan::Ptr& obs,
	const ParametersView3DPoints& p, mrpt::opengl::CSetOfObjects& out);
