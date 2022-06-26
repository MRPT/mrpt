/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "maps-precomp.h"  // Precomp header
//
#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/obs/customizable_obs_viz.h>
#include <mrpt/opengl/CAxis.h>
#include <mrpt/opengl/CPlanarLaserScan.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/stock_objects.h>

using namespace mrpt::obs;

void VisualizationParameters::save_to_ini_file(
	mrpt::config::CConfigFileBase& cfg, const std::string& section) const
{
	auto& c = cfg;
	const std::string s = section;

	MRPT_SAVE_CONFIG_VAR(axisTickFrequency, c, s);
	MRPT_SAVE_CONFIG_VAR(axisLimits, c, s);
	MRPT_SAVE_CONFIG_VAR(axisTickTextSize, c, s);
	MRPT_SAVE_CONFIG_VAR(colorFromRGBimage, c, s);
	MRPT_SAVE_CONFIG_VAR(colorizeByAxis, c, s);
	MRPT_SAVE_CONFIG_VAR(invertColorMapping, c, s);
	MRPT_SAVE_CONFIG_VAR(pointSize, c, s);
	MRPT_SAVE_CONFIG_VAR(colorMap, c, s);
	MRPT_SAVE_CONFIG_VAR(drawSensorPose, c, s);
	MRPT_SAVE_CONFIG_VAR(sensorPoseScale, c, s);
	MRPT_SAVE_CONFIG_VAR(showAxis, c, s);
	MRPT_SAVE_CONFIG_VAR(showSurfaceIn2Dscans, c, s);
	MRPT_SAVE_CONFIG_VAR(showPointsIn2Dscans, c, s);
	MRPT_SAVE_CONFIG_VAR(onlyPointsWithColor, c, s);

	MRPT_SAVE_CONFIG_VAR(surface2DscansColor.R, c, s);
	MRPT_SAVE_CONFIG_VAR(surface2DscansColor.G, c, s);
	MRPT_SAVE_CONFIG_VAR(surface2DscansColor.B, c, s);
	MRPT_SAVE_CONFIG_VAR(surface2DscansColor.A, c, s);

	MRPT_SAVE_CONFIG_VAR(points2DscansColor.R, c, s);
	MRPT_SAVE_CONFIG_VAR(points2DscansColor.G, c, s);
	MRPT_SAVE_CONFIG_VAR(points2DscansColor.B, c, s);
	MRPT_SAVE_CONFIG_VAR(points2DscansColor.A, c, s);
}

void VisualizationParameters::load_from_ini_file(
	const mrpt::config::CConfigFileBase& cfg, const std::string& section)
{
	const auto& c = cfg;
	const std::string s = section;

	MRPT_LOAD_CONFIG_VAR_CS(axisTickFrequency, double);
	MRPT_LOAD_CONFIG_VAR_CS(axisLimits, double);
	MRPT_LOAD_CONFIG_VAR_CS(axisTickTextSize, double);
	MRPT_LOAD_CONFIG_VAR_CS(colorFromRGBimage, bool);
	MRPT_LOAD_CONFIG_VAR_CS(colorizeByAxis, int);
	MRPT_LOAD_CONFIG_VAR_CS(invertColorMapping, bool);
	MRPT_LOAD_CONFIG_VAR_CS(pointSize, double);
	MRPT_LOAD_CONFIG_VAR_CS(drawSensorPose, bool);
	MRPT_LOAD_CONFIG_VAR_CS(sensorPoseScale, double);
	colorMap = c.read_enum(s, "colorMap", colorMap);
	MRPT_LOAD_CONFIG_VAR_CS(showAxis, bool);
	MRPT_LOAD_CONFIG_VAR_CS(showSurfaceIn2Dscans, bool);
	MRPT_LOAD_CONFIG_VAR_CS(showPointsIn2Dscans, bool);
	MRPT_LOAD_CONFIG_VAR_CS(onlyPointsWithColor, bool);

	MRPT_LOAD_CONFIG_VAR_CS(surface2DscansColor.R, int);
	MRPT_LOAD_CONFIG_VAR_CS(surface2DscansColor.G, int);
	MRPT_LOAD_CONFIG_VAR_CS(surface2DscansColor.B, int);
	MRPT_LOAD_CONFIG_VAR_CS(surface2DscansColor.A, int);

	MRPT_LOAD_CONFIG_VAR_CS(points2DscansColor.R, int);
	MRPT_LOAD_CONFIG_VAR_CS(points2DscansColor.G, int);
	MRPT_LOAD_CONFIG_VAR_CS(points2DscansColor.B, int);
	MRPT_LOAD_CONFIG_VAR_CS(points2DscansColor.A, int);
}

// Bounding box memory so we have consistent coloring across different sensors:
static thread_local std::optional<mrpt::math::TBoundingBox> bbMemory;
double bbMemoryFading = 0.99;

void mrpt::obs::recolorize3Dpc(
	const mrpt::opengl::CPointCloudColoured::Ptr& pnts,
	const VisualizationParameters& p)
{
	const auto newBb = pnts->getBoundingBox();

	// Slowly update bb memory:
	if (!bbMemory.has_value())
	{
		// first time:
		bbMemory = newBb;
	}
	else
	{
		bbMemory.value().min = bbMemory.value().min * bbMemoryFading +
			newBb.min * (1.0 - bbMemoryFading);
		bbMemory.value().max = bbMemory.value().max * bbMemoryFading +
			newBb.max * (1.0 - bbMemoryFading);
	}
	const auto& bb = bbMemory.value();

	// actual colorize:
	switch (p.colorizeByAxis)
	{
		case 0:
			pnts->recolorizeByCoordinate(
				p.invertColorMapping ? bb.max.x : bb.min.x,
				p.invertColorMapping ? bb.min.x : bb.max.x, 0 /* x */,
				p.colorMap);
			break;
		case 1:
			pnts->recolorizeByCoordinate(
				p.invertColorMapping ? bb.max.y : bb.min.y,
				p.invertColorMapping ? bb.min.y : bb.max.y, 1 /* y */,
				p.colorMap);
			break;
		case 2:
			pnts->recolorizeByCoordinate(
				p.invertColorMapping ? bb.max.z : bb.min.z,
				p.invertColorMapping ? bb.min.z : bb.max.z, 2 /* z */,
				p.colorMap);
			break;
		default: break;
	}
}

static void add_common_to_viz(
	const CObservation& obs, const VisualizationParameters& p,
	mrpt::opengl::CSetOfObjects& out)
{
	if (p.showAxis)
	{
		const float L = p.axisLimits;
		auto gl_axis = mrpt::opengl::CAxis::Create(
			-L, -L, -L, L, L, L, p.axisTickFrequency, 2, true);
		gl_axis->setTextScale(p.axisTickTextSize);
		gl_axis->setColor_u8(0xa0, 0xa0, 0xa0, 0x80);
		out.insert(gl_axis);

		// Show axis labels such that they can be read from the IIIrd-IVth
		// quadrant:
		const float yawIncrs[3] = {180.f, -90.f, 180.f};
		for (int axis = 0; axis < 3; axis++)
		{
			float yaw_deg, pitch_deg, roll_deg;
			gl_axis->getTextLabelOrientation(
				axis, yaw_deg, pitch_deg, roll_deg);
			yaw_deg += yawIncrs[axis];
			gl_axis->setTextLabelOrientation(
				axis, yaw_deg, pitch_deg, roll_deg);
		}
	}

	if (p.drawSensorPose)
	{
		const auto glCorner =
			mrpt::opengl::stock_objects::CornerXYZSimple(p.sensorPoseScale);
		glCorner->setPose(obs.sensorPose());
		out.insert(glCorner);
	}
}

void mrpt::obs::obs3Dscan_to_viz(
	const CObservation3DRangeScan::Ptr& obs, const VisualizationParameters& p,
	mrpt::opengl::CSetOfObjects& out)
{
	out.clear();

	// Generate/load 3D points
	// ----------------------
	mrpt::maps::CPointsMap::Ptr pointMap;
	mrpt::maps::CColouredPointsMap::Ptr pointMapCol;
	mrpt::obs::T3DPointsProjectionParams pp;
	pp.takeIntoAccountSensorPoseOnRobot = true;
	pp.onlyPointsWithIntensityColor = p.onlyPointsWithColor;

	// Color from intensity image?
	if (p.colorFromRGBimage && obs->hasRangeImage && obs->hasIntensityImage)
	{
		pointMapCol = mrpt::maps::CColouredPointsMap::Create();
		pointMapCol->colorScheme.scheme =
			mrpt::maps::CColouredPointsMap::cmFromIntensityImage;

		obs->unprojectInto(*pointMapCol, pp);
		pointMap = pointMapCol;
	}
	else
	{
		// Empty point set, or load from XYZ in observation:
		pointMap = mrpt::maps::CSimplePointsMap::Create();
		if (obs->hasPoints3D)
		{
			for (size_t i = 0; i < obs->points3D_x.size(); i++)
				pointMap->insertPoint(
					obs->points3D_x[i], obs->points3D_y[i], obs->points3D_z[i]);
		}
		else if (obs->hasRangeImage)
		{
			obs->unprojectInto(*pointMap, pp);
		}
	}

	add_common_to_viz(*obs, p, out);

	auto gl_pnts = mrpt::opengl::CPointCloudColoured::Create();
	// Load as RGB or grayscale points:
	if (pointMapCol) gl_pnts->loadFromPointsMap(pointMapCol.get());
	else
	{
		gl_pnts->loadFromPointsMap(pointMap.get());
		recolorize3Dpc(gl_pnts, p);
	}

	// No need to further transform 3D points
	gl_pnts->setPose(mrpt::poses::CPose3D());
	gl_pnts->setPointSize(p.pointSize);

	out.insert(gl_pnts);
}

void mrpt::obs::obsVelodyne_to_viz(
	const CObservationVelodyneScan::Ptr& obs, const VisualizationParameters& p,
	mrpt::opengl::CSetOfObjects& out)
{
	out.clear();

	add_common_to_viz(*obs, p, out);

	auto pnts = mrpt::opengl::CPointCloudColoured::Create();
	out.insert(pnts);

	mrpt::maps::CColouredPointsMap pntsMap;
	pntsMap.loadFromVelodyneScan(*obs);
	pnts->loadFromPointsMap(&pntsMap);
	pnts->setPointSize(p.pointSize);

	if (!p.colorFromRGBimage) recolorize3Dpc(pnts, p);
}

void mrpt::obs::obsPointCloud_to_viz(
	const mrpt::obs::CObservationPointCloud::Ptr& obs,
	const VisualizationParameters& p, mrpt::opengl::CSetOfObjects& out)
{
	out.clear();

	add_common_to_viz(*obs, p, out);

	auto pnts = mrpt::opengl::CPointCloudColoured::Create();
	out.insert(pnts);

	if (obs->pointcloud) pnts->loadFromPointsMap(obs->pointcloud.get());
	pnts->setPose(obs->sensorPose);

	pnts->setPointSize(p.pointSize);

	if (!p.colorFromRGBimage) recolorize3Dpc(pnts, p);
}

void mrpt::obs::obs2Dscan_to_viz(
	const CObservation2DRangeScan::Ptr& obs, const VisualizationParameters& p,
	mrpt::opengl::CSetOfObjects& out)
{
	out.clear();

	add_common_to_viz(*obs, p, out);

	auto pnts = mrpt::opengl::CPlanarLaserScan::Create();
	out.insert(pnts);

	pnts->setScan(*obs);
	pnts->setPointSize(p.pointSize);
	pnts->enableSurface(p.showSurfaceIn2Dscans);
	pnts->enablePoints(p.showPointsIn2Dscans);

	pnts->setSurfaceColor(
		mrpt::u8tof(p.surface2DscansColor.R),
		mrpt::u8tof(p.surface2DscansColor.G),
		mrpt::u8tof(p.surface2DscansColor.B),
		mrpt::u8tof(p.surface2DscansColor.A));
	pnts->setPointsColor(
		mrpt::u8tof(p.points2DscansColor.R),
		mrpt::u8tof(p.points2DscansColor.G),
		mrpt::u8tof(p.points2DscansColor.B),
		mrpt::u8tof(p.points2DscansColor.A));
}

bool mrpt::obs::obs_to_viz(
	const mrpt::obs::CObservation::Ptr& obs, const VisualizationParameters& p,
	mrpt::opengl::CSetOfObjects& out)
{
	using namespace mrpt::obs;

	if (const auto o3D =
			std::dynamic_pointer_cast<CObservation3DRangeScan>(obs);
		o3D)
	{
		obs3Dscan_to_viz(o3D, p, out);
		return true;
	}
	else if (const auto oVel =
				 std::dynamic_pointer_cast<CObservationVelodyneScan>(obs);
			 oVel)
	{
		obsVelodyne_to_viz(oVel, p, out);
		return true;
	}
	else if (const auto oPC =
				 std::dynamic_pointer_cast<CObservationPointCloud>(obs);
			 oPC)
	{
		obsPointCloud_to_viz(oPC, p, out);
		return true;
	}
	else if (const auto o2D =
				 std::dynamic_pointer_cast<CObservation2DRangeScan>(obs);
			 o2D)
	{
		obs2Dscan_to_viz(o2D, p, out);
		return true;
	}

	// No conversion found:
	out.clear();
	return false;
}
