/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "nav-precomp.h"  // Precomp header

#include <mrpt/nav/planners/PlannerRRT_common.h>
#include <mrpt/nav/tpspace/CPTG_DiffDrive_CollisionGridBased.h>
#include <mrpt/math/CPolygon.h>

using namespace mrpt::nav;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace std;

RRTAlgorithmParams::RRTAlgorithmParams()
	: ptg_cache_files_directory("."),

	  minAngBetweenNewNodes(mrpt::DEG2RAD(15))

{
	robot_shape.push_back(mrpt::math::TPoint2D(-0.5, -0.5));
	robot_shape.push_back(mrpt::math::TPoint2D(0.8, -0.4));
	robot_shape.push_back(mrpt::math::TPoint2D(0.8, 0.4));
	robot_shape.push_back(mrpt::math::TPoint2D(-0.5, 0.5));
}

PlannerTPS_VirtualBase::PlannerTPS_VirtualBase() = default;
void PlannerTPS_VirtualBase::internal_initialize_PTG()
{
	ASSERTMSG_(
		!m_PTGs.empty(),
		"No PTG was defined! At least one must be especified.");

	// Convert to CPolygon for API requisites:
	mrpt::math::CPolygon poly_robot_shape;
	poly_robot_shape.clear();
	if (!params.robot_shape.empty())
	{
		vector<double> xm, ym;
		params.robot_shape.getPlotData(xm, ym);
		poly_robot_shape.setAllVertices(xm, ym);
	}

	for (size_t i = 0; i < m_PTGs.size(); i++)
	{
		mrpt::system::CTimeLoggerEntry tle(m_timelogger, "PTG_initialization");

		// Polygonal robot shape?
		{
			auto* diff_ptg =
				dynamic_cast<mrpt::nav::CPTG_DiffDrive_CollisionGridBased*>(
					m_PTGs[i].get());
			if (diff_ptg)
			{
				ASSERTMSG_(
					!poly_robot_shape.empty(),
					"No polygonal robot shape specified, and PTG requires "
					"one!");
				diff_ptg->setRobotShape(poly_robot_shape);
			}
		}
		// Circular robot shape?
		{
			auto* ptg = dynamic_cast<mrpt::nav::CPTG_RobotShape_Circular*>(
				m_PTGs[i].get());
			if (ptg)
			{
				ASSERTMSG_(
					params.robot_shape_circular_radius > 0,
					"No circular robot shape specified, and PTG requires one!");
				ptg->setRobotShapeRadius(params.robot_shape_circular_radius);
			}
		}

		m_PTGs[i]->initialize(
			mrpt::format(
				"%s/TPRRT_PTG_%03u.dat.gz",
				params.ptg_cache_files_directory.c_str(),
				static_cast<unsigned int>(i)),
			params.ptg_verbose);
	}

	m_initialized_PTG = true;
}

void PlannerTPS_VirtualBase::internal_loadConfig_PTG(
	const mrpt::config::CConfigFileBase& ini, const std::string& sSect)
{
	// Robot shape:
	// ==========================
	// polygonal shape
	{
		// Robot shape is a bit special to load:
		params.robot_shape.clear();
		const std::string sShape = ini.read_string(sSect, "robot_shape", "");
		if (!sShape.empty())
		{
			CMatrixDouble mShape;
			if (!mShape.fromMatlabStringFormat(sShape))
				THROW_EXCEPTION_FMT(
					"Error parsing robot_shape matrix: '%s'", sShape.c_str());
			ASSERT_(mShape.rows() == 2);
			ASSERT_(mShape.cols() >= 3);

			for (int i = 0; i < mShape.cols(); i++)
				params.robot_shape.push_back(
					TPoint2D(mShape(0, i), mShape(1, i)));
		}
	}
	// circular shape
	params.robot_shape_circular_radius =
		ini.read_double(sSect, "robot_shape_circular_radius", 0.0);

	// Load PTG tables:
	// ==========================
	m_PTGs.clear();

	const size_t PTG_COUNT =
		ini.read_int(sSect, "PTG_COUNT", 0, true);  // load the number of PTGs
	for (unsigned int n = 0; n < PTG_COUNT; n++)
	{
		// Generate it:
		const std::string sPTGName =
			ini.read_string(sSect, format("PTG%u_Type", n), "", true);
		m_PTGs.push_back(CParameterizedTrajectoryGenerator::Ptr(
			CParameterizedTrajectoryGenerator::CreatePTG(
				sPTGName, ini, sSect, format("PTG%u_", n))));
	}
}

// Auxiliary function:
void PlannerTPS_VirtualBase::transformPointcloudWithSquareClipping(
	const mrpt::maps::CPointsMap& in_map, mrpt::maps::CPointsMap& out_map,
	const mrpt::poses::CPose2D& asSeenFrom, const double MAX_DIST_XY)
{
	size_t nObs;
	const float *obs_xs, *obs_ys, *obs_zs;
	in_map.getPointsBuffer(nObs, obs_xs, obs_ys, obs_zs);

	out_map.clear();
	out_map.reserve(nObs);  // Prealloc mem for speed-up

	const CPose2D invPose = -asSeenFrom;
	// We can safely discard the rest of obstacles, since they cannot be
	// converted into TP-Obstacles anyway!

	for (size_t obs = 0; obs < nObs; obs++)
	{
		const double gx = obs_xs[obs], gy = obs_ys[obs];

		if (std::abs(gx - asSeenFrom.x()) > MAX_DIST_XY ||
			std::abs(gy - asSeenFrom.y()) > MAX_DIST_XY)
			continue;  // ignore this obstacle: anyway, I don't know how to map
		// it to TP-Obs!

		double ox, oy;
		invPose.composePoint(gx, gy, ox, oy);

		out_map.insertPointFast(ox, oy, 0);
	}
}

/*---------------------------------------------------------------
SpaceTransformer
---------------------------------------------------------------*/
void PlannerTPS_VirtualBase::spaceTransformer(
	const mrpt::maps::CSimplePointsMap& in_obstacles,
	const mrpt::nav::CParameterizedTrajectoryGenerator* in_PTG,
	const double MAX_DIST, std::vector<double>& out_TPObstacles)
{
	using namespace mrpt::nav;
	try
	{
		// Take "k_rand"s and "distances" such that the collision hits the
		// obstacles
		// in the "grid" of the given PT
		// --------------------------------------------------------------------
		size_t nObs;
		const float *obs_xs, *obs_ys, *obs_zs;
		// = in_obstacles.getPointsCount();
		in_obstacles.getPointsBuffer(nObs, obs_xs, obs_ys, obs_zs);

		// Init obs ranges:
		in_PTG->initTPObstacles(out_TPObstacles);

		for (size_t obs = 0; obs < nObs; obs++)
		{
			const float ox = obs_xs[obs];
			const float oy = obs_ys[obs];

			if (std::abs(ox) > MAX_DIST || std::abs(oy) > MAX_DIST)
				continue;  // ignore this obstacle: anyway, I don't know how to
			// map it to TP-Obs!

			in_PTG->updateTPObstacle(ox, oy, out_TPObstacles);
		}

		// Leave distances in out_TPObstacles un-normalized ([0,1]), so they
		// just represent real distances in meters.
	}
	catch (const std::exception& e)
	{
		cerr << "[PT_RRT::SpaceTransformer] Exception:" << endl;
		cerr << e.what() << endl;
	}
	catch (...)
	{
		cerr << "\n[PT_RRT::SpaceTransformer] Unexpected exception!:\n";
		cerr << format("*in_PTG = %p\n", (void*)in_PTG);
		if (in_PTG)
			cerr << format("PTG = %s\n", in_PTG->getDescription().c_str());
		cerr << endl;
	}
}

void PlannerTPS_VirtualBase::spaceTransformerOneDirectionOnly(
	const int tp_space_k_direction,
	const mrpt::maps::CSimplePointsMap& in_obstacles,
	const mrpt::nav::CParameterizedTrajectoryGenerator* in_PTG,
	const double MAX_DIST, double& out_TPObstacle_k)
{
	using namespace mrpt::nav;
	try
	{
		// Take "k_rand"s and "distances" such that the collision hits the
		// obstacles
		// in the "grid" of the given PT
		// --------------------------------------------------------------------
		size_t nObs;
		const float *obs_xs, *obs_ys, *obs_zs;
		// = in_obstacles.getPointsCount();
		in_obstacles.getPointsBuffer(nObs, obs_xs, obs_ys, obs_zs);

		// Init obs ranges:
		in_PTG->initTPObstacleSingle(tp_space_k_direction, out_TPObstacle_k);

		for (size_t obs = 0; obs < nObs; obs++)
		{
			const float ox = obs_xs[obs];
			const float oy = obs_ys[obs];

			if (std::abs(ox) > MAX_DIST || std::abs(oy) > MAX_DIST)
				continue;  // ignore this obstacle: anyway, I don't know how to
			// map it to TP-Obs!

			in_PTG->updateTPObstacleSingle(
				ox, oy, tp_space_k_direction, out_TPObstacle_k);
		}

		// Leave distances in out_TPObstacles un-normalized ([0,1]), so they
		// just represent real distances in meters.
	}
	catch (const std::exception& e)
	{
		cerr << "[PT_RRT::SpaceTransformer] Exception:" << endl;
		cerr << e.what() << endl;
	}
	catch (...)
	{
		cerr << "\n[PT_RRT::SpaceTransformer] Unexpected exception!:\n";
		cerr << format("*in_PTG = %p\n", (void*)in_PTG);
		if (in_PTG)
			cerr << format("PTG = %s\n", in_PTG->getDescription().c_str());
		cerr << endl;
	}
}
