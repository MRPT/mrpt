/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

// For 3D log files
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/CText3D.h>

namespace mrpt::nav
{
template <typename node_pose_t, typename world_limits_t, typename tree_t>
void PlannerTPS_VirtualBase::renderMoveTree(
	mrpt::opengl::COpenGLScene& scene,
	const TPlannerInputTempl<node_pose_t, world_limits_t>& pi,
	const TPlannerResultTempl<tree_t>& result,
	const TRenderPlannedPathOptions& options)
{
	using std::string;

	// Build a model of the vehicle shape:
	mrpt::opengl::CSetOfLines::Ptr gl_veh_shape =
		mrpt::make_aligned_shared<mrpt::opengl::CSetOfLines>();
	double
		xyzcorners_scale;  // Size of XYZ corners (scaled to vehicle dimensions)
	{
		gl_veh_shape->setLineWidth(options.vehicle_line_width);
		gl_veh_shape->setColor_u8(options.color_vehicle);

		double max_veh_radius = 0.;
		if (!params.robot_shape.empty())
		{
			gl_veh_shape->appendLine(
				params.robot_shape[0].x, params.robot_shape[0].y, 0,
				params.robot_shape[1].x, params.robot_shape[1].y, 0);
			for (size_t i = 2; i <= params.robot_shape.size(); i++)
			{
				const size_t idx = i % params.robot_shape.size();
				mrpt::keep_max(max_veh_radius, params.robot_shape[idx].norm());
				gl_veh_shape->appendLineStrip(
					params.robot_shape[idx].x, params.robot_shape[idx].y, 0);
			}
		}
		else if (params.robot_shape_circular_radius > 0)
		{
			const int NUM_VERTICES = 10;
			const double R = params.robot_shape_circular_radius;
			for (int i = 0; i <= NUM_VERTICES; i++)
			{
				const size_t idx = i % NUM_VERTICES;
				const size_t idxn = (i + 1) % NUM_VERTICES;
				const double ang = idx * 2 * M_PI / (NUM_VERTICES - 1);
				const double angn = idxn * 2 * M_PI / (NUM_VERTICES - 1);
				gl_veh_shape->appendLine(
					R * cos(ang), R * sin(ang), 0, R * cos(angn), R * sin(angn),
					0);
			}
			mrpt::keep_max(max_veh_radius, R);
		}

		xyzcorners_scale = max_veh_radius * 0.5;
	}
	// Override with user scale?
	if (options.xyzcorners_scale != 0)
		xyzcorners_scale = options.xyzcorners_scale;

	// "ground"
	if (options.ground_xy_grid_frequency > 0)
	{
		mrpt::opengl::CGridPlaneXY::Ptr obj =
			mrpt::make_aligned_shared<mrpt::opengl::CGridPlaneXY>(
				pi.world_bbox_min.x, pi.world_bbox_max.x, pi.world_bbox_min.y,
				pi.world_bbox_max.y, 0, options.ground_xy_grid_frequency);
		obj->setColor_u8(options.color_ground_xy_grid);
		scene.insert(obj);
	}

	// Original randomly-pick pose:
	if (options.x_rand_pose)
	{
		mrpt::opengl::CSetOfObjects::Ptr obj =
			mrpt::opengl::stock_objects::CornerXYZ(xyzcorners_scale * 1.0);
		string m_name = "X_rand";
		obj->setName(m_name);
		obj->enableShowName();
		obj->setPose(mrpt::poses::CPose3D(*options.x_rand_pose));
		scene.insert(obj);
	}

	// Nearest state pose:
	if (options.x_nearest_pose)
	{
		mrpt::opengl::CSetOfObjects::Ptr obj =
			mrpt::opengl::stock_objects::CornerXYZ(xyzcorners_scale * 1.0);
		string m_name = "X_near";
		obj->setName(m_name);
		obj->enableShowName();
		obj->setPose(mrpt::poses::CPose3D(*options.x_nearest_pose));
		scene.insert(obj);
	}

	// Determine the up-to-now best solution, so we can highlight the best path
	// so far:
	typename tree_t::path_t best_path;
	if (options.highlight_path_to_node_id != INVALID_NODEID)
		result.move_tree.backtrackPath(
			options.highlight_path_to_node_id, best_path);

	// make list of nodes in the way of the best path:
	std::set<const typename tree_t::edge_t*> edges_best_path,
		edges_best_path_decim;
	if (!best_path.empty())
	{
		auto it_end = best_path.end();
		auto it_end_1 = best_path.end();
		std::advance(it_end_1, -1);

		for (auto it = best_path.begin(); it != it_end; ++it)
			if (it->edge_to_parent) edges_best_path.insert(it->edge_to_parent);

		// Decimate the path (always keeping the first and last entry):
		ASSERT_ABOVE_(options.draw_shape_decimation, 0);
		for (auto it = best_path.begin(); it != it_end;)
		{
			if (it->edge_to_parent)
				edges_best_path_decim.insert(it->edge_to_parent);
			if (it == it_end_1) break;
			for (size_t k = 0; k < options.draw_shape_decimation; k++)
			{
				if (it == it_end || it == it_end_1) break;
				++it;
			}
		}
	}

	// The starting pose vehicle shape must be inserted independently, because
	// the rest are edges and we draw the END pose of each edge:
	{
		mrpt::opengl::CSetOfLines::Ptr vehShape(
			new mrpt::opengl::CSetOfLines(*gl_veh_shape));
		mrpt::poses::CPose3D shapePose(mrpt::math::TPose3D(pi.start_pose));
		shapePose.z_incr(options.vehicle_shape_z);
		vehShape->setPose(shapePose);
		scene.insert(vehShape);
	}

	// Existing nodes & edges between them:
	{
		const typename tree_t::node_map_t& lstNodes =
			result.move_tree.getAllNodes();

		for (auto itNode = lstNodes.begin(); itNode != lstNodes.end(); ++itNode)
		{
			const typename tree_t::node_t& node = itNode->second;

			mrpt::math::TPose2D parent_state;
			if (node.parent_id != INVALID_NODEID)
			{
				parent_state = lstNodes.find(node.parent_id)->second.state;
			}
			const mrpt::math::TPose2D& trg_state = node.state;

			const bool is_new_one = (itNode == (lstNodes.end() - 1));
			const bool is_best_path =
				edges_best_path.count(node.edge_to_parent) != 0;
			const bool is_best_path_and_draw_shape =
				edges_best_path_decim.count(node.edge_to_parent) != 0;

			// Draw children nodes:
			{
				const float corner_scale =
					xyzcorners_scale * (is_new_one ? 1.5f : 1.0f);

				mrpt::opengl::CSetOfObjects::Ptr obj =
					mrpt::opengl::stock_objects::CornerXYZSimple(corner_scale);
				obj->setPose(mrpt::poses::CPose3D(trg_state));
				scene.insert(obj);

				// Insert vehicle shapes along optimal path:
				if (is_best_path_and_draw_shape)
				{
					mrpt::opengl::CSetOfLines::Ptr vehShape(
						new mrpt::opengl::CSetOfLines(*gl_veh_shape));
					auto shapePose = mrpt::poses::CPose3D(trg_state);
					shapePose.z_incr(options.vehicle_shape_z);
					vehShape->setPose(shapePose);
					scene.insert(vehShape);
				}
			}

			// Draw line parent -> children nodes.
			if (node.parent_id != INVALID_NODEID)
			{
				// Draw actual PT path between parent and children nodes:
				ASSERT_(node.edge_to_parent);
				const mrpt::nav::CParameterizedTrajectoryGenerator* ptg =
					m_PTGs[node.edge_to_parent->ptg_index].get();

				// Create the path shape, in relative coords to the parent node:
				mrpt::opengl::CSetOfLines::Ptr obj =
					mrpt::make_aligned_shared<mrpt::opengl::CSetOfLines>();
				obj->setPose(
					mrpt::poses::CPose3D(parent_state));  // Points are relative
				// to this pose: let
				// OpenGL to deal with
				// the coords.
				// composition

				ptg->renderPathAsSimpleLine(
					node.edge_to_parent->ptg_K, *obj, 0.25f /*decimation*/,
					node.edge_to_parent->ptg_dist /*max path length*/);

				if (is_new_one && options.highlight_last_added_edge)
				{
					// Last edge format:
					obj->setColor_u8(options.color_last_edge);
					obj->setLineWidth(options.width_last_edge);
				}
				else
				{
					// Normal format:
					obj->setColor_u8(options.color_normal_edge);
					obj->setLineWidth(options.width_normal_edge);
				}
				if (is_best_path)
				{
					obj->setColor_u8(options.color_optimal_edge);
					obj->setLineWidth(options.width_optimal_edge);
				}

				scene.insert(obj);
			}
		}
	}

	// The new node:
	if (options.new_state)
	{
		mrpt::opengl::CSetOfObjects::Ptr obj =
			mrpt::opengl::stock_objects::CornerXYZ(xyzcorners_scale * 1.2);
		string m_name = "X_new";
		obj->setName(m_name);
		obj->enableShowName();
		obj->setPose(mrpt::poses::CPose3D(*options.new_state));
		scene.insert(obj);
	}

	// Obstacles:
	if (options.draw_obstacles)
	{
		mrpt::opengl::CPointCloud::Ptr obj =
			mrpt::make_aligned_shared<mrpt::opengl::CPointCloud>();

		obj->loadFromPointsMap(&pi.obstacles_points);
		obj->setPose(mrpt::poses::CPose3D(mrpt::poses::CPose2D(
			0.0, 0.0, 0.0)));  // Points are relative to the origin

		obj->setPointSize(options.point_size_obstacles);
		obj->setColor_u8(options.color_obstacles);
		scene.insert(obj);
	}

	// The current set of local obstacles:
	// Draw this AFTER the global map so it's visible:
	if (options.draw_obstacles && options.local_obs_from_nearest_pose &&
		options.x_nearest_pose)
	{
		mrpt::opengl::CPointCloud::Ptr obj =
			mrpt::make_aligned_shared<mrpt::opengl::CPointCloud>();

		obj->loadFromPointsMap(options.local_obs_from_nearest_pose);
		obj->setPose(*options.x_nearest_pose);  // Points are relative to this
		// pose: let OpenGL to deal with
		// the coords. composition
		obj->setPointSize(options.point_size_local_obstacles);
		obj->setColor_u8(options.color_local_obstacles);
		scene.insert(obj);
	}

	// Start:
	{
		mrpt::opengl::CSetOfObjects::Ptr obj =
			mrpt::opengl::stock_objects::CornerXYZ(xyzcorners_scale * 1.5);
		obj->setName("START");
		obj->enableShowName();
		obj->setColor_u8(options.color_start);
		obj->setPose(pi.start_pose);
		scene.insert(obj);
	}

	// Target:
	{
		mrpt::opengl::CSetOfObjects::Ptr obj =
			mrpt::opengl::stock_objects::CornerXYZ(xyzcorners_scale * 1.5);
		string m_name = "GOAL";
		obj->setName(m_name);
		obj->enableShowName();
		obj->setColor_u8(options.color_goal);
		obj->setPose(pi.goal_pose);
		scene.insert(obj);
	}

	// Log msg:
	if (!options.log_msg.empty())
	{
		mrpt::opengl::CText3D::Ptr gl_txt =
			mrpt::make_aligned_shared<mrpt::opengl::CText3D>(
				options.log_msg, "sans", options.log_msg_scale);
		gl_txt->setLocation(options.log_msg_position);
		scene.insert(gl_txt);
	}

}  // end renderMoveTree()
}  // namespace mrpt::nav
