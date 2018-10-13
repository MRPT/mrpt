/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once
#include <mrpt/opengl/CDisk.h>
#include <mrpt/opengl/CPlanarLaserScan.h>

namespace mrpt::graphslam::deciders
{
// Ctors, Dtors
// //////////////////////////////////

template <class GRAPH_T>
CICPCriteriaERD<GRAPH_T>::CICPCriteriaERD()
	: params(*this),  // pass reference to self when initializing the parameters
	  m_search_disk_color(142, 142, 56),
	  m_laser_scans_color(0, 20, 255)

{
	MRPT_START;

	this->initializeLoggers("CICPCriteriaERD");

	// start ICP constraint registration only when
	m_edge_types_to_nums["ICP2D"] = 0;
	m_edge_types_to_nums["ICP3D"] = 0;
	m_edge_types_to_nums["LC"] = 0;

	this->m_last_total_num_nodes = 2;

	MRPT_LOG_DEBUG("Initialized class object");

	MRPT_END;
}

// Methods implementations
// //////////////////////////////////

template <class GRAPH_T>
bool CICPCriteriaERD<GRAPH_T>::updateState(
	mrpt::obs::CActionCollection::Ptr action,
	mrpt::obs::CSensoryFrame::Ptr observations,
	mrpt::obs::CObservation::Ptr observation)
{
	MRPT_START;
	MRPT_UNUSED_PARAM(action);
	using namespace mrpt::obs;

	// check possible prior node registration
	bool registered_new_node = false;

	if (this->m_last_total_num_nodes < this->m_graph->nodeCount())
	{
		registered_new_node = true;
		this->m_last_total_num_nodes = this->m_graph->nodeCount();
		MRPT_LOG_DEBUG("New node has been registered!");
	}

	if (observation)
	{  // observation-only rawlog format
		if (IS_CLASS(observation, CObservation2DRangeScan))
		{
			m_last_laser_scan2D =
				std::dynamic_pointer_cast<mrpt::obs::CObservation2DRangeScan>(
					observation);
			m_is_using_3DScan = false;
		}
		if (IS_CLASS(observation, CObservation3DRangeScan))
		{
			m_last_laser_scan3D =
				std::dynamic_pointer_cast<mrpt::obs::CObservation3DRangeScan>(
					observation);
			// just load the range/intensity images - CGraphSlanEngine takes
			// care
			// of the path
			m_last_laser_scan3D->load();

			// grab fake 2D range scan for visualization
			this->convert3DTo2DRangeScan(
				/*from = */ m_last_laser_scan3D,
				/*to   = */ &m_fake_laser_scan2D);

			m_is_using_3DScan = true;
		}

		// New node has been registered.
		// add the last laser_scan
		if (registered_new_node)
		{
			if (m_last_laser_scan2D)
			{
				this->m_nodes_to_laser_scans2D[this->m_graph->nodeCount() - 1] =
					m_last_laser_scan2D;
				MRPT_LOG_DEBUG(mrpt::format(
					"Added laser scans of nodeID: %u",
					(unsigned)(this->m_graph->nodeCount() - 1)));
			}
			if (m_last_laser_scan3D)
			{
				m_nodes_to_laser_scans3D[this->m_graph->nodeCount() - 1] =
					m_last_laser_scan3D;
				MRPT_LOG_DEBUG(mrpt::format(
					"Added laser scans of nodeID: %u",
					(unsigned)(this->m_graph->nodeCount() - 1)));
			}
		}
	}
	else
	{  // action-observations rawlog format
		// append current laser scan
		m_last_laser_scan2D =
			observations->getObservationByClass<CObservation2DRangeScan>();
		if (registered_new_node && m_last_laser_scan2D)
		{
			this->m_nodes_to_laser_scans2D[this->m_graph->nodeCount() - 1] =
				m_last_laser_scan2D;
		}
	}

	// edge registration procedure - same for both rawlog formats
	if (registered_new_node)
	{
		// get set of nodes within predefined distance for ICP
		std::set<mrpt::graphs::TNodeID> nodes_to_check_ICP;
		this->getNearbyNodesOf(
			&nodes_to_check_ICP, this->m_graph->nodeCount() - 1,
			params.ICP_max_distance);
		MRPT_LOG_DEBUG_FMT(
			"Found * %lu * nodes close to nodeID %lu",
			nodes_to_check_ICP.size(), this->m_graph->nodeCount() - 1);

		// reset the loop_closure flag and run registration
		this->m_just_inserted_lc = false;
		registered_new_node = false;

		if (m_is_using_3DScan)
		{
			checkRegistrationCondition3D(nodes_to_check_ICP);
		}
		else
		{
			checkRegistrationCondition2D(nodes_to_check_ICP);
		}
	}

	return true;
	MRPT_END;
}

template <class GRAPH_T>
void CICPCriteriaERD<GRAPH_T>::checkRegistrationCondition2D(
	const std::set<mrpt::graphs::TNodeID>& nodes_set)
{
	MRPT_START;
	using namespace mrpt;
	using namespace mrpt::obs;
	using namespace mrpt::math;

	mrpt::graphs::TNodeID curr_nodeID = this->m_graph->nodeCount() - 1;
	CObservation2DRangeScan::Ptr curr_laser_scan;
	typename nodes_to_scans2D_t::const_iterator search;

	// search for curr_laser_scan
	search = this->m_nodes_to_laser_scans2D.find(curr_nodeID);
	if (search != this->m_nodes_to_laser_scans2D.end())
	{
		curr_laser_scan = search->second;
	}

	// commence only if I have the current laser scan
	if (curr_laser_scan)
	{
		// try adding ICP constraints with each node in the previous set
		for (mrpt::graphs::TNodeID node_it : nodes_set)
		{
			// get the ICP edge between current and last node
			constraint_t rel_edge;
			mrpt::slam::CICP::TReturnInfo icp_info;
			CObservation2DRangeScan::Ptr prev_laser_scan;

			// search for prev_laser_scan
			search = this->m_nodes_to_laser_scans2D.find(node_it);
			if (search != this->m_nodes_to_laser_scans2D.end())
			{
				prev_laser_scan = search->second;

				// make use of initial node position difference for the ICP edge
				pose_t initial_pose = this->m_graph->nodes[curr_nodeID] -
									  this->m_graph->nodes[node_it];

				this->m_time_logger.enter("CICPCriteriaERD::getICPEdge");
				this->getICPEdge(
					*prev_laser_scan, *curr_laser_scan, &rel_edge,
					&initial_pose, &icp_info);
				this->m_time_logger.leave("CICPCriteriaERD::getICPEdge");

				// Debugging statements
				MRPT_LOG_DEBUG_STREAM(
					">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
					">>>>>>>>>");
				MRPT_LOG_DEBUG_STREAM(
					"ICP constraint between NON-successive nodes: "
					<< node_it << " => " << curr_nodeID << std::endl
					<< "\tnIterations = " << icp_info.nIterations
					<< "\tgoodness = " << icp_info.goodness);
				MRPT_LOG_DEBUG_STREAM(
					"ICP_goodness_thresh: " << params.ICP_goodness_thresh);
				MRPT_LOG_DEBUG_STREAM(
					"<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"
					"<<<<<<<<<");

				// criterion for registering a new node
				if (icp_info.goodness > params.ICP_goodness_thresh)
				{
					this->registerNewEdge(node_it, curr_nodeID, rel_edge);
					m_edge_types_to_nums["ICP2D"]++;
					// in case of loop closure
					if (absDiff(curr_nodeID, node_it) >
						params.LC_min_nodeid_diff)
					{
						m_edge_types_to_nums["LC"]++;
						this->m_just_inserted_lc = true;
					}
				}
			}
		}
	}

	MRPT_END;
}
template <class GRAPH_T>
void CICPCriteriaERD<GRAPH_T>::checkRegistrationCondition3D(
	const std::set<mrpt::graphs::TNodeID>& nodes_set)
{
	MRPT_START;
	using namespace std;
	using namespace mrpt::obs;
	using namespace mrpt::math;

	mrpt::graphs::TNodeID curr_nodeID = this->m_graph->nodeCount() - 1;
	CObservation3DRangeScan::Ptr curr_laser_scan;
	std::map<mrpt::graphs::TNodeID, mrpt::obs::CObservation3DRangeScan::Ptr>::
		const_iterator search;
	// search for curr_laser_scan
	search = m_nodes_to_laser_scans3D.find(curr_nodeID);
	if (search != m_nodes_to_laser_scans3D.end())
	{
		curr_laser_scan = search->second;
	}

	// commence only if I have the current laser scan
	if (curr_laser_scan)
	{
		// try adding ICP constraints with each node in the previous set
		for (mrpt::graphs::TNodeID node_it : nodes_set)
		{
			// get the ICP edge between current and last node
			constraint_t rel_edge;
			mrpt::slam::CICP::TReturnInfo icp_info;
			CObservation3DRangeScan::Ptr prev_laser_scan;

			// search for prev_laser_scan
			search = m_nodes_to_laser_scans3D.find(node_it);
			if (search != m_nodes_to_laser_scans3D.end())
			{
				prev_laser_scan = search->second;

				// TODO - use initial edge estimation
				this->m_time_logger.enter("CICPCriteriaERD::getICPEdge");
				this->getICPEdge(
					*prev_laser_scan, *curr_laser_scan, &rel_edge, nullptr,
					&icp_info);
				this->m_time_logger.leave("CICPCriteriaERD::getICPEdge");

				// criterion for registering a new node
				if (icp_info.goodness > params.ICP_goodness_thresh)
				{
					this->registerNewEdge(node_it, curr_nodeID, rel_edge);
					m_edge_types_to_nums["ICP3D"]++;
					// in case of loop closure
					if (absDiff(curr_nodeID, node_it) >
						params.LC_min_nodeid_diff)
					{
						m_edge_types_to_nums["LC"]++;
						this->m_just_inserted_lc = true;
					}
				}
			}
		}
	}

	MRPT_END;
}

template <class GRAPH_T>
void CICPCriteriaERD<GRAPH_T>::registerNewEdge(
	const mrpt::graphs::TNodeID& from, const mrpt::graphs::TNodeID& to,
	const constraint_t& rel_edge)
{
	parent_t::registerNewEdge(from, to, rel_edge);

	this->m_graph->insertEdge(from, to, rel_edge);
}

template <class GRAPH_T>
void CICPCriteriaERD<GRAPH_T>::getNearbyNodesOf(
	std::set<mrpt::graphs::TNodeID>* nodes_set,
	const mrpt::graphs::TNodeID& cur_nodeID, double distance)
{
	MRPT_START;

	if (distance > 0)
	{
		// check all but the last node.
		for (mrpt::graphs::TNodeID nodeID = 0;
			 nodeID < this->m_graph->nodeCount() - 1; ++nodeID)
		{
			double curr_distance = this->m_graph->nodes[nodeID].distanceTo(
				this->m_graph->nodes[cur_nodeID]);
			if (curr_distance <= distance)
			{
				nodes_set->insert(nodeID);
			}
		}
	}
	else
	{  // check against all nodes
		this->m_graph->getAllNodes(*nodes_set);
	}

	MRPT_END;
}

template <class GRAPH_T>
void CICPCriteriaERD<GRAPH_T>::notifyOfWindowEvents(
	const std::map<std::string, bool>& events_occurred)
{
	MRPT_START;
	parent_t::notifyOfWindowEvents(events_occurred);

	// I know the key exists - I put it there explicitly
	if (events_occurred.find(params.keystroke_laser_scans)->second)
	{
		this->toggleLaserScansVisualization();
	}

	MRPT_END;
}

template <class GRAPH_T>
void CICPCriteriaERD<GRAPH_T>::toggleLaserScansVisualization()
{
	MRPT_START;
	ASSERTDEBMSG_(this->m_win, "No CDisplayWindow3D* was provided");
	ASSERTDEBMSG_(this->m_win_manager, "No CWindowManager* was provided");

	using namespace mrpt::opengl;

	this->logFmt(
		mrpt::system::LVL_INFO, "Toggling LaserScans visualization...");

	COpenGLScene::Ptr scene = this->m_win->get3DSceneAndLock();

	if (params.visualize_laser_scans)
	{
		CRenderizable::Ptr obj = scene->getByName("laser_scan_viz");
		obj->setVisibility(!obj->isVisible());
	}
	else
	{
		dumpVisibilityErrorMsg("visualize_laser_scans");
	}

	this->m_win->unlockAccess3DScene();
	this->m_win->forceRepaint();

	MRPT_END;
}

template <class GRAPH_T>
void CICPCriteriaERD<GRAPH_T>::getEdgesStats(
	std::map<std::string, int>* edge_types_to_num) const
{
	MRPT_START;

	*edge_types_to_num = m_edge_types_to_nums;

	MRPT_END;
}

template <class GRAPH_T>
void CICPCriteriaERD<GRAPH_T>::initializeVisuals()
{
	MRPT_START;
	using namespace mrpt::opengl;
	this->logFmt(mrpt::system::LVL_DEBUG, "Initializing visuals");
	this->m_time_logger.enter("CICPCriteriaERD::Visuals");
	parent_t::initializeVisuals();

	ASSERTDEBMSG_(
		params.has_read_config, "Configuration parameters aren't loaded yet");

	this->m_win_observer->registerKeystroke(
		params.keystroke_laser_scans, "Toggle LaserScans Visualization");

	// ICP_max_distance disk
	if (params.ICP_max_distance > 0)
	{
		COpenGLScene::Ptr scene = this->m_win->get3DSceneAndLock();

		CDisk::Ptr obj = mrpt::make_aligned_shared<CDisk>();
		pose_t initial_pose;
		obj->setPose(initial_pose);
		obj->setName("ICP_max_distance");
		obj->setColor_u8(m_search_disk_color);
		obj->setDiskRadius(
			params.ICP_max_distance, params.ICP_max_distance - 0.1);
		scene->insert(obj);

		this->m_win->unlockAccess3DScene();
		this->m_win->forceRepaint();
	}

	// laser scan visualization
	if (params.visualize_laser_scans)
	{
		COpenGLScene::Ptr scene = this->m_win->get3DSceneAndLock();

		CPlanarLaserScan::Ptr laser_scan_viz =
			mrpt::make_aligned_shared<mrpt::opengl::CPlanarLaserScan>();
		laser_scan_viz->enablePoints(true);
		laser_scan_viz->enableLine(true);
		laser_scan_viz->enableSurface(true);
		laser_scan_viz->setSurfaceColor(
			m_laser_scans_color.R, m_laser_scans_color.G, m_laser_scans_color.B,
			m_laser_scans_color.A);

		laser_scan_viz->setName("laser_scan_viz");

		scene->insert(laser_scan_viz);
		this->m_win->unlockAccess3DScene();
		this->m_win->forceRepaint();
	}

	// max distance disk - textMessage
	if (this->m_win && this->m_win_manager && params.ICP_max_distance > 0)
	{
		this->m_win_manager->assignTextMessageParameters(
			&m_offset_y_search_disk, &m_text_index_search_disk);

		this->m_win_manager->addTextMessage(
			5, -m_offset_y_search_disk, mrpt::format("ICP Edges search radius"),
			mrpt::img::TColorf(m_search_disk_color),
			/* unique_index = */ m_text_index_search_disk);
	}

	this->m_time_logger.leave("CICPCriteriaERD::Visuals");
	MRPT_END;
}
template <class GRAPH_T>
void CICPCriteriaERD<GRAPH_T>::updateVisuals()
{
	MRPT_START;
	this->m_time_logger.enter("CICPCriteriaERD::Visuals");
	using namespace mrpt::opengl;
	using namespace mrpt::math;
	using namespace mrpt::poses;
	parent_t::updateVisuals();

	// update ICP_max_distance Disk
	if (this->m_win && params.ICP_max_distance > 0)
	{
		COpenGLScene::Ptr scene = this->m_win->get3DSceneAndLock();

		CRenderizable::Ptr obj = scene->getByName("ICP_max_distance");
		CDisk::Ptr disk_obj = std::dynamic_pointer_cast<CDisk>(obj);

		disk_obj->setPose(this->m_graph->nodes[this->m_graph->nodeCount() - 1]);

		this->m_win->unlockAccess3DScene();
		this->m_win->forceRepaint();
	}

	// update laser scan visual
	if (this->m_win && params.visualize_laser_scans &&
		(m_last_laser_scan2D || m_fake_laser_scan2D))
	{
		COpenGLScene::Ptr scene = this->m_win->get3DSceneAndLock();

		CRenderizable::Ptr obj = scene->getByName("laser_scan_viz");
		CPlanarLaserScan::Ptr laser_scan_viz =
			std::dynamic_pointer_cast<CPlanarLaserScan>(obj);

		// if fake 2D exists use it
		if (m_fake_laser_scan2D)
		{
			laser_scan_viz->setScan(*m_fake_laser_scan2D);
		}
		else
		{
			laser_scan_viz->setScan(*m_last_laser_scan2D);
		}

		// set the pose of the laser scan
		auto search = this->m_graph->nodes.find(this->m_graph->nodeCount() - 1);
		if (search != this->m_graph->nodes.end())
		{
			laser_scan_viz->setPose(
				this->m_graph->nodes[this->m_graph->nodeCount() - 1]);
			// put the laser scan *underneath* the graph, so that you can still
			// visualize the loop closures with the nodes ahead
			laser_scan_viz->setPose(CPose3D(
				laser_scan_viz->getPoseX(), laser_scan_viz->getPoseY(), -0.15,
				DEG2RAD(laser_scan_viz->getPoseYaw()),
				DEG2RAD(laser_scan_viz->getPosePitch()),
				DEG2RAD(laser_scan_viz->getPoseRoll())));
		}

		this->m_win->unlockAccess3DScene();
		this->m_win->forceRepaint();
	}

	this->m_time_logger.leave("CICPCriteriaERD::Visuals");
	MRPT_END;
}

template <class GRAPH_T>
void CICPCriteriaERD<GRAPH_T>::dumpVisibilityErrorMsg(
	std::string viz_flag, int sleep_time /* = 500 milliseconds */)
{
	MRPT_START;
	using namespace mrpt;

	MRPT_LOG_ERROR_FMT(
		"Cannot toggle visibility of specified object.\n "
		"Make sure that the corresponding visualization flag ( %s "
		") is set to true in the .ini file.\n",
		viz_flag.c_str());
	std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));

	MRPT_END;
}

template <class GRAPH_T>
void CICPCriteriaERD<GRAPH_T>::loadParams(const std::string& source_fname)
{
	MRPT_START;
	parent_t::loadParams(source_fname);

	params.loadFromConfigFileName(
		source_fname, "EdgeRegistrationDeciderParameters");
	MRPT_LOG_DEBUG("Successfully loaded parameters. ");

	// set the logging level if given by the user
	mrpt::config::CConfigFile source(source_fname);
	int min_verbosity_level = source.read_int(
		"EdgeRegistrationDeciderParameters", "class_verbosity", 1, false);
	this->setMinLoggingLevel(mrpt::system::VerbosityLevel(min_verbosity_level));

	MRPT_END;
}
template <class GRAPH_T>
void CICPCriteriaERD<GRAPH_T>::printParams() const
{
	MRPT_START;
	parent_t::printParams();
	params.dumpToConsole();

	MRPT_END;
}

template <class GRAPH_T>
void CICPCriteriaERD<GRAPH_T>::getDescriptiveReport(
	std::string* report_str) const
{
	MRPT_START;
	using namespace std;

	const std::string report_sep(2, '\n');
	const std::string header_sep(80, '#');

	// Report on graph
	stringstream class_props_ss;
	class_props_ss << "ICP Goodness-based Registration Procedure Summary: "
				   << std::endl;
	class_props_ss << header_sep << std::endl;

	// time and output logging
	const std::string time_res = this->m_time_logger.getStatsAsText();
	const std::string output_res = this->getLogAsString();

	// merge the individual reports
	report_str->clear();
	parent_t::getDescriptiveReport(report_str);

	*report_str += class_props_ss.str();
	*report_str += report_sep;

	*report_str += time_res;
	*report_str += report_sep;

	*report_str += output_res;
	*report_str += report_sep;

	MRPT_END;
}

// TParameter
// //////////////////////////////////

template <class GRAPH_T>
CICPCriteriaERD<GRAPH_T>::TParams::TParams(decider_t& d)
	: decider(d), keystroke_laser_scans("l"), has_read_config(false)
{
}

template <class GRAPH_T>
CICPCriteriaERD<GRAPH_T>::TParams::~TParams() = default;

template <class GRAPH_T>
void CICPCriteriaERD<GRAPH_T>::TParams::dumpToTextStream(
	std::ostream& out) const
{
	MRPT_START;

	out << mrpt::format(
		"------------------[ Goodness-based ICP Edge Registration "
		"]------------------\n");
	out << mrpt::format(
		"ICP goodness threshold         = %.2f%% \n",
		ICP_goodness_thresh * 100);
	out << mrpt::format(
		"ICP max radius for edge search = %.2f\n", ICP_max_distance);
	out << mrpt::format(
		"Min. node difference for LC    = %lu\n", LC_min_nodeid_diff);
	out << mrpt::format(
		"Visualize laser scans          = %d\n", visualize_laser_scans);
	out << mrpt::format(
		"3DScans Image Directory        = %s\n",
		scans_img_external_dir.c_str());

	MRPT_END;
}
template <class GRAPH_T>
void CICPCriteriaERD<GRAPH_T>::TParams::loadFromConfigFile(
	const mrpt::config::CConfigFileBase& source, const std::string& section)
{
	MRPT_START;

	LC_min_nodeid_diff = source.read_int(
		"GeneralConfiguration", "LC_min_nodeid_diff", 30, false);
	ICP_max_distance =
		source.read_double(section, "ICP_max_distance", 10, false);
	ICP_goodness_thresh =
		source.read_double(section, "ICP_goodness_thresh", 0.75, false);
	visualize_laser_scans = source.read_bool(
		"VisualizationParameters", "visualize_laser_scans", true, false);
	scans_img_external_dir = source.read_string(
		section, "scan_images_external_directory", "./", false);

	has_read_config = true;

	MRPT_END;
}
}  // namespace mrpt::graphslam::deciders
