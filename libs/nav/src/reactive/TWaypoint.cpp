/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "nav-precomp.h" // Precomp header

#include <mrpt/nav/reactive/TWaypoint.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CDisk.h>
#include <mrpt/opengl/CArrow.h>
#include <limits>

using namespace mrpt::nav;
using namespace std;

const double TWaypoint::INVALID_NUM = std::numeric_limits<double>::max();

// TWaypoint  ========== 
TWaypoint::TWaypoint() :
	target(INVALID_NUM,INVALID_NUM),
	target_heading(INVALID_NUM),
	target_frame_id("map"),
	allowed_distance(INVALID_NUM),
	allow_skip(true)
{
}

TWaypoint::TWaypoint(double target_x, double target_y, double allowed_distance_, bool allow_skip_, double target_heading_) :
	target(target_x,target_y),
	target_heading(target_heading_),
	target_frame_id("map"),
	allowed_distance(allowed_distance_),
	allow_skip(allow_skip_)
{
}

bool TWaypoint::isValid() const
{
	return 
	(target.x!=INVALID_NUM) && 
	(target.y!=INVALID_NUM) && 
	(allowed_distance!=INVALID_NUM);
}

std::string TWaypoint::getAsText() const
{
	std::string s;
	if (target.x!=INVALID_NUM && target.y!=INVALID_NUM)
		s+=mrpt::format("target=(%8.03f,%8.03f) ",target.x,target.y);
	else s+="target=(**Coordinates not set!!**) ";

	if (target_heading!=INVALID_NUM)
		s+=mrpt::format("phi=%8.03f deg ",mrpt::utils::RAD2DEG(target_heading));
	else s+=" (heading: any) ";

	if (allowed_distance!=INVALID_NUM)
		s+=mrpt::format("allowed_dist=%8.03f ",allowed_distance);
	else s+=" (**allowed_distance not set!!**) ";

	s+= (allow_skip ? " allow_skip: YES" : " allow_skip: NO ");

	return s;
}


// TWaypointSequence ========== 
TWaypointSequence::TWaypointSequence()
{
}

// Gets navigation params as a human-readable format:
std::string TWaypointSequence::getAsText() const 
{
	string s;
	s+=mrpt::format("List of %u waypoints:\n", static_cast<unsigned int>(waypoints.size()) );
	unsigned int i=0;
	for (const auto &wp : waypoints) {
		s+= mrpt::format(" #%3u: ",i++);
		s+= wp.getAsText();
		s+= "\n";
	}
	return s;
}

// TWaypointStatus ========== 
TWaypointStatus::TWaypointStatus() :
	reached(false),
	skipped(false),
	timestamp_reach(INVALID_TIMESTAMP),
	counter_seen_reachable(0)
{
}
TWaypointStatus & TWaypointStatus::operator =(const TWaypoint &wp)
{
	TWaypoint::operator =(wp);
	return *this;
}
std::string TWaypointStatus::getAsText() const 
{
	std::string s = TWaypoint::getAsText();
	s += mrpt::format(" reached=%s", (reached ? "YES":"NO ") );;
	return s;
}

// TWaypointStatusSequence ======
TWaypointStatusSequence::TWaypointStatusSequence() : 
	waypoints(),
	timestamp_nav_started(INVALID_TIMESTAMP),
	final_goal_reached(false),
	waypoint_index_current_goal(-1),
	last_robot_pose(TWaypoint::INVALID_NUM,TWaypoint::INVALID_NUM,TWaypoint::INVALID_NUM)
{
}

std::string TWaypointStatusSequence::getAsText() const
{
	string s;
	s+=mrpt::format("Status for %u waypoints:\n", static_cast<unsigned int>(waypoints.size()) );
	unsigned int i=0;
	for (const auto &wp : waypoints) {
		s+= mrpt::format(" #%3u: ",i++);
		s+= wp.getAsText();
		s+= "\n";
	}
	s+=mrpt::format(" final_goal_reached:%s  waypoint_index_current_goal=%d\n", (final_goal_reached ? "YES":"NO "), waypoint_index_current_goal );
	return s;
}

TWaypointsRenderingParams::TWaypointsRenderingParams() :
	outter_radius(.3), inner_radius(.2),
	outter_radius_non_skippable(.3), inner_radius_non_skippable(.0),
	outter_radius_reached(.2), inner_radius_reached(.1),
	heading_arrow_len(1.0),
	color_regular(mrpt::utils::TColor(0x00, 0x00, 0xff)),
	color_current_goal(mrpt::utils::TColor(0xff, 0x00, 0x20)),
	color_reached(mrpt::utils::TColor(0x00, 0x00, 0xc0,0xd0)),
	show_labels(true)
{
}


void TWaypointSequence::getAsOpenglVisualization(mrpt::opengl::CSetOfObjects &obj, const mrpt::nav::TWaypointsRenderingParams &params) const
{
	obj.clear();
	unsigned int idx = 0;
	for (const auto &p : waypoints)
	{
		auto gl_pt = mrpt::opengl::CDisk::Create(
			p.allow_skip ? params.outter_radius : params.outter_radius_non_skippable,
			p.allow_skip ? params.inner_radius : params.inner_radius_non_skippable,
			15);
		gl_pt->setLocation(p.target.x, p.target.y, 0.01);
		gl_pt->setColor_u8(params.color_regular);
		if (params.show_labels)
		{
			gl_pt->setName(mrpt::format("WayPt #%2u", idx));
			gl_pt->enableShowName(true);
		}
		obj.insert(gl_pt);

		if (p.target_heading != TWaypoint::INVALID_NUM)
		{
			auto o = mrpt::opengl::CArrow::Create(0, 0, 0, params.heading_arrow_len, 0.0f, 0.0f);
			o->setPose(mrpt::poses::CPose3D(p.target.x, p.target.y, 0.02, p.target_heading, 0, 0));
			obj.insert(o);
		}
		++idx;
	}
}

void TWaypointStatusSequence::getAsOpenglVisualization(mrpt::opengl::CSetOfObjects &obj, const mrpt::nav::TWaypointsRenderingParams &params) const
{
	obj.clear();
	{
		unsigned int idx = 0;
		for (const auto &p : waypoints)
		{
			const bool is_cur_goal = (int(idx) == waypoint_index_current_goal);

			mrpt::opengl::CDiskPtr gl_pt = mrpt::opengl::CDisk::Create(
				p.reached ? params.outter_radius_reached : (p.allow_skip ? params.outter_radius : params.outter_radius_non_skippable),
				p.reached ? params.inner_radius_reached  : (p.allow_skip ? params.inner_radius : params.inner_radius_non_skippable),
				15
				);
			gl_pt->setLocation(p.target.x, p.target.y, 0.01);
			if (params.show_labels)
			{
				gl_pt->setName(mrpt::format("WayPt #%2u Reach:%s", idx, p.reached ? "YES" : "NO"));
				gl_pt->enableShowName(true);
			}
			gl_pt->setColor_u8(is_cur_goal ? params.color_current_goal : (p.reached ? params.color_reached : params.color_regular));
			obj.insert(gl_pt);

			if (p.target_heading != TWaypoint::INVALID_NUM)
			{
				auto o = mrpt::opengl::CArrow::Create(0, 0, 0, params.heading_arrow_len, 0.0f, 0.0f);
				o->setPose(mrpt::poses::CPose3D(p.target.x, p.target.y, 0.02, p.target_heading, 0, 0));
				obj.insert(o);
			}
			++idx;
		}
	}
}
