/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "nav-precomp.h"  // Precomp header
//
#include <mrpt/nav/reactive/TWaypoint.h>
#include <mrpt/opengl/CArrow.h>
#include <mrpt/opengl/CDisk.h>
#include <mrpt/opengl/CSetOfObjects.h>

#include <limits>

using namespace mrpt::nav;
using namespace std;

// TWaypoint  ==========
TWaypoint::TWaypoint(
	double target_x, double target_y, double allowed_distance_,
	bool allow_skip_, std::optional<double> target_heading_,
	double speed_ratio_)
	: target(target_x, target_y),
	  target_heading(target_heading_),
	  allowed_distance(allowed_distance_),
	  speed_ratio(speed_ratio_),
	  allow_skip(allow_skip_)
{
	// Backwards-compatibility:
	if (target_heading.has_value() && *target_heading == TWaypoint::INVALID_NUM)
		target_heading.reset();
}

bool TWaypoint::isValid() const
{
	return (target.x != INVALID_NUM) && (target.y != INVALID_NUM) &&
		(allowed_distance != INVALID_NUM);
}

std::string TWaypoint::getAsText() const
{
	std::string s;
	if (target.x != INVALID_NUM && target.y != INVALID_NUM)
		s += mrpt::format("target=(%8.03f,%8.03f) ", target.x, target.y);
	else
		s += "target=(**Coordinates not set!!**) ";

	if (target_heading.has_value())
		s += mrpt::format(
			"phi=%8.03f deg ", mrpt::RAD2DEG(target_heading.value()));
	else
		s += " (heading: any) ";

	if (allowed_distance != INVALID_NUM)
		s += mrpt::format("allowed_dist=%8.03f ", allowed_distance);
	else
		s += " (**allowed_distance not set!!**) ";

	s += (allow_skip ? " allow_skip: YES" : " allow_skip: NO ");

	s += mrpt::format(" speed_ratio: %.01f", speed_ratio);
	return s;
}

// TWaypointSequence ==========
TWaypointSequence::TWaypointSequence() = default;
// Gets navigation params as a human-readable format:
std::string TWaypointSequence::getAsText() const
{
	string s;
	s += mrpt::format(
		"List of %u waypoints:\n", static_cast<unsigned int>(waypoints.size()));
	unsigned int i = 0;
	for (const auto& wp : waypoints)
	{
		s += mrpt::format(" #%3u: ", i++);
		s += wp.getAsText();
		s += "\n";
	}
	return s;
}

// TWaypointStatus ==========
TWaypointStatus& TWaypointStatus::operator=(const TWaypoint& wp)
{
	TWaypoint::operator=(wp);
	return *this;
}
std::string TWaypointStatus::getAsText() const
{
	std::string s = TWaypoint::getAsText();
	s += mrpt::format(" reached=%s", (reached ? "YES" : "NO "));
	return s;
}

// TWaypointStatusSequence ======
std::string TWaypointStatusSequence::getAsText() const
{
	string s;
	s += mrpt::format(
		"Status for %u waypoints:\n",
		static_cast<unsigned int>(waypoints.size()));
	unsigned int i = 0;
	for (const auto& wp : waypoints)
	{
		s += mrpt::format(" #%3u: ", i++);
		s += wp.getAsText();
		s += "\n";
	}
	s += mrpt::format(
		" final_goal_reached:%s  waypoint_index_current_goal=%d\n",
		(final_goal_reached ? "YES" : "NO "), waypoint_index_current_goal);
	return s;
}

TWaypointsRenderingParams::TWaypointsRenderingParams()
	: color_regular(mrpt::img::TColor(0x00, 0x00, 0xff)),
	  color_current_goal(mrpt::img::TColor(0xff, 0x00, 0x20)),
	  color_reached(mrpt::img::TColor(0x00, 0x00, 0xc0, 0xd0))

{
}

void TWaypointSequence::getAsOpenglVisualization(
	mrpt::opengl::CSetOfObjects& obj,
	const mrpt::nav::TWaypointsRenderingParams& params) const
{
	obj.clear();
	unsigned int idx = 0;
	for (const auto& p : waypoints)
	{
		auto gl_pt = mrpt::opengl::CDisk::Create(
			p.allow_skip ? params.outter_radius
						 : params.outter_radius_non_skippable,
			p.allow_skip ? params.inner_radius
						 : params.inner_radius_non_skippable,
			15);
		gl_pt->setLocation(p.target.x, p.target.y, 0.01);
		gl_pt->setColor_u8(params.color_regular);
		if (params.show_labels)
		{
			gl_pt->setName(mrpt::format("WayPt #%2u", idx));
			gl_pt->enableShowName(true);
		}
		obj.insert(gl_pt);

		if (p.target_heading.has_value())
		{
			auto o = mrpt::opengl::CArrow::Create(
				0, 0, 0, params.heading_arrow_len, 0.0f, 0.0f);
			o->setPose(mrpt::poses::CPose3D(
				p.target.x, p.target.y, 0.02, p.target_heading.value(), 0, 0));
			obj.insert(o);
		}
		++idx;
	}
}

void TWaypointStatusSequence::getAsOpenglVisualization(
	mrpt::opengl::CSetOfObjects& obj,
	const mrpt::nav::TWaypointsRenderingParams& params) const
{
	obj.clear();
	{
		unsigned int idx = 0;
		for (const auto& p : waypoints)
		{
			const bool is_cur_goal = (int(idx) == waypoint_index_current_goal);

			mrpt::opengl::CDisk::Ptr gl_pt = mrpt::opengl::CDisk::Create(
				p.reached ? params.outter_radius_reached
						  : (p.allow_skip ? params.outter_radius
										  : params.outter_radius_non_skippable),
				p.reached ? params.inner_radius_reached
						  : (p.allow_skip ? params.inner_radius
										  : params.inner_radius_non_skippable),
				15);
			gl_pt->setLocation(p.target.x, p.target.y, 0.01);
			if (params.show_labels)
			{
				gl_pt->setName(mrpt::format(
					"WayPt #%2u Reach:%s", idx, p.reached ? "YES" : "NO"));
				gl_pt->enableShowName(true);
			}
			gl_pt->setColor_u8(
				is_cur_goal ? params.color_current_goal
							: (p.reached ? params.color_reached
										 : params.color_regular));
			obj.insert(gl_pt);

			if (p.target_heading.has_value())
			{
				auto o = mrpt::opengl::CArrow::Create(
					0, 0, 0, params.heading_arrow_len, 0.0f, 0.0f);
				o->setPose(mrpt::poses::CPose3D(
					p.target.x, p.target.y, 0.02, p.target_heading.value(), 0,
					0));
				obj.insert(o);
			}
			++idx;
		}
	}
}

void TWaypointSequence::save(
	mrpt::config::CConfigFileBase& c, const std::string& s) const
{
	const unsigned int N = waypoints.size();
	c.write(s, "waypoint_count", N);

	const int NP = 27;	// name padding

	for (unsigned int i = 0; i < N; i++)
	{
		const auto& wp = waypoints[i];

		c.write(
			s, mrpt::format("wp%03u_allowed_distance", i), wp.allowed_distance,
			NP);
		c.write(s, mrpt::format("wp%03u_allow_skip", i), wp.allow_skip, NP);
		c.write(s, mrpt::format("wp%03u_target_x", i), wp.target.x, NP);
		c.write(s, mrpt::format("wp%03u_target_y", i), wp.target.y, NP);
		c.write(
			s, mrpt::format("wp%03u_target_frame_id", i), wp.target_frame_id,
			NP);
		if (wp.target_heading.has_value())
			c.write(
				s, mrpt::format("wp%03u_target_heading", i), *wp.target_heading,
				NP);
		c.write(s, mrpt::format("wp%03u_speed_ratio", i), wp.speed_ratio, NP);
	}
}

void TWaypointSequence::load(
	const mrpt::config::CConfigFileBase& c, const std::string& s)
{
	this->clear();

	const unsigned int N = c.read_int(s, "waypoint_count", 0, true);
	waypoints.resize(N);

	for (unsigned int i = 0; i < N; i++)
	{
		auto& wp = waypoints[i];

		wp.allowed_distance = c.read_double(
			s, mrpt::format("wp%03u_allowed_distance", i), 0, true);
		wp.allow_skip =
			c.read_bool(s, mrpt::format("wp%03u_allow_skip", i), true, true);
		wp.target.x =
			c.read_double(s, mrpt::format("wp%03u_target_x", i), 0, true);
		wp.target.y =
			c.read_double(s, mrpt::format("wp%03u_target_y", i), 0, true);
		wp.target_frame_id = c.read_string(
			s, mrpt::format("wp%03u_target_frame_id", i), "map", false);

		const auto sectHeading = mrpt::format("wp%03u_target_heading", i);
		if (c.keyExists(s, sectHeading))
			wp.target_heading = c.read_double(
				s, sectHeading, mrpt::nav::TWaypoint::INVALID_NUM);

		wp.speed_ratio =
			c.read_double(s, mrpt::format("wp%03u_speed_ratio", i), 1.0, false);
	}
}
