/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <vector>
#include <map>
#include <cstdint>
#include <mrpt/opengl/opengl_frwds.h>
#include <mrpt/serialization/serialization_frwds.h>

namespace mrpt::nav
{
/** Clearance information for one particular PTG and one set of obstacles.
 * Usage:
 * - Declare an object of this type (it will be initialized to "empty"),
 * - Call CParameterizedTrajectoryGenerator::initClearanceDiagram()
 * - Repeatedly call CParameterizedTrajectoryGenerator::updateClearance() for
 * each 2D obstacle point.
 *
 *  \ingroup nav_tpspace
 */
class ClearanceDiagram
{
   public:
	/** default ctor */
	ClearanceDiagram();
	/** Reset to default, empty state */
	void clear();
	/** Initializes the container to allocate `decimated_num_paths` entries, as
	 * a decimated
	 * subset of a total of `actual_num_paths` paths */
	void resize(size_t actual_num_paths, size_t decimated_num_paths);
	inline bool empty() const { return m_raw_clearances.empty(); }
	inline size_t get_actual_num_paths() const { return m_actual_num_paths; }
	inline size_t get_decimated_num_paths() const
	{
		return m_raw_clearances.size();
	}

	/** Gets the clearance for path `k` and distance `TPS_query_distance` in one
	 * of two modes:
	 * - [integrate_over_path=false] clearance from that specific spot, or
	 * - [integrate_over_path=true] average clearance over the path from the
	 * origin to that specific spot.
	 */
	double getClearance(
		uint16_t k, double TPS_query_distance, bool integrate_over_path) const;
	void renderAs3DObject(
		mrpt::opengl::CMesh& mesh, double min_x, double max_x, double min_y,
		double max_y, double cell_res, bool integrate_over_path) const;

	void readFromStream(mrpt::serialization::CArchive& in);
	void writeToStream(mrpt::serialization::CArchive& out) const;

	/** [TPS_distance] => normalized_clearance_for_exactly_that_robot_pose  */
	using dist2clearance_t = std::map<double, double>;
	dist2clearance_t& get_path_clearance(size_t actual_k);
	const dist2clearance_t& get_path_clearance(size_t actual_k) const;

	inline dist2clearance_t& get_path_clearance_decimated(size_t decim_k)
	{
		return m_raw_clearances[decim_k];
	}
	inline const dist2clearance_t& get_path_clearance_decimated(
		size_t decim_k) const
	{
		return m_raw_clearances[decim_k];
	}

	size_t real_k_to_decimated_k(size_t k) const;
	size_t decimated_k_to_real_k(size_t k) const;

   protected:
	/** Container: [decimated_path_k][TPS_distance] =>
	 * normalized_clearance_for_exactly_that_robot_pose  */
	std::vector<dist2clearance_t> m_raw_clearances;

	size_t m_actual_num_paths{
		0};  // The decimated number of paths is implicit in
	// raw_clearances.size()
	double m_k_a2d{.0}, m_k_d2a{.0};
};

}  // namespace mrpt::nav
