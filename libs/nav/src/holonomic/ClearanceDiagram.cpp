/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "nav-precomp.h"  // Precomp header

#include <mrpt/nav/holonomic/ClearanceDiagram.h>
#include <mrpt/nav/tpspace/CParameterizedTrajectoryGenerator.h>
#include <mrpt/opengl/CMesh.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/core/round.h>
#include <mrpt/serialization/stl_serialization.h>
#include <limits>

using namespace mrpt::nav;

ClearanceDiagram::ClearanceDiagram()

	= default;

void ClearanceDiagram::renderAs3DObject(
	mrpt::opengl::CMesh& mesh, double min_x, double max_x, double min_y,
	double max_y, double cell_res, bool integrate_over_path) const
{
	ASSERT_(cell_res > 0.0);
	ASSERT_(max_x > min_x);
	ASSERT_(max_y > min_y);

	mesh.setXBounds(min_x, max_x);
	mesh.setYBounds(min_y, max_y);
	const int nX = (int)::ceil((max_x - min_x) / cell_res);
	const int nY = (int)::ceil((max_y - min_y) / cell_res);
	const double dx = (max_x - min_x) / nX;
	const double dy = (max_y - min_y) / nY;

	mrpt::math::CMatrixFloat Z(nX, nY);

	if (m_raw_clearances.empty()) return;  // Nothing to do: empty structure!

	for (int iX = 0; iX < nX; iX++)
	{
		const double x = min_x + dx * (0.5 + iX);
		for (int iY = 0; iY < nY; iY++)
		{
			const double y = min_y + dy * (0.5 + iY);

			double clear_val = .0;
			if (x != 0 || y != 0)
			{
				const double alpha = ::atan2(y, x);
				const uint16_t actual_k =
					CParameterizedTrajectoryGenerator::alpha2index(
						alpha, m_actual_num_paths);
				const double dist = std::hypot(x, y);
				clear_val =
					this->getClearance(actual_k, dist, integrate_over_path);
			}
			Z(iX, iY) = clear_val;
		}
	}

	mesh.setZ(Z);
	mesh.enableColorFromZ(true);
	mesh.enableTransparency(true);
	mesh.setColorA_u8(0x50);
	mesh.enableWireFrame(false);
}

void mrpt::nav::ClearanceDiagram::readFromStream(
	mrpt::serialization::CArchive& in)
{
	uint8_t version;
	in >> version;
	switch (version)
	{
		case 0:
			uint32_t decim_num;
			in.ReadAsAndCastTo<uint32_t, size_t>(m_actual_num_paths);
			in >> decim_num;
			this->resize(m_actual_num_paths, decim_num);
			in >> m_raw_clearances;
			break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}

void mrpt::nav::ClearanceDiagram::writeToStream(
	mrpt::serialization::CArchive& out) const
{
	const uint8_t version = 0;
	out << version;

	out << uint32_t(m_actual_num_paths) << uint32_t(m_raw_clearances.size());
	out << m_raw_clearances;
}

ClearanceDiagram::dist2clearance_t& ClearanceDiagram::get_path_clearance(
	size_t actual_k)
{
	return m_raw_clearances[real_k_to_decimated_k(actual_k)];
}

const ClearanceDiagram::dist2clearance_t& ClearanceDiagram::get_path_clearance(
	size_t actual_k) const
{
	return m_raw_clearances[real_k_to_decimated_k(actual_k)];
}

size_t mrpt::nav::ClearanceDiagram::real_k_to_decimated_k(size_t k) const
{
	ASSERT_(m_actual_num_paths > 0 && !m_raw_clearances.empty());
	const size_t ret = mrpt::round(k * m_k_a2d);
	ASSERT_(ret < m_raw_clearances.size());
	return ret;
}

size_t mrpt::nav::ClearanceDiagram::decimated_k_to_real_k(size_t k) const
{
	ASSERT_(m_actual_num_paths > 0 && !m_raw_clearances.empty());
	const size_t ret = mrpt::round(k * m_k_d2a);
	ASSERT_(ret < m_actual_num_paths);
	return ret;
}

double ClearanceDiagram::getClearance(
	uint16_t actual_k, double dist, bool integrate_over_path) const
{
	if (this->empty())  // If we are not using clearance values, just return a
		// fixed value:
		return 0.0;

	ASSERT_BELOW_(actual_k, m_actual_num_paths);

	const size_t k = real_k_to_decimated_k(actual_k);

	const auto& rc_k = m_raw_clearances[k];

	double res = 0;
	int avr_count = 0;  // weighted avrg: closer to query points weight more
	// than at path start.
	for (const auto& e : rc_k)
	{
		if (!integrate_over_path)
		{
			// dont integrate: forget past part:
			res = 0;
			avr_count = 0;
		}
		// Keep min clearance along straight path:
		res += e.second;
		avr_count++;

		if (e.first > dist) break;  // target dist reached.
	}

	if (!avr_count)
	{
		res = rc_k.begin()->second;
	}
	else
	{
		res = res / avr_count;
	}
	return res;
}

void ClearanceDiagram::clear()
{
	m_actual_num_paths = 0;
	m_raw_clearances.clear();
	m_k_a2d = m_k_d2a = .0;
}

void mrpt::nav::ClearanceDiagram::resize(
	size_t actual_num_paths, size_t decimated_num_paths)
{
	if (decimated_num_paths == 0)
	{
		this->clear();
		return;
	}
	ASSERT_ABOVEEQ_(actual_num_paths, decimated_num_paths);

	m_actual_num_paths = actual_num_paths;
	m_raw_clearances.resize(decimated_num_paths);

	m_k_d2a = double(m_actual_num_paths - 1) / (m_raw_clearances.size() - 1);
	m_k_a2d = double(m_raw_clearances.size() - 1) / (m_actual_num_paths - 1);
}
