/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "nav-precomp.h" // Precomp header

#include <mrpt/nav/holonomic/ClearanceDiagram.h>
#include <mrpt/nav/tpspace/CParameterizedTrajectoryGenerator.h>
#include <mrpt/opengl/CMesh.h>
#include <limits>

using namespace mrpt::nav;

ClearanceDiagram::ClearanceDiagram()
{
}

void ClearanceDiagram::renderAs3DObject(
	mrpt::opengl::CMesh &mesh, double min_x, double max_x, double min_y, double max_y, double cell_res) const
{
	ASSERT_(cell_res>0.0);
	ASSERT_(max_x>min_x);
	ASSERT_(max_y>min_y);

	mesh.setXBounds(min_x, max_x);
	mesh.setYBounds(min_y, max_y);
	const int nX = (int)::ceil((max_x - min_x) / cell_res);
	const int nY = (int)::ceil((max_y - min_y) / cell_res);
	const double dx = (max_x - min_x) / nX;
	const double dy = (max_y - min_y) / nY;

	mrpt::math::CMatrixFloat Z(nX, nY);

	if (raw_clearances.empty())
		return; // Nothing to do: empty structure!

	const unsigned num_paths = raw_clearances.size();

	for (int iX = 0; iX < nX; iX++)
	{
		const double x = min_x + dx*(0.5+ iX);
		for (int iY = 0; iY < nY; iY++)
		{
			const double y = min_y + dy*(0.5 + iY);

			double clear_val = .0;
			if (x != 0 || y != 0)
			{
				const double alpha = ::atan2(y, x);
				const uint16_t k = CParameterizedTrajectoryGenerator::alpha2index(alpha, num_paths);
				const double dist = std::hypot(x, y);
				clear_val = this->getClearance(k, dist);
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

double ClearanceDiagram::getClearance(uint16_t k, double dist) const
{
	ASSERT_(k<raw_clearances.size());

	const auto & rc_k = raw_clearances[k];

	double res = std::numeric_limits<double>::max();
	bool valid = false;
	for (const auto &e : rc_k)
	{
		if (e.first == 0)
			continue;  // Ignore clearance at (0,0)

		if (e.first>dist)
			break; // target dist reached.
		
		// Keep min clearance along straight path:
		mrpt::utils::keep_min(res, e.second);
		valid = true;
	}

	if (!valid)
		res = rc_k.begin()->second;

	return res;
}

void ClearanceDiagram::clear()
{
	raw_clearances.clear();
}
