/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "nav-precomp.h" // Precomp header

#include <mrpt/nav/holonomic/ClearanceDiagram.h>
#include <mrpt/opengl/CMesh.h>

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

	for (int iX = 0; iX < nX; iX++)
	{
		for (int iY = 0; iY < nY; iY++)
		{
			MRPT_TODO("finish render");

		}
	}
}

double ClearanceDiagram::getClearance(uint16_t k, double dist) const
{
	MRPT_TODO("continue");
	return .0;
}

void ClearanceDiagram::clear()
{
	raw_clearances.clear();
}
