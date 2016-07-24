/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"   // Precompiled headers

#include <mrpt/math/CAtan2LookUpTable.h>
#include <vector>
#include <cmath>

using namespace mrpt::math;

CAtan2LookUpTable::CAtan2LookUpTable()
{
	this->resize(-1.0,1.0,-1.0,1.0,0.5);
}
CAtan2LookUpTable::CAtan2LookUpTable(double xmin,double xmax,double ymin,double ymax, double resolution)
{
	this->resize(xmin,xmax,ymin,ymax,resolution);
}

void CAtan2LookUpTable::resize(double xmin,double xmax,double ymin,double ymax, double resolution)
{
	const double def = .0;
	m_grid.resize(xmin,xmax,ymin,ymax, def, .0);
	
	const size_t nx = m_grid.getSizeX(), ny = m_grid.getSizeY();

	std::vector<double> idx2x(nx), idx2y(ny);

	for (size_t ix = 0;ix<nx; ix++) idx2x[ix] = m_grid.idx2x(ix);
	for (size_t iy = 0;iy<ny; iy++) idx2y[iy] = m_grid.idx2y(iy);

	for (size_t ix = 0;ix<nx; ix++) 
	{
		const double x = idx2x[ix];
		for (size_t iy = 0;iy<ny; iy++)
		{
			const double y = idx2y[iy];
			double *cp = m_grid.cellByIndex(ix,iy);
			ASSERT_(cp!=NULL);
			*cp =::atan2(y,x);
		}
	}
}

bool CAtan2LookUpTable::atan2(double y,double x, double &out_atan2) const
{
	const double *cp = m_grid.cellByPos(x,y);
	if (!cp) return false;
	out_atan2 = *cp; 
	return true;
}



