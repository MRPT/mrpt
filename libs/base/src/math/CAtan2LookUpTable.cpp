/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"   // Precompiled headers

#include <mrpt/math/CAtan2LookUpTable.h>
#include <vector>
#include <cmath>

using namespace mrpt::math;

CAtan2LookUpTable::CAtan2LookUpTable() MRPT_NO_THROWS
{
	this->resize(-1.0,1.0,-1.0,1.0,0.5);
}
CAtan2LookUpTable::CAtan2LookUpTable(double xmin,double xmax,double ymin,double ymax, double resolution) MRPT_NO_THROWS
{
	this->resize(xmin,xmax,ymin,ymax,resolution);
}

void CAtan2LookUpTable::resize(double xmin,double xmax,double ymin,double ymax, double resolution) MRPT_NO_THROWS
{
	const double def = .0;
	if (m_grid.getResolution()==resolution)
	     m_grid.resize(xmin,xmax,ymin,ymax, def, .0);
	else m_grid.setSize(xmin, xmax, ymin, ymax, resolution, &def);
	
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
			if(cp) *cp =::atan2(y,x);
		}
	}
}

bool CAtan2LookUpTable::atan2(double y,double x, double &out_atan2) const MRPT_NO_THROWS
{
	const double *cp = m_grid.cellByPos(x,y);
	if (!cp) return false;
	out_atan2 = *cp; 
	return true;
}



CAtan2LookUpTableMultiRes::CAtan2LookUpTableMultiRes() MRPT_NO_THROWS
{
}
CAtan2LookUpTableMultiRes::CAtan2LookUpTableMultiRes(const std::map<double,double> & lst_resolutions2extensions ) MRPT_NO_THROWS
{
	resize(lst_resolutions2extensions);
}
void CAtan2LookUpTableMultiRes::resize(const std::map<double,double> & lst_resolutions2extensions) MRPT_NO_THROWS
{
	m_grids.clear();

	for (const auto &it : lst_resolutions2extensions)
	{
		const double res = it.first;
		const double exten = it.second;

		mrpt::utils::CDynamicGrid<double> & dg = m_grids[exten] = mrpt::utils::CDynamicGrid<double>(-exten,exten,-exten,exten, res);

		const size_t nx = dg.getSizeX(), ny = dg.getSizeY();
		std::vector<double> idx2x(nx), idx2y(ny);

		for (size_t ix = 0;ix<nx; ix++) idx2x[ix] = dg.idx2x(ix);
		for (size_t iy = 0;iy<ny; iy++) idx2y[iy] = dg.idx2y(iy);

		for (size_t ix = 0;ix<nx; ix++) 
		{
			const double x = idx2x[ix];
			for (size_t iy = 0;iy<ny; iy++)
			{
				const double y = idx2y[iy];
				double *cp = dg.cellByIndex(ix,iy);
				if(cp) *cp =::atan2(y,x);
			}
		}
	}
}

bool CAtan2LookUpTableMultiRes::atan2(double y,double x, double &out_atan2) const MRPT_NO_THROWS
{
	for (const auto & it : m_grids)
	{
		const double exten = it.first;
		const mrpt::utils::CDynamicGrid<double> & dg = it.second;

		if (std::abs(x)>exten || std::abs(y)>exten)
			continue;

		const double *cp = dg.cellByPos(x,y);
		if (!cp) continue;
		out_atan2 = *cp; 
		return true;
	}
	return false;
}

