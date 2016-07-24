/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/utils/CDynamicGrid.h>
#include <mrpt/base/link_pragmas.h>
#include <map>

namespace mrpt
{
namespace math
{
	/** A look-up-table (LUT) of atan values for any (x,y) value in a square/rectangular grid of predefined resolution
	  *
	  * \sa mrpt::math::CAtan2LookUpTableMultiRes, mrpt::obs::CSinCosLookUpTableFor2DScans
	 * \ingroup mrpt_base_grp
	  */
	class BASE_IMPEXP CAtan2LookUpTable
	{
	public:
		CAtan2LookUpTable();
		CAtan2LookUpTable(double xmin,double xmax,double ymin,double ymax, double resolution);
		void resize(double xmin,double xmax,double ymin,double ymax, double resolution);

		/** Returns the precomputed value for atan2(y,x). \return false if out of grid bounds. */
		bool atan2(double y,double x, double &out_atan2) const;

		double getXMin() const { return m_grid.getXMin(); }
		double getXMax() const { return m_grid.getXMax(); }
		double getYMin() const { return m_grid.getYMin(); }
		double getYMax() const { return m_grid.getYMax(); }
		double getResolution() const { return m_grid.getResolution(); }
	private:
		mrpt::utils::CDynamicGrid<double> m_grid;
	};

} // end NS math
} // end NS mrpt
