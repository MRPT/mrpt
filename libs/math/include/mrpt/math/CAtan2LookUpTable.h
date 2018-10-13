/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/containers/CDynamicGrid.h>
#include <map>

namespace mrpt::math
{
/** A look-up-table (LUT) of atan values for any (x,y) value in a
 * square/rectangular grid of predefined resolution
 *
 * \sa mrpt::math::CAtan2LookUpTableMultiRes,
 * mrpt::obs::CSinCosLookUpTableFor2DScans
 * \ingroup mrpt_math_grp
 */
class CAtan2LookUpTable
{
   public:
	CAtan2LookUpTable() noexcept;
	CAtan2LookUpTable(
		double xmin, double xmax, double ymin, double ymax,
		double resolution) noexcept;
	void resize(
		double xmin, double xmax, double ymin, double ymax,
		double resolution) noexcept;

	/** Returns the precomputed value for atan2(y,x). \return false if out of
	 * grid bounds. */
	bool atan2(double y, double x, double& out_atan2) const noexcept;

	double getXMin() const { return m_grid.getXMin(); }
	double getXMax() const { return m_grid.getXMax(); }
	double getYMin() const { return m_grid.getYMin(); }
	double getYMax() const { return m_grid.getYMax(); }
	double getResolution() const { return m_grid.getResolution(); }

   private:
	mrpt::containers::CDynamicGrid<double> m_grid;
};

/** Like CAtan2LookUpTable but with a multiresolution grid for increasingly
 * better accuracy in points nearer to the origin.
 * Example of usage:
 * \code
 *  mrpt::math::CAtan2LookUpTableMultiRes atan2lut;
 *  std::map<double,double> res2extension;
 *  res2extension[0.001] = 0.5;  // 0.1 cm resolution up to 0.5 m
 *  res2extension[0.01]  = 1.0;  //  1 cm resolution up to 1 m
 *  res2extension[0.02]  = 3.0;  //  2 cm resolution up to 3 m
 *  res2extension[0.05]  = 7.5;  //  5 cm resolution up to 7.5 m
 *  res2extension[0.10]  = 12.0; // 10 cm resolution up to  12 m
 *  atan2lut.resize(res2extension);
 * \endcode
 * \ingroup mrpt_math_grp
 */
class CAtan2LookUpTableMultiRes
{
   public:
	CAtan2LookUpTableMultiRes() = default;
	/** See CAtan2LookUpTableMultiRes for a discussion of the parameters */
	CAtan2LookUpTableMultiRes(
		const std::map<double, double>& lst_resolutions2extensions) noexcept;
	/** See CAtan2LookUpTableMultiRes for a discussion of the parameters */
	void resize(
		const std::map<double, double>& lst_resolutions2extensions) noexcept;

	/** Returns the precomputed value for atan2(y,x). \return false if out of
	 * grid bounds. */
	bool atan2(double y, double x, double& out_atan2) const noexcept;

   private:
	/** Maps from maximum (X,Y) coordinates to LUT for [-L,L]x[-L,L] square
	 * area. */
	std::map<double, mrpt::containers::CDynamicGrid<double>> m_grids;
};

}  // namespace mrpt::math
