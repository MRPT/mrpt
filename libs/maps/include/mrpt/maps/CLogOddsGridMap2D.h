/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/maps/CLogOddsGridMapLUT.h>
#include <mrpt/maps/logoddscell_traits.h>

namespace mrpt::maps
{
/** A generic provider of log-odds grid-map maintainance functions.
 *  Map cells must be type TCELL, which can be only:
 *		- int8_t or
 *		- int16_t
 *
 *  \sa CLogOddsGridMapLUT, See derived classes for usage examples.
 * \ingroup mrpt_maps_grp
 */
template <typename TCELL>
struct CLogOddsGridMap2D : public detail::logoddscell_traits<TCELL>
{
	/** The type of cells */
	using cell_t = TCELL;
	using traits_t = detail::logoddscell_traits<TCELL>;

	/** Performs the Bayesian fusion of a new observation of a cell, without
	 * checking for grid limits nor updateInfoChangeOnly.
	 * This method increases the "occupancy-ness" of a cell, managing possible
	 * saturation.
	 *  \param x Cell index in X axis.
	 *  \param y Cell index in Y axis.
	 *  \param logodd_obs Observation of the cell, in log-odd form as
	 * transformed by p2l.
	 *  \param thres  This must be CELLTYPE_MIN+logodd_obs
	 * \sa updateCell, updateCell_fast_free
	 */
	inline static void updateCell_fast_occupied(
		const unsigned x, const unsigned y, const cell_t logodd_obs,
		const cell_t thres, cell_t* mapArray, const unsigned _size_x)
	{
		cell_t* theCell = mapArray + (x + y * _size_x);
		if (*theCell > thres)
			*theCell -= logodd_obs;
		else
			*theCell = traits_t::CELLTYPE_MIN;
	}

	/** Performs the Bayesian fusion of a new observation of a cell, without
	 * checking for grid limits nor updateInfoChangeOnly.
	 * This method increases the "occupancy-ness" of a cell, managing possible
	 * saturation.
	 *  \param theCell The cell to modify
	 *  \param logodd_obs Observation of the cell, in log-odd form as
	 * transformed by p2l.
	 *  \param thres  This must be CELLTYPE_MIN+logodd_obs
	 * \sa updateCell, updateCell_fast_free
	 */
	inline static void updateCell_fast_occupied(
		cell_t* theCell, const cell_t logodd_obs, const cell_t thres)
	{
		if (*theCell > thres)
			*theCell -= logodd_obs;
		else
			*theCell = traits_t::CELLTYPE_MIN;
	}

	/** Performs the Bayesian fusion of a new observation of a cell, without
	 * checking for grid limits nor updateInfoChangeOnly.
	 * This method increases the "free-ness" of a cell, managing possible
	 * saturation.
	 *  \param x Cell index in X axis.
	 *  \param y Cell index in Y axis.
	 *  \param logodd_obs Observation of the cell, in log-odd form as
	 * transformed by p2l.
	 *  \param thres  This must be CELLTYPE_MAX-logodd_obs
	 * \sa updateCell_fast_occupied
	 */
	inline static void updateCell_fast_free(
		const unsigned x, const unsigned y, const cell_t logodd_obs,
		const cell_t thres, cell_t* mapArray, const unsigned _size_x)
	{
		cell_t* theCell = mapArray + (x + y * _size_x);
		if (*theCell < thres)
			*theCell += logodd_obs;
		else
			*theCell = traits_t::CELLTYPE_MAX;
	}

	/** Performs the Bayesian fusion of a new observation of a cell, without
	 * checking for grid limits nor updateInfoChangeOnly.
	 * This method increases the "free-ness" of a cell, managing possible
	 * saturation.
	 *  \param x Cell index in X axis.
	 *  \param y Cell index in Y axis.
	 *  \param logodd_obs Observation of the cell, in log-odd form as
	 * transformed by p2l.
	 *  \param thres  This must be CELLTYPE_MAX-logodd_obs
	 * \sa updateCell_fast_occupied
	 */
	inline static void updateCell_fast_free(
		cell_t* theCell, const cell_t logodd_obs, const cell_t thres)
	{
		if (*theCell < thres)
			*theCell += logodd_obs;
		else
			*theCell = traits_t::CELLTYPE_MAX;
	}

};  // end of CLogOddsGridMap2D

}  // namespace mrpt::maps
