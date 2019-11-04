/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/maps/logoddscell_traits.h>
#include <cmath>

namespace mrpt::maps
{
/** One static instance of this struct should exist in any class implementing
 * CLogOddsGridMap2D to hold the Look-up-tables (LUTs) for log-odss Bayesian
 *update. Map cells must be type TCELL, which can be only:
 *		- int8_t or
 *		- int16_t
 *
 *  \sa CLogOddsGridMap2D, see derived classes for usage examples.
 * \ingroup mrpt_maps_grp
 */
template <typename TCELL>
struct CLogOddsGridMapLUT : public detail::logoddscell_traits<TCELL>
{
	/** The type of */
	using cell_t = TCELL;
	using traits_t = detail::logoddscell_traits<TCELL>;

	/** A lookup table to compute occupancy probabilities in [0,1] from integer
	 * log-odds values in the cells, using \f$ p(m_{xy}) =
	 * \frac{1}{1+exp(-log_odd)} \f$.
	 */
	std::vector<float> logoddsTable;

	/** A lookup table to compute occupancy probabilities in the range [0,255]
	 * from integer log-odds values in the cells, using \f$ p(m_{xy}) =
	 * \frac{1}{1+exp(-log_odd)} \f$.
	 *  This is used to speed-up conversions to grayscale images.
	 */
	std::vector<uint8_t> logoddsTable_255;

	/** A lookup table for passing from float to log-odds as cell_t. */
	std::vector<cell_t> p2lTable;

	/** Constructor: computes all the required stuff. */
	CLogOddsGridMapLUT()
	{
		// The factor for converting log2-odds into integers:
		static const double LOGODD_K = 16;
		static const double LOGODD_K_INV = 1.0 / LOGODD_K;

		logoddsTable.resize(traits_t::LOGODDS_LUT_ENTRIES);
		logoddsTable_255.resize(traits_t::LOGODDS_LUT_ENTRIES);
		for (int i = traits_t::CELLTYPE_MIN; i <= traits_t::CELLTYPE_MAX; i++)
		{
			float f = 1.0f / (1.0f + std::exp(-i * LOGODD_K_INV));
			unsigned int idx = -traits_t::CELLTYPE_MIN + i;
			logoddsTable[idx] = f;
			logoddsTable_255[idx] = (uint8_t)(f * 255.0f);
		}

		// Build the p2lTable as well:
		p2lTable.resize(traits_t::P2LTABLE_SIZE + 1);
		const double K = 1.0 / traits_t::P2LTABLE_SIZE;
		for (int j = 0; j <= traits_t::P2LTABLE_SIZE; j++)
		{
			const double p = std::min(1.0 - 1e-14, std::max(1e-14, j * K));
			const double logodd = log(p) - log(1 - p);
			int L = round(logodd * LOGODD_K);
			if (L > traits_t::CELLTYPE_MAX)
				L = traits_t::CELLTYPE_MAX;
			else if (L < traits_t::CELLTYPE_MIN)
				L = traits_t::CELLTYPE_MIN;
			p2lTable[j] = L;
		}
	}

	/** Scales an integer representation of the log-odd into a real valued
	 * probability in [0,1], using p=exp(l)/(1+exp(l))
	 */
	inline float l2p(const cell_t l)
	{
		if (l < traits_t::CELLTYPE_MIN)
			// This is needed since min can be -127 and int8_t can be -128.
			return logoddsTable[0];
		else
			return logoddsTable[-traits_t::CELLTYPE_MIN + l];
	}

	/** Scales an integer representation of the log-odd into a linear scale
	 * [0,255], using p=exp(l)/(1+exp(l))
	 */
	inline uint8_t l2p_255(const cell_t l)
	{
		if (l < traits_t::CELLTYPE_MIN)
			// This is needed since min can be -127 and int8_t can be -128.
			return logoddsTable_255[0];
		else
			return logoddsTable_255[-traits_t::CELLTYPE_MIN + l];
	}

	/** Scales a real valued probability in [0,1] to an integer representation
	 * of: log(p)-log(1-p)  in the valid range of cell_t.
	 */
	inline cell_t p2l(const float p)
	{
		return p2lTable[static_cast<unsigned int>(p * traits_t::P2LTABLE_SIZE)];
	}
};

}  // namespace mrpt::maps
