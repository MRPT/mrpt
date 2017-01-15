/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CLogOddsGridMap2D_H
#define CLogOddsGridMap2D_H

#include <mrpt/utils/core_defs.h>
#include <mrpt/utils/mrpt_stdint.h>
#include <vector>

namespace mrpt
{
	namespace maps
	{
		namespace detail
		{
			template <typename TCELL> struct logoddscell_traits;
			// Specializations:
			template <> struct logoddscell_traits<int8_t>
			{
				static const int8_t CELLTYPE_MIN  = -127; // In mrpt <0.9.4 was -128 (!) - This is to make it compatible with all architectures.
				static const int8_t CELLTYPE_MAX  = 127;
				static const int8_t P2LTABLE_SIZE = CELLTYPE_MAX;
				static const size_t LOGODDS_LUT_ENTRIES = 1<<8;
			};
			template <> struct logoddscell_traits<int16_t>
			{
				static const int16_t CELLTYPE_MIN  = -32767; // In mrpt <0.9.4 was -32768 (!).
				static const int16_t CELLTYPE_MAX  = 32767;
				static const int16_t P2LTABLE_SIZE = CELLTYPE_MAX;
				static const size_t  LOGODDS_LUT_ENTRIES = 1<<16;
			};
		}

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
			typedef TCELL cell_t; //!< The type of cells
			typedef detail::logoddscell_traits<TCELL>  traits_t;

			/** Performs the Bayesian fusion of a new observation of a cell, without checking for grid limits nor updateInfoChangeOnly.
			  * This method increases the "occupancy-ness" of a cell, managing possible saturation.
			  *  \param x Cell index in X axis.
			  *  \param y Cell index in Y axis.
			  *  \param logodd_obs Observation of the cell, in log-odd form as transformed by p2l.
			  *  \param thres  This must be CELLTYPE_MIN+logodd_obs
			  * \sa updateCell, updateCell_fast_free
			 */
			inline static void  updateCell_fast_occupied(
				const unsigned	x,
				const unsigned	y,
				const cell_t	logodd_obs,
				const cell_t  thres,
				cell_t		*mapArray,
				const unsigned	_size_x)
			{
				cell_t *theCell = mapArray + (x+y*_size_x);
				if (*theCell > thres )
						*theCell -= logodd_obs;
				else	*theCell = traits_t::CELLTYPE_MIN;
			}

			/** Performs the Bayesian fusion of a new observation of a cell, without checking for grid limits nor updateInfoChangeOnly.
			  * This method increases the "occupancy-ness" of a cell, managing possible saturation.
			  *  \param theCell The cell to modify
			  *  \param logodd_obs Observation of the cell, in log-odd form as transformed by p2l.
			  *  \param thres  This must be CELLTYPE_MIN+logodd_obs
			  * \sa updateCell, updateCell_fast_free
			 */
			inline static void  updateCell_fast_occupied(
				cell_t		*theCell,
				const cell_t	logodd_obs,
				const cell_t  thres )
			{
				if (*theCell > thres )
						*theCell -= logodd_obs;
				else	*theCell = traits_t::CELLTYPE_MIN;
			}

			/** Performs the Bayesian fusion of a new observation of a cell, without checking for grid limits nor updateInfoChangeOnly.
			  * This method increases the "free-ness" of a cell, managing possible saturation.
			  *  \param x Cell index in X axis.
			  *  \param y Cell index in Y axis.
			  *  \param logodd_obs Observation of the cell, in log-odd form as transformed by p2l.
			  *  \param thres  This must be CELLTYPE_MAX-logodd_obs
			  * \sa updateCell_fast_occupied
			 */
			inline static void  updateCell_fast_free(
				const unsigned	x,
				const unsigned	y,
				const cell_t	logodd_obs,
				const cell_t  thres,
				cell_t		*mapArray,
				const unsigned	_size_x)
			{
				cell_t *theCell = mapArray + (x+y*_size_x);
				if (*theCell < thres )
						*theCell += logodd_obs;
				else	*theCell = traits_t::CELLTYPE_MAX;
			}

			/** Performs the Bayesian fusion of a new observation of a cell, without checking for grid limits nor updateInfoChangeOnly.
			  * This method increases the "free-ness" of a cell, managing possible saturation.
			  *  \param x Cell index in X axis.
			  *  \param y Cell index in Y axis.
			  *  \param logodd_obs Observation of the cell, in log-odd form as transformed by p2l.
			  *  \param thres  This must be CELLTYPE_MAX-logodd_obs
			  * \sa updateCell_fast_occupied
			 */
			inline static void  updateCell_fast_free(
				cell_t		*theCell,
				const cell_t	logodd_obs,
				const cell_t  thres)
			{
				if (*theCell < thres )
						*theCell += logodd_obs;
				else	*theCell = traits_t::CELLTYPE_MAX;
			}

		};  // end of CLogOddsGridMap2D

		/** One static instance of this struct should exist in any class implementing CLogOddsGridMap2D to hold the Look-up-tables (LUTs) for log-odss Bayesian update.
		  *  Map cells must be type TCELL, which can be only:
		  *		- int8_t or
		  *		- int16_t
		  *
		  *  \sa CLogOddsGridMap2D, see derived classes for usage examples.
	  	  * \ingroup mrpt_maps_grp
		  */
		template <typename TCELL>
		struct CLogOddsGridMapLUT : public detail::logoddscell_traits<TCELL>
		{
			typedef TCELL cell_t; //!< The type of
			typedef detail::logoddscell_traits<TCELL>  traits_t;

			/** A lookup table to compute occupancy probabilities in [0,1] from integer log-odds values in the cells, using \f$ p(m_{xy}) = \frac{1}{1+exp(-log_odd)} \f$.
			  */
			std::vector<float>    logoddsTable;

			/** A lookup table to compute occupancy probabilities in the range [0,255] from integer log-odds values in the cells, using \f$ p(m_{xy}) = \frac{1}{1+exp(-log_odd)} \f$.
			  *  This is used to speed-up conversions to grayscale images.
			  */
			std::vector<uint8_t>   logoddsTable_255;

			/** A lookup table for passing from float to log-odds as cell_t. */
			std::vector<cell_t>    p2lTable;

			/** Constructor: computes all the required stuff. */
			CLogOddsGridMapLUT()
			{
				// The factor for converting log2-odds into integers:
				static const double LOGODD_K  = 16;
				static const double LOGODD_K_INV = 1.0/LOGODD_K;

				logoddsTable.resize( traits_t::LOGODDS_LUT_ENTRIES );
				logoddsTable_255.resize( traits_t::LOGODDS_LUT_ENTRIES );
				for (int i=traits_t::CELLTYPE_MIN;i<=traits_t::CELLTYPE_MAX;i++)
				{
					float f = 1.0f / (1.0f + exp( - i * LOGODD_K_INV ) );
					unsigned int idx =  -traits_t::CELLTYPE_MIN+i;
					logoddsTable[idx] = f;
					logoddsTable_255[idx] = (uint8_t)(f*255.0f);
				}

				// Build the p2lTable as well:
				p2lTable.resize( traits_t::P2LTABLE_SIZE+1 );
				double K = 1.0 / traits_t::P2LTABLE_SIZE;
				for (int j=0;j<=traits_t::P2LTABLE_SIZE;j++)
				{
					double p = j*K;
					if (p==0)
						p=1e-14;
					else if (p==1)
						p=1-1e-14;

					double logodd = log(p)-log(1-p);
					int   L = round(logodd * LOGODD_K);
					if (L>traits_t::CELLTYPE_MAX)
						L=traits_t::CELLTYPE_MAX;
					else if (L<traits_t::CELLTYPE_MIN)
						L=traits_t::CELLTYPE_MIN;
					p2lTable[j] = L;
				}
			}

			/** Scales an integer representation of the log-odd into a real valued probability in [0,1], using p=exp(l)/(1+exp(l))
			  */
			inline float l2p(const cell_t l)
			{
				if (l<traits_t::CELLTYPE_MIN)	
				     return logoddsTable[ 0 ];  // This is needed since min can be -127 and int8_t can be -128.
				else return logoddsTable[ -traits_t::CELLTYPE_MIN+l ];
			}

			/** Scales an integer representation of the log-odd into a linear scale [0,255], using p=exp(l)/(1+exp(l))
			  */
			inline uint8_t l2p_255(const cell_t l)
			{
				if (l<traits_t::CELLTYPE_MIN)	
				     return logoddsTable_255[ 0 ]; // This is needed since min can be -127 and int8_t can be -128.
				else return logoddsTable_255[ -traits_t::CELLTYPE_MIN+l ];
			}

			/** Scales a real valued probability in [0,1] to an integer representation of: log(p)-log(1-p)  in the valid range of cell_t.
			  */
			inline cell_t p2l(const float p)
			{
				return p2lTable[ static_cast<unsigned int>(p * traits_t::P2LTABLE_SIZE) ];
			}

		}; // end of CLogOddsGridMap2D

	} // End of namespace
} // End of namespace

#endif
