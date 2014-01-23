/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  CTICTAC_H
#define  CTICTAC_H

#include <mrpt/utils/utils_defs.h>

namespace mrpt
{
namespace utils
{
	/** This class implements a high-performance stopwatch.
	 *  Typical resolution is about 1e-6 seconds.
	 *  \note The class is named after the Spanish equivalent of "Tic-Toc" ;-)
		 * \ingroup mrpt_base_grp
	 */
	class BASE_IMPEXP CTicTac
	{
	private:
		uint8_t		largeInts[64];
	public:
		/** Default constructor.
		 */
		CTicTac();

		/** Destructor.
		 */
		virtual ~CTicTac();

		CTicTac(const CTicTac& o)
		{
			THROW_EXCEPTION("CTicTac objects cannot be copied.");
		}

		CTicTac & operator =(const CTicTac& o)
		{
			THROW_EXCEPTION("CTicTac objects cannot be copied.");
		}

		/** Starts the stopwatch
		 * \sa Tac
		 */
		void	Tic();

		/** Stops the stopwatch
		 * \return Returns the ellapsed time in seconds.
		 * \sa Tic
		 */
		double	Tac();

	}; // End of class def.

	} // End of namespace
} // End of namespace
#endif
