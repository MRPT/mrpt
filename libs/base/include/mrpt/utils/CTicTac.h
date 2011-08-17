/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
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
