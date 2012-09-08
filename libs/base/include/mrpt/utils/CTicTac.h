/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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
