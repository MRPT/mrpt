/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  CTICTAC_H
#define  CTICTAC_H

#include <mrpt/base/link_pragmas.h>
#include <mrpt/utils/CUncopiable.h>

namespace mrpt
{
namespace utils
{
	/** This class implements a high-performance stopwatch.
	 *  Typical resolution is about 1e-6 seconds.
	 *  \note The class is named after the Spanish equivalent of "Tic-Toc" ;-)
	 * \ingroup mrpt_base_grp
	 */
	class BASE_IMPEXP CTicTac : public mrpt::utils::CUncopiable
	{
	public:
		CTicTac(); //!< Default constructor. Implicitly calls Tic()
		virtual ~CTicTac();  //!< Dtor
		void   Tic();  //!< Starts the stopwatch. \sa Tac
		double Tac();  //!< Stops the stopwatch.  \return Returns the ellapsed time in seconds.  \sa Tic
	private:
		unsigned char largeInts[64];
	}; // End of class def.

} // End of namespace
} // End of namespace
#endif
