/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#ifndef CTICTAC_H
#define CTICTAC_H

#include <mrpt/base/link_pragmas.h>
#include <type_traits>

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
   public:
	/** Default constructor. Implicitly calls Tic() */
	CTicTac();
	CTicTac(const CTicTac&) = delete;
	CTicTac& operator=(const CTicTac&) = delete;
	/** Starts the stopwatch. \sa Tac */
	void Tic();
	/** Stops the stopwatch.  \return Returns the ellapsed time in seconds.  \sa
	 * Tic */
	double Tac();

   private:
	alignas(16) unsigned char largeInts[64];
};  // End of class def.
static_assert(
	!std::is_copy_constructible<CTicTac>::value &&
		!std::is_copy_assignable<CTicTac>::value,
	"Copy Check");

}  // End of namespace
}  // End of namespace
#endif
