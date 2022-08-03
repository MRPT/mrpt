/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

namespace mrpt::system
{
/** A high-performance stopwatch, with typical resolution of nanoseconds.
 *
 * This always uses the system MONOTONIC clock, despite the setting in
 * mrpt::Clock.
 *
 *  \note The class is named after the Spanish equivalent of "Tic-Toc" ;-)
 * \ingroup mrpt_system_grp
 */
class CTicTac
{
   public:
	/** Default constructor. Implicitly calls Tic() */
	CTicTac() noexcept;
	/** Starts the stopwatch. \sa Tac() */
	void Tic() noexcept;
	/** Stops the stopwatch.  \return Returns the ellapsed time in seconds.
	 * \sa Tic() */
	double Tac() const noexcept;

   private:
	alignas(16) unsigned long largeInts[4]{0, 0};
};	// End of class def.

}  // namespace mrpt::system
