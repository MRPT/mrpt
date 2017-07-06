/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#ifndef CHolonomicLogFileRecord_H
#define CHolonomicLogFileRecord_H

#include <mrpt/nav/link_pragmas.h>
#include <mrpt/utils/CSerializable.h>
#include <mrpt/math/CMatrixD.h>

namespace mrpt
{
  namespace nav
  {

	/** \addtogroup nav_holo Holonomic navigation methods
	  * \ingroup mrpt_nav_grp
	  * @{ */

	/** A base class for log records for different holonomic navigation methods.
	 *
	 * \sa CReactiveNavigationSystem, CHolonomicLogFileRecord
	 */
	class NAV_IMPEXP CHolonomicLogFileRecord : public utils::CSerializable
	{
		DEFINE_VIRTUAL_SERIALIZABLE( CHolonomicLogFileRecord )
	public:
		/** Final [0] and alternative [1..N] evaluation scores for each direction, in the same order of TP-Obstacles. May be not filled by all methods. */
		std::vector<std::vector<double> >  dirs_eval; 

		virtual const mrpt::math::CMatrixD * getDirectionScores() const { return nullptr; }
	};
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE(CHolonomicLogFileRecord, mrpt::utils::CSerializable, NAV_IMPEXP)

	  /** @} */
  }
}


#endif

