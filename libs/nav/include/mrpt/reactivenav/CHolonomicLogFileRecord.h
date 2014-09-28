/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CHolonomicLogFileRecord_H
#define CHolonomicLogFileRecord_H

#include <mrpt/nav/link_pragmas.h>
#include <mrpt/utils/CSerializable.h>

namespace mrpt
{
  namespace reactivenav
  {
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE(CHolonomicLogFileRecord, mrpt::utils::CSerializable, NAV_IMPEXP)

	/** A base class for log records for different holonomic navigation methods.
	 *
	 * \sa CReactiveNavigationSystem, CHolonomicLogFileRecord
	  *  \ingroup mrpt_nav_grp
	 */
	class NAV_IMPEXP CHolonomicLogFileRecord : public utils::CSerializable
	{
		DEFINE_VIRTUAL_SERIALIZABLE( CHolonomicLogFileRecord )
	public:

	};
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE(CHolonomicLogFileRecord, mrpt::utils::CSerializable, NAV_IMPEXP)

  }
}


#endif

