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
#ifndef CHolonomicLogFileRecord_H
#define CHolonomicLogFileRecord_H

#include <mrpt/reactivenav/link_pragmas.h>
#include <mrpt/utils/CSerializable.h>

namespace mrpt
{
  namespace reactivenav
  {
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE(CHolonomicLogFileRecord, mrpt::utils::CSerializable, REACTIVENAV_IMPEXP)

	/** A base class for log records for different holonomic navigation methods.
	 *
	 * \sa CReactiveNavigationSystem, CHolonomicLogFileRecord
	  *  \ingroup mrpt_reactivenav_grp
	 */
	class REACTIVENAV_IMPEXP CHolonomicLogFileRecord : public utils::CSerializable
	{
		DEFINE_VIRTUAL_SERIALIZABLE( CHolonomicLogFileRecord )
	public:

	};

  }
}


#endif

