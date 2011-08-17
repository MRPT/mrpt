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
#ifndef CAction_H
#define CAction_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/system/datetime.h>

#include <mrpt/obs/link_pragmas.h>


namespace mrpt
{
/** \ingroup mrpt_obs_grp */
namespace slam
{
	// This must be added to any CSerializable derived class:
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CAction, mrpt::utils::CSerializable, OBS_IMPEXP )

	/** Declares a class for storing a robot action. It is used in mrpt::slam::CRawlog,
	 *    for logs storage and particle filter based simulations.
	 *  See derived classes for implementations.
	 *
	 * \sa CActionCollection, CRawlog
	 * \ingroup mrpt_obs_grp
	 */
	class OBS_IMPEXP CAction : public mrpt::utils::CSerializable
	{
		// This must be added to any CSerializable derived class:
		DEFINE_VIRTUAL_SERIALIZABLE( CAction )

		/** Default constructor
  		  */
		CAction();

		/** Constructor
  		  */
		virtual ~CAction();

		 /** The associated time-stamp.
		   *  This was added at 2-Dec-2007, new serialization versions have been added to derived classes to manage this time-stamp.
		   *  Prior versions will be read as having a INVALID_TIMESTAMP value.
		  */
		mrpt::system::TTimeStamp	timestamp;

	}; // End of class def.


	} // End of namespace
} // End of namespace

#endif
