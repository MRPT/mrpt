/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CAction_H
#define CAction_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/system/datetime.h>

#include <mrpt/obs/link_pragmas.h>


namespace mrpt
{
/** \ingroup mrpt_obs_grp */
namespace obs
{
	// This must be added to any CSerializable derived class:
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CAction, mrpt::utils::CSerializable, OBS_IMPEXP )

	/** Declares a class for storing a robot action. It is used in mrpt::obs::CRawlog,
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
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CAction, mrpt::utils::CSerializable, OBS_IMPEXP )


	} // End of namespace
} // End of namespace

#endif
