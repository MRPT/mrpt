/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CObservationComment_H
#define CObservationComment_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/slam/CObservation.h>

namespace mrpt
{
namespace slam
{
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CObservationComment , CObservation, OBS_IMPEXP)


	/** This "observation" is actually a placeholder for a text block with comments or additional parameters attached to a given rawlog file.
	 *   There should be only one of this observations in a rawlog file, and it's recommended to insert/modify them from the application RawlogViewer.
	 *
	 * \sa CObservation
	 * \ingroup mrpt_obs_grp
	 */
	class OBS_IMPEXP CObservationComment : public CObservation
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CObservationComment )

	 public:
		/** Constructor.
		 */
		CObservationComment(  ) :
			text()
		{ }

		/** Destructor
		  */
		virtual ~CObservationComment()
		{ }

		/** The text block. */
		std::string text;

		// See base class docs
		void getSensorPose( CPose3D & ) const {  }
		// See base class docs
		void setSensorPose( const CPose3D & ) {  }

	}; // End of class def.
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CObservationComment , CObservation, OBS_IMPEXP)

	} // End of namespace
} // End of namespace

#endif
