/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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
#ifndef  interface_OnlineSLAM_H
#define  interface_OnlineSLAM_H

#include <mrpt/slam/CObservation.h>
#include <mrpt/utils/CConfigFileBase.h>

#include <mrpt/interfaces/link_pragmas.h>


namespace mrpt
{
	/** The namespace for generic interfaces or "algorithm templates".
	  */
	namespace interfaces
	{
		DEFINE_MRPT_OBJECT_PRE_CUSTOM_LINKAGE(OnlineSLAM, INTERF_IMPEXP)


		/** A base generic interface for all SLAM methods capable of on-line operation.
		  *   It's also a class factory which allows creating a SLAM class just from its textual name.
		  *
		  *  The expected sequence of methods invoked when using a SLAM method is:
		  *
		  *  \code
		  *     OnlineSLAMPtr  slamMethod = OnlineSLAM::Create("icp-slam");
		  *     slamMethod->setParameters(...);
		  *     slamMethod->initialize();
		  *
		  *     while ( get observation o  ) {
		  *       slamMethod->processObservations(o);
		  *     }
		  *  \endcode
		  *
		  *  Naturally, you can also directly instantiate any specific SLAM method if you
		  *    don't need to use the class factory, e.g.:
		  *
		  *  \code
		  *     CICPSLAM slamMethod;
		  *     slamMethod.setParameters(...);
		  *     ...
		  *  \endcode
		  *
		  * \sa mrpt::interfaces
		  */
		class INTERF_IMPEXP OnlineSLAM : public mrpt::utils::CObject
		{
			DEFINE_VIRTUAL_MRPT_OBJECT(OnlineSLAM)

			public:
				OnlineSLAM(); //!< Default constructor
				virtual ~OnlineSLAM(); //!< Destructor

				/** @name Standard methods for all online SLAM implementations
				    @{  */

				/** Reset the state of the SLAM method to its initial state (typically: no map, the robot located at the origin, etc).
				  */
				virtual void reset() {  /* The default: do nothing */ }

				/**
				  */
				void initialize();

				/** @} */

			protected:

				/** Virtual method to be implemented...
				  */
				virtual void initialize_onlineslam() = 0;

				bool m_onlineslam_init_done;

		}; // end of class

	} // End of namespace
} // End of namespace

#endif
