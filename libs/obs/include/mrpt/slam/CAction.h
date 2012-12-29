/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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
