/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
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
#ifndef CAbstractHolonomicReactiveMethod_H
#define CAbstractHolonomicReactiveMethod_H

#include <mrpt/utils.h>
#include <mrpt/poses.h>
#include "CHolonomicLogFileRecord.h"

namespace mrpt
{
	namespace reactivenav
	{
		using namespace mrpt::utils;
		using namespace mrpt::poses;

	/**  The implemented reactive navigation methods
	*  \ingroup mrpt_reactivenav_grp
	*/
	enum THolonomicMethod
	{
		hmVIRTUAL_FORCE_FIELDS = 0,
		hmSEARCH_FOR_BEST_GAP = 1
	};

	/** A base class for holonomic reactive navigation methods.
	 *  \sa CHolonomicVFF,CHolonomicND, CReactiveNavigationSystem
	 *  \ingroup mrpt_reactivenav_grp
	 */
	class REACTIVENAV_IMPEXP CAbstractHolonomicReactiveMethod
	{
	 public:
		 /** This method performs the holonomic navigation itself.
		   *  \param target [IN] The relative location (x,y) of target point.
		   *  \param obstacles [IN] Distance to obstacles from robot location (0,0). First index refers to -PI direction, and last one to +PI direction. Distances can be dealed as "meters", although they are "pseudometers", see note below, but normalized in the range [0,1]
		   *  \param maxRobotSpeed [IN] Maximum robot speed, in "pseudometers/sec". See note below.
		   *  \param desiredDirection [OUT] The desired motion direction, in the range [-PI,PI]
		   *  \param desiredSpeed [OUT] The desired motion speed in that direction, in "pseudometers"/sec. (See note below)
		   *  \param logRecord [IN/OUT] A placeholder for a pointer to a log record with extra info about the execution. Set to NULL if not required. User <b>must free memory</b> using "delete logRecord" after using it.
		   *
		   *  NOTE: With "pseudometers" we refer to the distance unit in TP-Space, thus:
		   *     <br><center><code>pseudometer<sup>2</sup>= meter<sup>2</sup> + (rad · r)<sup>2</sup></code><br></center>
		   */
		 virtual void  navigate(const mrpt::math::TPoint2D &target,
								const vector_double	&obstacles,
								double			maxRobotSpeed,
								double			&desiredDirection,
								double			&desiredSpeed,
								CHolonomicLogFileRecordPtr &logRecord) = 0;

        /** Virtual destructor
          */
        virtual ~CAbstractHolonomicReactiveMethod() { };

		 /**  Initialize the parameters of the navigator.
		   */
		 virtual void  initialize( const mrpt::utils::CConfigFileBase &INI_FILE  ) = 0;

	};
  }
}


#endif

