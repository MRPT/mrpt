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
#ifndef CHolonomicVFF_H
#define CHolonomicVFF_H

#include "CAbstractHolonomicReactiveMethod.h"
#include "CHolonomicLogFileRecord.h"

namespace mrpt
{
  namespace reactivenav
  {
	 DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CLogFileRecord_VFF, CHolonomicLogFileRecord, REACTIVENAV_IMPEXP )

	/** A class for storing extra information about the execution of
	 *    CHolonomicVFF navigation.
	 * \sa CHolonomicVFF, CHolonomicLogFileRecord
	 *  \ingroup mrpt_reactivenav_grp
	 */
	class REACTIVENAV_IMPEXP CLogFileRecord_VFF : public CHolonomicLogFileRecord
	{
		DEFINE_SERIALIZABLE( CLogFileRecord_VFF )
	 public:

		 /** Member data.
		   */

	};


	/** A holonomic reactive navigation method, based on Virtual Force Fields (VFF).
	 *  \sa CAbstractHolonomicReactiveMethod,CReactiveNavigationSystem
	 */
	class CHolonomicVFF : public CAbstractHolonomicReactiveMethod
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	public:
		 /**  Initialize the parameters of the navigator, from some configuration file, or default values if set to NULL.
		   */
		 CHolonomicVFF(const mrpt::utils::CConfigFileBase *INI_FILE=NULL);

		 /** This method performs the holonomic navigation itself.
		   *  \param target [IN] The relative location (x,y) of target point.
		   *  \param obstacles [IN] Distance to obstacles from robot location (0,0). First index refers to -PI direction, and last one to +PI direction. Distances can be dealed as "meters", although they are "pseudometers", see note below.
		   *  \param maxRobotSpeed [IN] Maximum robot speed, in "pseudometers/sec". See note below.
		   *  \param desiredDirection [OUT] The desired motion direction, in the range [-PI,PI]
		   *  \param desiredSpeed [OUT] The desired motion speed in that direction, in "pseudometers"/sec. (See note below)
		   *  \param logRecord [IN/OUT] A placeholder for a pointer to a log record with extra info about the execution. Set to NULL if not required. User <b>must free memory</b> using "delete logRecord" after using it.
		   *
		   *  NOTE: With "pseudometers" we refer to the distance unit in TP-Space, thus:
		   *     <br><center><code>pseudometer<sup>2</sup>= meter<sup>2</sup> + (rad · r)<sup>2</sup></code><br></center>
		   */
		 void  navigate(	poses::CPoint2D	&target,
							vector_double	&obstacles,
							double			maxRobotSpeed,
							double			&desiredDirection,
							double			&desiredSpeed,
							CHolonomicLogFileRecordPtr &logRecord );

		 /**  Initialize the parameters of the navigator.
		   */
		 void  initialize( const mrpt::utils::CConfigFileBase &INI_FILE  );

	};
  }
}


#endif



