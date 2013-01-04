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
	 * 
	 * These are the optional parameters of the method which can be set by means of a configuration file passed to the constructor or to CHolonomicND::initialize (see also the field CHolonomicVFF::options).
	 *
	 * \code
	 * [VFF_CONFIG]
	 * TARGET_SLOW_APPROACHING_DISTANCE = 0.10  // For stopping gradually
	 * TARGET_ATTRACTIVE_FORCE          = 20    // Dimension-less (may have to be tuned depending on the density of obstacle sampling)
	 * \endcode
	 * 
	 *  \sa CAbstractHolonomicReactiveMethod,CReactiveNavigationSystem
	 */
	class REACTIVENAV_IMPEXP CHolonomicVFF : public CAbstractHolonomicReactiveMethod
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
		void  navigate(	const mrpt::math::TPoint2D &target,
							const vector_double	&obstacles,
							double			maxRobotSpeed,
							double			&desiredDirection,
							double			&desiredSpeed,
							CHolonomicLogFileRecordPtr &logRecord );

		/**  Initialize the parameters of the navigator from section "VFF_CONFIG" of a config file. \sa options */
		void  initialize( const mrpt::utils::CConfigFileBase &INI_FILE )
		{
			options.loadFromConfigFile(INI_FILE, std::string("VFF_CONFIG"));
		}

		/** Algorithm options */
		struct REACTIVENAV_IMPEXP TOptions : public mrpt::utils::CLoadableOptions
		{
			double TARGET_SLOW_APPROACHING_DISTANCE; //!< For stopping gradually (Default: 0.10)
			double TARGET_ATTRACTIVE_FORCE;          //!< Dimension-less (may have to be tuned depending on the density of obstacle sampling) (Default: 20)

			TOptions();
			virtual void saveToConfigFile(const std::string &section,  mrpt::utils::CConfigFileBase &cfg ) const;
			virtual void loadFromConfigFile(const mrpt::utils::CConfigFileBase &source,const std::string &section);
		};

		TOptions options;  //!< Parameters of the algorithm (can be set manually or loaded from CHolonomicVFF::initialize or options.loadFromConfigFile(), etc.)

	};
  }
}


#endif



