/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CAbstractHolonomicReactiveMethod_H
#define CAbstractHolonomicReactiveMethod_H

#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/utils/TEnumType.h>
#include <mrpt/utils/CConfigFileBase.h>

#include "CHolonomicLogFileRecord.h"

namespace mrpt
{
	namespace nav
	{
	/** \addtogroup nav_holo Holonomic navigation methods
	  * \ingroup mrpt_nav_grp
	  * @{ */
	  
	/**  The implemented reactive navigation methods. This enum works with mrpt::utils::TEnumType */
	enum THolonomicMethod
	{
		hmVIRTUAL_FORCE_FIELDS = 0,
		hmSEARCH_FOR_BEST_GAP = 1
	};

	/** A base class for holonomic reactive navigation methods.
	 *  \sa CHolonomicVFF,CHolonomicND, CReactiveNavigationSystem
	 */
	class NAV_IMPEXP CAbstractHolonomicReactiveMethod
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
		   *     <br><center><code>pseudometer<sup>2</sup>= meter<sup>2</sup> + (rad * r)<sup>2</sup></code><br></center>
		   */
		 virtual void  navigate(const mrpt::math::TPoint2D &target,
								const std::vector<float>	&obstacles,
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
	  /** @} */
  }
	// Specializations MUST occur at the same namespace:
	namespace utils
	{
		template <>
		struct TEnumTypeFiller<nav::THolonomicMethod>
		{
			typedef nav::THolonomicMethod enum_t;
			static void fill(bimap<enum_t,std::string>  &m_map)
			{
				m_map.insert(nav::hmVIRTUAL_FORCE_FIELDS, "hmVIRTUAL_FORCE_FIELDS");
				m_map.insert(nav::hmSEARCH_FOR_BEST_GAP, "hmSEARCH_FOR_BEST_GAP");
			}
		};
	} // End of namespace
}


#endif

