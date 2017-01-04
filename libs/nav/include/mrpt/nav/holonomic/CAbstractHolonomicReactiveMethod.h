/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CAbstractHolonomicReactiveMethod_H
#define CAbstractHolonomicReactiveMethod_H

#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/utils/TEnumType.h>
#include <mrpt/utils/CConfigFileBase.h>
#include <mrpt/utils/CSerializable.h>
#include <mrpt/nav/tpspace/CParameterizedTrajectoryGenerator.h>
#include <mrpt/nav/holonomic/ClearanceDiagram.h>

#include "CHolonomicLogFileRecord.h"

namespace mrpt
{
	namespace nav
	{
	/** \addtogroup nav_holo Holonomic navigation methods
	  * \ingroup mrpt_nav_grp
	  * @{ */
	  
	/** The implemented reactive navigation methods. This enum works with mrpt::utils::TEnumType.
	  * Since MRPT 1.5.0 the preferred way to select a holonomic method is, instead of this enum, 
	  *  via the class factory method CAbstractHolonomicReactiveMethod::Create() via the class name. */
	enum THolonomicMethod
	{
		hmVIRTUAL_FORCE_FIELDS = 0,  //!< CHolonomicVFF
		hmSEARCH_FOR_BEST_GAP = 1,   //!< CHolonomicND
		hmFULL_EVAL = 2              //!< CHolonomicFullEval
	};

	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE(CAbstractHolonomicReactiveMethod, mrpt::utils::CSerializable, NAV_IMPEXP)

	/** A base class for holonomic reactive navigation methods.
	 *  \sa CHolonomicVFF,CHolonomicND,CHolonomicFullEval, CReactiveNavigationSystem
	 */
	class NAV_IMPEXP CAbstractHolonomicReactiveMethod :
		public mrpt::utils::CSerializable
	{
		DEFINE_VIRTUAL_SERIALIZABLE(CAbstractHolonomicReactiveMethod)
	public:
		 /** This method performs the holonomic navigation itself.
		   *  \param target [IN] The relative location (x,y) of target point.
		   *  \param obstacles [IN] Distance to obstacles from robot location (0,0). First index refers to -PI direction, and last one to +PI direction. Distances can be dealed as "meters", although they are "pseudometers", see note below, but normalized in the range [0,1]
		   *  \param maxRobotSpeed [IN] Maximum robot speed, in "pseudometers/sec". See note below.
		   *  \param desiredDirection [OUT] The desired motion direction, in the range [-PI,PI]
		   *  \param desiredSpeed [OUT] The desired motion speed in that direction, in "pseudometers"/sec. (See note below)
		   *  \param logRecord [IN/OUT] A placeholder for a pointer to a log record with extra info about the execution. Set to NULL if not required. User <b>must free memory</b> using "delete logRecord" after using it.
		   *  \param max_obstacle_dist[in] Maximum expected value to be found in `obstacles`. Typically, values in `obstacles` larger or equal to this value mean there is no visible obstacle in that direction.
		   *  \param clearance[in] The computed clearance for each direction (optional in some implementations).
		   *
		   *  NOTE: With "pseudometers" we refer to the distance unit in TP-Space, thus:
		   *     <br><center><code>pseudometer<sup>2</sup>= meter<sup>2</sup> + (rad * r)<sup>2</sup></code><br></center>
		   */
		virtual void  navigate(
			const mrpt::math::TPoint2D &target,
			const std::vector<double>	&obstacles,
			double			maxRobotSpeed,
			double			&desiredDirection,
			double			&desiredSpeed,
			CHolonomicLogFileRecordPtr &logRecord,
			const double    max_obstacle_dist,
			const mrpt::nav::ClearanceDiagram *clearance = NULL) = 0;

		/** Overload with a generic container for obstacles (for backwards compatibility with std::vector<float> and other future uses) */
		template <class OBSTACLES_LIST>
		void navigate(
			const mrpt::math::TPoint2D &target,
			const OBSTACLES_LIST &obstacles,
			double			maxRobotSpeed,
			double			&desiredDirection,
			double			&desiredSpeed,
			CHolonomicLogFileRecordPtr &logRecord,
			const double    max_obstacle_dist)
		{
			std::vector<double>  obs(obstacles.size());
			std::copy(obstacles.begin(), obstacles.end(), obs.begin() );
			this->navigate(target,obs, maxRobotSpeed,desiredDirection,desiredSpeed,logRecord,max_obstacle_dist);
		}

		CAbstractHolonomicReactiveMethod(const std::string &defaultCfgSectionName);  //!< ctor
		virtual ~CAbstractHolonomicReactiveMethod(); //!< virtual dtor

		/** Initialize the parameters of the navigator, reading from the default section name (see derived classes) or the one set via setConfigFileSectionName() */
		virtual void  initialize( const mrpt::utils::CConfigFileBase &INI_FILE  ) = 0;
		void setConfigFileSectionName(const std::string &sectName); //!< Defines the name of the section used in initialize()
		std::string getConfigFileSectionName() const; //!< Gets the name of the section used in initialize()

		/** Class factory from class name, e.g. `"CHolonomicVFF"`, etc.
		  * \exception std::logic_error On invalid or missing parameters. */
		static CAbstractHolonomicReactiveMethod * Create(const std::string &className) MRPT_NO_THROWS;

		void setAssociatedPTG(mrpt::nav::CParameterizedTrajectoryGenerator *ptg); //!< Optionally, sets the associated PTG, just in case a derived class requires this info (not required for methods where the robot kinematics are totally abstracted)
		mrpt::nav::CParameterizedTrajectoryGenerator * getAssociatedPTG() const; //!< Returns the pointer set by setAssociatedPTG()

		void enableApproachTargetSlowDown(bool enable) { m_enableApproachTargetSlowDown = enable; }
	protected:
		mrpt::nav::CParameterizedTrajectoryGenerator *m_associatedPTG; //!< If applicable, this will contain the argument of the most recent call to setAssociatedPTG()
		bool  m_enableApproachTargetSlowDown; //!< Whether to decrease speed when approaching target

	private:
		std::string m_cfgSectionName; //!< used in setConfigFileSectionName(), initialize()
	};
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CAbstractHolonomicReactiveMethod, mrpt::utils::CSerializable, NAV_IMPEXP )
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
				m_map.insert(nav::hmFULL_EVAL, "hmFULL_EVAL");
			}
		};
	} // End of namespace
}


#endif

