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
		/** Input parameters for CAbstractHolonomicReactiveMethod::navigate() */
		struct NAV_IMPEXP NavInput
		{
			/** Distance to obstacles in polar coordinates, relative to the robot. 
			  * First index refers to -PI direction, and last one to +PI direction. 
			  * Distances can be dealed as "meters", although when used inside the PTG-based navigation system, they are "pseudometers", normalized to the range [0,1].
			  */
			std::vector<double>   obstacles;
			std::vector<mrpt::math::TPoint2D> targets; //!< Relative location (x,y) of target point(s). In the same units than `obstacles`. If many, last targets have higher priority.
			double                maxRobotSpeed;       //!< Maximum robot speed, in the same units than `obstacles`, per second.
			double                maxObstacleDist;     //!< Maximum expected value to be found in `obstacles`. Typically, values in `obstacles` larger or equal to this value mean there is no visible obstacle in that direction.
			const mrpt::nav::ClearanceDiagram *clearance;    //!< The computed clearance for each direction (optional in some implementations). Leave to default (NULL) if not needed.

			NavInput();
		};

		/** Output for CAbstractHolonomicReactiveMethod::navigate() */
		struct NAV_IMPEXP NavOutput
		{
			double    desiredDirection; //!< The desired motion direction, in the range [-PI, PI]
			double    desiredSpeed;     //!< The desired motion speed in that direction, from 0 up to NavInput::maxRobotSpeed

			/** The navigation method will create a log record and store it here via a smart pointer. Input value is ignored. */
			CHolonomicLogFileRecordPtr  logRecord;

			NavOutput();
		};


		/** Invokes the holonomic navigation algorithm itself. See the description of the input/output structures for details on each parameter. */
		virtual void navigate(const NavInput & ni, NavOutput &no) = 0;

		CAbstractHolonomicReactiveMethod(const std::string &defaultCfgSectionName);  //!< ctor
		virtual ~CAbstractHolonomicReactiveMethod(); //!< virtual dtor

		/** Initialize the parameters of the navigator, reading from the default section name (see derived classes) or the one set via setConfigFileSectionName() */
		virtual void  initialize( const mrpt::utils::CConfigFileBase &c ) = 0;
		virtual void saveConfigFile(mrpt::utils::CConfigFileBase &c) const = 0; //!< saves all available parameters, in a forma loadable by `initialize()`
		void setConfigFileSectionName(const std::string &sectName); //!< Defines the name of the section used in initialize()
		std::string getConfigFileSectionName() const; //!< Gets the name of the section used in initialize()

		virtual double getTargetApproachSlowDownDistance() const = 0; //!< Returns the actual value of this parameter [m], as set via the children class options structure. \sa setTargetApproachSlowDownDistance()
		virtual void setTargetApproachSlowDownDistance(const double dist) = 0; //!< Sets the actual value of this parameter [m]. \sa getTargetApproachSlowDownDistance()

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

