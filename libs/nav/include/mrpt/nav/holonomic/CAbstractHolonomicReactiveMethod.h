/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/typemeta/TEnumType.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/nav/tpspace/CParameterizedTrajectoryGenerator.h>
#include <mrpt/nav/holonomic/ClearanceDiagram.h>

#include "CHolonomicLogFileRecord.h"

namespace mrpt::nav
{
/** \addtogroup nav_holo Holonomic navigation methods
 * \ingroup mrpt_nav_grp
 * @{ */

/** A base class for holonomic reactive navigation methods.
 *  \sa CHolonomicVFF,CHolonomicND,CHolonomicFullEval, CReactiveNavigationSystem
 */
class CAbstractHolonomicReactiveMethod
	: public mrpt::serialization::CSerializable
{
	DEFINE_VIRTUAL_SERIALIZABLE(CAbstractHolonomicReactiveMethod)
   public:
	/** Input parameters for CAbstractHolonomicReactiveMethod::navigate() */
	struct NavInput
	{
		/** Distance to obstacles in polar coordinates, relative to the robot.
		 * First index refers to -PI direction, and last one to +PI direction.
		 * Distances can be dealed as "meters", although when used inside the
		 * PTG-based navigation system, they are "pseudometers", normalized to
		 * the range [0,1].
		 */
		std::vector<double> obstacles;
		/** Relative location (x,y) of target point(s). In the same units than
		 * `obstacles`. If many, last targets have higher priority. */
		std::vector<mrpt::math::TPoint2D> targets;
		/** Maximum robot speed, in the same units than `obstacles`, per second.
		 */
		double maxRobotSpeed{1.0};
		/** Maximum expected value to be found in `obstacles`. Typically, values
		 * in `obstacles` larger or equal to this value mean there is no visible
		 * obstacle in that direction. */
		double maxObstacleDist{1.0};
		/** The computed clearance for each direction (optional in some
		 * implementations). Leave to default (NULL) if not needed. */
		const mrpt::nav::ClearanceDiagram* clearance{nullptr};

		NavInput();
	};

	/** Output for CAbstractHolonomicReactiveMethod::navigate() */
	struct NavOutput
	{
		/** The desired motion direction, in the range [-PI, PI] */
		double desiredDirection{0};
		/** The desired motion speed in that direction, from 0 up to
		 * NavInput::maxRobotSpeed */
		double desiredSpeed{0};

		/** The navigation method will create a log record and store it here via
		 * a smart pointer. Input value is ignored. */
		CHolonomicLogFileRecord::Ptr logRecord;

		NavOutput();
	};

	static CAbstractHolonomicReactiveMethod::Ptr Factory(
		const std::string& className) noexcept;

	/** Invokes the holonomic navigation algorithm itself. See the description
	 * of the input/output structures for details on each parameter. */
	virtual void navigate(const NavInput& ni, NavOutput& no) = 0;

	/** ctor */
	CAbstractHolonomicReactiveMethod(const std::string& defaultCfgSectionName);
	/** virtual dtor */
	~CAbstractHolonomicReactiveMethod() override;

	/** Initialize the parameters of the navigator, reading from the default
	 * section name (see derived classes) or the one set via
	 * setConfigFileSectionName() */
	virtual void initialize(const mrpt::config::CConfigFileBase& c) = 0;
	/** saves all available parameters, in a forma loadable by `initialize()` */
	virtual void saveConfigFile(mrpt::config::CConfigFileBase& c) const = 0;
	/** Defines the name of the section used in initialize() */
	void setConfigFileSectionName(const std::string& sectName);
	/** Gets the name of the section used in initialize() */
	std::string getConfigFileSectionName() const;

	/** Returns the actual value of this parameter [m], as set via the children
	 * class options structure. \sa setTargetApproachSlowDownDistance() */
	virtual double getTargetApproachSlowDownDistance() const = 0;
	/** Sets the actual value of this parameter [m]. \sa
	 * getTargetApproachSlowDownDistance() */
	virtual void setTargetApproachSlowDownDistance(const double dist) = 0;

	/** Class factory from class name, e.g. `"CHolonomicVFF"`, etc.
	 * \exception std::logic_error On invalid or missing parameters. */
	static CAbstractHolonomicReactiveMethod* Create(
		const std::string& className) noexcept;

	/** Optionally, sets the associated PTG, just in case a derived class
	 * requires this info (not required for methods where the robot kinematics
	 * are totally abstracted) */
	void setAssociatedPTG(mrpt::nav::CParameterizedTrajectoryGenerator* ptg);
	/** Returns the pointer set by setAssociatedPTG() */
	mrpt::nav::CParameterizedTrajectoryGenerator* getAssociatedPTG() const;

	void enableApproachTargetSlowDown(bool enable)
	{
		m_enableApproachTargetSlowDown = enable;
	}

   protected:
	/** If applicable, this will contain the argument of the most recent call to
	 * setAssociatedPTG() */
	mrpt::nav::CParameterizedTrajectoryGenerator* m_associatedPTG;
	/** Whether to decrease speed when approaching target */
	bool m_enableApproachTargetSlowDown;

   private:
	/** used in setConfigFileSectionName(), initialize() */
	std::string m_cfgSectionName;
};
/** @} */
}  // namespace mrpt::nav
