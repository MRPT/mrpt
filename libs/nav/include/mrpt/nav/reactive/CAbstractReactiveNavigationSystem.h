/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CAbstractReactiveNavigationSystem_H
#define CAbstractReactiveNavigationSystem_H

#include <mrpt/poses/CPose2D.h>
#include <mrpt/utils/CDebugOutputCapable.h>
#include <mrpt/obs/obs_frwds.h>

#include <mrpt/nav/link_pragmas.h>

namespace mrpt
{
  namespace nav
  {
	/** The pure virtual interface between a real or simulated robot and any `CAbstractReactiveNavigationSystem`-derived class.
	  *
	  *  The user must define a new class derived from `CReactiveInterfaceImplementation` and reimplement
	  *   all pure virtual and the desired virtual methods according to the documentation in this class.
	  * 
	  * [New in MRPT 1.5.0] This class does not make assumptions about the kinematic model of the robot, so it can work with either 
	  *  Ackermann, differential-driven or holonomic robots. It will depend on the used PTGs, so checkout 
	  *  each PTG documentation for the lenght and meaning of velocity commands.
	  *
	  * \sa CReactiveNavigationSystem, CAbstractReactiveNavigationSystem
	  *  \ingroup nav_reactive
	  */
	class NAV_IMPEXP CReactiveInterfaceImplementation
	{
	public:
		/** Get the current pose and speeds of the robot. The implementation should not take too much time to return,
		*   so if it might take more than ~10ms to ask the robot for the instantaneous data, it may be good enough to
		*   return the latest values from a cache which is updated in a parallel thread.
		*
		* \param[out] curPose The latest robot pose, in world coordinates. (x,y: meters, phi: radians)
		* \param[out] curVel  The latest robot velocity vector, in world coordinates. (vx,vy: m/s, omega: rad/s)
		* \return false on any error retrieving these values from the robot.
		*/
		virtual bool getCurrentPoseAndSpeeds(mrpt::math::TPose2D &curPose, mrpt::math::TTwist2D &curVel) = 0;

		/** Sends a velocity command to the robot.
		 * Length and meaning of this vector is PTG-dependent.
		 * See children classes of mrpt::nav::CParameterizedTrajectoryGenerator
		 * \return false on any error.
		 */
		virtual bool changeSpeeds(const std::vector<double> &vel_cmd) = 0;

		/** Stop the robot right now.
		 * \return false on any error.
		 */
		virtual bool stop() = 0;

		/** Start the watchdog timer of the robot platform, if any.
		 * \param T_ms Period, in ms.
		 * \return false on any error.
		 */
		virtual bool startWatchdog(float T_ms) {
			MRPT_UNUSED_PARAM(T_ms);
			return true;
		}

		/** Stop the watchdog timer.
		 * \return false on any error.
		 */
		virtual bool stopWatchdog() { return true; }

		/** Return the current set of obstacle points, as seen from the local coordinate frame of the robot.
		  * \return false on any error.
		  */
		virtual bool senseObstacles( mrpt::maps::CSimplePointsMap &obstacles ) = 0;

		virtual void sendNavigationStartEvent () { std::cout << "[sendNavigationStartEvent] Not implemented by the user." << std::endl; }
		virtual void sendNavigationEndEvent() {	std::cout << "[sendNavigationEndEvent] Not implemented by the user." << std::endl; }
		virtual void sendNavigationEndDueToErrorEvent() { std::cout << "[sendNavigationEndDueToErrorEvent] Not implemented by the user." << std::endl; }
		virtual void sendWaySeemsBlockedEvent() { std::cout << "[sendWaySeemsBlockedEvent] Not implemented by the user." << std::endl; }
		virtual void notifyHeadingDirection(const double heading_dir_angle) {
			MRPT_UNUSED_PARAM(heading_dir_angle);
		}
	};



	/** This is the base class for any reactive navigation system. Here is defined
	 *   the interface that users will use with derived classes where algorithms are really implemented.
	 *
	 * Changes history:
	 *		- 30/JUN/2004: Creation (JLBC)
     *		- 16/SEP/2004: Totally redesigned.
	 *		- 15/SEP/2005: Totally rewritten again, for integration into MRPT Applications Repository.
	 *		-  3/NOV/2009: All functors are finally replaced by the new virtual class CReactiveInterfaceImplementation
	 *		- 16/DEC/2013: Refactoring of code in 2D & 2.5D navigators.
	 *
	 *   How to use:
	 *      - A class with callbacks must be defined by the user and provided to the constructor.
	 *      - navigationStep() must be called periodically in order to effectively run the navigation. This method will internally call the callbacks to gather sensor data and robot positioning data.
	 *
	 * \sa CReactiveNavigationSystem, CReactiveInterfaceImplementation
	 *  \ingroup nav_reactive
	 */
	class NAV_IMPEXP CAbstractReactiveNavigationSystem : public mrpt::utils::CDebugOutputCapable
	{
	public:
		/** The struct for configuring navigation requests. Used in CAbstractPTGBasedReactive::navigate() */
		struct NAV_IMPEXP TNavigationParams
		{
			mrpt::math::TPose2D target;  //!< Coordinates of desired target location. Heading may be ignored by some reactive implementations.
			float               targetAllowedDistance;    //!< (Default=0.5 meters) Allowed distance to target in order to end the navigation.
			bool                targetIsRelative;  //!< (Default=false) Whether the \a target coordinates are in global coordinates (false) or are relative to the current robot pose (true).

			TNavigationParams(); //!< Ctor with default values
			virtual ~TNavigationParams() {}
			virtual std::string getAsText() const; //!< Gets navigation params as a human-readable format
			virtual TNavigationParams* clone() const { return new TNavigationParams(*this); }
		};
		
		/** Constructor */
		CAbstractReactiveNavigationSystem( CReactiveInterfaceImplementation &react_iterf_impl );

		/** Destructor */
		virtual ~CAbstractReactiveNavigationSystem();

		/** \name Navigation control API
		  * @{ */
		void navigationStep(); //!< This method must be called periodically in order to effectively run the navigation
		
		/** Navigation request. It starts a new navigation.
		  * \param[in] params Pointer to structure with navigation info (its contents will be copied, so the original can be freely destroyed upon return.)
		  */
		virtual void  navigate( const TNavigationParams *params )=0;

		void cancel(); //!< Cancel current navegation.
		void resume(); //!< Continues with suspended navigation. \sa suspend
		virtual void  suspend(); //!< Suspend current navegation. \sa resume

		/** The different states for the navigation system. */
		enum TState {
			IDLE=0,
			NAVIGATING,
			SUSPENDED,
			NAV_ERROR
		};

		/** Returns the current navigator state. */
		inline TState getCurrentState() const { return m_navigationState; }

		/** @}*/

	private:
		TState  m_lastNavigationState; //!< Last internal state of navigator:

	protected:
		/** To be implemented in derived classes */
		virtual void  performNavigationStep( )=0;

		TState             m_navigationState;  //!< Current internal state of navigator:
		TNavigationParams  *m_navigationParams;  //!< Current navigation parameters

		CReactiveInterfaceImplementation   &m_robot; //!< The navigator-robot interface.

	public:
		MRPT_MAKE_ALIGNED_OPERATOR_NEW
	};
  }
}


#endif

