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
	/** The pure virtual class that a user of CAbstractReactiveNavigationSystem-derived classes must implement in order to allow the navigator sense the world and send motion commands to the robot.
	  *
	  *  The user must define a new class derived from CReactiveInterfaceImplementation and reimplement
	  *   all pure virtual and the desired virtual methods according to the documentation in this class.
	  *
	  * \sa CReactiveNavigationSystem, CAbstractReactiveNavigationSystem
	  *  \ingroup nav_reactive
	  */
	class NAV_IMPEXP CReactiveInterfaceImplementation
	{
	public:
		/** Get the current pose and speeds of the robot.
		 *   \param curPose Current robot pose.
		 *   \param curV Current linear speed, in meters per second.
		 *	 \param curW Current angular speed, in radians per second.
		 * \return false on any error.
		 */
		virtual bool getCurrentPoseAndSpeeds( mrpt::poses::CPose2D &curPose, float &curV, float &curW) = 0;

		/** Change the instantaneous speeds of robot.
		 *   \param v Linear speed, in meters per second.
		 *	 \param w Angular speed, in radians per second.
		 * \return false on any error.
		 */
		virtual bool changeSpeeds( float v, float w ) = 0;

		/** Stop the robot right now.
		 * \return false on any error.
		 */
		virtual bool stop() {
			return changeSpeeds(0,0);
		}

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
		virtual bool senseObstacles( mrpt::maps::CSimplePointsMap 		&obstacles ) = 0;

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
		/** The struct for configuring navigation requests. See also: CAbstractPTGBasedReactive::TNavigationParamsPTG */
		struct NAV_IMPEXP TNavigationParams
		{
			mrpt::math::TPoint2D  target;  //!< Coordinates of desired target location.
			double                targetHeading; //!< Target location (heading, in radians).

			float                  targetAllowedDistance;    //!< Allowed distance to target in order to end the navigation.
			bool                   targetIsRelative;  //!< (Default=false) Whether the \a target coordinates are in global coordinates (false) or are relative to the current robot pose (true).

			TNavigationParams(); //!< Ctor with default values
			virtual ~TNavigationParams() {}
			virtual std::string getAsText() const; //!< Gets navigation params as a human-readable format
			virtual TNavigationParams* clone() const { return new TNavigationParams(*this); }
		};


		/** Constructor */
		CAbstractReactiveNavigationSystem( CReactiveInterfaceImplementation &react_iterf_impl );

        /** Destructor */
        virtual ~CAbstractReactiveNavigationSystem();

		/** Cancel current navegacion. */
		void cancel();

		/** Continues with suspended navigation. \sa suspend */
		void resume();

		/** This method must be called periodically in order to effectively run the navigation. */
		void navigationStep();

		/** Navigation request. It starts a new navigation.
		  * \param[in] params Pointer to structure with navigation info (its contents will be copied, so the original can be freely destroyed upon return.)
		 */
		virtual void  navigate( const TNavigationParams *params )=0;

		/** Suspend current navegation. \sa resume */
		virtual void  suspend();

		/** The different states for the navigation system. */
		enum TState
		{
			IDLE=0,
			NAVIGATING,
			SUSPENDED,
			NAV_ERROR
		};

		/** Returns the current navigator state. */
		inline TState getCurrentState() const { return m_navigationState; }

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

