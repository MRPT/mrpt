/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/nav/reactive/CRobot2NavInterface.h>
#include <mrpt/utils/CDebugOutputCapable.h>
#include <mrpt/obs/obs_frwds.h>

#include <mrpt/nav/link_pragmas.h>

namespace mrpt
{
  namespace nav
  {
	/** This is the base class for any reactive/planned navigation system. See derived classes.
	 *
	 * How to use:
	 *  - A class derived from `CRobot2NavInterface` with callbacks must be defined by the user and provided to the constructor.
	 *  - `navigationStep()` must be called periodically in order to effectively run the navigation. This method will internally call the callbacks to gather sensor data and robot positioning data.
	 *
	 * \sa CReactiveNavigationSystem, CRobot2NavInterface, all children classes
	 *  \ingroup nav_reactive
	 */
	class NAV_IMPEXP CAbstractNavigator : public mrpt::utils::CDebugOutputCapable
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
		CAbstractNavigator( CRobot2NavInterface &robot_interface_impl );

		/** Destructor */
		virtual ~CAbstractNavigator();

		/** \name Navigation control API
		  * @{ */
		virtual void loadConfigFile(const mrpt::utils::CConfigFileBase &cfg, const std::string &section_prefix="") = 0; //!< Loads the configuration from a file. To be called before initialize()
		virtual void initialize() = 0; //!<  Must be called before any other navigation command

		virtual void navigationStep(); //!< This method must be called periodically in order to effectively run the navigation
		
		/** Navigation request. It starts a new navigation.
		  * \param[in] params Pointer to structure with navigation info (its contents will be copied, so the original can be freely destroyed upon return.)
		  * \note A pointer is used so the passed object can be polymorphic with derived types.
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

		CRobot2NavInterface   &m_robot; //!< The navigator-robot interface.

	public:
		MRPT_MAKE_ALIGNED_OPERATOR_NEW
	};
  }
}

