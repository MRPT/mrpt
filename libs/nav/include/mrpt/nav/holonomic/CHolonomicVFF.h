/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#ifndef CHolonomicVFF_H
#define CHolonomicVFF_H

#include "CAbstractHolonomicReactiveMethod.h"
#include "CHolonomicLogFileRecord.h"
#include <mrpt/utils/CLoadableOptions.h>


namespace mrpt
{
  namespace nav
  {
	/** \addtogroup nav_holo Holonomic navigation methods
	  * \ingroup mrpt_nav_grp
	  * @{ */

	/** A class for storing extra information about the execution of
	 *    CHolonomicVFF navigation.
	 * \sa CHolonomicVFF, CHolonomicLogFileRecord
	 */
	class NAV_IMPEXP CLogFileRecord_VFF : public CHolonomicLogFileRecord
	{
		DEFINE_SERIALIZABLE( CLogFileRecord_VFF )
	 public:

	};
	 DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CLogFileRecord_VFF, CHolonomicLogFileRecord, NAV_IMPEXP )


	/** A holonomic reactive navigation method, based on Virtual Force Fields (VFF).
	 *
	 * These are the optional parameters of the method which can be set by means of a configuration file passed to the constructor or to CHolonomicND::initialize (see also the field CHolonomicVFF::options).
	 *
	 * \code
	 * # Section name can be changed via setConfigFileSectionName()
	 * [VFF_CONFIG]
	 * TARGET_SLOW_APPROACHING_DISTANCE = 0.10  // For stopping gradually
	 * TARGET_ATTRACTIVE_FORCE          = 20    // Dimension-less (may have to be tuned depending on the density of obstacle sampling)
	 * \endcode
	 *
	 *  \sa CAbstractHolonomicReactiveMethod,CReactiveNavigationSystem
	 */
	class NAV_IMPEXP CHolonomicVFF : public CAbstractHolonomicReactiveMethod
	{
		DEFINE_SERIALIZABLE( CHolonomicVFF )
	public:
		/**  Initialize the parameters of the navigator, from some configuration file, or default values if set to NULL. */
		CHolonomicVFF(const mrpt::utils::CConfigFileBase *INI_FILE=nullptr);

		// See base class docs
		void navigate(const NavInput & ni, NavOutput &no) override;

		void initialize(const mrpt::utils::CConfigFileBase &INI_FILE) override; // See base class docs
		virtual void saveConfigFile(mrpt::utils::CConfigFileBase &c) const override; // See base class docs

		/** Algorithm options */
		struct NAV_IMPEXP TOptions : public mrpt::utils::CLoadableOptions
		{
			double TARGET_SLOW_APPROACHING_DISTANCE; //!< For stopping gradually (Default: 0.10)
			double TARGET_ATTRACTIVE_FORCE;          //!< Dimension-less (may have to be tuned depending on the density of obstacle sampling) (Default: 20)

			TOptions();
			void loadFromConfigFile(const mrpt::utils::CConfigFileBase &source,const std::string &section) override; // See base docs
			void saveToConfigFile(mrpt::utils::CConfigFileBase &cfg ,const std::string &section) const override; // See base docs
		};

		TOptions options;  //!< Parameters of the algorithm (can be set manually or loaded from CHolonomicVFF::initialize or options.loadFromConfigFile(), etc.)

		double getTargetApproachSlowDownDistance() const override { return options.TARGET_SLOW_APPROACHING_DISTANCE; }
		void setTargetApproachSlowDownDistance(const double dist) override { options.TARGET_SLOW_APPROACHING_DISTANCE = dist; }

	};
	 DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CHolonomicVFF, CAbstractHolonomicReactiveMethod, NAV_IMPEXP )
	  /** @} */
  }
}


#endif



