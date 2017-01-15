/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARACTION_H
#define ARACTION_H

#include "ariaTypedefs.h"
#include "ArArg.h"
#include "ArActionDesired.h"
#include <map>
#include <string>

class ArRobot;

/** @brief Base class for actions
  
    @ref actions Actions are objects queried for desired behavior by
    ArActionResolver to determine robot movement commands.

    To implement an action object, define a subclass of ArAction,
    and implement the fire() method. You may also override
    setRobot() to obtain information from ArRobot, but you
    must also call ArAction::setRobot() so that the ArRobot pointer
    is stored by ArAction.

    Several predefined action objects are also included in ARIA,
    they are listed here as ArActions's subclasses.

    If an action is not active (it has been deactivated), then
    it will be ignored by the action resolver. 
    Actions may be grouped using ArActionGroup, and activated/deactivated as a group. For example, ArMode, and ArServerMode (from ArNetworking), activate/deactivate action groups when switching modes.

    @see @ref actions description in the ARIA overview.
    @see ArActionGroup
    @see ArResolver
    @see ArRobot
*/
class ArAction
{
public:
  /// Constructor
  AREXPORT ArAction(const char * name, const char * description = "");
  /// Desructor
  AREXPORT virtual ~ArAction();
  /// Returns whether the action is active or not
  AREXPORT virtual bool isActive(void) const;
  /// Activate the action
  AREXPORT virtual void activate(void);
  /// Deactivate the action
  AREXPORT virtual void deactivate(void);
  /// Fires the action, returning what the action wants to do
  /** 
      @param currentDesired this is the tentative result, based
      on the resolver's processing of previous, higher-priority actions.
      This is only for the purpose of giving information to the 
      action, changing it has no effect.
      @return pointer to what this action wants to do, NULL if it wants to do 
      nothing. Common practice is to keep an ArActionDesired
      object in your action subclass, and return a pointer to
      that object. This avoids the need to create 
      new objects during each invocation (which could never
      be deleted).
      Clear your stored ArActionDesired
      before modifying it with ArActionDesired::reset().
  */
  AREXPORT virtual ArActionDesired *fire(ArActionDesired currentDesired) = 0;
  /// Sets the robot this action is driving
  AREXPORT virtual void setRobot(ArRobot *robot);
  /// Find the number of arguments this action takes
  AREXPORT virtual int getNumArgs(void) const;
#ifndef SWIG
  /** Gets the numbered argument
   * @swignote Not available
   */
  AREXPORT virtual const ArArg *getArg(int number) const;
#endif // SWIG
  /// Gets the numbered argument
  AREXPORT virtual ArArg *getArg(int number);
  /// Gets the name of the action
  AREXPORT virtual const char *getName(void) const;
  /// Gets the long description of the action
  AREXPORT virtual const char *getDescription(void) const;
  /// Gets what this action wants to do (for display purposes)
  /*AREXPORT*/ virtual ArActionDesired *getDesired(void) { return NULL; }
  /// Gets what this action wants to do (for display purposes)
  /*AREXPORT*/ virtual const ArActionDesired *getDesired(void) const { return NULL; }
  /// Log information about this action using ArLog.
  AREXPORT virtual void log(bool verbose = true) const;

  /// Get the robot we are controlling, which was set by setRobot()
  /*AREXPORT*/ ArRobot* getRobot() const { return myRobot; }

protected:  
  /// Sets the argument type for the next argument (must only be used in a constructor!)
  AREXPORT void setNextArgument(ArArg const &arg);

  /// The robot we are controlling, set by the action resolver using setRobot()
  ArRobot *myRobot;

  // These are mostly for internal use by ArAction, not subclasses:
  bool myIsActive;
  int myNumArgs;
  std::map<int, ArArg> myArgumentMap;
  std::string myName;
  std::string myDescription;
};


#endif //ARACTION_H
