/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARMODE_H
#define ARMODE_H

#include "ariaTypedefs.h"
#include "ArFunctor.h"
#include <string>
#include <list>

class ArRobot;


/// A class for different modes, mostly as related to keyboard input
/**
  Each mode is going to need to add its keys to the keyHandler...
  each mode should only use the keys 1-0, the arrow keys (movement),
  the space bar (stop), z (zoom in), x (zoom out), and e (exercise)...
  then when its activate is called by that key handler it needs to
  first deactivate the ourActiveMode (if its not itself, in which case
  its done) then add its key handling stuff... activate and deactivate
  will need to add and remove their user tasks (or call the base class
  activate/deactivate to do it) as well as the key handling things for
  their other part of modes.  This mode will ALWAYS bind help to /, ?, h,
  and H when the first instance of an ArMode is made.
**/
class ArMode 
{
public:
  /// Constructor
  AREXPORT ArMode(ArRobot *robot, const char *name, char key, char key2);
  /// Destructor
  AREXPORT virtual ~ArMode();
  /// Gets the name of the mode
  AREXPORT const char *getName(void);
  /// The function called when the mode is activated, subclass must provide
  AREXPORT virtual void activate(void) = 0;
  /// The function called when the mode is deactivated, subclass must provide
  AREXPORT virtual void deactivate(void) = 0;
  /// The ArMode's user task, don't need one, subclass must provide if needed
  /*AREXPORT*/ virtual void userTask(void) {}
  /// The mode's help print out... subclass must provide if needed
  /** 
      This is called as soon as a mode is activated, and should give
      directions on to what keys do what and what this mode will do
  **/
  /*AREXPORT*/ virtual void help(void) {}
  /// The base activation, it MUST be called by inheriting classes,
  /// and inheriting classes MUST return if this returns false
  AREXPORT bool baseActivate(void); 
  /// The base deactivation, it MUST be called by inheriting classes,
  /// and inheriting classes MUST return if this returns false
  AREXPORT bool baseDeactivate(void);
  /// This is the base help function, its internal, bound to ? and h and H
  AREXPORT static void baseHelp(void);
  /// An internal function to get the first key this is bound to
  AREXPORT char getKey(void);
  /// An internal function to get the second key this is bound to
  AREXPORT char getKey2(void);
protected:
  void addKeyHandler(int keyToHandle, ArFunctor *functor);
  void remKeyHandler(ArFunctor *functor);
  // Our activeArMode
  static ArMode *ourActiveMode;
  std::string myName;
  ArRobot *myRobot;
  ArFunctorC<ArMode> myActivateCB;
  ArFunctorC<ArMode> myDeactivateCB;
  ArFunctorC<ArMode> myUserTaskCB;
  char myKey;
  char myKey2;
  // our help callback, its NULL until its initialized
  static ArGlobalFunctor *ourHelpCB;
  static std::list<ArMode *> ourModes;
};

#endif // ARMODE_H
