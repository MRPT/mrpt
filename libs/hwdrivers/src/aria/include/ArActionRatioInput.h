/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARACTIONRATIOINPUT_H
#define ARACTIONRATIOINPUT_H

#include "ariaTypedefs.h"
#include "ArAction.h"

/// Action that requests motion based on abstract ratios provided by diferent input sources 
/**
   This action interprets input drive commands as three abstract ratios, 
   translation, rotation, and throttle. (In this way it mimics many joysticks.)
   The translation speed input ranges from -100 to 100, where -100 requests maximum
   backwards speed, and 100 requests maximum forward speed, 0 requests no
   translational speed, and values in between request a linear percentage of the maximum.
   Similarly, rotation speed input ranges from -100 to 100,
   where -100 indicates maximum rightwards or clockwise rotation, 100 indicates
   maximum leftwards or counter-clockwise rotation, 0 requests no rotation, and
   values in between request a linear percentage of the maximum.  The throttle
   input scales the other speed, and ranges from 0 (no motion) to 100 (maximum motion).
   
   Seperate objects (e.g. ArRatioInputKeydrive, ArRatioInputJoydrive, ArRAtionInputRobotJoydrive) 
   are used to provide input.

   When this action is activated it resets all its input ratios to 0
   (including throttle).

   Configuration parameters are used to map the maximum ratios to actual robot speeds.
   These are set be default to the robot's maximum configured velocities at startup but you can
   override them with ArConfig parameters (and call addToConfig()) or setParameters().
   
   @see ArRatioInputKeydrive
   @see ArRatioInputJoydrive
   @see ArRatioInputRobotJoydrive
 **/
class ArActionRatioInput : public ArAction
{
public:
  /// Constructor
  AREXPORT ArActionRatioInput(const char *name = "RatioInput");
  /// Destructor
  AREXPORT virtual ~ArActionRatioInput();
  /// Set ratios
  AREXPORT void setRatios(double transRatio, double rotRatio, 
			  double throttleRatio);
  /// Sets the trans ratio (from -100 (full backwards) to 100 (full forwards)
  AREXPORT void setTransRatio(double transRatio);
  /// Sets the rot ratio (from -100 (full right) to 100 (full left)
  AREXPORT void setRotRatio(double rotRatio);
  /// Sets the throttle ratio (from 0 (stopped) to 100 (full throttle)
  AREXPORT void setThrottleRatio(double throttleRatio);
  /// Gets the trans ratio (from -100 (full backwards) to 100 (full forwards)
  double getTransRatio(void) { return myTransRatio; }
  /// Gets the rot ratio (from -100 (full right) to 100 (full left)
  double getRotRatio(void) { return myRotRatio; }
  /// Gets the throttle ratio (from 0 (stopped) to 100 (full throttle)
  double getThrottleRatio(void) { return myThrottleRatio; }
  /// Adds a callback that is called from this actions fire call
  AREXPORT void addFireCallback(int priority, ArFunctor *functor);
  /// Removes a callback that was called from this actions fire callback
  AREXPORT void remFireCallback(ArFunctor *functor);
  /// Adds a callback that is called when this action is activated
  AREXPORT void addActivateCallback(ArFunctor *functor, 
				    ArListPos::Pos position = ArListPos::LAST);
  /// Removes a callback that was called when this action is activated
  AREXPORT void remActivateCallback(ArFunctor *functor);
  /// Adds a callback that is called when this action is deactivated
  AREXPORT void addDeactivateCallback(ArFunctor *functor, 
			      ArListPos::Pos position = ArListPos::LAST);
  /// Removes a callback that was called when this action is deactivated
  AREXPORT void remDeactivateCallback(ArFunctor *functor);
  /// Sets the parameters
  AREXPORT void setParameters(double fullThrottleForwards, 
			      double fullThrottleBackwards, 
			      double rotAtFullForwards,
			      double rotAtFullBackwards,
			      double rotAtStopped);
  /// Adds to a section in a config
  AREXPORT void addToConfig(ArConfig *config, const char *section);
  AREXPORT virtual ArActionDesired *fire(ArActionDesired currentDesired);
  /*AREXPORT*/ virtual ArActionDesired *getDesired(void) { return &myDesired; }
#ifndef SWIG
  /*AREXPORT*/ virtual const ArActionDesired *getDesired(void) const 
                                                        { return &myDesired; }
#endif
  AREXPORT virtual void activate(void);
  AREXPORT virtual void deactivate(void);
protected:
  std::multimap<int, ArFunctor *> myFireCallbacks;
  std::list<ArFunctor *> myActivateCallbacks;
  std::list<ArFunctor *> myDeactivateCallbacks;
  // if we're printing extra information or not
  bool myPrinting;
  double myTransDeadZone;
  double myRotDeadZone;
  double myFullThrottleForwards;
  double myFullThrottleBackwards; 
  double myRotAtFullForwards;
  double myRotAtFullBackwards;
  double myRotAtStopped;
  double myTransRatio;
  double myRotRatio;
  double myThrottleRatio;
  ArActionDesired myDesired;
};

#endif // ARACTIONSTOP_H
