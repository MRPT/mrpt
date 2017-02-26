/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARACTIONDESIRED_H
#define ARACTIONDESIRED_H

#include "ariaUtil.h"

/// Class used by ArActionDesired for each channel, internal
class ArActionDesiredChannel
{
public:
  AREXPORT static const double NO_STRENGTH;
  AREXPORT static const double MIN_STRENGTH;
  AREXPORT static const double MAX_STRENGTH;
  
  ArActionDesiredChannel() { reset(); myOverrideDoesLessThan = true; }
  ~ArActionDesiredChannel() {}
  void setOverrideDoesLessThan(bool overrideDoesLessThan) 
    { myOverrideDoesLessThan = overrideDoesLessThan; }
  void setDesired(double desired, double desiredStrength, 
		  bool allowOverride = false) 
    {
      myDesired = desired;
      myStrength = desiredStrength; 
      myAllowOverride = allowOverride;
      if (myStrength > MAX_STRENGTH)
	myStrength = MAX_STRENGTH;
      if (myStrength < MIN_STRENGTH)
	myStrength = NO_STRENGTH;
    }
  double getDesired(void) const { return myDesired; }
  double getStrength(void) const { return myStrength; }
  double getAllowOverride(void) const { return myAllowOverride; }
  void reset(void) 
    { myDesired = 0; myStrength = NO_STRENGTH; myAllowOverride = true; }
  void merge(ArActionDesiredChannel *desiredChannel)
    {
      double otherStrength = desiredChannel->getStrength();
      double oldStrength = myStrength;
      if (myStrength + otherStrength > MAX_STRENGTH)
	otherStrength = MAX_STRENGTH - myStrength;
      myStrength = myStrength + otherStrength;
      myAllowOverride = myAllowOverride && desiredChannel->getAllowOverride();
      // if we're allowing override just set myDesired to the least
      // (or greatest) value
      if (myAllowOverride && myStrength >= MIN_STRENGTH)
      {
	// if both have strength get the min/max
	if (oldStrength >= MIN_STRENGTH && 
	    desiredChannel->getStrength() >= MIN_STRENGTH)
	{
	  if (myOverrideDoesLessThan)
	    myDesired = ArUtil::findMin(myDesired, 
					desiredChannel->getDesired());
	  else if (!myOverrideDoesLessThan)
	    myDesired = ArUtil::findMax(myDesired, 
					desiredChannel->getDesired());
	}
	// if only it has strength use it
	else if (desiredChannel->getStrength() >= MIN_STRENGTH)
	{
	  myDesired = desiredChannel->getDesired();
	}
	// if only this has strength then we don't need to do anything
      }
      else if (myStrength >= MIN_STRENGTH)
	myDesired = (((oldStrength * myDesired) + 
		      (desiredChannel->getDesired() * otherStrength)) 
		     / (myStrength));
    }
  void startAverage(void)
    {
      myDesiredTotal = myDesired * myStrength;
      myStrengthTotal = myStrength;
    }
  void addAverage(ArActionDesiredChannel *desiredChannel)
    {
      myAllowOverride = myAllowOverride && desiredChannel->getAllowOverride();
      // if we're allowing override then myDesired is just the least
      // of the values thats going to come through... still compute
      // the old way in case something doesn't want to override it
      if (myAllowOverride)
      {
	// if both have strength get the min/max
	if (myStrength >= MIN_STRENGTH && 
	    desiredChannel->getStrength() >= MIN_STRENGTH)
	{
	  if (myOverrideDoesLessThan)
	    myDesired = ArUtil::findMin(myDesired, 
					desiredChannel->getDesired());
	  else if (!myOverrideDoesLessThan)
	    myDesired = ArUtil::findMax(myDesired, 
					desiredChannel->getDesired());
	}
	// if only it has strength use it
	else if (desiredChannel->getStrength() >= MIN_STRENGTH)
	{
	  myDesired = desiredChannel->getDesired();
	}
	// if only this has strength then we don't need to do anything
      }
      myDesiredTotal += (desiredChannel->getDesired() * 
			 desiredChannel->getStrength());
      myStrengthTotal += desiredChannel->getStrength();
    }
  void endAverage(void)
    {
      if (myStrengthTotal < MIN_STRENGTH)
      {
	myStrength = NO_STRENGTH;
	return;
      }
      // if we're overriding we just use what myDesired already is
      if (!myAllowOverride)
	myDesired = (myDesiredTotal / myStrengthTotal);
      myStrength = myStrengthTotal;
      if (myStrength > MAX_STRENGTH)
	myStrength = MAX_STRENGTH;
    }
protected:
  double myDesired;
  double myStrength;
  bool myAllowOverride;
  double myDesiredTotal;
  double myStrengthTotal;
  bool myOverrideDoesLessThan;
};

/// Class used to say what movement is desired
/**
   This class is use by actions to report what they want to want to do (hence
   the name). 

   The way it works, is that translational (front/back) and rotational
   (right/left) are separate.  Translational movement uses velocity, while 
   rotational movement uses change in heading from current heading. 
   Translational and rotational each have their own strength value.  
   Both translational and rotational movement have maximum velocities as well,
   that also have their own strengths.

   The strength value reflects how strongly an action wants to do the chosen 
   movement command, the resolver (ArResolver) will combine these strengths 
   and figure out what to do based on them.

   For all strength values there is a total of 1.0 strength to be had.
   The range for strength is from 0 to 1.  This is simply a convention that 
   ARIA uses by default, if you don't like it, you can override this
   class and make an ArResolver.

   Note that for the different maximum/accel/decel values they take an
   additional argument of whether just to use the slowest speed,
   slowest accel, or fastest decel.  By default these will just use
   safer values (slowest speed, slowest accel, fastest decel)... you
   can specify false on these for the old behavior.  Note that if
   you're safest values then the strength is largely ignored though it
   is still tracked and must still be greater than MIN_STRENGTH to
   work and it is still capped at MAX_STRENGTH).

*/
class ArActionDesired
{
public:
  AREXPORT static const double NO_STRENGTH;
  AREXPORT static const double MIN_STRENGTH;
  AREXPORT static const double MAX_STRENGTH;
  /// Constructor
  ArActionDesired() 
    { myHeadingSet = false; 
    myTransDecelDes.setOverrideDoesLessThan(false); 
    myRotDecelDes.setOverrideDoesLessThan(false); }
  /// Destructor
  virtual ~ArActionDesired() {}
  
  /// Sets the velocity (mm/sec) and strength
  /**
     @param vel desired vel (mm/sec)
     @param strength strength given to this, defaults to MAX_STRENGTH (1.0)
  */
  virtual void setVel(double vel, double strength = MAX_STRENGTH)
    { myVelDes.setDesired(vel, strength); }
  /// Sets the delta heading (deg) and strength
  /**
     If there's already a rotVel set this WILL NOT work.
     @param deltaHeading desired change in heading (deg)
     @param strength strength given to this, defaults to MAX_STRENGTH (1.0)
  */
  virtual void setDeltaHeading(double deltaHeading, 
			       double strength = MAX_STRENGTH)
    { myDeltaHeadingDes.setDesired(deltaHeading, strength); }
  /// Sets the absolute heading (deg) 
  /**
     If there's already a rotVel set this WILL NOT work.
     This is a way to set the heading instead of using a delta, there is no
     get for this, because accountForRobotHeading MUST be called (this should
     be called by all resolvers, but if you want to call it you can,
     thats fine).
     @param heading desired heading (deg)
     @param strength strength given to this, defaults to MAX_STRENGTH (1.0)
  */
  virtual void setHeading(double heading, double strength = MAX_STRENGTH)
    { myHeading = heading; myHeadingStrength = strength; myHeadingSet = true; }

  /// Sets the rotational velocity
  /**
     If there's already a delta heading or heading this WILL NOT work.
     @param rotVel desired rotational velocity (deg/sec)
     @param strength strength given to this, defaults to MAX_STRENGTH (1.0)
  **/
  virtual void setRotVel(double rotVel, double strength = MAX_STRENGTH)
    { myRotVelDes.setDesired(rotVel, strength); }


  /// Sets the maximum velocity (+mm/sec) and strength
  /**
     This sets the maximum positive velocity for this cycle.  Check
     the ArRobot class notes for more details.
     
     @param maxVel desired maximum velocity (+mm/sec)
     @param strength strength given to this, defaults to MAX_STRENGTH
     (1.0)
     @param useSlowest if this is true (the default) everywhere
     then the slowest maximum vel is what will be selected
  **/
  virtual void setMaxVel(double maxVel, double strength = MAX_STRENGTH,
			 bool useSlowest = true)
    { myMaxVelDes.setDesired(maxVel, strength, useSlowest); }
  /// Sets the maximum velocity for going backwards (-mm/sec) and strength
  /**
     This sets the maximum negative velocity for this cycle.  Check
     the ArRobot class notes for more details.

     @param maxVel desired maximum velocity for going backwards (-mm/sec)
     @param strength strength given to this, defaults to MAX_STRENGTH (1.0)
     @param useSlowest if this is true (the default) everywhere
     then the slowest max neg vel is what will be selected
  **/
  virtual void setMaxNegVel(double maxVel, double strength = MAX_STRENGTH,
			    bool useSlowest = true)
    { myMaxNegVelDes.setDesired(maxVel, strength, useSlowest); }

  /// Sets the translation acceleration (deg/sec/sec) and strength
  /**
     This sets the translation acceleration for this cycle (this is
     sent down to the robot).  Check the ArRobot class notes for more
     details.

     @param transAccel desired translation acceleration (deg/sec/sec)
     @param strength strength given to this, defaults to MAX_STRENGTH (1.0) 
     @param useSlowest if this is true (the default) everywhere then
     the slowest accel is what will be selected
  **/
  virtual void setTransAccel(double transAccel, 
			     double strength = MAX_STRENGTH,
			     bool useSlowest = true)
    { myTransAccelDes.setDesired(transAccel, strength, useSlowest);  }

  /// Sets the translation deceleration (deg/sec/sec) and strength
  /**
     This sets the translation deceleration for this cycle (this is
     sent down to the robot).  Check the ArRobot class notes for more
     details.

     @param transDecel desired translation deceleration (deg/sec/sec)
     @param strength strength given to this, defaults to MAX_STRENGTH (1.0) 
     @param useFastestDecel if this is true (the default) everywhere
     then the fastest decel is what will be selected
  **/
  virtual void setTransDecel(double transDecel, double strength = MAX_STRENGTH,
			     bool useFastestDecel = true)
    { myTransDecelDes.setDesired(transDecel, strength, useFastestDecel);  }

  /// Sets the maximum rotational velocity (deg/sec) and strength
  /**
     This sets the maximum rotational velocity for this cycle (this is
     sent down to the robot).  Check the ArRobot class notes for more
     details.

     @param maxVel desired maximum rotational velocity (deg/sec)
     @param strength strength given to this, defaults to MAX_STRENGTH (1.0) 
     @param useSlowest if this is true (the default) everywhere
     then the slowest rot vel is what will be selected
  **/
  virtual void setMaxRotVel(double maxVel, double strength = MAX_STRENGTH,
			    bool useSlowest = true)
    { myMaxRotVelDes.setDesired(maxVel, strength, useSlowest); }

  /// Sets the rotational acceleration (deg/sec/sec) and strength
  /**
     This sets the rotational acceleration for this cycle (this is
     sent down to the robot).  Check the ArRobot class notes for more
     details.

     @param rotAccel desired rotational acceleration (deg/sec/sec)
     @param strength strength given to this, defaults to MAX_STRENGTH (1.0) 
     @param useSlowest if this is true (the default) everywhere
     then the slowest rot accel is what will be selected
  **/
  virtual void setRotAccel(double rotAccel, double strength = MAX_STRENGTH,
			   bool useSlowest = true)
    { myRotAccelDes.setDesired(rotAccel, strength, useSlowest);  }

  /// Sets the rotational deceleration (deg/sec/sec) and strength
  /**
     This sets the rotational deceleration for this cycle (this is
     sent down to the robot).  Check the ArRobot class notes for more
     details.

     @param rotDecel desired rotational deceleration (deg/sec/sec)
     @param strength strength given to this, defaults to MAX_STRENGTH (1.0) 
     @param useFastest if this is true (the default) everywhere
     then the fastest rot decel is what will be selected
  **/
  virtual void setRotDecel(double rotDecel, double strength = MAX_STRENGTH,
			   bool useFastest = true)
    { myRotDecelDes.setDesired(rotDecel, strength, useFastest);  }

  /// Resets the strengths to 0
  virtual void reset(void) 
    {
      myVelDes.reset(); myDeltaHeadingDes.reset(); 
      myMaxVelDes.reset(); myMaxNegVelDes.reset(); myMaxRotVelDes.reset(); 
      myRotVelDes.reset();
      myTransAccelDes.reset(); myTransDecelDes.reset();
      myRotAccelDes.reset(); myRotDecelDes.reset();
      myHeadingSet = false;
    }

  /// Gets the translational velocity desired (mm/sec)
  virtual double getVel(void) const
    { return myVelDes.getDesired(); }
  /// Gets the strength of the translational velocity desired
  virtual double getVelStrength(void) const
    { return myVelDes.getStrength(); }
  /// Gets the heading desired (deg)
  virtual double getHeading(void) const
    { return myHeading; }
  /// Gets the strength of the heading desired
  virtual double getHeadingStrength(void) const
    { return myHeadingStrength; }
  /// Gets the delta heading desired (deg)
  virtual double getDeltaHeading(void) const
    { return myDeltaHeadingDes.getDesired(); }
  /// Gets the strength of the delta heading desired
  virtual double getDeltaHeadingStrength(void) const
    { return myDeltaHeadingDes.getStrength(); }
  /// Gets the rot vel that was set
  virtual double getRotVel(void) const { return myRotVelDes.getDesired(); }
  /// Gets the rot vel des (deg/sec)
  virtual double getRotVelStrength(void) const 
    { return myRotVelDes.getStrength(); }

  /// Gets the desired maximum velocity (mm/sec)
  virtual double getMaxVel(void) const
    { return myMaxVelDes.getDesired(); }
  /// Gets the maximum velocity strength
  virtual double getMaxVelStrength(void) const
    { return myMaxVelDes.getStrength(); }
  /// Gets whether the slowest is being used or not
  virtual double getMaxVelSlowestUsed(void) const
    { return myMaxVelDes.getAllowOverride(); }
  /// Gets the desired maximum negative velocity (-mm/sec)
  virtual double getMaxNegVel(void) const
    { return myMaxNegVelDes.getDesired(); }
  /// Gets the desired maximum negative velocity strength
  virtual double getMaxNegVelStrength(void) const
    { return myMaxNegVelDes.getStrength(); }
  /// Gets whether the slowest is being used or not
  virtual double getMaxNegVelSlowestUsed(void) const
    { return myMaxNegVelDes.getAllowOverride(); }
  /// Gets the desired trans acceleration (mm/sec)
  virtual double getTransAccel(void) const
    { return myTransAccelDes.getDesired(); }
  /// Gets the desired trans acceleration strength
  virtual double getTransAccelStrength(void) const
    { return myTransAccelDes.getStrength(); }
  /// Gets whether the slowest accel is being used or not
  virtual double getTransAccelSlowestUsed(void) const
    { return myTransAccelDes.getAllowOverride(); }
  /// Gets the desired trans deceleration (-mm/sec/sec)
  virtual double getTransDecel(void) const
    { return myTransDecelDes.getDesired(); }
  /// Gets the desired trans deceleration strength
  virtual double getTransDecelStrength(void) const
    { return myTransDecelDes.getStrength(); }
  /// Gets whether the fastest decel is being used or not
  virtual double getTransDecelFastestUsed(void) const
    { return myTransDecelDes.getAllowOverride(); }

  /// Gets the maximum rotational velocity
  virtual double getMaxRotVel(void) const
    { return myMaxRotVelDes.getDesired(); }
  /// Gets the maximum rotational velocity strength
  virtual double getMaxRotVelStrength(void) const
    { return myMaxRotVelDes.getStrength(); }
  /// Gets whether the slowest rot vel is being used or not
  virtual double getMaxRotVelSlowestUsed(void) const
    { return myMaxRotVelDes.getAllowOverride(); }
  /// Gets the desired rotational acceleration (mm/sec)
  virtual double getRotAccel(void) const
    { return myRotAccelDes.getDesired(); }
  /// Gets the desired rotational acceleration strength
  virtual double getRotAccelStrength(void) const
    { return myRotAccelDes.getStrength(); }
  /// Gets whether the slowest rot accel is being used or not
  virtual double getMaxAccelSlowestUsed(void) const
    { return myRotAccelDes.getAllowOverride(); }
  /// Gets the desired rotational deceleration (-mm/sec/sec)
  virtual double getRotDecel(void) const
    { return myRotDecelDes.getDesired(); }
  /// Gets the desired rotational deceleration strength
  virtual double getRotDecelStrength(void) const
    { return myRotDecelDes.getStrength(); }
  /// Gets whether the fastest rot decel is being used or not
  virtual double getRotDecelFastestUsed(void) const
    { return myRotDecelDes.getAllowOverride(); }


  /// Merges the given ArActionDesired into this one (this one has precedence),
  /// internal
  /** 
      This merges in the two different action values, accountForRobotHeading
      MUST be done before this is called (on both actions), since this merges
      their delta headings, and the deltas can't be known unless the account
      for angle is done.
      @param actDesired the actionDesired to merge with this one
  */
  virtual void merge(ArActionDesired *actDesired)
    {
      if (actDesired == NULL)
	return;
      myVelDes.merge(&actDesired->myVelDes);
      // if we're already using rot or delt use that, otherwise use what it wants
      if (myDeltaHeadingDes.getStrength() > NO_STRENGTH)
      {
	myDeltaHeadingDes.merge(&actDesired->myDeltaHeadingDes);
      }
      else if (myRotVelDes.getStrength() > NO_STRENGTH)
      {
	myRotVelDes.merge(&actDesired->myRotVelDes);
      }
      else
      {
	myDeltaHeadingDes.merge(&actDesired->myDeltaHeadingDes);
	myRotVelDes.merge(&actDesired->myRotVelDes);
      }
      myMaxVelDes.merge(&actDesired->myMaxVelDes);
      myMaxNegVelDes.merge(&actDesired->myMaxNegVelDes);
      myMaxRotVelDes.merge(&actDesired->myMaxRotVelDes);
      myTransAccelDes.merge(&actDesired->myTransAccelDes);
      myTransDecelDes.merge(&actDesired->myTransDecelDes);
      myRotAccelDes.merge(&actDesired->myRotAccelDes);
      myRotDecelDes.merge(&actDesired->myRotDecelDes);
    }
  /// Starts the process of avereraging together different desireds
  /**
     There is a three step process for averaging actionDesireds together,
     first startAverage must be done to set up the process, then addAverage
     must be done with each average that is desired, then finally endAverage
     should be used, after that is done then the normal process of getting
     the results out should be done.
  */
  virtual void startAverage(void)
    {
      myVelDes.startAverage();
      myRotVelDes.startAverage();
      myDeltaHeadingDes.startAverage();
      myMaxVelDes.startAverage();
      myMaxNegVelDes.startAverage();
      myMaxRotVelDes.startAverage();
      myTransAccelDes.startAverage();
      myTransDecelDes.startAverage();
      myRotAccelDes.startAverage();
      myRotDecelDes.startAverage();
    }
  /// Adds another actionDesired into the mix to average
  /**
     For a description of how to use this, see startAverage.
     @param actDesired the actionDesired to add into the average
  */
  virtual void addAverage(ArActionDesired *actDesired)
    {
      if (actDesired == NULL)
	return;
      myVelDes.addAverage(&actDesired->myVelDes);
      // if we're using one of rot or delta heading use that,
      // otherwise use whatever they're using
      if (myRotVelDes.getStrength() > NO_STRENGTH)
      {
	myRotVelDes.addAverage(
		&actDesired->myRotVelDes);
      }
      else if (myDeltaHeadingDes.getStrength() > NO_STRENGTH)
      {
	myDeltaHeadingDes.addAverage(
		&actDesired->myDeltaHeadingDes);
      }
      else
      {
	myRotVelDes.addAverage(
		&actDesired->myRotVelDes);
	myDeltaHeadingDes.addAverage(
		&actDesired->myDeltaHeadingDes);
      }

      myMaxVelDes.addAverage(&actDesired->myMaxVelDes);
      myMaxNegVelDes.addAverage(&actDesired->myMaxNegVelDes);
      myMaxRotVelDes.addAverage(&actDesired->myMaxRotVelDes);
      myTransAccelDes.addAverage(&actDesired->myTransAccelDes);
      myTransDecelDes.addAverage(&actDesired->myTransDecelDes);
      myRotAccelDes.addAverage(&actDesired->myRotAccelDes);
      myRotDecelDes.addAverage(&actDesired->myRotDecelDes);
    }
  /// Ends the process of avereraging together different desireds
  /**
     For a description of how to use this, see startAverage.
  */
  virtual void endAverage(void)
    {
      myVelDes.endAverage();
      myRotVelDes.endAverage();
      myDeltaHeadingDes.endAverage();
      myMaxVelDes.endAverage();
      myMaxNegVelDes.endAverage();
      myMaxRotVelDes.endAverage();
      myTransAccelDes.endAverage();
      myTransDecelDes.endAverage();
      myRotAccelDes.endAverage();
      myRotDecelDes.endAverage();
    }
  /// Accounts for robot heading, mostly internal
  /**
     This accounts for the robots heading, and transforms the set heading
     on this actionDesired into a delta heading so it can be merged and 
     averaged and the like
     @param robotHeading the heading the real actual robot is at now
   */
  virtual void accountForRobotHeading(double robotHeading)
    {
      if (myHeadingSet)
	setDeltaHeading(ArMath::subAngle(myHeading, robotHeading), 
			myHeadingStrength);
      myHeadingSet = false;
    }
  /// Logs what is desired
  AREXPORT virtual void log(void) const;
  /// Gets whether anything is desired (should only really be used in relation to logging)
  AREXPORT virtual bool isAnythingDesired(void) const;

protected:
  double myHeading;
  double myHeadingStrength;
  bool myHeadingSet;
  ArActionDesiredChannel myRotVelDes;
  ArActionDesiredChannel myVelDes;
  ArActionDesiredChannel myDeltaHeadingDes;
  ArActionDesiredChannel myMaxVelDes;
  ArActionDesiredChannel myMaxNegVelDes;
  ArActionDesiredChannel myMaxRotVelDes;
  ArActionDesiredChannel myTransAccelDes;
  ArActionDesiredChannel myTransDecelDes;
  ArActionDesiredChannel myRotAccelDes;
  ArActionDesiredChannel myRotDecelDes;
};


#endif
