/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARIAUTIL_H
#define ARIAUTIL_H

#define _GNU_SOURCE 1
#include <string>
// #define _XOPEN_SOURCE 500
#include <stdlib.h>
#include <string.h>
#include <memory.h>
#include <list>
#include <cmath>
#include <stdarg.h>
#include <limits.h>

#if defined(_WIN32) || defined(WIN32)
#include <sys/timeb.h>
#include <sys/stat.h>
#include <winsock2.h> // required before windows.h
#include <windows.h> // timeGetTime()
#else
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>
#endif // ifndef win32

#if !defined(_MSC_VER)
#include <stdio.h> // snprintf
#endif

#include <time.h>
#include "ariaTypedefs.h"
#include "ArLog.h"
#include "ArFunctor.h"
#include "ArArgumentParser.h"
//#include "ariaInternal.h"
#include "ariaOSDef.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif // of M_PI, windows has a function call instead of a define

/// This class has utility functions
class ArUtil
{
public:
  /// Values for the bits from 0 to 16
  enum BITS {
    BIT0 = 0x1, ///< value of BIT0
    BIT1 = 0x2, ///< value of BIT1
    BIT2 = 0x4, ///< value of BIT2
    BIT3 = 0x8, ///< value of BIT3
    BIT4 = 0x10, ///< value of BIT4
    BIT5 = 0x20, ///< value of BIT5
    BIT6 = 0x40, ///< value of BIT6
    BIT7 = 0x80, ///< value of BIT7
    BIT8 = 0x100, ///< value of BIT8
    BIT9 = 0x200, ///< value of BIT9
    BIT10 = 0x400, ///< value of BIT10
    BIT11 = 0x800, ///< value of BIT11
    BIT12 = 0x1000, ///< value of BIT12
    BIT13 = 0x2000, ///< value of BIT13
    BIT14 = 0x4000, ///< value of BIT14
    BIT15 = 0x8000 ///< value of BIT15
  };

  /// Sleep for the given number of milliseconds
  AREXPORT static void sleep(unsigned int ms);

  /// Get the time in milliseconds
  AREXPORT static unsigned int getTime(void);

  /// Delete all members of a set. Does NOT empty the set.
  /**
      Assumes that T is an iterator that supports the operator*, operator!=
      and operator++. The return is assumed to be a pointer to a class that
      needs to be deleted.
  */
  template<class T> static void deleteSet(T begin, T end)
    {
      for (; begin != end; ++begin)
      {
	delete (*begin);
      }
    }

  /// Delete all members of a set. Does NOT empty the set.
  /**
     Assumes that T is an iterator that supports the operator**, operator!=
     and operator++. The return is assumed to be a pair. The second value of
     the pair is assumed to be a pointer to a class that needs to be deleted.
  */
  template<class T> static void deleteSetPairs(T begin, T end)
    {
      for (; begin != end; ++begin)
      {
	delete (*begin).second;
      }
    }

  /// Returns the minimum of the two values
  static int findMin(int first, int second)
    { if (first < second) return first; else return second; }
  /// Returns the maximum of the two values
  static int findMax(int first, int second)
    { if (first > second) return first; else return second; }

  /// Returns the minimum of the two values
  static double findMin(double first, double second)
    { if (first < second) return first; else return second; }
  /// Returns the maximum of the two values
  static double findMax(double first, double second)
    { if (first > second) return first; else return second; }

  /// OS-independent way of finding the size of a file.
  AREXPORT static long sizeFile(const char *fileName);

  /// OS-independent way of finding the size of a file.
  AREXPORT static long sizeFile(std::string fileName);

  /// OS-independent way of checking to see if a file exists and is readable.
  AREXPORT static bool findFile(const char *fileName);

  // OS-independent way of stripping the directory from the fileName.
  // commented out with std::string changes since this didn't seem worth fixing right now
  //AREXPORT static bool stripDir(std::string fileIn, std::string &fileOut);

  // OS-independent way of stripping the fileName from the directory.
  // commented out with std::string changes since this didn't seem worth fixing right now
  //AREXPORT static bool stripFile(std::string fileIn, std::string &fileOut);

  /// Appends a slash to a path if there is not one there already
  AREXPORT static void appendSlash(char *path, size_t pathLength);

  /// Fix the slash orientation in file path string for windows or linux
  AREXPORT static void fixSlashes(char *path, size_t pathLength);

  /// Fix the slash orientation in file path string to be all forward
  AREXPORT static void fixSlashesForward(char *path, size_t pathLength);

  /// Fix the slash orientation in file path string to be all backward
  AREXPORT static void fixSlashesBackward(char *path, size_t pathLength);

  /// Adds two directories, taking care of all slash issues
  AREXPORT static void addDirectories(char *dest, size_t destLength,
				      const char *baseDir,
				      const char *insideDir);

  /// Finds out if two strings are equal
  AREXPORT static int strcmp(std::string str, std::string str2);

  /// Finds out if two strings are equal
  AREXPORT static int strcmp(std::string str, const char *str2);

  /// Finds out if two strings are equal
  AREXPORT static int strcmp(const char *str, std::string str2);

  /// Finds out if two strings are equal
  AREXPORT static int strcmp(const char *str, const char *str2);

  /// Finds out if two strings are equal (ignoring case)
  AREXPORT static int strcasecmp(std::string str, std::string str2);

  /// Finds out if two strings are equal (ignoring case)
  AREXPORT static int strcasecmp(std::string str, const char *str2);

  /// Finds out if two strings are equal (ignoring case)
  AREXPORT static int strcasecmp(const char *str, std::string str2);

  /// Finds out if two strings are equal (ignoring case)
  AREXPORT static int strcasecmp(const char *str, const char *str2);

  /// Puts a \ before spaces in src, puts it into dest
  AREXPORT static void escapeSpaces(char *dest, const char *src,
				    size_t maxLen);

  /// Strips out the quotes in the src buffer into the dest buffer
  AREXPORT static bool stripQuotes(char *dest, const char *src,size_t destLen);

  /// Lowers a string from src into dest, make sure there's enough space
  AREXPORT static void lower(char *dest, const char *src,
			     size_t maxLen);
  /// Returns true if this string is only alphanumeric, false otherwise
  AREXPORT static bool isOnlyAlphaNumeric(const char *str);

	/// Returns true if the given string is null or of zero length, false otherwise
	AREXPORT static bool isStrEmpty(const char *str);

  /// Does an atof but if its inf or -inf deals with it fine
  AREXPORT static double atof(const char *nptr);

  /// Converts an integer value into a string for true or false
  AREXPORT static const char *convertBool(int val);

  /// Function for doing a printf style call to a functor
  AREXPORT static void functorPrintf(ArFunctor1<const char *> *functor,
				     const char *str, ...);

  /// Function for doing a fprintf to a file (here to make a functor for)
  AREXPORT static void writeToFile(const char *str, FILE *file);

  /// Gets a string contained in an arbitrary file
  AREXPORT static bool getStringFromFile(const char *fileName,
					 char *str, size_t strLen);
  /**
  These are for passing into getStringFromRegistry
  **/
  enum REGKEY {
    REGKEY_CLASSES_ROOT, ///< use HKEY_CLASSES_ROOT
    REGKEY_CURRENT_CONFIG, ///< use HKEY_CURRENT_CONFIG
    REGKEY_CURRENT_USER, ///< use HKEY_CURRENT_USER
    REGKEY_LOCAL_MACHINE, ///< use HKEY_LOCAL_MACHIE
    REGKEY_USERS ///< use HKEY_USERS
  };

  /// Returns a string from the Windows registry
  AREXPORT static bool getStringFromRegistry(REGKEY root,
					     const char *key,
					     const char *value,
					     char *str,
					     int len);

  /// Returns a string from the Windows registry, searching each of the following registry root paths in order: REGKEY_CURRENT_USER, REGKEY_LOCAL_MACHINE
  /*AREXPORT*/ static bool findFirstStringInRegistry(const char* key, const char* value, char* str, int len) {
	if(!getStringFromRegistry(REGKEY_CURRENT_USER, key, value, str, len))
		return getStringFromRegistry(REGKEY_LOCAL_MACHINE, key, value, str, len);
	return true;
  }

  AREXPORT static const char *COM1; ///< First serial port device name (value depends on compilation platform)
  AREXPORT static const char *COM2; ///< Second serial port device name (value depends on compilation platform)
  AREXPORT static const char *COM3; ///< Third serial port device name (value depends on compilation platform)
  AREXPORT static const char *COM4; ///< Fourth serial port device name (value depends on compilation platform)
  AREXPORT static const char *TRUESTRING; ///< "true"
  AREXPORT static const char *FALSESTRING; ///< "false"

  /** Put the current year (GMT) in s (e.g. "2005").
   *  @param s String buffer (allocated) to write year into
   *  @param len Size of @a s
   */
  AREXPORT static void putCurrentYearInString(char* s, size_t len);
  /** Put the current month (GMT) in s (e.g. "09" if September).
   *  @param s String buffer (allocated) to write month into
   *  @param len Size of @a s
   */
  AREXPORT static void putCurrentMonthInString(char* s, size_t len);
  /** Put the current day (GMT) of the month in s (e.g. "20").
   *  @param s String buffer (allocated) to write day into
   *  @param len Size of @a s
   */
  AREXPORT static void putCurrentDayInString(char* s, size_t len);
  /** Put the current hour (GMT) in s (e.g. "13" for 1 o'clock PM).
   *  @param s String buffer (allocated) to write hour into
   *  @param len Size of @a s
   */
  AREXPORT static void putCurrentHourInString(char* s, size_t len);
  /** Put the current minute (GMT) in s (e.g. "05").
   *  @param s String buffer (allocated) to write minutes into
   *  @param len Size of @a s
   */
  AREXPORT static void putCurrentMinuteInString(char* s, size_t len);
  /** Put the current second (GMT) in s (e.g. "59").
   *  @param s String buffer (allocated) to write seconds into
   *  @param len Size of @a s
   */
  AREXPORT static void putCurrentSecondInString(char* s, size_t len);

  /** Interface to native platform localtime() function.
   *  On Linux, this is equivalent to a call to localtime_r(@a timep, @a result) (which is threadsafe, including the returned pointer, since it uses a different time struct for each thread)
   *  On Windows, this is equivalent to a call to localtime(@a timep, @a result) (which is NOT threadsafe, since the same global time struct is shared by all threads)
   *
   *  @param timep Pointer to current time (Unix time_t; seconds since epoch).
   *  @param result The result of calling platform localtime function is copied into this struct, so it must have been allocated.
   *  @return false on error (e.g. invalid input), otherwise true.
   *
   *  Example:
   *  @code
   *  struct tm t;
   *  ArUtil::localtime(time(NULL), &t);
   *  ArLog::log("Current month is %d.\n", t.tm_mon);
   *  @endcode
   */
  AREXPORT static bool localtime(const time_t *timep, struct tm *result);


  /** Call ArUtil::localtime() with the current time obtained by calling
   * time(NULL).
   *  @return false on error (e.g. invalid input), otherwise true.
   */
  AREXPORT static bool localtime(struct tm *result);
};

///  This class has static members to do common math operations.
class ArMath
{
public:

  /// This adds two angles together and fixes the result to [-180, 180]
  /**
     @param ang1 first angle
     @param ang2 second angle, added to first
     @return sum of the angles, in range [-180,180]
     @see subAngle
     @see fixAngle */
  static double addAngle(double ang1, double ang2)
    { return fixAngle(ang1 + ang2); }

  /// This subtracts one angle from another and fixes the result to [-180,180]
  /**
     @param ang1 first angle
     @param ang2 second angle, subtracted from first angle
     @return resulting angle, in range [-180,180]
     @see addAngle
     @see fixAngle
  */
  static double subAngle(double ang1, double ang2)
    { return fixAngle(ang1 - ang2); }

  /// Takes an angle and returns the angle in range (-180,180]
  /**
     @param angle the angle to fix
     @return the angle in range (-180,180]
     @see addAngle
     @see subAngle
  */
  static double fixAngle(double angle)
    {
      if (angle >= 360)
	angle = angle - 360.0 * (double)((int)angle / 360);
      if (angle < -360)
	angle = angle + 360.0 * (double)((int)angle / -360);
      if (angle <= -180)
	angle = + 180.0 + (angle + 180.0);
      if (angle > 180)
	angle = - 180.0 + (angle - 180.0);
      return angle;
    }

  /// Converts an angle in degrees to an angle in radians
  /**
     @param deg the angle in degrees
     @return the angle in radians
     @see radToDeg
  */
  static double degToRad(double deg) { return deg * M_PI / 180.0; }

  /// Converts an angle in radians to an angle in degrees
  /**
     @param rad the angle in radians
     @return the angle in degrees
     @see degToRad
  */
  static double radToDeg(double rad) { return rad * 180.0 / M_PI; }

  /// Finds the cos, from angles in degrees
  /**
     @param angle angle to find the cos of, in degrees
     @return the cos of the angle
     @see sin
  */
  static double cos(double angle) { return ::cos(ArMath::degToRad(angle)); }

  /// Finds the sin, from angles in degrees
  /**
     @param angle angle to find the sin of, in degrees
     @return the sin of the angle
     @see cos
  */
  static double sin(double angle) { return ::sin(ArMath::degToRad(angle)); }

  /// Finds the tan, from angles in degrees
  /**
     @param angle angle to find the tan of, in degrees
     @return the tan of the angle
  */
  static double tan(double angle) { return ::tan(ArMath::degToRad(angle)); }

  /// Finds the arctan of the given y/x pair
  /**
     @param y the y distance
     @param x the x distance
     @return the angle y and x form
  */
  static double atan2(double y, double x)
    { return ArMath::radToDeg(::atan2(y, x)); }

  /// Finds if one angle is between two other angles
  static bool angleBetween(double angle, double startAngle, double endAngle)
    {
      angle = fixAngle(angle);
      startAngle = fixAngle(startAngle);
      endAngle = fixAngle(endAngle);
      if ((startAngle < endAngle && angle > startAngle && angle < endAngle) ||
	  (startAngle > endAngle && (angle > startAngle || angle < endAngle)))
	return true;
      else
	return false;
    }

  /// Finds the absolute value of a double
  /**
     @param val the number to find the absolute value of
     @return the absolute value of the number
  */
  static double fabs(double val)
    {
      if (val < 0.0)
	return -val;
      else
	return val;
    }

  /// Finds the closest integer to double given
  /**
     @param val the double to find the nearest integer to
     @return the integer the value is nearest to (also caps it within
     int bounds)
  */
  static int roundInt(double val)
    {
      val += .49;
      if (val > INT_MAX)
	return (int) INT_MAX;
      else if (val < INT_MIN)
	return (int) INT_MIN;
      else
	return((int) floor(val));
    }

  /// Finds the closest short to double given
  /**
     @param val the double to find the nearest short to
     @return the integer the value is nearest to (also caps it within
     short bounds)
  */
  static short roundShort(double val)
    {
      val += .49;
      if (val > 32767)
	return (short) 32767;
      else if (val < -32768)
	return (short) -32768;
      else
	return((short) floor(val));
    }


  /// Rotates a point around 0 by degrees given
  static void pointRotate(double *x, double *y, double th)
    {
      double cs, sn, xt, yt;
      cs = cos(th);
      sn = sin(th);
      xt = *x;
      yt = *y;
      *x = cs*xt + sn*yt;
      *y = cs*yt - sn*xt;
    }

  /// Returns a long between 0 and some arbitrary huge number
  static long random(void)
    {
#ifdef WIN32
      return(rand());
#else
      return(lrand48());
#endif
    }
  /// Finds the distance between two coordinates
  /**
     @param x1 the first coords x position
     @param y1 the first coords y position
     @param x2 the second coords x position
     @param y2 the second coords y position
     @return the distance between (x1, y1) and (x2, y2)
  **/
  static double distanceBetween(double x1, double y1, double x2, double y2)
    { return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));  }

  /// Finds the squared distance between two coordinates
  /**
     use this only where speed really matters
     @param x1 the first coords x position
     @param y1 the first coords y position
     @param x2 the second coords x position
     @param y2 the second coords y position
     @return the distance between (x1, y1) and (x2, y2)
  **/
  static double squaredDistanceBetween(double x1, double y1, double x2, double y2)
    { return (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);  }

  /** Base-2 logarithm */
  static double log2(double x)
  {
#ifdef WIN32
    return log10(x) / 0.3010303;  // that's log10(2.0);
#else
    return log10(x) / 0.3010303;  // that's log10(2.0);
//    return log2(x);
#endif
  }
};

/// The class which represents a position
/**
    This class represents a robot position with heading.  The heading defaults
    to 0, and so does not need to be used (this avoids having 2 types of
    positions).  Everything in the class is inline so it should be fast.
*/
class ArPose
{
public:
  /// Constructor, with optional initial values
  /**
      Sets the position with the given values, can be used with no variables,
      with just x and y, or with x, y, and th
      @param x the position to set the x position to, default of 0
      @param y the position to set the y position to, default of 0
      @param th the position to set the th position to, default of 0
  */
  ArPose(double x = 0, double y = 0, double th = 0)
    { myX = x; myY = y; myTh = th; }
  /// Copy Constructor
  ArPose(const ArPose &pose) :
    myX(pose.myX), myY(pose.myY), myTh(pose.myTh) {}

  /// Destructor
  virtual ~ArPose() {}
  /// Sets the position to the given values
  /**
      Sets the position with the given three values, but the theta does not
      need to be given as it defaults to 0.
      @param x the position to set the x position to
      @param y the position to set the y position to
      @param th the position to set the th position to, default of 0
  */
  virtual void setPose(double x, double y, double th = 0)
    { setX(x); setY(y); setTh(th); }
  /// Sets the position equal to the given position
  /** @param position the position value this instance should be set to */
  virtual void setPose(ArPose position)
    {
      setX(position.getX());
      setY(position.getY());
      setTh(position.getTh());
    }
  /// Sets the x position
  void setX(double x) { myX = x; }
  /// Sets the y position
  void setY(double y) { myY = y; }
  /// Sets the heading
  void setTh(double th) { myTh = ArMath::fixAngle(th); }
  /// Sets the heading, using radians
  void setThRad(double th) { myTh = ArMath::fixAngle(ArMath::radToDeg(th)); }
  /// Gets the x position
  double getX(void) const { return myX; }
  /// Gets the y position
  double getY(void) const { return myY; }
  /// Gets the heading
  double getTh(void) const { return myTh; }
  /// Gets the heading, in radians
  double getThRad(void) const { return ArMath::degToRad(myTh); }
  /// Gets the whole position in one function call
  /**
     Gets the whole position at once, by giving it 2 or 3 pointers to
     doubles.  If you give the function a null pointer for a value it won't
     try to use the null pointer, so you can pass in a NULL if you don't
     care about that value.  Also note that th defaults to NULL so you can
     use this with just x and y.
     @param x a pointer to a double to set the x position to
     @param y a pointer to a double to set the y position to
     @param th a pointer to a double to set the heading to, defaults to NULL
   */
  void getPose(double *x, double *y, double *th = NULL) const
    {
      if (x != NULL)
	*x = myX;
      if (y != NULL)
	*y = myY;
      if (th != NULL)
	*th = myTh;
    }
  /// Finds the distance from this position to the given position
  /**
     @param position the position to find the distance to
     @return the distance to the position from this instance
  */
  virtual double findDistanceTo(ArPose position) const
    {
      return ArMath::distanceBetween(getX(), getY(),
				     position.getX(),
				     position.getY());
    }

  /// Finds the square distance from this position to the given position
  /**
     This is only here for speed, if you aren't doing this thousands
     of times a second don't use this one use findDistanceTo

     @param position the position to find the distance to
     @return the distance to the position from this instance
  **/
  virtual double squaredFindDistanceTo(ArPose position) const
    {
      return ArMath::squaredDistanceBetween(getX(), getY(),
					    position.getX(),
					    position.getY());
    }
  /// Finds the angle between this position and the given position
  /**
      @param position the position to find the angle to
      @return the angle to the given position from this instance, in degrees
  */
  virtual double findAngleTo(ArPose position) const
    {
      return ArMath::radToDeg(atan2(position.getY() - getY(),
				    position.getX() - getX()));
    }
  /// Logs the coordinates using ArLog
  virtual void log(void) const
    { ArLog::log(ArLog::Terse, "%.0f %.0f %.1f", myX, myY, myTh); }

  /// Add the other pose's X, Y and theta to this pose's X, Y, and theta (sum in theta will be normalized to (-180,180)), and return the result
  virtual ArPose operator+(const ArPose& other)
  {
    return ArPose( myX + other.getX(), myY + other.getY(), ArMath::fixAngle(myTh + other.getTh()) );
  }

  /// Substract the other pose's X, Y, and theta from this pose's X, Y, and theta (difference in theta will be normalized to (-180,180)), and return the result
  virtual ArPose operator-(const ArPose& other)
  {
    return ArPose( myX - other.getX(), myY - other.getY(), ArMath::fixAngle(myTh - other.getTh()) );
  }

protected:
  double myX;
  double myY;
  double myTh;
};


/// A class for time readings and measuring durations
/**
    This class is for timing durations or time between events.
    The time values it stores are relative to an abritrary starting time; it
    does not correspond to "real world" or "wall clock" time in any way,
    so DON'T use this for keeping track of what time it is,
    just for timestamps and relative timing (e.g. "this loop needs to sleep another 100 ms").

    The recommended methods to use are setToNow() to reset the time,
    mSecSince() to obtain the number of miliseconds elapsed since it was
    last reset (or secSince() if you don't need milisecond precision), and
    mSecSince(ArTime) or secSince(ArTime) to find the difference between
    two ArTime objects.

*/

class ArTime
{
public:
  /// Constructor
  ArTime() { setToNow(); }
  /// Destructor
  ~ArTime() {}

  /// Gets the number of milliseconds since the given timestamp to this one
  long mSecSince(ArTime since) const
    {
      long timeSince, timeThis;

      timeSince = since.getSec() * 1000 + since.getMSec();
      timeThis = mySec * 1000 + myMSec;
      return timeSince - timeThis;
    }
  /// Gets the number of seconds since the given timestamp to this one
  long secSince(ArTime since) const
    {
      return mSecSince(since)/1000;
    }
  /// Finds the number of millisecs from when this timestamp is set to to now (the inverse of mSecSince())
  long mSecTo(void) const
    {
      ArTime now;
      now.setToNow();
      return -mSecSince(now);
    }
  /// Finds the number of seconds from when this timestamp is set to to now (the inverse of secSince())
  long secTo(void) const
    {
      return mSecTo()/1000;
    }
  /// Finds the number of milliseconds from this timestamp to now
  long mSecSince(void) const
    {
      ArTime now;
      now.setToNow();
      return mSecSince(now);
    }
  /// Finds the number of seconds from when this timestamp was set to now
  long secSince(void) const
    {
      return mSecSince()/1000;
    }
  /// returns whether the given time is before this one or not
  bool isBefore(ArTime testTime) const
    {
      if (mSecSince(testTime) < 0)
	return true;
      else
	return false;
    }
  /// returns whether the given time is equal to this time or not
  bool isAt(ArTime testTime) const
    {
      if (mSecSince(testTime) == 0)
	return true;
      else
	return false;
    }
  /// returns whether the given time is after this one or not
  bool isAfter(ArTime testTime) const
    {
      if (mSecSince(testTime) > 0)
	return true;
      else
	return false;
    }
  /// Resets the time
  void setToNow(void)
    {
#ifdef WIN32
      /* this should be the better way, but it doesn't really work...
	 this would be seconds from 1970, but it is based on the
	 hardware timer or something and so winds up not being updated
	 all the time and winds up being some number of ms < 20 ms off
      struct _timeb startTime;
      _ftime(&startTime);
      mySec = startTime.time;
      myMSec = startTime.millitm;*/
      // so we're going with just their normal function, msec since boot
      long timeNow;
      timeNow = timeGetTime();
      mySec = timeNow / 1000;
      myMSec = timeNow % 1000;
#else // if not win32
      struct timeval timeNow;

      if (gettimeofday(&timeNow, NULL) == 0)
      {
	mySec = timeNow.tv_sec;
	myMSec = timeNow.tv_usec / 1000;
      }
      else
	{
		mySec=0;
		myMSec=0;
		ArLog::log(ArLog::Terse, "ArTime::setToNow: invalid return from gettimeofday.\n");
	}
#endif //linux
    }
  /// Add some milliseconds (can be negative) to this time
  void addMSec(long ms)
    {
      unsigned long timeThis;
      timeThis = mySec * 1000 + myMSec;
      if (ms < 0 && (unsigned)std::abs((double)ms) > timeThis)
      {
	ArLog::log(ArLog::Terse, "ArTime::addMsec: tried to subtract too many milliseconds, would result in a negative time.");
	mySec = 0;
	myMSec = 0;
      }
      else
      {
	timeThis += ms;
	mySec = timeThis / 1000;
	myMSec = timeThis % 1000;
      }
    }
  /// Sets the seconds value (since the arbitrary starting time)
  void setSec(time_t sec) { mySec = sec; }
  /// Sets the milliseconds value (occuring after the seconds value)
  void setMSec(time_t msec) { myMSec = msec; }
  /// Gets the seconds value (since the arbitrary starting time)
  time_t getSec(void) const { return mySec; }
  /// Gets the milliseconds value (occuring after the seconds value)
  time_t getMSec(void) const { return myMSec; }
  /// Logs the time
  void log(void) const
    { ArLog::log(ArLog::Terse, "Time: %ld.%ld", getSec(), getMSec()); }
protected:
  time_t mySec;
  time_t myMSec;

};

/// A subclass of pose that also has the time the pose was taken
/**

 */
class ArPoseWithTime : public ArPose
{
public:
  ArPoseWithTime(double x = 0, double y = 0, double th = 0,
	 ArTime thisTime = ArTime()) : ArPose(x, y, th)
    { myTime = thisTime; }
  virtual ~ArPoseWithTime() {}
  void setTime(ArTime newTime) { myTime = newTime; }
  void setTimeToNow(void) { myTime.setToNow(); }
  ArTime getTime(void) const { return myTime; }
protected:
  ArTime myTime;
};

/// A class for keeping track of if a complete revolution has been attained
/**
   This class can be used to keep track of if a complete revolution has been
   done, it is used by doing doing a clearQuadrants when you want to stat
   the revolution.  Then at each point doing an updateQuadrant with the current
   heading of the robot.  When didAllQuadrants returns true, then all the
   quadrants have been done.
*/
class ArSectors
{
public:
  /// Constructor
  ArSectors(int numSectors = 8)
    {
      mySectorSize = 360/numSectors;
      mySectors = new int[numSectors];
      myNumSectors = numSectors;
      clear();
    }
  /// Destructor
  virtual ~ArSectors() { delete mySectors; }
  /// Clears all quadrants
  void clear(void)
    {
      int i;
      for (i = 0; i < myNumSectors; i++)
	mySectors[i] = 0 /*false*/;
    }
  /// Updates the appropriate quadrant for the given angle
  void update(double angle)
    {
      int angleInt;
      angleInt = ArMath::roundInt(ArMath::fixAngle(angle) + 180);
      mySectors[angleInt / mySectorSize] = 1 /* true */;
    }
  /// Returns true if the all of the quadrants have been gone through
  bool didAll(void) const
    {
      int i;
      for (i = 0; i < myNumSectors; i++)
	if (mySectors[i] == false)
	  return false;
      return true;
    }
protected:
  int *mySectors;
  int myNumSectors;
  int mySectorSize;
};




/// Represents geometry of a line in two-dimensional space.
/**
   Note this the theoretical line, i.e. it goes infinitely.
   For a line segment with endpoints, use ArLineSegment.
   @sa ArLineSegment
**/
class ArLine
{
public:
  ///// Empty constructor
  ArLine() {}
  /// Constructor with parameters
  ArLine(double a, double b, double c) { newParameters(a, b, c); }
  /// Constructor with endpoints
  ArLine(double x1, double y1, double x2, double y2)
  { newParametersFromEndpoints(x1, y1, x2, y2); }
  /// Destructor
  virtual ~ArLine() {}
  /// Sets the line parameters (make it not a segment)
  void newParameters(double a, double b, double c)
    { myA = a; myB = b; myC = c; }
  /// Sets the line parameters from endpoints, but makes it not a segment
  void newParametersFromEndpoints(double x1, double y1, double x2, double y2)
    { myA = y1 - y2; myB = x2 - x1; myC = (y2 *x1) - (x2 * y1); }
  /// Gets the A line parameter
  double getA(void) const { return myA; }
  /// Gets the B line parameter
  double getB(void) const { return myB; }
  /// Gets the C line parameter
  double getC(void) const { return myC; }
  /// finds the intersection of this line with another line
  /**
      @param line the line to check if it intersects with this line
      @param pose if the lines intersect, the pose is set to the location
      @return true if they intersect, false if they do not
  **/
  bool intersects(const ArLine *line, ArPose *pose)
    {
      double x, y;
      double n;
      n = (line->getB() * getA()) - (line->getA() * getB());
      // if this is 0 the lines are parallel
      if (fabs(n) < .0000000000001)
      {
	return false;
      }
      // they weren't parallel so see where the intersection is
      x = ((line->getC() * getB()) - (line->getB() * getC())) / n;
      y = ((getC() * line->getA()) - (getA() * line->getC())) / n;
      pose->setPose(x, y);
      return true;
    }
  /// Makes the given line perpendicular to this one though the given pose
  void makeLinePerp(const ArPose *pose, ArLine *line) const
    {
      line->newParameters(getB(), -getA(),
			  (getA() * pose->getY()) - (getB() * pose->getX()));
    }
protected:
  double myA, myB, myC;
};

/// Represents a line segment in two-dimensional space.
/** The segment is defined by the coordinates of each endpoint. */
class ArLineSegment
{
public:
#ifndef SWIG
  /** @swigomit */
  ArLineSegment() {}
  /** @brief Constructor with endpoints
   *  @swigomit
   */
  ArLineSegment(double x1, double y1, double x2, double y2)
    { 	newEndPoints(x1, y1, x2, y2); }
#endif // SWIG
  /// Constructor with endpoints as ArPose objects. Only X and Y components of the poses will be used.
  ArLineSegment(ArPose pose1, ArPose pose2)
    { 	newEndPoints(pose1.getX(), pose1.getY(), pose2.getX(), pose2.getY()); }
  virtual ~ArLineSegment() {}
  /// Set new end points for this line segment
  void newEndPoints(double x1, double y1, double x2, double y2)
    {
      myX1 = x1; myY1 = y1; myX2 = x2; myY2 = y2;
      myLine.newParametersFromEndpoints(myX1, myY1, myX2, myY2);
    }
  /// Set new end points for this line segment
  void newEndPoints(const ArPose& pt1, const ArPose& pt2)
    {
      newEndPoints(pt1.getX(), pt1.getY(), pt2.getX(), pt2.getY());
    }
  /// Get the first endpoint (X1, Y1)
  ArPose getEndPoint1(void) const { return ArPose(myX1, myY1); }
  /// Get the second endpoint of (X2, Y2)
  ArPose getEndPoint2(void) const { return ArPose(myX2, myY2); }
  /// Determine where a line intersects this line segment
  /**
      @param line Line to check for intersection against this line segment.
      @param pose if the lines intersect, the X and Y components of this pose are set to the point of intersection.
      @return true if they intersect, false if they do not
   **/
  bool intersects(const ArLine *line, ArPose *pose)
    {
      // see if it intersects, then make sure its in the coords of this line
      if (myLine.intersects(line, &myIntersection) &&
	  linePointIsInSegment(&myIntersection))
      {
	pose->setPose(myIntersection);
	return true;
      }
      else
	return false;
    }

  /** @copydoc intersects(const ArLine*, ArPose*) */
  bool intersects(ArLineSegment *line, ArPose *pose)
    {
      // see if it intersects, then make sure its in the coords of this line
      if (myLine.intersects(line->getLine(), &myIntersection) &&
	  linePointIsInSegment(&myIntersection) &&
	  line->linePointIsInSegment(&myIntersection))
      {
	pose->setPose(myIntersection);
	return true;
      }
      else
	return false;
    }
#ifndef SWIG
  /// Determine the intersection point between this line segment, and a perpendicular line passing through the given pose (i.e. projects the given pose onto this line segment.)
  /**
   * If there is no intersection, false is returned.
     @param pose The X and Y components of this pose object indicate the point to project onto this line segment.
     @param perpPoint The X and Y components of this pose object are set to indicate the intersection point
     @return true if an intersection was found and perpPoint was modified, false otherwise.
     @swigomit
  **/
  bool getPerpPoint(ArPose pose, ArPose *perpPoint)
    {
      myLine.makeLinePerp(&pose, &myPerpLine);
      return intersects(&myPerpLine, perpPoint);
    }
#endif
  /** @copydoc getPerpPoint(ArPose, ArPose*)
   *  (This version simply allows you to pass the first pose as a pointer, in
   *  time-critical situations where a full copy of the object would impact
   *  performance.)
  */
  bool getPerpPoint(const ArPose *pose, ArPose *perpPoint)
    {
      myLine.makeLinePerp(pose, &myPerpLine);
      return intersects(&myPerpLine, perpPoint);
    }
   /// Calculate the distance from the given point to (its projection on) this line segment
  /**
     @param pose the the pose to find the perp point of

     @return if the pose does not intersect segment it will return < 0
     if the pose intersects the segment it will return the distance to
     the intersection
  **/
  double getPerpDist(const ArPose pose)
    {
      ArPose perpPose;
      myLine.makeLinePerp(&pose, &myPerpLine);
      if (!intersects(&myPerpLine, &perpPose))
	return -1;
      return (perpPose.findDistanceTo(pose));
    }
   /// Gets the distance from this line segment to a point.
  /**
   * If the point can be projected onto this line segment (i.e. a
   * perpendicular line can be drawn through the point), then
   * return that distance. Otherwise, return the distance to the closest
   * endpoint.
     @param pose the pointer of the pose to find the distance to
  **/
  double getDistToLine(const ArPose pose)
    {
      ArPose perpPose;
      myLine.makeLinePerp(&pose, &myPerpLine);
      if (!intersects(&myPerpLine, &perpPose))
      {
	return ArUtil::findMin(
		ArMath::roundInt(getEndPoint1().findDistanceTo(pose)),
		ArMath::roundInt(getEndPoint2().findDistanceTo(pose)));
      }
      return (perpPose.findDistanceTo(pose));
    }
  /// Gets the x coordinate of the first endpoint
  double getX1(void) const { return myX1; }
  /// Gets the y coordinate of the first endpoint
  double getY1(void) const { return myY1; }
  /// Gets the x coordinate of the second endpoint
  double getX2(void) const { return myX2; }
  /// Gets the y coordinate of the second endpoint
  double getY2(void) const { return myY2; }
  /// Gets the A line parameter (see ArLine)
  double getA(void) const { return myLine.getA(); }
  /// Gets the B line parameter (see ArLine)
  double getB(void) const { return myLine.getB(); }
  /// Gets the C line parameter (see ArLine)
  double getC(void) const { return myLine.getC(); }
  /// Internal function for seeing if a point on our line is within our segment
  bool linePointIsInSegment(ArPose *pose) const
    {
      return (((myX1 == myX2) ||
	       (pose->getX() >= myX1 && pose->getX() <= myX2) ||
	       (pose->getX() <= myX1 && pose->getX() >= myX2)) &&
	      ((myY1 == myY2) ||
	       (pose->getY() >= myY1 && pose->getY() <= myY2) ||
	       (pose->getY() <= myY1 && pose->getY() >= myY2)));
    }
  const ArLine *getLine(void) const { return &myLine; }
protected:
  double myX1, myY1, myX2, myY2;
  ArLine myLine;
  ArPose myIntersection;
  ArLine myPerpLine;
};

/// This is a class for computing a running average of a number of elements
class ArRunningAverage
{
public:
  /// Constructor, give it the number of elements you want to average
  AREXPORT ArRunningAverage(size_t numToAverage);
  /// Destructor
  AREXPORT ~ArRunningAverage();
  /// Gets the average
  AREXPORT double getAverage(void) const;
  /// Adds a number
  AREXPORT void add(double val);
  /// Clears the average
  AREXPORT void clear(void);
  /// Gets the number of elements
  AREXPORT size_t getNumToAverage(void) const;
  /// Sets the number of elements
  AREXPORT void setNumToAverage(size_t numToAverage);
protected:
  size_t myNumToAverage;
  double myTotal;
  size_t myNum;
  std::list<double> myVals;
};


//class ArStrCaseCmpOp :  public std::binary_function <const std::string&, const std::string&, bool>
struct ArStrCaseCmpOp
{
public:
  bool operator() (const std::string &s1, const std::string &s2) const
  {
    return strcasecmp(s1.c_str(), s2.c_str()) < 0;
  }
};

#if !defined(WIN32) && !defined(SWIG)
/** Run the program as a background daemon (i.e. fork it) (Only available in Linux).
 *  @swigomit
 *  @notwindows
 */
class ArDaemonizer
{
public:
  /// Constructor that sets up for daemonizing if arg checking
  AREXPORT ArDaemonizer(int *argc, char **argv);
  /// Destructor
  AREXPORT ~ArDaemonizer();
  /// Daemonizes if asked too by arguments
  AREXPORT bool daemonize(void);
  /// Daemonizes always
  AREXPORT bool forceDaemonize(void);
  /// Logs the options
  AREXPORT void logOptions(void) const;
  /// Returns if we're daemonized or not
  bool isDaemonized(void) { return myIsDaemonized; }
protected:
  ArArgumentParser myParser;
  bool myIsDaemonized;
  ArConstFunctorC<ArDaemonizer> myLogOptionsCB;
};
#endif // !win32 && !swig



/// Contains enumeration of four user-oriented priority levels (used primarily by ArConfig)
class ArPriority
{
public:
  enum Priority
  {
    IMPORTANT, ///< Things that should be modified to suit
    NORMAL, ///< Things that may want to be modified
    DETAILED, ///< Things that probably shouldn't be modified
    TRIVIAL = DETAILED, ///< Things that probably shouldn't be modified (alias for historic reasons)

    LAST_PRIORITY = DETAILED ///< Last value in the enumeration
  };

  enum {
    PRIORITY_COUNT = LAST_PRIORITY + 1 ///< Number of priority values
  };

  AREXPORT static const char * getPriorityName(Priority priority);
protected:
  static bool ourStringsInited;
  static std::map<Priority, std::string> ourPriorityNames;
  static std::string ourUnknownPriorityName;
};

/// holds information about ArStringInfo component strings (it's a helper class for other things)
/**
   This class holds information for about different strings that are available
 **/
class ArStringInfoHolder
{
public:
  /// Constructor
  ArStringInfoHolder(const char *name, ArTypes::UByte2 maxLength,
		     ArFunctor2<char *, ArTypes::UByte2> *functor)
    { myName = name; myMaxLength = maxLength; myFunctor = functor; }
  /// Destructor
  virtual ~ArStringInfoHolder() {}
  /// Gets the name of this piece of info
  const char *getName(void) { return myName.c_str(); }
  /// Gets the maximum length of this piece of info
  ArTypes::UByte2 getMaxLength(void) { return myMaxLength; }
  /// Gets the function that will fill in this piece of info
  ArFunctor2<char *, ArTypes::UByte2> *getFunctor(void) { return myFunctor; }
protected:
  std::string myName;
  ArTypes::UByte2 myMaxLength;
  ArFunctor2<char *, ArTypes::UByte2> *myFunctor;
};

/// This class just holds some helper functions for the ArStringInfoHolder
class ArStringInfoHolderFunctions
{
public:
  static void intWrapper(char * buffer, ArTypes::UByte2 bufferLen,
			 ArRetFunctor<int> *functor, const char *format)
    { snprintf(buffer, bufferLen - 1, format, functor->invokeR());
    buffer[bufferLen-1] = '\0'; }
  static void doubleWrapper(char * buffer, ArTypes::UByte2 bufferLen,
			    ArRetFunctor<double> *functor, const char *format)
    { snprintf(buffer, bufferLen - 1, format, functor->invokeR());
    buffer[bufferLen-1] = '\0'; }
  static void boolWrapper(char * buffer, ArTypes::UByte2 bufferLen,
			  ArRetFunctor<bool> *functor, const char *format)
    { snprintf(buffer, bufferLen - 1, format,
	       ArUtil::convertBool(functor->invokeR()));
    buffer[bufferLen-1] = '\0'; }
  static void stringWrapper(char * buffer, ArTypes::UByte2 bufferLen,
			    ArRetFunctor<const char *> *functor,
			    const char *format)
  { snprintf(buffer, bufferLen - 1, format, functor->invokeR());
  buffer[bufferLen-1] = '\0'; }

};

/// A class to hold a list of callbacks to call
class ArCallbackList
{
public:
  /// Constructor
  AREXPORT ArCallbackList(const char *name = "",
			  ArLog::LogLevel logLevel = ArLog::Verbose);
  /// Destructor
  AREXPORT virtual ~ArCallbackList();
  /// Adds a callback
  AREXPORT void addCallback(ArFunctor *functor, int position = 50);
  /// Removes a callback
  AREXPORT void remCallback(ArFunctor *functor);
  /// Sets the name
  AREXPORT void setName(const char *name);
  /// Sets the log level
  AREXPORT void setLogLevel(ArLog::LogLevel logLevel);
  /// Calls the callback list
  AREXPORT void invoke(void);
protected:
  ArMutex myDataMutex;
  ArLog::LogLevel myLogLevel;
  std::string myName;
  std::map<int, ArFunctor *> myList;
};

#endif // ARIAUTIL_H


