/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARARG_H
#define ARARG_H

#include "ariaTypedefs.h"
#include "ariaUtil.h"
#include "ArFunctor.h"

class ArArgumentBuilder;

/// Argument class, mostly for actions, could be used for other things
/** 
    This is designed to be easy to add another type to the arguments... 
    All you have to do to do so, is add an enum to the Type enum, add a 
    newType getNewType(void), add a void setNewType(newType nt), and add
    a case statement for the newType to ArArg::print.  You should probably also
    add an @see newType to the documentation for ArArg::getType.

*/
class ArArg
{
public:
  typedef enum 
  { 
    INVALID, ///< An invalid argument, the argument wasn't created correctly
    INT, ///< Integer argument
    DOUBLE, ///< Double argument
    STRING, ///< String argument
    BOOL, ///< Boolean argument
    POSE, ///< ArPose argument
    FUNCTOR, ///< Argument that handles things with functors
    DESCRIPTION_HOLDER, ///< Argument that just holds a description
    
    LAST_TYPE = DESCRIPTION_HOLDER  ///< Last value in the enumeration
  } Type;


  enum {
    TYPE_COUNT = LAST_TYPE + 1 ///< Number of argument types
  };

  /// Default empty contructor
  AREXPORT ArArg();
  /// Constructor for making an integer argument
  AREXPORT ArArg(const char * name, int *pointer, 
		 const char * description = "", 
		 int minInt = INT_MIN, 
		 int maxInt = INT_MAX); 
  /// Constructor for making a double argument
  AREXPORT ArArg(const char * name, double *pointer,
		 const char * description = "", 
		 double minDouble = -HUGE_VAL,
		 double maxDouble = HUGE_VAL); 
  /// Constructor for making a boolean argument
  AREXPORT ArArg(const char * name, bool *pointer,
		 const char * description = ""); 
  /// Constructor for making a position argument
  AREXPORT ArArg(const char * name, ArPose *pointer,
		 const char * description = ""); 
  /// Constructor for making an argument of a string
  AREXPORT ArArg(const char *name, char *pointer, 
		 const char *description,
		 size_t maxStrLen);
  /// Constructor for making an argument that has functors to handle things
  AREXPORT ArArg(const char *name, 
		 ArRetFunctor1<bool, ArArgumentBuilder *> *setFunctor, 
		 ArRetFunctor<const std::list<ArArgumentBuilder *> *> *getFunctor,
		 const char *description);
  /// Constructor for just holding a description (for ArConfig)
  AREXPORT ArArg(const char *description);
  /// Copy constructor
  AREXPORT ArArg(const ArArg & arg);
  /// Assignment operator
  AREXPORT ArArg &operator=(const ArArg &arg);
  /// Destructor
  AREXPORT virtual ~ArArg();

  /// Gets the type of the argument
  AREXPORT Type getType(void) const;
  /// Gets the name of the argument
  AREXPORT const char *getName(void) const;
  /// Gets the long description of the argument
  AREXPORT const char *getDescription(void) const;
  /// Sets the argument value, for int arguments
  AREXPORT bool setInt(int val);
  /// Sets the argument value, for double arguments
  AREXPORT bool setDouble(double val);
  /// Sets the argument value, for bool arguments
  AREXPORT bool setBool(bool val);
  /// Sets the argument value, for ArPose arguments
  AREXPORT bool setPose(ArPose pose);
  /// Sets the argument value for ArArgumentBuilder arguments
  AREXPORT bool setString(const char *str);
  /// Sets the argument by calling the setFunctor callback
  AREXPORT bool setArgWithFunctor(ArArgumentBuilder *argument);
  /// Gets the argument value, for int arguments
  AREXPORT int getInt(void) const; 
  /// Gets the argument value, for double arguments
  AREXPORT double getDouble(void) const;
  /// Gets the argument value, for bool arguments
  AREXPORT bool getBool(void) const;
  /// Gets the argument value, for pose arguments
  AREXPORT ArPose getPose(void) const;
  /// Gets the argument value, for string arguments
  AREXPORT const char *getString(void) const;
  /// Gets the argument value, which is a list of argumentbuilders here
  AREXPORT const std::list<ArArgumentBuilder *> *getArgsWithFunctor(void) const;
  /// Logs the type, name, and value of this argument
  AREXPORT void log(void) const;
  /// Gets the minimum int value
  AREXPORT int getMinInt(void) const;
  /// Gets the maximum int value
  AREXPORT int getMaxInt(void) const;
  /// Gets the minimum double value
  AREXPORT double getMinDouble(void) const;
  /// Gets the maximum double value
  AREXPORT double getMaxDouble(void) const;
  /// Gets if the config priority is set
  AREXPORT bool getConfigPrioritySet(void) const;
  /// Gets the priority (only used by ArConfig)
  AREXPORT ArPriority::Priority getConfigPriority(void) const;
  /// Sets the priority (only used by ArConfig)
  AREXPORT void setConfigPriority(ArPriority::Priority priority);

private:
  /// Internal helper function
  AREXPORT void clear(void);

protected:

  ArArg::Type myType;
  std::string myName;
  std::string myDescription;
  int *myIntPointer;
  int myMinInt, myMaxInt;
  double *myDoublePointer;
  double myMinDouble, myMaxDouble;
  bool *myBoolPointer;
  ArPose *myPosePointer;
  char *myStringPointer;
  size_t myMaxStrLen;
  bool myConfigPrioritySet;
  ArPriority::Priority myConfigPriority;
  ArRetFunctor1<bool, ArArgumentBuilder *> *mySetFunctor;
  ArRetFunctor<const std::list<ArArgumentBuilder *> *> *myGetFunctor;
};

#endif // ARARGUMENT_H
